#ifndef CHAMP_UNITREE_MOTION_SWITCHER_H
#define CHAMP_UNITREE_MOTION_SWITCHER_H

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <unitree_api/msg/request.hpp>
#include <unitree_api/msg/response.hpp>

// Reads one string field out of the flat JSON object the motion_switcher
// service answers with ({"form":"0","name":"normal"}). Enough for CheckMode;
// avoids pulling nlohmann/json into the package.
inline bool unitreeJsonStringField(
  const std::string & json, const std::string & key, std::string & value)
{
  const std::string quoted_key = "\"" + key + "\"";
  size_t pos = json.find(quoted_key);
  if (pos == std::string::npos) {
    return false;
  }
  pos = json.find(':', pos + quoted_key.size());
  if (pos == std::string::npos) {
    return false;
  }
  pos = json.find_first_not_of(" \t\r\n", pos + 1);
  if (pos == std::string::npos || json[pos] != '"') {
    return false;
  }
  std::string out;
  for (size_t i = pos + 1; i < json.size(); ++i) {
    const char c = json[i];
    if (c == '\\' && i + 1 < json.size()) {
      out += json[++i];
      continue;
    }
    if (c == '"') {
      value = out;
      return true;
    }
    out += c;
  }
  return false;
}

// Request/response client of the Unitree motion_switcher service over
// unitree_ros2 (unitree_api/msg/Request on /api/motion_switcher/request,
// unitree_api/msg/Response on /api/motion_switcher/response). Port of the
// unitree_ros2 example b2_base_client.hpp + b2_motion_switch_client.hpp; the
// service and its api ids are the same on every Unitree robot (the Go2 and B2
// stand examples both use it), robot selection does not happen here.
//
// The response subscription lives in its own callback group spun by a private
// executor thread, so calls can block while the owning node is not spinning
// yet (the bridge releases Sport in its constructor).
class UnitreeMotionSwitcher
{
public:
  using Request = unitree_api::msg::Request;
  using Response = unitree_api::msg::Response;

  static constexpr int64_t kApiCheckMode = 1001;
  static constexpr int64_t kApiReleaseMode = 1003;
  static constexpr int32_t kSuccess = 0;
  static constexpr int32_t kTimeout = -1;        // UT_ROBOT_TASK_TIMEOUT
  static constexpr int32_t kUnknownError = -2;   // UT_ROBOT_TASK_UNKNOWN_ERROR

  UnitreeMotionSwitcher(
    rclcpp::Node & node, std::chrono::milliseconds timeout,
    const std::string & request_topic = "/api/motion_switcher/request",
    const std::string & response_topic = "/api/motion_switcher/response")
  : node_(node), timeout_(timeout), request_topic_(request_topic),
    response_topic_(response_topic)
  {
    callback_group_ = node_.create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive, false);
    rclcpp::SubscriptionOptions options;
    options.callback_group = callback_group_;
    request_pub_ = node_.create_publisher<Request>(request_topic_, rclcpp::QoS(10));
    response_sub_ = node_.create_subscription<Response>(
      response_topic_, rclcpp::QoS(20),
      std::bind(&UnitreeMotionSwitcher::onResponse, this, std::placeholders::_1), options);
    executor_.add_callback_group(callback_group_, node_.get_node_base_interface());
    spin_thread_ = std::thread(
      [this]() {
        executor_.spin();
        spin_finished_.store(true);
      });
  }

  ~UnitreeMotionSwitcher()
  {
    // cancel() only stops a spin() that has already begun; repeat it until the
    // thread reports that spin() returned, so a cancel racing the thread start
    // cannot leave join() waiting forever.
    while (!spin_finished_.load()) {
      executor_.cancel();
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
    executor_.remove_callback_group(callback_group_);
  }

  UnitreeMotionSwitcher(const UnitreeMotionSwitcher &) = delete;
  UnitreeMotionSwitcher & operator=(const UnitreeMotionSwitcher &) = delete;

  // True once the robot's motion_switcher endpoints are discovered.
  bool waitForService(std::chrono::milliseconds max_wait)
  {
    const auto deadline = std::chrono::steady_clock::now() + max_wait;
    while (rclcpp::ok()) {
      if (node_.count_subscribers(request_topic_) > 0 &&
        node_.count_publishers(response_topic_) > 0)
      {
        return true;
      }
      if (std::chrono::steady_clock::now() >= deadline) {
        return false;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    return false;
  }

  // name is the active motion service ("normal", "ai", ...), empty when none.
  // Returns kUnknownError (instead of kSuccess with an empty name) when the
  // reply JSON has no "name" field: the unitree_ros2 client does
  // js["name"].get_to(name) and throws on a missing key, so a missing key
  // must not be treated as "released".
  int32_t checkMode(std::string & form, std::string & name)
  {
    std::string data;
    const int32_t ret = call(kApiCheckMode, data);
    form.clear();
    name.clear();
    if (ret != kSuccess) {
      return ret;
    }
    unitreeJsonStringField(data, "form", form);
    if (!unitreeJsonStringField(data, "name", name)) {
      return kUnknownError;
    }
    return kSuccess;
  }

  int32_t releaseMode()
  {
    std::string data;
    return call(kApiReleaseMode, data);
  }

private:
  int32_t call(int64_t api_id, std::string & data)
  {
    std::lock_guard<std::mutex> call_lock(call_mutex_);
    Request request;
    request.header.identity.api_id = api_id;
    // Any id unique per request; the reply echoes it (uptime ns as in unitree_ros2).
    // 0 is reserved for "not waiting", so never publish a zero id.
    int64_t id = std::chrono::steady_clock::now().time_since_epoch().count();
    if (id == 0) {
      id = 1;
    }
    request.header.identity.id = id;
    {
      std::lock_guard<std::mutex> lock(response_mutex_);
      expected_id_ = request.header.identity.id;
      response_.reset();
    }
    request_pub_->publish(request);

    std::unique_lock<std::mutex> lock(response_mutex_);
    const auto deadline = std::chrono::steady_clock::now() + timeout_;
    bool got = false;
    // Short slices so a Ctrl-C during the wait is honoured immediately.
    while (!got && rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      got = response_cv_.wait_for(
        lock, std::chrono::milliseconds(100), [this]() {return response_ != nullptr;});
    }
    const Response::ConstSharedPtr response = response_;
    expected_id_ = 0;
    response_.reset();
    if (!got || !response) {
      return kTimeout;
    }
    if (response->header.status.code != 0) {
      return response->header.status.code;
    }
    data = response->data;
    return kSuccess;
  }

  void onResponse(Response::ConstSharedPtr msg)
  {
    {
      std::lock_guard<std::mutex> lock(response_mutex_);
      if (expected_id_ == 0 || msg->header.identity.id != expected_id_) {
        return;
      }
      response_ = msg;
    }
    response_cv_.notify_all();
  }

  rclcpp::Node & node_;
  std::chrono::milliseconds timeout_;
  std::string request_topic_;
  std::string response_topic_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Publisher<Request>::SharedPtr request_pub_;
  rclcpp::Subscription<Response>::SharedPtr response_sub_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread spin_thread_;
  std::atomic<bool> spin_finished_{false};

  std::mutex call_mutex_;
  std::mutex response_mutex_;
  std::condition_variable response_cv_;
  int64_t expected_id_{0};
  Response::ConstSharedPtr response_;
};

#endif
