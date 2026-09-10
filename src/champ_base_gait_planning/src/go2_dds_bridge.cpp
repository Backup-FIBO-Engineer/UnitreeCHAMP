#include "go2_lowcmd_map.h"
#include "motor_crc.h"

#include <champ_msgs/msg/contacts_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <builtin_interfaces/msg/time.hpp>

#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

namespace
{
using LowCmd = unitree_go::msg::dds_::LowCmd_;
using LowState = unitree_go::msg::dds_::LowState_;
}

class Go2DdsBridge : public rclcpp::Node
{
public:
  Go2DdsBridge()
  : Node("go2_dds_bridge")
  {
    declare_parameter("command_topic", std::string("joint_commands"));
    declare_parameter("joint_state_topic", std::string("joint_states"));
    declare_parameter("contact_topic", std::string("foot_contacts"));
    declare_parameter("imu_topic", std::string("imu/data"));
    declare_parameter("network_interface", std::string(""));
    declare_parameter("publish_rate", 500.0);
    declare_parameter("kp", 40.0);
    declare_parameter("kd", 1.0);
    declare_parameter("tau", 0.0);
    declare_parameter("motor_mode", 0x01);
    declare_parameter("command_timeout_sec", 0.5);
    declare_parameter("ramp_sec", 2.0);
    declare_parameter("contact_force_threshold", 20.0);
    declare_parameter("release_sport", true);

    kp_ = static_cast<float>(get_parameter("kp").as_double());
    kd_ = static_cast<float>(get_parameter("kd").as_double());
    tau_ = static_cast<float>(get_parameter("tau").as_double());
    motor_mode_ = static_cast<uint8_t>(get_parameter("motor_mode").as_int());
    command_timeout_sec_ = get_parameter("command_timeout_sec").as_double();
    ramp_sec_ = std::max(0.0, get_parameter("ramp_sec").as_double());
    contact_force_threshold_ = get_parameter("contact_force_threshold").as_double();

    const auto iface = get_parameter("network_interface").as_string();
    if (iface.empty()) {
      unitree::robot::ChannelFactory::Instance()->Init(0);
      RCLCPP_WARN(
        get_logger(),
        "unitree_sdk2 DDS domain 0 on the default interface. "
        "Pass network_interface:=eth0 (or the NIC cabled to the Go2).");
    } else {
      unitree::robot::ChannelFactory::Instance()->Init(0, iface.c_str());
      RCLCPP_INFO(get_logger(), "unitree_sdk2 DDS domain 0 on %s", iface.c_str());
    }

    if (get_parameter("release_sport").as_bool()) {
      releaseSport();
    }

    initLowCmd();
    lowcmd_pub_.reset(new unitree::robot::ChannelPublisher<LowCmd>("rt/lowcmd"));
    lowcmd_pub_->InitChannel();
    lowstate_sub_.reset(new unitree::robot::ChannelSubscriber<LowState>("rt/lowstate"));
    lowstate_sub_->InitChannel(
      std::bind(&Go2DdsBridge::onLowState, this, std::placeholders::_1), 1);

    command_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      get_parameter("command_topic").as_string(), 10,
      std::bind(&Go2DdsBridge::onJointCommand, this, std::placeholders::_1));
    joint_pub_ = create_publisher<sensor_msgs::msg::JointState>(
      get_parameter("joint_state_topic").as_string(), 10);
    contact_pub_ = create_publisher<champ_msgs::msg::ContactsStamped>(
      get_parameter("contact_topic").as_string(), 10);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(
      get_parameter("imu_topic").as_string(), 10);

    const double publish_rate = std::max(1.0, get_parameter("publish_rate").as_double());
    timer_ = create_wall_timer(
      std::chrono::microseconds(static_cast<int>(1e6 / publish_rate)),
      std::bind(&Go2DdsBridge::onTimer, this));

    RCLCPP_WARN(
      get_logger(),
      "Go2 Sim2Real DDS bridge: publish rt/lowcmd, subscribe rt/lowstate at %.0f Hz. "
      "Sport/control services must stay off. Do not mix with the Sport API.",
      publish_rate);
  }

private:
  void releaseSport()
  {
    try {
      unitree::robot::b2::MotionSwitcherClient msc;
      msc.SetTimeout(5.0f);
      msc.Init();
      for (int attempt = 0; attempt < 6; ++attempt) {
        std::string form;
        std::string name;
        const int32_t check = msc.CheckMode(form, name);
        if (check == 0 && name.empty()) {
          RCLCPP_INFO(get_logger(), "Motion control services are off (Sport released).");
          return;
        }
        const int32_t ret = msc.ReleaseMode();
        RCLCPP_INFO(
          get_logger(),
          "ReleaseMode attempt %d returned %d (active='%s').",
          attempt + 1, ret, name.c_str());
        std::this_thread::sleep_for(std::chrono::seconds(1));
      }
      RCLCPP_WARN(
        get_logger(),
        "Sport may still be active. Stop it on the robot before walking.");
    } catch (const std::exception & ex) {
      RCLCPP_WARN(
        get_logger(),
        "Could not ReleaseMode (%s). Stop Sport manually before sending LowCmd.",
        ex.what());
    }
  }

  void initLowCmd()
  {
    low_cmd_.head()[0] = 0xFE;
    low_cmd_.head()[1] = 0xEF;
    low_cmd_.level_flag() = LOWLEVEL;
    low_cmd_.gpio() = 0;
    for (int i = 0; i < 20; ++i) {
      // Official unitree_sdk2 go2_stand_example: PMSM servo mode 0x01.
      low_cmd_.motor_cmd()[i].mode() = motor_mode_;
      low_cmd_.motor_cmd()[i].q() = static_cast<float>(PosStopF);
      low_cmd_.motor_cmd()[i].dq() = static_cast<float>(VelStopF);
      low_cmd_.motor_cmd()[i].kp() = 0.0f;
      low_cmd_.motor_cmd()[i].kd() = 0.0f;
      low_cmd_.motor_cmd()[i].tau() = 0.0f;
    }
  }

  void onLowState(const void * message)
  {
    const auto * state = static_cast<const LowState *>(message);
    std::array<double, kGo2MotorCount> q{};
    std::array<double, kGo2MotorCount> dq{};
    std::array<double, 4> force{};
    for (int i = 0; i < kGo2MotorCount; ++i) {
      q[static_cast<size_t>(i)] = state->motor_state()[i].q();
      dq[static_cast<size_t>(i)] = state->motor_state()[i].dq();
    }
    for (int i = 0; i < 4; ++i) {
      force[static_cast<size_t>(i)] = static_cast<double>(state->foot_force()[i]);
    }

    sensor_msgs::msg::Imu imu;
    const auto & imu_state = state->imu_state();
    imu.header.frame_id = "imu";
    imu.orientation.w = imu_state.quaternion()[0];
    imu.orientation.x = imu_state.quaternion()[1];
    imu.orientation.y = imu_state.quaternion()[2];
    imu.orientation.z = imu_state.quaternion()[3];
    imu.angular_velocity.x = imu_state.gyroscope()[0];
    imu.angular_velocity.y = imu_state.gyroscope()[1];
    imu.angular_velocity.z = imu_state.gyroscope()[2];
    imu.linear_acceleration.x = imu_state.accelerometer()[0];
    imu.linear_acceleration.y = imu_state.accelerometer()[1];
    imu.linear_acceleration.z = imu_state.accelerometer()[2];

    {
      std::lock_guard<std::mutex> lock(mutex_);
      measured_q_ = q;
      measured_dq_ = dq;
      foot_force_ = force;
      last_imu_ = imu;
      has_lowstate_ = true;
    }
  }

  void onJointCommand(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    std::array<double, kGo2MotorCount> q{};
    std::array<bool, kGo2MotorCount> seen{};
    const size_t n = std::min(msg->name.size(), msg->position.size());
    for (size_t i = 0; i < n; ++i) {
      const int idx = go2MotorIndex(msg->name[i]);
      if (idx < 0) {
        continue;
      }
      if (!std::isfinite(msg->position[i])) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000, "Ignoring non-finite joint %s",
          msg->name[i].c_str());
        return;
      }
      q[static_cast<size_t>(idx)] = msg->position[i];
      seen[static_cast<size_t>(idx)] = true;
    }
    if (!std::all_of(seen.begin(), seen.end(), [](bool v) { return v; })) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "JointState is missing a Go2 motor name (need FR/FL/RR/RL hip, thigh, calf)");
      return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    command_q_ = q;
    has_command_ = true;
    last_command_monotonic_ = std::chrono::steady_clock::now();
  }

  void onTimer()
  {
    std::array<double, kGo2MotorCount> measured{};
    std::array<double, kGo2MotorCount> command{};
    std::array<double, kGo2MotorCount> start_q{};
    std::array<double, kGo2MotorCount> measured_dq{};
    std::array<double, 4> force{};
    sensor_msgs::msg::Imu imu;
    bool have_state = false;
    bool have_command = false;
    double ramp_t = 1.0;
    bool command_stale = true;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      have_state = has_lowstate_;
      have_command = has_command_;
      measured = measured_q_;
      measured_dq = measured_dq_;
      command = command_q_;
      force = foot_force_;
      imu = last_imu_;
      const auto now = std::chrono::steady_clock::now();
      if (have_state && have_command && !ramp_started_) {
        ramp_started_ = true;
        ramp_start_monotonic_ = now;
        start_q_ = measured;
      }
      start_q = start_q_;
      if (have_command) {
        command_stale = command_timeout_sec_ > 0.0 &&
          std::chrono::duration<double>(now - last_command_monotonic_).count() >
          command_timeout_sec_;
        if (ramp_started_ && ramp_sec_ > 0.0) {
          ramp_t = std::chrono::duration<double>(now - ramp_start_monotonic_).count() / ramp_sec_;
        }
      }
    }

    if (!have_state) {
      return;
    }

    const builtin_interfaces::msg::Time stamp = this->now();
    publishMeasurements(stamp, measured, measured_dq, force, imu);

    const float blend = static_cast<float>(std::min(1.0, std::max(0.0, ramp_t)));
    const float kp = kp_;
    const float kd = command_stale ? std::max(kd_, 2.0f) : kd_;
    for (int i = 0; i < kGo2MotorCount; ++i) {
      const float q_meas = static_cast<float>(measured[static_cast<size_t>(i)]);
      float q_des = q_meas;
      if (have_command) {
        const float q_cmd = static_cast<float>(command[static_cast<size_t>(i)]);
        const float q_start = static_cast<float>(start_q[static_cast<size_t>(i)]);
        q_des = (1.0f - blend) * q_start + blend * q_cmd;
      }
      low_cmd_.motor_cmd()[i].mode() = motor_mode_;
      low_cmd_.motor_cmd()[i].q() = q_des;
      low_cmd_.motor_cmd()[i].dq() = 0.0f;
      low_cmd_.motor_cmd()[i].kp() = kp;
      low_cmd_.motor_cmd()[i].kd() = kd;
      low_cmd_.motor_cmd()[i].tau() = tau_;
    }
    low_cmd_.crc() = crc32_core(
      reinterpret_cast<uint32_t *>(&low_cmd_), (sizeof(LowCmd) >> 2) - 1);
    lowcmd_pub_->Write(low_cmd_);
  }

  void publishMeasurements(
    const builtin_interfaces::msg::Time & stamp,
    const std::array<double, kGo2MotorCount> & q,
    const std::array<double, kGo2MotorCount> & dq,
    const std::array<double, 4> & force,
    sensor_msgs::msg::Imu imu)
  {
    sensor_msgs::msg::JointState joints;
    joints.header.stamp = stamp;
    joints.name.assign(kGo2MotorJointNames, kGo2MotorJointNames + kGo2MotorCount);
    joints.position.assign(q.begin(), q.end());
    joints.velocity.assign(dq.begin(), dq.end());
    joint_pub_->publish(joints);

    champ_msgs::msg::ContactsStamped contacts;
    contacts.header.stamp = stamp;
    contacts.header.frame_id = "base";
    contacts.contacts = {false, false, false, false};
    for (int i = 0; i < 4; ++i) {
      contacts.contacts[kChampIndexFromFootForce[i]] =
        force[static_cast<size_t>(i)] > contact_force_threshold_;
    }
    contact_pub_->publish(contacts);

    imu.header.stamp = stamp;
    imu_pub_->publish(imu);
  }

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr command_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Publisher<champ_msgs::msg::ContactsStamped>::SharedPtr contact_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  unitree::robot::ChannelPublisherPtr<LowCmd> lowcmd_pub_;
  unitree::robot::ChannelSubscriberPtr<LowState> lowstate_sub_;
  LowCmd low_cmd_{};

  std::mutex mutex_;
  std::array<double, kGo2MotorCount> command_q_{};
  std::array<double, kGo2MotorCount> start_q_{};
  std::array<double, kGo2MotorCount> measured_q_{};
  std::array<double, kGo2MotorCount> measured_dq_{};
  std::array<double, 4> foot_force_{};
  sensor_msgs::msg::Imu last_imu_;
  bool has_command_{false};
  bool has_lowstate_{false};
  bool ramp_started_{false};
  std::chrono::steady_clock::time_point last_command_monotonic_{};
  std::chrono::steady_clock::time_point ramp_start_monotonic_{};

  float kp_{40.0f};
  float kd_{1.0f};
  float tau_{0.0f};
  uint8_t motor_mode_{0x01};
  double command_timeout_sec_{0.5};
  double ramp_sec_{2.0};
  double contact_force_threshold_{20.0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Go2DdsBridge>());
  rclcpp::shutdown();
  return 0;
}
