#include "unitree_lowcmd_map.h"
#include "unitree_lowcmd_crc.h"
#include "unitree_motion_switcher.h"
#include "motor_crc.h"

#include <champ_msgs/msg/contacts_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmw/rmw.h>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <urdf/model.h>

// Every Unitree quadruped speaks the unitree_go LowCmd/LowState messages on
// the ROS 2 topics /lowcmd and /lowstate (DDS rt/lowcmd, rt/lowstate) through
// unitree_ros2 (https://github.com/unitreerobotics/unitree_ros2). Which robot
// is behind the topics is decided only by the URDF + yaml files passed in as
// parameters; see unitree_ros2 example/src/src/{go2,b2}/*_stand_example.cpp.
#include <unitree_go/msg/low_cmd.hpp>
#include <unitree_go/msg/low_state.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <functional>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace
{
using LowCmd = unitree_go::msg::LowCmd;
using LowState = unitree_go::msg::LowState;

constexpr const char * kNodeName = "unitree_ros2_bridge";

// Robot-specific values (gains, motor mode, contact threshold) have no
// sensible universal default, so they must come from the <robot>_lowcmd.yaml.
std::string missingParam(const std::string & name, const std::string & hint)
{
  return std::string(kNodeName) + " parameter '" + name + "' is missing. " + hint +
         " Load config/<robot>_lowcmd.yaml (see unitree_sim2real.launch.py robot:=<robot>).";
}

double requireNumber(rclcpp::Node & node, const std::string & name, const std::string & hint)
{
  rclcpp::Parameter param;
  if (!node.get_parameter(name, param)) {
    throw std::invalid_argument(missingParam(name, hint));
  }
  switch (param.get_type()) {
    case rclcpp::ParameterType::PARAMETER_DOUBLE:
      return param.as_double();
    case rclcpp::ParameterType::PARAMETER_INTEGER:
      return static_cast<double>(param.as_int());
    default:
      throw std::invalid_argument(
              std::string(kNodeName) + " parameter '" + name + "' must be a number");
  }
}

double numberOr(rclcpp::Node & node, const std::string & name, double fallback)
{
  rclcpp::Parameter param;
  if (!node.get_parameter(name, param)) {
    return fallback;
  }
  switch (param.get_type()) {
    case rclcpp::ParameterType::PARAMETER_DOUBLE:
      return param.as_double();
    case rclcpp::ParameterType::PARAMETER_INTEGER:
      return static_cast<double>(param.as_int());
    default:
      throw std::invalid_argument(
              std::string(kNodeName) + " parameter '" + name + "' must be a number");
  }
}

std::string stringOr(rclcpp::Node & node, const std::string & name, const std::string & fallback)
{
  std::string value = fallback;
  node.get_parameter(name, value);
  return value;
}

std::string requireString(rclcpp::Node & node, const std::string & name, const std::string & hint)
{
  std::string value;
  if (!node.get_parameter(name, value) || value.empty()) {
    throw std::invalid_argument(missingParam(name, hint));
  }
  return value;
}

bool boolOr(rclcpp::Node & node, const std::string & name, bool fallback)
{
  bool value = fallback;
  node.get_parameter(name, value);
  return value;
}
}  // namespace

class UnitreeRos2Bridge : public rclcpp::Node
{
public:
  UnitreeRos2Bridge()
  : Node(
      kNodeName,
      rclcpp::NodeOptions()
      .allow_undeclared_parameters(true)
      .automatically_declare_parameters_from_overrides(true))
  {
    loadRobotDescription();

    kp_ = static_cast<float>(requireNumber(*this, "kp", "Position gain of every motor."));
    kd_ = static_cast<float>(requireNumber(*this, "kd", "Velocity gain of every motor."));
    tau_ = static_cast<float>(numberOr(*this, "tau", 0.0));
    const double motor_mode = requireNumber(
      *this, "motor_mode", "Servo mode byte of the robot's stand example.");
    if (motor_mode < 0.0 || motor_mode > 255.0 || motor_mode != std::floor(motor_mode)) {
      throw std::invalid_argument(
              std::string(kNodeName) + " parameter 'motor_mode' must be 0..255");
    }
    motor_mode_ = static_cast<uint8_t>(motor_mode);
    contact_force_threshold_ = requireNumber(
      *this, "contact_force_threshold", "/lowstate foot_force level that counts as contact.");
    command_timeout_sec_ = numberOr(*this, "command_timeout_sec", 0.5);
    ramp_sec_ = std::max(0.0, numberOr(*this, "ramp_sec", 2.0));
    lowcmd_topic_ = stringOr(*this, "lowcmd_topic", "/lowcmd");
    lowstate_topic_ = stringOr(*this, "lowstate_topic", "/lowstate");

    reportMiddleware();

    if (boolOr(*this, "release_sport", true)) {
      releaseSport();
    }
    if (!rclcpp::ok()) {
      throw std::runtime_error("shutdown requested while releasing Sport");
    }

    initLowCmd();
    lowcmd_pub_ = create_publisher<LowCmd>(lowcmd_topic_, rclcpp::QoS(10));
    lowstate_sub_ = create_subscription<LowState>(
      lowstate_topic_, rclcpp::QoS(10),
      std::bind(&UnitreeRos2Bridge::onLowState, this, std::placeholders::_1));

    command_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      stringOr(*this, "command_topic", "joint_commands"), 10,
      std::bind(&UnitreeRos2Bridge::onJointCommand, this, std::placeholders::_1));
    joint_pub_ = create_publisher<sensor_msgs::msg::JointState>(
      stringOr(*this, "joint_state_topic", "joint_states"), 10);
    contact_pub_ = create_publisher<champ_msgs::msg::ContactsStamped>(
      stringOr(*this, "contact_topic", "foot_contacts"), 10);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(
      stringOr(*this, "imu_topic", "imu/data"), 10);

    const double publish_rate = std::max(1.0, numberOr(*this, "publish_rate", 500.0));
    timer_ = create_wall_timer(
      std::chrono::microseconds(static_cast<int>(1e6 / publish_rate)),
      std::bind(&UnitreeRos2Bridge::onTimer, this));

    RCLCPP_WARN(
      get_logger(),
      "%s Sim2Real bridge: publish %s (motor mode 0x%02X, kp %.0f, kd %.1f), "
      "subscribe %s at %.0f Hz. Sport/control services must stay off. "
      "Do not mix with the Sport API.",
      robot_name_.c_str(), lowcmd_topic_.c_str(), static_cast<unsigned>(motor_mode_),
      static_cast<double>(kp_), static_cast<double>(kd_), lowstate_topic_.c_str(), publish_rate);
  }

private:
  // Joint names come from joints_map (CHAMP yaml), joint limits from the URDF,
  // frame ids from links_map. Nothing about a particular robot lives here.
  void loadRobotDescription()
  {
    const std::string urdf_xml = requireString(
      *this, "urdf", "Pass the robot URDF string (the launch files do this).");
    urdf::Model model;
    if (!model.initString(urdf_xml)) {
      throw std::invalid_argument(
              std::string(kNodeName) + " could not parse the 'urdf' parameter");
    }
    robot_name_ = model.getName();
    if (robot_name_.empty()) {
      robot_name_ = "unitree";
    }

    std::array<std::vector<std::string>, kUnitreeLegCount> legs;
    for (int leg = 0; leg < kUnitreeLegCount; ++leg) {
      const std::string key = std::string("joints_map.") + kUnitreeLegKeys[leg];
      if (!get_parameter(key, legs[static_cast<size_t>(leg)])) {
        throw std::invalid_argument(
                missingParam(key, "Joint names of that leg (hip, thigh, calf, foot)."));
      }
    }
    map_ = unitreeMotorMapFromLegs(legs);
    limits_ = unitreeJointLimitsFromUrdf(model, map_);

    base_frame_ = requireString(*this, "links_map.base", "Name of the URDF base link.");
    if (!model.getLink(base_frame_)) {
      throw std::invalid_argument(
              "links_map.base '" + base_frame_ + "' is not a link of the URDF");
    }
    imu_frame_ = requireString(*this, "links_map.imu", "Name of the URDF IMU link.");
    if (!model.getLink(imu_frame_)) {
      throw std::invalid_argument(
              "links_map.imu '" + imu_frame_ + "' is not a link of the URDF");
    }

    RCLCPP_INFO(
      get_logger(),
      "%s: LowCmd slots FR/FL/RR/RL x hip/thigh/calf -> %s ... %s; limits from URDF",
      robot_name_.c_str(), map_.joint_names.front().c_str(), map_.joint_names.back().c_str());
  }

  // unitree_ros2 talks to the robot through the ROS 2 graph itself: the robot
  // is a CycloneDDS participant on domain 0 reachable over the cabled NIC. The
  // node cannot choose the NIC (that is RMW_IMPLEMENTATION + CYCLONEDDS_URI,
  // set by unitree_ros2/setup.sh or the launch network_interface argument),
  // it can only tell the operator what it ended up with.
  void reportMiddleware()
  {
    const std::string rmw = rmw_get_implementation_identifier();
    const char * cyclone_uri = std::getenv("CYCLONEDDS_URI");
    if (rmw != "rmw_cyclonedds_cpp") {
      RCLCPP_WARN(
        get_logger(),
        "RMW is %s; unitree_ros2 needs RMW_IMPLEMENTATION=rmw_cyclonedds_cpp to reach the %s.",
        rmw.c_str(), robot_name_.c_str());
    } else if (cyclone_uri == nullptr || std::string(cyclone_uri).empty()) {
      RCLCPP_WARN(
        get_logger(),
        "CYCLONEDDS_URI is not set: CycloneDDS picks a NIC by itself. Source unitree_ros2 "
        "setup.sh or launch with network_interface:=<nic cabled to the %s>.",
        robot_name_.c_str());
    } else {
      RCLCPP_INFO(get_logger(), "RMW %s, CYCLONEDDS_URI set", rmw.c_str());
    }
  }

  void releaseSport()
  {
    try {
      UnitreeMotionSwitcher msc(*this, std::chrono::seconds(5));
      if (!msc.waitForService(std::chrono::seconds(2))) {
        RCLCPP_WARN(
          get_logger(),
          "motion_switcher service not discovered on the ROS 2 graph. Is the %s cabled to the "
          "NIC in CYCLONEDDS_URI, on ROS domain 0, with RMW cyclonedds?",
          robot_name_.c_str());
      }
      // Same policy as the unitree_ros2 stand examples: while the robot answers
      // that a motion service is still active, keep releasing and never start
      // LowCmd underneath it. Give up (and continue) only when CheckMode times
      // out repeatedly — the robot is not on the graph. A non-timeout error is
      // still an answer, so it is not treated as "unreachable".
      int timeouts = 0;
      for (int attempt = 1; rclcpp::ok(); ++attempt) {
        std::string form;
        std::string name;
        const int32_t check = msc.checkMode(form, name);
        if (check == UnitreeMotionSwitcher::kSuccess && name.empty()) {
          RCLCPP_INFO(get_logger(), "Motion control services are off (Sport released).");
          return;
        }
        const int32_t ret = msc.releaseMode();
        if (check == UnitreeMotionSwitcher::kTimeout) {
          ++timeouts;
          RCLCPP_INFO(
            get_logger(),
            "Attempt %d: motion_switcher did not answer (CheckMode %d, ReleaseMode %d).",
            attempt, check, ret);
          if (timeouts >= 6) {
            RCLCPP_WARN(
              get_logger(),
              "motion_switcher unreachable after %d timed-out attempts. Continuing; make sure "
              "Sport is off on the %s before it joins the network.",
              timeouts, robot_name_.c_str());
            return;
          }
        } else {
          timeouts = 0;
          if (check == UnitreeMotionSwitcher::kSuccess) {
            RCLCPP_WARN(
              get_logger(),
              "Attempt %d: motion service '%s' (form %s) still active on the %s, ReleaseMode "
              "returned %d; retrying, no LowCmd until it is off.",
              attempt, name.c_str(), form.c_str(), robot_name_.c_str(), ret);
          } else {
            RCLCPP_WARN(
              get_logger(),
              "Attempt %d: motion_switcher answered CheckMode %d (ReleaseMode %d) on the %s; "
              "retrying, no LowCmd until CheckMode reports an empty service name.",
              attempt, check, ret, robot_name_.c_str());
          }
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
      }
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(
        get_logger(),
        "Could not ReleaseMode (%s). Not sending LowCmd.",
        ex.what());
      throw;
    }
  }

  void initLowCmd()
  {
    low_cmd_.head[0] = 0xFE;
    low_cmd_.head[1] = 0xEF;
    low_cmd_.level_flag = LOWLEVEL;
    low_cmd_.gpio = 0;
    for (size_t i = 0; i < low_cmd_.motor_cmd.size(); ++i) {
      low_cmd_.motor_cmd[i].mode = motor_mode_;
      low_cmd_.motor_cmd[i].q = static_cast<float>(PosStopF);
      low_cmd_.motor_cmd[i].dq = static_cast<float>(VelStopF);
      low_cmd_.motor_cmd[i].kp = 0.0f;
      low_cmd_.motor_cmd[i].kd = 0.0f;
      low_cmd_.motor_cmd[i].tau = 0.0f;
    }
  }

  void onLowState(LowState::ConstSharedPtr state)
  {
    std::array<double, kUnitreeMotorCount> q{};
    std::array<double, kUnitreeMotorCount> dq{};
    std::array<double, kUnitreeLegCount> force{};
    for (int i = 0; i < kUnitreeMotorCount; ++i) {
      q[static_cast<size_t>(i)] = state->motor_state[static_cast<size_t>(i)].q;
      dq[static_cast<size_t>(i)] = state->motor_state[static_cast<size_t>(i)].dq;
    }
    for (int i = 0; i < kUnitreeLegCount; ++i) {
      force[static_cast<size_t>(i)] = static_cast<double>(state->foot_force[static_cast<size_t>(i)]);
    }

    sensor_msgs::msg::Imu imu;
    const auto & imu_state = state->imu_state;
    imu.header.frame_id = imu_frame_;
    imu.orientation.w = imu_state.quaternion[0];
    imu.orientation.x = imu_state.quaternion[1];
    imu.orientation.y = imu_state.quaternion[2];
    imu.orientation.z = imu_state.quaternion[3];
    imu.angular_velocity.x = imu_state.gyroscope[0];
    imu.angular_velocity.y = imu_state.gyroscope[1];
    imu.angular_velocity.z = imu_state.gyroscope[2];
    imu.linear_acceleration.x = imu_state.accelerometer[0];
    imu.linear_acceleration.y = imu_state.accelerometer[1];
    imu.linear_acceleration.z = imu_state.accelerometer[2];

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
    std::array<double, kUnitreeMotorCount> q{};
    std::array<bool, kUnitreeMotorCount> seen{};
    const size_t n = std::min(msg->name.size(), msg->position.size());
    for (size_t i = 0; i < n; ++i) {
      const int idx = map_.motorIndex(msg->name[i]);
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
    for (int i = 0; i < kUnitreeMotorCount; ++i) {
      if (!seen[static_cast<size_t>(i)]) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "JointState is missing joint '%s' (joints_map name of LowCmd motor %d)",
          map_.joint_names[static_cast<size_t>(i)].c_str(), i);
        return;
      }
    }

    std::lock_guard<std::mutex> lock(mutex_);
    command_q_ = q;
    has_command_ = true;
    last_command_monotonic_ = std::chrono::steady_clock::now();
  }

  void onTimer()
  {
    std::array<double, kUnitreeMotorCount> measured{};
    std::array<double, kUnitreeMotorCount> command{};
    std::array<double, kUnitreeMotorCount> start_q{};
    std::array<double, kUnitreeMotorCount> measured_dq{};
    std::array<double, kUnitreeLegCount> force{};
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
    // A stale command holds position with at least a modest damping gain.
    const float kd = command_stale ? std::max(kd_, 2.0f) : kd_;
    int clamped = 0;
    for (int i = 0; i < kUnitreeMotorCount; ++i) {
      const float q_meas = static_cast<float>(measured[static_cast<size_t>(i)]);
      float q_des = q_meas;
      if (have_command) {
        const float q_cmd = static_cast<float>(command[static_cast<size_t>(i)]);
        const float q_start = static_cast<float>(start_q[static_cast<size_t>(i)]);
        q_des = (1.0f - blend) * q_start + blend * q_cmd;
      }
      const float q_safe = unitreeClampJoint(limits_, i, q_des);
      if (q_safe != q_des) {
        ++clamped;
        q_des = q_safe;
      }
      auto & cmd = low_cmd_.motor_cmd[static_cast<size_t>(i)];
      cmd.mode = motor_mode_;
      cmd.q = q_des;
      cmd.dq = 0.0f;
      cmd.kp = kp;
      cmd.kd = kd;
      cmd.tau = tau_;
    }
    if (clamped > 0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "%d joint target(s) outside %s URDF limits were clamped before %s",
        clamped, robot_name_.c_str(), lowcmd_topic_.c_str());
    }
    unitreeSetLowCmdCrc(low_cmd_);
    lowcmd_pub_->publish(low_cmd_);
  }

  void publishMeasurements(
    const builtin_interfaces::msg::Time & stamp,
    const std::array<double, kUnitreeMotorCount> & q,
    const std::array<double, kUnitreeMotorCount> & dq,
    const std::array<double, kUnitreeLegCount> & force,
    sensor_msgs::msg::Imu imu)
  {
    sensor_msgs::msg::JointState joints;
    joints.header.stamp = stamp;
    joints.name.assign(map_.joint_names.begin(), map_.joint_names.end());
    joints.position.assign(q.begin(), q.end());
    joints.velocity.assign(dq.begin(), dq.end());
    joint_pub_->publish(joints);

    champ_msgs::msg::ContactsStamped contacts;
    contacts.header.stamp = stamp;
    contacts.header.frame_id = base_frame_;
    contacts.contacts = {false, false, false, false};
    for (int i = 0; i < kUnitreeLegCount; ++i) {
      contacts.contacts[kChampLegFromUnitreeLeg[i]] =
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

  rclcpp::Publisher<LowCmd>::SharedPtr lowcmd_pub_;
  rclcpp::Subscription<LowState>::SharedPtr lowstate_sub_;
  LowCmd low_cmd_{};

  std::mutex mutex_;
  std::array<double, kUnitreeMotorCount> command_q_{};
  std::array<double, kUnitreeMotorCount> start_q_{};
  std::array<double, kUnitreeMotorCount> measured_q_{};
  std::array<double, kUnitreeMotorCount> measured_dq_{};
  std::array<double, kUnitreeLegCount> foot_force_{};
  sensor_msgs::msg::Imu last_imu_;
  bool has_command_{false};
  bool has_lowstate_{false};
  bool ramp_started_{false};
  std::chrono::steady_clock::time_point last_command_monotonic_{};
  std::chrono::steady_clock::time_point ramp_start_monotonic_{};

  std::string robot_name_;
  UnitreeMotorMap map_;
  UnitreeJointLimits limits_;
  std::string base_frame_;
  std::string imu_frame_;
  std::string lowcmd_topic_;
  std::string lowstate_topic_;
  float kp_{0.0f};
  float kd_{0.0f};
  float tau_{0.0f};
  uint8_t motor_mode_{0};
  double command_timeout_sec_{0.5};
  double ramp_sec_{2.0};
  double contact_force_threshold_{0.0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int status = 0;
  try {
    rclcpp::spin(std::make_shared<UnitreeRos2Bridge>());
  } catch (const std::exception & e) {
    if (rclcpp::ok()) {
      RCLCPP_FATAL(rclcpp::get_logger(kNodeName), "%s", e.what());
      status = 1;
    }
  }
  rclcpp::shutdown();
  return status;
}
