#include "unitree_lowcmd_map.h"
#include "motor_crc.h"

#include <champ_msgs/msg/contacts_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <urdf/model.h>

// Every Unitree quadruped speaks the unitree_go LowCmd_/LowState_ IDL on
// rt/lowcmd / rt/lowstate; the "go2" in the include path and the "b2" in the
// MotionSwitcherClient namespace are unitree_sdk2 naming, not robot selection
// (see unitree_sdk2 example/b2/b2_stand_example.cpp which uses both).
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
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace
{
using LowCmd = unitree_go::msg::dds_::LowCmd_;
using LowState = unitree_go::msg::dds_::LowState_;

// Robot-specific values (gains, motor mode, contact threshold) have no
// sensible universal default, so they must come from the <robot>_lowcmd.yaml.
std::string missingParam(const std::string & name, const std::string & hint)
{
  return "unitree_dds_bridge parameter '" + name + "' is missing. " + hint +
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
              "unitree_dds_bridge parameter '" + name + "' must be a number");
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
              "unitree_dds_bridge parameter '" + name + "' must be a number");
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

class UnitreeDdsBridge : public rclcpp::Node
{
public:
  UnitreeDdsBridge()
  : Node(
      "unitree_dds_bridge",
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
      throw std::invalid_argument("unitree_dds_bridge parameter 'motor_mode' must be 0..255");
    }
    motor_mode_ = static_cast<uint8_t>(motor_mode);
    contact_force_threshold_ = requireNumber(
      *this, "contact_force_threshold", "rt/lowstate foot_force level that counts as contact.");
    command_timeout_sec_ = numberOr(*this, "command_timeout_sec", 0.5);
    ramp_sec_ = std::max(0.0, numberOr(*this, "ramp_sec", 2.0));

    const auto iface = stringOr(*this, "network_interface", "");
    if (iface.empty()) {
      unitree::robot::ChannelFactory::Instance()->Init(0);
      RCLCPP_WARN(
        get_logger(),
        "unitree_sdk2 DDS domain 0 on the default interface. "
        "Pass network_interface:=eth0 (or the NIC cabled to the %s).",
        robot_name_.c_str());
    } else {
      unitree::robot::ChannelFactory::Instance()->Init(0, iface.c_str());
      RCLCPP_INFO(get_logger(), "unitree_sdk2 DDS domain 0 on %s", iface.c_str());
    }

    if (boolOr(*this, "release_sport", true)) {
      releaseSport();
    }

    initLowCmd();
    lowcmd_pub_.reset(new unitree::robot::ChannelPublisher<LowCmd>("rt/lowcmd"));
    lowcmd_pub_->InitChannel();
    lowstate_sub_.reset(new unitree::robot::ChannelSubscriber<LowState>("rt/lowstate"));
    lowstate_sub_->InitChannel(
      std::bind(&UnitreeDdsBridge::onLowState, this, std::placeholders::_1), 1);

    command_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      stringOr(*this, "command_topic", "joint_commands"), 10,
      std::bind(&UnitreeDdsBridge::onJointCommand, this, std::placeholders::_1));
    joint_pub_ = create_publisher<sensor_msgs::msg::JointState>(
      stringOr(*this, "joint_state_topic", "joint_states"), 10);
    contact_pub_ = create_publisher<champ_msgs::msg::ContactsStamped>(
      stringOr(*this, "contact_topic", "foot_contacts"), 10);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(
      stringOr(*this, "imu_topic", "imu/data"), 10);

    const double publish_rate = std::max(1.0, numberOr(*this, "publish_rate", 500.0));
    timer_ = create_wall_timer(
      std::chrono::microseconds(static_cast<int>(1e6 / publish_rate)),
      std::bind(&UnitreeDdsBridge::onTimer, this));

    RCLCPP_WARN(
      get_logger(),
      "%s Sim2Real DDS bridge: publish rt/lowcmd (motor mode 0x%02X, kp %.0f, kd %.1f), "
      "subscribe rt/lowstate at %.0f Hz. Sport/control services must stay off. "
      "Do not mix with the Sport API.",
      robot_name_.c_str(), static_cast<unsigned>(motor_mode_),
      static_cast<double>(kp_), static_cast<double>(kd_), publish_rate);
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
      throw std::invalid_argument("unitree_dds_bridge could not parse the 'urdf' parameter");
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
      "%s: rt/lowcmd slots FR/FL/RR/RL x hip/thigh/calf -> %s ... %s; limits from URDF",
      robot_name_.c_str(), map_.joint_names.front().c_str(), map_.joint_names.back().c_str());
  }

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
    std::array<double, kUnitreeMotorCount> q{};
    std::array<double, kUnitreeMotorCount> dq{};
    std::array<double, kUnitreeLegCount> force{};
    for (int i = 0; i < kUnitreeMotorCount; ++i) {
      q[static_cast<size_t>(i)] = state->motor_state()[i].q();
      dq[static_cast<size_t>(i)] = state->motor_state()[i].dq();
    }
    for (int i = 0; i < kUnitreeLegCount; ++i) {
      force[static_cast<size_t>(i)] = static_cast<double>(state->foot_force()[i]);
    }

    sensor_msgs::msg::Imu imu;
    const auto & imu_state = state->imu_state();
    imu.header.frame_id = imu_frame_;
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
          "JointState is missing joint '%s' (joints_map name of rt/lowcmd motor %d)",
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
      low_cmd_.motor_cmd()[i].mode() = motor_mode_;
      low_cmd_.motor_cmd()[i].q() = q_des;
      low_cmd_.motor_cmd()[i].dq() = 0.0f;
      low_cmd_.motor_cmd()[i].kp() = kp;
      low_cmd_.motor_cmd()[i].kd() = kd;
      low_cmd_.motor_cmd()[i].tau() = tau_;
    }
    if (clamped > 0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "%d joint target(s) outside %s URDF limits were clamped before rt/lowcmd",
        clamped, robot_name_.c_str());
    }
    low_cmd_.crc() = crc32_core(
      reinterpret_cast<uint32_t *>(&low_cmd_), (sizeof(LowCmd) >> 2) - 1);
    lowcmd_pub_->Write(low_cmd_);
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

  unitree::robot::ChannelPublisherPtr<LowCmd> lowcmd_pub_;
  unitree::robot::ChannelSubscriberPtr<LowState> lowstate_sub_;
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
    rclcpp::spin(std::make_shared<UnitreeDdsBridge>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("unitree_dds_bridge"), "%s", e.what());
    status = 1;
  }
  rclcpp::shutdown();
  return status;
}
