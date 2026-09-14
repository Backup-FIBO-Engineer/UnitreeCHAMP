#include "body_pose_controller.h"

#include <urdf/model.h>

#include <algorithm>
#include <cmath>
#include <functional>
#include <stdexcept>
#include <vector>

namespace
{
constexpr const char * kNodeName = "body_pose_controller_node";

std::string missingParam(const std::string & name, const std::string & hint)
{
  return std::string(kNodeName) + " parameter '" + name + "' is missing. " + hint +
         " Load config/<robot>_body_pose.yaml (mujoco_sim.launch.py / "
         "unitree_sim2real.launch.py body_pose_control:=true).";
}

double asNumber(const rclcpp::Parameter & param, const std::string & name)
{
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

double requireNumber(rclcpp::Node & node, const std::string & name, const std::string & hint)
{
  rclcpp::Parameter param;
  if (!node.get_parameter(name, param)) {
    throw std::invalid_argument(missingParam(name, hint));
  }
  return asNumber(param, name);
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

champ_body_pose::Quaternion toQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  return champ_body_pose::Quaternion{q.w, q.x, q.y, q.z};
}

geometry_msgs::msg::Quaternion toMsg(const champ_body_pose::Quaternion & q)
{
  geometry_msgs::msg::Quaternion msg;
  msg.w = q.w;
  msg.x = q.x;
  msg.y = q.y;
  msg.z = q.z;
  return msg;
}

bool finiteQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  return std::isfinite(q.w) && std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z) &&
         (q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z) > 1e-6;
}
}  // namespace

BodyPoseController::BodyPoseController()
: Node(
    kNodeName,
    rclcpp::NodeOptions()
    .allow_undeclared_parameters(true)
    .automatically_declare_parameters_from_overrides(true))
{
  loadController(loadImuMount());

  imu_topic_ = stringOr(*this, "imu_topic", "imu/data");
  const std::string desired_topic = stringOr(*this, "desired_pose_topic", "body_pose");
  const std::string command_topic = stringOr(*this, "command_pose_topic", "body_pose/corrected");

  // Best-effort matches both reliable and sensor-data IMU publishers
  // (unitree_ros2_bridge, mujoco_sim.py or an external IMU driver).
  imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
    imu_topic_, rclcpp::SensorDataQoS(),
    std::bind(&BodyPoseController::imuCallback, this, std::placeholders::_1));
  desired_subscription_ = create_subscription<geometry_msgs::msg::Pose>(
    desired_topic, 10,
    std::bind(&BodyPoseController::desiredPoseCallback, this, std::placeholders::_1));
  command_publisher_ = create_publisher<geometry_msgs::msg::Pose>(command_topic, 10);
  measured_publisher_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
    "body_pose/measured_rpy", 10);
  correction_publisher_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
    "body_pose/correction", 10);

  last_loop_time_ = Clock::now();
  loop_timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / control_rate_),
    std::bind(&BodyPoseController::controlLoop, this));

  const auto & cfg = controller_.config();
  RCLCPP_INFO(
    get_logger(),
    "IMU body roll/pitch loop: %s + %s -> %s at %.0f Hz (kp %.3f ki %.3f kd %.3f, "
    "|u| <= %.3f rad at <= %.2f rad/s, desired ramped at <= %.2f rad/s, |roll| <= %.3f, "
    "|pitch| <= %.3f, base %s, imu %s)",
    desired_topic.c_str(), imu_topic_.c_str(), command_topic.c_str(), control_rate_,
    cfg.gains.kp, cfg.gains.ki, cfg.gains.kd, cfg.gains.max_correction, cfg.gains.max_rate,
    desired_rate_, cfg.max_roll, cfg.max_pitch, base_frame_.c_str(), imu_frame_.c_str());
}

champ_body_pose::Quaternion BodyPoseController::loadImuMount()
{
  const std::string urdf_xml = requireString(*this, "urdf", "URDF text of the robot.");
  urdf::Model model;
  if (!model.initString(urdf_xml)) {
    throw std::invalid_argument(std::string(kNodeName) + ": parameter 'urdf' is not a valid URDF");
  }
  base_frame_ = requireString(*this, "links_map.base", "Name of the URDF base link.");
  imu_frame_ = requireString(*this, "links_map.imu", "Name of the URDF IMU link.");
  if (!model.getLink(base_frame_)) {
    throw std::invalid_argument(
            std::string(kNodeName) + ": links_map.base '" + base_frame_ +
            "' is not a link of the URDF");
  }

  rclcpp::Parameter mount_param;
  if (get_parameter("body_pose.imu_mount_rpy", mount_param)) {
    // Override for IMU data that is not expressed in links_map.imu (e.g. an
    // external driver that already aligns the axes with the base: [0, 0, 0]).
    const std::vector<double> rpy = mount_param.as_double_array();
    if (rpy.size() != 3) {
      throw std::invalid_argument(
              std::string(kNodeName) + " parameter 'body_pose.imu_mount_rpy' needs [roll, pitch, yaw]");
    }
    if (!std::isfinite(rpy[0]) || !std::isfinite(rpy[1]) || !std::isfinite(rpy[2])) {
      throw std::invalid_argument(
              std::string(kNodeName) + " parameter 'body_pose.imu_mount_rpy' must be finite");
    }
    RCLCPP_INFO(
      get_logger(), "IMU mount from body_pose.imu_mount_rpy: %.4f %.4f %.4f rad",
      rpy[0], rpy[1], rpy[2]);
    return champ_body_pose::quaternionFromRpy(rpy[0], rpy[1], rpy[2]);
  }

  urdf::LinkConstSharedPtr link = model.getLink(imu_frame_);
  if (!link) {
    throw std::invalid_argument(
            std::string(kNodeName) + ": links_map.imu '" + imu_frame_ +
            "' is not a link of the URDF (set body_pose.imu_mount_rpy for an external IMU)");
  }
  // base <- imu = product of the fixed joint rotations from the base down to the IMU link.
  urdf::Rotation rotation;
  while (link && link->name != base_frame_) {
    const urdf::JointConstSharedPtr joint = link->parent_joint;
    if (!joint) {
      throw std::invalid_argument(
              std::string(kNodeName) + ": links_map.imu '" + imu_frame_ +
              "' is not below links_map.base '" + base_frame_ + "' in the URDF");
    }
    if (joint->type != urdf::Joint::FIXED) {
      throw std::invalid_argument(
              std::string(kNodeName) + ": joint '" + joint->name +
              "' between the base and the IMU is not fixed");
    }
    rotation = joint->parent_to_joint_origin_transform.rotation * rotation;
    link = link->getParent();
  }
  champ_body_pose::Quaternion base_from_imu;
  rotation.getQuaternion(base_from_imu.x, base_from_imu.y, base_from_imu.z, base_from_imu.w);
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  rotation.getRPY(roll, pitch, yaw);
  RCLCPP_INFO(
    get_logger(), "IMU mount from URDF %s -> %s: rpy %.4f %.4f %.4f rad",
    base_frame_.c_str(), imu_frame_.c_str(), roll, pitch, yaw);
  return base_from_imu;
}

void BodyPoseController::loadController(const champ_body_pose::Quaternion & base_from_imu)
{
  champ_body_pose::BodyOrientationConfig cfg;
  cfg.base_from_imu = base_from_imu;
  cfg.gains.kp = requireNumber(*this, "body_pose.kp", "Proportional gain (rad per rad).");
  cfg.gains.ki = requireNumber(*this, "body_pose.ki", "Integral gain (rad/s per rad).");
  cfg.gains.kd = requireNumber(*this, "body_pose.kd", "Body-rate damping (rad per rad/s).");
  cfg.gains.filter_cutoff_hz = requireNumber(
    *this, "body_pose.filter_cutoff_hz", "Low-pass cutoff of the IMU roll/pitch (Hz).");
  cfg.gains.deadband = requireNumber(*this, "body_pose.deadband", "Error deadband (rad).");
  cfg.gains.max_correction = requireNumber(
    *this, "body_pose.max_correction", "Bound of the closed-loop correction (rad).");
  cfg.gains.max_rate = requireNumber(
    *this, "body_pose.max_rate", "Slew limit of the correction (rad/s).");
  cfg.gains.max_error = requireNumber(
    *this, "body_pose.max_error", "Error above which the body is not standing (rad).");
  cfg.max_roll = requireNumber(*this, "body_pose.max_roll", "Bound of the commanded body roll (rad).");
  cfg.max_pitch = requireNumber(
    *this, "body_pose.max_pitch", "Bound of the commanded body pitch (rad).");
  try {
    controller_.configure(cfg);
  } catch (const std::invalid_argument & ex) {
    throw std::invalid_argument(std::string(kNodeName) + " body_pose.*: " + ex.what());
  }

  control_rate_ = requireNumber(*this, "body_pose.control_rate", "Outer loop rate (Hz).");
  if (!std::isfinite(control_rate_) || control_rate_ <= 0.0) {
    throw std::invalid_argument(std::string(kNodeName) + " body_pose.control_rate must be > 0");
  }
  imu_timeout_sec_ = requireNumber(
    *this, "body_pose.imu_timeout_sec", "IMU age after which the correction is released (s).");
  if (!std::isfinite(imu_timeout_sec_) || imu_timeout_sec_ <= 0.0) {
    throw std::invalid_argument(std::string(kNodeName) + " body_pose.imu_timeout_sec must be > 0");
  }
  desired_rate_ = requireNumber(
    *this, "body_pose.desired_rate",
    "Slew limit of the desired roll/pitch/yaw handed to CHAMP (rad/s).");
  if (!std::isfinite(desired_rate_) || desired_rate_ <= 0.0) {
    throw std::invalid_argument(std::string(kNodeName) + " body_pose.desired_rate must be > 0");
  }
  // Runtime switch: ros2 param set /body_pose_controller_node body_pose.enabled false
  if (!has_parameter("body_pose.enabled")) {
    declare_parameter("body_pose.enabled", true);
  }
}

void BodyPoseController::imuCallback(sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
  if (!finiteQuaternion(msg->orientation)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "IMU on %s has no orientation quaternion; the body pose loop needs an orientation-fused IMU",
      imu_topic_.c_str());
    return;
  }
  if (!msg->orientation_covariance.empty() &&
    !champ_body_pose::orientationCovariancePresent(msg->orientation_covariance[0]))
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "IMU on %s sets orientation_covariance[0] = -1 (no orientation); ignored",
      imu_topic_.c_str());
    return;
  }
  if (!imu_frame_warned_ && !msg->header.frame_id.empty() && msg->header.frame_id != imu_frame_) {
    imu_frame_warned_ = true;
    RCLCPP_WARN(
      get_logger(),
      "IMU frame_id '%s' differs from links_map.imu '%s'; using the mount rotation of '%s' "
      "anyway (set body_pose.imu_mount_rpy if the axes differ)",
      msg->header.frame_id.c_str(), imu_frame_.c_str(), imu_frame_.c_str());
  }
  champ_body_pose::Vector3 rate{
    msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z};
  if (!std::isfinite(rate.x) || !std::isfinite(rate.y) || !std::isfinite(rate.z)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "IMU on %s has a non-finite gyro; damping term uses 0 this sample",
      imu_topic_.c_str());
    rate = champ_body_pose::Vector3{};
  }
  const champ_body_pose::BodyMeasurement measured =
    controller_.measure(toQuaternion(msg->orientation), rate);
  if (!std::isfinite(measured.roll) || !std::isfinite(measured.pitch) ||
    !std::isfinite(measured.roll_rate) || !std::isfinite(measured.pitch_rate))
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "IMU on %s produced a non-finite base roll/pitch; ignored",
      imu_topic_.c_str());
    return;
  }
  measurement_ = measured;
  have_imu_ = true;
  last_imu_time_ = Clock::now();
}

void BodyPoseController::desiredPoseCallback(geometry_msgs::msg::Pose::ConstSharedPtr msg)
{
  if (!finiteQuaternion(msg->orientation)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "Ignoring a desired body pose with an invalid quaternion");
    return;
  }
  if (!std::isfinite(msg->position.x) || !std::isfinite(msg->position.y) ||
    !std::isfinite(msg->position.z))
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "Ignoring a desired body pose with a non-finite position");
    return;
  }
  const champ_body_pose::RollPitchYaw rpy =
    champ_body_pose::rpyFromQuaternion(toQuaternion(msg->orientation));
  if (!std::isfinite(rpy.roll) || !std::isfinite(rpy.pitch) || !std::isfinite(rpy.yaw)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "Ignoring a desired body pose whose RPY is not finite");
    return;
  }
  const auto & cfg = controller_.config();
  desired_target_ = rpy;
  desired_target_.roll = champ_body_pose::clampAbs(rpy.roll, cfg.max_roll);
  desired_target_.pitch = champ_body_pose::clampAbs(rpy.pitch, cfg.max_pitch);
  if (std::fabs(rpy.roll) > cfg.max_roll + 1e-6 || std::fabs(rpy.pitch) > cfg.max_pitch + 1e-6) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Desired body pose (roll %.3f, pitch %.3f rad) exceeds max_roll %.3f / max_pitch %.3f; "
      "clamped (this is the yaml limit, not the kinematic ceiling)",
      rpy.roll, rpy.pitch, cfg.max_roll, cfg.max_pitch);
  }
  desired_position_ = msg->position;
}

void BodyPoseController::controlLoop()
{
  const Clock::time_point now = Clock::now();
  // Real elapsed time, bounded so a stalled timer cannot integrate a huge step.
  const double nominal_dt = 1.0 / control_rate_;
  double dt = std::chrono::duration<double>(now - last_loop_time_).count();
  last_loop_time_ = now;
  dt = std::max(0.5 * nominal_dt, std::min(2.0 * nominal_dt, dt));

  // The desired pose reaches CHAMP ramped, never stepped (the stance feet
  // follow it directly).
  const double step = desired_rate_ * dt;
  desired_rpy_.roll = champ_body_pose::slewAngle(desired_rpy_.roll, desired_target_.roll, step);
  desired_rpy_.pitch = champ_body_pose::slewAngle(desired_rpy_.pitch, desired_target_.pitch, step);
  desired_rpy_.yaw = champ_body_pose::slewAngle(desired_rpy_.yaw, desired_target_.yaw, step);

  bool enabled = true;
  get_parameter("body_pose.enabled", enabled);

  champ_body_pose::BodyCommand command;
  double imu_age = 0.0;
  State state = State::kPassThrough;
  if (!enabled) {
    // Off: release the correction along max_rate, then pass the desired through.
    command = controller_.relax(desired_rpy_, dt);
  } else {
    imu_age = have_imu_ ?
      std::chrono::duration<double>(now - last_imu_time_).count() : imu_timeout_sec_ + 1.0;
    if (imu_age > imu_timeout_sec_) {
      command = controller_.relax(desired_rpy_, dt);
      state = State::kNoImu;
    } else {
      command = controller_.update(desired_rpy_, measurement_, dt);
      state = command.error_reset ? State::kErrorReset : State::kActive;
    }
  }

  reportState(state, imu_age);
  publishCommand(command);
  publishStatus(command, state == State::kActive || state == State::kErrorReset);
}

void BodyPoseController::reportState(State state, double imu_age)
{
  const bool changed = state != state_;
  state_ = state;
  switch (state) {
    case State::kPassThrough:
      if (changed) {
        RCLCPP_INFO(
          get_logger(),
          "body_pose.enabled is false: releasing the correction at max_rate, then passing the "
          "desired pose through");
      }
      break;
    case State::kNoImu:
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "No IMU on %s for %.2f s: releasing the roll/pitch correction",
        imu_topic_.c_str(), have_imu_ ? imu_age : 0.0);
      break;
    case State::kErrorReset:
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Body roll/pitch (%.3f, %.3f rad) is more than %.3f rad from the desired (%.3f, %.3f): "
        "not standing, correction released",
        measurement_.roll, measurement_.pitch, controller_.config().gains.max_error,
        desired_rpy_.roll, desired_rpy_.pitch);
      break;
    case State::kActive:
      if (changed) {
        RCLCPP_INFO(
          get_logger(), "IMU roll/pitch loop active (body roll %.3f, pitch %.3f rad)",
          measurement_.roll, measurement_.pitch);
      }
      break;
  }
}

void BodyPoseController::publishCommand(const champ_body_pose::BodyCommand & command)
{
  if (!std::isfinite(command.roll) || !std::isfinite(command.pitch) ||
    !std::isfinite(desired_rpy_.yaw) ||
    !std::isfinite(desired_position_.x) || !std::isfinite(desired_position_.y) ||
    !std::isfinite(desired_position_.z))
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Skipping a non-finite body pose command; CHAMP keeps the last pose");
    return;
  }
  geometry_msgs::msg::Pose pose;
  pose.position = desired_position_;
  pose.orientation = toMsg(
    champ_body_pose::quaternionFromRpy(command.roll, command.pitch, desired_rpy_.yaw));
  command_publisher_->publish(pose);
}

void BodyPoseController::publishStatus(
  const champ_body_pose::BodyCommand & command, bool measured_valid)
{
  const auto stamp = get_clock()->now();
  geometry_msgs::msg::Vector3Stamped correction;
  correction.header.stamp = stamp;
  correction.header.frame_id = base_frame_;
  correction.vector.x = command.roll_correction;
  correction.vector.y = command.pitch_correction;
  correction.vector.z = 0.0;
  correction_publisher_->publish(correction);

  if (measured_valid) {
    geometry_msgs::msg::Vector3Stamped measured;
    measured.header.stamp = stamp;
    measured.header.frame_id = base_frame_;
    measured.vector.x = controller_.rollAxis().filteredAngle();
    measured.vector.y = controller_.pitchAxis().filteredAngle();
    measured.vector.z = measurement_.yaw;
    measured_publisher_->publish(measured);
  }
}
