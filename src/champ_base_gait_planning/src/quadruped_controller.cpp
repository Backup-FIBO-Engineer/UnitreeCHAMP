#include <quadruped_controller.h>

namespace
{
champ::PhaseGenerator::Time rosTimeToChampTime(const rclcpp::Time & time)
{
  return time.nanoseconds() / 1000ul;
}

double slewToward(double current, double target, double max_delta)
{
  if (!(max_delta > 0.0) || !std::isfinite(max_delta) || !std::isfinite(target)) {
    return std::isfinite(target) ? target : current;
  }
  const double delta = target - current;
  if (std::fabs(delta) <= max_delta) {
    return target;
  }
  return current + std::copysign(max_delta, delta);
}
}  // namespace

QuadrupedController::QuadrupedController()
: Node(
    "quadruped_controller_node",
    rclcpp::NodeOptions()
      .allow_undeclared_parameters(true)
      .automatically_declare_parameters_from_overrides(true)),
  clock_(*get_clock()),
  body_controller_(base_),
  leg_controller_(base_, rosTimeToChampTime(clock_.now())),
  kinematics_(base_)
{
  std::string joint_control_topic = "joint_group_position_controller/command";
  std::string urdf;

  double loop_rate = 200.0;

  get_parameter("gait.max_linear_velocity_x", gait_config_.max_linear_velocity_x);
  get_parameter("gait.max_linear_velocity_y", gait_config_.max_linear_velocity_y);
  get_parameter("gait.max_angular_velocity_z", gait_config_.max_angular_velocity_z);
  get_parameter("gait.com_x_translation", gait_config_.com_x_translation);
  get_parameter("gait.swing_height", gait_config_.swing_height);
  get_parameter("gait.stance_depth", gait_config_.stance_depth);
  get_parameter("gait.stance_duration", gait_config_.stance_duration);
  get_parameter("gait.nominal_height", gait_config_.nominal_height);
  get_parameter("gait.knee_orientation", knee_orientation_);
  get_parameter_or("gait.max_linear_acceleration", max_linear_acceleration_, 0.0);
  get_parameter_or("gait.max_angular_acceleration", max_angular_acceleration_, 0.0);
  get_parameter("publish_foot_contacts", publish_foot_contacts_);
  get_parameter("publish_joint_states", publish_joint_states_);
  get_parameter("publish_joint_control", publish_joint_control_);
  get_parameter("gazebo", in_gazebo_);
  get_parameter("joint_controller_topic", joint_control_topic);
  get_parameter("loop_rate", loop_rate);
  get_parameter("urdf", urdf);
  loop_dt_ = 1.0 / std::max(1.0, loop_rate);

  auto cmd_vel_cb = [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
      cmdVelCallback_(msg);
    };

  cmd_vel_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 10, cmd_vel_cb);

  cmd_pose_subscription_ = create_subscription<geometry_msgs::msg::Pose>(
    "body_pose", 1,
    std::bind(&QuadrupedController::cmdPoseCallback_, this, std::placeholders::_1));

  if (publish_joint_control_) {
    joint_commands_publisher_ =
      create_publisher<trajectory_msgs::msg::JointTrajectory>(joint_control_topic, 10);
  }

  if (publish_joint_states_ && !in_gazebo_) {
    joint_states_publisher_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
  }

  if (publish_foot_contacts_ && !in_gazebo_) {
    foot_contacts_publisher_ =
      create_publisher<champ_msgs::msg::ContactsStamped>("foot_contacts", 10);
  }

  gait_config_.knee_orientation = knee_orientation_.c_str();

  base_.setGaitConfig(gait_config_);
  champ::URDF::loadFromString(base_, get_node_parameters_interface(), urdf);
  joint_names_ = champ::URDF::getJointNames(get_node_parameters_interface());

  const auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / loop_rate));
  loop_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(period),
    std::bind(&QuadrupedController::controlLoop_, this));

  req_pose_.position.z = gait_config_.nominal_height;

  if (max_linear_acceleration_ > 0.0 || max_angular_acceleration_ > 0.0) {
    RCLCPP_INFO(
      get_logger(),
      "Quadruped controller ready (%zu joints); cmd_vel slew %.2f m/s^2, %.2f rad/s^2",
      joint_names_.size(), max_linear_acceleration_, max_angular_acceleration_);
  } else {
    RCLCPP_INFO(get_logger(), "Quadruped controller ready (%zu joints)", joint_names_.size());
  }
}

void QuadrupedController::controlLoop_()
{
  slewReqVel_();
  float target_joint_positions[12] = {};
  if (has_last_joints_) {
    std::memcpy(target_joint_positions, last_joint_positions_, sizeof(last_joint_positions_));
  }
  geometry::Transformation target_foot_positions[4];

  body_controller_.poseCommand(target_foot_positions, req_pose_);
  leg_controller_.velocityCommand(
    target_foot_positions, req_vel_, rosTimeToChampTime(clock_.now()));
  touchdown_tick_ = false;
  for (size_t i = 0; i < 4; ++i) {
    const bool stance = base_.legs[i]->gait_phase();
    touchdown_tick_ = touchdown_tick_ || (stance && !leg_in_stance_[i]);
    leg_in_stance_[i] = stance;
  }
  kinematics_.inverse(target_joint_positions, target_foot_positions);

  bool finite = true;
  for (int i = 0; i < 12; ++i) {
    if (!std::isfinite(target_joint_positions[i])) {
      finite = false;
      break;
    }
  }
  if (finite) {
    std::memcpy(last_joint_positions_, target_joint_positions, sizeof(last_joint_positions_));
    has_last_joints_ = true;
  } else if (has_last_joints_) {
    std::memcpy(target_joint_positions, last_joint_positions_, sizeof(last_joint_positions_));
  }

  publishFootContacts_();
  publishJoints_(target_joint_positions);
}

void QuadrupedController::slewReqVel_()
{
  const double dv = max_linear_acceleration_ * loop_dt_;
  const double dw = max_angular_acceleration_ * loop_dt_;
  const float vx = static_cast<float>(
    slewToward(req_vel_.linear.x, cmd_vel_target_.linear.x, dv));
  const float vy = static_cast<float>(
    slewToward(req_vel_.linear.y, cmd_vel_target_.linear.y, dv));
  const float wz = static_cast<float>(
    slewToward(req_vel_.angular.z, cmd_vel_target_.angular.z, dw));

  // CHAMP resets the gait the instant every velocity is exactly zero and
  // plants all four feet at the stance position in that same tick. A foot
  // still in its swing would be slammed down from up to swing_height. Hold the
  // last (already ramped-down, so tiny) velocity until a foot has just touched
  // down: at that tick the other pair has barely left the ground, so the reset
  // moves nothing but a few millimetres. The same applies when a reversal
  // passes through zero.
  const bool moving =
    req_vel_.linear.x != 0.0f || req_vel_.linear.y != 0.0f || req_vel_.angular.z != 0.0f;
  const bool stopping = vx == 0.0f && vy == 0.0f && wz == 0.0f;
  const bool feet_down = touchdown_tick_ ||
    (leg_in_stance_[0] && leg_in_stance_[1] && leg_in_stance_[2] && leg_in_stance_[3]);
  if (moving && stopping && !feet_down) {
    return;
  }
  req_vel_.linear.x = vx;
  req_vel_.linear.y = vy;
  req_vel_.angular.z = wz;
}

void QuadrupedController::cmdVelCallback_(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  const double max_x = gait_config_.max_linear_velocity_x;
  const double max_y = gait_config_.max_linear_velocity_y;
  const double max_z = gait_config_.max_angular_velocity_z;
  if (std::fabs(msg->linear.x) > max_x + 1e-6 ||
    std::fabs(msg->linear.y) > max_y + 1e-6 ||
    std::fabs(msg->angular.z) > max_z + 1e-6)
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "cmd_vel (%.2f, %.2f, %.2f) exceeds gait limits ±(%.2f, %.2f, %.2f). "
      "CHAMP clamps it. Lower the teleop speed (z/x) to gait.max_linear_velocity_* of this robot.",
      msg->linear.x, msg->linear.y, msg->angular.z, max_x, max_y, max_z);
  }
  cmd_vel_target_.linear.x = static_cast<float>(
    std::max(-max_x, std::min(max_x, msg->linear.x)));
  cmd_vel_target_.linear.y = static_cast<float>(
    std::max(-max_y, std::min(max_y, msg->linear.y)));
  cmd_vel_target_.angular.z = static_cast<float>(
    std::max(-max_z, std::min(max_z, msg->angular.z)));
}

void QuadrupedController::cmdPoseCallback_(const geometry_msgs::msg::Pose::SharedPtr msg)
{
  tf2::Quaternion quat(
    msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);

  tf2::Matrix3x3 m(quat);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  req_pose_.orientation.roll = roll;
  req_pose_.orientation.pitch = pitch;
  req_pose_.orientation.yaw = yaw;
  req_pose_.position.x = msg->position.x;
  req_pose_.position.y = msg->position.y;
  req_pose_.position.z = msg->position.z + gait_config_.nominal_height;
}

void QuadrupedController::publishJoints_(float target_joints[12])
{
  if (publish_joint_control_) {
    trajectory_msgs::msg::JointTrajectory joints_cmd_msg;
    joints_cmd_msg.header.stamp = clock_.now();
    joints_cmd_msg.joint_names = joint_names_;

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions.resize(12);
    point.time_from_start = rclcpp::Duration::from_seconds(1.0 / 60.0);
    for (size_t i = 0; i < 12; i++) {
      point.positions[i] = target_joints[i];
    }

    joints_cmd_msg.points.push_back(point);
    joint_commands_publisher_->publish(joints_cmd_msg);
  }

  if (publish_joint_states_ && !in_gazebo_) {
    sensor_msgs::msg::JointState joints_msg;
    joints_msg.header.stamp = clock_.now();
    joints_msg.name = joint_names_;
    joints_msg.position.resize(joint_names_.size());

    for (size_t i = 0; i < joint_names_.size(); ++i) {
      joints_msg.position[i] = target_joints[i];
    }

    joint_states_publisher_->publish(joints_msg);
  }
}

void QuadrupedController::publishFootContacts_()
{
  if (!publish_foot_contacts_ || in_gazebo_) {
    return;
  }

  champ_msgs::msg::ContactsStamped contacts_msg;
  contacts_msg.header.stamp = clock_.now();
  contacts_msg.contacts.resize(4);

  for (size_t i = 0; i < 4; i++) {
    contacts_msg.contacts[i] = base_.legs[i]->gait_phase();
  }

  foot_contacts_publisher_->publish(contacts_msg);
}
