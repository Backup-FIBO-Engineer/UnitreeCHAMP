#include <quadruped_controller.h>

namespace
{
champ::PhaseGenerator::Time rosTimeToChampTime(const rclcpp::Time & time)
{
  return time.nanoseconds() / 1000ul;
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
  get_parameter("publish_foot_contacts", publish_foot_contacts_);
  get_parameter("publish_joint_states", publish_joint_states_);
  get_parameter("publish_joint_control", publish_joint_control_);
  get_parameter("gazebo", in_gazebo_);
  get_parameter("joint_controller_topic", joint_control_topic);
  get_parameter("loop_rate", loop_rate);
  get_parameter("urdf", urdf);

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

  RCLCPP_INFO(get_logger(), "Quadruped controller ready (%zu joints)", joint_names_.size());
}

void QuadrupedController::controlLoop_()
{
  float target_joint_positions[12] = {};
  if (has_last_joints_) {
    std::memcpy(target_joint_positions, last_joint_positions_, sizeof(last_joint_positions_));
  }
  geometry::Transformation target_foot_positions[4];

  body_controller_.poseCommand(target_foot_positions, req_pose_);
  leg_controller_.velocityCommand(
    target_foot_positions, req_vel_, rosTimeToChampTime(clock_.now()));
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
  req_vel_.linear.x = msg->linear.x;
  req_vel_.linear.y = msg->linear.y;
  req_vel_.angular.z = msg->angular.z;
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
