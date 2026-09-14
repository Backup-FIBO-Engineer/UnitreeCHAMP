#include <state_estimation.h>

#include <algorithm>

namespace
{
champ::Odometry::Time rosTimeToChampTime(const rclcpp::Time & time)
{
  return time.nanoseconds() / 1000ul;
}
}  // namespace

StateEstimation::StateEstimation()
: Node(
    "state_estimation_node",
    rclcpp::NodeOptions()
      .allow_undeclared_parameters(true)
      .automatically_declare_parameters_from_overrides(true)),
  clock_(*get_clock()),
  odometry_(base_, rosTimeToChampTime(clock_.now()))
{
  last_vel_time_ = clock_.now();

  joint_states_subscriber_.subscribe(this, "joint_states");
  foot_contacts_subscriber_.subscribe(this, "foot_contacts");

  sync_ = std::make_unique<Sync>(
    SyncPolicy(10), joint_states_subscriber_, foot_contacts_subscriber_);

  sync_->registerCallback(
    std::bind(
      &StateEstimation::synchronized_callback_, this, std::placeholders::_1,
      std::placeholders::_2));

  footprint_to_odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>("odom/raw", 1);
  base_to_footprint_publisher_ =
    create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "base_to_footprint_pose", 1);
  foot_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>("foot", 1);
  estimated_cmd_vel_publisher_ =
    create_publisher<geometry_msgs::msg::Twist>("cmd_vel/estimated", 10);

  std::string urdf;

  get_parameter("links_map.base", base_name_);
  get_parameter("gait.odom_scaler", gait_config_.odom_scaler);
  get_parameter("orientation_from_imu", orientation_from_imu_);
  get_parameter("gait.max_linear_velocity_x", gait_config_.max_linear_velocity_x);
  get_parameter("gait.max_linear_velocity_y", gait_config_.max_linear_velocity_y);
  get_parameter("gait.max_angular_velocity_z", gait_config_.max_angular_velocity_z);
  get_parameter("gait.com_x_translation", gait_config_.com_x_translation);
  get_parameter("gait.swing_height", gait_config_.swing_height);
  get_parameter("gait.stance_depth", gait_config_.stance_depth);
  get_parameter("gait.stance_duration", gait_config_.stance_duration);
  get_parameter_or("gait.swing_duration", gait_config_.swing_duration, 0.25f);
  get_parameter("gait.nominal_height", gait_config_.nominal_height);
  get_parameter("gait.knee_orientation", knee_orientation_);
  get_parameter("urdf", urdf);

  gait_config_.knee_orientation = knee_orientation_.c_str();

  if (orientation_from_imu_) {
    imu_subscriber_ = create_subscription<sensor_msgs::msg::Imu>(
      "imu/data", 1, std::bind(&StateEstimation::imu_callback_, this, std::placeholders::_1));
  }

  base_.setGaitConfig(gait_config_);
  champ::URDF::loadFromString(base_, get_node_parameters_interface(), urdf);
  joint_names_ = champ::URDF::getJointNames(get_node_parameters_interface());

  odom_frame_ = "odom";
  base_footprint_frame_ = "base_footprint";
  base_link_frame_ = base_name_;

  const auto period = std::chrono::milliseconds(20);
  odom_data_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(period),
    std::bind(&StateEstimation::publishFootprintToOdom_, this));

  base_pose_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(period),
    std::bind(&StateEstimation::publishBaseToFootprint_, this));

  RCLCPP_INFO(get_logger(), "State estimation ready (base: %s)", base_link_frame_.c_str());
}

void StateEstimation::synchronized_callback_(
  const std::shared_ptr<sensor_msgs::msg::JointState const> & joints_msg,
  const std::shared_ptr<champ_msgs::msg::ContactsStamped const> & contacts_msg)
{
  float current_joint_positions[12] = {};

  for (size_t i = 0; i < joints_msg->name.size(); i++) {
    const auto itr = std::find(joint_names_.begin(), joint_names_.end(), joints_msg->name[i]);
    if (itr == joint_names_.end()) {
      continue;
    }
    const int index = static_cast<int>(std::distance(joint_names_.begin(), itr));
    current_joint_positions[index] = static_cast<float>(joints_msg->position[i]);
  }

  base_.updateJointPositions(current_joint_positions);

  for (size_t i = 0; i < 4; i++) {
    base_.legs[i]->in_contact(contacts_msg->contacts[i]);
  }
}

void StateEstimation::imu_callback_(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  last_imu_ = msg;
}

void StateEstimation::publishFootprintToOdom_()
{
  odometry_.getVelocities(current_velocities_, rosTimeToChampTime(clock_.now()));

  const rclcpp::Time current_time = clock_.now();
  const double vel_dt = (current_time - last_vel_time_).seconds();
  last_vel_time_ = current_time;

  const double delta_heading = current_velocities_.angular.z * vel_dt;
  const double delta_x =
    (current_velocities_.linear.x * cos(heading_) -
    current_velocities_.linear.y * sin(heading_)) * vel_dt;
  const double delta_y =
    (current_velocities_.linear.x * sin(heading_) +
    current_velocities_.linear.y * cos(heading_)) * vel_dt;

  x_pos_ += static_cast<float>(delta_x);
  y_pos_ += static_cast<float>(delta_y);
  heading_ += static_cast<float>(delta_heading);

  tf2::Quaternion odom_quat;
  odom_quat.setRPY(0, 0, heading_);

  nav_msgs::msg::Odometry odom;
  odom.header.stamp = current_time;
  odom.header.frame_id = odom_frame_;
  odom.child_frame_id = base_footprint_frame_;
  odom.pose.pose.position.x = x_pos_;
  odom.pose.pose.position.y = y_pos_;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation.x = odom_quat.x();
  odom.pose.pose.orientation.y = odom_quat.y();
  odom.pose.pose.orientation.z = odom_quat.z();
  odom.pose.pose.orientation.w = odom_quat.w();
  odom.pose.covariance[0] = 0.25;
  odom.pose.covariance[7] = 0.25;
  odom.pose.covariance[35] = 0.017;

  odom.twist.twist.linear.x = current_velocities_.linear.x;
  odom.twist.twist.linear.y = current_velocities_.linear.y;
  odom.twist.twist.angular.z = current_velocities_.angular.z;
  odom.twist.covariance[0] = 0.3;
  odom.twist.covariance[7] = 0.3;
  odom.twist.covariance[35] = 0.017;

  footprint_to_odom_publisher_->publish(odom);

  geometry_msgs::msg::Twist estimated;
  estimated.linear.x = current_velocities_.linear.x;
  estimated.linear.y = current_velocities_.linear.y;
  estimated.angular.z = current_velocities_.angular.z;
  estimated_cmd_vel_publisher_->publish(estimated);
}

visualization_msgs::msg::Marker StateEstimation::createMarker_(
  geometry::Transformation foot_pos, int id, const std::string & frame_id)
{
  visualization_msgs::msg::Marker foot_marker;
  foot_marker.header.frame_id = frame_id;
  foot_marker.type = visualization_msgs::msg::Marker::SPHERE;
  foot_marker.action = visualization_msgs::msg::Marker::ADD;
  foot_marker.id = id;
  foot_marker.pose.position.x = foot_pos.X();
  foot_marker.pose.position.y = foot_pos.Y();
  foot_marker.pose.position.z = foot_pos.Z();
  foot_marker.pose.orientation.w = 1.0;
  foot_marker.scale.x = 0.05;
  foot_marker.scale.y = 0.05;
  foot_marker.scale.z = 0.05;
  foot_marker.color.r = 0.78f;
  foot_marker.color.g = 0.082f;
  foot_marker.color.b = 0.521f;
  foot_marker.color.a = 0.5f;
  return foot_marker;
}

void StateEstimation::publishBaseToFootprint_()
{
  base_.getFootPositions(current_foot_positions_);

  visualization_msgs::msg::MarkerArray marker_array;
  float robot_height = 0.0f;
  float all_height = 0.0f;
  int foot_in_contact = 0;
  geometry::Transformation touching_feet[4];
  bool no_contact = false;

  for (size_t i = 0; i < 4; i++) {
    marker_array.markers.push_back(
      createMarker_(current_foot_positions_[i], static_cast<int>(i), base_link_frame_));
    if (base_.legs[i]->in_contact()) {
      robot_height += current_foot_positions_[i].Z();
      touching_feet[foot_in_contact] = current_foot_positions_[i];
      foot_in_contact++;
    }
    all_height += current_foot_positions_[i].Z();
  }

  if (foot_in_contact == 0) {
    no_contact = true;
    robot_height = all_height;
    foot_in_contact = 4;
    for (size_t i = 0; i < 4; ++i) {
      touching_feet[i] = current_foot_positions_[i];
    }
  }

  if (foot_publisher_->get_subscription_count() > 0) {
    foot_publisher_->publish(marker_array);
  }

  tf2::Vector3 x_axis(1, 0, 0);
  tf2::Vector3 y_axis(0, 1, 0);
  tf2::Vector3 z_axis(0, 0, 1);

  tf2::Matrix3x3 imu_rotation;
  if (orientation_from_imu_ && last_imu_ != nullptr) {
    tf2::Quaternion imu_orientation(
      last_imu_->orientation.x, last_imu_->orientation.y,
      last_imu_->orientation.z, last_imu_->orientation.w);
    imu_rotation.setRotation(imu_orientation);
  } else {
    imu_rotation.setIdentity();
  }

  if (foot_in_contact >= 3 && !no_contact) {
    x_axis = tf2::Vector3(
      touching_feet[0].X() - touching_feet[2].X(),
      touching_feet[0].Y() - touching_feet[2].Y(),
      touching_feet[0].Z() - touching_feet[2].Z());
    x_axis.normalize();

    y_axis = tf2::Vector3(
      touching_feet[1].X() - touching_feet[2].X(),
      touching_feet[1].Y() - touching_feet[2].Y(),
      touching_feet[1].Z() - touching_feet[2].Z());
    y_axis.normalize();

    z_axis = x_axis.cross(y_axis);
    z_axis.normalize();
    if (z_axis.dot(tf2::Vector3(0, 0, 1)) < 0) {
      z_axis = -z_axis;
    }

    y_axis = (tf2::Vector3(0, 1, 0) - (tf2::Vector3(0, 1, 0).dot(z_axis) * z_axis)).normalized();
    x_axis = y_axis.cross(z_axis);
  } else if (foot_in_contact == 2) {
    if ((base_.legs[0]->in_contact() && base_.legs[2]->in_contact()) ||
      (base_.legs[1]->in_contact() && base_.legs[3]->in_contact()))
    {
      x_axis = tf2::Vector3(
        touching_feet[0].X() - touching_feet[1].X(),
        touching_feet[0].Y() - touching_feet[1].Y(),
        touching_feet[0].Z() - touching_feet[1].Z());
      x_axis.normalize();
      z_axis = imu_rotation.inverse() * z_axis;
      y_axis = z_axis.cross(x_axis);
      x_axis = y_axis.cross(z_axis);
    } else if ((base_.legs[0]->in_contact() && base_.legs[1]->in_contact()) ||
      (base_.legs[2]->in_contact() && base_.legs[3]->in_contact()))
    {
      y_axis = tf2::Vector3(
        touching_feet[0].X() - touching_feet[1].X(),
        touching_feet[0].Y() - touching_feet[1].Y(),
        touching_feet[0].Z() - touching_feet[1].Z());
      y_axis.normalize();
      z_axis = imu_rotation.inverse() * z_axis;
      x_axis = y_axis.cross(z_axis);
      y_axis = z_axis.cross(x_axis);
    } else {
      tf2::Vector3 axis1(
        touching_feet[0].X() - touching_feet[1].X(),
        touching_feet[0].Y() - touching_feet[1].Y(),
        touching_feet[0].Z() - touching_feet[1].Z());
      axis1.normalize();
      z_axis = imu_rotation.inverse() * z_axis;
      auto axis2 = z_axis.cross(axis1);
      z_axis = axis1.cross(axis2);
      x_axis = (x_axis - (x_axis.dot(z_axis) * z_axis)).normalized();
      y_axis = z_axis.cross(x_axis);
    }
  } else if (foot_in_contact == 1 || no_contact) {
    z_axis = imu_rotation.inverse() * z_axis;
    x_axis = (x_axis - (x_axis.dot(z_axis) * z_axis)).normalized();
    y_axis = z_axis.cross(x_axis);
  }

  tf2::Matrix3x3 rotation_matrix(
    x_axis.x(), y_axis.x(), z_axis.x(),
    x_axis.y(), y_axis.y(), z_axis.y(),
    x_axis.z(), y_axis.z(), z_axis.z());

  tf2::Quaternion quaternion;
  rotation_matrix.getRotation(quaternion);
  quaternion.normalize();

  geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
  pose_msg.header.frame_id = base_footprint_frame_;
  pose_msg.header.stamp = clock_.now();
  pose_msg.pose.pose.position.z = -(robot_height / static_cast<float>(foot_in_contact));
  pose_msg.pose.pose.orientation.x = quaternion.x();
  pose_msg.pose.pose.orientation.y = quaternion.y();
  pose_msg.pose.pose.orientation.z = quaternion.z();
  pose_msg.pose.pose.orientation.w = -quaternion.w();

  // covariance values from upstream chvmp/champ - required if this pose is
  // ever fused (e.g. robot_localization); all-zero covariance would make an
  // EKF treat it as a perfect measurement
  pose_msg.pose.covariance[0] = 0.001;
  pose_msg.pose.covariance[7] = 0.001;
  pose_msg.pose.covariance[14] = 0.001;
  pose_msg.pose.covariance[21] = 0.0001;
  pose_msg.pose.covariance[28] = 0.0001;
  pose_msg.pose.covariance[35] = 0.017;

  base_to_footprint_publisher_->publish(pose_msg);
}
