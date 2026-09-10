#ifndef STATE_ESTIMATION_H
#define STATE_ESTIMATION_H

#include <rclcpp/rclcpp.hpp>

#include <champ_msgs/msg/contacts_stamped.hpp>

#include <champ/odometry/odometry.h>
#include <champ/quadruped_base/quadruped_base.h>
#include <champ/utils/urdf_loader.h>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

#include <string>

class StateEstimation : public rclcpp::Node
{
public:
  StateEstimation();

private:
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::JointState, champ_msgs::msg::ContactsStamped>;
  using Sync = message_filters::Synchronizer<SyncPolicy>;

  std::unique_ptr<Sync> sync_;

  message_filters::Subscriber<sensor_msgs::msg::JointState> joint_states_subscriber_;
  message_filters::Subscriber<champ_msgs::msg::ContactsStamped> foot_contacts_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr footprint_to_odom_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr base_to_footprint_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr foot_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr estimated_cmd_vel_publisher_;

  rclcpp::TimerBase::SharedPtr odom_data_timer_;
  rclcpp::TimerBase::SharedPtr base_pose_timer_;

  champ::Velocities current_velocities_;
  geometry::Transformation current_foot_positions_[4];

  float x_pos_{0.0f};
  float y_pos_{0.0f};
  float heading_{0.0f};

  rclcpp::Time last_vel_time_;
  rclcpp::Clock clock_;

  sensor_msgs::msg::Imu::SharedPtr last_imu_;

  champ::GaitConfig gait_config_;
  champ::QuadrupedBase base_;
  champ::Odometry odometry_;

  std::vector<std::string> joint_names_;
  std::string knee_orientation_;
  std::string base_name_;
  std::string odom_frame_;
  std::string base_footprint_frame_;
  std::string base_link_frame_;
  bool orientation_from_imu_{false};

  void publishFootprintToOdom_();
  void publishBaseToFootprint_();
  void synchronized_callback_(
    const std::shared_ptr<sensor_msgs::msg::JointState const> & joints_msg,
    const std::shared_ptr<champ_msgs::msg::ContactsStamped const> & contacts_msg);
  void imu_callback_(const sensor_msgs::msg::Imu::SharedPtr msg);

  visualization_msgs::msg::Marker createMarker_(
    geometry::Transformation foot_pos, int id, const std::string & frame_id);
};

#endif
