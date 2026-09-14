#ifndef QUADRUPED_CONTROLLER_H
#define QUADRUPED_CONTROLLER_H

#include <rclcpp/rclcpp.hpp>

#include <champ_msgs/msg/contacts_stamped.hpp>

#include <champ/body_controller/body_controller.h>
#include <champ/kinematics/kinematics.h>
#include <champ/leg_controller/leg_controller.h>
#include <champ/quadruped_base/quadruped_base.h>
#include <champ/utils/urdf_loader.h>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <cmath>
#include <cstring>
#include <string>

class QuadrupedController : public rclcpp::Node
{
public:
  QuadrupedController();

private:
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr cmd_pose_subscription_;

  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_commands_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_publisher_;
  rclcpp::Publisher<champ_msgs::msg::ContactsStamped>::SharedPtr foot_contacts_publisher_;

  rclcpp::TimerBase::SharedPtr loop_timer_;
  rclcpp::Clock clock_;

  champ::Velocities req_vel_;
  champ::Velocities cmd_vel_target_;
  champ::Pose req_pose_;

  champ::GaitConfig gait_config_;
  double max_linear_acceleration_{0.0};
  double max_angular_acceleration_{0.0};
  double loop_dt_{0.005};

  champ::QuadrupedBase base_;
  champ::BodyController body_controller_;
  champ::LegController leg_controller_;
  champ::Kinematics kinematics_;

  std::vector<std::string> joint_names_;
  std::string knee_orientation_;

  bool publish_foot_contacts_{true};
  bool publish_joint_states_{true};
  bool publish_joint_control_{false};
  bool in_gazebo_{false};
  bool has_last_joints_{false};
  float last_joint_positions_[12]{};

  void controlLoop_();
  void slewReqVel_();
  void publishJoints_(float target_joints[12]);
  void publishFootContacts_();
  void cmdVelCallback_(const geometry_msgs::msg::Twist::SharedPtr msg);
  void cmdPoseCallback_(const geometry_msgs::msg::Pose::SharedPtr msg);
};

#endif
