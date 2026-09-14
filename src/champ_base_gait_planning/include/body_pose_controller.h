// IMU-closed body roll/pitch for CHAMP, as a ROS 2 node.
//
//   desired body pose (geometry_msgs/Pose, e.g. /body_pose from the user)
//   IMU (sensor_msgs/Imu: real robot /imu/data from /lowstate, MuJoCo /imu/data)
//                 |
//                 v   ~50-100 Hz: e = desired - measured -> bounded PI(D) correction
//   corrected body pose (geometry_msgs/Pose) -> quadruped_controller_node body_pose
//
// The quadruped controller keeps walking with gait + IK + joint PD exactly as
// before; only the roll/pitch its body controller receives is closed on the
// IMU. Position (body height/shift) passes through unchanged; yaw passes
// through slew-limited like roll/pitch (a stepped desired pose is ramped at
// desired_rate so the legs never jump).
//
// Robot-agnostic: the IMU mounting comes from the URDF (fixed joints between
// links_map.base and links_map.imu), gains and angle limits from
// config/<robot>_body_pose.yaml. Nothing about a particular robot lives here.
#ifndef BODY_POSE_CONTROLLER_H
#define BODY_POSE_CONTROLLER_H

#include <chrono>
#include <string>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "body_orientation_controller.h"

class BodyPoseController : public rclcpp::Node
{
public:
  BodyPoseController();

private:
  using Clock = std::chrono::steady_clock;

  enum class State { kPassThrough, kNoImu, kActive, kErrorReset };

  champ_body_pose::Quaternion loadImuMount();
  void loadController(const champ_body_pose::Quaternion & base_from_imu);
  void imuCallback(sensor_msgs::msg::Imu::ConstSharedPtr msg);
  void desiredPoseCallback(geometry_msgs::msg::Pose::ConstSharedPtr msg);
  void controlLoop();
  void reportState(State state, double imu_age);
  void publishCommand(const champ_body_pose::BodyCommand & command);
  void publishStatus(const champ_body_pose::BodyCommand & command, bool measured_valid);

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr desired_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr command_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr measured_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr correction_publisher_;
  rclcpp::TimerBase::SharedPtr loop_timer_;

  champ_body_pose::BodyOrientationController controller_;
  champ_body_pose::BodyMeasurement measurement_;
  // Orientation last received on the desired topic, and the slew-limited
  // (desired_rate) version of it that the loop and CHAMP actually follow.
  champ_body_pose::RollPitchYaw desired_target_;
  champ_body_pose::RollPitchYaw desired_rpy_;
  geometry_msgs::msg::Point desired_position_;

  std::string base_frame_;
  std::string imu_frame_;
  std::string imu_topic_;
  double control_rate_{0.0};
  double imu_timeout_sec_{0.0};
  double desired_rate_{0.0};
  bool imu_frame_warned_{false};
  bool have_imu_{false};
  State state_{State::kNoImu};
  Clock::time_point last_imu_time_{};
  Clock::time_point last_loop_time_{};
};

#endif  // BODY_POSE_CONTROLLER_H
