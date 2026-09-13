#include <body_pose_controller.h>

#include <cstdio>
#include <exception>
#include <memory>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int status = 0;
  try {
    rclcpp::spin(std::make_shared<BodyPoseController>());
  } catch (const std::exception & ex) {
    if (rclcpp::ok()) {
      std::fprintf(stderr, "body_pose_controller_node: %s\n", ex.what());
      status = 1;
    }
  }
  rclcpp::shutdown();
  return status;
}
