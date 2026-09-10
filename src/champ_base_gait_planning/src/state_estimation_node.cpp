#include <state_estimation.h>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StateEstimation>());
  rclcpp::shutdown();
  return 0;
}
