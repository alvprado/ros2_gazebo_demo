#include <rclcpp/rclcpp.hpp>

#include "controller/infrastructure/target_velocity_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<controller::infrastructure::TargetVelocityNode>());
  rclcpp::shutdown();
  return 0;
}
