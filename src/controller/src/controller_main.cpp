#include <rclcpp/rclcpp.hpp>

#include "controller/infrastructure/controller_node.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<controller::infrastructure::ControllerNode>());
  rclcpp::shutdown();
  return 0;
}
