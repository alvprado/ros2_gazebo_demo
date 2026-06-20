#include <rclcpp/rclcpp.hpp>

#include "controller/infrastructure/targets_generator_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<controller::infrastructure::TargetsGeneratorNode>());
  rclcpp::shutdown();
  return 0;
}
