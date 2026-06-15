#include <rclcpp/rclcpp.hpp>

#include "simulation/infrastructure/simulator_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<simulation::infrastructure::SimulatorNode>());
  rclcpp::shutdown();
  return 0;
}
