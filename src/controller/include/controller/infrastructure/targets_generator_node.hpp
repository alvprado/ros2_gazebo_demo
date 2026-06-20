#ifndef CONTROLLER_INFRASTRUCTURE_TARGETS_GENERATOR_NODE_HPP
#define CONTROLLER_INFRASTRUCTURE_TARGETS_GENERATOR_NODE_HPP

#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

#include "controller/domain/targets_generator.hpp"

namespace controller
{
namespace infrastructure
{

/// @brief Targets Generator ROS2 node
class TargetsGeneratorNode : public rclcpp::Node
{
public:
  TargetsGeneratorNode();

private:
  void publishCallback();

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr target_long_velocity_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr target_curvature_pub_;
  rclcpp::TimerBase::SharedPtr target_velocity_pub_timer_;

  std::optional<domain::TargetsGenerator> long_velocity_targets_;
  std::optional<domain::TargetsGenerator> curvature_targets_;
};

}  // namespace infrastructure
}  // namespace controller

#endif  // CONTROLLER_INFRASTRUCTURE_TARGETS_GENERATOR_NODE_HPP
