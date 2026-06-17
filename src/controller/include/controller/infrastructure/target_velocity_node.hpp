#ifndef CONTROLLER_INFRASTRUCTURE_TARGET_VELOCITY_NODE_HPP
#define CONTROLLER_INFRASTRUCTURE_TARGET_VELOCITY_NODE_HPP

#include <optional>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

#include "controller/domain/velocity_profile.hpp"

namespace controller
{
namespace infrastructure
{

/// @brief Target Velocity ROS2 node
class TargetVelocityNode : public rclcpp::Node
{
public:
  TargetVelocityNode();

private:
  void publishCallback();

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr target_velocity_pub_;
  rclcpp::TimerBase::SharedPtr target_velocity_pub_timer_;

  std::optional<domain::VelocityProfile> velocity_profile_;
};

}  // namespace infrastructure
}  // namespace controller

#endif  // CONTROLLER_INFRASTRUCTURE_TARGET_VELOCITY_NODE_HPP
