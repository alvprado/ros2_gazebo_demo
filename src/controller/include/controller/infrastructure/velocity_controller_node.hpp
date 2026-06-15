#ifndef CONTROLLER_INFRASTRUCTURE_VELOCITY_CONTROLLER_NODE_HPP
#define CONTROLLER_INFRASTRUCTURE_VELOCITY_CONTROLLER_NODE_HPP

#include <optional>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include "controller/domain/velocity_control.hpp"
#include "controller/domain/pid_controller.hpp"

namespace controller
{
namespace infrastructure
{

/// @brief Velocity Controller ROS2 Node
class VelocityControllerNode : public rclcpp::Node
{
public:
  VelocityControllerNode();

private:
  void targetVelocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void controlCallback();

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr target_velocity_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_command_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  std::optional<domain::VelocityControl<domain::PidController>> velocity_control_;

  geometry_msgs::msg::Twist target_velocity_;
  nav_msgs::msg::Odometry odometry_;

  double dt_{0.0};
};

}  // namespace infrastructure
}  // namespace controller

#endif  // CONTROLLER_INFRASTRUCTURE_VELOCITY_CONTROLLER_NODE_HPP
