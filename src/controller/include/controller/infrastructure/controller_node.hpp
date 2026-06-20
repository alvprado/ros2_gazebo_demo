#ifndef CONTROLLER_INFRASTRUCTURE_CONTROLLER_NODE_HPP
#define CONTROLLER_INFRASTRUCTURE_CONTROLLER_NODE_HPP

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

#include "controller/domain/curvature_controller.hpp"
#include "controller/domain/low_pass_filter.hpp"
#include "controller/domain/pid_controller.hpp"

namespace controller
{
namespace infrastructure
{

/// @brief Controller ROS2 Node
class ControllerNode : public rclcpp::Node
{
public:
  ControllerNode();

private:
  using PidControllerType = domain::PidController<domain::LowPassFilter, domain::LowPassFilter>;
  using CurvCtrlType = domain::CurvatureController<PidControllerType>;

  void targetLongVelocityCallback(const std_msgs::msg::Float64::SharedPtr msg);
  void targetCurvatureCallback(const std_msgs::msg::Float64::SharedPtr msg);
  void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void controlCallback();

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr target_long_velocity_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr target_curvature_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_command_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  std::optional<PidControllerType> long_velocity_controller_;
  std::optional<CurvCtrlType> curvature_controller_;

  double target_longitudinal_velocity_mps_{0.0};
  double target_curvature_per_m_{0.0};
  nav_msgs::msg::Odometry odometry_;

  double dt_s_{0.0};
};

}  // namespace infrastructure
}  // namespace controller

#endif  // CONTROLLER_INFRASTRUCTURE_CONTROLLER_NODE_HPP
