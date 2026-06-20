#include "controller/infrastructure/velocity_controller_node.hpp"

#include <chrono>
#include <stdexcept>

namespace controller
{
namespace infrastructure
{

VelocityControllerNode::VelocityControllerNode() : rclcpp::Node("velocity_controller")
{
  // Longitudinal (linear.x) parameters
  declare_parameter<double>("long.kp", 1.0);
  declare_parameter<double>("long.ki", 0.0);
  declare_parameter<double>("long.kd", 0.0);
  declare_parameter<double>("long.kff", 0.0);
  declare_parameter<double>("long.setpoint_filter_cutoff_frequency_hz", 1.0);
  declare_parameter<double>("long.derivative_filter_cutoff_frequency_hz", 1.0);
  declare_parameter<double>("long.min_velocity_mps", -100.0);
  declare_parameter<double>("long.max_velocity_mps", 100.0);

  domain::PidConfig long_config;
  long_config.kp = get_parameter("long.kp").as_double();
  long_config.ki = get_parameter("long.ki").as_double();
  long_config.kd = get_parameter("long.kd").as_double();
  long_config.kff = get_parameter("long.kff").as_double();
  long_config.min_output = get_parameter("long.min_velocity_mps").as_double();
  long_config.max_output = get_parameter("long.max_velocity_mps").as_double();

  const domain::LowPassFilter long_setpoint_filter(
    get_parameter("long.setpoint_filter_cutoff_frequency_hz").as_double());
  const domain::LowPassFilter long_derivative_filter(
    get_parameter("long.derivative_filter_cutoff_frequency_hz").as_double());

  long_velocity_controller_.emplace(long_config, long_setpoint_filter, long_derivative_filter);

  // Angular (angular.z) parameters
  declare_parameter<double>("angular.kp", 1.0);
  declare_parameter<double>("angular.ki", 0.0);
  declare_parameter<double>("angular.kd", 0.0);
  declare_parameter<double>("angular.kff", 0.0);
  declare_parameter<double>("angular.setpoint_filter_cutoff_frequency_hz", 1.0);
  declare_parameter<double>("angular.derivative_filter_cutoff_frequency_hz", 1.0);
  declare_parameter<double>("angular.min_velocity_radps", -100.0);
  declare_parameter<double>("angular.max_velocity_radps", 100.0);

  domain::PidConfig angular_config;
  angular_config.kp = get_parameter("angular.kp").as_double();
  angular_config.ki = get_parameter("angular.ki").as_double();
  angular_config.kd = get_parameter("angular.kd").as_double();
  angular_config.kff = get_parameter("angular.kff").as_double();
  angular_config.min_output = get_parameter("angular.min_velocity_radps").as_double();
  angular_config.max_output = get_parameter("angular.max_velocity_radps").as_double();

  const domain::LowPassFilter angular_setpoint_filter(
    get_parameter("angular.setpoint_filter_cutoff_frequency_hz").as_double());
  const domain::LowPassFilter angular_derivative_filter(
    get_parameter("angular.derivative_filter_cutoff_frequency_hz").as_double());

  angular_pid_.emplace(angular_config, angular_setpoint_filter, angular_derivative_filter);

  const rclcpp::QoS command_qos = rclcpp::QoS(10).reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);

  target_velocity_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    "target_velocity", command_qos,
    std::bind(&VelocityControllerNode::targetVelocityCallback, this, std::placeholders::_1));

  odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "odom", command_qos,
    std::bind(&VelocityControllerNode::odometryCallback, this, std::placeholders::_1));

  velocity_command_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", command_qos);

  declare_parameter<double>("control_frequency", 50.0);
  const double control_frequency = get_parameter("control_frequency").as_double();
  if (control_frequency <= 0.0)
  {
    throw std::invalid_argument("control_frequency must be positive, got: " +
                                std::to_string(control_frequency));
  }
  dt_s_ = 1.0 / control_frequency;
  const auto control_period = std::chrono::duration<double>(dt_s_);
  control_timer_ =
    create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(control_period),
                      std::bind(&VelocityControllerNode::controlCallback, this));

  RCLCPP_INFO(get_logger(), "Velocity controller node started");
}

void VelocityControllerNode::targetVelocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  target_velocity_ = *msg;
}

void VelocityControllerNode::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  odometry_ = *msg;
}

void VelocityControllerNode::controlCallback()
{
  double long_velocity_command_mps{0.0};
  auto maybe_long_velocity_command = long_velocity_controller_->step(
    target_velocity_.linear.x, odometry_.twist.twist.linear.x, dt_s_);
  if (!maybe_long_velocity_command)
  {
    RCLCPP_WARN(get_logger(), "Longitudinal controller failed: %s",
                domain::toString(maybe_long_velocity_command.error()).data());
  }
  else
  {
    long_velocity_command_mps = maybe_long_velocity_command.value();
  }

  double angular_velocity_command_radps{0.0};
  auto maybe_angular_velocity_command =
    angular_pid_->step(target_velocity_.angular.z, odometry_.twist.twist.angular.z, dt_s_);
  if (!maybe_angular_velocity_command)
  {
    RCLCPP_WARN(get_logger(), "Angular controller failed: %s",
                domain::toString(maybe_angular_velocity_command.error()).data());
  }
  else
  {
    angular_velocity_command_radps = maybe_angular_velocity_command.value();
  }

  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = long_velocity_command_mps;
  cmd_vel.angular.z = angular_velocity_command_radps;
  velocity_command_pub_->publish(cmd_vel);
}

}  // namespace infrastructure
}  // namespace controller
