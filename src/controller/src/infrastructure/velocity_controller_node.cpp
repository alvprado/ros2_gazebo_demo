#include "controller/infrastructure/velocity_controller_node.hpp"

#include <chrono>
#include <stdexcept>

namespace controller
{
namespace infrastructure
{

VelocityControllerNode::VelocityControllerNode() : rclcpp::Node("velocity_controller")
{
  declare_parameter<double>("kp", 1.0);
  declare_parameter<double>("ki", 0.0);
  declare_parameter<double>("kd", 0.0);
  declare_parameter<double>("kff", 0.0);
  declare_parameter<double>("setpoint_filter_cutoff_frequency_hz", 1.0);
  declare_parameter<double>("derivative_filter_cutoff_frequency_hz", 1.0);
  declare_parameter<double>("min_velocity_mps", -100.0);
  declare_parameter<double>("max_velocity_mps", 100.0);
  declare_parameter<double>("control_frequency", 50.0);

  domain::PidConfig pid_config;
  pid_config.kp = get_parameter("kp").as_double();
  pid_config.ki = get_parameter("ki").as_double();
  pid_config.kd = get_parameter("kd").as_double();
  pid_config.kff = get_parameter("kff").as_double();
  pid_config.min_output = get_parameter("min_velocity_mps").as_double();
  pid_config.max_output = get_parameter("max_velocity_mps").as_double();

  const domain::LowPassFilter feedforward_filter(
    get_parameter("setpoint_filter_cutoff_frequency_hz").as_double());
  const domain::LowPassFilter derivative_filter(
    get_parameter("derivative_filter_cutoff_frequency_hz").as_double());

  pid_controller_.emplace(pid_config, feedforward_filter, derivative_filter);

  const rclcpp::QoS command_qos = rclcpp::QoS(10).reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);

  target_velocity_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    "target_velocity", command_qos,
    std::bind(&VelocityControllerNode::targetVelocityCallback, this, std::placeholders::_1));

  odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "odom", command_qos,
    std::bind(&VelocityControllerNode::odometryCallback, this, std::placeholders::_1));

  velocity_command_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", command_qos);

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
  // Fallback velocity command is zero
  double velocity_command{0.0};

  // Step controller
  auto maybe_velocity_command =
    pid_controller_->step(target_velocity_.linear.x, odometry_.twist.twist.linear.x, dt_s_);

  if (!maybe_velocity_command)
  {
    RCLCPP_WARN(get_logger(), "Controller failed with error %s",
                domain::toString(maybe_velocity_command.error()).data());
  }
  else
  {
    velocity_command = maybe_velocity_command.value();
  }

  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = velocity_command;
  cmd_vel.angular.z = 0.0;
  velocity_command_pub_->publish(cmd_vel);
}

}  // namespace infrastructure
}  // namespace controller
