#include "controller/infrastructure/velocity_controller_node.hpp"

#include <chrono>

using namespace std::chrono_literals;

namespace controller
{
namespace infrastructure
{

VelocityControllerNode::VelocityControllerNode()
: rclcpp::Node("velocity_controller")
{
  declare_parameter<double>("kp", 1.0);
  declare_parameter<double>("ki", 0.0);
  declare_parameter<double>("kd", 0.0);
  declare_parameter<double>("min_velocity_mps", -100.0);
  declare_parameter<double>("max_velocity_mps", 100.0);
  declare_parameter<double>("control_frequency", 50.0);

  domain::PidGains gains{
    get_parameter("kp").as_double(),
    get_parameter("ki").as_double(),
    get_parameter("kd").as_double()};

  auto const pid_controller = domain::PidController{gains};
  auto const velocity_control_config = domain::VelocityControlConfig{get_parameter("max_velocity_mps").as_double(), get_parameter("min_velocity_mps").as_double()};
  velocity_control_.emplace(pid_controller, velocity_control_config);

  const rclcpp::QoS command_qos =
    rclcpp::QoS(10).reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);

  target_velocity_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    "target_velocity", command_qos,
    std::bind(&VelocityControllerNode::targetVelocityCallback, this, std::placeholders::_1));

  odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "odom", command_qos,
    std::bind(&VelocityControllerNode::odometryCallback, this, std::placeholders::_1));

  velocity_command_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", command_qos);

  const double control_frequency = get_parameter("control_frequency").as_double();
  dt_ = 1.0 / control_frequency;
  const auto control_period = std::chrono::duration<double>(dt_);
  control_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(control_period),
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
  const double velocity_command = velocity_control_->step(
    target_velocity_.linear.x, odometry_.twist.twist.linear.x, dt_);

  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = velocity_command;
  cmd_vel.angular.z = 0.0;

  velocity_command_pub_->publish(cmd_vel);
}

}  // namespace infrastructure
}  // namespace controller
