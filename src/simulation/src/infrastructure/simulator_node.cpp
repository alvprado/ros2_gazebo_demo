#include "simulation/infrastructure/simulator_node.hpp"

#include <tf2/LinearMath/Quaternion.h>

#include <chrono>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;

namespace simulation
{
namespace infrastructure
{

SimulatorNode::SimulatorNode() : rclcpp::Node("robot_simulation")
{
  declare_parameter<double>("long_velocity_time_constant_s", 0.1);
  declare_parameter<double>("angular_velocity_time_constant_s", 0.1);
  declare_parameter<double>("simulation_frequency", 20.0);

  declare_parameter<double>("wheel_radius_m", 0.05);
  declare_parameter<double>("wheel_separation_m", 0.30);

  domain::RobotModelConfig config;
  config.long_velocity_time_constant_s = get_parameter("long_velocity_time_constant_s").as_double();
  config.angular_velocity_time_constant_s =
    get_parameter("angular_velocity_time_constant_s").as_double();
  config.wheel_radius_m = get_parameter("wheel_radius_m").as_double();
  config.wheel_separation_m = get_parameter("wheel_separation_m").as_double();
  robot_model_.emplace(config);

  const rclcpp::QoS command_qos = rclcpp::QoS(10).reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);

  cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", command_qos, std::bind(&SimulatorNode::cmdVelCallback, this, std::placeholders::_1));

  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", command_qos);
  joint_state_pub_ =
    create_publisher<sensor_msgs::msg::JointState>("joint_states", command_qos);

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

  const double simulation_frequency = get_parameter("simulation_frequency").as_double();
  dt_s_ = 1.0 / simulation_frequency;
  const auto simulation_period = std::chrono::duration<double>(dt_s_);
  simulation_timer_ =
    create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(simulation_period),
                      std::bind(&SimulatorNode::simulationTimerCallback, this));

  RCLCPP_INFO(get_logger(), "Simulation node started");
}

void SimulatorNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  cmd_vel_.linear_mps = msg->linear.x;
  cmd_vel_.angular_radps = msg->angular.z;
}

void SimulatorNode::simulationTimerCallback()
{
  const rclcpp::Time now = get_clock()->now();

  const auto maybe_state = robot_model_->step(cmd_vel_, dt_s_);
  if (!maybe_state)
  {
    RCLCPP_WARN(get_logger(), "Robot model step failed: %s",
                domain::toString(maybe_state.error()).data());
    return;
  }
  const auto& state = maybe_state.value();

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, state.pose.theta_rad);

  nav_msgs::msg::Odometry odom;
  odom.header.stamp = now;
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_link";
  odom.pose.pose.position.x = state.pose.x_m;
  odom.pose.pose.position.y = state.pose.y_m;
  odom.pose.pose.orientation = tf2::toMsg(q);
  odom.twist.twist.linear.x = state.velocity.linear_mps;
  odom.twist.twist.angular.z = state.velocity.angular_radps;

  odom_pub_->publish(odom);

  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = now;
  transform.header.frame_id = "odom";
  transform.child_frame_id = "base_link";
  transform.transform.translation.x = state.pose.x_m;
  transform.transform.translation.y = state.pose.y_m;
  transform.transform.rotation = tf2::toMsg(q);

  tf_broadcaster_->sendTransform(transform);

  sensor_msgs::msg::JointState joint_state;
  joint_state.header.stamp = now;
  joint_state.name = {"left_wheel_joint", "right_wheel_joint"};
  joint_state.position = {state.wheel_angles.left_rad, state.wheel_angles.right_rad};

  joint_state_pub_->publish(joint_state);
}

}  // namespace infrastructure
}  // namespace simulation
