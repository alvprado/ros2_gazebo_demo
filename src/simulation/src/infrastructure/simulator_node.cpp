#include "simulation/infrastructure/simulator_node.hpp"

#include <chrono>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;

namespace simulation
{
namespace infrastructure
{

SimulatorNode::SimulatorNode()
: rclcpp::Node("robot_simulation")
{
  declare_parameter<double>("long_time_constant", 0.1);
  declare_parameter<double>("simulation_frequency", 20.0);

  domain::RobotModelConfig config{get_parameter("long_time_constant").as_double()};
  robot_model_.emplace(config);

  const rclcpp::QoS command_qos =
    rclcpp::QoS(10).reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);

  cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", command_qos,
    std::bind(&SimulatorNode::cmdVelCallback, this, std::placeholders::_1));

  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", command_qos);

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

  const double simulation_frequency = get_parameter("simulation_frequency").as_double();
  dt_ = 1.0 / simulation_frequency;
  const auto simulation_period = std::chrono::duration<double>(dt_);
  simulation_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(simulation_period),
    std::bind(&SimulatorNode::simulationTimerCallback, this));

  RCLCPP_INFO(get_logger(), "simulation node started");
}

void SimulatorNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  cmd_vel_.linear = msg->linear.x;
  cmd_vel_.angular = msg->angular.z;
}

void SimulatorNode::simulationTimerCallback()
{
  const rclcpp::Time now = get_clock()->now();

  const auto state = robot_model_->step(cmd_vel_, dt_);

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, state.pose.theta);

  nav_msgs::msg::Odometry odom;
  odom.header.stamp = now;
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_link";
  odom.pose.pose.position.x = state.pose.x;
  odom.pose.pose.position.y = state.pose.y;
  odom.pose.pose.orientation = tf2::toMsg(q);
  odom.twist.twist.linear.x = state.velocity.linear;
  odom.twist.twist.angular.z = state.velocity.angular;

  odom_pub_->publish(odom);

  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = now;
  transform.header.frame_id = "odom";
  transform.child_frame_id = "base_link";
  transform.transform.translation.x = state.pose.x;
  transform.transform.translation.y = state.pose.y;
  transform.transform.rotation = tf2::toMsg(q);

  tf_broadcaster_->sendTransform(transform);
}

}  // namespace infrastructure
}  // namespace simulation
