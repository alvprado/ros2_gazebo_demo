#ifndef SIMULATION__INFRASTRUCTURE__SIMULATOR_NODE_HPP_
#define SIMULATION__INFRASTRUCTURE__SIMULATOR_NODE_HPP_

#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <string>

#include "simulation/domain/robot_model.hpp"

namespace simulation
{
namespace infrastructure
{

/// @brief Simulation node
class SimulatorNode : public rclcpp::Node
{
public:
  SimulatorNode();

private:
  void velocityCommandCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void simulationCallback();

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_command_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr simulation_timer_;

  std::optional<domain::RobotModel> robot_model_;

  domain::Velocity2D cmd_vel_;

  double dt_s_{0.0};
};

}  // namespace infrastructure
}  // namespace simulation

#endif  // SIMULATION__INFRASTRUCTURE__SIMULATOR_NODE_HPP_
