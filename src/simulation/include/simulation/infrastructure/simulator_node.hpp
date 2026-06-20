#ifndef SIMULATION__INFRASTRUCTURE__SIMULATOR_NODE_HPP_
#define SIMULATION__INFRASTRUCTURE__SIMULATOR_NODE_HPP_

#include <optional>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

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
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void simulationTimerCallback();

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr simulation_timer_;

  std::optional<domain::RobotModel> robot_model_;

  domain::Velocity2D cmd_vel_;

  double dt_s_{0.0};
};

}  // namespace infrastructure
}  // namespace simulation

#endif  // SIMULATION__INFRASTRUCTURE__SIMULATOR_NODE_HPP_
