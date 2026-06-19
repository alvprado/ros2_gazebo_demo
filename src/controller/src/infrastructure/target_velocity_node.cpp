#include "controller/infrastructure/target_velocity_node.hpp"

#include <algorithm>
#include <chrono>
#include <iterator>

namespace controller
{
namespace infrastructure
{

namespace
{
constexpr double kfallback_velocity_mps = 10.0;
constexpr double kfallback_duration_s = 10.0;
}  // namespace

TargetVelocityNode::TargetVelocityNode()
: rclcpp::Node("target_velocity_node")
{
  declare_parameter<std::vector<double>>("target_velocities_mps", std::vector<double>{});
  declare_parameter<std::vector<double>>("durations_s", std::vector<double>{});
  declare_parameter<double>("target_velocity_update_frequency_hz", 50.0);
  declare_parameter<bool>("loop", false);

  const auto target_velocities_mps = get_parameter("target_velocities_mps").as_double_array();
  const auto durations_s = get_parameter("durations_s").as_double_array();

  std::vector<domain::VelocityStep> velocity_steps;

  if (target_velocities_mps.empty()) {
    RCLCPP_ERROR(get_logger(),
      "Parameter 'target_velocities_mps' is empty — using fallback values");
    velocity_steps = {{kfallback_velocity_mps, kfallback_duration_s}};
  } else if (target_velocities_mps.size() != durations_s.size()) {
    RCLCPP_ERROR(get_logger(),
      "'target_velocities_mps' and 'durations_s' must have the same length — using fallback values");
    velocity_steps = {{kfallback_velocity_mps, kfallback_duration_s}};
  } else {
    velocity_steps.reserve(target_velocities_mps.size());
    std::transform(
      target_velocities_mps.begin(), target_velocities_mps.end(),
      durations_s.begin(),
      std::back_inserter(velocity_steps),
      [](double velocity_mps, double duration_s) {
        return domain::VelocityStep{velocity_mps, duration_s};
      });
  }

  velocity_profile_.emplace(std::move(velocity_steps), get_parameter("loop").as_bool());

  const rclcpp::QoS qos = rclcpp::QoS(10).reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  target_velocity_pub_ = create_publisher<geometry_msgs::msg::Twist>("target_velocity", qos);

  const double dt_s = 1.0 / get_parameter("target_velocity_update_frequency_hz").as_double();
  const auto period = std::chrono::duration<double>(dt_s);
  target_velocity_pub_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&TargetVelocityNode::publishCallback, this));

  RCLCPP_INFO(get_logger(), "Target velocity node started");
}

void TargetVelocityNode::publishCallback()
{
  geometry_msgs::msg::Twist msg;
  msg.linear.x = velocity_profile_->step(this->now().seconds());
  target_velocity_pub_->publish(msg);

  if (velocity_profile_->isFinished()) {
    RCLCPP_INFO(get_logger(), "Velocity profile complete, shutting down");
    rclcpp::shutdown();
  }
}

}  // namespace infrastructure
}  // namespace controller
