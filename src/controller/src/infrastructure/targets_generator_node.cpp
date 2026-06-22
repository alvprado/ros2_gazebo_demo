#include "controller/infrastructure/targets_generator_node.hpp"

#include <algorithm>
#include <chrono>
#include <iterator>

namespace controller
{
namespace infrastructure
{

namespace
{
constexpr double kfallback_value = 0.0;
constexpr double kfallback_duration_s = 10.0;
}  // namespace

TargetsGeneratorNode::TargetsGeneratorNode() : rclcpp::Node("targets_generator")
{
  declare_parameter<std::vector<double>>("longitudinal.target_velocities_mps",
                                         std::vector<double>{});
  declare_parameter<std::vector<double>>("longitudinal.durations_s", std::vector<double>{});
  declare_parameter<std::vector<double>>("curvature.target_curvatures_pm", std::vector<double>{});
  declare_parameter<std::vector<double>>("curvature.durations_s", std::vector<double>{});
  declare_parameter<double>("targets_update_frequency_hz", 20.0);
  declare_parameter<bool>("loop", false);

  const bool loop = get_parameter("loop").as_bool();

  auto build_targets = [&](const std::vector<double>& values, const std::vector<double>& durations,
                           const std::string& val_param,
                           const std::string& dur_param) -> std::vector<domain::TimedTarget>
  {
    if (values.empty())
    {
      RCLCPP_ERROR(get_logger(), "Parameter '%s' is empty — using fallback values",
                   val_param.c_str());
      return {{kfallback_value, kfallback_duration_s}};
    }
    if (values.size() != durations.size())
    {
      RCLCPP_ERROR(get_logger(), "'%s' and '%s' must have the same length — using fallback values",
                   val_param.c_str(), dur_param.c_str());
      return {{kfallback_value, kfallback_duration_s}};
    }
    std::vector<domain::TimedTarget> targets;
    targets.reserve(values.size());
    std::transform(values.begin(), values.end(), durations.begin(), std::back_inserter(targets),
                   [](double v, double d) { return domain::TimedTarget{v, d}; });
    return targets;
  };

  long_velocity_targets_.emplace(
    build_targets(get_parameter("longitudinal.target_velocities_mps").as_double_array(),
                  get_parameter("longitudinal.durations_s").as_double_array(),
                  "longitudinal.target_velocities_mps", "longitudinal.durations_s"),
    loop);

  curvature_targets_.emplace(
    build_targets(get_parameter("curvature.target_curvatures_pm").as_double_array(),
                  get_parameter("curvature.durations_s").as_double_array(),
                  "curvature.target_curvatures_pm", "curvature.durations_s"),
    loop);

  const rclcpp::QoS qos = rclcpp::QoS(10).reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  target_long_velocity_pub_ =
    create_publisher<std_msgs::msg::Float64>("target_longitudinal_velocity", qos);
  target_curvature_pub_ = create_publisher<std_msgs::msg::Float64>("target_curvature", qos);

  // TODO: Protect division by zero - rename period
  const double dt_s = 1.0 / get_parameter("targets_update_frequency_hz").as_double();
  const auto period = std::chrono::duration<double>(dt_s);
  targets_generator_timer_ =
    create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period),
                      std::bind(&TargetsGeneratorNode::targetsGeneratorCallback, this));

  RCLCPP_INFO(get_logger(), "Targets generator node started");
}

void TargetsGeneratorNode::targetsGeneratorCallback()
{
  const double now_s = get_clock()->now().seconds();

  std_msgs::msg::Float64 long_msg;
  long_msg.data = long_velocity_targets_->step(now_s);
  target_long_velocity_pub_->publish(long_msg);

  std_msgs::msg::Float64 curv_msg;
  curv_msg.data = curvature_targets_->step(now_s);
  target_curvature_pub_->publish(curv_msg);

  if (long_velocity_targets_->isFinished() && curvature_targets_->isFinished())
  {
    RCLCPP_INFO(get_logger(), "Targets profile complete, shutting down");
    rclcpp::shutdown();
  }
}

}  // namespace infrastructure
}  // namespace controller
