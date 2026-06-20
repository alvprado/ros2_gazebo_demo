#include "simulation/domain/robot_model.hpp"

#include <cmath>

namespace simulation
{
namespace domain
{

RobotModel::RobotModel(const RobotModelConfig& config) : config_(config) {}

std::expected<State, SimulationErrorCodes> RobotModel::step(const Velocity2D& velocity_cmd,
                                                            double dt_s)
{
  if (dt_s <= 0.0)
  {
    return std::unexpected(SimulationErrorCodes::InvalidTimeStep);
  }
  if (config_.long_velocity_time_constant_s <= 0.0 ||
      config_.angular_velocity_time_constant_s <= 0.0)
  {
    return std::unexpected(SimulationErrorCodes::NonPositiveTimeConstant);
  }
  if (config_.wheel_radius_m <= 0.0 || config_.wheel_separation_m <= 0.0)
  {
    return std::unexpected(SimulationErrorCodes::InvalidWheelConfig);
  }

  State state{};

  const double long_alpha = dt_s / (config_.long_velocity_time_constant_s + dt_s);
  const double long_beta =
    config_.long_velocity_time_constant_s / (config_.long_velocity_time_constant_s + dt_s);
  state.velocity.linear_mps =
    long_alpha * velocity_cmd.linear_mps + long_beta * previous_state_.velocity.linear_mps;

  const double ang_alpha = dt_s / (config_.angular_velocity_time_constant_s + dt_s);
  const double ang_beta =
    config_.angular_velocity_time_constant_s / (config_.angular_velocity_time_constant_s + dt_s);
  state.velocity.angular_radps =
    ang_alpha * velocity_cmd.angular_radps + ang_beta * previous_state_.velocity.angular_radps;

  // Integrate pose using previous heading (forward Euler)
  state.pose.theta_rad = previous_state_.pose.theta_rad + state.velocity.angular_radps * dt_s;
  state.pose.x_m = previous_state_.pose.x_m +
                   state.velocity.linear_mps * std::cos(previous_state_.pose.theta_rad) * dt_s;
  state.pose.y_m = previous_state_.pose.y_m +
                   state.velocity.linear_mps * std::sin(previous_state_.pose.theta_rad) * dt_s;

  // Integrate wheel angles using differential drive kinematics
  const double left_vel_radps =
    (state.velocity.linear_mps - state.velocity.angular_radps * config_.wheel_separation_m / 2.0) /
    config_.wheel_radius_m;
  const double right_vel_radps =
    (state.velocity.linear_mps + state.velocity.angular_radps * config_.wheel_separation_m / 2.0) /
    config_.wheel_radius_m;
  // Update wheel angles and wrap to (-pi, pi]
  state.wheel_angles.left_rad =
    std::remainder(previous_state_.wheel_angles.left_rad + left_vel_radps * dt_s, 2.0 * M_PI);
  state.wheel_angles.right_rad =
    std::remainder(previous_state_.wheel_angles.right_rad + right_vel_radps * dt_s, 2.0 * M_PI);

  previous_state_ = state;
  return state;
}

}  // namespace domain
}  // namespace simulation
