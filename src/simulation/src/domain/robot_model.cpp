#include "simulation/domain/robot_model.hpp"

#include <cmath>

namespace simulation
{
namespace domain
{

RobotModel::RobotModel(IntegrationMethod integrator, const RobotModelConfig& config)
  : config_(config), integrator_(std::move(integrator))
{
}

StateDerivative RobotModel::computeDerivative(const State& state, const Command& velocity_cmd) const
{
  StateDerivative state_derivative;

  // Compute velocities derivatives
  state_derivative.velocity.linear_dot_mps2 =
    (velocity_cmd.linear_mps - state.velocity.linear_mps) / config_.long_velocity_time_constant_s;
  state_derivative.velocity.angular_dot_radps2 =
    (velocity_cmd.angular_radps - state.velocity.angular_radps) /
    config_.angular_velocity_time_constant_s;

  // Compute pose derivatives
  state_derivative.pose.x_dot_mps = state.velocity.linear_mps * std::cos(state.pose.theta_rad);
  state_derivative.pose.y_dot_mps = state.velocity.linear_mps * std::sin(state.pose.theta_rad);
  state_derivative.pose.theta_dot_radps = state.velocity.angular_radps;

  // Compute wheel angle derivatives
  state_derivative.wheel_angles.left_dot_radps =
    (state.velocity.linear_mps - state.velocity.angular_radps * config_.wheel_separation_m / 2.0) /
    config_.wheel_radius_m;
  state_derivative.wheel_angles.right_dot_radps =
    (state.velocity.linear_mps + state.velocity.angular_radps * config_.wheel_separation_m / 2.0) /
    config_.wheel_radius_m;

  return state_derivative;
}

std::expected<State, SimulationErrorCodes> RobotModel::step(const Command& velocity_cmd,
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

  const DerivativeFunction derivative_function = [this](const State& state, const Command& command)
  { return computeDerivative(state, command); };

  const State next_state = integrator_(state_, velocity_cmd, derivative_function, dt_s);
  state_ = next_state;
  return next_state;
}

}  // namespace domain
}  // namespace simulation
