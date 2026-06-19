#ifndef CONTROLLER_DOMAIN_PID_CONTROLLER_INL
#define CONTROLLER_DOMAIN_PID_CONTROLLER_INL

#include <algorithm>

#include "controller/domain/pid_controller.hpp"

namespace controller
{
namespace domain
{

template <typename SetpointFilter_T, typename DerivativeFilter_T>
PidController<SetpointFilter_T, DerivativeFilter_T>::PidController(
  const PidConfig& config, const SetpointFilter_T& setpoint_filter,
  const DerivativeFilter_T& derivative_filter)
  : config_(config), setpoint_filter_(setpoint_filter), derivative_filter_(derivative_filter)
{
}

template <typename SetpointFilter_T, typename DerivativeFilter_T>
std::expected<double, PidErrorCodes> PidController<SetpointFilter_T, DerivativeFilter_T>::step(
  double setpoint, double current, double dt_s)
{
  if ((config_.kp < 0) || (config_.ki < 0) || (config_.kd < 0) || (config_.kff < 0))
  {
    return std::unexpected(PidErrorCodes::NonPositiveGains);
  }

  if (config_.min_output >= config_.max_output)
  {
    return std::unexpected(PidErrorCodes::InvalidLimits);
  }

  if (dt_s <= 0)
  {
    return std::unexpected(PidErrorCodes::InvalidTimeStep);
  }

  // Filter setpoint for feedforward - fallback to raw setpoint on filter error
  double filtered_setpoint{setpoint};
  auto const maybe_filtered_setpoint = setpoint_filter_.step(setpoint, dt_s);
  filtered_setpoint = maybe_filtered_setpoint.value_or(filtered_setpoint);

  // Compute error
  const double error = filtered_setpoint - current;

  // Compute derivative only if gain is positive to avoid unnecesary filtering
  double derivative{0.0};
  if (config_.kd > 0)
  {
    double raw_derivative = (error - state_.previous_error_) / dt_s;
    auto const maybe_filtered_derivative = derivative_filter_.step(raw_derivative, dt_s);
    derivative = maybe_filtered_derivative.value_or(raw_derivative);
  }

  // Update state for next step
  state_.previous_error_ = error;

  // Feedforward + PID output
  const double feedforward = config_.kff * filtered_setpoint;
  const double raw_command =
    feedforward + config_.kp * error + config_.ki * state_.integral_ + config_.kd * derivative;
  const double limited_command = std::clamp(raw_command, config_.min_output, config_.max_output);

  // Anti-windup: freeze integration when saturated and error pushes further into saturation
  const double saturation_error = raw_command - limited_command;
  if (error * saturation_error <= 0.0)
  {
    state_.integral_ += error * dt_s;
  }

  return limited_command;
}

}  // namespace domain
}  // namespace controller

#endif  // CONTROLLER_DOMAIN_PID_CONTROLLER_INL
