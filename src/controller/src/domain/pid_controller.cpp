#include "controller/domain/pid_controller.hpp"

#include <algorithm>

namespace controller
{
namespace domain
{

PidController::PidController(const PidGains& gains)
: gains_(gains)
{
}

double PidController::step(double setpoint, double current, double dt)
{
  const double error = setpoint - current;

  constexpr double tol{1E-4};
  double derivative{0.0};
  if (dt > tol) {
    derivative = (error - previous_error_) / dt;
  }
  previous_error_ = error;

  const double raw = gains_.kp * error + gains_.ki * integral_ + gains_.kd * derivative;
  const double clamped = std::clamp(raw, gains_.min_output, gains_.max_output);

  // Anti-windup: only integrate when the error would not push a saturated
  // output further into saturation.  saturation_error is non-zero only when
  // clamped; its sign matches the saturation direction.  When error and
  // saturation_error have the same sign the integral would worsen windup —
  // freeze it.  When they have opposite signs the error is unwinding the
  // integrator, so allow integration to continue even while saturated.
  const double saturation_error = raw - clamped;
  if (error * saturation_error <= 0.0) {
    integral_ += error * dt;
  }

  return clamped;
}

}  // namespace domain
}  // namespace controller
