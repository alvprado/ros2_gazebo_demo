#include "controller/domain/pid_controller.hpp"

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

  integral_ += error * dt;

  constexpr double tol{1E-4};
  double derivative{0.0};
  if(dt > tol)
  {
      derivative = (error - previous_error_) / dt;
  }

  previous_error_ = error;

  return gains_.kp * error + gains_.ki * integral_ + gains_.kd * derivative;
}

}  // namespace domain
}  // namespace controller
