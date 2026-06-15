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

  const double derivative = (error - previous_error_) / dt;

  previous_error_ = error;

  return gains_.kp * error + gains_.ki * integral_ + gains_.kd * derivative;
}

}  // namespace domain
}  // namespace controller
