#ifndef CONTROLLER_DOMAIN_PID_CONTROLLER_HPP
#define CONTROLLER_DOMAIN_PID_CONTROLLER_HPP

#include <limits>

namespace controller
{
namespace domain
{

/// @brief PID controller gains and output limits
struct PidGains
{
  double kp{0.0};
  double ki{0.0};
  double kd{0.0};
  // Output clamp — also used for anti-windup: integration is frozen when the
  // output is saturated and the error would push it further into saturation.
  double min_output{-std::numeric_limits<double>::infinity()};
  double max_output{ std::numeric_limits<double>::infinity()};
};

/// @brief PID controller class
class PidController
{
public:
  explicit PidController(const PidGains& gains);

  /// @brief Step the PID controller 
  /// @param setpoint desired value
  /// @param current current measured value
  /// @param dt controller time step
  /// @return control command
  double step(double setpoint, double current, double dt);

private:
  PidGains gains_;
  double integral_{0.0};
  double previous_error_{0.0};
};

}  // namespace domain
}  // namespace controller

#endif  // CONTROLLER_DOMAIN_PID_CONTROLLER_HPP
