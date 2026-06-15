#ifndef CONTROLLER_DOMAIN_PID_CONTROLLER_HPP
#define CONTROLLER_DOMAIN_PID_CONTROLLER_HPP

namespace controller
{
namespace domain
{

/// @brief PID controller gains
struct PidGains
{
  double kp{0.0};
  double ki{0.0};
  double kd{0.0};
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
