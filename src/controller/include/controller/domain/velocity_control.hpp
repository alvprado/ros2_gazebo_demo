#ifndef CONTROLLER_DOMAIN_VELOCITY_CONTROL_HPP
#define CONTROLLER_DOMAIN_VELOCITY_CONTROL_HPP

namespace controller
{
namespace domain
{

/// @brief Config of the velocity controller
struct VelocityControlConfig
{
  double max_velocity_mps{100.0};
  double min_velocity_mps{-100.0};
};

/// @brief Generic velocity controller
/// @tparam ControlLaw_T Control law used to track velocity
template<class ControlLaw_T>
class VelocityControl
{
public:
  VelocityControl(const ControlLaw_T& controller, const VelocityControlConfig& config);

  /// @brief Compute the velocity command
  /// @param target_velocity_mps desired velocity in m/s
  /// @param current_velocity_mps current measured velocity in m/s
  /// @param dt_s step time in s
  /// @returns A velocity command
  double step(
    double target_velocity_mps,
    double current_velocity_mps,
    double dt_s);


private:
  ControlLaw_T controller_;
  VelocityControlConfig config_;
};

}  // namespace domain
}  // namespace controller

#include "controller/domain/velocity_control.inl"

#endif  // CONTROLLER_DOMAIN_VELOCITY_CONTROL_HPP
