#ifndef CONTROLLER_DOMAIN_CURVATURE_CONTROLLER_INL
#define CONTROLLER_DOMAIN_CURVATURE_CONTROLLER_INL

#include <cmath>

#include "controller/domain/curvature_controller.hpp"

namespace controller
{
namespace domain
{

template <ControlLaw ControlLaw_T>
CurvatureController<ControlLaw_T>::CurvatureController(const ControlLaw_T& control_law,
                                                       const CurvatureControllerConfig& config)
  : control_law_(control_law), config_(config)
{
}

template <ControlLaw ControlLaw_T>
std::expected<double, CurvatureControllerErrorCodes> CurvatureController<ControlLaw_T>::step(
  double target_curvature_pm, double current_angular_velocity_radps,
  double current_longitudinal_velocity_mps, double dt_s)
{
  if (dt_s <= 0)
  {
    return std::unexpected(CurvatureControllerErrorCodes::InvalidTimeStep);
  }

  double target_angular_velocity_radps{0.0};
  if (std::abs(current_longitudinal_velocity_mps) >= config_.lower_long_velocity_threshold_mps)
  {
    target_angular_velocity_radps = target_curvature_pm * current_longitudinal_velocity_mps;
  }
  else if (config_.spin_in_place)
  {
    target_angular_velocity_radps = config_.spin_velocity_radps;
  }

  auto const angular_velocity_command =
    control_law_.step(target_angular_velocity_radps, current_angular_velocity_radps, dt_s);

  if (!angular_velocity_command)
  {
    return std::unexpected(CurvatureControllerErrorCodes::FailedControlLaw);
  }

  return angular_velocity_command.value();
}

}  // namespace domain
}  // namespace controller

#endif  // CONTROLLER_DOMAIN_CURVATURE_CONTROLLER_INL
