#ifndef CONTROLLER_DOMAIN_VELOCITY_CONTROL_INL
#define CONTROLLER_DOMAIN_VELOCITY_CONTROL_INL

#include "controller/domain/velocity_control.hpp"
#include <algorithm>

namespace controller
{
namespace domain
{

template<class Controller_T>
VelocityControl<Controller_T>::VelocityControl(const Controller_T& controller, const VelocityControlConfig& config)
: controller_(controller), config_(config)
{
}

template<class Controller_T>
double VelocityControl<Controller_T>::step(
    double target_velocity_mps,
    double current_velocity_mps,
    double dt_s)
{
  return std::clamp(controller_.step(target_velocity_mps, current_velocity_mps, dt_s), config_.min_velocity_mps, config_.max_velocity_mps);
}


}  // namespace domain
}  // namespace controller

#endif  // CONTROLLER_DOMAIN_VELOCITY_CONTROL_INL
