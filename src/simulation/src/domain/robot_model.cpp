#include "simulation/domain/robot_model.hpp"

namespace simulation
{
namespace domain
{

RobotModel::RobotModel(const RobotModelConfig & config)
: config_(config)
{
}

State RobotModel::step(const Velocity2D & velocity_cmd, const double dt)
{
  State state{};

  double const first_order_lag_a = (dt / (config_.long_time_constant + dt));
  double const first_order_lag_b = (config_.long_time_constant / (config_.long_time_constant + dt));

  state.velocity.linear = first_order_lag_a * velocity_cmd.linear + first_order_lag_b * previous_state_.velocity.linear;
  state.pose.x = state.velocity.linear * dt + previous_state_.pose.x;

  previous_state_ = state;
  return state;
}

}  // namespace domain
}  // namespace simulation
