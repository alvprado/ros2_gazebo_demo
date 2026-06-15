#ifndef SIMULATION_DOMAIN_ROBOT_MODEL_HPP
#define SIMULATION_DOMAIN_ROBOT_MODEL_HPP

#include "simulation/domain/types.hpp"

namespace simulation
{
namespace domain
{

/// @brief Config for the robot model
struct RobotModelConfig
{
  /// Longitudinal time constant
  double long_time_constant{0.0};
};

/// @brief Robot simulation model
class RobotModel
{
public:
  explicit RobotModel(const RobotModelConfig& config);

  /// @brief Compute the next robot state depending on the velocity command and time step
  /// @param velocity_cmd velocity command
  /// @returns An updated state of the robot
  State step(const Velocity2D & velocity_cmd, const double dt);

private:
  RobotModelConfig config_;
  State previous_state_;
};

}  // namespace domain
}  // namespace simulation

#endif  // SIMULATION_DOMAIN_ROBOT_MODEL_HPP
