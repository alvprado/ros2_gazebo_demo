#ifndef SIMULATION_DOMAIN_ROBOT_MODEL_HPP
#define SIMULATION_DOMAIN_ROBOT_MODEL_HPP

#include <cstdint>
#include <expected>
#include <string_view>

#include "simulation/domain/types.hpp"

namespace simulation
{
namespace domain
{

/// @brief Config for the robot model
struct RobotModelConfig
{
  /// Longitudinal velocity first-order lag time constant [s]
  double long_velocity_time_constant_s{0.1};
  /// Angular velocity first-order lag time constant [s]
  double angular_velocity_time_constant_s{0.1};
};

/// @brief Robot model error codes
enum class SimulationErrorCodes : std::uint8_t
{
  NonPositiveTimeConstant = 0U,
  InvalidTimeStep = 1U,
};

/// @brief Map error codes to a string view
/// @param code simulation error code
/// @return a string view of the error code for logging purposes
inline std::string_view toString(SimulationErrorCodes code)
{
  switch (code)
  {
    case SimulationErrorCodes::NonPositiveTimeConstant:
      return "NonPositiveTimeConstant";
    case SimulationErrorCodes::InvalidTimeStep:
      return "InvalidTimeStep";
  }
  return "Unknown";
}

/// @brief Robot simulation model
/// @details Robot is modeled as a first-order system; pose is integrated with forward Euler
class RobotModel
{
public:
  /// @brief Construct a robot model from its config
  /// @param config Time constants for longitudinal and angular velocity lags
  explicit RobotModel(const RobotModelConfig& config);

  /// @brief Compute the next robot state
  /// @param velocity_cmd Longitudinal [m/s] and angular [rad/s] velocity commands
  /// @param dt_s         Time step [s]
  /// @return Updated robot state or an error code
  std::expected<State, SimulationErrorCodes> step(const Velocity2D& velocity_cmd, double dt_s);

private:
  RobotModelConfig config_;
  State previous_state_;
};

}  // namespace domain
}  // namespace simulation

#endif  // SIMULATION_DOMAIN_ROBOT_MODEL_HPP
