#ifndef SIMULATION_DOMAIN_ROBOT_MODEL_HPP
#define SIMULATION_DOMAIN_ROBOT_MODEL_HPP

#include <cstdint>
#include <expected>
#include <functional>
#include <string_view>

#include "simulation/domain/robot_state_types.hpp"

namespace simulation
{
namespace domain
{

/// @brief Config for the robot model
struct RobotModelConfig
{
  /// Longitudinal velocity first-order lag time constant in s
  double long_velocity_time_constant_s{0.1};
  /// Angular velocity first-order lag time constant in s
  double angular_velocity_time_constant_s{0.1};
  /// Wheel radius in m
  double wheel_radius_m{0.05};
  /// Distance between left and right wheel centres in m
  double wheel_separation_m{0.30};
};

/// @brief Robot model error codes
enum class SimulationErrorCodes : std::uint8_t
{
  NonPositiveTimeConstant = 0U,
  InvalidTimeStep = 1U,
  InvalidWheelConfig = 2U,
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
    case SimulationErrorCodes::InvalidWheelConfig:
      return "InvalidWheelConfig";
  }
  return "Unknown";
}

/// @brief The robot model used for simulation
/// @details It steps the state of the robot given an
/// integration method and the robot parameters based on the robot dynamics described in the
/// computeDerivative method
class RobotModel
{
public:
  /// @brief Alias for the robot commands - linear and angular velocities
  using Command = Velocity2D;

  /// @brief Alias for callable that evaluates the robot dynamics x_dot = f(x, u)
  using DerivativeFunction = std::function<StateDerivative(const State&, const Command&)>;

  /// @brief Alias for a functor that advances the state by one time step using any integration
  /// scheme
  using IntegrationMethod =
    std::function<State(const State&, const Command&, const DerivativeFunction&, double)>;

  /// @brief Construct a robot model from its config and a pluggable integration method
  /// @param integrator Integration method (e.g. forward euler, heun, Runge-Kutta, etc)
  /// @param config     Robot model parameters
  RobotModel(IntegrationMethod integrator, const RobotModelConfig& config);

  /// @brief Compute the next robot state
  /// @param velocity_cmd longitudinal and angular velocity commands
  /// @param dt_s         time step in seconds
  /// @return Updated robot state or an error code
  std::expected<State, SimulationErrorCodes> step(const Command& velocity_cmd, double dt_s);

private:
  /// @brief Computation of a robot state derivative based on the robot dynamics
  /// @param state The current robot state
  /// @param velocity_cmd The velocity commands
  /// @return A state derivative
  StateDerivative computeDerivative(const State& state, const Command& velocity_cmd) const;

  // Robot parameters
  RobotModelConfig config_;

  // The current robot state
  State state_{};

  // The integration scheme
  IntegrationMethod integrator_;
};

}  // namespace domain
}  // namespace simulation

#endif  // SIMULATION_DOMAIN_ROBOT_MODEL_HPP
