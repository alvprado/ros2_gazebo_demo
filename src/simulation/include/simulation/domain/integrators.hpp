#ifndef SIMULATION_DOMAIN_INTEGRATORS_HPP
#define SIMULATION_DOMAIN_INTEGRATORS_HPP

#include <concepts>
#include <functional>
#include <type_traits>
#include <utility>

namespace simulation
{
namespace domain
{

/// @brief State_T and StateDerivative_T support the algebraic operations required for a numeric
/// integration scheme
template <typename State_T, typename StateDerivative_T>
concept NumericIntegrable = requires(State_T state, StateDerivative_T derivative, double dt_s) {
  // State derivative scaled with time yields a state
  { derivative * dt_s } -> std::convertible_to<State_T>;
  { dt_s * derivative } -> std::convertible_to<State_T>;
  // States support addition
  { state + state } -> std::convertible_to<State_T>;
};

/// @brief Forward Euler integration scheme
/// @details x_n+1 = x_n + f(x_n, u_n) * dt
/// @tparam State_T The type of the state x_n
/// @tparam Command_T The type of the command u_n
/// @tparam Derivative_F Callable that computes the state derivative x_dot = f(x, u)
/// @param current_state The current state
/// @param command The command
/// @param derivative_fn The derivative function callable
/// @param dt_s The time step in seconds
/// @return The next state based on forward euler integration
template <typename State_T, typename Command_T, typename Derivative_F>
  requires std::invocable<Derivative_F, const State_T&, const Command_T&> &&
           NumericIntegrable<State_T,
                             std::invoke_result_t<Derivative_F, const State_T&, const Command_T&>>
State_T forwardEulerStep(const State_T& current_state, const Command_T& command,
                         Derivative_F&& derivative_fn, double dt_s);

}  // namespace domain
}  // namespace simulation

#include "simulation/domain/integrators.inl"

#endif  // SIMULATION_DOMAIN_INTEGRATORS_HPP
