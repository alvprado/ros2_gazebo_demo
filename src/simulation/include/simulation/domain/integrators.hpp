#ifndef SIMULATION_DOMAIN_INTEGRATORS_HPP
#define SIMULATION_DOMAIN_INTEGRATORS_HPP

#include <functional>
#include <utility>

namespace simulation
{
namespace domain
{

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
State_T forwardEulerStep(const State_T& current_state, const Command_T& command,
                         Derivative_F&& derivative_fn, double dt_s)
{
  return current_state +
         std::invoke(std::forward<Derivative_F>(derivative_fn), current_state, command) * dt_s;
}

}  // namespace domain
}  // namespace simulation

#endif  // SIMULATION_DOMAIN_INTEGRATORS_HPP
