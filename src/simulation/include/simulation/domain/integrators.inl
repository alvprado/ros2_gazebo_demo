#ifndef SIMULATION_DOMAIN_INTEGRATORS_INL
#define SIMULATION_DOMAIN_INTEGRATORS_INL

#include "simulation/domain/integrators.hpp"

namespace simulation
{
namespace domain
{

template <typename State_T, typename Command_T, typename Derivative_F>
  requires std::invocable<Derivative_F, const State_T&, const Command_T&> &&
           NumericIntegrable<State_T,
                             std::invoke_result_t<Derivative_F, const State_T&, const Command_T&>>
State_T forwardEulerStep(const State_T& current_state, const Command_T& command,
                         Derivative_F&& derivative_fn, double dt_s)
{
  return current_state +
         std::invoke(std::forward<Derivative_F>(derivative_fn), current_state, command) * dt_s;
}

}  // namespace domain
}  // namespace simulation

#endif  // SIMULATION_DOMAIN_INTEGRATORS_INL
