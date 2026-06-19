#ifndef CONTROLLER_DOMAIN_PID_CONTROLLER_HPP
#define CONTROLLER_DOMAIN_PID_CONTROLLER_HPP

#include <expected>
#include <limits>
#include <ostream>
#include <string_view>

#include "controller/domain/low_pass_filter.hpp"

namespace controller
{
namespace domain
{

/// @brief PID controller config
struct PidConfig
{
  // PID gains
  double kp{0.0};
  double ki{0.0};
  double kd{0.0};
  // Feedforward gain
  double kff{0.0};
  // Controller limits
  double min_output{-std::numeric_limits<double>::infinity()};
  double max_output{std::numeric_limits<double>::infinity()};
};

/// @brief PID states
struct PidState
{
  // Integral term
  double integral_{0.0};
  // Previous error
  double previous_error_{0.0};
};

/// @brief PID error codes
enum class PidErrorCodes : std::uint8_t
{
  NonPositiveGains = 0U,
  InvalidLimits = 1U,
  FailedFilter = 2U,
  InvalidTimeStep = 3U,
};

inline std::string_view toString(PidErrorCodes code)
{
  switch (code)
  {
    case PidErrorCodes::NonPositiveGains:
      return "NonPositiveGains";
    case PidErrorCodes::InvalidLimits:
      return "InvalidLimits";
    case PidErrorCodes::FailedFilter:
      return "FailedFilter";
    case PidErrorCodes::InvalidTimeStep:
      return "InvalidTimeStep";
  }
  return "Unknown";
}

/// @brief PID controller with feedforward
/// @tparam SetpointFilter_T Filter for target values
/// @tparam DerivativeFilter_T Derivative term filter
template <typename SetpointFilter_T, typename DerivativeFilter_T>
class PidController
{
public:
  /// @brief Constructs a PID controller from its config
  /// @param config Config containing gains, filter frequencies and limits
  explicit PidController(const PidConfig& config, const SetpointFilter_T& setpoint_filter,
                         const DerivativeFilter_T& derivative_filter);

  /// @brief Step the PID controller
  /// @param setpoint  Desired value
  /// @param current   Current measured value
  /// @param dt        Time step in seconds
  /// @return          Control command or an error code
  std::expected<double, PidErrorCodes> step(double setpoint, double current, double dt_s);

private:
  PidConfig config_;
  PidState state_;
  SetpointFilter_T setpoint_filter_;
  DerivativeFilter_T derivative_filter_;
};

}  // namespace domain
}  // namespace controller

#include "controller/domain/pid_controller.inl"

#endif  // CONTROLLER_DOMAIN_PID_CONTROLLER_HPP
