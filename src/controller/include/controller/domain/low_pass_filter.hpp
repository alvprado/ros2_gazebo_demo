#ifndef CONTROLLER_DOMAIN_LOW_PASS_FILTER_HPP
#define CONTROLLER_DOMAIN_LOW_PASS_FILTER_HPP

#include <cstdint>
#include <expected>
#include <optional>

namespace controller
{
namespace domain
{

enum class LowPassFilterErrorCodes : std::uint8_t
{
  NonPositiveFrequency = 0U,
  NonPositiveTimeStep = 1U,
};

/// @brief First-order IIR low-pass filter
class LowPassFilter
{
public:
  /// @brief Construct an IIR low-pass filter
  /// @param cutoff_frequency_hz cutoff frequency in Hz
  explicit LowPassFilter(double cutoff_frequency_hz);

  /// @param input Raw input sample
  /// @param dt Time step in seconds
  /// @details The filter law is y[k] = alpha * input + (1 - alpha) * y[k-1]
  ///          where alpha = dt / (tau + dt) and tau = 1 / (2 * pi * cutoff_frequency_hz)
  /// @returns An expected filtered output or an error code
  std::expected<double, LowPassFilterErrorCodes> step(double input, double dt_s);

private:
  double cutoff_frequency_hz_;
  std::optional<double> state_;
};

}  // namespace domain
}  // namespace controller

#endif  // CONTROLLER_DOMAIN_LOW_PASS_FILTER_HPP
