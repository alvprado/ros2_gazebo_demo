#include "controller/domain/low_pass_filter.hpp"

#include <cmath>

namespace controller
{
namespace domain
{

LowPassFilter::LowPassFilter(double cutoff_frequency_hz) : cutoff_frequency_hz_(cutoff_frequency_hz)
{
}

std::expected<double, LowPassFilterErrorCodes> LowPassFilter::step(double input, double dt_s)
{
  if (!state_.has_value())
  {
    state_ = input;
    return input;
  }

  if (cutoff_frequency_hz_ <= 0)
  {
    return std::unexpected(LowPassFilterErrorCodes::NonPositiveFrequency);
  }

  if (dt_s <= 0)
  {
    return std::unexpected(LowPassFilterErrorCodes::NonPositiveTimeStep);
  }

  const double tau = 1.0 / (2.0 * M_PI * cutoff_frequency_hz_);
  const double alpha = dt_s / (tau + dt_s);

  state_.value() = alpha * input + (1.0 - alpha) * state_.value();
  return state_.value();
}

}  // namespace domain
}  // namespace controller
