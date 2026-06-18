#ifndef CONTROLLER_DOMAIN_RATE_LIMITER_HPP
#define CONTROLLER_DOMAIN_RATE_LIMITER_HPP

#include <optional>

namespace controller
{
namespace domain
{

/// Limits the rate of change of a signal to ±max_rate units/second.
/// On the first call the output is seeded from the input so there is no
/// initial ramp from zero.
class RateLimiter
{
public:
  explicit RateLimiter(double max_rate);

  /// @param target  Desired value
  /// @param dt      Time step in seconds
  /// @returns       Rate-limited value
  double step(double target, double dt);

  void reset();

private:
  double max_rate_;
  std::optional<double> current_;
};

}  // namespace domain
}  // namespace controller

#endif  // CONTROLLER_DOMAIN_RATE_LIMITER_HPP
