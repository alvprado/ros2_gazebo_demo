#include "controller/domain/rate_limiter.hpp"

#include <algorithm>

namespace controller
{
namespace domain
{

RateLimiter::RateLimiter(double max_rate)
: max_rate_(max_rate)
{
}

double RateLimiter::step(double target, double dt)
{
  if (!current_.has_value()) {
    current_ = target;
    return target;
  }
  const double max_delta = max_rate_ * dt;
  current_ = std::clamp(target, *current_ - max_delta, *current_ + max_delta);
  return *current_;
}

void RateLimiter::reset()
{
  current_.reset();
}

}  // namespace domain
}  // namespace controller
