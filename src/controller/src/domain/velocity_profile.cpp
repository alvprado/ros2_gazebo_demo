#include "controller/domain/velocity_profile.hpp"

#include <cmath>
#include <functional>
#include <numeric>

namespace controller
{
namespace domain
{

VelocityProfile::VelocityProfile(std::vector<VelocityStep> steps, bool loop)
: steps_(std::move(steps)), loop_(loop)
{
  total_duration_s_ = std::transform_reduce(
    steps_.begin(), steps_.end(), 0.0,
    std::plus<double>{},
    [](const VelocityStep & step) { return step.duration_s; });
}

double VelocityProfile::step(double time_s)
{
  if (finished_) {
    return 0.0;
  }

  if (!start_time_s_.has_value()) {
    start_time_s_ = time_s;
  }

  double elapsed_time_s = time_s - start_time_s_.value();

  if (!loop_ && elapsed_time_s >= total_duration_s_) {
    finished_ = true;
    return 0.0;
  }

  if (loop_) {
    elapsed_time_s = std::fmod(elapsed_time_s, total_duration_s_);
  }

  double cumulative_time_s{0.0};
  for (const auto & step : steps_) {
    cumulative_time_s += step.duration_s;
    if (elapsed_time_s < cumulative_time_s) {
      return step.velocity_mps;
    }
  }

  return steps_.back().velocity_mps;
}

bool VelocityProfile::isFinished() const
{
  return finished_;
}

}  // namespace domain
}  // namespace controller
