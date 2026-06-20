#include "controller/domain/targets_generator.hpp"

#include <cmath>
#include <functional>
#include <numeric>

namespace controller
{
namespace domain
{

TargetsGenerator::TargetsGenerator(std::vector<TimedTarget> targets, bool loop)
: targets_(std::move(targets)), loop_(loop)
{
  total_duration_s_ =
    std::transform_reduce(targets_.begin(), targets_.end(), 0.0, std::plus<double>{},
      [](const TimedTarget & target) {return target.duration_s;});
}

double TargetsGenerator::step(double time_s)
{
  // If maneuver is finished, return a zero target value
  if (finished_) {
    return 0.0;
  }

  // Fill start time with first incoming timestamp
  if (!start_time_s_.has_value()) {
    start_time_s_ = time_s;
  }

  double elapsed_time_s = time_s - start_time_s_.value();

  // If no loop and elapsed time is greater than total duration, set finish flag to true and return
  // a zero command
  if (!loop_ && elapsed_time_s >= total_duration_s_) {
    finished_ = true;
    return 0.0;
  }

  // If looping mode is active, wrap elapsed time to total duration
  if (loop_) {
    elapsed_time_s = std::fmod(elapsed_time_s, total_duration_s_);
  }

  // Search for step on which the current time falls into and return the corresponding target value
  double cumulative_time_s{0.0};
  for (const auto & [target_value, duration] : targets_) {
    cumulative_time_s += duration;
    if (elapsed_time_s < cumulative_time_s) {
      return target_value;
    }
  }

  return targets_.back().value;
}

bool TargetsGenerator::isFinished() const {return finished_;}

}  // namespace domain
}  // namespace controller
