#ifndef CONTROLLER_DOMAIN_TARGETS_GENERATOR_HPP
#define CONTROLLER_DOMAIN_TARGETS_GENERATOR_HPP

#include <optional>
#include <vector>

namespace controller
{
namespace domain
{

/// @brief A target value paired with a duration
struct TimedTarget
{
  double value{0.0};
  double duration_s{0.0};
};

/// @brief Stepwise constant target generator
class TargetsGenerator
{
public:
  /// @brief Constructs a targets generator
  /// @param targets A sequence of target values with their durations
  /// @param loop    If true the profile restarts from the beginning once the total duration elapses
  TargetsGenerator(std::vector<TimedTarget> targets, bool loop);

  /// @brief Look up the target value for the given absolute time
  /// @param time_s Absolute current time in seconds
  /// @return Target value, or 0.0 once the profile completes in non-looping mode
  double step(double time_s);

  /// @brief Gets the finish state
  /// @return True once the profile has completed in non-looping mode
  bool isFinished() const;

private:
  std::vector<TimedTarget> targets_;
  double total_duration_s_{0.0};
  std::optional<double> start_time_s_;
  bool loop_;
  bool finished_{false};
};

}  // namespace domain
}  // namespace controller

#endif  // CONTROLLER_DOMAIN_TARGETS_GENERATOR_HPP
