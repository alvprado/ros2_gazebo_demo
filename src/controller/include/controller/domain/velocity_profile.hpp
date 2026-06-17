#ifndef CONTROLLER_DOMAIN_VELOCITY_PROFILE_HPP
#define CONTROLLER_DOMAIN_VELOCITY_PROFILE_HPP

#include <optional>
#include <vector>

namespace controller
{
namespace domain
{

/// @brief A bundle of a velocity command with a duration
struct VelocityStep
{
  double velocity_mps{0.0};
  double duration_s{0.0};
};

/// @brief Stepwise constant velocity profile
class VelocityProfile
{
public:
  /// @brief Constructs a velocity profile
  /// @param steps A sequence of velocity steps (velocity command and duration)
  /// @param loop Flag for looping mode - if true, the profile restarts from the beginning once the total duration elapses
  VelocityProfile(std::vector<VelocityStep> steps, bool loop);

  /// @brief Look up the target velocity for the given absolute time
  /// @param time_s absolute current time in seconds
  /// @return target velocity in m/s, or 0.0 once the profile completes in non-looping mode
  /// @note In looping mode the profile restarts from the beginning once the total duration elapses
  double step(double time_s);

  /// @brief Gets the finish state
  /// @return The finish state
  bool isFinished() const;

private:
  // Sequence of velocity commands with duration that describe the velocity profile
  std::vector<VelocityStep> steps_;

  // The total duration of the velocity profile
  double total_duration_s_{0.0};

  // Start time of the velocity profile
  std::optional<double> start_time_s_;

  // Flag for looping mode - velocity profile restarts from the beginning after the total duration elapses
  bool loop_;

  // State for indicating that the velocity profile is finished
  bool finished_{false};
};

}  // namespace domain
}  // namespace controller

#endif  // CONTROLLER_DOMAIN_VELOCITY_PROFILE_HPP
