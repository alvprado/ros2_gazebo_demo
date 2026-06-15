#ifndef SIMULATION_DOMAIN_TYPES_HPP
#define SIMULATION_DOMAIN_TYPES_HPP

namespace simulation
{
namespace domain
{

/// @brief Planar (2D) velocity
struct Velocity2D
{
  double linear{0.0};
  double angular{0.0};
};

/// @brief Planar (2D) pose.
struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double theta{0.0};
};

/// @brief State of the robot
struct State
{
  Pose2D pose;
  Velocity2D velocity;
};

}  // namespace domain
}  // namespace simulation

#endif  // SIMULATION_DOMAIN_TYPES_HPP
