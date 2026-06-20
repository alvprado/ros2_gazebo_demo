#ifndef SIMULATION_DOMAIN_TYPES_HPP
#define SIMULATION_DOMAIN_TYPES_HPP

namespace simulation
{
namespace domain
{

/// @brief Planar (2D) velocity
struct Velocity2D
{
  double linear_mps{0.0};
  double angular_radps{0.0};
};

/// @brief Planar (2D) pose.
struct Pose2D
{
  double x_m{0.0};
  double y_m{0.0};
  double theta_rad{0.0};
};

/// @brief Wheel rotation angles
/// @details Angles should be wrapped between -pi and pi
struct WheelAngles
{
  double left_rad{0.0};
  double right_rad{0.0};
};

/// @brief State of the robot
struct State
{
  Pose2D pose;
  Velocity2D velocity;
  WheelAngles wheel_angles;
};

}  // namespace domain
}  // namespace simulation

#endif  // SIMULATION_DOMAIN_TYPES_HPP
