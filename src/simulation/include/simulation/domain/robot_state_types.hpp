#ifndef SIMULATION_DOMAIN_ROBOT_STATE_TYPES_HPP
#define SIMULATION_DOMAIN_ROBOT_STATE_TYPES_HPP

#include <cmath>

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

/// @brief The time derivative of the State
struct StateDerivative
{
  /// @brief The time derivative of the pose
  struct
  {
    double x_dot_mps{0.0};
    double y_dot_mps{0.0};
    double theta_dot_radps{0.0};
  } pose;

  /// @brief The time derivative of the velocity
  struct
  {
    double linear_dot_mps2{0.0};
    double angular_dot_radps2{0.0};
  } velocity;

  /// @brief The time derivative of the wheel angles
  struct
  {
    double left_dot_radps{0.0};
    double right_dot_radps{0.0};
  } wheel_angles;
};

/// @brief Wrap an angle to (-pi, pi]
/// @param angle_rad The angle in radians to be wrapped
/// @returns A wrapped angle value in radians
inline double wrapAngle(double angle_rad) { return std::remainder(angle_rad, 2.0 * M_PI); }

/// @brief Operator overload for scalar multiplication of a state derivative with a time step value
/// @param state_derivative The time derivative of the state
/// @param dt_s The time step in seconds
/// @returns The state increment
inline State operator*(const StateDerivative& state_derivative, double dt_s)
{
  State delta;
  delta.pose.x_m = state_derivative.pose.x_dot_mps * dt_s;
  delta.pose.y_m = state_derivative.pose.y_dot_mps * dt_s;
  delta.pose.theta_rad = state_derivative.pose.theta_dot_radps * dt_s;
  delta.velocity.linear_mps = state_derivative.velocity.linear_dot_mps2 * dt_s;
  delta.velocity.angular_radps = state_derivative.velocity.angular_dot_radps2 * dt_s;
  delta.wheel_angles.left_rad = state_derivative.wheel_angles.left_dot_radps * dt_s;
  delta.wheel_angles.right_rad = state_derivative.wheel_angles.right_dot_radps * dt_s;
  return delta;
}

/// @brief Commutative scalar multiplication
inline State operator*(double dt_s, const StateDerivative& state_derivative)
{
  return state_derivative * dt_s;
}

/// @brief Add two States
/// @details Angles should be wrapped between -pi and pi
/// @param state_a The first state addend
/// @param state_b The second state addend
/// @returns The sum of the states
inline State operator+(const State& state_a, const State& state_b)
{
  State result;
  result.pose.x_m = state_a.pose.x_m + state_b.pose.x_m;
  result.pose.y_m = state_a.pose.y_m + state_b.pose.y_m;
  result.pose.theta_rad = wrapAngle(state_a.pose.theta_rad + state_b.pose.theta_rad);
  result.velocity.linear_mps = state_a.velocity.linear_mps + state_b.velocity.linear_mps;
  result.velocity.angular_radps = state_a.velocity.angular_radps + state_b.velocity.angular_radps;
  result.wheel_angles.left_rad =
    wrapAngle(state_a.wheel_angles.left_rad + state_b.wheel_angles.left_rad);
  result.wheel_angles.right_rad =
    wrapAngle(state_a.wheel_angles.right_rad + state_b.wheel_angles.right_rad);
  return result;
}

}  // namespace domain
}  // namespace simulation

#endif  // SIMULATION_DOMAIN_ROBOT_STATE_TYPES_HPP
