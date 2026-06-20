#ifndef CONTROLLER_DOMAIN_CURVATURE_CONTROLLER_HPP
#define CONTROLLER_DOMAIN_CURVATURE_CONTROLLER_HPP

#include <cstdint>
#include <expected>
#include <string_view>

namespace controller
{
namespace domain
{

/// @brief Curvature controller config
struct CurvatureControllerConfig
{
  // Longitudinal velocity threshold below which spin-in-place mode activates
  double lower_long_velocity_threshold_mps{0.1};
  // Spin the robot in place when |v_long| < threshold
  bool spin_in_place{false};
  // Angular velocity used when spinning in place [rad/s]
  double spin_velocity_radps{0.0};
};

/// @brief Curvature controller error codes
enum class CurvatureControllerErrorCodes : std::uint8_t
{
  InvalidTimeStep = 0U,
  FailedControlLaw = 1U,
};

/// @brief Map error codes to a string view
/// @param code curvature controller error code
/// @return a string view of the error code for logging purposes
inline std::string_view toString(CurvatureControllerErrorCodes code)
{
  switch (code) {
    case CurvatureControllerErrorCodes::InvalidTimeStep:
      return "InvalidTimeStep";
    case CurvatureControllerErrorCodes::FailedControlLaw:
      return "FailedControlLaw";
  }
  return "Unknown";
}

/// @brief Controls the target curvature with an angular velocity command
/// @tparam ControlLaw_T Control law that takes a target, a current and a time step value
template<typename ControlLaw_T>
class CurvatureController
{
public:
  /// @brief Constructs a curvature controller
  /// @param control_law Control law that takes a target, a current and a time step value
  /// @param config     Config with threshold and spin-in-place settings
  explicit CurvatureController(
    const ControlLaw_T & control_law,
    const CurvatureControllerConfig & config);

  /// @brief Step the curvature controller
  /// @param target_curvature             Desired curvature in one per meter
  /// @param current_angular_velocity     Measured angular velocity in radps
  /// @param current_longitudinal_velocity Measured longitudinal velocity in mps
  /// @param dt_s                         Time step in seconds
  /// @return Angular velocity command in radps or an error code
  std::expected<double, CurvatureControllerErrorCodes> step(
    double target_curvature_pm, double current_angular_velocity_radps,
    double current_longitudinal_velocity_mps, double dt_s);

private:
  ControlLaw_T control_law_;
  CurvatureControllerConfig config_;
};

}  // namespace domain
}  // namespace controller

#include "controller/domain/curvature_controller.inl"

#endif  // CONTROLLER_DOMAIN_CURVATURE_CONTROLLER_HPP
