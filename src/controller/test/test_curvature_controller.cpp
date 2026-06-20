#include <gtest/gtest.h>

#include "controller/domain/curvature_controller.hpp"
#include "controller/domain/low_pass_filter.hpp"
#include "controller/domain/pid_controller.hpp"

using controller::domain::CurvatureController;
using controller::domain::CurvatureControllerConfig;
using controller::domain::CurvatureControllerErrorCodes;
using controller::domain::LowPassFilter;
using controller::domain::PidConfig;
using controller::domain::PidController;

using TestPid = PidController<LowPassFilter, LowPassFilter>;
using TestCurv = CurvatureController<TestPid>;

static TestCurv make_curv(double threshold = 0.1, bool spin = false, double spin_vel = 0.0,
                          double kp = 1.0)
{
  CurvatureControllerConfig cfg;
  cfg.lower_long_velocity_threshold_mps = threshold;
  cfg.spin_in_place = spin;
  cfg.spin_velocity_radps = spin_vel;

  PidConfig pid_cfg;
  pid_cfg.kp = kp;

  return TestCurv(TestPid(pid_cfg, LowPassFilter(100.0), LowPassFilter(100.0)), cfg);
}

// ── Error codes ───────────────────────────────────────────────────────────────

TEST(CurvatureControllerTest, InvalidTimeStep)
{
  auto ctrl = make_curv();
  auto result = ctrl.step(0.5, 0.0, 0.5, 0.0);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error(), CurvatureControllerErrorCodes::InvalidTimeStep);
}

TEST(CurvatureControllerTest, ZeroTimeStepIsInvalid)
{
  auto ctrl = make_curv();
  auto result = ctrl.step(0.5, 0.0, 0.5, 0.0);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error(), CurvatureControllerErrorCodes::InvalidTimeStep);
}

// ── Curvature mode (|v_long| >= threshold) ───────────────────────────────────

TEST(CurvatureControllerTest, AboveThresholdUsesKappaConversion)
{
  // kp=1, κ=0.5, v_long=1.0 → ω_setpoint=0.5, current_ω=0 → error=0.5 → output=0.5
  auto ctrl = make_curv(0.1, false, 0.0, 1.0);
  auto result = ctrl.step(0.5, 0.0, 1.0, 0.1);
  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(*result, 0.5, 1e-9);
}

TEST(CurvatureControllerTest, ZeroCurvatureGoesForwardStraight)
{
  // κ=0, v_long=1.0 → ω_setpoint=0, current_ω=0 → output=0
  auto ctrl = make_curv();
  auto result = ctrl.step(0.0, 0.0, 1.0, 0.1);
  ASSERT_TRUE(result.has_value());
  EXPECT_DOUBLE_EQ(*result, 0.0);
}

TEST(CurvatureControllerTest, NegativeCurvatureRightTurn)
{
  // κ=-0.5, v_long=1.0 → ω_setpoint=-0.5 → negative output
  auto ctrl = make_curv(0.1, false, 0.0, 1.0);
  auto result = ctrl.step(-0.5, 0.0, 1.0, 0.1);
  ASSERT_TRUE(result.has_value());
  EXPECT_LT(*result, 0.0);
}

TEST(CurvatureControllerTest, ThresholdBoundaryUsesKappa)
{
  // |v_long| exactly at threshold → curvature mode, not spin mode
  auto ctrl = make_curv(0.1, true, 99.0, 1.0);
  auto result = ctrl.step(0.5, 0.0, 0.1, 0.1);
  ASSERT_TRUE(result.has_value());
  // ω_setpoint = 0.5 * 0.1 = 0.05; output << spin_vel 99.0
  EXPECT_LT(std::abs(*result), 1.0);
}

// ── Low-speed / spin-in-place mode (|v_long| < threshold) ───────────────────

TEST(CurvatureControllerTest, BelowThresholdSpinInPlaceActive)
{
  // v_long=0.0 < 0.1 threshold, spin_in_place=true, spin_vel=0.5
  // PID: setpoint=0.5, current=0.0 → error=0.5 → kp*error=0.5
  auto ctrl = make_curv(0.1, true, 0.5, 1.0);
  auto result = ctrl.step(2.0, 0.0, 0.0, 0.1);
  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(*result, 0.5, 1e-9);
}

TEST(CurvatureControllerTest, BelowThresholdSpinInPlaceInactive)
{
  // v_long=0.0 < threshold, spin_in_place=false → ω_setpoint=0 → output=0
  auto ctrl = make_curv(0.1, false, 0.5, 1.0);
  auto result = ctrl.step(2.0, 0.0, 0.0, 0.1);
  ASSERT_TRUE(result.has_value());
  EXPECT_DOUBLE_EQ(*result, 0.0);
}

TEST(CurvatureControllerTest, NegativeLongitudinalVelocityAlsoGatesOnThreshold)
{
  // |v_long|=0.05 < threshold → low-speed branch, spin inactive → 0
  auto ctrl = make_curv(0.1, false, 0.0);
  auto result = ctrl.step(1.0, 0.0, -0.05, 0.1);
  ASSERT_TRUE(result.has_value());
  EXPECT_DOUBLE_EQ(*result, 0.0);
}
