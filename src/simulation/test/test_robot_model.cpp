#include <gtest/gtest.h>

#include <cmath>

#include "simulation/domain/integrators.hpp"
#include "simulation/domain/robot_model.hpp"

using simulation::domain::forwardEulerStep;
using simulation::domain::RobotModel;
using simulation::domain::RobotModelConfig;
using simulation::domain::SimulationErrorCodes;
using simulation::domain::State;
using simulation::domain::Velocity2D;

// Default tau=1.0 so that with dt=1.0, Euler gives exactly: v_new = v_cmd in one step.
static RobotModel make_model(double long_tc = 1.0, double ang_tc = 1.0)
{
  RobotModelConfig cfg;
  cfg.long_velocity_time_constant_s = long_tc;
  cfg.angular_velocity_time_constant_s = ang_tc;
  return RobotModel([](const State& state, const Velocity2D& velocity_cmd,
                       const RobotModel::DerivativeFunction& derivative_fn, double dt_s)
                    { return forwardEulerStep(state, velocity_cmd, derivative_fn, dt_s); },
                    cfg);
}

TEST(RobotModelTest, StepWithZeroCommandKeepsRobotAtRest)
{
  auto model = make_model();
  const auto state = model.step(Velocity2D{0.0, 0.0}, 1.0);
  ASSERT_TRUE(state.has_value());
  EXPECT_DOUBLE_EQ(state.value().pose.x_m, 0.0);
  EXPECT_DOUBLE_EQ(state.value().pose.y_m, 0.0);
  EXPECT_DOUBLE_EQ(state.value().pose.theta_rad, 0.0);
}

// With Euler and tau = dt, v_new = v_prev + (v_cmd - v_prev)/tau * dt = v_cmd exactly.
TEST(RobotModelTest, LinearVelocityReachesCommandWhenTauEqualsTimeStep)
{
  auto model = make_model(1.0, 1.0);
  const auto state = model.step(Velocity2D{1.0, 0.0}, 1.0);
  ASSERT_TRUE(state.has_value());
  EXPECT_NEAR(state.value().velocity.linear_mps, 1.0, 1e-9);
}

TEST(RobotModelTest, AngularVelocityReachesCommandWhenTauEqualsTimeStep)
{
  auto model = make_model(1.0, 1.0);
  const auto state = model.step(Velocity2D{0.0, 1.0}, 1.0);
  ASSERT_TRUE(state.has_value());
  EXPECT_NEAR(state.value().velocity.angular_radps, 1.0, 1e-9);
}

TEST(RobotModelTest, AngularLagSlowsResponse)
{
  // tau_ang=1 > dt=0.1 -> first step output < command
  auto model = make_model(1.0, 1.0);
  const auto state = model.step(Velocity2D{0.0, 1.0}, 0.1);
  ASSERT_TRUE(state.has_value());
  EXPECT_LT(state.value().velocity.angular_radps, 1.0);
  EXPECT_GT(state.value().velocity.angular_radps, 0.0);
}

// Euler uses v_prev for pose: step 1 warms up velocity, step 2 accumulates pose.
TEST(RobotModelTest, ForwardMotionIntegratesX)
{
  auto model = make_model();
  model.step(Velocity2D{1.0, 0.0}, 1.0);                     // v → 1.0, pose unchanged
  const auto state = model.step(Velocity2D{1.0, 0.0}, 1.0);  // x += v_prev * dt = 1.0
  ASSERT_TRUE(state.has_value());
  EXPECT_NEAR(state.value().pose.x_m, 1.0, 1e-9);
  EXPECT_NEAR(state.value().pose.y_m, 0.0, 1e-9);
}

TEST(RobotModelTest, PureTurnIntegratesTheta)
{
  auto model = make_model();
  model.step(Velocity2D{0.0, 1.0}, 1.0);                     // omega → 1.0, theta unchanged
  const auto state = model.step(Velocity2D{0.0, 1.0}, 1.0);  // theta += omega_prev * dt = 1.0
  ASSERT_TRUE(state.has_value());
  EXPECT_NEAR(state.value().pose.theta_rad, 1.0, 1e-9);
  EXPECT_NEAR(state.value().pose.x_m, 0.0, 1e-9);
  EXPECT_NEAR(state.value().pose.y_m, 0.0, 1e-9);
}

TEST(RobotModelTest, ForwardMotionAfterTurnIntegratesY)
{
  // 4 steps: accelerate angular, stop angular (theta reaches pi/2), accelerate linear, drive
  auto model = make_model();
  model.step(Velocity2D{0.0, M_PI / 2.0}, 1.0);  // omega -> pi/2, theta=0 (uses omega_prev=0)
  model.step(Velocity2D{0.0, 0.0}, 1.0);         // omega -> 0, theta -> pi/2 (uses omega_prev=pi/2)
  model.step(Velocity2D{1.0, 0.0}, 1.0);         // v -> 1.0, theta=pi/2, pose unchanged (v_prev=0)
  const auto state = model.step(Velocity2D{1.0, 0.0}, 1.0);  // y += v_prev*sin(pi/2)*dt = 1.0
  ASSERT_TRUE(state.has_value());
  EXPECT_NEAR(state.value().pose.y_m, 1.0, 1e-9);
  EXPECT_NEAR(state.value().pose.x_m, 0.0, 1e-6);  // cos(pi/2) -> 0
}

TEST(RobotModelTest, NonPositiveTimeConstantReturnsError)
{
  auto model = make_model(0.0, 0.1);
  const auto result = model.step(Velocity2D{1.0, 0.0}, 0.1);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error(), SimulationErrorCodes::NonPositiveTimeConstant);
}

TEST(RobotModelTest, InvalidTimeStepReturnsError)
{
  auto model = make_model();
  const auto result = model.step(Velocity2D{1.0, 0.0}, 0.0);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error(), SimulationErrorCodes::InvalidTimeStep);
}

TEST(RobotModelTest, NegativeDtReturnsError)
{
  auto model = make_model();
  const auto result = model.step(Velocity2D{1.0, 0.0}, -1.0);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error(), SimulationErrorCodes::InvalidTimeStep);
}

// Euler uses v_prev for wheel integration: step 1 warms up velocity, step 2 spins wheels.
TEST(RobotModelTest, ForwardMotionSpinsWheelsSymmetrically)
{
  auto model = make_model();
  model.step(Velocity2D{1.0, 0.0}, 1.0);                     // v -> 1.0, wheels unchanged
  const auto state = model.step(Velocity2D{1.0, 0.0}, 1.0);  // both wheels spin forward
  ASSERT_TRUE(state.has_value());
  EXPECT_NEAR(state.value().wheel_angles.left_rad, state.value().wheel_angles.right_rad, 1e-9);
  EXPECT_GT(state.value().wheel_angles.left_rad, 0.0);
}

TEST(RobotModelTest, PureTurnSpinsWheelsOppositely)
{
  auto model = make_model();
  model.step(Velocity2D{0.0, 1.0}, 1.0);                     // omega -> 1.0, wheels unchanged
  const auto state = model.step(Velocity2D{0.0, 1.0}, 1.0);  // left backward, right forward
  ASSERT_TRUE(state.has_value());
  EXPECT_LT(state.value().wheel_angles.left_rad, 0.0);
  EXPECT_GT(state.value().wheel_angles.right_rad, 0.0);
}

TEST(RobotModelTest, InvalidWheelConfigReturnsError)
{
  RobotModelConfig cfg;
  cfg.long_velocity_time_constant_s = 1.0;
  cfg.angular_velocity_time_constant_s = 1.0;
  cfg.wheel_radius_m = 0.0;  // invalid
  RobotModel model([](const State& state, const Velocity2D& velocity_cmd,
                      const RobotModel::DerivativeFunction& derivative_fn, double dt_s)
                   { return forwardEulerStep(state, velocity_cmd, derivative_fn, dt_s); },
                   cfg);
  const auto result = model.step(Velocity2D{1.0, 0.0}, 0.1);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error(), SimulationErrorCodes::InvalidWheelConfig);
}
