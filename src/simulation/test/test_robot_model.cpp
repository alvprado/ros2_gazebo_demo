#include <gtest/gtest.h>

#include <cmath>

#include "simulation/domain/robot_model.hpp"

using simulation::domain::RobotModel;
using simulation::domain::RobotModelConfig;
using simulation::domain::SimulationErrorCodes;
using simulation::domain::Velocity2D;

static RobotModel make_model(double long_tc = 1e-6, double ang_tc = 1e-6)
{
  RobotModelConfig cfg;
  cfg.long_velocity_time_constant_s = long_tc;
  cfg.angular_velocity_time_constant_s = ang_tc;
  return RobotModel(cfg);
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

TEST(RobotModelTest, TinyTimeConstantLinearTracksCommandImmediately)
{
  // τ→0 → α→1 → output ≈ command after one step
  auto model = make_model(1e-6, 1e-6);
  const auto state = model.step(Velocity2D{1.0, 0.0}, 0.1);
  ASSERT_TRUE(state.has_value());
  EXPECT_NEAR(state.value().velocity.linear_mps, 1.0, 1e-4);
}

TEST(RobotModelTest, TinyTimeConstantAngularTracksCommandImmediately)
{
  auto model = make_model(1e-6, 1e-6);
  const auto state = model.step(Velocity2D{0.0, 1.0}, 0.1);
  ASSERT_TRUE(state.has_value());
  EXPECT_NEAR(state.value().velocity.angular_radps, 1.0, 1e-4);
}

TEST(RobotModelTest, AngularLagSlowsResponse)
{
  // τ_ang=1 → first step output < command
  auto model = make_model(1e-6, 1.0);
  const auto state = model.step(Velocity2D{0.0, 1.0}, 0.1);
  ASSERT_TRUE(state.has_value());
  EXPECT_LT(state.value().velocity.angular_radps, 1.0);
  EXPECT_GT(state.value().velocity.angular_radps, 0.0);
}

TEST(RobotModelTest, ForwardMotionIntegratesX)
{
  // heading=0, v≈1, dt=1 → x should advance by ~1
  auto model = make_model();
  const auto state = model.step(Velocity2D{1.0, 0.0}, 1.0);
  ASSERT_TRUE(state.has_value());
  EXPECT_NEAR(state.value().pose.x_m, 1.0, 1e-4);
  EXPECT_DOUBLE_EQ(state.value().pose.y_m, 0.0);
}

TEST(RobotModelTest, PureTurnIntegratesTheta)
{
  // v=0, ω≈1, dt=1 → theta should advance by ~1 rad, x/y unchanged
  auto model = make_model();
  const auto state = model.step(Velocity2D{0.0, 1.0}, 1.0);
  ASSERT_TRUE(state.has_value());
  EXPECT_NEAR(state.value().pose.theta_rad, 1.0, 1e-4);
  EXPECT_DOUBLE_EQ(state.value().pose.x_m, 0.0);
  EXPECT_DOUBLE_EQ(state.value().pose.y_m, 0.0);
}

TEST(RobotModelTest, ForwardMotionAfterTurnIntegratesY)
{
  // step 1: rotate 90°; step 2: drive forward 1 m → should advance in y
  auto model = make_model();
  model.step(Velocity2D{0.0, M_PI / 2.0}, 1.0);  // theta ≈ π/2 after step
  const auto state = model.step(Velocity2D{1.0, 0.0}, 1.0);
  ASSERT_TRUE(state.has_value());
  EXPECT_NEAR(state.value().pose.x_m, 0.0, 1e-4);
  EXPECT_NEAR(state.value().pose.y_m, 1.0, 1e-4);
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
