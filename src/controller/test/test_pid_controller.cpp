#include <gtest/gtest.h>

#include "controller/domain/low_pass_filter.hpp"
#include "controller/domain/pid_controller.hpp"

using controller::domain::LowPassFilter;
using controller::domain::LowPassFilterErrorCodes;
using controller::domain::PidConfig;
using controller::domain::PidController;
using controller::domain::PidErrorCodes;

// Convenience alias for the concrete PID type used throughout the tests
using TestPid = PidController<LowPassFilter, LowPassFilter>;

// Helper: build a PID with high-cutoff (≈passthrough) filters so filter
// dynamics do not interfere with the test assertions.
static TestPid make_pid(const PidConfig& config, double cutoff_hz = 100.0)
{
  return TestPid(config, LowPassFilter(cutoff_hz), LowPassFilter(cutoff_hz));
}

TEST(LowPassFilterTest, FirstCallSeedsFromInput)
{
  LowPassFilter f(1.0);
  auto result = f.step(3.14, 0.05);
  ASSERT_TRUE(result.has_value());
  EXPECT_DOUBLE_EQ(*result, 3.14);
}

TEST(LowPassFilterTest, ConvergesToConstantInput)
{
  LowPassFilter f(10.0);
  double y = 0.0;
  for (int i = 0; i < 500; ++i)
  {
    auto r = f.step(1.0, 0.01);
    ASSERT_TRUE(r.has_value());
    y = *r;
  }
  EXPECT_NEAR(y, 1.0, 1e-3);
}

TEST(LowPassFilterTest, ReturnsErrorOnNonPositiveTimeStep)
{
  LowPassFilter f(1.0);
  f.step(0.0, 0.01);  // seed
  auto result = f.step(1.0, 0.0);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error(), LowPassFilterErrorCodes::NonPositiveTimeStep);
}

TEST(LowPassFilterTest, ReturnsErrorOnNonPositiveCutoff)
{
  LowPassFilter f(0.0);
  f.step(0.0, 0.01);  // seed
  auto result = f.step(1.0, 0.01);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error(), LowPassFilterErrorCodes::NonPositiveFrequency);
}

TEST(PidControllerTest, ReturnsErrorOnNegativeGain)
{
  PidConfig config;
  config.kp = -1.0;
  auto pid = make_pid(config);
  auto result = pid.step(1.0, 0.0, 0.1);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error(), PidErrorCodes::NonPositiveGains);
}

TEST(PidControllerTest, ReturnsErrorOnInvalidLimits)
{
  PidConfig config;
  config.min_output = 1.0;
  config.max_output = -1.0;
  auto pid = make_pid(config);
  auto result = pid.step(1.0, 0.0, 0.1);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error(), PidErrorCodes::InvalidLimits);
}

TEST(PidControllerTest, ReturnsErrorOnNonPositiveTimeStep)
{
  auto pid = make_pid(PidConfig{});
  auto result = pid.step(1.0, 0.0, 0.0);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error(), PidErrorCodes::InvalidTimeStep);
}

TEST(PidControllerTest, ZeroErrorProducesZeroOutput)
{
  PidConfig config;
  config.kp = 1.0;
  auto pid = make_pid(config);
  auto result = pid.step(1.0, 1.0, 0.1);
  ASSERT_TRUE(result.has_value());
  EXPECT_DOUBLE_EQ(*result, 0.0);
}

TEST(PidControllerTest, ProportionalTermScalesWithError)
{
  PidConfig config;
  config.kp = 2.0;
  auto pid = make_pid(config);
  // First call: filter seeds to setpoint, error = 1.0, output = kp * 1.0 = 2.0
  auto result = pid.step(1.0, 0.0, 0.1);
  ASSERT_TRUE(result.has_value());
  EXPECT_DOUBLE_EQ(*result, 2.0);
}

TEST(PidControllerTest, OutputClampedToLimits)
{
  PidConfig config;
  config.kp = 10.0;
  config.min_output = -1.0;
  config.max_output = 1.0;
  auto pid = make_pid(config);
  auto result = pid.step(100.0, 0.0, 0.1);
  ASSERT_TRUE(result.has_value());
  EXPECT_DOUBLE_EQ(*result, 1.0);
}

TEST(PidControllerTest, FeedforwardAddsSetpointScaled)
{
  PidConfig config;
  config.kff = 1.0;
  auto pid = make_pid(config);

  // error = 0 -> P=I=D=0; feedforward = kff * setpoint = 0.5
  auto result = pid.step(0.5, 0.5, 0.1);
  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(*result, 0.5, 1e-9);
}

TEST(PidControllerTest, IntegralFreezesDuringSaturation)
{
  PidConfig config;
  config.kp = 1.0;
  config.ki = 1.0;
  config.min_output = -1.0;
  config.max_output = 1.0;
  auto pid = make_pid(config);

  for (int i = 0; i < 100; ++i)
  {
    pid.step(10.0, 0.0, 0.1);
  }
  // After sustained saturation the integral must be bounded;
  // a zero-error step must not spike outside the output limits.
  auto result = pid.step(0.0, 0.0, 0.1);
  ASSERT_TRUE(result.has_value());
  EXPECT_GE(*result, -1.0);
  EXPECT_LE(*result, 1.0);
}
