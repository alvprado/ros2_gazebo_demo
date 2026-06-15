#include <gtest/gtest.h>

#include "controller/domain/pid_controller.hpp"

using controller::domain::PidController;
using controller::domain::PidGains;

TEST(PidControllerTest, ZeroErrorProducesZeroOutput)
{
  PidController pid(PidGains{1.0, 0.0, 0.0});

  // TODO: once compute() implements the PID law, a zero error
  // (setpoint == measurement) should yield zero output.
  EXPECT_DOUBLE_EQ(pid.step(1.0, 1.0, 0.1), 0.0);
}

TEST(PidControllerTest, ProportionalTermScalesWithError)
{
  PidController pid(PidGains{2.0, 0.0, 0.0});

  // TODO: once compute() implements the PID law, with ki = kd = 0,
  // output should equal kp * (setpoint - measurement).
  EXPECT_DOUBLE_EQ(pid.step(1.0, 0.0, 0.1), 0.0);
}
