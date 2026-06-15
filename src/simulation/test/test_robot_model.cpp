#include <gtest/gtest.h>

#include "simulation/domain/robot_model.hpp"

using simulation::domain::RobotModelConfig;
using simulation::domain::RobotModel;
using simulation::domain::Velocity2D;

TEST(RobotModelTest, StepWithZeroCommandKeepsRobotAtRest)
{
  RobotModel model(RobotModelConfig{0.0});

  const auto state = model.step(Velocity2D{0.0, 0.0}, 1.0);
  EXPECT_DOUBLE_EQ(state.pose.x, 0.0);
  EXPECT_DOUBLE_EQ(state.pose.y, 0.0);
}

