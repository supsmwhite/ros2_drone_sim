#include <cmath>
#include <limits>
#include <stdexcept>

#include <gtest/gtest.h>

#include "drone_planning/goal_completion_gate.hpp"

namespace drone_planning
{
namespace
{

constexpr double kDegreesToRadians = 3.14159265358979323846 / 180.0;

GoalCompletionGate gate()
{
  return GoalCompletionGate({0.20, 0.15, 0.10, 0.20, 1.0});
}

}  // namespace

TEST(GoalCompletionGateTest, UsesShortestYawErrorAcrossPiBoundary)
{
  EXPECT_NEAR(
    shortest_yaw_error(-179.0 * kDegreesToRadians, 179.0 * kDegreesToRadians),
    2.0 * kDegreesToRadians, 1.0e-12);
  EXPECT_NEAR(
    shortest_yaw_error(179.0 * kDegreesToRadians, -179.0 * kDegreesToRadians),
    -2.0 * kDegreesToRadians, 1.0e-12);
}

TEST(GoalCompletionGateTest, RejectsSatisfiedPositionWhenYawIsOutsideTolerance)
{
  auto completion = gate();
  const auto result = completion.update({0.01, 0.01, 0.20, 0.01}, 0.0, 2.0);
  EXPECT_FALSE(result.settled);
  EXPECT_FALSE(result.complete);
  EXPECT_DOUBLE_EQ(result.stable_duration, 0.0);
}

TEST(GoalCompletionGateTest, RejectsSatisfiedYawWhenAngularSpeedIsTooHigh)
{
  auto completion = gate();
  const auto result = completion.update({0.01, 0.01, 0.01, 0.25}, 0.0, 2.0);
  EXPECT_FALSE(result.settled);
  EXPECT_FALSE(result.complete);
}

TEST(GoalCompletionGateTest, RequiresAllConditionsForTheFullHoldDuration)
{
  auto completion = gate();
  const GoalCompletionSample settled{0.01, 0.01, 0.01, 0.01};
  EXPECT_FALSE(completion.update(settled, 0.0, 0.40).complete);
  EXPECT_FALSE(completion.update(settled, 0.0, 0.40).complete);
  const auto complete = completion.update(settled, 0.0, 0.20);
  EXPECT_TRUE(complete.settled);
  EXPECT_TRUE(complete.complete);
  EXPECT_NEAR(complete.stable_duration, 1.0, 1.0e-12);

  completion.reset();
  EXPECT_FALSE(completion.update(settled, 0.0, 0.99).complete);
  EXPECT_FALSE(completion.update({0.21, 0.01, 0.01, 0.01}, 0.0, 0.02).settled);
  EXPECT_FALSE(completion.update(settled, 0.0, 0.02).complete);
}

TEST(GoalCompletionGateTest, SelectsFixedOrMissionGoalAcceptanceYaw)
{
  EXPECT_DOUBLE_EQ(goal_acceptance_target_yaw(YawMode::Fixed, 0.35, -1.2), 0.35);
  EXPECT_DOUBLE_EQ(goal_acceptance_target_yaw(YawMode::PathTangent, 0.35, -1.2), -1.2);

  auto completion = gate();
  const GoalCompletionSample at_fixed_yaw{0.01, 0.01, 0.35, 0.01};
  EXPECT_TRUE(
    completion.update(
      at_fixed_yaw,
      goal_acceptance_target_yaw(YawMode::Fixed, 0.35, -1.2), 1.0).complete);
  completion.reset();
  EXPECT_FALSE(
    completion.update(
      at_fixed_yaw,
      goal_acceptance_target_yaw(YawMode::PathTangent, 0.35, -1.2), 1.0).settled);
  EXPECT_TRUE(
    completion.update(
      {0.01, 0.01, -1.2, 0.01},
      goal_acceptance_target_yaw(YawMode::PathTangent, 0.35, -1.2), 1.0).complete);
}

TEST(GoalCompletionGateTest, RejectsNonFiniteSamples)
{
  auto completion = gate();
  EXPECT_FALSE(
    completion.update(
      {0.01, 0.01, std::numeric_limits<double>::quiet_NaN(), 0.01}, 0.0, 1.0).settled);
  EXPECT_FALSE(
    completion.update(
      {0.01, 0.01, 0.0, std::numeric_limits<double>::infinity()}, 0.0, 1.0).settled);
}

TEST(GoalCompletionGateTest, RejectsNonPositiveOrNonFiniteTolerances)
{
  EXPECT_THROW(
    GoalCompletionGate({0.20, 0.15, 0.0, 0.20, 1.0}), std::invalid_argument);
  EXPECT_THROW(
    GoalCompletionGate(
      {0.20, 0.15, 0.10, std::numeric_limits<double>::infinity(), 1.0}),
    std::invalid_argument);
}

}  // namespace drone_planning
