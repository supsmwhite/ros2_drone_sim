#include <limits>
#include <stdexcept>

#include <gtest/gtest.h>

#include "drone_planning/turn_speed_policy.hpp"

namespace drone_planning
{

TEST(TurnSpeedPolicy, ClassifiesStraightMildAndSharpTurns)
{
  const TurnSpeedPolicyParameters parameters;
  const Eigen::Vector3d current = Eigen::Vector3d::Zero();
  EXPECT_DOUBLE_EQ(
    turn_speed_scale(
      Eigen::Vector3d(-1.0, 0.0, 0.0), current,
      Eigen::Vector3d(1.0, 0.0, 0.0), parameters),
    1.0);
  EXPECT_DOUBLE_EQ(
    turn_speed_scale(
      Eigen::Vector3d(-1.0, 0.0, 0.0), current,
      Eigen::Vector3d(1.0, 1.0, 0.0), parameters),
    parameters.mild_turn_scale);
  EXPECT_DOUBLE_EQ(
    turn_speed_scale(
      Eigen::Vector3d(-1.0, 0.0, 0.0), current,
      Eigen::Vector3d(0.0, 1.0, 0.0), parameters),
    parameters.sharp_turn_scale);
}

TEST(TurnSpeedPolicy, RejectsInvalidPolicyAndDegenerateSegments)
{
  TurnSpeedPolicyParameters parameters;
  parameters.sharp_turn_scale = parameters.mild_turn_scale;
  EXPECT_THROW(validate_turn_speed_policy(parameters), std::invalid_argument);

  parameters = TurnSpeedPolicyParameters{};
  EXPECT_THROW(
    turn_speed_scale(
      Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
      Eigen::Vector3d::UnitX(), parameters),
    std::invalid_argument);
  EXPECT_THROW(
    turn_speed_scale(
      Eigen::Vector3d(
        std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0),
      Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitX(), parameters),
    std::invalid_argument);
}

}  // namespace drone_planning
