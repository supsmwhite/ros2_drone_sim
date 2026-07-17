#include <limits>
#include <optional>

#include <gtest/gtest.h>

#include <Eigen/Core>

#include "drone_planning/mission_failure_safety.hpp"

namespace drone_planning
{
namespace
{

TEST(MissionFailureSafety, GroundFailureNeverPublishesHold)
{
  EXPECT_FALSE(make_failure_hold_command(false, std::nullopt));
  EXPECT_FALSE(make_failure_hold_command(false, Eigen::Vector3d(2.0, 3.0, 1.5)));
}

TEST(MissionFailureSafety, AirborneFailureHoldsLatestSafePositionWithZeroDerivatives)
{
  const Eigen::Vector3d safe_position(2.1, -0.4, 1.8);
  const auto command = make_failure_hold_command(true, safe_position);
  ASSERT_TRUE(command);
  EXPECT_TRUE(command->position_world.isApprox(safe_position, 0.0));
  EXPECT_TRUE(command->velocity_world.isZero(0.0));
  EXPECT_TRUE(command->acceleration_world.isZero(0.0));
  EXPECT_FALSE(command->position_world.isApprox(Eigen::Vector3d::Zero(), 0.0));
}

TEST(MissionFailureSafety, AirborneFailureRejectsMissingOrInvalidSafePosition)
{
  EXPECT_FALSE(make_failure_hold_command(true, std::nullopt));
  const Eigen::Vector3d invalid(
    std::numeric_limits<double>::quiet_NaN(), 0.0, 1.5);
  EXPECT_FALSE(make_failure_hold_command(true, invalid));
}

}  // namespace
}  // namespace drone_planning
