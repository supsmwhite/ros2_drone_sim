#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

#include "drone_mission/goal_input.hpp"

namespace drone_mission
{

TEST(GoalInput, ParsesFiniteNumbersAndRejectsMalformedValues)
{
  const auto values = parse_finite_numbers({"2.0", "-1", "1.5", "0.25"});
  ASSERT_EQ(values.size(), 4U);
  EXPECT_DOUBLE_EQ(values[0], 2.0);
  EXPECT_THROW(parse_finite_numbers({"2oops"}), std::invalid_argument);
  EXPECT_THROW(parse_finite_numbers({"nan"}), std::invalid_argument);
  EXPECT_THROW(parse_finite_numbers({"inf"}), std::invalid_argument);
}

TEST(GoalInput, BuildsYawQuaternionAndChecksWorkspace)
{
  GoalConstraints constraints{-1.0, 3.0, -2.0, 2.0, 0.5, 5.0};
  const auto pose = make_goal_pose(2.0, 1.0, 1.5, 1.0, constraints);
  EXPECT_NEAR(pose.orientation.z, std::sin(0.5), 1.0e-12);
  EXPECT_NEAR(pose.orientation.w, std::cos(0.5), 1.0e-12);
  EXPECT_THROW(make_goal_pose(4.0, 0.0, 1.0, 0.0, constraints), std::out_of_range);
  EXPECT_THROW(make_goal_pose(0.0, 0.0, 0.4, 0.0, constraints), std::out_of_range);
  EXPECT_THROW(
    make_goal_pose(0.0, 0.0, 1.0, std::numeric_limits<double>::infinity(), constraints),
    std::invalid_argument);
}

TEST(GoalInput, MultiRequiresCompleteNonEmptyGroups)
{
  GoalConstraints constraints;
  EXPECT_THROW(make_goal_poses({}, constraints), std::invalid_argument);
  EXPECT_THROW(make_goal_poses({0.0, 0.0, 1.0}, constraints), std::invalid_argument);
  EXPECT_EQ(make_goal_poses({0.0, 0.0, 1.0, 0.0, 1.0, 1.0, 2.0, 0.5}, constraints).size(), 2U);
}

TEST(GoalInput, ExplicitYawDegreesAreConvertedWhilePlainYawRemainsRadians)
{
  const auto degrees = parse_goal_arguments({"1", "2", "3", "yaw=90"});
  ASSERT_EQ(degrees.size(), 4U);
  EXPECT_NEAR(degrees[3], 3.14159265358979323846 / 2.0, 1.0e-12);

  const auto radians = parse_goal_arguments({"1", "2", "3", "1.57"});
  EXPECT_DOUBLE_EQ(radians[3], 1.57);
  EXPECT_THROW(parse_goal_arguments({"1", "2", "3", "yaw=nan"}), std::invalid_argument);
}

}  // namespace drone_mission
