#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

#include "drone_planning/astar_planner.hpp"

namespace drone_planning
{
namespace
{

AxisAlignedBox box(
  double xmin, double xmax, double ymin, double ymax, double zmin, double zmax)
{
  return {
    Eigen::Vector3d(xmin, ymin, zmin),
    Eigen::Vector3d(xmax, ymax, zmax)};
}

CollisionChecker empty_checker()
{
  return CollisionChecker(
    StaticEnvironment(box(-1.25, 2.25, -1.25, 2.25, -1.25, 2.25), {}), 0.25);
}

CollisionChecker default_checker()
{
  return CollisionChecker(
    StaticEnvironment(
      box(-1.0, 13.0, -2.5, 6.5, -0.5, 5.0),
      {
        box(2.4, 3.2, -2.5, 1.5, 0.0, 4.7),
        box(5.8, 6.6, 1.0, 6.5, 0.0, 4.7),
        box(5.8, 6.6, -2.5, -1.2, 0.0, 4.7),
        box(9.2, 10.0, -2.5, 1.5, 0.0, 4.7),
        box(9.2, 10.0, 4.0, 6.5, 0.0, 4.7),
      }),
    0.35);
}

void expect_safe_path(
  const CollisionChecker & checker, const AStarResult & result,
  const Eigen::Vector3d & start, const Eigen::Vector3d & goal)
{
  ASSERT_TRUE(result.success());
  ASSERT_GE(result.path_world.size(), 2U);
  EXPECT_TRUE(result.path_world.front().isApprox(start, 0.0));
  EXPECT_TRUE(result.path_world.back().isApprox(goal, 0.0));
  EXPECT_TRUE(std::isfinite(result.path_length));
  EXPECT_GT(result.path_length, 0.0);
  EXPECT_GT(result.expanded_nodes, 0U);
  double measured_length = 0.0;
  for (std::size_t index = 0U; index < result.path_world.size(); ++index) {
    EXPECT_TRUE(result.path_world[index].allFinite());
    EXPECT_FALSE(checker.point_in_collision(result.path_world[index]));
    if (index > 0U) {
      EXPECT_FALSE(checker.segment_in_collision(
        result.path_world[index - 1U], result.path_world[index]));
      measured_length +=
        (result.path_world[index] - result.path_world[index - 1U]).norm();
    }
  }
  EXPECT_NEAR(result.path_length, measured_length, 1.0e-12);
}

TEST(AStarPlanner, InvalidResolutionIsRejected)
{
  EXPECT_THROW(AStarPlanner(empty_checker(), 0.0, 1000U), std::invalid_argument);
  EXPECT_THROW(AStarPlanner(empty_checker(), -0.1, 1000U), std::invalid_argument);
  EXPECT_THROW(
    AStarPlanner(empty_checker(), std::numeric_limits<double>::infinity(), 1000U),
    std::invalid_argument);
  EXPECT_THROW(
    AStarPlanner(
      empty_checker(), std::numeric_limits<double>::quiet_NaN(), 1000U),
    std::invalid_argument);
}

TEST(AStarPlanner, ZeroMaxGridNodesIsRejected)
{
  EXPECT_THROW(AStarPlanner(empty_checker(), 1.0, 0U), std::invalid_argument);
}

TEST(AStarPlanner, GridLargerThanResourceLimitIsRejected)
{
  const CollisionChecker checker(
    StaticEnvironment(box(0.0, 10.0, 0.0, 10.0, 0.0, 10.0), {}), 0.0);
  EXPECT_THROW(AStarPlanner(checker, 0.1, 200000U), std::invalid_argument);
}

TEST(AStarPlanner, CollidingStartReturnsInvalidStart)
{
  const auto checker = default_checker();
  const AStarResult result = AStarPlanner(checker, 0.25, 200000U).plan(
    Eigen::Vector3d(2.8, -0.5, 1.5), Eigen::Vector3d(12.0, 2.7, 1.5));
  EXPECT_EQ(result.status, PlanningStatus::kInvalidStart);
  EXPECT_FALSE(result.success());
  EXPECT_TRUE(result.path_world.empty());
}

TEST(AStarPlanner, CollidingGoalReturnsInvalidGoal)
{
  const auto checker = default_checker();
  const AStarResult result = AStarPlanner(checker, 0.25, 200000U).plan(
    Eigen::Vector3d(0.0, 0.0, 1.5), Eigen::Vector3d(6.2, 3.75, 1.5));
  EXPECT_EQ(result.status, PlanningStatus::kInvalidGoal);
  EXPECT_FALSE(result.success());
  EXPECT_TRUE(result.path_world.empty());
}

TEST(AStarPlanner, EmptyEnvironmentUsesThreeDimensionalDiagonal)
{
  const auto checker = empty_checker();
  const Eigen::Vector3d start(0.0, 0.0, 0.0);
  const Eigen::Vector3d goal(1.0, 1.0, 1.0);
  const AStarResult result = AStarPlanner(checker, 1.0, 1000U).plan(start, goal);
  expect_safe_path(checker, result, start, goal);
  ASSERT_EQ(result.path_world.size(), 2U);
  EXPECT_NEAR(result.path_length, std::sqrt(3.0), 1.0e-12);
}

TEST(AStarPlanner, DefaultPlanningDemoFindsSafeBlockedPath)
{
  const auto checker = default_checker();
  const Eigen::Vector3d start(0.0, 0.0, 1.5);
  const Eigen::Vector3d goal(12.0, 2.7, 1.5);
  ASSERT_TRUE(checker.segment_in_collision(start, goal));
  const AStarResult result = AStarPlanner(checker, 0.25, 200000U).plan(start, goal);
  expect_safe_path(checker, result, start, goal);
  EXPECT_GT(result.path_length, (goal - start).norm());
}

TEST(AStarPlanner, RepeatedPlanningIsExactlyDeterministic)
{
  const auto checker = default_checker();
  const AStarPlanner planner(checker, 0.25, 200000U);
  const Eigen::Vector3d start(0.0, 0.0, 1.5);
  const Eigen::Vector3d goal(12.0, 2.7, 1.5);
  const AStarResult first = planner.plan(start, goal);
  const AStarResult second = planner.plan(start, goal);
  ASSERT_TRUE(first.success());
  ASSERT_EQ(first.path_world.size(), second.path_world.size());
  EXPECT_DOUBLE_EQ(first.path_length, second.path_length);
  EXPECT_EQ(first.expanded_nodes, second.expanded_nodes);
  for (std::size_t index = 0U; index < first.path_world.size(); ++index) {
    EXPECT_TRUE(first.path_world[index].isApprox(second.path_world[index], 0.0));
  }
}

TEST(AStarPlanner, SolidWallReturnsNoPath)
{
  const CollisionChecker checker(
    StaticEnvironment(
      box(-1.0, 3.0, -1.0, 3.0, -1.0, 3.0),
      {box(0.75, 1.25, -1.0, 3.0, -1.0, 3.0)}),
    0.0);
  const AStarResult result = AStarPlanner(checker, 0.5, 10000U).plan(
    Eigen::Vector3d(0.0, 1.0, 1.0), Eigen::Vector3d(2.0, 1.0, 1.0));
  EXPECT_EQ(result.status, PlanningStatus::kNoPath);
  EXPECT_FALSE(result.success());
  EXPECT_TRUE(result.path_world.empty());
  EXPECT_GT(result.expanded_nodes, 0U);
}

TEST(AStarPlanner, NonAlignedEndpointsConnectSafelyAndRemainExact)
{
  const CollisionChecker checker(
    StaticEnvironment(box(-1.0, 3.0, -1.0, 3.0, -1.0, 3.0), {}), 0.1);
  const Eigen::Vector3d start(0.12, 0.13, 0.14);
  const Eigen::Vector3d goal(1.82, 1.77, 1.69);
  const AStarResult result = AStarPlanner(checker, 0.5, 10000U).plan(start, goal);
  expect_safe_path(checker, result, start, goal);
  EXPECT_TRUE(result.path_world.front().isApprox(start, 0.0));
  EXPECT_TRUE(result.path_world.back().isApprox(goal, 0.0));
}

TEST(AStarPlanner, SnappedStartInCollisionReturnsInvalidStart)
{
  const CollisionChecker checker(
    StaticEnvironment(
      box(-1.0, 3.0, -1.0, 3.0, -1.0, 3.0),
      {box(0.9, 1.1, 0.9, 1.1, 0.9, 1.1)}),
    0.0);
  const AStarResult result = AStarPlanner(checker, 1.0, 1000U).plan(
    Eigen::Vector3d(0.55, 0.55, 0.55), Eigen::Vector3d(2.0, 2.0, 2.0));
  EXPECT_EQ(result.status, PlanningStatus::kInvalidStart);
}

TEST(AStarPlanner, DiagonalNeighborCannotCutThroughObstacleCorner)
{
  const CollisionChecker checker(
    StaticEnvironment(
      box(-0.5, 2.5, -0.5, 2.5, -0.5, 2.5),
      {box(0.9, 1.1, 0.9, 1.1, 0.0, 1.0)}),
    0.0);
  const Eigen::Vector3d start(0.5, 0.5, 0.5);
  const Eigen::Vector3d goal(1.5, 1.5, 0.5);
  ASSERT_TRUE(checker.segment_in_collision(start, goal));
  const AStarResult result = AStarPlanner(checker, 1.0, 1000U).plan(start, goal);
  expect_safe_path(checker, result, start, goal);
  EXPECT_GT(result.path_length, (goal - start).norm());
}

}  // namespace
}  // namespace drone_planning
