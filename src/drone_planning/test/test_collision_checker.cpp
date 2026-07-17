#include <limits>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

#include "drone_planning/collision_checker.hpp"

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

CollisionChecker make_checker()
{
  return CollisionChecker(
    StaticEnvironment(
      box(-2.0, 2.0, -2.0, 2.0, -2.0, 2.0),
      {box(-0.5, 0.5, -0.5, 0.5, -0.5, 0.5)}),
    0.25);
}

TEST(CollisionChecker, LegalEnvironmentPreservesGeometryAndRadius)
{
  const auto checker = make_checker();
  EXPECT_EQ(checker.environment().obstacles().size(), 1U);
  EXPECT_DOUBLE_EQ(checker.safety_radius(), 0.25);
  EXPECT_TRUE(checker.safe_workspace().min_corner.isApprox(Eigen::Vector3d::Constant(-1.75)));
  EXPECT_TRUE(checker.safe_workspace().max_corner.isApprox(Eigen::Vector3d::Constant(1.75)));
  EXPECT_TRUE(
    checker.inflated_obstacles().front().min_corner.isApprox(
      Eigen::Vector3d::Constant(-0.75)));
}

TEST(CollisionChecker, InvalidWorkspaceIsRejected)
{
  EXPECT_THROW(
    StaticEnvironment(box(1.0, 1.0, -1.0, 1.0, -1.0, 1.0), {}),
    std::invalid_argument);
  auto invalid = box(-1.0, 1.0, -1.0, 1.0, -1.0, 1.0);
  invalid.min_corner.x() = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(StaticEnvironment(invalid, {}), std::invalid_argument);
}

TEST(CollisionChecker, InvalidObstacleIsRejected)
{
  EXPECT_THROW(
    StaticEnvironment(
      box(-2.0, 2.0, -2.0, 2.0, -2.0, 2.0),
      {box(0.0, 0.0, -0.5, 0.5, -0.5, 0.5)}),
    std::invalid_argument);
  auto invalid = box(-0.5, 0.5, -0.5, 0.5, -0.5, 0.5);
  invalid.max_corner.z() = std::numeric_limits<double>::infinity();
  EXPECT_THROW(
    StaticEnvironment(box(-2.0, 2.0, -2.0, 2.0, -2.0, 2.0), {invalid}),
    std::invalid_argument);
}

TEST(CollisionChecker, InvalidSafetyRadiusIsRejected)
{
  const StaticEnvironment environment(box(-2.0, 2.0, -2.0, 2.0, -2.0, 2.0), {});
  EXPECT_THROW(CollisionChecker(environment, -0.01), std::invalid_argument);
  EXPECT_THROW(
    CollisionChecker(environment, std::numeric_limits<double>::quiet_NaN()),
    std::invalid_argument);
  EXPECT_THROW(
    CollisionChecker(environment, std::numeric_limits<double>::infinity()),
    std::invalid_argument);
}

TEST(CollisionChecker, RadiusThatEliminatesWorkspaceIsRejected)
{
  const StaticEnvironment environment(box(-1.0, 1.0, -1.0, 1.0, -1.0, 1.0), {});
  EXPECT_THROW(CollisionChecker(environment, 1.0), std::invalid_argument);
}

TEST(CollisionChecker, SafePointIsNotInCollision)
{
  EXPECT_FALSE(make_checker().point_in_collision(Eigen::Vector3d(1.0, 1.0, 1.0)));
}

TEST(CollisionChecker, PointInsideOriginalObstacleIsInCollision)
{
  EXPECT_TRUE(make_checker().point_in_collision(Eigen::Vector3d::Zero()));
}

TEST(CollisionChecker, PointInInflatedMarginIsInCollision)
{
  EXPECT_TRUE(make_checker().point_in_collision(Eigen::Vector3d(0.70, 0.0, 0.0)));
}

TEST(CollisionChecker, PointOutsideWorkspaceIsInCollision)
{
  EXPECT_TRUE(make_checker().point_in_collision(Eigen::Vector3d(1.80, 0.0, 0.0)));
}

TEST(CollisionChecker, SafeWorkspaceBoundaryIsInCollision)
{
  EXPECT_TRUE(make_checker().point_in_collision(Eigen::Vector3d(1.75, 1.0, 1.0)));
}

TEST(CollisionChecker, NonFinitePointIsInCollision)
{
  const auto checker = make_checker();
  EXPECT_TRUE(checker.point_in_collision(
    Eigen::Vector3d(std::numeric_limits<double>::quiet_NaN(), 1.0, 1.0)));
  EXPECT_TRUE(checker.point_in_collision(
    Eigen::Vector3d(1.0, std::numeric_limits<double>::infinity(), 1.0)));
}

TEST(CollisionChecker, CompletelySafeSegmentIsNotInCollision)
{
  EXPECT_FALSE(make_checker().segment_in_collision(
    Eigen::Vector3d(-1.0, -1.0, 1.0), Eigen::Vector3d(1.0, -1.0, 1.0)));
}

TEST(CollisionChecker, SegmentThroughObstacleIsInCollision)
{
  EXPECT_TRUE(make_checker().segment_in_collision(
    Eigen::Vector3d(-1.0, 0.0, 0.0), Eigen::Vector3d(1.0, 0.0, 0.0)));
}

TEST(CollisionChecker, SegmentTangentToSurfaceIsInCollision)
{
  EXPECT_TRUE(make_checker().segment_in_collision(
    Eigen::Vector3d(-1.0, 0.75, 0.0), Eigen::Vector3d(1.0, 0.75, 0.0)));
}

TEST(CollisionChecker, SegmentTouchingOnlyObstacleCornerIsInCollision)
{
  EXPECT_TRUE(make_checker().segment_in_collision(
    Eigen::Vector3d(-1.0, -0.5, -0.5), Eigen::Vector3d(-0.5, -1.0, -1.0)));
}

TEST(CollisionChecker, ParallelSegmentOutsideObstacleIsSafe)
{
  EXPECT_FALSE(make_checker().segment_in_collision(
    Eigen::Vector3d(-1.0, 1.0, 0.0), Eigen::Vector3d(1.0, 1.0, 0.0)));
}

TEST(CollisionChecker, ParallelSegmentInsideObstacleSlabCollides)
{
  EXPECT_TRUE(make_checker().segment_in_collision(
    Eigen::Vector3d(0.0, -1.0, 0.0), Eigen::Vector3d(0.0, 1.0, 0.0)));
}

TEST(CollisionChecker, SegmentWithEndpointInsideObstacleCollides)
{
  EXPECT_TRUE(make_checker().segment_in_collision(
    Eigen::Vector3d(-1.0, 0.0, 0.0), Eigen::Vector3d(0.0, 0.0, 0.0)));
}

TEST(CollisionChecker, ZeroLengthSegmentUsesPointSemantics)
{
  const auto checker = make_checker();
  EXPECT_FALSE(checker.segment_in_collision(
    Eigen::Vector3d(1.0, 1.0, 1.0), Eigen::Vector3d(1.0, 1.0, 1.0)));
  EXPECT_TRUE(checker.segment_in_collision(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()));
}

TEST(CollisionChecker, VeryShortSafeSegmentIsHandledDeterministically)
{
  EXPECT_FALSE(make_checker().segment_in_collision(
    Eigen::Vector3d(-1.0, -1.0, -1.0),
    Eigen::Vector3d(-1.0 + 1.0e-12, -1.0, -1.0)));
}

TEST(CollisionChecker, NonFiniteSegmentIsInCollision)
{
  const auto checker = make_checker();
  EXPECT_TRUE(checker.segment_in_collision(
    Eigen::Vector3d::Zero(),
    Eigen::Vector3d(std::numeric_limits<double>::quiet_NaN(), 1.0, 1.0)));
}

TEST(CollisionChecker, PlanningDemoHasBlockedDirectPathAndKnownSafeSnakeRoute)
{
  const CollisionChecker checker(
    StaticEnvironment(
      box(-1.0, 14.5, -2.5, 7.0, -0.5, 5.0),
      {
        box(2.2, 3.0, -2.5, 1.5, 0.0, 4.7),
        box(4.2, 5.0, 1.8, 6.5, 0.0, 4.7),
        box(6.3, 7.1, -0.8, 2.4, 0.0, 4.7),
        box(8.5, 9.3, 1.0, 6.5, 0.0, 4.7),
        box(11.0, 11.8, -1.5, -0.2, 0.0, 4.7),
        box(11.0, 11.8, 1.7, 4.2, 0.0, 4.7),
      }),
    0.25);
  const Eigen::Vector3d start(0.0, 0.0, 1.5);
  const Eigen::Vector3d goal(12.1, 1.1, 1.5);
  const std::vector<Eigen::Vector3d> known_safe_route{
    start,
    Eigen::Vector3d(1.85, 2.1, 1.6),
    Eigen::Vector3d(3.6, 1.85, 1.6),
    Eigen::Vector3d(3.85, 1.35, 1.6),
    Eigen::Vector3d(6.1, -1.4, 1.6),
    Eigen::Vector3d(7.85, -1.15, 1.6),
    Eigen::Vector3d(10.1, 0.35, 1.6),
    goal};

  EXPECT_FALSE(checker.point_in_collision(start));
  EXPECT_FALSE(checker.point_in_collision(goal));
  EXPECT_TRUE(checker.segment_in_collision(start, goal));

  for (std::size_t index = 1U; index < known_safe_route.size(); ++index) {
    EXPECT_FALSE(checker.segment_in_collision(
      known_safe_route[index - 1U], known_safe_route[index]));
  }

  EXPECT_TRUE(checker.point_in_collision(Eigen::Vector3d(2.6, -0.5, 1.5)));
  EXPECT_TRUE(checker.point_in_collision(Eigen::Vector3d(4.6, 4.15, 1.5)));
  EXPECT_TRUE(checker.point_in_collision(Eigen::Vector3d(6.7, 0.8, 1.5)));
  EXPECT_TRUE(checker.point_in_collision(Eigen::Vector3d(8.9, 3.75, 1.5)));
  EXPECT_TRUE(checker.point_in_collision(Eigen::Vector3d(11.4, -0.85, 1.5)));
  EXPECT_TRUE(checker.point_in_collision(Eigen::Vector3d(11.4, 2.95, 1.5)));
}

TEST(CollisionChecker, StaticAvoidanceEvaluationGoalsArePlanningSafe)
{
  const CollisionChecker checker(
    StaticEnvironment(
      box(-1.0, 14.5, -2.5, 7.0, -0.5, 5.0),
      {
        box(2.2, 3.0, -2.5, 1.5, 0.0, 4.7),
        box(4.2, 5.0, 1.8, 6.5, 0.0, 4.7),
        box(6.3, 7.1, -0.8, 2.4, 0.0, 4.7),
        box(8.5, 9.3, 1.0, 6.5, 0.0, 4.7),
        box(11.0, 11.8, -1.5, -0.2, 0.0, 4.7),
        box(11.0, 11.8, 1.7, 4.2, 0.0, 4.7),
      }),
    0.35);

  EXPECT_FALSE(checker.point_in_collision(Eigen::Vector3d(0.0, 0.0, 1.5)));
  EXPECT_EQ(checker.inflated_obstacles().size(), 6U);
  EXPECT_DOUBLE_EQ(
    checker.inflated_obstacles()[5].min_corner.y() -
    checker.inflated_obstacles()[4].max_corner.y(),
    1.2);
  EXPECT_DOUBLE_EQ(
    checker.inflated_obstacles()[4].min_corner.x() -
    checker.inflated_obstacles()[3].max_corner.x(),
    1.0);
  EXPECT_FALSE(checker.point_in_collision(Eigen::Vector3d(13.2, 5.5, 1.5)));
  EXPECT_FALSE(checker.point_in_collision(Eigen::Vector3d(12.1, 1.1, 1.5)));
  EXPECT_FALSE(checker.point_in_collision(Eigen::Vector3d(12.1, 1.1, 2.5)));
  EXPECT_FALSE(checker.point_in_collision(Eigen::Vector3d(7.0, 5.0, 4.0)));
  EXPECT_FALSE(checker.point_in_collision(Eigen::Vector3d(0.8, 0.7, 2.0)));
}

}  // namespace
}  // namespace drone_planning
