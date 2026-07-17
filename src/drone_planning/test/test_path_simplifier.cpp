#include <limits>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

#include "drone_planning/path_simplifier.hpp"

namespace drone_planning
{
namespace
{

AxisAlignedBox box(
  double xmin, double xmax, double ymin, double ymax, double zmin, double zmax)
{
  return {Eigen::Vector3d(xmin, ymin, zmin), Eigen::Vector3d(xmax, ymax, zmax)};
}

CollisionChecker empty_checker()
{
  return CollisionChecker(
    StaticEnvironment(box(-5.0, 5.0, -5.0, 5.0, -5.0, 5.0), {}), 0.0);
}

CollisionChecker obstacle_checker()
{
  return CollisionChecker(
    StaticEnvironment(
      box(-5.0, 5.0, -5.0, 5.0, -5.0, 5.0),
      {box(-0.5, 0.5, -0.5, 0.5, -1.0, 1.0)}),
    0.0);
}

std::vector<Eigen::Vector3d> detour_path()
{
  return {
    Eigen::Vector3d(-2.0, 0.0, 0.0),
    Eigen::Vector3d(-1.0, 1.1, 0.0),
    Eigen::Vector3d(1.0, 1.1, 0.0),
    Eigen::Vector3d(2.0, 0.0, 0.0)};
}

TEST(PathSimplifier, RejectsTooFewPoints)
{
  const PathSimplifier simplifier(empty_checker());
  EXPECT_THROW(simplifier.simplify({}), std::invalid_argument);
  EXPECT_THROW(simplifier.simplify({Eigen::Vector3d::Zero()}), std::invalid_argument);
}

TEST(PathSimplifier, RejectsNonFinitePoint)
{
  EXPECT_THROW(
    PathSimplifier(empty_checker()).simplify({
      Eigen::Vector3d::Zero(),
      Eigen::Vector3d(std::numeric_limits<double>::quiet_NaN(), 1.0, 0.0)}),
    std::invalid_argument);
}

TEST(PathSimplifier, RejectsCollidingPointAndSegment)
{
  const PathSimplifier simplifier(obstacle_checker());
  EXPECT_THROW(
    simplifier.simplify({Eigen::Vector3d(-2.0, 0.0, 0.0), Eigen::Vector3d::Zero()}),
    std::invalid_argument);
  EXPECT_THROW(
    simplifier.simplify({
      Eigen::Vector3d(-2.0, 0.0, 0.0), Eigen::Vector3d(2.0, 0.0, 0.0)}),
    std::invalid_argument);
}

TEST(PathSimplifier, EmptyEnvironmentReducesToExactEndpoints)
{
  const std::vector<Eigen::Vector3d> input{
    Eigen::Vector3d(-1.25, -0.75, 0.125), Eigen::Vector3d::Zero(),
    Eigen::Vector3d(1.5, 1.25, 0.875)};
  const auto result = PathSimplifier(empty_checker()).simplify(input);
  ASSERT_EQ(result.size(), 2U);
  EXPECT_TRUE(result.front().isApprox(input.front(), 0.0));
  EXPECT_TRUE(result.back().isApprox(input.back(), 0.0));
}

TEST(PathSimplifier, ObstacleKeepsNecessaryTurnAndEveryOutputSegmentIsSafe)
{
  const auto checker = obstacle_checker();
  const auto input = detour_path();
  const auto result = PathSimplifier(checker).simplify(input);
  EXPECT_GT(result.size(), 2U);
  EXPECT_LE(result.size(), input.size());
  EXPECT_TRUE(result.front().isApprox(input.front(), 0.0));
  EXPECT_TRUE(result.back().isApprox(input.back(), 0.0));
  for (std::size_t index = 1U; index < result.size(); ++index) {
    EXPECT_FALSE(checker.segment_in_collision(result[index - 1U], result[index]));
  }
}

TEST(PathSimplifier, IndexedResultPreservesExactStrictlyIncreasingRawIndices)
{
  const auto input = detour_path();
  const auto result = PathSimplifier(obstacle_checker()).simplify_with_indices(input);
  ASSERT_EQ(result.points.size(), result.raw_indices.size());
  ASSERT_GE(result.raw_indices.size(), 2U);
  EXPECT_EQ(result.raw_indices.front(), 0U);
  EXPECT_EQ(result.raw_indices.back(), input.size() - 1U);
  for (std::size_t index = 0U; index < result.raw_indices.size(); ++index) {
    EXPECT_TRUE(result.points[index].isApprox(input[result.raw_indices[index]], 0.0));
    if (index > 0U) {
      EXPECT_LT(result.raw_indices[index - 1U], result.raw_indices[index]);
    }
  }
}

TEST(PathSimplifier, RepeatedResultIsExactlyDeterministic)
{
  const PathSimplifier simplifier(obstacle_checker());
  const auto first = simplifier.simplify(detour_path());
  const auto second = simplifier.simplify(detour_path());
  ASSERT_EQ(first.size(), second.size());
  for (std::size_t index = 0U; index < first.size(); ++index) {
    EXPECT_TRUE(first[index].isApprox(second[index], 0.0));
  }
}

}  // namespace
}  // namespace drone_planning
