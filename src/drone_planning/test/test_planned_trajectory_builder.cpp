#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

#include "drone_planning/astar_planner.hpp"
#include "drone_planning/planned_trajectory_builder.hpp"

namespace drone_planning
{
namespace
{

AxisAlignedBox box(
  double xmin, double xmax, double ymin, double ymax, double zmin, double zmax)
{
  return {Eigen::Vector3d(xmin, ymin, zmin), Eigen::Vector3d(xmax, ymax, zmax)};
}

CollisionChecker default_checker()
{
  return CollisionChecker(
    StaticEnvironment(
      box(-1.0, 9.0, -2.5, 7.5, -0.5, 5.0),
      {box(2.1, 2.9, -0.5, 2.5, 0.0, 3.0),
        box(5.6, 6.4, 2.5, 5.5, 0.0, 3.0)}),
    0.35);
}

CollisionChecker navigation_floor_checker()
{
  return CollisionChecker(
    StaticEnvironment(
      box(-1.0, 9.0, -2.5, 7.5, 0.15, 5.0),
      {box(2.1, 2.9, -0.5, 2.5, 0.0, 3.0),
        box(5.6, 6.4, 2.5, 5.5, 0.0, 3.0)}),
    0.35);
}

std::vector<Eigen::Vector3d> default_raw_path()
{
  const auto checker = default_checker();
  const auto result = AStarPlanner(checker, 0.25, 200000U).plan(
    Eigen::Vector3d(0.0, 0.0, 1.5), Eigen::Vector3d(8.0, 5.0, 1.5));
  EXPECT_TRUE(result.success());
  return result.path_world;
}

void expect_valid_result(
  const CollisionChecker & checker, const PlannedTrajectoryParameters & parameters,
  const std::vector<Eigen::Vector3d> & raw_path,
  const PlannedTrajectoryResult & result)
{
  ASSERT_TRUE(result.success);
  ASSERT_TRUE(result.trajectory.has_value());
  ASSERT_LE(result.simplified_path_world.size(), raw_path.size());
  EXPECT_TRUE(result.simplified_path_world.front().isApprox(raw_path.front(), 0.0));
  EXPECT_TRUE(result.simplified_path_world.back().isApprox(raw_path.back(), 0.0));
  EXPECT_LE(result.max_reference_speed, parameters.max_reference_speed);
  EXPECT_LE(result.max_reference_acceleration, parameters.max_reference_acceleration);
  EXPECT_GT(result.total_duration, 0.0);
  EXPECT_GT(result.validation_sample_count, 1U);
  std::optional<Eigen::Vector3d> previous;
  for (double time = 0.0; time < result.total_duration;
    time += parameters.validation_sample_period)
  {
    const auto sample = result.trajectory->sample(time);
    EXPECT_TRUE(sample.position_world.allFinite());
    EXPECT_FALSE(checker.point_in_collision(sample.position_world));
    if (previous) {
      EXPECT_FALSE(checker.segment_in_collision(*previous, sample.position_world));
    }
    previous = sample.position_world;
  }
  const auto final_sample = result.trajectory->sample(result.total_duration);
  EXPECT_TRUE(final_sample.position_world.isApprox(raw_path.back(), 0.0));
  EXPECT_FALSE(checker.segment_in_collision(*previous, final_sample.position_world));
}

TEST(PlannedTrajectoryBuilder, DefaultAStarScenarioProducesSafeBoundedTrajectory)
{
  const auto checker = default_checker();
  PlannedTrajectoryParameters parameters;
  const auto raw_path = default_raw_path();
  const auto result = PlannedTrajectoryBuilder(checker, parameters).build(raw_path);
  expect_valid_result(checker, parameters, raw_path, result);
  EXPECT_LT(result.simplified_path_world.size(), raw_path.size());
}

TEST(PlannedTrajectoryBuilder, RepeatedBuildIsExactlyDeterministic)
{
  const PlannedTrajectoryBuilder builder(default_checker());
  const auto raw_path = default_raw_path();
  const auto first = builder.build(raw_path);
  const auto second = builder.build(raw_path);
  ASSERT_TRUE(first.success);
  ASSERT_TRUE(second.success);
  EXPECT_EQ(first.simplified_path_world.size(), second.simplified_path_world.size());
  EXPECT_EQ(first.segment_durations, second.segment_durations);
  EXPECT_DOUBLE_EQ(first.selected_velocity_scale, second.selected_velocity_scale);
  EXPECT_DOUBLE_EQ(first.total_duration, second.total_duration);
  EXPECT_DOUBLE_EQ(first.max_reference_speed, second.max_reference_speed);
  EXPECT_DOUBLE_EQ(first.max_reference_acceleration, second.max_reference_acceleration);
}

TEST(PlannedTrajectoryBuilder, MultiGoalFirstSegmentUsesBoundedMissionSpeed)
{
  const auto checker = navigation_floor_checker();
  EXPECT_DOUBLE_EQ(checker.safe_workspace().min_corner.z(), 0.50);
  const auto astar_result = AStarPlanner(checker, 0.25, 200000U).plan(
    Eigen::Vector3d(0.0, 0.0, 1.469), Eigen::Vector3d(4.0, 0.0, 1.5));
  ASSERT_TRUE(astar_result.success());
  PlannedTrajectoryParameters parameters;
  parameters.nominal_speed = 0.25;
  const auto trajectory_result =
    PlannedTrajectoryBuilder(checker, parameters).build(astar_result.path_world);
  expect_valid_result(checker, parameters, astar_result.path_world, trajectory_result);
  EXPECT_DOUBLE_EQ(trajectory_result.selected_velocity_scale, 1.0);
}

TEST(PlannedTrajectoryBuilder, DefaultOrderedMultiGoalSegmentsAllValidate)
{
  const auto checker = navigation_floor_checker();
  PlannedTrajectoryParameters parameters;
  parameters.nominal_speed = 0.25;
  const std::vector<Eigen::Vector3d> goals{
    Eigen::Vector3d(4.0, 0.0, 1.5),
    Eigen::Vector3d(8.0, 5.0, 1.5),
    Eigen::Vector3d(4.0, 6.5, 3.5),
    Eigen::Vector3d(0.0, 4.0, 1.5)};
  Eigen::Vector3d start(0.0, 0.0, 1.469);
  for (const auto & goal : goals) {
    const auto astar_result =
      AStarPlanner(checker, 0.25, 200000U).plan(start, goal);
    ASSERT_TRUE(astar_result.success());
    const auto trajectory_result =
      PlannedTrajectoryBuilder(checker, parameters).build(astar_result.path_world);
    expect_valid_result(checker, parameters, astar_result.path_world, trajectory_result);
    for (double time = 0.0; time <= trajectory_result.total_duration; time += 0.02) {
      EXPECT_GT(trajectory_result.trajectory->sample(time).position_world.z(), 0.50);
    }
    start = goal;
  }
}

TEST(PlannedTrajectoryBuilder, UnsafeCornerCandidateFallsBackToZeroScale)
{
  const CollisionChecker checker(
    StaticEnvironment(
      box(-4.0, 4.0, -4.0, 4.0, -2.0, 2.0),
      {box(-0.5, 0.5, -0.5, 0.5, -1.0, 1.0),
        box(-0.6, -0.4, 0.57, 0.64, -1.0, 1.0)}),
    0.0);
  PlannedTrajectoryParameters parameters;
  parameters.nominal_speed = 1.0;
  parameters.min_segment_duration = 1.0;
  parameters.validation_sample_period = 0.005;
  parameters.max_reference_speed = 3.0;
  parameters.max_reference_acceleration = 10.0;
  parameters.velocity_scale_candidates = {1.0, 0.0};
  const std::vector<Eigen::Vector3d> path{
    Eigen::Vector3d(-2.0, 0.0, 0.0),
    Eigen::Vector3d(0.0, 0.68, 0.0),
    Eigen::Vector3d(2.0, 0.0, 0.0)};
  const auto result = PlannedTrajectoryBuilder(checker, parameters).build(path);
  ASSERT_TRUE(result.success);
  EXPECT_DOUBLE_EQ(result.selected_velocity_scale, 0.0);
  expect_valid_result(checker, parameters, path, result);
}

TEST(PlannedTrajectoryBuilder, InvalidParametersAreRejected)
{
  PlannedTrajectoryParameters parameters;
  parameters.nominal_speed = 0.0;
  EXPECT_THROW(PlannedTrajectoryBuilder(default_checker(), parameters), std::invalid_argument);
  parameters = PlannedTrajectoryParameters{};
  parameters.velocity_scale_candidates = {};
  EXPECT_THROW(PlannedTrajectoryBuilder(default_checker(), parameters), std::invalid_argument);
  parameters.velocity_scale_candidates = {
    std::numeric_limits<double>::quiet_NaN()};
  EXPECT_THROW(PlannedTrajectoryBuilder(default_checker(), parameters), std::invalid_argument);
}

}  // namespace
}  // namespace drone_planning
