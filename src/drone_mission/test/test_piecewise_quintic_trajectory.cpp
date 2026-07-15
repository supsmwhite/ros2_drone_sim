#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

#include "drone_mission/piecewise_quintic_trajectory.hpp"

namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kTolerance = 1.0e-9;

drone_mission::TrajectoryWaypoint waypoint(
  double x, double y, double z, double yaw = 0.0)
{
  return {Eigen::Vector3d(x, y, z), yaw};
}

std::vector<drone_mission::TrajectoryWaypoint> representative_waypoints()
{
  return {
    waypoint(0.0, 0.0, 1.0, 0.0),
    waypoint(2.0, 0.0, 1.0, 0.4),
    waypoint(2.0, 2.0, 2.0, 1.0),
  };
}

TEST(PiecewiseQuinticTrajectory, StartsAndEndsAtConfiguredBoundaryStates)
{
  const drone_mission::PiecewiseQuinticTrajectory trajectory(
    representative_waypoints(), {4.0, 5.0});
  const auto start = trajectory.sample(0.0);
  const auto end = trajectory.sample(trajectory.total_duration());

  EXPECT_TRUE(start.position_world.isApprox(Eigen::Vector3d(0.0, 0.0, 1.0), kTolerance));
  EXPECT_TRUE(start.velocity_world.isZero(kTolerance));
  EXPECT_TRUE(start.acceleration_world.isZero(kTolerance));
  EXPECT_FALSE(start.complete);
  EXPECT_TRUE(end.position_world.isApprox(Eigen::Vector3d(2.0, 2.0, 2.0), kTolerance));
  EXPECT_TRUE(end.velocity_world.isZero(kTolerance));
  EXPECT_TRUE(end.acceleration_world.isZero(kTolerance));
  EXPECT_NEAR(end.yaw, 1.0, kTolerance);
  EXPECT_TRUE(end.complete);
}

TEST(PiecewiseQuinticTrajectory, IntermediateWaypointHasSharedNonzeroVelocity)
{
  const drone_mission::PiecewiseQuinticTrajectory trajectory(
    representative_waypoints(), {4.0, 5.0});
  const auto boundary = trajectory.sample(4.0);
  const Eigen::Vector3d expected_velocity =
    0.5 * (Eigen::Vector3d(2.0, 0.0, 0.0) / 4.0 +
    Eigen::Vector3d(0.0, 2.0, 1.0) / 5.0);

  EXPECT_EQ(boundary.segment_index, 1U);
  EXPECT_TRUE(boundary.position_world.isApprox(Eigen::Vector3d(2.0, 0.0, 1.0), kTolerance));
  EXPECT_TRUE(boundary.velocity_world.isApprox(expected_velocity, kTolerance));
  EXPECT_GT(boundary.velocity_world.norm(), 0.1);
  EXPECT_TRUE(boundary.acceleration_world.isZero(kTolerance));
}

TEST(PiecewiseQuinticTrajectory, SegmentJunctionIsC2Continuous)
{
  const drone_mission::PiecewiseQuinticTrajectory trajectory(
    representative_waypoints(), {4.0, 5.0});
  constexpr double epsilon = 1.0e-6;
  const auto before = trajectory.sample(4.0 - epsilon);
  const auto at = trajectory.sample(4.0);
  const auto after = trajectory.sample(4.0 + epsilon);

  EXPECT_LT((before.position_world - at.position_world).norm(), 1.0e-5);
  EXPECT_LT((after.position_world - at.position_world).norm(), 1.0e-5);
  EXPECT_LT((before.velocity_world - at.velocity_world).norm(), 1.0e-5);
  EXPECT_LT((after.velocity_world - at.velocity_world).norm(), 1.0e-5);
  EXPECT_LT((before.acceleration_world - at.acceleration_world).norm(), 1.0e-5);
  EXPECT_LT((after.acceleration_world - at.acceleration_world).norm(), 1.0e-5);
  EXPECT_LT(std::abs(before.yaw - at.yaw), 1.0e-5);
  EXPECT_LT(std::abs(after.yaw - at.yaw), 1.0e-5);
}

TEST(PiecewiseQuinticTrajectory, YawWrapUsesShortestContinuousDirection)
{
  const drone_mission::PiecewiseQuinticTrajectory trajectory(
    {waypoint(0.0, 0.0, 0.0, kPi - 0.02),
      waypoint(1.0, 0.0, 0.0, -kPi + 0.02)},
    {2.0});
  const auto midpoint = trajectory.sample(1.0);
  const auto end = trajectory.sample(2.0);

  EXPECT_NEAR(midpoint.yaw, kPi, 1.0e-9);
  EXPECT_NEAR(end.yaw, kPi + 0.02, 1.0e-9);
}

TEST(PiecewiseQuinticTrajectory, ExactHalfTurnUsesDeterministicPositiveDirection)
{
  const drone_mission::PiecewiseQuinticTrajectory trajectory(
    {waypoint(0.0, 0.0, 0.0, 0.0), waypoint(1.0, 0.0, 0.0, -kPi)}, {2.0});
  EXPECT_GT(trajectory.sample(1.0).yaw, 0.0);
  EXPECT_NEAR(trajectory.sample(2.0).yaw, kPi, kTolerance);
}

TEST(PiecewiseQuinticTrajectory, InvalidConfigurationIsRejected)
{
  EXPECT_THROW(
    drone_mission::PiecewiseQuinticTrajectory({waypoint(0.0, 0.0, 0.0)}, {}),
    std::invalid_argument);
  EXPECT_THROW(
    drone_mission::PiecewiseQuinticTrajectory(
      {waypoint(0.0, 0.0, 0.0), waypoint(1.0, 0.0, 0.0)}, {}),
    std::invalid_argument);
  EXPECT_THROW(
    drone_mission::PiecewiseQuinticTrajectory(
      {waypoint(0.0, 0.0, 0.0), waypoint(1.0, 0.0, 0.0)}, {0.0}),
    std::invalid_argument);
  EXPECT_THROW(
    drone_mission::PiecewiseQuinticTrajectory(
      {waypoint(0.0, 0.0, 0.0),
        waypoint(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0)}, {1.0}),
    std::invalid_argument);
  const drone_mission::PiecewiseQuinticTrajectory trajectory(
    {waypoint(0.0, 0.0, 0.0), waypoint(1.0, 0.0, 0.0)}, {1.0});
  EXPECT_THROW(
    trajectory.sample(std::numeric_limits<double>::infinity()), std::invalid_argument);
}

TEST(PiecewiseQuinticTrajectory, SamplingAfterEndHoldsFinalState)
{
  const drone_mission::PiecewiseQuinticTrajectory trajectory(
    representative_waypoints(), {4.0, 5.0});
  const auto result = trajectory.sample(100.0);

  EXPECT_TRUE(result.complete);
  EXPECT_EQ(result.segment_index, 1U);
  EXPECT_TRUE(result.position_world.isApprox(Eigen::Vector3d(2.0, 2.0, 2.0), kTolerance));
  EXPECT_TRUE(result.velocity_world.isZero(kTolerance));
  EXPECT_TRUE(result.acceleration_world.isZero(kTolerance));
  EXPECT_NEAR(result.yaw, 1.0, kTolerance);
}

}  // namespace
