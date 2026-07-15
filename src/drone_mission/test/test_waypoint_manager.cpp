#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drone_mission/waypoint_manager.hpp"

namespace drone_mission
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

Waypoint waypoint(double x, double y, double z, double yaw = 0.0)
{
  return Waypoint{Eigen::Vector3d(x, y, z), yaw};
}

VehicleState settled_at(const Waypoint & target)
{
  VehicleState state;
  state.position_world = target.position_world;
  state.yaw = target.yaw;
  return state;
}

WaypointManager manager_with(std::vector<Waypoint> waypoints)
{
  return WaypointManager(std::move(waypoints), 0.20, 0.15, 0.10, 0.20, 1.0);
}

TEST(WaypointManager, EmptyWaypointListIsRejected)
{
  EXPECT_THROW(manager_with({}), std::invalid_argument);
}

TEST(WaypointManager, NonFiniteWaypointIsRejected)
{
  EXPECT_THROW(
    manager_with({waypoint(std::numeric_limits<double>::quiet_NaN(), 0.0, 1.0)}),
    std::invalid_argument);
}

TEST(WaypointManager, InvalidTolerancesAndHoldDurationAreRejected)
{
  const auto target = waypoint(0.0, 0.0, 1.0);
  EXPECT_THROW(WaypointManager({target}, 0.0, 0.15, 0.10, 0.20, 1.0), std::invalid_argument);
  EXPECT_THROW(WaypointManager({target}, 0.2, -0.1, 0.10, 0.20, 1.0), std::invalid_argument);
  EXPECT_THROW(WaypointManager({target}, 0.2, 0.15, 0.0, 0.20, 1.0), std::invalid_argument);
  EXPECT_THROW(WaypointManager({target}, 0.2, 0.15, 0.10, 0.0, 1.0), std::invalid_argument);
  EXPECT_THROW(WaypointManager({target}, 0.2, 0.15, 0.10, 0.20, 0.0), std::invalid_argument);
}

TEST(WaypointManager, InitialIndexIsZero)
{
  auto manager = manager_with({waypoint(1.0, 2.0, 3.0), waypoint(4.0, 5.0, 6.0)});
  EXPECT_EQ(manager.current_index(), 0U);
  EXPECT_FALSE(manager.mission_complete());
  EXPECT_TRUE(manager.current_waypoint().position_world.isApprox(Eigen::Vector3d(1.0, 2.0, 3.0)));
}

TEST(WaypointManager, OutsidePositionToleranceDoesNotAdvance)
{
  auto manager = manager_with({waypoint(0.0, 0.0, 1.0), waypoint(1.0, 0.0, 1.0)});
  VehicleState state = settled_at(manager.current_waypoint());
  state.position_world.x() = 0.3;
  for (int step = 0; step < 20; ++step) {
    manager.update(state, 0.1);
  }
  EXPECT_EQ(manager.current_index(), 0U);
}

TEST(WaypointManager, ExcessiveLinearSpeedDoesNotAdvance)
{
  auto manager = manager_with({waypoint(0.0, 0.0, 1.0), waypoint(1.0, 0.0, 1.0)});
  VehicleState state = settled_at(manager.current_waypoint());
  state.linear_velocity.x() = 0.2;
  for (int step = 0; step < 20; ++step) {
    manager.update(state, 0.1);
  }
  EXPECT_EQ(manager.current_index(), 0U);
}

TEST(WaypointManager, ExcessiveAngularSpeedDoesNotAdvance)
{
  auto manager = manager_with({waypoint(0.0, 0.0, 1.0), waypoint(1.0, 0.0, 1.0)});
  VehicleState state = settled_at(manager.current_waypoint());
  state.angular_velocity.z() = 0.3;
  for (int step = 0; step < 20; ++step) {
    manager.update(state, 0.1);
  }
  EXPECT_EQ(manager.current_index(), 0U);
}

TEST(WaypointManager, YawOutsideToleranceDoesNotAdvance)
{
  auto manager = manager_with({waypoint(0.0, 0.0, 1.0), waypoint(1.0, 0.0, 1.0)});
  VehicleState state = settled_at(manager.current_waypoint());
  state.yaw = 0.2;
  for (int step = 0; step < 20; ++step) {
    manager.update(state, 0.1);
  }
  EXPECT_EQ(manager.current_index(), 0U);
}

TEST(WaypointManager, ContinuousAcceptanceAdvancesAfterOneSecond)
{
  auto manager = manager_with({waypoint(0.0, 0.0, 1.0), waypoint(1.0, 0.0, 1.0)});
  const VehicleState state = settled_at(manager.current_waypoint());
  for (int step = 0; step < 9; ++step) {
    EXPECT_FALSE(manager.update(state, 0.1).waypoint_accepted);
  }
  const auto output = manager.update(state, 0.1);
  EXPECT_TRUE(output.waypoint_accepted);
  EXPECT_EQ(output.current_index, 1U);
}

TEST(WaypointManager, LeavingAcceptanceResetsHoldTimer)
{
  auto manager = manager_with({waypoint(0.0, 0.0, 1.0), waypoint(1.0, 0.0, 1.0)});
  VehicleState state = settled_at(manager.current_waypoint());
  for (int step = 0; step < 6; ++step) {
    manager.update(state, 0.1);
  }
  state.linear_velocity.x() = 0.3;
  manager.update(state, 0.1);
  state.linear_velocity.setZero();
  for (int step = 0; step < 9; ++step) {
    manager.update(state, 0.1);
  }
  EXPECT_EQ(manager.current_index(), 0U);
  manager.update(state, 0.1);
  EXPECT_EQ(manager.current_index(), 1U);
}

TEST(WaypointManager, ExplicitResetBreaksContinuousAcceptance)
{
  auto manager = manager_with({waypoint(0.0, 0.0, 1.0), waypoint(1.0, 0.0, 1.0)});
  const VehicleState state = settled_at(manager.current_waypoint());
  manager.update(state, 0.6);

  manager.reset_acceptance_progress();

  EXPECT_FALSE(manager.update(state, 0.9).waypoint_accepted);
  EXPECT_EQ(manager.current_index(), 0U);
  EXPECT_TRUE(manager.update(state, 0.1).waypoint_accepted);
  EXPECT_EQ(manager.current_index(), 1U);
}

TEST(WaypointManager, WaypointsAdvanceSequentiallyWithoutSkipping)
{
  const std::vector<Waypoint> targets{
    waypoint(0.0, 0.0, 1.0), waypoint(1.0, 0.0, 1.0), waypoint(1.0, 1.0, 1.0)};
  auto manager = manager_with(targets);
  for (std::size_t index = 0U; index < targets.size() - 1U; ++index) {
    const VehicleState state = settled_at(targets[index]);
    const auto output = manager.update(state, 1.0);
    EXPECT_TRUE(output.waypoint_accepted);
    EXPECT_EQ(output.current_index, index + 1U);
    EXPECT_FALSE(output.mission_complete);
  }
}

TEST(WaypointManager, FinalWaypointCompletesAndRetainsFinalGoal)
{
  const auto first = waypoint(0.0, 0.0, 1.0);
  const auto final = waypoint(1.0, 2.0, 3.0, 0.5);
  auto manager = manager_with({first, final});
  manager.update(settled_at(first), 1.0);
  const auto completion = manager.update(settled_at(final), 1.0);
  EXPECT_TRUE(completion.waypoint_accepted);
  EXPECT_TRUE(completion.mission_complete);
  EXPECT_EQ(completion.current_index, 1U);

  VehicleState far_state;
  far_state.position_world = Eigen::Vector3d(100.0, 100.0, 100.0);
  const auto after_completion = manager.update(far_state, 1.0);
  EXPECT_TRUE(after_completion.mission_complete);
  EXPECT_FALSE(after_completion.waypoint_accepted);
  EXPECT_EQ(after_completion.current_index, 1U);
  EXPECT_TRUE(after_completion.current_waypoint.position_world.isApprox(final.position_world));
  EXPECT_DOUBLE_EQ(after_completion.current_waypoint.yaw, final.yaw);
}

TEST(WaypointManager, ExplicitResetDoesNotChangeCompletedMission)
{
  const auto final = waypoint(1.0, 2.0, 3.0, 0.5);
  auto manager = manager_with({final});
  manager.update(settled_at(final), 1.0);
  ASSERT_TRUE(manager.mission_complete());

  manager.reset_acceptance_progress();

  EXPECT_TRUE(manager.mission_complete());
  EXPECT_EQ(manager.current_index(), 0U);
  EXPECT_TRUE(manager.current_waypoint().position_world.isApprox(final.position_world));
  EXPECT_DOUBLE_EQ(manager.current_waypoint().yaw, final.yaw);
}

TEST(WaypointManager, YawWrapUsesShortestAngularError)
{
  const auto target = waypoint(0.0, 0.0, 1.0, kPi - 0.02);
  auto manager = manager_with({target});
  VehicleState state = settled_at(target);
  state.yaw = -kPi + 0.02;
  const auto output = manager.update(state, 1.0);
  EXPECT_TRUE(output.waypoint_accepted);
  EXPECT_TRUE(output.mission_complete);
}

}  // namespace
}  // namespace drone_mission
