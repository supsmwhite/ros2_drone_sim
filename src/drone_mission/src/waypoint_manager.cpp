#include "drone_mission/waypoint_manager.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace drone_mission
{
namespace
{

bool finite_vector(const Eigen::Vector3d & vector)
{
  return vector.allFinite();
}

bool finite_positive(double value)
{
  return std::isfinite(value) && value > 0.0;
}

}  // namespace

WaypointManager::WaypointManager(
  std::vector<Waypoint> waypoints,
  double position_tolerance,
  double linear_speed_tolerance,
  double yaw_tolerance,
  double angular_speed_tolerance,
  double hold_duration)
: waypoints_(std::move(waypoints)),
  position_tolerance_(position_tolerance),
  linear_speed_tolerance_(linear_speed_tolerance),
  yaw_tolerance_(yaw_tolerance),
  angular_speed_tolerance_(angular_speed_tolerance),
  hold_duration_(hold_duration)
{
  if (waypoints_.empty()) {
    throw std::invalid_argument("waypoints must contain at least one target");
  }
  for (const auto & waypoint : waypoints_) {
    if (!finite_vector(waypoint.position_world) || !std::isfinite(waypoint.yaw)) {
      throw std::invalid_argument("all waypoint values must be finite");
    }
  }
  if (!finite_positive(position_tolerance_) ||
    !finite_positive(linear_speed_tolerance_) ||
    !finite_positive(yaw_tolerance_) ||
    !finite_positive(angular_speed_tolerance_) ||
    !finite_positive(hold_duration_))
  {
    throw std::invalid_argument("waypoint tolerances and hold duration must be finite and positive");
  }
}

WaypointManagerOutput WaypointManager::update(const VehicleState & state, double dt)
{
  if (!std::isfinite(dt) || dt <= 0.0) {
    throw std::invalid_argument("update dt must be finite and positive");
  }

  WaypointManagerOutput output;
  output.current_waypoint = current_waypoint();
  output.current_index = current_index_;
  output.mission_complete = mission_complete_;
  if (mission_complete_) {
    return output;
  }

  const bool finite_state = finite_vector(state.position_world) &&
    std::isfinite(state.yaw) && finite_vector(state.linear_velocity) &&
    finite_vector(state.angular_velocity);
  if (!finite_state || !state_is_within_acceptance(state)) {
    stable_duration_ = 0.0;
    return output;
  }

  stable_duration_ += dt;
  if (stable_duration_ + 1.0e-12 < hold_duration_) {
    return output;
  }

  stable_duration_ = 0.0;
  output.waypoint_accepted = true;
  if (current_index_ + 1U < waypoints_.size()) {
    ++current_index_;
  } else {
    mission_complete_ = true;
  }

  output.current_waypoint = current_waypoint();
  output.current_index = current_index_;
  output.mission_complete = mission_complete_;
  return output;
}

void WaypointManager::reset_acceptance_progress()
{
  stable_duration_ = 0.0;
}

const Waypoint & WaypointManager::current_waypoint() const
{
  return waypoints_.at(current_index_);
}

std::size_t WaypointManager::current_index() const
{
  return current_index_;
}

bool WaypointManager::mission_complete() const
{
  return mission_complete_;
}

bool WaypointManager::state_is_within_acceptance(const VehicleState & state) const
{
  const Waypoint & target = current_waypoint();
  return (state.position_world - target.position_world).norm() < position_tolerance_ &&
         state.linear_velocity.norm() < linear_speed_tolerance_ &&
         std::abs(shortest_yaw_error(target.yaw, state.yaw)) < yaw_tolerance_ &&
         state.angular_velocity.norm() < angular_speed_tolerance_;
}

double WaypointManager::shortest_yaw_error(double target, double current)
{
  return std::remainder(target - current, 2.0 * 3.14159265358979323846);
}

}  // namespace drone_mission
