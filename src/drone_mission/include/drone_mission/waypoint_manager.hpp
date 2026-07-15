#ifndef DRONE_MISSION__WAYPOINT_MANAGER_HPP_
#define DRONE_MISSION__WAYPOINT_MANAGER_HPP_

#include <cstddef>
#include <vector>

#include <Eigen/Core>

namespace drone_mission
{

struct Waypoint
{
  Eigen::Vector3d position_world{Eigen::Vector3d::Zero()};
  double yaw{0.0};
};

struct VehicleState
{
  Eigen::Vector3d position_world{Eigen::Vector3d::Zero()};
  double yaw{0.0};
  Eigen::Vector3d linear_velocity{Eigen::Vector3d::Zero()};
  Eigen::Vector3d angular_velocity{Eigen::Vector3d::Zero()};
};

struct WaypointManagerOutput
{
  Waypoint current_waypoint;
  std::size_t current_index{0U};
  bool mission_complete{false};
  bool waypoint_accepted{false};
};

class WaypointManager
{
public:
  WaypointManager(
    std::vector<Waypoint> waypoints,
    double position_tolerance,
    double linear_speed_tolerance,
    double yaw_tolerance,
    double angular_speed_tolerance,
    double hold_duration);

  WaypointManagerOutput update(const VehicleState & state, double dt);

  void reset_acceptance_progress();

  const Waypoint & current_waypoint() const;
  std::size_t current_index() const;
  bool mission_complete() const;

private:
  bool state_is_within_acceptance(const VehicleState & state) const;
  static double shortest_yaw_error(double target, double current);

  std::vector<Waypoint> waypoints_;
  double position_tolerance_;
  double linear_speed_tolerance_;
  double yaw_tolerance_;
  double angular_speed_tolerance_;
  double hold_duration_;
  std::size_t current_index_{0U};
  double stable_duration_{0.0};
  bool mission_complete_{false};
};

}  // namespace drone_mission

#endif  // DRONE_MISSION__WAYPOINT_MANAGER_HPP_
