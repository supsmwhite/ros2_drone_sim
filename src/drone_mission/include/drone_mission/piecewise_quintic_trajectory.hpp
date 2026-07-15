#ifndef DRONE_MISSION__PIECEWISE_QUINTIC_TRAJECTORY_HPP_
#define DRONE_MISSION__PIECEWISE_QUINTIC_TRAJECTORY_HPP_

#include <array>
#include <cstddef>
#include <vector>

#include <Eigen/Core>

namespace drone_mission
{

struct TrajectoryWaypoint
{
  Eigen::Vector3d position_world{Eigen::Vector3d::Zero()};
  double yaw{0.0};
};

struct TrajectorySample
{
  Eigen::Vector3d position_world{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity_world{Eigen::Vector3d::Zero()};
  Eigen::Vector3d acceleration_world{Eigen::Vector3d::Zero()};
  double yaw{0.0};
  std::size_t segment_index{0U};
  bool complete{false};
};

class PiecewiseQuinticTrajectory
{
public:
  PiecewiseQuinticTrajectory(
    std::vector<TrajectoryWaypoint> waypoints,
    std::vector<double> segment_durations);

  TrajectorySample sample(double elapsed_time) const;
  std::size_t segment_count() const;
  double total_duration() const;

private:
  struct Segment
  {
    std::array<Eigen::Vector3d, 6> position_coefficients;
    std::array<double, 6> yaw_coefficients{};
    double duration{0.0};
    double start_time{0.0};
  };

  static std::array<double, 6> quintic_coefficients(
    double start_position, double start_velocity, double start_acceleration,
    double end_position, double end_velocity, double end_acceleration,
    double duration);
  static double deterministic_shortest_delta(double target_minus_current);

  std::vector<TrajectoryWaypoint> waypoints_;
  std::vector<Segment> segments_;
  double total_duration_{0.0};
};

}  // namespace drone_mission

#endif  // DRONE_MISSION__PIECEWISE_QUINTIC_TRAJECTORY_HPP_
