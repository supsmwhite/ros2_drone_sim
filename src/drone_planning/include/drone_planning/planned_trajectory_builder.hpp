#pragma once

#include <cstddef>
#include <optional>
#include <vector>

#include <Eigen/Core>

#include "drone_mission/piecewise_quintic_trajectory.hpp"
#include "drone_planning/collision_checker.hpp"

namespace drone_planning
{

struct PlannedTrajectoryParameters
{
  double nominal_speed{0.35};
  double min_segment_duration{2.0};
  double validation_sample_period{0.02};
  double max_reference_speed{0.70};
  double max_reference_acceleration{0.35};
  std::vector<double> velocity_scale_candidates{1.0, 0.75, 0.5, 0.25, 0.0};
  double fixed_yaw{0.0};
};

struct PlannedTrajectoryResult
{
  bool success{false};
  std::vector<Eigen::Vector3d> simplified_path_world;
  std::vector<double> segment_durations;
  double selected_velocity_scale{0.0};
  double total_duration{0.0};
  double max_reference_speed{0.0};
  double max_reference_acceleration{0.0};
  std::size_t validation_sample_count{0U};
  std::optional<drone_mission::PiecewiseQuinticTrajectory> trajectory;
};

class PlannedTrajectoryBuilder
{
public:
  PlannedTrajectoryBuilder(
    CollisionChecker collision_checker,
    PlannedTrajectoryParameters parameters = PlannedTrajectoryParameters{});

  PlannedTrajectoryResult build(
    const std::vector<Eigen::Vector3d> & raw_path_world) const;

private:
  CollisionChecker collision_checker_;
  PlannedTrajectoryParameters parameters_;
};

}  // namespace drone_planning
