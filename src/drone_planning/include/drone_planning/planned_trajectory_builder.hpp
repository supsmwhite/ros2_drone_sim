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
  double shortcut_preferred_clearance{0.0};
  std::vector<double> velocity_scale_candidates{1.0, 0.75, 0.5, 0.25, 0.0};
  std::vector<double> duration_scale_candidates{
    1.0, 1.05, 1.10, 1.15, 1.20, 1.25, 1.5, 2.0, 3.0, 4.0};
  std::size_t max_refinement_iterations{8U};
  std::size_t max_insertions_per_refinement{3U};
  double fixed_yaw{0.0};
};

enum class TrajectoryFailureReason
{
  none,
  nonfinite,
  speed_limit,
  acceleration_limit,
  point_collision,
  segment_collision,
  endpoint_mismatch
};

struct PlannedTrajectoryResult
{
  bool success{false};
  std::vector<Eigen::Vector3d> simplified_path_world;
  std::vector<std::size_t> simplified_path_raw_indices;
  std::vector<double> segment_durations;
  std::size_t initial_simplified_point_count{0U};
  std::size_t preferred_shortcut_count{0U};
  std::size_t fallback_shortcut_count{0U};
  std::size_t collision_only_shortcut_count{0U};
  bool clearance_preference_enabled{false};
  std::size_t refinement_iterations{0U};
  double selected_velocity_scale{0.0};
  double selected_duration_scale{0.0};
  double total_duration{0.0};
  double max_reference_speed{0.0};
  double max_reference_acceleration{0.0};
  std::size_t validation_sample_count{0U};
  TrajectoryFailureReason failure_reason{TrajectoryFailureReason::none};
  std::size_t failure_segment_index{0U};
  double failure_time{0.0};
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
