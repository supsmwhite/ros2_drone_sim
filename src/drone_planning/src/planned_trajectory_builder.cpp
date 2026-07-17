#include "drone_planning/planned_trajectory_builder.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

#include "drone_planning/path_simplifier.hpp"

namespace drone_planning
{
namespace
{

bool finite_positive(double value)
{
  return std::isfinite(value) && value > 0.0;
}

void validate_parameters(const PlannedTrajectoryParameters & parameters)
{
  if (!finite_positive(parameters.nominal_speed) ||
    !finite_positive(parameters.min_segment_duration) ||
    !finite_positive(parameters.validation_sample_period) ||
    !finite_positive(parameters.max_reference_speed) ||
    !finite_positive(parameters.max_reference_acceleration) ||
    !std::isfinite(parameters.fixed_yaw))
  {
    throw std::invalid_argument("planned trajectory scalar parameters are invalid");
  }
  if (parameters.velocity_scale_candidates.empty()) {
    throw std::invalid_argument("velocity scale candidates must not be empty");
  }
  for (const double scale : parameters.velocity_scale_candidates) {
    if (!std::isfinite(scale) || scale < 0.0 || scale > 1.0) {
      throw std::invalid_argument("velocity scale candidates must be finite and in [0, 1]");
    }
  }
  if (parameters.duration_scale_candidates.empty() ||
    parameters.duration_scale_candidates.front() != 1.0)
  {
    throw std::invalid_argument("duration scale candidates must start with 1.0");
  }
  double previous_duration_scale = 0.0;
  for (const double scale : parameters.duration_scale_candidates) {
    if (!std::isfinite(scale) || scale < 1.0 || scale <= previous_duration_scale) {
      throw std::invalid_argument(
              "duration scale candidates must be finite, increasing, and at least 1.0");
    }
    previous_duration_scale = scale;
  }
  if (parameters.max_insertions_per_refinement == 0U) {
    throw std::invalid_argument("max insertions per refinement must be positive");
  }
}

bool sample_is_finite(const drone_mission::TrajectorySample & sample)
{
  return sample.position_world.allFinite() && sample.velocity_world.allFinite() &&
         sample.acceleration_world.allFinite() && std::isfinite(sample.yaw);
}

struct CandidateValidation
{
  bool success{false};
  TrajectoryFailureReason reason{TrajectoryFailureReason::none};
  std::size_t trajectory_segment_index{0U};
  double failure_time{0.0};
  double max_speed{0.0};
  double max_acceleration{0.0};
  std::size_t sample_count{0U};
};

bool collision_failure(TrajectoryFailureReason reason)
{
  return reason == TrajectoryFailureReason::point_collision ||
         reason == TrajectoryFailureReason::segment_collision;
}

CandidateValidation validate_candidate(
  const drone_mission::PiecewiseQuinticTrajectory & trajectory,
  const CollisionChecker & checker, const PlannedTrajectoryParameters & parameters,
  const Eigen::Vector3d & expected_start, const Eigen::Vector3d & expected_end)
{
  CandidateValidation result;
  std::optional<Eigen::Vector3d> previous_position;
  for (double time = 0.0; time < trajectory.total_duration();
    time += parameters.validation_sample_period)
  {
    const auto sample = trajectory.sample(time);
    ++result.sample_count;
    result.trajectory_segment_index = sample.segment_index;
    result.failure_time = time;
    if (!sample_is_finite(sample)) {
      result.reason = TrajectoryFailureReason::nonfinite;
      return result;
    }
    const double speed = sample.velocity_world.norm();
    const double acceleration = sample.acceleration_world.norm();
    result.max_speed = std::max(result.max_speed, speed);
    result.max_acceleration = std::max(result.max_acceleration, acceleration);
    if (!std::isfinite(speed) || !std::isfinite(acceleration)) {
      result.reason = TrajectoryFailureReason::nonfinite;
      return result;
    }
    if (checker.point_in_collision(sample.position_world)) {
      result.reason = TrajectoryFailureReason::point_collision;
      return result;
    }
    if (previous_position && checker.segment_in_collision(
        *previous_position, sample.position_world))
    {
      result.reason = TrajectoryFailureReason::segment_collision;
      return result;
    }
    if (speed > parameters.max_reference_speed) {
      result.reason = TrajectoryFailureReason::speed_limit;
      return result;
    }
    if (acceleration > parameters.max_reference_acceleration) {
      result.reason = TrajectoryFailureReason::acceleration_limit;
      return result;
    }
    previous_position = sample.position_world;
  }

  const auto final_sample = trajectory.sample(trajectory.total_duration());
  ++result.sample_count;
  result.trajectory_segment_index = final_sample.segment_index;
  result.failure_time = trajectory.total_duration();
  const double final_speed = final_sample.velocity_world.norm();
  const double final_acceleration = final_sample.acceleration_world.norm();
  result.max_speed = std::max(result.max_speed, final_speed);
  result.max_acceleration = std::max(result.max_acceleration, final_acceleration);
  if (!sample_is_finite(final_sample) || !std::isfinite(final_speed) ||
    !std::isfinite(final_acceleration))
  {
    result.reason = TrajectoryFailureReason::nonfinite;
  } else if (checker.point_in_collision(final_sample.position_world)) {
    result.reason = TrajectoryFailureReason::point_collision;
  } else if (previous_position && checker.segment_in_collision(
      *previous_position, final_sample.position_world))
  {
    result.reason = TrajectoryFailureReason::segment_collision;
  } else if (final_speed > parameters.max_reference_speed) {
    result.reason = TrajectoryFailureReason::speed_limit;
  } else if (final_acceleration > parameters.max_reference_acceleration) {
    result.reason = TrajectoryFailureReason::acceleration_limit;
  } else if (!final_sample.position_world.isApprox(expected_end, 0.0) ||
    !trajectory.sample(0.0).position_world.isApprox(expected_start, 0.0))
  {
    result.reason = TrajectoryFailureReason::endpoint_mismatch;
  } else {
    result.success = true;
    result.reason = TrajectoryFailureReason::none;
  }
  return result;
}

std::vector<double> base_segment_durations(
  const std::vector<Eigen::Vector3d> & points,
  const PlannedTrajectoryParameters & parameters)
{
  std::vector<double> durations;
  durations.reserve(points.size() - 1U);
  for (std::size_t index = 1U; index < points.size(); ++index) {
    const double length = (points[index] - points[index - 1U]).norm();
    const double duration = std::max(
      length / parameters.nominal_speed, parameters.min_segment_duration);
    if (!finite_positive(duration)) {
      throw std::invalid_argument("planned trajectory segment duration is invalid");
    }
    durations.push_back(duration);
  }
  return durations;
}

std::vector<Eigen::Vector3d> points_from_indices(
  const std::vector<Eigen::Vector3d> & raw_path,
  const std::vector<std::size_t> & indices)
{
  std::vector<Eigen::Vector3d> points;
  points.reserve(indices.size());
  for (const std::size_t index : indices) {
    points.push_back(raw_path[index]);
  }
  return points;
}

bool refine_near_collision(
  std::vector<std::size_t> & indices, std::size_t collision_segment,
  std::size_t max_insertions)
{
  if (indices.size() < 2U) {
    return false;
  }
  collision_segment = std::min(collision_segment, indices.size() - 2U);
  std::vector<std::size_t> segment_candidates;
  if (collision_segment > 0U) {
    segment_candidates.push_back(collision_segment - 1U);
  }
  segment_candidates.push_back(collision_segment);
  if (collision_segment + 1U < indices.size() - 1U) {
    segment_candidates.push_back(collision_segment + 1U);
  }

  std::vector<std::size_t> insertions;
  for (const std::size_t segment : segment_candidates) {
    const std::size_t first = indices[segment];
    const std::size_t last = indices[segment + 1U];
    if (last > first + 1U) {
      insertions.push_back(first + (last - first) / 2U);
      if (insertions.size() == max_insertions) {
        break;
      }
    }
  }
  if (insertions.empty()) {
    return false;
  }
  indices.insert(indices.end(), insertions.begin(), insertions.end());
  std::sort(indices.begin(), indices.end());
  indices.erase(std::unique(indices.begin(), indices.end()), indices.end());
  return true;
}

}  // namespace

PlannedTrajectoryBuilder::PlannedTrajectoryBuilder(
  CollisionChecker collision_checker, PlannedTrajectoryParameters parameters)
: collision_checker_(std::move(collision_checker)), parameters_(std::move(parameters))
{
  validate_parameters(parameters_);
}

PlannedTrajectoryResult PlannedTrajectoryBuilder::build(
  const std::vector<Eigen::Vector3d> & raw_path_world) const
{
  PlannedTrajectoryResult result;
  const auto initial = PathSimplifier(collision_checker_).simplify_with_indices(raw_path_world);
  result.initial_simplified_point_count = initial.points.size();
  std::vector<std::size_t> path_indices = initial.raw_indices;

  for (std::size_t refinement = 0U;
    refinement <= parameters_.max_refinement_iterations; ++refinement)
  {
    const auto points = points_from_indices(raw_path_world, path_indices);
    const auto base_durations = base_segment_durations(points, parameters_);
    std::vector<drone_mission::TrajectoryWaypoint> waypoints;
    waypoints.reserve(points.size());
    for (const auto & point : points) {
      waypoints.push_back({point, parameters_.fixed_yaw});
    }

    std::optional<CandidateValidation> first_collision;
    for (std::size_t duration_index = 0U;
      duration_index < parameters_.duration_scale_candidates.size(); ++duration_index)
    {
      if (duration_index > 0U && first_collision &&
        refinement < parameters_.max_refinement_iterations)
      {
        break;
      }
      const double duration_scale = parameters_.duration_scale_candidates[duration_index];
      std::vector<double> scaled_durations = base_durations;
      for (double & duration : scaled_durations) {
        duration *= duration_scale;
      }
      for (const double velocity_scale : parameters_.velocity_scale_candidates) {
        drone_mission::PiecewiseQuinticTrajectory trajectory(
          waypoints, scaled_durations, velocity_scale);
        const auto validation = validate_candidate(
          trajectory, collision_checker_, parameters_, raw_path_world.front(),
          raw_path_world.back());
        result.failure_reason = validation.reason;
        result.failure_segment_index = validation.trajectory_segment_index;
        result.failure_time = validation.failure_time;
        result.max_reference_speed = validation.max_speed;
        result.max_reference_acceleration = validation.max_acceleration;
        result.validation_sample_count = validation.sample_count;
        if (validation.success) {
          result.success = true;
          result.simplified_path_world = points;
          result.simplified_path_raw_indices = path_indices;
          result.segment_durations = std::move(scaled_durations);
          result.refinement_iterations = refinement;
          result.selected_velocity_scale = velocity_scale;
          result.selected_duration_scale = duration_scale;
          result.total_duration = trajectory.total_duration();
          result.failure_reason = TrajectoryFailureReason::none;
          result.trajectory = std::move(trajectory);
          return result;
        }
        if (collision_failure(validation.reason) && !first_collision) {
          first_collision = validation;
        }
      }
    }

    result.simplified_path_world = points;
    result.simplified_path_raw_indices = path_indices;
    result.segment_durations = base_durations;
    result.refinement_iterations = refinement;
    if (!first_collision || refinement == parameters_.max_refinement_iterations ||
      !refine_near_collision(
        path_indices, first_collision->trajectory_segment_index,
        parameters_.max_insertions_per_refinement))
    {
      break;
    }
  }
  return result;
}

}  // namespace drone_planning
