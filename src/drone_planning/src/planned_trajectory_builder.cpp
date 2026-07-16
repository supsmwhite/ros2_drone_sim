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
}

bool sample_is_finite(const drone_mission::TrajectorySample & sample)
{
  return sample.position_world.allFinite() && sample.velocity_world.allFinite() &&
         sample.acceleration_world.allFinite() && std::isfinite(sample.yaw);
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
  result.simplified_path_world = PathSimplifier(collision_checker_).simplify(raw_path_world);
  result.segment_durations.reserve(result.simplified_path_world.size() - 1U);
  for (std::size_t index = 1U; index < result.simplified_path_world.size(); ++index) {
    const double length =
      (result.simplified_path_world[index] - result.simplified_path_world[index - 1U]).norm();
    const double duration = std::max(
      length / parameters_.nominal_speed, parameters_.min_segment_duration);
    if (!finite_positive(duration)) {
      throw std::invalid_argument("planned trajectory segment duration is invalid");
    }
    result.segment_durations.push_back(duration);
  }

  std::vector<drone_mission::TrajectoryWaypoint> waypoints;
  waypoints.reserve(result.simplified_path_world.size());
  for (const auto & point : result.simplified_path_world) {
    waypoints.push_back({point, parameters_.fixed_yaw});
  }

  for (const double scale : parameters_.velocity_scale_candidates) {
    drone_mission::PiecewiseQuinticTrajectory trajectory(
      waypoints, result.segment_durations, scale);
    bool valid = true;
    double max_speed = 0.0;
    double max_acceleration = 0.0;
    std::size_t sample_count = 0U;
    std::optional<Eigen::Vector3d> previous_position;
    for (double time = 0.0; time < trajectory.total_duration();
      time += parameters_.validation_sample_period)
    {
      const auto sample = trajectory.sample(time);
      ++sample_count;
      if (!sample_is_finite(sample)) {
        valid = false;
        break;
      }
      const double speed = sample.velocity_world.norm();
      const double acceleration = sample.acceleration_world.norm();
      max_speed = std::max(max_speed, speed);
      max_acceleration = std::max(max_acceleration, acceleration);
      if (!std::isfinite(speed) || !std::isfinite(acceleration) ||
        speed > parameters_.max_reference_speed ||
        acceleration > parameters_.max_reference_acceleration ||
        collision_checker_.point_in_collision(sample.position_world) ||
        (previous_position && collision_checker_.segment_in_collision(
          *previous_position, sample.position_world)))
      {
        valid = false;
        break;
      }
      previous_position = sample.position_world;
    }
    if (valid) {
      const auto final_sample = trajectory.sample(trajectory.total_duration());
      ++sample_count;
      const double final_speed = final_sample.velocity_world.norm();
      const double final_acceleration = final_sample.acceleration_world.norm();
      max_speed = std::max(max_speed, final_speed);
      max_acceleration = std::max(max_acceleration, final_acceleration);
      valid = sample_is_finite(final_sample) && std::isfinite(final_speed) &&
        std::isfinite(final_acceleration) &&
        final_speed <= parameters_.max_reference_speed &&
        final_acceleration <= parameters_.max_reference_acceleration &&
        !collision_checker_.point_in_collision(final_sample.position_world) &&
        (!previous_position || !collision_checker_.segment_in_collision(
          *previous_position, final_sample.position_world)) &&
        final_sample.position_world.isApprox(raw_path_world.back(), 0.0) &&
        trajectory.sample(0.0).position_world.isApprox(raw_path_world.front(), 0.0);
    }
    if (valid) {
      result.success = true;
      result.selected_velocity_scale = scale;
      result.total_duration = trajectory.total_duration();
      result.max_reference_speed = max_speed;
      result.max_reference_acceleration = max_acceleration;
      result.validation_sample_count = sample_count;
      result.trajectory = std::move(trajectory);
      return result;
    }
  }
  return result;
}

}  // namespace drone_planning
