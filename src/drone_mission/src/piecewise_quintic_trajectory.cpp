#include "drone_mission/piecewise_quintic_trajectory.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace drone_mission
{
namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;
constexpr double kHalfTurnTolerance = 1.0e-12;

bool finite_positive(double value)
{
  return std::isfinite(value) && value > 0.0;
}

}  // namespace

PiecewiseQuinticTrajectory::PiecewiseQuinticTrajectory(
  std::vector<TrajectoryWaypoint> waypoints,
  std::vector<double> segment_durations,
  double intermediate_velocity_scale)
: waypoints_(std::move(waypoints))
{
  if (waypoints_.size() < 2U) {
    throw std::invalid_argument("trajectory requires at least two waypoints");
  }
  if (segment_durations.size() + 1U != waypoints_.size()) {
    throw std::invalid_argument("segment duration count must equal waypoint count minus one");
  }
  for (const auto & waypoint : waypoints_) {
    if (!waypoint.position_world.allFinite() || !std::isfinite(waypoint.yaw)) {
      throw std::invalid_argument("all trajectory waypoint values must be finite");
    }
  }
  for (const double duration : segment_durations) {
    if (!finite_positive(duration)) {
      throw std::invalid_argument("all segment durations must be finite and positive");
    }
  }
  if (!std::isfinite(intermediate_velocity_scale) ||
    intermediate_velocity_scale < 0.0 || intermediate_velocity_scale > 1.0)
  {
    throw std::invalid_argument("intermediate velocity scale must be finite and in [0, 1]");
  }

  std::vector<Eigen::Vector3d> waypoint_velocities(
    waypoints_.size(), Eigen::Vector3d::Zero());
  for (std::size_t index = 1U; index + 1U < waypoints_.size(); ++index) {
    const Eigen::Vector3d incoming =
      (waypoints_[index].position_world - waypoints_[index - 1U].position_world) /
      segment_durations[index - 1U];
    const Eigen::Vector3d outgoing =
      (waypoints_[index + 1U].position_world - waypoints_[index].position_world) /
      segment_durations[index];
    waypoint_velocities[index] =
      intermediate_velocity_scale * 0.5 * (incoming + outgoing);
  }

  std::vector<double> unwrapped_yaw(waypoints_.size(), waypoints_.front().yaw);
  for (std::size_t index = 1U; index < waypoints_.size(); ++index) {
    const double raw_delta = waypoints_[index].yaw - waypoints_[index - 1U].yaw;
    unwrapped_yaw[index] =
      unwrapped_yaw[index - 1U] + deterministic_shortest_delta(raw_delta);
  }
  std::vector<double> yaw_rates(waypoints_.size(), 0.0);
  for (std::size_t index = 1U; index + 1U < waypoints_.size(); ++index) {
    const double incoming =
      (unwrapped_yaw[index] - unwrapped_yaw[index - 1U]) /
      segment_durations[index - 1U];
    const double outgoing =
      (unwrapped_yaw[index + 1U] - unwrapped_yaw[index]) /
      segment_durations[index];
    yaw_rates[index] = 0.5 * (incoming + outgoing);
  }

  segments_.reserve(segment_durations.size());
  for (std::size_t index = 0U; index < segment_durations.size(); ++index) {
    Segment segment;
    segment.duration = segment_durations[index];
    segment.start_time = total_duration_;
    for (Eigen::Index axis = 0; axis < 3; ++axis) {
      const auto coefficients = quintic_coefficients(
        waypoints_[index].position_world[axis], waypoint_velocities[index][axis], 0.0,
        waypoints_[index + 1U].position_world[axis],
        waypoint_velocities[index + 1U][axis], 0.0, segment.duration);
      for (std::size_t coefficient = 0U; coefficient < coefficients.size(); ++coefficient) {
        if (!std::isfinite(coefficients[coefficient])) {
          throw std::invalid_argument("trajectory coefficients must remain finite");
        }
        segment.position_coefficients[coefficient][axis] = coefficients[coefficient];
      }
    }
    segment.yaw_coefficients = quintic_coefficients(
      unwrapped_yaw[index], yaw_rates[index], 0.0,
      unwrapped_yaw[index + 1U], yaw_rates[index + 1U], 0.0, segment.duration);
    if (!std::all_of(
        segment.yaw_coefficients.begin(), segment.yaw_coefficients.end(),
        [](double coefficient) {return std::isfinite(coefficient);}))
    {
      throw std::invalid_argument("trajectory yaw coefficients must remain finite");
    }
    segments_.push_back(segment);
    total_duration_ += segment.duration;
    if (!std::isfinite(total_duration_)) {
      throw std::invalid_argument("trajectory total duration must remain finite");
    }
  }
}

TrajectorySample PiecewiseQuinticTrajectory::sample(double elapsed_time) const
{
  if (!std::isfinite(elapsed_time)) {
    throw std::invalid_argument("elapsed time must be finite");
  }
  if (elapsed_time >= total_duration_) {
    TrajectorySample result;
    result.position_world = waypoints_.back().position_world;
    result.yaw = segments_.back().yaw_coefficients[0];
    const auto & yaw_coefficients = segments_.back().yaw_coefficients;
    const double t = segments_.back().duration;
    result.yaw = yaw_coefficients[0] + yaw_coefficients[1] * t +
      yaw_coefficients[2] * t * t + yaw_coefficients[3] * t * t * t +
      yaw_coefficients[4] * t * t * t * t +
      yaw_coefficients[5] * t * t * t * t * t;
    result.segment_index = segments_.size() - 1U;
    result.complete = true;
    return result;
  }

  const double bounded_time = std::max(0.0, elapsed_time);
  std::size_t segment_index = 0U;
  while (segment_index + 1U < segments_.size() &&
    bounded_time >= segments_[segment_index].start_time + segments_[segment_index].duration)
  {
    ++segment_index;
  }
  const Segment & segment = segments_[segment_index];
  const double t = bounded_time - segment.start_time;
  const double t2 = t * t;
  const double t3 = t2 * t;
  const double t4 = t3 * t;
  const double t5 = t4 * t;

  TrajectorySample result;
  result.segment_index = segment_index;
  for (Eigen::Index axis = 0; axis < 3; ++axis) {
    const auto coefficient = [&segment, axis](std::size_t index) {
        return segment.position_coefficients[index][axis];
      };
    result.position_world[axis] = coefficient(0) + coefficient(1) * t +
      coefficient(2) * t2 + coefficient(3) * t3 + coefficient(4) * t4 +
      coefficient(5) * t5;
    result.velocity_world[axis] = coefficient(1) + 2.0 * coefficient(2) * t +
      3.0 * coefficient(3) * t2 + 4.0 * coefficient(4) * t3 +
      5.0 * coefficient(5) * t4;
    result.acceleration_world[axis] = 2.0 * coefficient(2) +
      6.0 * coefficient(3) * t + 12.0 * coefficient(4) * t2 +
      20.0 * coefficient(5) * t3;
  }
  const auto & yaw = segment.yaw_coefficients;
  result.yaw = yaw[0] + yaw[1] * t + yaw[2] * t2 + yaw[3] * t3 +
    yaw[4] * t4 + yaw[5] * t5;
  return result;
}

std::size_t PiecewiseQuinticTrajectory::segment_count() const
{
  return segments_.size();
}

double PiecewiseQuinticTrajectory::total_duration() const
{
  return total_duration_;
}

std::array<double, 6> PiecewiseQuinticTrajectory::quintic_coefficients(
  double start_position, double start_velocity, double start_acceleration,
  double end_position, double end_velocity, double end_acceleration,
  double duration)
{
  const double duration2 = duration * duration;
  const double duration3 = duration2 * duration;
  const double duration4 = duration3 * duration;
  const double duration5 = duration4 * duration;
  const double c0 = start_position;
  const double c1 = start_velocity;
  const double c2 = 0.5 * start_acceleration;
  const double position_remainder = end_position - (c0 + c1 * duration + c2 * duration2);
  const double velocity_remainder = end_velocity - (c1 + 2.0 * c2 * duration);
  const double acceleration_remainder = end_acceleration - 2.0 * c2;
  return {
    c0,
    c1,
    c2,
    10.0 * position_remainder / duration3 - 4.0 * velocity_remainder / duration2 +
      0.5 * acceleration_remainder / duration,
    -15.0 * position_remainder / duration4 + 7.0 * velocity_remainder / duration3 -
      acceleration_remainder / duration2,
    6.0 * position_remainder / duration5 - 3.0 * velocity_remainder / duration4 +
      0.5 * acceleration_remainder / duration3};
}

double PiecewiseQuinticTrajectory::deterministic_shortest_delta(double target_minus_current)
{
  double delta = std::remainder(target_minus_current, kTwoPi);
  if (std::abs(std::abs(delta) - kPi) <= kHalfTurnTolerance) {
    delta = kPi;
  }
  return delta;
}

}  // namespace drone_mission
