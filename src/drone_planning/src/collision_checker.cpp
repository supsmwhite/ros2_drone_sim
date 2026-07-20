#include "drone_planning/collision_checker.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace drone_planning
{

CollisionChecker::CollisionChecker(StaticEnvironment environment, double safety_radius)
: environment_(std::move(environment)), safety_radius_(safety_radius)
{
  if (!std::isfinite(safety_radius_) || safety_radius_ < 0.0) {
    throw std::invalid_argument("safety radius must be finite and non-negative");
  }

  const Eigen::Vector3d margin = Eigen::Vector3d::Constant(safety_radius_);
  safe_workspace_.min_corner = environment_.workspace().min_corner + margin;
  safe_workspace_.max_corner = environment_.workspace().max_corner - margin;
  if (!safe_workspace_.min_corner.allFinite() || !safe_workspace_.max_corner.allFinite() ||
    (safe_workspace_.min_corner.array() >= safe_workspace_.max_corner.array()).any())
  {
    throw std::invalid_argument("safety radius leaves no valid workspace interior");
  }

  inflated_obstacles_.reserve(environment_.obstacles().size());
  for (const auto & obstacle : environment_.obstacles()) {
    AxisAlignedBox inflated;
    inflated.min_corner = obstacle.min_corner - margin;
    inflated.max_corner = obstacle.max_corner + margin;
    if (!inflated.min_corner.allFinite() || !inflated.max_corner.allFinite()) {
      throw std::invalid_argument("inflated obstacle bounds must remain finite");
    }
    inflated_obstacles_.push_back(inflated);
  }
}

bool CollisionChecker::point_in_collision(const Eigen::Vector3d & point_world) const
{
  if (!point_world.allFinite()) {
    return true;
  }

  if ((point_world.array() <= safe_workspace_.min_corner.array()).any() ||
    (point_world.array() >= safe_workspace_.max_corner.array()).any())
  {
    return true;
  }

  return std::any_of(
    inflated_obstacles_.begin(), inflated_obstacles_.end(),
    [&point_world](const AxisAlignedBox & obstacle) {
      return point_inside_closed_box(point_world, obstacle);
    });
}

bool CollisionChecker::segment_in_collision(
  const Eigen::Vector3d & start_world, const Eigen::Vector3d & end_world) const
{
  if (!start_world.allFinite() || !end_world.allFinite()) {
    return true;
  }
  if (point_in_collision(start_world) || point_in_collision(end_world)) {
    return true;
  }

  return std::any_of(
    inflated_obstacles_.begin(), inflated_obstacles_.end(),
    [&start_world, &end_world](const AxisAlignedBox & obstacle) {
      return segment_intersects_closed_box(start_world, end_world, obstacle);
    });
}

bool CollisionChecker::segment_respects_additional_clearance(
  const Eigen::Vector3d & start_world, const Eigen::Vector3d & end_world,
  double additional_clearance) const
{
  if (!std::isfinite(additional_clearance) || additional_clearance < 0.0) {
    throw std::invalid_argument("additional clearance must be finite and non-negative");
  }
  if (segment_in_collision(start_world, end_world)) {
    return false;
  }

  const double preferred_radius = safety_radius_ + additional_clearance;
  if (!std::isfinite(preferred_radius)) {
    throw std::invalid_argument("combined preferred clearance must remain finite");
  }
  const Eigen::Vector3d margin = Eigen::Vector3d::Constant(preferred_radius);
  for (const auto & obstacle : environment_.obstacles()) {
    const AxisAlignedBox preferred_obstacle{
      obstacle.min_corner - margin,
      obstacle.max_corner + margin};
    if (!preferred_obstacle.min_corner.allFinite() ||
      !preferred_obstacle.max_corner.allFinite())
    {
      throw std::invalid_argument("preferred obstacle bounds must remain finite");
    }
    if (segment_intersects_closed_box(start_world, end_world, preferred_obstacle)) {
      return false;
    }
  }
  return true;
}

const StaticEnvironment & CollisionChecker::environment() const
{
  return environment_;
}

double CollisionChecker::safety_radius() const
{
  return safety_radius_;
}

const AxisAlignedBox & CollisionChecker::safe_workspace() const
{
  return safe_workspace_;
}

const std::vector<AxisAlignedBox> & CollisionChecker::inflated_obstacles() const
{
  return inflated_obstacles_;
}

bool CollisionChecker::point_inside_closed_box(
  const Eigen::Vector3d & point, const AxisAlignedBox & box)
{
  return (point.array() >= box.min_corner.array()).all() &&
         (point.array() <= box.max_corner.array()).all();
}

bool CollisionChecker::segment_intersects_closed_box(
  const Eigen::Vector3d & start, const Eigen::Vector3d & end,
  const AxisAlignedBox & box)
{
  const Eigen::Vector3d direction = end - start;
  double enter = 0.0;
  double exit = 1.0;
  for (Eigen::Index axis = 0; axis < 3; ++axis) {
    if (direction[axis] == 0.0) {
      if (start[axis] < box.min_corner[axis] || start[axis] > box.max_corner[axis]) {
        return false;
      }
      continue;
    }

    double first = (box.min_corner[axis] - start[axis]) / direction[axis];
    double second = (box.max_corner[axis] - start[axis]) / direction[axis];
    if (first > second) {
      std::swap(first, second);
    }
    enter = std::max(enter, first);
    exit = std::min(exit, second);
    if (enter > exit) {
      return false;
    }
  }
  return true;
}

}  // namespace drone_planning
