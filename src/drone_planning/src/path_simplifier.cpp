#include "drone_planning/path_simplifier.hpp"

#include <cstddef>
#include <stdexcept>
#include <utility>

namespace drone_planning
{

PathSimplifier::PathSimplifier(CollisionChecker collision_checker)
: collision_checker_(std::move(collision_checker))
{
}

std::vector<Eigen::Vector3d> PathSimplifier::simplify(
  const std::vector<Eigen::Vector3d> & path_world) const
{
  return simplify_with_indices(path_world).points;
}

SimplifiedPathResult PathSimplifier::simplify_with_indices(
  const std::vector<Eigen::Vector3d> & path_world) const
{
  if (path_world.size() < 2U) {
    throw std::invalid_argument("path simplification requires at least two points");
  }
  for (std::size_t index = 0U; index < path_world.size(); ++index) {
    if (!path_world[index].allFinite()) {
      throw std::invalid_argument("all path points must be finite");
    }
    if (collision_checker_.point_in_collision(path_world[index])) {
      throw std::invalid_argument("path contains a colliding point");
    }
    if (index > 0U && collision_checker_.segment_in_collision(
        path_world[index - 1U], path_world[index]))
    {
      throw std::invalid_argument("path contains a colliding segment");
    }
  }

  SimplifiedPathResult result;
  result.points.reserve(path_world.size());
  result.raw_indices.reserve(path_world.size());
  result.points.push_back(path_world.front());
  result.raw_indices.push_back(0U);
  std::size_t anchor = 0U;
  while (anchor + 1U < path_world.size()) {
    std::size_t visible = path_world.size() - 1U;
    while (visible > anchor + 1U && collision_checker_.segment_in_collision(
        path_world[anchor], path_world[visible]))
    {
      --visible;
    }
    if (collision_checker_.segment_in_collision(path_world[anchor], path_world[visible])) {
      throw std::logic_error("validated path has no safe successor");
    }
    result.points.push_back(path_world[visible]);
    result.raw_indices.push_back(visible);
    anchor = visible;
  }
  return result;
}

}  // namespace drone_planning
