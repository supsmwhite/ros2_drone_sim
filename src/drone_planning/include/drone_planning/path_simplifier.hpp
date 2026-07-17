#pragma once

#include <cstddef>
#include <vector>

#include <Eigen/Core>

#include "drone_planning/collision_checker.hpp"

namespace drone_planning
{

struct SimplifiedPathResult
{
  std::vector<Eigen::Vector3d> points;
  std::vector<std::size_t> raw_indices;
};

class PathSimplifier
{
public:
  explicit PathSimplifier(CollisionChecker collision_checker);

  std::vector<Eigen::Vector3d> simplify(
    const std::vector<Eigen::Vector3d> & path_world) const;

  SimplifiedPathResult simplify_with_indices(
    const std::vector<Eigen::Vector3d> & path_world) const;

private:
  CollisionChecker collision_checker_;
};

}  // namespace drone_planning
