#pragma once

#include <vector>

#include <Eigen/Core>

#include "drone_planning/collision_checker.hpp"

namespace drone_planning
{

class PathSimplifier
{
public:
  explicit PathSimplifier(CollisionChecker collision_checker);

  std::vector<Eigen::Vector3d> simplify(
    const std::vector<Eigen::Vector3d> & path_world) const;

private:
  CollisionChecker collision_checker_;
};

}  // namespace drone_planning
