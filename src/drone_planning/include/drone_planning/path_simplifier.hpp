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
  std::size_t preferred_shortcut_count{0U};
  std::size_t fallback_shortcut_count{0U};
  std::size_t collision_only_shortcut_count{0U};
  bool clearance_preference_enabled{false};
};

class PathSimplifier
{
public:
  explicit PathSimplifier(
    CollisionChecker collision_checker,
    double shortcut_preferred_clearance = 0.0);

  std::vector<Eigen::Vector3d> simplify(
    const std::vector<Eigen::Vector3d> & path_world) const;

  SimplifiedPathResult simplify_with_indices(
    const std::vector<Eigen::Vector3d> & path_world) const;

private:
  CollisionChecker collision_checker_;
  double shortcut_preferred_clearance_{0.0};
};

}  // namespace drone_planning
