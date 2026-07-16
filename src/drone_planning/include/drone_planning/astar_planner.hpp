#pragma once

#include <array>
#include <cstddef>
#include <vector>

#include <Eigen/Core>

#include "drone_planning/collision_checker.hpp"
#include "drone_planning/grid_index.hpp"

namespace drone_planning
{

enum class PlanningStatus
{
  kSuccess,
  kInvalidStart,
  kInvalidGoal,
  kNoPath
};

struct AStarResult
{
  PlanningStatus status{PlanningStatus::kNoPath};
  std::vector<Eigen::Vector3d> path_world;
  double path_length{0.0};
  std::size_t expanded_nodes{0U};

  bool success() const;
};

class AStarPlanner
{
public:
  AStarPlanner(
    CollisionChecker collision_checker,
    double resolution,
    std::size_t max_grid_nodes);

  AStarResult plan(
    const Eigen::Vector3d & start_world,
    const Eigen::Vector3d & goal_world) const;

private:
  GridIndex world_to_grid(const Eigen::Vector3d & point_world) const;
  Eigen::Vector3d grid_to_world(const GridIndex & index) const;
  bool index_is_valid(const GridIndex & index) const;
  std::size_t flatten(const GridIndex & index) const;
  GridIndex unflatten(std::size_t flat_index) const;

  CollisionChecker collision_checker_;
  double resolution_{0.0};
  Eigen::Vector3d origin_{Eigen::Vector3d::Zero()};
  std::array<std::size_t, 3> dimensions_{{0U, 0U, 0U}};
  std::size_t grid_node_count_{0U};
};

}  // namespace drone_planning
