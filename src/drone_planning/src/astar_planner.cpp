#include "drone_planning/astar_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <stdexcept>
#include <utility>

namespace drone_planning
{
namespace
{

constexpr std::size_t kNoParent = std::numeric_limits<std::size_t>::max();

struct OpenEntry
{
  double f{0.0};
  double h{0.0};
  double g{0.0};
  GridIndex index;
  std::size_t flat_index{0U};
};

struct OpenEntryHasLowerPriority
{
  bool operator()(const OpenEntry & lhs, const OpenEntry & rhs) const
  {
    if (lhs.f != rhs.f) {
      return lhs.f > rhs.f;
    }
    if (lhs.h != rhs.h) {
      return lhs.h > rhs.h;
    }
    if (lhs.index.x != rhs.index.x) {
      return lhs.index.x > rhs.index.x;
    }
    if (lhs.index.y != rhs.index.y) {
      return lhs.index.y > rhs.index.y;
    }
    return lhs.index.z > rhs.index.z;
  }
};

bool same_point(const Eigen::Vector3d & lhs, const Eigen::Vector3d & rhs)
{
  return lhs == rhs;
}

}  // namespace

bool AStarResult::success() const
{
  return status == PlanningStatus::kSuccess;
}

AStarPlanner::AStarPlanner(
  CollisionChecker collision_checker, double resolution, std::size_t max_grid_nodes)
: collision_checker_(std::move(collision_checker)), resolution_(resolution),
  origin_(collision_checker_.safe_workspace().min_corner)
{
  if (!std::isfinite(resolution_) || resolution_ <= 0.0) {
    throw std::invalid_argument("resolution must be finite and positive");
  }
  if (max_grid_nodes == 0U) {
    throw std::invalid_argument("max_grid_nodes must be positive");
  }

  const Eigen::Vector3d span =
    collision_checker_.safe_workspace().max_corner - origin_;
  grid_node_count_ = 1U;
  for (Eigen::Index axis = 0; axis < 3; ++axis) {
    const double intervals = std::floor(span[axis] / resolution_);
    if (!std::isfinite(intervals) || intervals < 0.0 ||
      intervals >= static_cast<double>(std::numeric_limits<int>::max()))
    {
      throw std::invalid_argument("grid dimension is not representable");
    }
    const std::size_t dimension = static_cast<std::size_t>(intervals) + 1U;
    dimensions_[static_cast<std::size_t>(axis)] = dimension;
    if (grid_node_count_ > max_grid_nodes / dimension) {
      throw std::invalid_argument("grid node count exceeds max_grid_nodes");
    }
    grid_node_count_ *= dimension;
  }
  if (grid_node_count_ > max_grid_nodes) {
    throw std::invalid_argument("grid node count exceeds max_grid_nodes");
  }
}

AStarResult AStarPlanner::plan(
  const Eigen::Vector3d & start_world, const Eigen::Vector3d & goal_world) const
{
  AStarResult result;
  if (collision_checker_.point_in_collision(start_world)) {
    result.status = PlanningStatus::kInvalidStart;
    return result;
  }
  if (collision_checker_.point_in_collision(goal_world)) {
    result.status = PlanningStatus::kInvalidGoal;
    return result;
  }

  const GridIndex start_index = world_to_grid(start_world);
  const GridIndex goal_index = world_to_grid(goal_world);
  if (!index_is_valid(start_index)) {
    result.status = PlanningStatus::kInvalidStart;
    return result;
  }
  if (!index_is_valid(goal_index)) {
    result.status = PlanningStatus::kInvalidGoal;
    return result;
  }

  const Eigen::Vector3d snapped_start = grid_to_world(start_index);
  const Eigen::Vector3d snapped_goal = grid_to_world(goal_index);
  if (collision_checker_.point_in_collision(snapped_start) ||
    collision_checker_.segment_in_collision(start_world, snapped_start))
  {
    result.status = PlanningStatus::kInvalidStart;
    return result;
  }
  if (collision_checker_.point_in_collision(snapped_goal) ||
    collision_checker_.segment_in_collision(snapped_goal, goal_world))
  {
    result.status = PlanningStatus::kInvalidGoal;
    return result;
  }

  const double infinity = std::numeric_limits<double>::infinity();
  std::vector<double> g_score(grid_node_count_, infinity);
  std::vector<std::size_t> parent(grid_node_count_, kNoParent);
  std::vector<bool> closed(grid_node_count_, false);
  std::priority_queue<OpenEntry, std::vector<OpenEntry>, OpenEntryHasLowerPriority> open;

  const std::size_t start_flat = flatten(start_index);
  const std::size_t goal_flat = flatten(goal_index);
  const double start_h = (snapped_start - snapped_goal).norm();
  g_score[start_flat] = 0.0;
  open.push({start_h, start_h, 0.0, start_index, start_flat});

  bool found = false;
  while (!open.empty()) {
    const OpenEntry current = open.top();
    open.pop();
    if (closed[current.flat_index] || current.g != g_score[current.flat_index]) {
      continue;
    }
    closed[current.flat_index] = true;
    ++result.expanded_nodes;
    if (current.flat_index == goal_flat) {
      found = true;
      break;
    }

    const Eigen::Vector3d current_world = grid_to_world(current.index);
    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dz = -1; dz <= 1; ++dz) {
          if (dx == 0 && dy == 0 && dz == 0) {
            continue;
          }
          const GridIndex neighbor{
            current.index.x + dx,
            current.index.y + dy,
            current.index.z + dz};
          if (!index_is_valid(neighbor)) {
            continue;
          }
          const std::size_t neighbor_flat = flatten(neighbor);
          if (closed[neighbor_flat]) {
            continue;
          }
          const Eigen::Vector3d neighbor_world = grid_to_world(neighbor);
          if (collision_checker_.point_in_collision(neighbor_world) ||
            collision_checker_.segment_in_collision(current_world, neighbor_world))
          {
            continue;
          }

          const double step_cost = resolution_ * std::sqrt(
            static_cast<double>(dx * dx + dy * dy + dz * dz));
          const double tentative_g = current.g + step_cost;
          if (tentative_g >= g_score[neighbor_flat]) {
            continue;
          }
          const double h = (neighbor_world - snapped_goal).norm();
          g_score[neighbor_flat] = tentative_g;
          parent[neighbor_flat] = current.flat_index;
          open.push({tentative_g + h, h, tentative_g, neighbor, neighbor_flat});
        }
      }
    }
  }

  if (!found) {
    result.status = PlanningStatus::kNoPath;
    return result;
  }

  std::vector<Eigen::Vector3d> grid_path_reversed;
  for (std::size_t flat = goal_flat;; flat = parent[flat]) {
    grid_path_reversed.push_back(grid_to_world(unflatten(flat)));
    if (flat == start_flat) {
      break;
    }
    if (parent[flat] == kNoParent) {
      throw std::logic_error("A* parent chain is incomplete");
    }
  }
  std::reverse(grid_path_reversed.begin(), grid_path_reversed.end());

  result.path_world.reserve(grid_path_reversed.size() + 2U);
  result.path_world.push_back(start_world);
  for (const auto & point : grid_path_reversed) {
    if (!same_point(result.path_world.back(), point)) {
      result.path_world.push_back(point);
    }
  }
  if (!same_point(result.path_world.back(), goal_world)) {
    result.path_world.push_back(goal_world);
  }
  result.path_length = 0.0;
  for (std::size_t index = 1U; index < result.path_world.size(); ++index) {
    result.path_length += (result.path_world[index] - result.path_world[index - 1U]).norm();
  }
  result.status = PlanningStatus::kSuccess;
  return result;
}

GridIndex AStarPlanner::world_to_grid(const Eigen::Vector3d & point_world) const
{
  const Eigen::Vector3d continuous = (point_world - origin_) / resolution_;
  return {
    static_cast<int>(std::lround(continuous.x())),
    static_cast<int>(std::lround(continuous.y())),
    static_cast<int>(std::lround(continuous.z()))};
}

Eigen::Vector3d AStarPlanner::grid_to_world(const GridIndex & index) const
{
  return origin_ + resolution_ * Eigen::Vector3d(index.x, index.y, index.z);
}

bool AStarPlanner::index_is_valid(const GridIndex & index) const
{
  return index.x >= 0 && index.y >= 0 && index.z >= 0 &&
         static_cast<std::size_t>(index.x) < dimensions_[0] &&
         static_cast<std::size_t>(index.y) < dimensions_[1] &&
         static_cast<std::size_t>(index.z) < dimensions_[2];
}

std::size_t AStarPlanner::flatten(const GridIndex & index) const
{
  return (static_cast<std::size_t>(index.x) * dimensions_[1] +
         static_cast<std::size_t>(index.y)) * dimensions_[2] +
         static_cast<std::size_t>(index.z);
}

GridIndex AStarPlanner::unflatten(std::size_t flat_index) const
{
  const std::size_t z = flat_index % dimensions_[2];
  flat_index /= dimensions_[2];
  const std::size_t y = flat_index % dimensions_[1];
  const std::size_t x = flat_index / dimensions_[1];
  return {
    static_cast<int>(x), static_cast<int>(y), static_cast<int>(z)};
}

}  // namespace drone_planning
