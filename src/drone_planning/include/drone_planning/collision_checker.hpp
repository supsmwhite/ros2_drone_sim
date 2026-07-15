#pragma once

#include <vector>

#include <Eigen/Core>

#include "drone_planning/static_environment.hpp"

namespace drone_planning
{

class CollisionChecker
{
public:
  CollisionChecker(StaticEnvironment environment, double safety_radius);

  bool point_in_collision(const Eigen::Vector3d & point_world) const;
  bool segment_in_collision(
    const Eigen::Vector3d & start_world,
    const Eigen::Vector3d & end_world) const;

  const StaticEnvironment & environment() const;
  double safety_radius() const;
  const AxisAlignedBox & safe_workspace() const;
  const std::vector<AxisAlignedBox> & inflated_obstacles() const;

private:
  static bool point_inside_closed_box(
    const Eigen::Vector3d & point, const AxisAlignedBox & box);
  static bool segment_intersects_closed_box(
    const Eigen::Vector3d & start, const Eigen::Vector3d & end,
    const AxisAlignedBox & box);

  StaticEnvironment environment_;
  double safety_radius_{0.0};
  AxisAlignedBox safe_workspace_;
  std::vector<AxisAlignedBox> inflated_obstacles_;
};

}  // namespace drone_planning
