#pragma once

#include <optional>

#include <Eigen/Core>

namespace drone_planning
{

struct FailureHoldCommand
{
  Eigen::Vector3d position_world{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity_world{Eigen::Vector3d::Zero()};
  Eigen::Vector3d acceleration_world{Eigen::Vector3d::Zero()};
};

std::optional<FailureHoldCommand> make_failure_hold_command(
  bool flight_started, const std::optional<Eigen::Vector3d> & safe_hold_position);

}  // namespace drone_planning
