#pragma once

#include <optional>

#include <Eigen/Core>

namespace drone_planning
{

struct TurnSpeedPolicyParameters
{
  double mild_turn_angle_rad{0.5235987755982988};
  double sharp_turn_angle_rad{1.0471975511965976};
  double mild_turn_scale{0.90};
  double sharp_turn_scale{0.80};
};

void validate_turn_speed_policy(const TurnSpeedPolicyParameters & parameters);

double turn_speed_scale(
  const Eigen::Vector3d & previous, const Eigen::Vector3d & current,
  const Eigen::Vector3d & next, const TurnSpeedPolicyParameters & parameters);

double segment_turn_speed_scale(
  bool enabled, const Eigen::Vector3d & previous, const Eigen::Vector3d & current,
  const std::optional<Eigen::Vector3d> & next,
  const TurnSpeedPolicyParameters & parameters);

}  // namespace drone_planning
