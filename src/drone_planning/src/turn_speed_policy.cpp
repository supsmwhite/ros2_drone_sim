#include "drone_planning/turn_speed_policy.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace drone_planning
{

void validate_turn_speed_policy(const TurnSpeedPolicyParameters & parameters)
{
  if (!std::isfinite(parameters.mild_turn_angle_rad) ||
    !std::isfinite(parameters.sharp_turn_angle_rad) ||
    parameters.mild_turn_angle_rad <= 0.0 ||
    parameters.sharp_turn_angle_rad <= parameters.mild_turn_angle_rad ||
    parameters.sharp_turn_angle_rad >= 3.14159265358979323846 ||
    !std::isfinite(parameters.mild_turn_scale) ||
    !std::isfinite(parameters.sharp_turn_scale) ||
    parameters.sharp_turn_scale <= 0.0 ||
    parameters.mild_turn_scale <= parameters.sharp_turn_scale ||
    parameters.mild_turn_scale > 1.0)
  {
    throw std::invalid_argument("turn speed policy parameters are invalid");
  }
}

double turn_speed_scale(
  const Eigen::Vector3d & previous, const Eigen::Vector3d & current,
  const Eigen::Vector3d & next, const TurnSpeedPolicyParameters & parameters)
{
  validate_turn_speed_policy(parameters);
  const Eigen::Vector3d incoming = current - previous;
  const Eigen::Vector3d outgoing = next - current;
  const double incoming_norm = incoming.norm();
  const double outgoing_norm = outgoing.norm();
  if (!incoming.allFinite() || !outgoing.allFinite() ||
    !std::isfinite(incoming_norm) || !std::isfinite(outgoing_norm) ||
    incoming_norm <= 1.0e-9 || outgoing_norm <= 1.0e-9)
  {
    throw std::invalid_argument("turn speed policy points must define finite nonzero segments");
  }
  const double cosine = std::clamp(
    incoming.dot(outgoing) / (incoming_norm * outgoing_norm), -1.0, 1.0);
  const double angle = std::acos(cosine);
  if (angle >= parameters.sharp_turn_angle_rad) {
    return parameters.sharp_turn_scale;
  }
  if (angle >= parameters.mild_turn_angle_rad) {
    return parameters.mild_turn_scale;
  }
  return 1.0;
}

double segment_turn_speed_scale(
  bool enabled, const Eigen::Vector3d & previous, const Eigen::Vector3d & current,
  const std::optional<Eigen::Vector3d> & next,
  const TurnSpeedPolicyParameters & parameters)
{
  if (!enabled || !next) {
    return 1.0;
  }
  return turn_speed_scale(previous, current, *next, parameters);
}

}  // namespace drone_planning
