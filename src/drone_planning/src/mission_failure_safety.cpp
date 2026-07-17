#include "drone_planning/mission_failure_safety.hpp"

namespace drone_planning
{

std::optional<FailureHoldCommand> make_failure_hold_command(
  bool flight_started, const std::optional<Eigen::Vector3d> & safe_hold_position)
{
  if (!flight_started || !safe_hold_position || !safe_hold_position->allFinite()) {
    return std::nullopt;
  }
  FailureHoldCommand command;
  command.position_world = *safe_hold_position;
  return command;
}

}  // namespace drone_planning
