#include "drone_planning/goal_completion_gate.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

namespace drone_planning
{
namespace
{

constexpr double kTwoPi = 2.0 * 3.14159265358979323846;

bool finite_positive(double value)
{
  return std::isfinite(value) && value > 0.0;
}

}  // namespace

double shortest_yaw_error(double target_yaw, double actual_yaw)
{
  return std::remainder(target_yaw - actual_yaw, kTwoPi);
}

double goal_acceptance_target_yaw(
  YawMode mode, double fixed_yaw, double mission_goal_yaw)
{
  return mode == YawMode::Fixed ? fixed_yaw : mission_goal_yaw;
}

GoalCompletionGate::GoalCompletionGate(GoalCompletionTolerances tolerances)
: tolerances_(tolerances)
{
  if (!finite_positive(tolerances_.position) || !finite_positive(tolerances_.speed) ||
    !finite_positive(tolerances_.yaw) || !finite_positive(tolerances_.angular_speed) ||
    !finite_positive(tolerances_.hold_duration))
  {
    throw std::invalid_argument("goal completion tolerances must be finite and positive");
  }
}

GoalCompletionEvaluation GoalCompletionGate::update(
  const GoalCompletionSample & sample, double target_yaw, double dt)
{
  GoalCompletionEvaluation result;
  result.yaw_error = std::abs(shortest_yaw_error(target_yaw, sample.actual_yaw));
  const bool finite_sample =
    std::isfinite(sample.position_error) && sample.position_error >= 0.0 &&
    std::isfinite(sample.speed) && sample.speed >= 0.0 &&
    std::isfinite(sample.actual_yaw) && std::isfinite(target_yaw) &&
    std::isfinite(result.yaw_error) && std::isfinite(sample.angular_speed) &&
    sample.angular_speed >= 0.0 && finite_positive(dt);
  result.settled = finite_sample && sample.position_error < tolerances_.position &&
    sample.speed < tolerances_.speed && result.yaw_error < tolerances_.yaw &&
    sample.angular_speed < tolerances_.angular_speed;
  stable_duration_ = result.settled ? stable_duration_ + dt : 0.0;
  result.stable_duration = stable_duration_;
  result.complete = stable_duration_ >= tolerances_.hold_duration;
  if (!std::isfinite(result.yaw_error)) {
    result.yaw_error = std::numeric_limits<double>::infinity();
  }
  return result;
}

void GoalCompletionGate::reset()
{
  stable_duration_ = 0.0;
}

}  // namespace drone_planning
