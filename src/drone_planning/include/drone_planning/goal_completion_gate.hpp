#pragma once

#include "drone_planning/yaw_reference_generator.hpp"

namespace drone_planning
{

struct GoalCompletionTolerances
{
  double position{0.20};
  double speed{0.15};
  double yaw{0.10};
  double angular_speed{0.20};
  double hold_duration{1.0};
};

struct GoalCompletionSample
{
  double position_error{0.0};
  double speed{0.0};
  double actual_yaw{0.0};
  double angular_speed{0.0};
};

struct GoalCompletionEvaluation
{
  bool settled{false};
  bool complete{false};
  double yaw_error{0.0};
  double stable_duration{0.0};
};

double shortest_yaw_error(double target_yaw, double actual_yaw);

double goal_acceptance_target_yaw(
  YawMode mode, double fixed_yaw, double mission_goal_yaw);

class GoalCompletionGate
{
public:
  explicit GoalCompletionGate(GoalCompletionTolerances tolerances = {});

  GoalCompletionEvaluation update(
    const GoalCompletionSample & sample, double target_yaw, double dt);

  void reset();

private:
  GoalCompletionTolerances tolerances_;
  double stable_duration_{0.0};
};

}  // namespace drone_planning
