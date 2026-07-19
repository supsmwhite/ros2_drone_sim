#ifndef DRONE_MISSION__GOAL_INPUT_HPP_
#define DRONE_MISSION__GOAL_INPUT_HPP_

#include <string>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"

namespace drone_mission
{

struct GoalConstraints
{
  double x_min{-1.0};
  double x_max{14.5};
  double y_min{-2.5};
  double y_max{7.0};
  double z_min{0.0};
  double z_max{5.0};
};

std::vector<double> parse_finite_numbers(const std::vector<std::string> & arguments);
std::vector<double> parse_goal_arguments(const std::vector<std::string> & arguments);
void validate_constraints(const GoalConstraints & constraints);
geometry_msgs::msg::Pose make_goal_pose(
  double x, double y, double z, double yaw, const GoalConstraints & constraints);
std::vector<geometry_msgs::msg::Pose> make_goal_poses(
  const std::vector<double> & values, const GoalConstraints & constraints);

}  // namespace drone_mission

#endif  // DRONE_MISSION__GOAL_INPUT_HPP_
