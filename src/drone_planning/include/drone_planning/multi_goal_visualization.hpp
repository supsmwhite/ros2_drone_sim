#ifndef DRONE_PLANNING__MULTI_GOAL_VISUALIZATION_HPP_
#define DRONE_PLANNING__MULTI_GOAL_VISUALIZATION_HPP_

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "builtin_interfaces/msg/time.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace drone_planning
{

struct MissionGoal
{
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  double yaw{0.0};
};

enum class MissionVisualizationState
{
  Running,
  Complete,
  Failed
};

std::vector<MissionGoal> parse_goals(const std::vector<double> & values);

visualization_msgs::msg::MarkerArray make_goal_markers(
  const std::vector<MissionGoal> & goals,
  std::size_t current_goal_index,
  std::size_t visited_goals,
  MissionVisualizationState state,
  const std::string & frame_id,
  const builtin_interfaces::msg::Time & stamp,
  std::optional<double> actual_speed,
  double reference_speed,
  double nominal_speed);

}  // namespace drone_planning

#endif  // DRONE_PLANNING__MULTI_GOAL_VISUALIZATION_HPP_
