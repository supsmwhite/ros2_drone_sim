#ifndef DRONE_MISSION__GOAL_VISUALIZATION_HPP_
#define DRONE_MISSION__GOAL_VISUALIZATION_HPP_

#include <cstddef>
#include <string>

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace drone_mission
{

visualization_msgs::msg::MarkerArray make_single_goal_markers(
  const geometry_msgs::msg::Pose & goal, const std::string & frame_id,
  const builtin_interfaces::msg::Time & stamp);

visualization_msgs::msg::MarkerArray make_mission_goal_markers(
  const geometry_msgs::msg::PoseArray & goals, std::size_t current_index,
  bool mission_complete, const builtin_interfaces::msg::Time & stamp);

}  // namespace drone_mission

#endif  // DRONE_MISSION__GOAL_VISUALIZATION_HPP_
