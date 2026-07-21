#include "drone_mission/goal_visualization.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <string>

#include "geometry_msgs/msg/point.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace drone_mission
{
namespace
{

visualization_msgs::msg::Marker clear_marker()
{
  visualization_msgs::msg::Marker marker;
  marker.action = visualization_msgs::msg::Marker::DELETEALL;
  return marker;
}

visualization_msgs::msg::Marker base_marker(
  const std::string & frame, const builtin_interfaces::msg::Time & stamp,
  const std::string & ns, int id, int type)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame;
  marker.header.stamp = stamp;
  marker.ns = ns;
  marker.id = id;
  marker.type = type;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.color.a = 1.0F;
  return marker;
}

std::string goal_label(
  const std::string & name, const std::string & state,
  const geometry_msgs::msg::Pose & pose)
{
  const auto & orientation = pose.orientation;
  const double yaw = std::atan2(
    2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
    1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z));
  std::ostringstream text;
  text << name << " " << state << "\n" << std::fixed << std::setprecision(2)
       << "(" << pose.position.x << "," << pose.position.y << "," << pose.position.z
       << ")  yaw=" << std::lround(yaw * 180.0 / M_PI) << "°";
  return text.str();
}

}  // namespace

visualization_msgs::msg::MarkerArray make_single_goal_markers(
  const geometry_msgs::msg::Pose & goal, const std::string & frame_id,
  const builtin_interfaces::msg::Time & stamp)
{
  visualization_msgs::msg::MarkerArray result;
  result.markers.push_back(clear_marker());
  auto point = base_marker(frame_id, stamp, "goal_single_point", 100, 2);
  point.pose = goal;
  point.scale.x = point.scale.y = point.scale.z = 0.34;
  point.color.r = 1.0F;
  point.color.g = 0.35F;
  point.color.b = 0.05F;
  result.markers.push_back(point);
  auto arrow = base_marker(frame_id, stamp, "goal_single_direction", 101, 0);
  arrow.pose = goal;
  arrow.scale.x = 0.65; arrow.scale.y = 0.10; arrow.scale.z = 0.10;
  arrow.color = point.color;
  result.markers.push_back(arrow);
  auto label = base_marker(frame_id, stamp, "goal_single_label", 102, 9);
  label.pose = goal;
  label.pose.position.z += 0.35;
  label.scale.z = 0.25;
  label.color = point.color;
  label.text = goal_label("P1", "CURRENT", goal);
  result.markers.push_back(label);
  return result;
}

visualization_msgs::msg::MarkerArray make_mission_goal_markers(
  const geometry_msgs::msg::PoseArray & goals, std::size_t current_index,
  bool mission_complete, const builtin_interfaces::msg::Time & stamp)
{
  visualization_msgs::msg::MarkerArray result;
  result.markers.reserve(2U + goals.poses.size() * 3U);
  result.markers.push_back(clear_marker());
  if (goals.poses.size() > 1U) {
    auto line = base_marker(goals.header.frame_id, stamp, "mission_connections", 10, 4);
    line.scale.x = 0.045;
    line.color.r = 0.35F; line.color.g = 0.65F; line.color.b = 1.0F;
    for (const auto & pose : goals.poses) {
      geometry_msgs::msg::Point point = pose.position;
      line.points.push_back(point);
    }
    result.markers.push_back(line);
  }
  for (std::size_t index = 0U; index < goals.poses.size(); ++index) {
    const bool done = mission_complete || index < current_index;
    const bool current = !mission_complete && index == current_index;
    auto point = base_marker(
      goals.header.frame_id, stamp, "mission_points", 1000 + static_cast<int>(index), 2);
    point.pose = goals.poses[index];
    const double scale = current ? 0.42 : 0.32;
    point.scale.x = point.scale.y = point.scale.z = scale;
    if (done) {point.color.g = 0.85F; point.color.r = 0.10F;}
    else if (current) {point.color.r = 1.0F; point.color.g = 0.25F;}
    else {point.color.r = 0.95F; point.color.g = 0.75F; point.color.b = 0.15F;}
    result.markers.push_back(point);
    auto arrow = base_marker(
      goals.header.frame_id, stamp, "mission_directions", 2000 + static_cast<int>(index), 0);
    arrow.pose = goals.poses[index];
    arrow.scale.x = 0.45; arrow.scale.y = 0.075; arrow.scale.z = 0.075;
    arrow.color = point.color;
    result.markers.push_back(arrow);
    auto label = base_marker(
      goals.header.frame_id, stamp, "mission_labels", 3000 + static_cast<int>(index), 9);
    label.pose = goals.poses[index];
    label.pose.position.z += 0.35;
    label.scale.z = 0.25;
    label.color = point.color;
    const std::string prefix = "P" + std::to_string(index + 1U);
    label.text = goal_label(
      prefix, done ? "DONE" : (current ? "CURRENT" : "WAITING"), goals.poses[index]);
    result.markers.push_back(label);
  }
  return result;
}

}  // namespace drone_mission
