#include "drone_mission/goal_visualization.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
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

}  // namespace

visualization_msgs::msg::MarkerArray make_single_goal_markers(
  const geometry_msgs::msg::Pose & goal, const std::string & frame_id,
  const builtin_interfaces::msg::Time & stamp, bool goal_complete)
{
  visualization_msgs::msg::MarkerArray result;
  result.markers.push_back(clear_marker());
  auto point = base_marker(frame_id, stamp, "goal_single_point", 100, 2);
  point.pose = goal;
  point.scale.x = point.scale.y = point.scale.z = 0.34;
  if (goal_complete) {
    point.color.r = 0.10F; point.color.g = 0.85F; point.color.b = 0.20F;
  } else {
    point.color.r = 1.0F; point.color.g = 0.35F; point.color.b = 0.05F;
  }
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
  label.text = goal_complete ? "GOAL DONE" : "GOAL CURRENT";
  result.markers.push_back(label);
  return result;
}

bool single_goal_within_tolerance(
  const geometry_msgs::msg::Pose & goal, const nav_msgs::msg::Odometry & odometry,
  double position_tolerance, double linear_speed_tolerance, double yaw_tolerance,
  double angular_speed_tolerance)
{
  if (!std::isfinite(position_tolerance) || position_tolerance <= 0.0 ||
    !std::isfinite(linear_speed_tolerance) || linear_speed_tolerance <= 0.0 ||
    !std::isfinite(yaw_tolerance) || yaw_tolerance <= 0.0 ||
    !std::isfinite(angular_speed_tolerance) || angular_speed_tolerance <= 0.0)
  {
    throw std::invalid_argument("single-goal tolerances must be finite and positive");
  }
  const auto yaw_from_quaternion = [](const geometry_msgs::msg::Quaternion & q) {
      const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
      if (!std::isfinite(norm) || norm <= 1.0e-12) {
        return std::numeric_limits<double>::quiet_NaN();
      }
      const double scale = 1.0 / std::sqrt(norm);
      const double x = q.x * scale, y = q.y * scale, z = q.z * scale, w = q.w * scale;
      return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
    };
  const auto & actual = odometry.pose.pose;
  const double dx = goal.position.x - actual.position.x;
  const double dy = goal.position.y - actual.position.y;
  const double dz = goal.position.z - actual.position.z;
  const auto & twist = odometry.twist.twist;
  const double position_error = std::sqrt(dx * dx + dy * dy + dz * dz);
  const double linear_speed = std::sqrt(
    twist.linear.x * twist.linear.x + twist.linear.y * twist.linear.y +
    twist.linear.z * twist.linear.z);
  const double angular_speed = std::sqrt(
    twist.angular.x * twist.angular.x + twist.angular.y * twist.angular.y +
    twist.angular.z * twist.angular.z);
  const double yaw_error = std::remainder(
    yaw_from_quaternion(goal.orientation) - yaw_from_quaternion(actual.orientation),
    2.0 * 3.14159265358979323846);
  return std::isfinite(position_error) && std::isfinite(linear_speed) &&
         std::isfinite(angular_speed) && std::isfinite(yaw_error) &&
         position_error < position_tolerance && linear_speed < linear_speed_tolerance &&
         std::abs(yaw_error) < yaw_tolerance && angular_speed < angular_speed_tolerance;
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
    label.text = "P" + std::to_string(index + 1U) +
      (done ? " DONE" : (current ? " CURRENT" : ""));
    result.markers.push_back(label);
  }
  return result;
}

}  // namespace drone_mission
