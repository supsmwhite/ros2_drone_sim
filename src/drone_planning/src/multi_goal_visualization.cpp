#include "drone_planning/multi_goal_visualization.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>

#include "visualization_msgs/msg/marker.hpp"

namespace drone_planning
{

std::vector<MissionGoal> parse_goals(const std::vector<double> & values)
{
  if (values.empty() || values.size() % 4U != 0U) {
    throw std::invalid_argument("goals must contain non-empty flat [x,y,z,yaw] groups");
  }
  std::vector<MissionGoal> goals;
  goals.reserve(values.size() / 4U);
  for (std::size_t offset = 0U; offset < values.size(); offset += 4U) {
    MissionGoal goal{
      Eigen::Vector3d(values[offset], values[offset + 1U], values[offset + 2U]),
      values[offset + 3U]};
    if (!goal.position.allFinite() || !std::isfinite(goal.yaw)) {
      throw std::invalid_argument("all multi-goal positions and yaw values must be finite");
    }
    if (goal.yaw != 0.0) {
      throw std::invalid_argument("the first multi-goal avoidance version requires yaw=0");
    }
    goals.push_back(goal);
  }
  return goals;
}

visualization_msgs::msg::MarkerArray make_goal_markers(
  const std::vector<MissionGoal> & goals,
  std::size_t current_goal_index,
  std::size_t visited_goals,
  MissionVisualizationState state,
  const std::string & frame_id,
  const builtin_interfaces::msg::Time & stamp,
  std::optional<double> actual_speed,
  double reference_speed,
  double nominal_speed)
{
  visualization_msgs::msg::MarkerArray result;
  result.markers.reserve(goals.size() * 2U + 1U);
  const std::size_t completed_count = state == MissionVisualizationState::Complete ?
    goals.size() : std::min(visited_goals, goals.size());

  for (std::size_t index = 0U; index < goals.size(); ++index) {
    const bool completed = index < completed_count;
    const bool current = !completed && index == current_goal_index;

    visualization_msgs::msg::Marker body;
    body.header.frame_id = frame_id;
    body.header.stamp = stamp;
    body.ns = "multi_goal_points";
    body.id = static_cast<int>(2U * index);
    body.type = visualization_msgs::msg::Marker::SPHERE;
    body.action = visualization_msgs::msg::Marker::ADD;
    body.pose.position.x = goals[index].position.x();
    body.pose.position.y = goals[index].position.y();
    body.pose.position.z = goals[index].position.z();
    body.pose.orientation.w = 1.0;
    const double scale = current ? 0.40 : 0.32;
    body.scale.x = scale;
    body.scale.y = scale;
    body.scale.z = scale;
    body.color.a = 1.0F;
    if (completed) {
      body.color.r = 0.10F;
      body.color.g = 0.85F;
      body.color.b = 0.20F;
    } else if (current) {
      body.color.r = 1.00F;
      body.color.g = 0.25F;
      body.color.b = 0.05F;
    } else {
      body.color.r = 0.95F;
      body.color.g = 0.75F;
      body.color.b = 0.15F;
    }
    result.markers.push_back(body);

    visualization_msgs::msg::Marker label;
    label.header = body.header;
    label.ns = "multi_goal_labels";
    label.id = static_cast<int>(2U * index + 1U);
    label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    label.action = visualization_msgs::msg::Marker::ADD;
    label.pose = body.pose;
    label.pose.position.z += 0.35;
    label.scale.z = 0.30;
    label.color = body.color;
    label.text = "P" + std::to_string(index + 1U) + "  " +
      (completed ? "DONE" : (current ? "CURRENT" : "WAITING"));
    result.markers.push_back(label);
  }

  visualization_msgs::msg::Marker status;
  status.header.frame_id = frame_id;
  status.header.stamp = stamp;
  status.ns = "multi_goal_status";
  status.id = 0;
  status.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  status.action = visualization_msgs::msg::Marker::ADD;
  status.pose.position.x = -0.5;
  status.pose.position.y = 6.2;
  status.pose.position.z = 4.5;
  status.pose.orientation.w = 1.0;
  status.scale.z = 0.34;
  status.color.a = 1.0F;
  status.color.r = 0.95F;
  status.color.g = 0.95F;
  status.color.b = 0.95F;
  std::ostringstream text;
  text << std::fixed << std::setprecision(2);
  if (state == MissionVisualizationState::Complete) {
    status.color.r = 0.10F;
    status.color.g = 0.85F;
    status.color.b = 0.20F;
    text << "MISSION COMPLETE\nGoals: " << goals.size() << " / " << goals.size();
  } else if (state == MissionVisualizationState::Failed) {
    status.color.r = 1.00F;
    status.color.g = 0.25F;
    status.color.b = 0.05F;
    text << "MISSION FAILED\nGoal: P" << (current_goal_index + 1U) << " / " << goals.size();
  } else {
    text << "Mission: RUNNING\nGoal: P" << (current_goal_index + 1U) << " / " << goals.size()
         << "\nVisited: " << completed_count << " / " << goals.size();
  }
  text << "\nActual: ";
  if (actual_speed && std::isfinite(*actual_speed)) {
    text << *actual_speed << " m/s";
  } else {
    text << "--";
  }
  text << "\nReference: " << reference_speed << " m/s"
       << "\nNominal: " << nominal_speed << " m/s";
  status.text = text.str();
  result.markers.push_back(status);
  return result;
}

}  // namespace drone_planning
