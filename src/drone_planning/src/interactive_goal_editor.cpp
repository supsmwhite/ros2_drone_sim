#include "drone_planning/interactive_goal_editor.hpp"

#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <utility>

#include "visualization_msgs/msg/interactive_marker_control.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace drone_planning
{
namespace
{

bool inside_closed_box(const Eigen::Vector3d & point, const AxisAlignedBox & box)
{
  return (point.array() >= box.min_corner.array()).all() &&
         (point.array() <= box.max_corner.array()).all();
}

bool inside_open_box(const Eigen::Vector3d & point, const AxisAlignedBox & box)
{
  return (point.array() > box.min_corner.array()).all() &&
         (point.array() < box.max_corner.array()).all();
}

std_msgs::msg::ColorRGBA candidate_color(GoalDraftState state)
{
  std_msgs::msg::ColorRGBA color;
  color.a = 0.95F;
  if (state == GoalDraftState::CandidateInvalid || state == GoalDraftState::Rejected) {
    color.r = 0.95F;
    color.g = 0.12F;
    color.b = 0.08F;
  } else if (state == GoalDraftState::Validating) {
    color.r = 0.10F;
    color.g = 0.35F;
    color.b = 1.0F;
  } else if (state == GoalDraftState::Editing) {
    color.r = 1.0F;
    color.g = 0.78F;
    color.b = 0.05F;
  } else {
    color.r = 0.10F;
    color.g = 0.90F;
    color.b = 0.20F;
  }
  return color;
}

std_msgs::msg::ColorRGBA goal_color(
  GoalDraftState state, std::size_t index, std::optional<std::size_t> failed_index)
{
  if (failed_index && index == *failed_index) {
    return candidate_color(GoalDraftState::Rejected);
  }
  if (state == GoalDraftState::Ready) {
    return candidate_color(GoalDraftState::CandidateValid);
  }
  if (state == GoalDraftState::Validating) {
    return candidate_color(GoalDraftState::Validating);
  }
  return candidate_color(GoalDraftState::Editing);
}

double degrees(double yaw)
{
  return yaw * 180.0 / M_PI;
}

visualization_msgs::msg::InteractiveMarkerControl world_z_control(
  const std::string & name, std::uint8_t mode)
{
  visualization_msgs::msg::InteractiveMarkerControl control;
  control.name = name;
  control.interaction_mode = mode;
  control.orientation_mode = visualization_msgs::msg::InteractiveMarkerControl::FIXED;
  // Interactive Marker controls act along their local x axis. This +90 degree
  // rotation around world y maps local x to world -z (the sign does not matter).
  const double half_sqrt = std::sqrt(0.5);
  control.orientation.w = half_sqrt;
  control.orientation.y = half_sqrt;
  return control;
}

visualization_msgs::msg::InteractiveMarkerControl world_x_move_control()
{
  visualization_msgs::msg::InteractiveMarkerControl control;
  control.name = "move_x";
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
  control.orientation_mode = visualization_msgs::msg::InteractiveMarkerControl::FIXED;
  control.orientation.w = 1.0;
  return control;
}

visualization_msgs::msg::InteractiveMarkerControl world_y_move_control()
{
  visualization_msgs::msg::InteractiveMarkerControl control;
  control.name = "move_y";
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
  control.orientation_mode = visualization_msgs::msg::InteractiveMarkerControl::FIXED;
  const double half_sqrt = std::sqrt(0.5);
  control.orientation.w = half_sqrt;
  control.orientation.z = half_sqrt;
  return control;
}

}  // namespace

double normalize_angle(double yaw)
{
  if (!std::isfinite(yaw)) {
    return yaw;
  }
  double result = std::remainder(yaw, 2.0 * M_PI);
  if (result <= -M_PI) {
    result += 2.0 * M_PI;
  }
  return result;
}

geometry_msgs::msg::Quaternion quaternion_from_yaw(double yaw)
{
  if (!std::isfinite(yaw)) {
    throw std::invalid_argument("yaw must be finite");
  }
  geometry_msgs::msg::Quaternion orientation;
  const double normalized = normalize_angle(yaw);
  orientation.z = std::sin(0.5 * normalized);
  orientation.w = std::cos(0.5 * normalized);
  return orientation;
}

std::optional<double> yaw_from_quaternion(
  const geometry_msgs::msg::Quaternion & orientation)
{
  if (!std::isfinite(orientation.x) || !std::isfinite(orientation.y) ||
    !std::isfinite(orientation.z) || !std::isfinite(orientation.w))
  {
    return std::nullopt;
  }
  const double norm = std::sqrt(
    orientation.x * orientation.x + orientation.y * orientation.y +
    orientation.z * orientation.z + orientation.w * orientation.w);
  if (!std::isfinite(norm) || norm <= 1.0e-12) {
    return std::nullopt;
  }
  const double x = orientation.x / norm;
  const double y = orientation.y / norm;
  const double z = orientation.z / norm;
  const double w = orientation.w / norm;
  return normalize_angle(std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)));
}

CandidateValidation validate_goal_candidate(
  const Eigen::Vector3d & candidate, const CollisionChecker & checker,
  double minimum_navigation_altitude)
{
  if (!candidate.allFinite()) {
    return {false, "NON-FINITE COORDINATE"};
  }
  if (!std::isfinite(minimum_navigation_altitude) || minimum_navigation_altitude < 0.0) {
    throw std::invalid_argument("minimum navigation altitude must be finite and non-negative");
  }
  if (candidate.z() < minimum_navigation_altitude) {
    return {false, "BELOW NAVIGATION FLOOR"};
  }
  if (!inside_open_box(candidate, checker.safe_workspace())) {
    return {false, "OUTSIDE SAFE WORKSPACE"};
  }
  for (const auto & obstacle : checker.inflated_obstacles()) {
    if (inside_closed_box(candidate, obstacle)) {
      return {false, "INSIDE PLANNING-INFLATED OBSTACLE"};
    }
  }
  return {true, "GEOMETRY VALID"};
}

Eigen::Vector3d snap_goal_candidate(
  const Eigen::Vector3d & candidate, double resolution)
{
  if (!std::isfinite(resolution) || resolution <= 0.0) {
    throw std::invalid_argument("snap resolution must be finite and positive");
  }
  if (!candidate.allFinite()) {
    return candidate;
  }
  return (candidate.array() / resolution).round().matrix() * resolution;
}

visualization_msgs::msg::InteractiveMarker make_goal_candidate_marker(
  const InteractiveGoal & candidate, std::size_t next_goal_number,
  GoalDraftState state, const std::string & status, const std::string & frame_id)
{
  visualization_msgs::msg::InteractiveMarker marker;
  marker.header.frame_id = frame_id;
  marker.name = "goal_candidate";
  marker.description = "3D goal candidate";
  marker.scale = 0.75;
  marker.pose.position.x = candidate.position.x();
  marker.pose.position.y = candidate.position.y();
  marker.pose.position.z = candidate.position.z();
  marker.pose.orientation = quaternion_from_yaw(candidate.yaw);

  visualization_msgs::msg::InteractiveMarkerControl body;
  body.name = "menu";
  body.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MENU;
  body.always_visible = true;
  visualization_msgs::msg::Marker sphere;
  sphere.type = visualization_msgs::msg::Marker::SPHERE;
  sphere.scale.x = 0.34;
  sphere.scale.y = 0.34;
  sphere.scale.z = 0.34;
  sphere.color = candidate_color(state);
  body.markers.push_back(sphere);

  visualization_msgs::msg::Marker arrow;
  arrow.type = visualization_msgs::msg::Marker::ARROW;
  arrow.pose.position.x = 0.12;
  arrow.scale.x = 0.58;
  arrow.scale.y = 0.10;
  arrow.scale.z = 0.10;
  arrow.color = candidate_color(state);
  body.markers.push_back(arrow);

  visualization_msgs::msg::Marker label;
  label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  label.pose.position.z = 0.46;
  label.scale.z = 0.18;
  label.color.r = 1.0F;
  label.color.g = 1.0F;
  label.color.b = 1.0F;
  label.color.a = 1.0F;
  std::ostringstream text;
  text << "Candidate P" << next_goal_number << "\nx=" << std::fixed << std::setprecision(2)
       << candidate.position.x() << " y=" << candidate.position.y() << " z=" <<
    candidate.position.z() << " yaw=" << std::setprecision(0) << degrees(candidate.yaw) <<
    " deg\n" << status;
  label.text = text.str();
  body.markers.push_back(label);
  marker.controls.push_back(body);

  marker.controls.push_back(world_x_move_control());
  marker.controls.push_back(world_y_move_control());
  marker.controls.push_back(
    world_z_control(
      "move_z", visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS));
  marker.controls.push_back(
    world_z_control(
      "rotate_z", visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS));

  return marker;
}

geometry_msgs::msg::PoseArray make_selected_goals(
  const std::vector<InteractiveGoal> & goals, const std::string & frame_id)
{
  geometry_msgs::msg::PoseArray result;
  result.header.frame_id = frame_id;
  result.poses.reserve(goals.size());
  for (const auto & goal : goals) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = goal.position.x();
    pose.position.y = goal.position.y();
    pose.position.z = goal.position.z();
    pose.orientation = quaternion_from_yaw(goal.yaw);
    result.poses.push_back(pose);
  }
  return result;
}

visualization_msgs::msg::MarkerArray make_interactive_goal_markers(
  const std::vector<InteractiveGoal> & goals, GoalDraftState state,
  std::optional<std::size_t> failed_goal_index, const std::string & frame_id)
{
  visualization_msgs::msg::MarkerArray result;
  visualization_msgs::msg::Marker clear;
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  result.markers.push_back(clear);
  for (std::size_t index = 0U; index < goals.size(); ++index) {
    visualization_msgs::msg::Marker sphere;
    sphere.header.frame_id = frame_id;
    sphere.ns = "interactive_goals";
    sphere.id = static_cast<int>(3U * index);
    sphere.type = visualization_msgs::msg::Marker::SPHERE;
    sphere.action = visualization_msgs::msg::Marker::ADD;
    sphere.pose.position.x = goals[index].position.x();
    sphere.pose.position.y = goals[index].position.y();
    sphere.pose.position.z = goals[index].position.z();
    sphere.pose.orientation.w = 1.0;
    sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.30;
    sphere.color = goal_color(state, index, failed_goal_index);
    result.markers.push_back(sphere);

    auto arrow = sphere;
    arrow.id = static_cast<int>(3U * index + 1U);
    arrow.type = visualization_msgs::msg::Marker::ARROW;
    arrow.pose.orientation = quaternion_from_yaw(goals[index].yaw);
    arrow.scale.x = 0.58;
    arrow.scale.y = 0.10;
    arrow.scale.z = 0.10;
    result.markers.push_back(arrow);

    auto label = sphere;
    label.id = static_cast<int>(3U * index + 2U);
    label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    label.pose.position.z += 0.35;
    label.scale.x = label.scale.y = 0.0;
    label.scale.z = 0.22;
    label.color.r = label.color.g = label.color.b = label.color.a = 1.0F;
    std::ostringstream text;
    text << "P" << index + 1U << " yaw=" << std::fixed << std::setprecision(0) <<
      degrees(goals[index].yaw) << " deg";
    label.text = text.str();
    result.markers.push_back(label);
  }
  return result;
}

std::string format_mission_yaml(const std::vector<InteractiveGoal> & goals)
{
  std::ostringstream yaml;
  yaml << "goals:\n  [\n" << std::fixed << std::setprecision(6);
  for (std::size_t index = 0U; index < goals.size(); ++index) {
    const auto & goal = goals[index];
    yaml << "    " << goal.position.x() << ", " << goal.position.y() << ", " <<
      goal.position.z() << ", " << normalize_angle(goal.yaw) <<
      (index + 1U == goals.size() ? "\n" : ",\n");
  }
  yaml << "  ]";
  return yaml.str();
}

InteractiveGoalEditor::InteractiveGoalEditor(std::size_t max_goals)
: max_goals_(max_goals)
{
  if (max_goals_ == 0U) {
    throw std::invalid_argument("max goals must be positive");
  }
}

const InteractiveGoal & InteractiveGoalEditor::candidate() const {return candidate_;}
const std::vector<InteractiveGoal> & InteractiveGoalEditor::goals() const {return goals_;}
std::size_t InteractiveGoalEditor::max_goals() const {return max_goals_;}
std::uint64_t InteractiveGoalEditor::draft_revision() const {return draft_revision_;}
GoalDraftState InteractiveGoalEditor::state() const {return state_;}
const std::string & InteractiveGoalEditor::status_message() const {return status_message_;}
bool InteractiveGoalEditor::preview_valid() const {return preview_valid_;}
std::optional<std::size_t> InteractiveGoalEditor::failed_goal_index() const
{
  return failed_goal_index_;
}

void InteractiveGoalEditor::invalidate_preview()
{
  ++draft_revision_;
  preview_valid_ = false;
  failed_goal_index_.reset();
}

bool InteractiveGoalEditor::set_candidate(const InteractiveGoal & candidate)
{
  if (!candidate.position.allFinite() || !std::isfinite(candidate.yaw)) {
    return false;
  }
  InteractiveGoal normalized{candidate.position, normalize_angle(candidate.yaw)};
  if (candidate_.position.isApprox(normalized.position, 1.0e-12) &&
    std::abs(normalize_angle(candidate_.yaw - normalized.yaw)) <= 1.0e-12)
  {
    return true;
  }
  candidate_ = normalized;
  invalidate_preview();
  state_ = GoalDraftState::Editing;
  status_message_ = "EDITING";
  return true;
}

bool InteractiveGoalEditor::set_candidate_position(const Eigen::Vector3d & position)
{
  return set_candidate({position, candidate_.yaw});
}

bool InteractiveGoalEditor::set_candidate_yaw(double yaw)
{
  return set_candidate({candidate_.position, yaw});
}

void InteractiveGoalEditor::set_candidate_validation(const CandidateValidation & validation)
{
  state_ = validation.valid ? GoalDraftState::CandidateValid : GoalDraftState::CandidateInvalid;
  status_message_ = validation.reason;
}

bool InteractiveGoalEditor::add_goal(
  const CandidateValidation & validation, std::string & reason)
{
  if (!validation.valid) {
    set_candidate_validation(validation);
    reason = validation.reason;
    return false;
  }
  if (goals_.size() >= max_goals_) {
    state_ = GoalDraftState::Rejected;
    status_message_ = "MAX GOALS REACHED";
    reason = status_message_;
    return false;
  }
  goals_.push_back(candidate_);
  invalidate_preview();
  state_ = GoalDraftState::CandidateValid;
  status_message_ = "GOAL ADDED";
  reason = status_message_;
  return true;
}

bool InteractiveGoalEditor::undo_last_goal()
{
  if (goals_.empty()) {
    state_ = GoalDraftState::Editing;
    status_message_ = "NO GOAL TO UNDO";
    return false;
  }
  candidate_ = goals_.back();
  goals_.pop_back();
  invalidate_preview();
  state_ = GoalDraftState::Editing;
  status_message_ = "LAST GOAL REMOVED";
  return true;
}

void InteractiveGoalEditor::clear_goals()
{
  goals_.clear();
  invalidate_preview();
  state_ = GoalDraftState::Editing;
  status_message_ = "EDITING";
}

bool InteractiveGoalEditor::begin_validation(
  std::uint64_t & revision, std::vector<InteractiveGoal> & goals)
{
  if (goals_.empty()) {
    state_ = GoalDraftState::Rejected;
    status_message_ = "REJECTED: GOAL LIST IS EMPTY";
    return false;
  }
  preview_valid_ = false;
  failed_goal_index_.reset();
  state_ = GoalDraftState::Validating;
  status_message_ = "VALIDATING";
  revision = draft_revision_;
  goals = goals_;
  return true;
}

bool InteractiveGoalEditor::accept_validation(
  std::uint64_t revision, bool success, const std::string & message,
  std::optional<std::size_t> failed_goal_index)
{
  if (revision != draft_revision_) {
    return false;
  }
  preview_valid_ = success;
  state_ = success ? GoalDraftState::Ready : GoalDraftState::Rejected;
  status_message_ = message;
  failed_goal_index_ = success ? std::nullopt : failed_goal_index;
  return true;
}

}  // namespace drone_planning
