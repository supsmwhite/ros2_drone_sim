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

visualization_msgs::msg::InteractiveMarkerControl world_z_control(
  const std::string & name, std::uint8_t mode)
{
  visualization_msgs::msg::InteractiveMarkerControl control;
  control.name = name;
  control.interaction_mode = mode;
  control.orientation_mode = visualization_msgs::msg::InteractiveMarkerControl::FIXED;
  // Interactive Marker controls act along their local x axis.  This +90 degree
  // rotation around world y maps local x to world -z, so MOVE_PLANE is world XY
  // and MOVE_AXIS is the world z line (the sign does not affect dragging).
  const double half_sqrt = std::sqrt(0.5);
  control.orientation.w = half_sqrt;
  control.orientation.y = half_sqrt;
  return control;
}

}  // namespace

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
  const Eigen::Vector3d & candidate, std::size_t next_goal_number,
  GoalDraftState state, const std::string & status, const std::string & frame_id)
{
  visualization_msgs::msg::InteractiveMarker marker;
  marker.header.frame_id = frame_id;
  marker.name = "goal_candidate";
  marker.description = "3D goal candidate";
  marker.scale = 0.75;
  marker.pose.position.x = candidate.x();
  marker.pose.position.y = candidate.y();
  marker.pose.position.z = candidate.z();
  marker.pose.orientation.w = 1.0;

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
       << candidate.x() << " y=" << candidate.y() << " z=" << candidate.z()
       << "\n" << status;
  label.text = text.str();
  body.markers.push_back(label);
  marker.controls.push_back(body);

  marker.controls.push_back(
    world_z_control(
      "move_xy", visualization_msgs::msg::InteractiveMarkerControl::MOVE_PLANE));
  marker.controls.push_back(
    world_z_control(
      "move_z", visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS));

  return marker;
}

InteractiveGoalEditor::InteractiveGoalEditor(std::size_t max_goals)
: max_goals_(max_goals)
{
  if (max_goals_ == 0U) {
    throw std::invalid_argument("max goals must be positive");
  }
}

const Eigen::Vector3d & InteractiveGoalEditor::candidate() const {return candidate_;}
const std::vector<Eigen::Vector3d> & InteractiveGoalEditor::goals() const {return goals_;}
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

void InteractiveGoalEditor::set_candidate(const Eigen::Vector3d & candidate)
{
  candidate_ = candidate;
  invalidate_preview();
  state_ = GoalDraftState::Editing;
  status_message_ = "EDITING";
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
  std::uint64_t & revision, std::vector<Eigen::Vector3d> & goals)
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
