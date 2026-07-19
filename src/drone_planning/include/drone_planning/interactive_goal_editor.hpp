#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "drone_planning/collision_checker.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "visualization_msgs/msg/interactive_marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace drone_planning
{

enum class GoalDraftState
{
  Editing,
  CandidateValid,
  CandidateInvalid,
  Validating,
  Ready,
  Rejected
};

struct CandidateValidation
{
  bool valid{false};
  std::string reason;
};

struct InteractiveGoal
{
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  double yaw{0.0};
};

double normalize_angle(double yaw);
geometry_msgs::msg::Quaternion quaternion_from_yaw(double yaw);
std::optional<double> yaw_from_quaternion(const geometry_msgs::msg::Quaternion & orientation);

CandidateValidation validate_goal_candidate(
  const Eigen::Vector3d & candidate, const CollisionChecker & checker,
  double minimum_navigation_altitude);

Eigen::Vector3d snap_goal_candidate(
  const Eigen::Vector3d & candidate, double resolution);

visualization_msgs::msg::InteractiveMarker make_goal_candidate_marker(
  const InteractiveGoal & candidate, std::size_t next_goal_number,
  GoalDraftState state, const std::string & status, const std::string & frame_id = "map");

geometry_msgs::msg::PoseArray make_selected_goals(
  const std::vector<InteractiveGoal> & goals, const std::string & frame_id = "map");

visualization_msgs::msg::MarkerArray make_interactive_goal_markers(
  const std::vector<InteractiveGoal> & goals, GoalDraftState state,
  std::optional<std::size_t> failed_goal_index, const std::string & frame_id = "map");

std::string format_mission_yaml(const std::vector<InteractiveGoal> & goals);

class InteractiveGoalEditor
{
public:
  explicit InteractiveGoalEditor(std::size_t max_goals);

  const InteractiveGoal & candidate() const;
  const std::vector<InteractiveGoal> & goals() const;
  std::size_t max_goals() const;
  std::uint64_t draft_revision() const;
  GoalDraftState state() const;
  const std::string & status_message() const;
  bool preview_valid() const;
  std::optional<std::size_t> failed_goal_index() const;

  bool set_candidate(const InteractiveGoal & candidate);
  bool set_candidate_position(const Eigen::Vector3d & position);
  bool set_candidate_yaw(double yaw);
  void set_candidate_validation(const CandidateValidation & validation);
  bool add_goal(const CandidateValidation & validation, std::string & reason);
  bool undo_last_goal();
  void clear_goals();
  bool begin_validation(std::uint64_t & revision, std::vector<InteractiveGoal> & goals);
  bool accept_validation(
    std::uint64_t revision, bool success, const std::string & message,
    std::optional<std::size_t> failed_goal_index = std::nullopt);

private:
  void invalidate_preview();

  std::size_t max_goals_{0U};
  InteractiveGoal candidate_{Eigen::Vector3d(0.0, 0.0, 1.5), 0.0};
  std::vector<InteractiveGoal> goals_;
  std::uint64_t draft_revision_{0U};
  GoalDraftState state_{GoalDraftState::Editing};
  std::string status_message_{"EDITING"};
  bool preview_valid_{false};
  std::optional<std::size_t> failed_goal_index_;
};

}  // namespace drone_planning
