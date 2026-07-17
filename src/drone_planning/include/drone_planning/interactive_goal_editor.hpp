#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "drone_planning/collision_checker.hpp"
#include "visualization_msgs/msg/interactive_marker.hpp"

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

CandidateValidation validate_goal_candidate(
  const Eigen::Vector3d & candidate, const CollisionChecker & checker,
  double minimum_navigation_altitude);

Eigen::Vector3d snap_goal_candidate(
  const Eigen::Vector3d & candidate, double resolution);

visualization_msgs::msg::InteractiveMarker make_goal_candidate_marker(
  const Eigen::Vector3d & candidate, std::size_t next_goal_number,
  GoalDraftState state, const std::string & status, const std::string & frame_id = "map");

class InteractiveGoalEditor
{
public:
  explicit InteractiveGoalEditor(std::size_t max_goals);

  const Eigen::Vector3d & candidate() const;
  const std::vector<Eigen::Vector3d> & goals() const;
  std::size_t max_goals() const;
  std::uint64_t draft_revision() const;
  GoalDraftState state() const;
  const std::string & status_message() const;
  bool preview_valid() const;
  std::optional<std::size_t> failed_goal_index() const;

  void set_candidate(const Eigen::Vector3d & candidate);
  void set_candidate_validation(const CandidateValidation & validation);
  bool add_goal(const CandidateValidation & validation, std::string & reason);
  bool undo_last_goal();
  void clear_goals();
  bool begin_validation(std::uint64_t & revision, std::vector<Eigen::Vector3d> & goals);
  bool accept_validation(
    std::uint64_t revision, bool success, const std::string & message,
    std::optional<std::size_t> failed_goal_index = std::nullopt);

private:
  void invalidate_preview();

  std::size_t max_goals_{0U};
  Eigen::Vector3d candidate_{0.0, 0.0, 1.5};
  std::vector<Eigen::Vector3d> goals_;
  std::uint64_t draft_revision_{0U};
  GoalDraftState state_{GoalDraftState::Editing};
  std::string status_message_{"EDITING"};
  bool preview_valid_{false};
  std::optional<std::size_t> failed_goal_index_;
};

}  // namespace drone_planning
