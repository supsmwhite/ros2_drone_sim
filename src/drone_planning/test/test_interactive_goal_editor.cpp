#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <Eigen/Core>

#include "drone_planning/interactive_goal_editor.hpp"
#include "drone_planning/static_environment.hpp"
#include "visualization_msgs/msg/interactive_marker_control.hpp"

namespace drone_planning
{
namespace
{

CollisionChecker checker()
{
  return CollisionChecker(
    StaticEnvironment(
      {Eigen::Vector3d(-1.0, -2.0, 0.15), Eigen::Vector3d(5.0, 3.0, 4.85)},
      {{Eigen::Vector3d(1.5, -0.5, 0.0), Eigen::Vector3d(2.5, 0.5, 4.7)}}),
    0.35);
}

CandidateValidation valid() {return {true, "GEOMETRY VALID"};}

void add_at(InteractiveGoalEditor & editor, const Eigen::Vector3d & point)
{
  editor.set_candidate(point);
  std::string reason;
  ASSERT_TRUE(editor.add_goal(valid(), reason));
}

}  // namespace

TEST(InteractiveGoalEditorTest, StartsEmptyAndKeepsOrderedArbitraryGoalCounts)
{
  InteractiveGoalEditor editor(8U);
  EXPECT_TRUE(editor.goals().empty());
  for (std::size_t index = 0U; index < 5U; ++index) {
    add_at(editor, Eigen::Vector3d(static_cast<double>(index), 0.1 * index, 1.5));
  }
  ASSERT_EQ(editor.goals().size(), 5U);
  for (std::size_t index = 0U; index < editor.goals().size(); ++index) {
    EXPECT_DOUBLE_EQ(editor.goals()[index].x(), static_cast<double>(index));
  }
}

TEST(InteractiveGoalEditorTest, EnforcesLimitUndoAndClearSafely)
{
  InteractiveGoalEditor editor(3U);
  EXPECT_FALSE(editor.undo_last_goal());
  add_at(editor, {0.0, 0.0, 1.5});
  add_at(editor, {0.5, 0.0, 1.5});
  add_at(editor, {1.0, 0.0, 1.5});
  editor.set_candidate({1.5, 0.0, 1.5});
  std::string reason;
  EXPECT_FALSE(editor.add_goal(valid(), reason));
  EXPECT_EQ(reason, "MAX GOALS REACHED");
  EXPECT_TRUE(editor.undo_last_goal());
  ASSERT_EQ(editor.goals().size(), 2U);
  EXPECT_DOUBLE_EQ(editor.goals().back().x(), 0.5);
  editor.clear_goals();
  EXPECT_TRUE(editor.goals().empty());
  EXPECT_EQ(editor.state(), GoalDraftState::Editing);
}

TEST(InteractiveGoalEditorTest, DraftChangesInvalidateReadyPreviewAndStaleResult)
{
  InteractiveGoalEditor editor(8U);
  add_at(editor, {0.5, 0.0, 1.5});
  std::uint64_t revision = 0U;
  std::vector<Eigen::Vector3d> goals;
  ASSERT_TRUE(editor.begin_validation(revision, goals));
  ASSERT_TRUE(editor.accept_validation(revision, true, "READY"));
  EXPECT_TRUE(editor.preview_valid());
  editor.set_candidate({0.6, 0.0, 1.5});
  EXPECT_FALSE(editor.preview_valid());
  EXPECT_FALSE(editor.accept_validation(revision, true, "STALE"));
  EXPECT_EQ(editor.state(), GoalDraftState::Editing);
}

TEST(InteractiveGoalEditorTest, EmptyValidationIsRejected)
{
  InteractiveGoalEditor editor(8U);
  std::uint64_t revision = 0U;
  std::vector<Eigen::Vector3d> goals;
  EXPECT_FALSE(editor.begin_validation(revision, goals));
  EXPECT_EQ(editor.state(), GoalDraftState::Rejected);
  EXPECT_NE(editor.status_message().find("EMPTY"), std::string::npos);
}

TEST(InteractiveGoalEditorGeometryTest, ReportsSpecificGeometryFailures)
{
  const auto collision_checker = checker();
  EXPECT_TRUE(validate_goal_candidate({0.0, 0.0, 1.5}, collision_checker, 0.5).valid);
  EXPECT_EQ(
    validate_goal_candidate({-2.0, 0.0, 1.5}, collision_checker, 0.5).reason,
    "OUTSIDE SAFE WORKSPACE");
  EXPECT_EQ(
    validate_goal_candidate({0.0, 0.0, 0.4}, collision_checker, 0.5).reason,
    "BELOW NAVIGATION FLOOR");
  EXPECT_EQ(
    validate_goal_candidate({2.0, 0.0, 1.5}, collision_checker, 0.5).reason,
    "INSIDE PLANNING-INFLATED OBSTACLE");
  EXPECT_EQ(
    validate_goal_candidate(
      {std::numeric_limits<double>::quiet_NaN(), 0.0, 1.5}, collision_checker, 0.5).reason,
    "NON-FINITE COORDINATE");
  EXPECT_EQ(
    validate_goal_candidate(
      {std::numeric_limits<double>::infinity(), 0.0, 1.5}, collision_checker, 0.5).reason,
    "NON-FINITE COORDINATE");
}

TEST(InteractiveGoalEditorGeometryTest, SnapsOnlyToRequestedResolutionAndCanRevalidate)
{
  const Eigen::Vector3d snapped = snap_goal_candidate({1.98, 0.02, 1.48}, 0.05);
  EXPECT_TRUE(snapped.isApprox(Eigen::Vector3d(2.0, 0.0, 1.5), 1.0e-12));
  EXPECT_FALSE(validate_goal_candidate(snapped, checker(), 0.5).valid);
  EXPECT_THROW(snap_goal_candidate({0.0, 0.0, 1.5}, 0.0), std::invalid_argument);
}

TEST(InteractiveGoalEditorMarkerTest, ContainsOnlyWorldFixedPlaneAxisAndMenuControls)
{
  const auto marker = make_goal_candidate_marker(
    {0.0, 0.0, 1.5}, 4U, GoalDraftState::Editing, "EDITING");
  std::size_t planes = 0U;
  std::size_t axes = 0U;
  std::size_t menus = 0U;
  for (const auto & control : marker.controls) {
    EXPECT_NE(
      control.interaction_mode,
      visualization_msgs::msg::InteractiveMarkerControl::MOVE_3D);
    EXPECT_NE(
      control.interaction_mode,
      visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS);
    EXPECT_NE(
      control.interaction_mode,
      visualization_msgs::msg::InteractiveMarkerControl::MOVE_ROTATE);
    if (control.interaction_mode ==
      visualization_msgs::msg::InteractiveMarkerControl::MOVE_PLANE)
    {
      ++planes;
      EXPECT_EQ(
        control.orientation_mode,
        visualization_msgs::msg::InteractiveMarkerControl::FIXED);
      EXPECT_NEAR(std::abs(control.orientation.y), std::sqrt(0.5), 1.0e-12);
    } else if (control.interaction_mode ==
      visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS)
    {
      ++axes;
      EXPECT_EQ(
        control.orientation_mode,
        visualization_msgs::msg::InteractiveMarkerControl::FIXED);
      EXPECT_NEAR(std::abs(control.orientation.y), std::sqrt(0.5), 1.0e-12);
    } else if (control.interaction_mode ==
      visualization_msgs::msg::InteractiveMarkerControl::MENU)
    {
      ++menus;
    }
  }
  EXPECT_EQ(planes, 1U);
  EXPECT_EQ(axes, 1U);
  EXPECT_EQ(menus, 1U);
}

}  // namespace drone_planning
