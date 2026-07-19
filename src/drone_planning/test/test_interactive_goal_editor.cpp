#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <set>
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

void add_at(InteractiveGoalEditor & editor, const Eigen::Vector3d & point, double yaw = 0.0)
{
  ASSERT_TRUE(editor.set_candidate({point, yaw}));
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
    EXPECT_DOUBLE_EQ(editor.goals()[index].position.x(), static_cast<double>(index));
  }
}

TEST(InteractiveGoalEditorTest, EnforcesLimitUndoAndClearSafely)
{
  InteractiveGoalEditor editor(3U);
  EXPECT_FALSE(editor.undo_last_goal());
  add_at(editor, {0.0, 0.0, 1.5});
  add_at(editor, {0.5, 0.0, 1.5});
  add_at(editor, {1.0, 0.0, 1.5});
  editor.set_candidate_position({1.5, 0.0, 1.5});
  std::string reason;
  EXPECT_FALSE(editor.add_goal(valid(), reason));
  EXPECT_EQ(reason, "MAX GOALS REACHED");
  EXPECT_TRUE(editor.undo_last_goal());
  ASSERT_EQ(editor.goals().size(), 2U);
  EXPECT_DOUBLE_EQ(editor.goals().back().position.x(), 0.5);
  editor.clear_goals();
  EXPECT_TRUE(editor.goals().empty());
  EXPECT_EQ(editor.state(), GoalDraftState::Editing);
}

TEST(InteractiveGoalEditorTest, DraftChangesInvalidateReadyPreviewAndStaleResult)
{
  InteractiveGoalEditor editor(8U);
  add_at(editor, {0.5, 0.0, 1.5});
  std::uint64_t revision = 0U;
  std::vector<InteractiveGoal> goals;
  ASSERT_TRUE(editor.begin_validation(revision, goals));
  ASSERT_TRUE(editor.accept_validation(revision, true, "READY"));
  EXPECT_TRUE(editor.preview_valid());
  editor.set_candidate_position({0.6, 0.0, 1.5});
  EXPECT_FALSE(editor.preview_valid());
  EXPECT_FALSE(editor.accept_validation(revision, true, "STALE"));
  EXPECT_EQ(editor.state(), GoalDraftState::Editing);
}

TEST(InteractiveGoalEditorTest, EmptyValidationIsRejected)
{
  InteractiveGoalEditor editor(8U);
  std::uint64_t revision = 0U;
  std::vector<InteractiveGoal> goals;
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

TEST(InteractiveGoalEditorMarkerTest, SeparatesOuterTranslationRingFromInnerYawRing)
{
  const auto marker = make_goal_candidate_marker(
    {{0.0, 0.0, 1.5}, M_PI / 2.0}, 4U, GoalDraftState::Editing, "EDITING");
  std::size_t axes = 0U;
  std::size_t planes = 0U;
  std::size_t menus = 0U;
  double translation_ring_scale = 0.0;
  double yaw_ring_scale = 0.0;
  for (const auto & control : marker.controls) {
    EXPECT_NE(
      control.interaction_mode,
      visualization_msgs::msg::InteractiveMarkerControl::MOVE_3D);
    EXPECT_NE(
      control.interaction_mode,
      visualization_msgs::msg::InteractiveMarkerControl::MOVE_ROTATE);
    if (control.interaction_mode ==
      visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS)
    {
      ++axes;
      EXPECT_EQ(
        control.orientation_mode,
        visualization_msgs::msg::InteractiveMarkerControl::FIXED);
      if (control.name == "move_z") {
        EXPECT_NEAR(control.orientation.w, std::sqrt(0.5), 1.0e-12);
        EXPECT_NEAR(control.orientation.y, std::sqrt(0.5), 1.0e-12);
      } else {
        ADD_FAILURE() << "unexpected translation control: " << control.name;
      }
    } else if (control.interaction_mode ==
      visualization_msgs::msg::InteractiveMarkerControl::MOVE_PLANE)
    {
      ++planes;
      EXPECT_EQ(control.name, "move_xy");
      ASSERT_EQ(control.markers.size(), 1U);
      EXPECT_EQ(control.markers.front().type, visualization_msgs::msg::Marker::TRIANGLE_LIST);
      translation_ring_scale = control.markers.front().scale.x;
    } else if (control.interaction_mode ==
      visualization_msgs::msg::InteractiveMarkerControl::MENU)
    {
      ++menus;
    }
    if (control.interaction_mode ==
      visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS)
    {
      EXPECT_EQ(control.name, "rotate_z");
      ASSERT_EQ(control.markers.size(), 1U);
      EXPECT_EQ(control.markers.front().type, visualization_msgs::msg::Marker::TRIANGLE_LIST);
      yaw_ring_scale = control.markers.front().scale.x;
    }
  }
  EXPECT_EQ(axes, 1U);
  EXPECT_EQ(planes, 1U);
  EXPECT_EQ(menus, 1U);
  EXPECT_GT(translation_ring_scale, yaw_ring_scale * 2.0);
  EXPECT_EQ(
    std::count_if(
      marker.controls.begin(), marker.controls.end(), [](const auto & control) {
        return control.interaction_mode ==
               visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
      }),
    1);
}

TEST(InteractiveGoalEditorYawTest, PreservesIndependentYawAndUndoRestoresWholeGoal)
{
  InteractiveGoalEditor editor(8U);
  EXPECT_DOUBLE_EQ(editor.candidate().yaw, 0.0);
  add_at(editor, {1.0, 2.0, 1.5}, 179.0 * M_PI / 180.0);
  add_at(editor, {3.0, 4.0, 2.5}, -179.0 * M_PI / 180.0);
  ASSERT_EQ(editor.goals().size(), 2U);
  EXPECT_NEAR(editor.goals()[0].yaw, 179.0 * M_PI / 180.0, 1.0e-12);
  EXPECT_NEAR(editor.goals()[1].yaw, -179.0 * M_PI / 180.0, 1.0e-12);
  ASSERT_TRUE(editor.undo_last_goal());
  EXPECT_TRUE(editor.candidate().position.isApprox(Eigen::Vector3d(3.0, 4.0, 2.5)));
  EXPECT_NEAR(editor.candidate().yaw, -179.0 * M_PI / 180.0, 1.0e-12);
}

TEST(InteractiveGoalEditorYawTest, PositionAndYawEditsAreIndependentAndValidated)
{
  InteractiveGoalEditor editor(8U);
  const auto initial_revision = editor.draft_revision();
  ASSERT_TRUE(editor.set_candidate_yaw(M_PI / 2.0));
  EXPECT_EQ(editor.draft_revision(), initial_revision + 1U);
  EXPECT_TRUE(editor.candidate().position.isApprox(Eigen::Vector3d(0.0, 0.0, 1.5)));
  ASSERT_TRUE(editor.set_candidate_position({1.0, 2.0, 4.0}));
  EXPECT_DOUBLE_EQ(editor.candidate().yaw, M_PI / 2.0);
  EXPECT_FALSE(editor.set_candidate_yaw(std::numeric_limits<double>::infinity()));
  EXPECT_DOUBLE_EQ(editor.candidate().yaw, M_PI / 2.0);
  const auto revision = editor.draft_revision();
  ASSERT_TRUE(editor.set_candidate(editor.candidate()));
  EXPECT_EQ(editor.draft_revision(), revision);
}

TEST(InteractiveGoalEditorYawTest, QuaternionConversionNormalizesAndRejectsInvalidInput)
{
  const auto quaternion = quaternion_from_yaw(M_PI / 2.0);
  EXPECT_NEAR(quaternion.z, std::sqrt(0.5), 1.0e-12);
  EXPECT_NEAR(quaternion.w, std::sqrt(0.5), 1.0e-12);
  auto scaled = quaternion;
  scaled.z *= 3.0;
  scaled.w *= 3.0;
  ASSERT_TRUE(yaw_from_quaternion(scaled));
  EXPECT_NEAR(*yaw_from_quaternion(scaled), M_PI / 2.0, 1.0e-12);
  geometry_msgs::msg::Quaternion zero;
  zero.w = 0.0;
  EXPECT_FALSE(yaw_from_quaternion(zero));
  zero.w = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(yaw_from_quaternion(zero));
  EXPECT_THROW(
    quaternion_from_yaw(std::numeric_limits<double>::infinity()), std::invalid_argument);
  EXPECT_NEAR(normalize_angle(-M_PI), M_PI, 1.0e-12);
}

TEST(InteractiveGoalEditorYawTest, PoseArrayYamlAndDirectionMarkersPreserveYaw)
{
  const std::vector<InteractiveGoal> goals{
    {{1.0, 2.0, 1.5}, M_PI / 2.0}, {{3.0, 4.0, 2.5}, M_PI}};
  const auto poses = make_selected_goals(goals);
  ASSERT_EQ(poses.poses.size(), 2U);
  ASSERT_TRUE(yaw_from_quaternion(poses.poses[0].orientation));
  EXPECT_NEAR(*yaw_from_quaternion(poses.poses[0].orientation), M_PI / 2.0, 1.0e-12);
  const auto yaml = format_mission_yaml(goals);
  EXPECT_NE(yaml.find("1.570796"), std::string::npos);
  EXPECT_NE(yaml.find("3.141593"), std::string::npos);

  const auto markers = make_interactive_goal_markers(
    goals, GoalDraftState::Ready, std::nullopt);
  ASSERT_EQ(markers.markers.size(), 7U);
  EXPECT_EQ(markers.markers[2].type, visualization_msgs::msg::Marker::ARROW);
  ASSERT_TRUE(yaw_from_quaternion(markers.markers[2].pose.orientation));
  EXPECT_NEAR(*yaw_from_quaternion(markers.markers[2].pose.orientation), M_PI / 2.0, 1.0e-12);
  EXPECT_NE(markers.markers[3].text.find("yaw=90"), std::string::npos);
  std::set<std::pair<std::string, int>> identifiers;
  for (auto iterator = markers.markers.begin() + 1; iterator != markers.markers.end(); ++iterator) {
    EXPECT_TRUE(identifiers.emplace(iterator->ns, iterator->id).second);
  }
}

}  // namespace drone_planning
