#include <set>

#include <gtest/gtest.h>

#include "drone_mission/goal_visualization.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace drone_mission
{

TEST(GoalVisualization, SingleGoalHasPointArrowLabelAndClear)
{
  const builtin_interfaces::msg::Time stamp;
  geometry_msgs::msg::Pose pose;
  pose.orientation.w = 1.0;
  const auto markers = make_single_goal_markers(pose, "map", stamp);
  ASSERT_EQ(markers.markers.size(), 4U);
  EXPECT_EQ(markers.markers.front().action, visualization_msgs::msg::Marker::DELETEALL);
  EXPECT_EQ(markers.markers[1].id, 100);
  EXPECT_EQ(markers.markers[2].type, visualization_msgs::msg::Marker::ARROW);
  EXPECT_EQ(markers.markers[3].text, "GOAL CURRENT");

  EXPECT_FLOAT_EQ(markers.markers[1].color.r, 1.0F);
  EXPECT_FLOAT_EQ(markers.markers[1].color.g, 0.35F);
}

TEST(GoalVisualization, MissionMarkersAreStableAndClearOldTask)
{
  const builtin_interfaces::msg::Time stamp;
  geometry_msgs::msg::PoseArray goals;
  goals.header.frame_id = "map";
  goals.poses.resize(3);
  for (auto & pose : goals.poses) {pose.orientation.w = 1.0;}
  const auto markers = make_mission_goal_markers(goals, 1U, false, stamp);
  ASSERT_EQ(markers.markers.front().action, visualization_msgs::msg::Marker::DELETEALL);
  EXPECT_EQ(markers.markers.size(), 11U);
  std::set<int> ids;
  for (std::size_t index = 1U; index < markers.markers.size(); ++index) {
    EXPECT_EQ(markers.markers[index].header.frame_id, "map");
    ids.insert(markers.markers[index].id);
  }
  EXPECT_GE(ids.size(), 9U);
  EXPECT_EQ(markers.markers[7].text, "P2 CURRENT");

  goals.poses.resize(1);
  const auto replacement = make_mission_goal_markers(goals, 0U, false, stamp);
  EXPECT_EQ(replacement.markers.front().action, visualization_msgs::msg::Marker::DELETEALL);
  EXPECT_EQ(replacement.markers.size(), 4U);
}

TEST(GoalVisualization, CompleteMissionMarksEveryGoalDone)
{
  const builtin_interfaces::msg::Time stamp;
  geometry_msgs::msg::PoseArray goals;
  goals.header.frame_id = "map";
  goals.poses.resize(2);
  const auto markers = make_mission_goal_markers(goals, 1U, true, stamp);
  EXPECT_EQ(markers.markers[4].text, "P1 DONE");
  EXPECT_EQ(markers.markers[7].text, "P2 DONE");
}

}  // namespace drone_mission
