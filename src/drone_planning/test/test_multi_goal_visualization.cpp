#include <limits>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

#include "drone_planning/multi_goal_visualization.hpp"

namespace drone_planning
{
namespace
{

std::vector<double> goal_values(std::size_t count)
{
  std::vector<double> values;
  for (std::size_t index = 0U; index < count; ++index) {
    values.insert(values.end(), {
      static_cast<double>(index), 0.5 * static_cast<double>(index), 1.5, 0.0});
  }
  return values;
}

TEST(MultiGoalVisualizationTest, ParsesOneThreeAndFiveGoals)
{
  EXPECT_EQ(parse_goals(goal_values(1U)).size(), 1U);
  EXPECT_EQ(parse_goals(goal_values(3U)).size(), 3U);
  EXPECT_EQ(parse_goals(goal_values(5U)).size(), 5U);
}

TEST(MultiGoalVisualizationTest, RejectsInvalidGoalLists)
{
  EXPECT_THROW(parse_goals({}), std::invalid_argument);
  EXPECT_THROW(parse_goals({0.0, 0.0, 1.5}), std::invalid_argument);
  EXPECT_THROW(
    parse_goals({0.0, 0.0, std::numeric_limits<double>::quiet_NaN(), 0.0}),
    std::invalid_argument);
  EXPECT_THROW(
    parse_goals({0.0, 0.0, 1.5, std::numeric_limits<double>::infinity()}),
    std::invalid_argument);
  const auto nonzero_yaw = parse_goals({0.0, 0.0, 1.5, 0.1});
  ASSERT_EQ(nonzero_yaw.size(), 1U);
  EXPECT_DOUBLE_EQ(nonzero_yaw.front().yaw, 0.1);
}

TEST(MultiGoalVisualizationTest, MarksCurrentCompletedAndWaitingGoals)
{
  const auto goals = parse_goals(goal_values(5U));
  const builtin_interfaces::msg::Time stamp;
  const auto markers = make_goal_markers(
    goals, 1U, 1U, MissionVisualizationState::Running, "map", stamp, 0.42, 0.45, 0.35);
  ASSERT_EQ(markers.markers.size(), 16U);
  EXPECT_EQ(markers.markers[0].ns, "multi_goal_points");
  EXPECT_FLOAT_EQ(markers.markers[0].color.g, 0.85F);
  EXPECT_EQ(markers.markers[1].type, visualization_msgs::msg::Marker::ARROW);
  EXPECT_EQ(markers.markers[2].text, "P1 DONE\n(0.00,0.00,1.50)  yaw=0°");
  EXPECT_FLOAT_EQ(markers.markers[3].scale.x, 0.40);
  EXPECT_FLOAT_EQ(markers.markers[3].color.r, 1.00F);
  EXPECT_EQ(markers.markers[5].text, "P2 CURRENT\n(1.00,0.50,1.50)  yaw=0°");
  EXPECT_FLOAT_EQ(markers.markers[6].color.r, 0.95F);
  EXPECT_EQ(markers.markers[8].text, "P3 WAITING\n(2.00,1.00,1.50)  yaw=0°");
  EXPECT_NE(markers.markers.back().text.find("Goal: P2 / 5"), std::string::npos);
  EXPECT_NE(markers.markers.back().text.find("Actual: 0.42 m/s"), std::string::npos);
  EXPECT_NE(markers.markers.back().text.find("Reference: 0.45 m/s"), std::string::npos);
  EXPECT_NE(markers.markers.back().text.find("Nominal: 0.35 m/s"), std::string::npos);
  EXPECT_EQ(markers.markers.back().text.find("Speed:"), std::string::npos);
}

TEST(MultiGoalVisualizationTest, ShowsUnavailableActualSpeedExplicitly)
{
  const auto goals = parse_goals(goal_values(1U));
  const builtin_interfaces::msg::Time stamp;
  const auto markers = make_goal_markers(
    goals, 0U, 0U, MissionVisualizationState::Running, "map", stamp,
    std::nullopt, 0.0, 0.35);
  EXPECT_NE(markers.markers.back().text.find("Actual: --"), std::string::npos);
  EXPECT_NE(markers.markers.back().text.find("Reference: 0.00 m/s"), std::string::npos);
  EXPECT_NE(markers.markers.back().text.find("Nominal: 0.35 m/s"), std::string::npos);
}

TEST(MultiGoalVisualizationTest, MarksEveryGoalComplete)
{
  const auto goals = parse_goals(goal_values(3U));
  const builtin_interfaces::msg::Time stamp;
  const auto markers = make_goal_markers(
    goals, 2U, 3U, MissionVisualizationState::Complete, "map", stamp, 0.0, 0.0, 0.35);
  ASSERT_EQ(markers.markers.size(), 10U);
  for (std::size_t index = 0U; index < goals.size(); ++index) {
    EXPECT_FLOAT_EQ(markers.markers[3U * index].color.g, 0.85F);
    EXPECT_NE(markers.markers[3U * index + 2U].text.find("DONE"), std::string::npos);
  }
  EXPECT_NE(markers.markers.back().text.find("MISSION COMPLETE"), std::string::npos);
  EXPECT_NE(markers.markers.back().text.find("Reference: 0.00 m/s"), std::string::npos);
}

}  // namespace
}  // namespace drone_planning
