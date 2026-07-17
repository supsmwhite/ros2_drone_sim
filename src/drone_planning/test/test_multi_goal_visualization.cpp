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
  EXPECT_THROW(parse_goals({0.0, 0.0, 1.5, 0.1}), std::invalid_argument);
}

TEST(MultiGoalVisualizationTest, MarksCurrentCompletedAndWaitingGoals)
{
  const auto goals = parse_goals(goal_values(5U));
  const builtin_interfaces::msg::Time stamp;
  const auto markers = make_goal_markers(
    goals, 1U, 1U, MissionVisualizationState::Running, "map", stamp, 0.35);
  ASSERT_EQ(markers.markers.size(), 11U);
  EXPECT_EQ(markers.markers[0].ns, "multi_goal_points");
  EXPECT_FLOAT_EQ(markers.markers[0].color.g, 0.85F);
  EXPECT_NE(markers.markers[1].text.find("DONE"), std::string::npos);
  EXPECT_FLOAT_EQ(markers.markers[2].scale.x, 0.40);
  EXPECT_FLOAT_EQ(markers.markers[2].color.r, 1.00F);
  EXPECT_NE(markers.markers[3].text.find("CURRENT"), std::string::npos);
  EXPECT_FLOAT_EQ(markers.markers[4].color.r, 0.95F);
  EXPECT_NE(markers.markers[5].text.find("WAITING"), std::string::npos);
  EXPECT_NE(markers.markers.back().text.find("Goal: P2 / 5"), std::string::npos);
}

TEST(MultiGoalVisualizationTest, MarksEveryGoalComplete)
{
  const auto goals = parse_goals(goal_values(3U));
  const builtin_interfaces::msg::Time stamp;
  const auto markers = make_goal_markers(
    goals, 2U, 3U, MissionVisualizationState::Complete, "map", stamp, 0.35);
  ASSERT_EQ(markers.markers.size(), 7U);
  for (std::size_t index = 0U; index < goals.size(); ++index) {
    EXPECT_FLOAT_EQ(markers.markers[2U * index].color.g, 0.85F);
    EXPECT_NE(markers.markers[2U * index + 1U].text.find("DONE"), std::string::npos);
  }
  EXPECT_NE(markers.markers.back().text.find("MISSION COMPLETE"), std::string::npos);
}

}  // namespace
}  // namespace drone_planning
