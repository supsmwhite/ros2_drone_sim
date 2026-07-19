#include <cstddef>
#include <memory>
#include <optional>

#include "drone_mission/goal_visualization.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int32.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace drone_mission
{

class GoalVisualizerNode : public rclcpp::Node
{
public:
  GoalVisualizerNode()
  : Node("goal_visualizer_node")
  {
    const auto state_qos = rclcpp::QoS(1).transient_local().reliable();
    marker_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/drone/mission/goal_markers", state_qos);
    mission_goals_subscription_ = create_subscription<geometry_msgs::msg::PoseArray>(
      "/drone/mission/goals", state_qos,
      [this](geometry_msgs::msg::PoseArray::SharedPtr message) {
        mission_goals_ = *message;
        publish();
      });
    index_subscription_ = create_subscription<std_msgs::msg::UInt32>(
      "/drone/mission/current_waypoint_index", state_qos,
      [this](std_msgs::msg::UInt32::SharedPtr message) {
        current_index_ = message->data;
        publish();
      });
    complete_subscription_ = create_subscription<std_msgs::msg::Bool>(
      "/drone/mission/complete", state_qos,
      [this](std_msgs::msg::Bool::SharedPtr message) {
        mission_complete_ = message->data;
        publish();
      });
    goal_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/drone/goal", 10,
      [this](geometry_msgs::msg::PoseStamped::SharedPtr message) {
        latest_goal_ = *message;
        if (!mission_goals_ || mission_goals_->poses.empty()) {
          publish();
        }
      });
  }

private:
  void publish()
  {
    if (mission_goals_ && !mission_goals_->poses.empty()) {
      marker_publisher_->publish(make_mission_goal_markers(
        *mission_goals_, current_index_, mission_complete_, now()));
    } else if (latest_goal_) {
      marker_publisher_->publish(make_single_goal_markers(
        latest_goal_->pose, latest_goal_->header.frame_id, now()));
    }
  }

  std::optional<geometry_msgs::msg::PoseArray> mission_goals_;
  std::optional<geometry_msgs::msg::PoseStamped> latest_goal_;
  std::size_t current_index_{0U};
  bool mission_complete_{false};
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr mission_goals_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_subscription_;
  rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr index_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr complete_subscription_;
};

}  // namespace drone_mission

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<drone_mission::GoalVisualizerNode>());
  rclcpp::shutdown();
  return 0;
}
