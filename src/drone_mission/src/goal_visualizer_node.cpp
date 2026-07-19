#include <chrono>
#include <cmath>
#include <cstddef>
#include <memory>
#include <optional>
#include <stdexcept>

#include "drone_mission/goal_visualization.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
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
    position_tolerance_ = declare_parameter<double>("single_goal_position_tolerance", 0.20);
    linear_speed_tolerance_ = declare_parameter<double>(
      "single_goal_linear_speed_tolerance", 0.15);
    yaw_tolerance_ = declare_parameter<double>("single_goal_yaw_tolerance", 0.10);
    angular_speed_tolerance_ = declare_parameter<double>(
      "single_goal_angular_speed_tolerance", 0.20);
    hold_duration_ = declare_parameter<double>("single_goal_hold_duration", 1.0);
    if (!finite_positive(position_tolerance_) || !finite_positive(linear_speed_tolerance_) ||
      !finite_positive(yaw_tolerance_) || !finite_positive(angular_speed_tolerance_) ||
      !finite_positive(hold_duration_))
    {
      throw std::invalid_argument("single-goal visualization tolerances must be positive");
    }
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
        single_goal_complete_ = false;
        single_goal_stable_since_.reset();
        publish();
      });
    odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      "/drone/odom", 10,
      [this](nav_msgs::msg::Odometry::SharedPtr message) {update_single_goal_state(*message);});
  }

private:
  static bool finite_positive(double value)
  {
    return std::isfinite(value) && value > 0.0;
  }

  void update_single_goal_state(const nav_msgs::msg::Odometry & odometry)
  {
    if (!latest_goal_ || single_goal_complete_ ||
      (mission_goals_ && !mission_goals_->poses.empty()))
    {
      return;
    }
    if (!single_goal_within_tolerance(
        latest_goal_->pose, odometry, position_tolerance_, linear_speed_tolerance_,
        yaw_tolerance_, angular_speed_tolerance_))
    {
      single_goal_stable_since_.reset();
      return;
    }
    const auto steady_now = std::chrono::steady_clock::now();
    if (!single_goal_stable_since_) {
      single_goal_stable_since_ = steady_now;
      return;
    }
    if (std::chrono::duration<double>(steady_now - *single_goal_stable_since_).count() >=
      hold_duration_)
    {
      single_goal_complete_ = true;
      publish();
      RCLCPP_INFO(get_logger(), "single goal reached and held; visualization marked DONE");
    }
  }

  void publish()
  {
    if (mission_goals_ && !mission_goals_->poses.empty()) {
      marker_publisher_->publish(make_mission_goal_markers(
        *mission_goals_, current_index_, mission_complete_, now()));
    } else if (latest_goal_) {
      marker_publisher_->publish(make_single_goal_markers(
        latest_goal_->pose, latest_goal_->header.frame_id, now(), single_goal_complete_));
    }
  }

  std::optional<geometry_msgs::msg::PoseArray> mission_goals_;
  std::optional<geometry_msgs::msg::PoseStamped> latest_goal_;
  std::size_t current_index_{0U};
  bool mission_complete_{false};
  bool single_goal_complete_{false};
  double position_tolerance_{0.20};
  double linear_speed_tolerance_{0.15};
  double yaw_tolerance_{0.10};
  double angular_speed_tolerance_{0.20};
  double hold_duration_{1.0};
  std::optional<std::chrono::steady_clock::time_point> single_goal_stable_since_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr mission_goals_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
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
