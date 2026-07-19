#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "drone_mission/waypoint_manager.hpp"
#include "drone_msgs/srv/execute_goal_sequence.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int32.hpp"

namespace drone_mission
{
namespace
{

constexpr double kQuaternionNormMinimum = 1.0e-12;

bool finite_positive(double value) {return std::isfinite(value) && value > 0.0;}

std::vector<Waypoint> parse_waypoints(const std::vector<double> & values)
{
  if (values.empty() || values.size() % 4U != 0U) {
    throw std::invalid_argument("waypoints must be non-empty [x,y,z,yaw] groups");
  }
  std::vector<Waypoint> result;
  result.reserve(values.size() / 4U);
  for (std::size_t offset = 0U; offset < values.size(); offset += 4U) {
    const Waypoint waypoint{
      Eigen::Vector3d(values[offset], values[offset + 1U], values[offset + 2U]),
      values[offset + 3U]};
    if (!waypoint.position_world.allFinite() || !std::isfinite(waypoint.yaw)) {
      throw std::invalid_argument("all waypoint values must be finite");
    }
    result.push_back(waypoint);
  }
  return result;
}

std::optional<double> yaw_from_pose(const geometry_msgs::msg::Pose & pose)
{
  const auto & q = pose.orientation;
  const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  if (!std::isfinite(norm) || norm <= kQuaternionNormMinimum) {return std::nullopt;}
  const double scale = 1.0 / std::sqrt(norm);
  const double x = q.x * scale, y = q.y * scale, z = q.z * scale, w = q.w * scale;
  const double yaw = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
  return std::isfinite(yaw) ? std::optional<double>(yaw) : std::nullopt;
}

}  // namespace

class WaypointManagerNode : public rclcpp::Node
{
public:
  WaypointManagerNode()
  : Node("waypoint_manager_node")
  {
    const auto values = declare_parameter<std::vector<double>>(
      "waypoints", std::vector<double>{});
    const bool start_configured = declare_parameter<bool>("start_with_configured_waypoints", true);
    position_tolerance_ = declare_parameter<double>("position_tolerance", 0.20);
    linear_speed_tolerance_ = declare_parameter<double>("linear_speed_tolerance", 0.15);
    yaw_tolerance_ = declare_parameter<double>("yaw_tolerance", 0.10);
    angular_speed_tolerance_ = declare_parameter<double>("angular_speed_tolerance", 0.20);
    hold_duration_ = declare_parameter<double>("hold_duration", 1.0);
    const double frequency = declare_parameter<double>("update_frequency", 20.0);
    odometry_timeout_ = declare_parameter<double>("odometry_timeout", 0.25);
    frame_id_ = declare_parameter<std::string>("frame_id", "map");
    workspace_ = declare_parameter<std::vector<double>>(
      "workspace", {-1.0, 14.5, -2.5, 7.0, -0.5, 5.0});
    minimum_altitude_ = declare_parameter<double>("minimum_navigation_altitude", 0.0);
    if (!finite_positive(frequency) || !finite_positive(odometry_timeout_) ||
      frame_id_ != "map" || workspace_.size() != 6U || !std::isfinite(minimum_altitude_))
    {
      throw std::invalid_argument("invalid waypoint manager timing, frame, or workspace parameters");
    }
    fixed_update_dt_ = 1.0 / frequency;
    if (start_configured) {
      replace_mission(parse_waypoints(values));
    } else if (!values.empty()) {
      RCLCPP_INFO(get_logger(), "configured waypoints ignored; runtime mission mode is waiting");
    }

    odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      "/drone/odom", 10, [this](nav_msgs::msg::Odometry::SharedPtr message) {
        latest_odometry_ = *message;
        latest_odometry_reception_time_ = std::chrono::steady_clock::now();
      });
    goal_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>("/drone/goal", 10);
    const auto state_qos = rclcpp::QoS(1).transient_local().reliable();
    goals_publisher_ = create_publisher<geometry_msgs::msg::PoseArray>(
      "/drone/mission/goals", state_qos);
    waypoint_index_publisher_ = create_publisher<std_msgs::msg::UInt32>(
      "/drone/mission/current_waypoint_index", state_qos);
    mission_complete_publisher_ = create_publisher<std_msgs::msg::Bool>(
      "/drone/mission/complete", state_qos);
    execute_service_ = create_service<drone_msgs::srv::ExecuteGoalSequence>(
      "/drone/mission/execute",
      [this](
        const drone_msgs::srv::ExecuteGoalSequence::Request::SharedPtr request,
        drone_msgs::srv::ExecuteGoalSequence::Response::SharedPtr response)
      {
        handle_execute(*request, *response);
      });
    update_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(fixed_update_dt_)), [this]() {update();});
    publish_status();
    RCLCPP_INFO(
      get_logger(), manager_ ? "mission started with %zu waypoints" : "waiting for runtime mission",
      manager_ ? manager_->waypoints().size() : 0U);
  }

private:
  void replace_mission(std::vector<Waypoint> waypoints)
  {
    if (manager_) {
      manager_->replace_waypoints(std::move(waypoints));
    } else {
      manager_ = std::make_unique<WaypointManager>(
        std::move(waypoints), position_tolerance_, linear_speed_tolerance_, yaw_tolerance_,
        angular_speed_tolerance_, hold_duration_);
    }
    odometry_stale_ = false;
    invalid_odometry_logged_ = false;
  }

  std::optional<std::string> validate_request(
    const drone_msgs::srv::ExecuteGoalSequence::Request & request,
    std::vector<Waypoint> & result) const
  {
    if (request.goals.header.frame_id != frame_id_) {return "goals frame_id must be map";}
    if (request.goals.poses.empty()) {return "goal list is empty";}
    result.clear();
    result.reserve(request.goals.poses.size());
    for (std::size_t index = 0U; index < request.goals.poses.size(); ++index) {
      const auto & pose = request.goals.poses[index];
      const auto yaw = yaw_from_pose(pose);
      const bool finite_position = std::isfinite(pose.position.x) &&
        std::isfinite(pose.position.y) && std::isfinite(pose.position.z);
      if (!finite_position || !yaw) {return "P" + std::to_string(index + 1U) + " is non-finite or has an invalid quaternion";}
      if (pose.position.x < workspace_[0] || pose.position.x > workspace_[1] ||
        pose.position.y < workspace_[2] || pose.position.y > workspace_[3] ||
        pose.position.z < std::max(workspace_[4], minimum_altitude_) ||
        pose.position.z > workspace_[5])
      {
        return "P" + std::to_string(index + 1U) + " is outside workspace/height bounds";
      }
      result.push_back({Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z), *yaw});
    }
    return std::nullopt;
  }

  void handle_execute(
    const drone_msgs::srv::ExecuteGoalSequence::Request & request,
    drone_msgs::srv::ExecuteGoalSequence::Response & response)
  {
    if (manager_ && !manager_->mission_complete()) {
      response.message = "another waypoint mission is active; preemption is disabled";
      return;
    }
    std::vector<Waypoint> waypoints;
    if (const auto error = validate_request(request, waypoints)) {
      response.message = *error;
      return;
    }
    replace_mission(std::move(waypoints));
    response.accepted = true;
    response.message = "mission accepted and state reset to P1";
    publish_status();
    RCLCPP_INFO(get_logger(), "accepted runtime mission with %zu waypoints", manager_->waypoints().size());
  }

  bool vehicle_state_from_odometry(const nav_msgs::msg::Odometry & message, VehicleState & state)
  {
    const auto & pose = message.pose.pose;
    const auto yaw = yaw_from_pose(pose);
    if (!yaw) {return false;}
    state.position_world = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
    state.linear_velocity = Eigen::Vector3d(
      message.twist.twist.linear.x, message.twist.twist.linear.y, message.twist.twist.linear.z);
    state.angular_velocity = Eigen::Vector3d(
      message.twist.twist.angular.x, message.twist.twist.angular.y,
      message.twist.twist.angular.z);
    state.yaw = *yaw;
    return state.position_world.allFinite() && state.linear_velocity.allFinite() &&
           state.angular_velocity.allFinite();
  }

  void publish_status()
  {
    geometry_msgs::msg::PoseArray goals;
    goals.header.frame_id = frame_id_;
    goals.header.stamp = now();
    if (manager_) {
      goals.poses.reserve(manager_->waypoints().size());
      for (const auto & waypoint : manager_->waypoints()) {
        geometry_msgs::msg::Pose pose;
        pose.position.x = waypoint.position_world.x();
        pose.position.y = waypoint.position_world.y();
        pose.position.z = waypoint.position_world.z();
        pose.orientation.z = std::sin(0.5 * waypoint.yaw);
        pose.orientation.w = std::cos(0.5 * waypoint.yaw);
        goals.poses.push_back(pose);
      }
    }
    goals_publisher_->publish(goals);
    std_msgs::msg::UInt32 index;
    index.data = manager_ ? static_cast<std::uint32_t>(manager_->current_index()) : 0U;
    waypoint_index_publisher_->publish(index);
    std_msgs::msg::Bool complete;
    complete.data = manager_ && manager_->mission_complete();
    mission_complete_publisher_->publish(complete);
  }

  void publish_goal()
  {
    if (!manager_) {return;}
    const auto & waypoint = manager_->current_waypoint();
    geometry_msgs::msg::PoseStamped goal;
    goal.header.stamp = now(); goal.header.frame_id = frame_id_;
    goal.pose.position.x = waypoint.position_world.x();
    goal.pose.position.y = waypoint.position_world.y();
    goal.pose.position.z = waypoint.position_world.z();
    goal.pose.orientation.z = std::sin(0.5 * waypoint.yaw);
    goal.pose.orientation.w = std::cos(0.5 * waypoint.yaw);
    goal_publisher_->publish(goal);
  }

  void update()
  {
    publish_goal();
    publish_status();
    if (!manager_) {return;}
    const auto steady_now = std::chrono::steady_clock::now();
    const bool fresh = latest_odometry_ && latest_odometry_reception_time_ &&
      std::chrono::duration<double>(steady_now - *latest_odometry_reception_time_).count() <=
      odometry_timeout_;
    if (!fresh) {
      if (!odometry_stale_) {manager_->reset_acceptance_progress(); odometry_stale_ = true;}
      return;
    }
    odometry_stale_ = false;
    VehicleState state;
    if (!vehicle_state_from_odometry(*latest_odometry_, state)) {
      manager_->reset_acceptance_progress(); invalid_odometry_logged_ = true; return;
    }
    invalid_odometry_logged_ = false;
    const std::size_t accepted = manager_->current_index();
    const auto output = manager_->update(state, fixed_update_dt_);
    if (output.waypoint_accepted) {
      RCLCPP_INFO(get_logger(), "waypoint %zu accepted%s", accepted,
        output.mission_complete ? "; mission complete" : "");
      publish_goal(); publish_status();
    }
  }

  std::unique_ptr<WaypointManager> manager_;
  double position_tolerance_{0.20}, linear_speed_tolerance_{0.15};
  double yaw_tolerance_{0.10}, angular_speed_tolerance_{0.20}, hold_duration_{1.0};
  double fixed_update_dt_{0.05}, odometry_timeout_{0.25}, minimum_altitude_{0.0};
  std::string frame_id_{"map"};
  std::vector<double> workspace_;
  bool odometry_stale_{false}, invalid_odometry_logged_{false};
  std::optional<nav_msgs::msg::Odometry> latest_odometry_;
  std::optional<std::chrono::steady_clock::time_point> latest_odometry_reception_time_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr goals_publisher_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr waypoint_index_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr mission_complete_publisher_;
  rclcpp::Service<drone_msgs::srv::ExecuteGoalSequence>::SharedPtr execute_service_;
  rclcpp::TimerBase::SharedPtr update_timer_;
};

}  // namespace drone_mission

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<drone_mission::WaypointManagerNode>());
  rclcpp::shutdown();
  return 0;
}
