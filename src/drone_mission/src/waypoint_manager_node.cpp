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

bool finite_positive(double value)
{
  return std::isfinite(value) && value > 0.0;
}

std::vector<Waypoint> parse_waypoints(const std::vector<double> & values)
{
  if (values.empty() || values.size() % 4U != 0U) {
    throw std::invalid_argument(
            "waypoints must be a non-empty flat [x,y,z,yaw,...] array");
  }

  std::vector<Waypoint> waypoints;
  waypoints.reserve(values.size() / 4U);
  for (std::size_t offset = 0U; offset < values.size(); offset += 4U) {
    if (!std::isfinite(values[offset]) || !std::isfinite(values[offset + 1U]) ||
      !std::isfinite(values[offset + 2U]) || !std::isfinite(values[offset + 3U]))
    {
      throw std::invalid_argument("all waypoint values must be finite");
    }
    waypoints.push_back(Waypoint{
      Eigen::Vector3d(values[offset], values[offset + 1U], values[offset + 2U]),
      values[offset + 3U]});
  }
  return waypoints;
}

}  // namespace

class WaypointManagerNode : public rclcpp::Node
{
public:
  WaypointManagerNode()
  : Node("waypoint_manager_node")
  {
    const auto waypoint_values = declare_parameter<std::vector<double>>(
      "waypoints", std::vector<double>{});
    const double position_tolerance = declare_parameter<double>("position_tolerance", 0.20);
    const double linear_speed_tolerance =
      declare_parameter<double>("linear_speed_tolerance", 0.15);
    const double yaw_tolerance = declare_parameter<double>("yaw_tolerance", 0.10);
    const double angular_speed_tolerance =
      declare_parameter<double>("angular_speed_tolerance", 0.20);
    const double hold_duration = declare_parameter<double>("hold_duration", 1.0);
    const double update_frequency = declare_parameter<double>("update_frequency", 20.0);
    odometry_timeout_ = declare_parameter<double>("odometry_timeout", 0.25);
    if (!finite_positive(update_frequency) || !finite_positive(odometry_timeout_)) {
      throw std::invalid_argument("update_frequency and odometry_timeout must be finite and positive");
    }

    waypoints_ = parse_waypoints(waypoint_values);
    manager_ = std::make_unique<WaypointManager>(
      waypoints_, position_tolerance, linear_speed_tolerance, yaw_tolerance,
      angular_speed_tolerance, hold_duration);
    fixed_update_dt_ = 1.0 / update_frequency;

    odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      "/drone/odom", 10,
      [this](const nav_msgs::msg::Odometry::SharedPtr message) {
        latest_odometry_ = *message;
        latest_odometry_reception_time_ = std::chrono::steady_clock::now();
      });
    goal_publisher_ =
      create_publisher<geometry_msgs::msg::PoseStamped>("/drone/goal", 10);
    waypoint_index_publisher_ = create_publisher<std_msgs::msg::UInt32>(
      "/drone/mission/current_waypoint_index", 10);
    mission_complete_publisher_ = create_publisher<std_msgs::msg::Bool>(
      "/drone/mission/complete", 10);

    const auto period = std::chrono::duration<double>(fixed_update_dt_);
    update_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() {update();});

    RCLCPP_INFO(
      get_logger(), "mission started with %zu waypoints", waypoints_.size());
  }

private:
  bool vehicle_state_from_odometry(const nav_msgs::msg::Odometry & message, VehicleState & state)
  {
    const auto & pose = message.pose.pose;
    const auto & twist = message.twist.twist;
    const double qx = pose.orientation.x;
    const double qy = pose.orientation.y;
    const double qz = pose.orientation.z;
    const double qw = pose.orientation.w;
    const double norm_squared = qx * qx + qy * qy + qz * qz + qw * qw;
    if (!std::isfinite(norm_squared) || norm_squared <= kQuaternionNormMinimum) {
      return false;
    }
    const double inverse_norm = 1.0 / std::sqrt(norm_squared);
    const double nx = qx * inverse_norm;
    const double ny = qy * inverse_norm;
    const double nz = qz * inverse_norm;
    const double nw = qw * inverse_norm;

    state.position_world = Eigen::Vector3d(
      pose.position.x, pose.position.y, pose.position.z);
    state.linear_velocity = Eigen::Vector3d(
      twist.linear.x, twist.linear.y, twist.linear.z);
    state.angular_velocity = Eigen::Vector3d(
      twist.angular.x, twist.angular.y, twist.angular.z);
    state.yaw = std::atan2(
      2.0 * (nw * nz + nx * ny),
      1.0 - 2.0 * (ny * ny + nz * nz));
    return state.position_world.allFinite() && state.linear_velocity.allFinite() &&
           state.angular_velocity.allFinite() && std::isfinite(state.yaw);
  }

  void publish_goal_and_status()
  {
    const Waypoint & waypoint = manager_->current_waypoint();
    geometry_msgs::msg::PoseStamped goal;
    goal.header.stamp = now();
    goal.header.frame_id = "map";
    goal.pose.position.x = waypoint.position_world.x();
    goal.pose.position.y = waypoint.position_world.y();
    goal.pose.position.z = waypoint.position_world.z();
    goal.pose.orientation.z = std::sin(0.5 * waypoint.yaw);
    goal.pose.orientation.w = std::cos(0.5 * waypoint.yaw);
    goal_publisher_->publish(goal);

    std_msgs::msg::UInt32 index;
    index.data = static_cast<std::uint32_t>(manager_->current_index());
    waypoint_index_publisher_->publish(index);
    std_msgs::msg::Bool complete;
    complete.data = manager_->mission_complete();
    mission_complete_publisher_->publish(complete);
  }

  void update()
  {
    publish_goal_and_status();

    const auto steady_now = std::chrono::steady_clock::now();
    const bool odometry_fresh = latest_odometry_.has_value() &&
      latest_odometry_reception_time_.has_value() &&
      std::chrono::duration<double>(steady_now - *latest_odometry_reception_time_).count() <=
      odometry_timeout_;
    if (!odometry_fresh) {
      if (!odometry_stale_) {
        manager_->reset_acceptance_progress();
        RCLCPP_WARN(get_logger(), "odometry stale; waypoint progression paused");
        odometry_stale_ = true;
      }
      return;
    }
    if (odometry_stale_) {
      RCLCPP_INFO(get_logger(), "odometry recovered; waypoint progression resumed");
      odometry_stale_ = false;
    }

    VehicleState state;
    if (!vehicle_state_from_odometry(*latest_odometry_, state)) {
      if (!invalid_odometry_logged_) {
        manager_->reset_acceptance_progress();
        RCLCPP_WARN(get_logger(), "invalid odometry; waypoint progression paused");
        invalid_odometry_logged_ = true;
      }
      return;
    }
    invalid_odometry_logged_ = false;

    const std::size_t accepted_index = manager_->current_index();
    const WaypointManagerOutput output = manager_->update(state, fixed_update_dt_);
    if (!output.waypoint_accepted) {
      return;
    }

    RCLCPP_INFO(get_logger(), "waypoint %zu accepted", accepted_index);
    if (output.mission_complete) {
      RCLCPP_INFO(get_logger(), "mission complete; holding final waypoint");
    } else {
      RCLCPP_INFO(get_logger(), "switching to waypoint %zu", output.current_index);
    }
    publish_goal_and_status();
  }

  std::vector<Waypoint> waypoints_;
  std::unique_ptr<WaypointManager> manager_;
  double fixed_update_dt_{0.05};
  double odometry_timeout_{0.25};
  bool odometry_stale_{false};
  bool invalid_odometry_logged_{false};
  std::optional<nav_msgs::msg::Odometry> latest_odometry_;
  std::optional<std::chrono::steady_clock::time_point> latest_odometry_reception_time_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_publisher_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr waypoint_index_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr mission_complete_publisher_;
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
