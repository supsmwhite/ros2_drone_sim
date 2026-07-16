#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include "drone_mission/piecewise_quintic_trajectory.hpp"
#include "drone_msgs/msg/trajectory_setpoint.hpp"
#include "drone_planning/planned_trajectory_builder.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/u_int32.hpp"

namespace drone_planning
{
namespace
{

bool finite_positive(double value)
{
  return std::isfinite(value) && value > 0.0;
}

AxisAlignedBox parse_workspace(const std::vector<double> & values)
{
  if (values.size() != 6U) {
    throw std::invalid_argument("workspace must be [xmin,xmax,ymin,ymax,zmin,zmax]");
  }
  return {
    Eigen::Vector3d(values[0], values[2], values[4]),
    Eigen::Vector3d(values[1], values[3], values[5])};
}

std::vector<AxisAlignedBox> parse_obstacles(const std::vector<double> & values)
{
  if (values.size() % 6U != 0U) {
    throw std::invalid_argument("obstacles must contain flat center and size groups");
  }
  std::vector<AxisAlignedBox> obstacles;
  obstacles.reserve(values.size() / 6U);
  for (std::size_t offset = 0U; offset < values.size(); offset += 6U) {
    const Eigen::Vector3d center(values[offset], values[offset + 1U], values[offset + 2U]);
    const Eigen::Vector3d size(values[offset + 3U], values[offset + 4U], values[offset + 5U]);
    if (!center.allFinite() || !size.allFinite() || (size.array() <= 0.0).any()) {
      throw std::invalid_argument("obstacle centers must be finite and sizes finite and positive");
    }
    obstacles.push_back({center - 0.5 * size, center + 0.5 * size});
  }
  return obstacles;
}

double path_length(const std::vector<Eigen::Vector3d> & path)
{
  double length = 0.0;
  for (std::size_t index = 1U; index < path.size(); ++index) {
    length += (path[index] - path[index - 1U]).norm();
  }
  return length;
}

}  // namespace

class PlannedTrajectoryNode : public rclcpp::Node
{
public:
  PlannedTrajectoryNode()
  : Node("planned_trajectory_node")
  {
    frame_id_ = declare_parameter<std::string>("frame_id", "map");
    if (frame_id_ != "map") {
      throw std::invalid_argument("planned trajectory frame_id must be map");
    }
    const auto workspace_values =
      declare_parameter<std::vector<double>>("workspace", std::vector<double>{});
    const auto obstacle_values =
      declare_parameter<std::vector<double>>("obstacles", std::vector<double>{});
    const double safety_radius = declare_parameter<double>("safety_radius", 0.25);
    const double planning_margin = declare_parameter<double>("planning_margin", 0.10);
    if (!std::isfinite(planning_margin) || planning_margin < 0.0) {
      throw std::invalid_argument("planning_margin must be finite and non-negative");
    }
    const double effective_planning_radius = safety_radius + planning_margin;
    if (!std::isfinite(effective_planning_radius)) {
      throw std::invalid_argument("effective planning radius must be finite");
    }

    PlannedTrajectoryParameters parameters;
    parameters.nominal_speed = declare_parameter<double>("nominal_speed", 0.35);
    parameters.min_segment_duration =
      declare_parameter<double>("min_segment_duration", 2.0);
    parameters.validation_sample_period =
      declare_parameter<double>("validation_sample_period", 0.02);
    reference_path_sample_period_ =
      declare_parameter<double>("reference_path_sample_period", 0.05);
    parameters.max_reference_speed =
      declare_parameter<double>("max_reference_speed", 0.70);
    parameters.max_reference_acceleration =
      declare_parameter<double>("max_reference_acceleration", 0.35);
    parameters.velocity_scale_candidates = declare_parameter<std::vector<double>>(
      "velocity_scale_candidates", {1.0, 0.75, 0.5, 0.25, 0.0});
    parameters.fixed_yaw = declare_parameter<double>("fixed_yaw", 0.0);

    execution_enabled_ = declare_parameter<bool>("execution_enabled", false);
    const double publish_frequency = declare_parameter<double>("publish_frequency", 50.0);
    odometry_timeout_ = declare_parameter<double>("odometry_timeout", 0.25);
    preparation_position_tolerance_ =
      declare_parameter<double>("preparation_position_tolerance", 0.20);
    preparation_speed_tolerance_ =
      declare_parameter<double>("preparation_speed_tolerance", 0.15);
    preparation_hold_duration_ =
      declare_parameter<double>("preparation_hold_duration", 1.0);
    if (!finite_positive(reference_path_sample_period_) || !finite_positive(publish_frequency) ||
      !finite_positive(odometry_timeout_) ||
      !finite_positive(preparation_position_tolerance_) ||
      !finite_positive(preparation_speed_tolerance_) ||
      !finite_positive(preparation_hold_duration_))
    {
      throw std::invalid_argument("planned trajectory timing and tolerances must be positive");
    }

    builder_ = std::make_unique<PlannedTrajectoryBuilder>(
      CollisionChecker(
        StaticEnvironment(parse_workspace(workspace_values), parse_obstacles(obstacle_values)),
        effective_planning_radius),
      parameters);

    const auto result_qos = rclcpp::QoS(1).transient_local().reliable();
    simplified_path_publisher_ = create_publisher<nav_msgs::msg::Path>(
      "/drone/simplified_path", result_qos);
    reference_path_publisher_ = create_publisher<nav_msgs::msg::Path>(
      "/drone/reference_path", result_qos);
    success_publisher_ = create_publisher<std_msgs::msg::Bool>(
      "/drone/trajectory_generation/success", result_qos);
    simplified_waypoints_publisher_ = create_publisher<std_msgs::msg::UInt32>(
      "/drone/trajectory_generation/simplified_waypoints", result_qos);
    selected_velocity_scale_publisher_ = create_publisher<std_msgs::msg::Float64>(
      "/drone/trajectory_generation/selected_velocity_scale", result_qos);
    duration_publisher_ = create_publisher<std_msgs::msg::Float64>(
      "/drone/trajectory_generation/duration", result_qos);
    planned_path_subscription_ = create_subscription<nav_msgs::msg::Path>(
      "/drone/planned_path", result_qos,
      [this](const nav_msgs::msg::Path::SharedPtr message) {handle_path(*message);});

    if (execution_enabled_) {
      odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
        "/drone/odom", 10,
        [this](const nav_msgs::msg::Odometry::SharedPtr message) {
          latest_odometry_ = *message;
          latest_odometry_reception_time_ = std::chrono::steady_clock::now();
        });
      setpoint_publisher_ = create_publisher<drone_msgs::msg::TrajectorySetpoint>(
        "/drone/trajectory_setpoint", 10);
      segment_publisher_ = create_publisher<std_msgs::msg::UInt32>(
        "/drone/planned_trajectory/current_segment", 10);
      complete_publisher_ = create_publisher<std_msgs::msg::Bool>(
        "/drone/planned_trajectory/complete", 10);
      const auto period = std::chrono::duration<double>(1.0 / publish_frequency);
      last_update_time_ = std::chrono::steady_clock::now();
      update_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        [this]() {update_execution();});
    }

    RCLCPP_INFO(
      get_logger(),
      "planned trajectory node waiting for raw path; effective planning radius %.3f m; "
      "execution %s",
      effective_planning_radius, execution_enabled_ ? "enabled" : "disabled");
  }

private:
  enum class ExecutionState
  {
    WaitingForPath,
    PreparingStart,
    Executing,
    HoldingFinal,
    Failed
  };

  nav_msgs::msg::Path make_path(const std::vector<Eigen::Vector3d> & points, double yaw) const
  {
    nav_msgs::msg::Path message;
    message.header.stamp = now();
    message.header.frame_id = frame_id_;
    message.poses.reserve(points.size());
    for (const auto & point : points) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = message.header;
      pose.pose.position.x = point.x();
      pose.pose.position.y = point.y();
      pose.pose.position.z = point.z();
      pose.pose.orientation.z = std::sin(0.5 * yaw);
      pose.pose.orientation.w = std::cos(0.5 * yaw);
      message.poses.push_back(pose);
    }
    return message;
  }

  bool valid_odometry(Eigen::Vector3d & position, double & speed) const
  {
    if (!latest_odometry_) {
      return false;
    }
    const auto & pose = latest_odometry_->pose.pose;
    const auto & twist = latest_odometry_->twist.twist;
    const Eigen::Vector3d linear_velocity(
      twist.linear.x, twist.linear.y, twist.linear.z);
    position = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
    speed = linear_velocity.norm();
    return position.allFinite() && linear_velocity.allFinite() && std::isfinite(speed) &&
           std::isfinite(pose.orientation.x) && std::isfinite(pose.orientation.y) &&
           std::isfinite(pose.orientation.z) && std::isfinite(pose.orientation.w) &&
           std::isfinite(twist.angular.x) && std::isfinite(twist.angular.y) &&
           std::isfinite(twist.angular.z);
  }

  bool odometry_is_fresh(std::chrono::steady_clock::time_point steady_now) const
  {
    return latest_odometry_reception_time_ &&
           std::chrono::duration<double>(
      steady_now - *latest_odometry_reception_time_).count() <= odometry_timeout_;
  }

  void publish_failure(const std::string & reason)
  {
    execution_state_ = ExecutionState::Failed;
    std_msgs::msg::Bool success;
    success.data = false;
    success_publisher_->publish(success);
    RCLCPP_ERROR(get_logger(), "planned trajectory generation failed: %s", reason.c_str());
  }

  void publish_sample(const drone_mission::TrajectorySample & sample, bool complete_value)
  {
    drone_msgs::msg::TrajectorySetpoint setpoint;
    setpoint.header.stamp = now();
    setpoint.header.frame_id = frame_id_;
    setpoint.position.x = sample.position_world.x();
    setpoint.position.y = sample.position_world.y();
    setpoint.position.z = sample.position_world.z();
    setpoint.velocity.x = sample.velocity_world.x();
    setpoint.velocity.y = sample.velocity_world.y();
    setpoint.velocity.z = sample.velocity_world.z();
    setpoint.acceleration.x = sample.acceleration_world.x();
    setpoint.acceleration.y = sample.acceleration_world.y();
    setpoint.acceleration.z = sample.acceleration_world.z();
    setpoint.yaw = sample.yaw;
    setpoint_publisher_->publish(setpoint);

    std_msgs::msg::UInt32 segment;
    segment.data = static_cast<std::uint32_t>(std::min(
        sample.segment_index,
        static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max())));
    segment_publisher_->publish(segment);
    std_msgs::msg::Bool complete;
    complete.data = complete_value;
    complete_publisher_->publish(complete);
  }

  void update_execution()
  {
    const auto steady_now = std::chrono::steady_clock::now();
    const double dt = std::chrono::duration<double>(steady_now - last_update_time_).count();
    last_update_time_ = steady_now;
    if (!trajectory_ || execution_state_ == ExecutionState::WaitingForPath ||
      execution_state_ == ExecutionState::Failed)
    {
      return;
    }

    Eigen::Vector3d position;
    double speed = 0.0;
    const bool valid_fresh_odometry = odometry_is_fresh(steady_now) &&
      valid_odometry(position, speed);

    if (execution_state_ == ExecutionState::PreparingStart) {
      const auto sample = trajectory_->sample(0.0);
      if (valid_fresh_odometry &&
        (position - simplified_path_world_.front()).norm() < preparation_position_tolerance_ &&
        speed < preparation_speed_tolerance_)
      {
        preparation_stable_duration_ += dt;
      } else {
        preparation_stable_duration_ = 0.0;
      }
      if (preparation_stable_duration_ >= preparation_hold_duration_) {
        execution_state_ = ExecutionState::Executing;
        trajectory_elapsed_ = 0.0;
        current_segment_ = 0U;
        RCLCPP_INFO(get_logger(), "takeoff preparation complete");
        RCLCPP_INFO(get_logger(), "planned trajectory execution started");
      }
      publish_sample(sample, false);
      return;
    }

    if (execution_state_ == ExecutionState::Executing && valid_fresh_odometry) {
      trajectory_elapsed_ += dt;
    }
    const auto sample = trajectory_->sample(trajectory_elapsed_);
    if (execution_state_ == ExecutionState::Executing && sample.segment_index != current_segment_) {
      current_segment_ = sample.segment_index;
      RCLCPP_INFO(
        get_logger(), "planned trajectory switched to segment %zu", current_segment_);
    }
    if (execution_state_ == ExecutionState::Executing && sample.complete) {
      execution_state_ = ExecutionState::HoldingFinal;
      RCLCPP_INFO(get_logger(), "planned trajectory complete; holding final setpoint");
    }
    publish_sample(sample, execution_state_ == ExecutionState::HoldingFinal);
  }

  void handle_path(const nav_msgs::msg::Path & message)
  {
    if (processed_path_) {
      return;
    }
    if (message.header.frame_id != frame_id_) {
      publish_failure("planned path frame must be map");
      return;
    }

    std::vector<Eigen::Vector3d> raw_path;
    raw_path.reserve(message.poses.size());
    for (const auto & pose : message.poses) {
      raw_path.emplace_back(
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
    }

    try {
      PlannedTrajectoryResult result = builder_->build(raw_path);
      processed_path_ = true;
      if (!result.success || !result.trajectory) {
        publish_failure("all velocity scale candidates failed validation");
        return;
      }

      trajectory_ = std::move(result.trajectory);
      trajectory_total_duration_ = result.total_duration;
      simplified_path_world_ = std::move(result.simplified_path_world);
      selected_velocity_scale_ = result.selected_velocity_scale;
      const double yaw = trajectory_->sample(0.0).yaw;
      simplified_path_publisher_->publish(make_path(simplified_path_world_, yaw));

      std::vector<Eigen::Vector3d> reference_points;
      for (double time = 0.0; time < trajectory_total_duration_;
        time += reference_path_sample_period_)
      {
        reference_points.push_back(trajectory_->sample(time).position_world);
      }
      reference_points.push_back(
        trajectory_->sample(trajectory_total_duration_).position_world);
      reference_path_publisher_->publish(make_path(reference_points, yaw));

      std_msgs::msg::Bool success;
      success.data = true;
      success_publisher_->publish(success);
      std_msgs::msg::UInt32 simplified_waypoints;
      simplified_waypoints.data = static_cast<std::uint32_t>(std::min(
          simplified_path_world_.size(),
          static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max())));
      simplified_waypoints_publisher_->publish(simplified_waypoints);
      std_msgs::msg::Float64 selected_velocity_scale;
      selected_velocity_scale.data = selected_velocity_scale_;
      selected_velocity_scale_publisher_->publish(selected_velocity_scale);
      std_msgs::msg::Float64 duration;
      duration.data = trajectory_total_duration_;
      duration_publisher_->publish(duration);

      execution_state_ = execution_enabled_ ?
        ExecutionState::PreparingStart : ExecutionState::HoldingFinal;
      preparation_stable_duration_ = 0.0;
      trajectory_elapsed_ = 0.0;
      last_update_time_ = std::chrono::steady_clock::now();

      RCLCPP_INFO(
        get_logger(),
        "planned trajectory generated: raw_points=%zu simplified_points=%zu "
        "raw_length=%.6f m simplified_length=%.6f m velocity_scale=%.2f "
        "duration=%.6f s max_speed=%.6f m/s max_acceleration=%.6f m/s^2 "
        "validation_samples=%zu",
        raw_path.size(), simplified_path_world_.size(), path_length(raw_path),
        path_length(simplified_path_world_), selected_velocity_scale_,
        trajectory_total_duration_, result.max_reference_speed,
        result.max_reference_acceleration, result.validation_sample_count);
    } catch (const std::invalid_argument & error) {
      processed_path_ = true;
      publish_failure(error.what());
    }
  }

  std::string frame_id_{"map"};
  double reference_path_sample_period_{0.05};
  double odometry_timeout_{0.25};
  double preparation_position_tolerance_{0.20};
  double preparation_speed_tolerance_{0.15};
  double preparation_hold_duration_{1.0};
  double preparation_stable_duration_{0.0};
  double trajectory_elapsed_{0.0};
  double trajectory_total_duration_{0.0};
  double selected_velocity_scale_{0.0};
  std::size_t current_segment_{0U};
  bool processed_path_{false};
  bool execution_enabled_{false};
  ExecutionState execution_state_{ExecutionState::WaitingForPath};
  std::vector<Eigen::Vector3d> simplified_path_world_;
  std::optional<drone_mission::PiecewiseQuinticTrajectory> trajectory_;
  std::unique_ptr<PlannedTrajectoryBuilder> builder_;
  std::chrono::steady_clock::time_point last_update_time_;
  std::optional<nav_msgs::msg::Odometry> latest_odometry_;
  std::optional<std::chrono::steady_clock::time_point> latest_odometry_reception_time_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr planned_path_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr simplified_path_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr reference_path_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr success_publisher_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr simplified_waypoints_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr selected_velocity_scale_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr duration_publisher_;
  rclcpp::Publisher<drone_msgs::msg::TrajectorySetpoint>::SharedPtr setpoint_publisher_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr segment_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr complete_publisher_;
  rclcpp::TimerBase::SharedPtr update_timer_;
};

}  // namespace drone_planning

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<drone_planning::PlannedTrajectoryNode>());
  rclcpp::shutdown();
  return 0;
}
