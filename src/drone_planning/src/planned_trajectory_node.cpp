#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "drone_planning/planned_trajectory_builder.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/u_int32.hpp"

namespace drone_planning
{
namespace
{

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
    if (!std::isfinite(reference_path_sample_period_) || reference_path_sample_period_ <= 0.0) {
      throw std::invalid_argument("reference_path_sample_period must be finite and positive");
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

    RCLCPP_INFO(
      get_logger(),
      "planned trajectory node waiting for raw path; effective planning radius %.3f m",
      effective_planning_radius);
  }

private:
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

  void publish_failure(const std::string & reason)
  {
    std_msgs::msg::Bool success;
    success.data = false;
    success_publisher_->publish(success);
    RCLCPP_ERROR(get_logger(), "planned trajectory generation failed: %s", reason.c_str());
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
      const PlannedTrajectoryResult result = builder_->build(raw_path);
      processed_path_ = true;
      if (!result.success || !result.trajectory) {
        publish_failure("all velocity scale candidates failed validation");
        return;
      }

      const double yaw = result.trajectory->sample(0.0).yaw;
      simplified_path_publisher_->publish(make_path(result.simplified_path_world, yaw));

      std::vector<Eigen::Vector3d> reference_points;
      for (double time = 0.0; time < result.total_duration;
        time += reference_path_sample_period_)
      {
        reference_points.push_back(result.trajectory->sample(time).position_world);
      }
      reference_points.push_back(
        result.trajectory->sample(result.total_duration).position_world);
      reference_path_publisher_->publish(make_path(reference_points, yaw));

      std_msgs::msg::Bool success;
      success.data = true;
      success_publisher_->publish(success);
      std_msgs::msg::UInt32 simplified_waypoints;
      simplified_waypoints.data = static_cast<std::uint32_t>(std::min(
        result.simplified_path_world.size(),
        static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max())));
      simplified_waypoints_publisher_->publish(simplified_waypoints);
      std_msgs::msg::Float64 selected_velocity_scale;
      selected_velocity_scale.data = result.selected_velocity_scale;
      selected_velocity_scale_publisher_->publish(selected_velocity_scale);
      std_msgs::msg::Float64 duration;
      duration.data = result.total_duration;
      duration_publisher_->publish(duration);

      RCLCPP_INFO(
        get_logger(),
        "planned trajectory generated: raw_points=%zu simplified_points=%zu "
        "raw_length=%.6f m simplified_length=%.6f m velocity_scale=%.2f "
        "duration=%.6f s max_speed=%.6f m/s max_acceleration=%.6f m/s^2 "
        "validation_samples=%zu",
        raw_path.size(), result.simplified_path_world.size(), path_length(raw_path),
        path_length(result.simplified_path_world), result.selected_velocity_scale,
        result.total_duration, result.max_reference_speed,
        result.max_reference_acceleration, result.validation_sample_count);
    } catch (const std::invalid_argument & error) {
      publish_failure(error.what());
    }
  }

  std::string frame_id_{"map"};
  double reference_path_sample_period_{0.05};
  bool processed_path_{false};
  std::unique_ptr<PlannedTrajectoryBuilder> builder_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr planned_path_subscription_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr simplified_path_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr reference_path_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr success_publisher_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr simplified_waypoints_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr selected_velocity_scale_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr duration_publisher_;
};

}  // namespace drone_planning

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<drone_planning::PlannedTrajectoryNode>());
  rclcpp::shutdown();
  return 0;
}
