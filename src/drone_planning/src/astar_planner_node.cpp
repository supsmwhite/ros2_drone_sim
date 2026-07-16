#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "drone_planning/astar_planner.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
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

Eigen::Vector3d parse_point(const std::vector<double> & values, const char * name)
{
  if (values.size() != 3U) {
    throw std::invalid_argument(std::string(name) + " must contain x, y and z");
  }
  const Eigen::Vector3d point(values[0], values[1], values[2]);
  if (!point.allFinite()) {
    throw std::invalid_argument(std::string(name) + " must be finite");
  }
  return point;
}

const char * status_name(PlanningStatus status)
{
  switch (status) {
    case PlanningStatus::kSuccess:
      return "success";
    case PlanningStatus::kInvalidStart:
      return "invalid_start";
    case PlanningStatus::kInvalidGoal:
      return "invalid_goal";
    case PlanningStatus::kNoPath:
      return "no_path";
  }
  return "unknown";
}

}  // namespace

class AStarPlannerNode : public rclcpp::Node
{
public:
  AStarPlannerNode()
  : Node("astar_planner_node")
  {
    const std::string frame_id = declare_parameter<std::string>("frame_id", "map");
    if (frame_id.empty()) {
      throw std::invalid_argument("frame_id must not be empty");
    }
    const auto workspace_values =
      declare_parameter<std::vector<double>>("workspace", std::vector<double>{});
    const auto obstacle_values =
      declare_parameter<std::vector<double>>("obstacles", std::vector<double>{});
    const double safety_radius = declare_parameter<double>("safety_radius", 0.25);
    const Eigen::Vector3d start = parse_point(
      declare_parameter<std::vector<double>>("start", std::vector<double>{}), "start");
    const Eigen::Vector3d goal = parse_point(
      declare_parameter<std::vector<double>>("goal", std::vector<double>{}), "goal");
    const double resolution = declare_parameter<double>("resolution", 0.25);
    const std::int64_t max_grid_nodes_parameter =
      declare_parameter<std::int64_t>("max_grid_nodes", 200000);
    if (max_grid_nodes_parameter <= 0) {
      throw std::invalid_argument("max_grid_nodes must be positive");
    }
    const std::size_t max_grid_nodes = static_cast<std::size_t>(max_grid_nodes_parameter);

    const auto result_qos = rclcpp::QoS(1).transient_local().reliable();
    path_publisher_ = create_publisher<nav_msgs::msg::Path>(
      "/drone/planned_path", result_qos);
    success_publisher_ = create_publisher<std_msgs::msg::Bool>(
      "/drone/planning/success", result_qos);
    expanded_nodes_publisher_ = create_publisher<std_msgs::msg::UInt32>(
      "/drone/planning/expanded_nodes", result_qos);

    AStarPlanner planner(
      CollisionChecker(
        StaticEnvironment(parse_workspace(workspace_values), parse_obstacles(obstacle_values)),
        safety_radius),
      resolution, max_grid_nodes);
    const auto planning_start = std::chrono::steady_clock::now();
    const AStarResult result = planner.plan(start, goal);
    const double planning_time_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - planning_start).count();

    nav_msgs::msg::Path path_message;
    path_message.header.stamp = now();
    path_message.header.frame_id = frame_id;
    if (result.success()) {
      path_message.poses.reserve(result.path_world.size());
      for (const auto & point : result.path_world) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path_message.header;
        pose.pose.position.x = point.x();
        pose.pose.position.y = point.y();
        pose.pose.position.z = point.z();
        pose.pose.orientation.w = 1.0;
        path_message.poses.push_back(pose);
      }
    }
    std_msgs::msg::Bool success_message;
    success_message.data = result.success();
    std_msgs::msg::UInt32 expanded_nodes_message;
    expanded_nodes_message.data = static_cast<std::uint32_t>(std::min(
      result.expanded_nodes,
      static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max())));

    path_publisher_->publish(path_message);
    success_publisher_->publish(success_message);
    expanded_nodes_publisher_->publish(expanded_nodes_message);

    if (result.success()) {
      RCLCPP_INFO(
        get_logger(),
        "planning succeeded: time=%.3f ms path_nodes=%zu path_length=%.6f m "
        "expanded_nodes=%zu",
        planning_time_ms, result.path_world.size(), result.path_length, result.expanded_nodes);
    } else {
      RCLCPP_ERROR(
        get_logger(),
        "planning failed: status=%s time=%.3f ms path_nodes=0 path_length=0.000000 m "
        "expanded_nodes=%zu",
        status_name(result.status), planning_time_ms, result.expanded_nodes);
    }
  }

private:
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr success_publisher_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr expanded_nodes_publisher_;
};

}  // namespace drone_planning

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<drone_planning::AStarPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
