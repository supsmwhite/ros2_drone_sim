#include <array>
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

#include "drone_planning/collision_checker.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

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

geometry_msgs::msg::Point point_message(const Eigen::Vector3d & point)
{
  geometry_msgs::msg::Point message;
  message.x = point.x();
  message.y = point.y();
  message.z = point.z();
  return message;
}

bool odometry_is_finite(const nav_msgs::msg::Odometry & message)
{
  const auto & position = message.pose.pose.position;
  const auto & orientation = message.pose.pose.orientation;
  const auto & linear = message.twist.twist.linear;
  const auto & angular = message.twist.twist.angular;
  const std::array<double, 13> values{
    position.x, position.y, position.z,
    orientation.x, orientation.y, orientation.z, orientation.w,
    linear.x, linear.y, linear.z,
    angular.x, angular.y, angular.z};
  for (const double value : values) {
    if (!std::isfinite(value)) {
      return false;
    }
  }
  return true;
}

}  // namespace

class StaticEnvironmentNode : public rclcpp::Node
{
public:
  StaticEnvironmentNode()
  : Node("static_environment_node")
  {
    frame_id_ = declare_parameter<std::string>("frame_id", "map");
    if (frame_id_.empty()) {
      throw std::invalid_argument("frame_id must not be empty");
    }
    const auto workspace_values =
      declare_parameter<std::vector<double>>("workspace", std::vector<double>{});
    const auto obstacle_values =
      declare_parameter<std::vector<double>>("obstacles", std::vector<double>{});
    const double safety_radius = declare_parameter<double>("safety_radius", 0.25);
    odometry_timeout_ = declare_parameter<double>("odometry_timeout", 0.25);
    if (!std::isfinite(odometry_timeout_) || odometry_timeout_ <= 0.0) {
      throw std::invalid_argument("odometry_timeout must be finite and positive");
    }

    checker_ = std::make_unique<CollisionChecker>(
      StaticEnvironment(parse_workspace(workspace_values), parse_obstacles(obstacle_values)),
      safety_radius);

    marker_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/drone/environment/markers", rclcpp::QoS(1).transient_local().reliable());
    collision_publisher_ = create_publisher<std_msgs::msg::Bool>(
      "/drone/environment/in_collision", 10);
    odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      "/drone/odom", 10,
      [this](const nav_msgs::msg::Odometry::SharedPtr message) {
        if (!odometry_is_finite(*message)) {
          latest_position_.reset();
          latest_odometry_reception_time_.reset();
          return;
        }
        latest_position_ = Eigen::Vector3d(
          message->pose.pose.position.x,
          message->pose.pose.position.y,
          message->pose.pose.position.z);
        latest_odometry_reception_time_ = std::chrono::steady_clock::now();
      });

    publish_markers();
    collision_timer_ = create_wall_timer(
      std::chrono::milliseconds(50), [this]() {publish_collision_state();});
    RCLCPP_INFO(
      get_logger(), "static environment started with %zu obstacles and safety radius %.3f m",
      checker_->environment().obstacles().size(), checker_->safety_radius());
  }

private:
  visualization_msgs::msg::Marker cube_marker(
    const AxisAlignedBox & box, const std::string & marker_namespace,
    std::int32_t id, float red, float green, float blue, float alpha) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = now();
    marker.header.frame_id = frame_id_;
    marker.ns = marker_namespace;
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    const Eigen::Vector3d center = 0.5 * (box.min_corner + box.max_corner);
    const Eigen::Vector3d size = box.max_corner - box.min_corner;
    marker.pose.position = point_message(center);
    marker.pose.orientation.w = 1.0;
    marker.scale.x = size.x();
    marker.scale.y = size.y();
    marker.scale.z = size.z();
    marker.color.r = red;
    marker.color.g = green;
    marker.color.b = blue;
    marker.color.a = alpha;
    return marker;
  }

  visualization_msgs::msg::Marker workspace_marker() const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = now();
    marker.header.frame_id = frame_id_;
    marker.ns = "workspace";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.025;
    marker.color.r = 0.20F;
    marker.color.g = 0.85F;
    marker.color.b = 0.35F;
    marker.color.a = 0.90F;

    const auto & box = checker_->environment().workspace();
    const std::array<Eigen::Vector3d, 8> corners{
      Eigen::Vector3d(box.min_corner.x(), box.min_corner.y(), box.min_corner.z()),
      Eigen::Vector3d(box.max_corner.x(), box.min_corner.y(), box.min_corner.z()),
      Eigen::Vector3d(box.max_corner.x(), box.max_corner.y(), box.min_corner.z()),
      Eigen::Vector3d(box.min_corner.x(), box.max_corner.y(), box.min_corner.z()),
      Eigen::Vector3d(box.min_corner.x(), box.min_corner.y(), box.max_corner.z()),
      Eigen::Vector3d(box.max_corner.x(), box.min_corner.y(), box.max_corner.z()),
      Eigen::Vector3d(box.max_corner.x(), box.max_corner.y(), box.max_corner.z()),
      Eigen::Vector3d(box.min_corner.x(), box.max_corner.y(), box.max_corner.z())};
    const std::array<std::array<std::size_t, 2>, 12> edges{{
      {{0, 1}}, {{1, 2}}, {{2, 3}}, {{3, 0}},
      {{4, 5}}, {{5, 6}}, {{6, 7}}, {{7, 4}},
      {{0, 4}}, {{1, 5}}, {{2, 6}}, {{3, 7}}}};
    for (const auto & edge : edges) {
      marker.points.push_back(point_message(corners[edge[0]]));
      marker.points.push_back(point_message(corners[edge[1]]));
    }
    return marker;
  }

  void publish_markers()
  {
    visualization_msgs::msg::MarkerArray markers;
    markers.markers.push_back(workspace_marker());
    const auto & obstacles = checker_->environment().obstacles();
    const auto & inflated = checker_->inflated_obstacles();
    for (std::size_t index = 0U; index < obstacles.size(); ++index) {
      markers.markers.push_back(cube_marker(
        obstacles[index], "obstacles", static_cast<std::int32_t>(index),
        0.85F, 0.18F, 0.12F, 0.85F));
      markers.markers.push_back(cube_marker(
        inflated[index], "inflated_obstacles", static_cast<std::int32_t>(index),
        1.00F, 0.65F, 0.10F, 0.18F));
    }
    marker_publisher_->publish(markers);
  }

  void publish_collision_state()
  {
    if (!latest_position_ || !latest_odometry_reception_time_) {
      return;
    }
    const auto steady_now = std::chrono::steady_clock::now();
    if (std::chrono::duration<double>(
        steady_now - *latest_odometry_reception_time_).count() > odometry_timeout_)
    {
      return;
    }

    const bool in_collision = checker_->point_in_collision(*latest_position_);
    std_msgs::msg::Bool message;
    message.data = in_collision;
    collision_publisher_->publish(message);
    if (last_collision_state_ && *last_collision_state_ != in_collision) {
      if (in_collision) {
        RCLCPP_WARN(get_logger(), "drone entered geometric collision state");
      } else {
        RCLCPP_INFO(get_logger(), "drone returned to geometrically safe state");
      }
    }
    last_collision_state_ = in_collision;
  }

  std::string frame_id_{"map"};
  double odometry_timeout_{0.25};
  std::unique_ptr<CollisionChecker> checker_;
  std::optional<Eigen::Vector3d> latest_position_;
  std::optional<std::chrono::steady_clock::time_point> latest_odometry_reception_time_;
  std::optional<bool> last_collision_state_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr collision_publisher_;
  rclcpp::TimerBase::SharedPtr collision_timer_;
};

}  // namespace drone_planning

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<drone_planning::StaticEnvironmentNode>());
  rclcpp::shutdown();
  return 0;
}
