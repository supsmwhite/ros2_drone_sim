#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <stdexcept>
#include <vector>

#include <Eigen/Core>

#include "drone_mission/piecewise_quintic_trajectory.hpp"
#include "drone_msgs/msg/trajectory_setpoint.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int32.hpp"

namespace drone_mission
{
namespace
{

bool finite_positive(double value)
{
  return std::isfinite(value) && value > 0.0;
}

std::vector<TrajectoryWaypoint> parse_waypoints(const std::vector<double> & values)
{
  if (values.size() < 8U || values.size() % 4U != 0U) {
    throw std::invalid_argument(
            "trajectory waypoints must contain at least two flat [x,y,z,yaw] groups");
  }
  std::vector<TrajectoryWaypoint> waypoints;
  waypoints.reserve(values.size() / 4U);
  for (std::size_t offset = 0U; offset < values.size(); offset += 4U) {
    const Eigen::Vector3d position(values[offset], values[offset + 1U], values[offset + 2U]);
    const double yaw = values[offset + 3U];
    if (!position.allFinite() || !std::isfinite(yaw)) {
      throw std::invalid_argument("all trajectory waypoint values must be finite");
    }
    waypoints.push_back({position, yaw});
  }
  return waypoints;
}

}  // namespace

class TrajectoryMissionNode : public rclcpp::Node
{
public:
  TrajectoryMissionNode()
  : Node("trajectory_mission_node")
  {
    waypoints_ = parse_waypoints(
      declare_parameter<std::vector<double>>("waypoints", std::vector<double>{}));
    const auto durations =
      declare_parameter<std::vector<double>>("segment_durations", std::vector<double>{});
    trajectory_ = std::make_unique<PiecewiseQuinticTrajectory>(waypoints_, durations);

    const double publish_frequency = declare_parameter<double>("publish_frequency", 50.0);
    odometry_timeout_ = declare_parameter<double>("odometry_timeout", 0.25);
    preparation_position_tolerance_ =
      declare_parameter<double>("preparation_position_tolerance", 0.20);
    preparation_speed_tolerance_ =
      declare_parameter<double>("preparation_speed_tolerance", 0.15);
    preparation_hold_duration_ =
      declare_parameter<double>("preparation_hold_duration", 1.0);
    reference_path_sample_period_ =
      declare_parameter<double>("reference_path_sample_period", 0.05);
    if (!finite_positive(publish_frequency) || !finite_positive(odometry_timeout_) ||
      !finite_positive(preparation_position_tolerance_) ||
      !finite_positive(preparation_speed_tolerance_) ||
      !finite_positive(preparation_hold_duration_) ||
      !finite_positive(reference_path_sample_period_))
    {
      throw std::invalid_argument("trajectory mission timing and tolerances must be positive");
    }

    odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      "/drone/odom", 10,
      [this](const nav_msgs::msg::Odometry::SharedPtr message) {
        latest_odometry_ = *message;
        latest_odometry_reception_time_ = std::chrono::steady_clock::now();
      });
    setpoint_publisher_ = create_publisher<drone_msgs::msg::TrajectorySetpoint>(
      "/drone/trajectory_setpoint", 10);
    segment_publisher_ = create_publisher<std_msgs::msg::UInt32>(
      "/drone/trajectory/current_segment", 10);
    complete_publisher_ = create_publisher<std_msgs::msg::Bool>(
      "/drone/trajectory/complete", 10);
    reference_path_publisher_ = create_publisher<nav_msgs::msg::Path>(
      "/drone/reference_path", rclcpp::QoS(1).transient_local().reliable());
    publish_reference_path();

    const auto period = std::chrono::duration<double>(1.0 / publish_frequency);
    last_update_time_ = std::chrono::steady_clock::now();
    update_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() {update();});
  }

private:
  bool valid_preparation_state(Eigen::Vector3d & position, double & speed) const
  {
    if (!latest_odometry_) {
      return false;
    }
    const auto & pose = latest_odometry_->pose.pose;
    const auto & twist = latest_odometry_->twist.twist;
    position = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
    const Eigen::Vector3d linear_velocity(
      twist.linear.x, twist.linear.y, twist.linear.z);
    speed = linear_velocity.norm();
    return position.allFinite() && linear_velocity.allFinite() && std::isfinite(speed);
  }

  bool odometry_is_fresh(std::chrono::steady_clock::time_point steady_now) const
  {
    return latest_odometry_ && latest_odometry_reception_time_ &&
           std::chrono::duration<double>(
      steady_now - *latest_odometry_reception_time_).count() <= odometry_timeout_;
  }

  void publish_reference_path()
  {
    nav_msgs::msg::Path path;
    path.header.stamp = now();
    path.header.frame_id = "map";
    for (double time = 0.0; time < trajectory_->total_duration();
      time += reference_path_sample_period_)
    {
      append_reference_pose(path, trajectory_->sample(time));
    }
    append_reference_pose(path, trajectory_->sample(trajectory_->total_duration()));
    reference_path_publisher_->publish(path);
  }

  void append_reference_pose(nav_msgs::msg::Path & path, const TrajectorySample & sample)
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = sample.position_world.x();
    pose.pose.position.y = sample.position_world.y();
    pose.pose.position.z = sample.position_world.z();
    pose.pose.orientation.z = std::sin(0.5 * sample.yaw);
    pose.pose.orientation.w = std::cos(0.5 * sample.yaw);
    path.poses.push_back(pose);
  }

  void publish_sample(const TrajectorySample & sample)
  {
    drone_msgs::msg::TrajectorySetpoint setpoint;
    setpoint.header.stamp = now();
    setpoint.header.frame_id = "map";
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
    segment.data = static_cast<std::uint32_t>(sample.segment_index);
    segment_publisher_->publish(segment);
    std_msgs::msg::Bool complete;
    complete.data = sample.complete;
    complete_publisher_->publish(complete);
  }

  void update()
  {
    const auto steady_now = std::chrono::steady_clock::now();
    const double dt = std::chrono::duration<double>(steady_now - last_update_time_).count();
    last_update_time_ = steady_now;
    const bool odometry_fresh = odometry_is_fresh(steady_now);

    if (!trajectory_started_) {
      TrajectorySample preparation_sample = trajectory_->sample(0.0);
      Eigen::Vector3d position;
      double speed = 0.0;
      if (odometry_fresh && valid_preparation_state(position, speed) &&
        (position - waypoints_.front().position_world).norm() <
        preparation_position_tolerance_ && speed < preparation_speed_tolerance_)
      {
        preparation_stable_duration_ += dt;
      } else {
        preparation_stable_duration_ = 0.0;
      }
      if (preparation_stable_duration_ >= preparation_hold_duration_) {
        trajectory_started_ = true;
        trajectory_elapsed_ = 0.0;
        current_segment_ = 0U;
        RCLCPP_INFO(get_logger(), "takeoff preparation complete");
        RCLCPP_INFO(get_logger(), "trajectory started");
      }
      publish_sample(preparation_sample);
      return;
    }

    if (!trajectory_complete_ && odometry_fresh) {
      Eigen::Vector3d position;
      double speed = 0.0;
      if (valid_preparation_state(position, speed)) {
        trajectory_elapsed_ += dt;
      }
    }
    const TrajectorySample sample = trajectory_->sample(trajectory_elapsed_);
    if (sample.segment_index != current_segment_) {
      current_segment_ = sample.segment_index;
      RCLCPP_INFO(get_logger(), "trajectory switched to segment %zu", current_segment_);
    }
    if (sample.complete && !trajectory_complete_) {
      trajectory_complete_ = true;
      RCLCPP_INFO(get_logger(), "trajectory mission complete; holding final setpoint");
    }
    publish_sample(sample);
  }

  std::vector<TrajectoryWaypoint> waypoints_;
  std::unique_ptr<PiecewiseQuinticTrajectory> trajectory_;
  double odometry_timeout_{0.25};
  double preparation_position_tolerance_{0.20};
  double preparation_speed_tolerance_{0.15};
  double preparation_hold_duration_{1.0};
  double reference_path_sample_period_{0.05};
  double preparation_stable_duration_{0.0};
  double trajectory_elapsed_{0.0};
  std::size_t current_segment_{0U};
  bool trajectory_started_{false};
  bool trajectory_complete_{false};
  std::chrono::steady_clock::time_point last_update_time_;
  std::optional<nav_msgs::msg::Odometry> latest_odometry_;
  std::optional<std::chrono::steady_clock::time_point> latest_odometry_reception_time_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  rclcpp::Publisher<drone_msgs::msg::TrajectorySetpoint>::SharedPtr setpoint_publisher_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr segment_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr complete_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr reference_path_publisher_;
  rclcpp::TimerBase::SharedPtr update_timer_;
};

}  // namespace drone_mission

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<drone_mission::TrajectoryMissionNode>());
  rclcpp::shutdown();
  return 0;
}
