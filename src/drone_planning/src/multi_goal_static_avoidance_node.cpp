#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <future>
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
#include "drone_msgs/srv/execute_goal_sequence.hpp"
#include "drone_planning/astar_planner.hpp"
#include "drone_planning/mission_failure_safety.hpp"
#include "drone_planning/multi_goal_visualization.hpp"
#include "drone_planning/planned_trajectory_builder.hpp"
#include "drone_planning/yaw_reference_generator.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int32.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace drone_planning
{
namespace
{

bool finite_positive(double value)
{
  return std::isfinite(value) && value > 0.0;
}

double path_length(const std::vector<Eigen::Vector3d> & points)
{
  double length = 0.0;
  for (std::size_t index = 1U; index < points.size(); ++index) {
    length += (points[index] - points[index - 1U]).norm();
  }
  return length;
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

struct SegmentPlan
{
  AStarResult astar_result;
  PlannedTrajectoryResult trajectory_result;
  std::string error;
};

struct PreflightResult
{
  bool success{false};
  std::string message;
};

std::optional<double> quaternion_yaw(const geometry_msgs::msg::Quaternion & orientation)
{
  const double norm_squared = orientation.x * orientation.x +
    orientation.y * orientation.y + orientation.z * orientation.z +
    orientation.w * orientation.w;
  if (!std::isfinite(norm_squared) || norm_squared < 1.0e-12) {
    return std::nullopt;
  }
  const double inverse_norm = 1.0 / std::sqrt(norm_squared);
  const double x = orientation.x * inverse_norm;
  const double y = orientation.y * inverse_norm;
  const double z = orientation.z * inverse_norm;
  const double w = orientation.w * inverse_norm;
  const double yaw = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
  return std::isfinite(yaw) ? std::optional<double>(yaw) : std::nullopt;
}

}  // namespace

class MultiGoalStaticAvoidanceNode : public rclcpp::Node
{
public:
  MultiGoalStaticAvoidanceNode()
  : Node("multi_goal_static_avoidance_node")
  {
    goal_source_ = declare_parameter<std::string>("goal_source", "parameters");
    if (goal_source_ != "parameters" && goal_source_ != "interactive") {
      throw std::invalid_argument("goal_source must be parameters or interactive");
    }
    interactive_mode_ = goal_source_ == "interactive";
    const auto max_goals_parameter = declare_parameter<std::int64_t>("max_goals", 8);
    interactive_mission_odom_wait_timeout_ =
      declare_parameter<double>("interactive_mission_odom_wait_timeout", 3.0);
    if (max_goals_parameter <= 0 || !finite_positive(interactive_mission_odom_wait_timeout_)) {
      throw std::invalid_argument("interactive mission limits are invalid");
    }
    max_goals_ = static_cast<std::size_t>(max_goals_parameter);

    frame_id_ = declare_parameter<std::string>("frame_id", "map");
    if (frame_id_ != "map") {
      throw std::invalid_argument("multi-goal static avoidance frame_id must be map");
    }

    const auto workspace_values =
      declare_parameter<std::vector<double>>("workspace", std::vector<double>{});
    const auto obstacle_values =
      declare_parameter<std::vector<double>>("obstacles", std::vector<double>{});
    const double safety_radius = declare_parameter<double>("safety_radius", 0.25);
    const double planning_margin = declare_parameter<double>("planning_margin", 0.10);
    if (!std::isfinite(safety_radius) || safety_radius < 0.0 ||
      !std::isfinite(planning_margin) || planning_margin < 0.0)
    {
      throw std::invalid_argument(
              "safety_radius and planning_margin must be finite and non-negative");
    }
    effective_planning_radius_ = safety_radius + planning_margin;
    if (!std::isfinite(effective_planning_radius_)) {
      throw std::invalid_argument("effective planning radius must be finite");
    }

    takeoff_height_ = declare_parameter<double>("takeoff_height", 1.5);
    minimum_navigation_altitude_ =
      declare_parameter<double>("minimum_navigation_altitude", 0.50);
    const auto goal_values =
      declare_parameter<std::vector<double>>("goals", std::vector<double>{});
    if (interactive_mode_) {
      if (!goal_values.empty()) {
        throw std::invalid_argument("interactive goal_source must not preload parameter goals");
      }
    } else {
      goals_ = parse_goals(goal_values);
    }
    const double publish_frequency = declare_parameter<double>("publish_frequency", 50.0);
    visualization_update_frequency_ =
      declare_parameter<double>("visualization_update_frequency", 5.0);
    odometry_timeout_ = declare_parameter<double>("odometry_timeout", 0.25);
    takeoff_position_tolerance_ =
      declare_parameter<double>("takeoff_position_tolerance", 0.20);
    takeoff_speed_tolerance_ =
      declare_parameter<double>("takeoff_speed_tolerance", 0.15);
    takeoff_hold_duration_ = declare_parameter<double>("takeoff_hold_duration", 1.0);
    goal_position_tolerance_ =
      declare_parameter<double>("goal_position_tolerance", 0.20);
    goal_speed_tolerance_ = declare_parameter<double>("goal_speed_tolerance", 0.15);
    goal_hold_duration_ = declare_parameter<double>("goal_hold_duration", 1.0);
    resolution_ = declare_parameter<double>("resolution", 0.25);
    const auto max_grid_nodes_parameter =
      declare_parameter<std::int64_t>("max_grid_nodes", 200000);
    if (max_grid_nodes_parameter <= 0) {
      throw std::invalid_argument("max_grid_nodes must be positive");
    }
    max_grid_nodes_ = static_cast<std::size_t>(max_grid_nodes_parameter);

    trajectory_parameters_.nominal_speed =
      declare_parameter<double>("nominal_speed", 0.35);
    trajectory_parameters_.min_segment_duration =
      declare_parameter<double>("min_segment_duration", 2.0);
    trajectory_parameters_.validation_sample_period =
      declare_parameter<double>("validation_sample_period", 0.02);
    reference_path_sample_period_ =
      declare_parameter<double>("reference_path_sample_period", 0.05);
    trajectory_parameters_.max_reference_speed =
      declare_parameter<double>("max_reference_speed", 0.70);
    trajectory_parameters_.max_reference_acceleration =
      declare_parameter<double>("max_reference_acceleration", 0.35);
    trajectory_parameters_.velocity_scale_candidates =
      declare_parameter<std::vector<double>>(
      "velocity_scale_candidates", {1.0, 0.75, 0.5, 0.25, 0.0});
    trajectory_parameters_.duration_scale_candidates =
      declare_parameter<std::vector<double>>(
      "duration_scale_candidates",
      {1.0, 1.05, 1.10, 1.15, 1.20, 1.25, 1.5, 2.0, 3.0, 4.0});
    const auto max_refinement_iterations =
      declare_parameter<std::int64_t>("max_refinement_iterations", 8);
    const auto max_insertions_per_refinement =
      declare_parameter<std::int64_t>("max_insertions_per_refinement", 3);
    if (max_refinement_iterations < 0 || max_insertions_per_refinement <= 0) {
      throw std::invalid_argument("trajectory refinement limits are invalid");
    }
    trajectory_parameters_.max_refinement_iterations =
      static_cast<std::size_t>(max_refinement_iterations);
    trajectory_parameters_.max_insertions_per_refinement =
      static_cast<std::size_t>(max_insertions_per_refinement);
    trajectory_parameters_.fixed_yaw = declare_parameter<double>("fixed_yaw", 0.0);

    YawReferenceParameters yaw_parameters;
    yaw_parameters.mode = parse_yaw_mode(declare_parameter<std::string>("yaw_mode", "fixed"));
    yaw_parameters.fixed_yaw = trajectory_parameters_.fixed_yaw;
    yaw_parameters.tangent_speed_threshold =
      declare_parameter<double>("tangent_speed_threshold", 0.10);
    yaw_parameters.terminal_blend_distance =
      declare_parameter<double>("terminal_blend_distance", 0.80);
    yaw_parameters.filter_time_constant =
      declare_parameter<double>("yaw_filter_time_constant", 0.30);
    yaw_parameters.max_yaw_rate = declare_parameter<double>("max_yaw_rate", 0.80);
    yaw_generator_ = std::make_unique<YawReferenceGenerator>(yaw_parameters);

    if (!finite_positive(takeoff_height_) ||
      !std::isfinite(minimum_navigation_altitude_) || minimum_navigation_altitude_ < 0.0 ||
      takeoff_height_ <= minimum_navigation_altitude_ ||
      !finite_positive(publish_frequency) || !finite_positive(visualization_update_frequency_) ||
      !finite_positive(odometry_timeout_) ||
      !finite_positive(takeoff_position_tolerance_) ||
      !finite_positive(takeoff_speed_tolerance_) || !finite_positive(takeoff_hold_duration_) ||
      !finite_positive(goal_position_tolerance_) || !finite_positive(goal_speed_tolerance_) ||
      !finite_positive(goal_hold_duration_) || !finite_positive(resolution_) ||
      !finite_positive(reference_path_sample_period_))
    {
      throw std::invalid_argument("multi-goal mission scalar parameters are invalid");
    }

    const AxisAlignedBox original_workspace = parse_workspace(workspace_values);
    const auto obstacles = parse_obstacles(obstacle_values);
    takeoff_collision_checker_ = std::make_unique<CollisionChecker>(
      StaticEnvironment(original_workspace, obstacles), effective_planning_radius_);

    AxisAlignedBox navigation_workspace = original_workspace;
    navigation_workspace.min_corner.z() =
      minimum_navigation_altitude_ - effective_planning_radius_;
    navigation_collision_checker_ = std::make_unique<CollisionChecker>(
      StaticEnvironment(navigation_workspace, obstacles), effective_planning_radius_);
    if (!std::isfinite(navigation_collision_checker_->safe_workspace().min_corner.z()) ||
      std::abs(
        navigation_collision_checker_->safe_workspace().min_corner.z() -
        minimum_navigation_altitude_) > 1.0e-12)
    {
      throw std::logic_error("navigation workspace did not produce the requested safe floor");
    }
    for (const auto & goal : goals_) {
      if (goal.position.z() < minimum_navigation_altitude_) {
        throw std::invalid_argument("all goals must satisfy the minimum navigation altitude");
      }
      if (navigation_collision_checker_->point_in_collision(goal.position)) {
        throw std::invalid_argument("a configured multi-goal position is not planning-safe");
      }
    }

    const auto path_qos = rclcpp::QoS(1).transient_local().reliable();
    planned_path_publisher_ = create_publisher<nav_msgs::msg::Path>(
      "/drone/planned_path", path_qos);
    simplified_path_publisher_ = create_publisher<nav_msgs::msg::Path>(
      "/drone/simplified_path", path_qos);
    reference_path_publisher_ = create_publisher<nav_msgs::msg::Path>(
      "/drone/reference_path", path_qos);
    setpoint_publisher_ = create_publisher<drone_msgs::msg::TrajectorySetpoint>(
      "/drone/trajectory_setpoint", 10);
    current_goal_publisher_ = create_publisher<std_msgs::msg::UInt32>(
      "/drone/multi_goal/current_goal_index", 10);
    current_segment_publisher_ = create_publisher<std_msgs::msg::UInt32>(
      "/drone/multi_goal/current_segment", 10);
    complete_publisher_ = create_publisher<std_msgs::msg::Bool>(
      "/drone/multi_goal/complete", 10);
    success_publisher_ = create_publisher<std_msgs::msg::Bool>(
      "/drone/multi_goal/success", 10);
    visited_goals_publisher_ = create_publisher<std_msgs::msg::UInt32>(
      "/drone/multi_goal/visited_goals", 10);
    const auto visualization_qos = rclcpp::QoS(1).transient_local().reliable();
    goal_markers_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/drone/multi_goal/goal_markers", visualization_qos);
    current_goal_pose_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/drone/multi_goal/current_goal_pose", visualization_qos);
    interactive_active_publisher_ = create_publisher<std_msgs::msg::Bool>(
      "/drone/interactive_mission/active", visualization_qos);
    interactive_status_publisher_ = create_publisher<std_msgs::msg::String>(
      "/drone/interactive_mission/status", visualization_qos);
    interactive_revision_publisher_ = create_publisher<std_msgs::msg::UInt64>(
      "/drone/interactive_mission/draft_revision", visualization_qos);
    state_ = interactive_mode_ ? State::WaitingForMission : State::WaitingForOdometry;
    if (interactive_mode_) {
      execute_service_ = create_service<drone_msgs::srv::ExecuteGoalSequence>(
        "/drone/interactive_goals/execute",
        [this](
          const drone_msgs::srv::ExecuteGoalSequence::Request::SharedPtr request,
          drone_msgs::srv::ExecuteGoalSequence::Response::SharedPtr response)
        {
          handle_execute_request(*request, *response);
        });
    }
    publish_mission_visualization(true);

    odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      "/drone/odom", 10,
      [this](const nav_msgs::msg::Odometry::SharedPtr message) {
        latest_odometry_ = *message;
        latest_odometry_reception_time_ = std::chrono::steady_clock::now();
      });

    const auto period = std::chrono::duration<double>(1.0 / publish_frequency);
    last_update_time_ = std::chrono::steady_clock::now();
    update_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() {update();});

    RCLCPP_INFO(
      get_logger(),
      "multi-goal static avoidance started: goal_source=%s goals=%zu takeoff=%.2f m "
      "navigation_floor=%.2f m effective_radius=%.2f m",
      goal_source_.c_str(), goals_.size(), takeoff_height_, minimum_navigation_altitude_,
      effective_planning_radius_);
  }

private:
  enum class State
  {
    WaitingForMission,
    WaitingForPreflightOdometry,
    PreflightValidation,
    WaitingForOdometry,
    CheckingTakeoff,
    TakingOff,
    PlanningSegment,
    ExecutingSegment,
    HoldingGoal,
    MissionComplete,
    Failed
  };

  bool mission_active() const
  {
    return state_ == State::WaitingForPreflightOdometry ||
           state_ == State::PreflightValidation || state_ == State::CheckingTakeoff ||
           state_ == State::TakingOff || state_ == State::PlanningSegment ||
           state_ == State::ExecutingSegment || state_ == State::HoldingGoal;
  }

  std::string mission_status_text() const
  {
    switch (state_) {
      case State::WaitingForMission:
        return "WAITING FOR VALIDATED MISSION";
      case State::WaitingForPreflightOdometry:
        return "WAITING FOR FRESH ODOMETRY";
      case State::PreflightValidation:
        return "PREFLIGHT VALIDATING";
      case State::WaitingForOdometry:
        return "WAITING FOR ODOMETRY";
      case State::CheckingTakeoff:
        return "CHECKING TAKEOFF";
      case State::TakingOff:
        return "TAKING OFF";
      case State::PlanningSegment:
        return "PLANNING P" + std::to_string(current_goal_index_ + 1U) + " / " +
               std::to_string(goals_.size());
      case State::ExecutingSegment:
        return "EXECUTING P" + std::to_string(current_goal_index_ + 1U) + " / " +
               std::to_string(goals_.size());
      case State::HoldingGoal:
        return "HOLDING P" + std::to_string(current_goal_index_ + 1U) + " / " +
               std::to_string(goals_.size());
      case State::MissionComplete:
        return "MISSION COMPLETE";
      case State::Failed:
        return "MISSION FAILED: " + failure_reason_;
    }
    return "MISSION STATE UNKNOWN";
  }

  std::optional<std::string> validate_request(
    const drone_msgs::srv::ExecuteGoalSequence::Request & request,
    std::vector<MissionGoal> & validated_goals) const
  {
    if (request.goals.header.frame_id != frame_id_) {
      return "REJECTED: goals frame_id must be map";
    }
    if (request.goals.poses.empty()) {
      return "REJECTED: goal list is empty";
    }
    if (request.goals.poses.size() > max_goals_) {
      return "REJECTED: goal count exceeds max_goals";
    }
    validated_goals.clear();
    validated_goals.reserve(request.goals.poses.size());
    for (std::size_t index = 0U; index < request.goals.poses.size(); ++index) {
      const auto & pose = request.goals.poses[index];
      const Eigen::Vector3d position(pose.position.x, pose.position.y, pose.position.z);
      if (!position.allFinite() || !std::isfinite(pose.orientation.x) ||
        !std::isfinite(pose.orientation.y) || !std::isfinite(pose.orientation.z) ||
        !std::isfinite(pose.orientation.w))
      {
        return "REJECTED: P" + std::to_string(index + 1U) + " contains non-finite values";
      }
      const auto yaw = quaternion_yaw(pose.orientation);
      if (!yaw) {
        return "REJECTED: P" + std::to_string(index + 1U) + " has invalid orientation";
      }
      if (position.z() < minimum_navigation_altitude_) {
        return "REJECTED: P" + std::to_string(index + 1U) +
               " is below the navigation floor";
      }
      if (navigation_collision_checker_->point_in_collision(position)) {
        return "REJECTED: P" + std::to_string(index + 1U) +
               " is outside the safe workspace or inside an inflated obstacle";
      }
      validated_goals.push_back({position, *yaw});
    }
    return std::nullopt;
  }

  void handle_execute_request(
    const drone_msgs::srv::ExecuteGoalSequence::Request & request,
    drone_msgs::srv::ExecuteGoalSequence::Response & response)
  {
    if (!interactive_mode_) {
      response.message = "REJECTED: executor is not in interactive mode";
      return;
    }
    if (mission_active()) {
      response.message = "REJECTED: another mission is active";
      return;
    }
    if (mission_ever_accepted_) {
      response.message = "REJECTED: restart the interactive navigation launch for a new mission";
      return;
    }
    std::vector<MissionGoal> snapshot;
    if (const auto rejection = validate_request(request, snapshot)) {
      response.message = *rejection;
      return;
    }

    goals_ = std::move(snapshot);
    accepted_draft_revision_ = request.draft_revision;
    mission_ever_accepted_ = true;
    current_goal_index_ = 0U;
    current_segment_ = 0U;
    visited_goals_ = 0U;
    failure_reason_.clear();
    mission_request_time_ = std::chrono::steady_clock::now();
    state_ = State::WaitingForPreflightOdometry;
    response.accepted = true;
    response.message = "ACCEPTED: mission snapshot received; preflight validation started";
    publish_mission_visualization(true);
    RCLCPP_INFO(
      get_logger(), "accepted interactive mission revision=%lu goals=%zu",
      accepted_draft_revision_, goals_.size());
  }

  void start_preflight(const Eigen::Vector3d & actual_position)
  {
    initial_position_ = actual_position;
    preflight_requires_takeoff_ = actual_position.z() < minimum_navigation_altitude_;
    Eigen::Vector3d preflight_start = actual_position;
    if (preflight_requires_takeoff_) {
      takeoff_anchor_ = Eigen::Vector3d(
        actual_position.x(), actual_position.y(), takeoff_height_);
      if (takeoff_collision_checker_->segment_in_collision(actual_position, takeoff_anchor_)) {
        fail("preflight vertical takeoff segment is unsafe");
        return;
      }
      if (navigation_collision_checker_->point_in_collision(takeoff_anchor_)) {
        fail("preflight takeoff anchor is not navigation-safe");
        return;
      }
      preflight_start = takeoff_anchor_;
    } else {
      takeoff_anchor_ = actual_position;
      hold_position_ = actual_position;
      safe_hold_position_ = actual_position;
      flight_started_ = true;
    }

    const auto goals_snapshot = goals_;
    const CollisionChecker checker = *navigation_collision_checker_;
    const double resolution = resolution_;
    const std::size_t max_grid_nodes = max_grid_nodes_;
    const PlannedTrajectoryParameters parameters = trajectory_parameters_;
    preflight_future_.emplace(std::async(
      std::launch::async,
      [checker, resolution, max_grid_nodes, parameters, preflight_start,
      goals_snapshot]() mutable {
        PreflightResult result;
        Eigen::Vector3d segment_start = preflight_start;
        try {
          for (std::size_t index = 0U; index < goals_snapshot.size(); ++index) {
            const auto astar = AStarPlanner(checker, resolution, max_grid_nodes).plan(
              segment_start, goals_snapshot[index].position);
            const std::string segment = index == 0U ? "START -> P1" :
              "P" + std::to_string(index) + " -> P" + std::to_string(index + 1U);
            if (!astar.success()) {
              result.message = "preflight segment " + segment + " A* failed";
              return result;
            }
            const auto trajectory = PlannedTrajectoryBuilder(checker, parameters).build(
              astar.path_world);
            if (!trajectory.success || !trajectory.trajectory) {
              result.message = "preflight segment " + segment +
                " has no valid continuous trajectory";
              return result;
            }
            segment_start = goals_snapshot[index].position;
          }
          result.success = true;
          result.message = "preflight validation passed";
        } catch (const std::exception & error) {
          result.message = std::string("preflight exception: ") + error.what();
        }
        return result;
      }));
    state_ = State::PreflightValidation;
    RCLCPP_INFO(
      get_logger(), "preflight started from [%.3f, %.3f, %.3f] for %zu goals",
      preflight_start.x(), preflight_start.y(), preflight_start.z(), goals_.size());
  }

  bool valid_odometry(Eigen::Vector3d & position, double & speed) const
  {
    if (!latest_odometry_) {
      return false;
    }
    const auto & pose = latest_odometry_->pose.pose;
    const auto & twist = latest_odometry_->twist.twist;
    position = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
    const Eigen::Vector3d linear_velocity(twist.linear.x, twist.linear.y, twist.linear.z);
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

  nav_msgs::msg::Path make_path(const std::vector<Eigen::Vector3d> & points) const
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
      pose.pose.orientation.w = 1.0;
      message.poses.push_back(pose);
    }
    return message;
  }

  void publish_setpoint(
    const Eigen::Vector3d & position, const Eigen::Vector3d & velocity,
    const Eigen::Vector3d & acceleration, double terminal_yaw)
  {
    drone_msgs::msg::TrajectorySetpoint setpoint;
    setpoint.header.stamp = now();
    setpoint.header.frame_id = frame_id_;
    setpoint.position.x = position.x();
    setpoint.position.y = position.y();
    setpoint.position.z = position.z();
    setpoint.velocity.x = velocity.x();
    setpoint.velocity.y = velocity.y();
    setpoint.velocity.z = velocity.z();
    setpoint.acceleration.x = acceleration.x();
    setpoint.acceleration.y = acceleration.y();
    setpoint.acceleration.z = acceleration.z();
    const Eigen::Vector3d goal_position = goals_.empty() ? position :
      goals_[std::min(current_goal_index_, goals_.size() - 1U)].position;
    setpoint.yaw = yaw_generator_->update(
      position, velocity, goal_position, terminal_yaw, current_update_dt_);
    setpoint_publisher_->publish(setpoint);
  }

  void publish_hold(const Eigen::Vector3d & position)
  {
    const double terminal_yaw = goals_.empty() ? trajectory_parameters_.fixed_yaw :
      goals_[std::min(current_goal_index_, goals_.size() - 1U)].yaw;
    publish_setpoint(
      position, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), terminal_yaw);
  }

  void publish_status()
  {
    std_msgs::msg::UInt32 current_goal;
    current_goal.data = static_cast<std::uint32_t>(std::min(
        current_goal_index_,
        static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max())));
    current_goal_publisher_->publish(current_goal);
    std_msgs::msg::UInt32 current_segment;
    current_segment.data = static_cast<std::uint32_t>(std::min(
        current_segment_,
        static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max())));
    current_segment_publisher_->publish(current_segment);
    std_msgs::msg::Bool complete;
    complete.data = state_ == State::MissionComplete;
    complete_publisher_->publish(complete);
    std_msgs::msg::Bool success;
    success.data = state_ != State::Failed;
    success_publisher_->publish(success);
    std_msgs::msg::UInt32 visited;
    visited.data = static_cast<std::uint32_t>(std::min(
        visited_goals_,
        static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max())));
    visited_goals_publisher_->publish(visited);
    std_msgs::msg::Bool active;
    active.data = mission_active();
    interactive_active_publisher_->publish(active);
    std_msgs::msg::String interactive_status;
    interactive_status.data = mission_status_text();
    interactive_status_publisher_->publish(interactive_status);
    std_msgs::msg::UInt64 revision;
    revision.data = accepted_draft_revision_;
    interactive_revision_publisher_->publish(revision);
    publish_mission_visualization();
  }

  MissionVisualizationState visualization_state() const
  {
    if (state_ == State::WaitingForMission || state_ == State::WaitingForPreflightOdometry) {
      return MissionVisualizationState::Waiting;
    }
    if (state_ == State::PreflightValidation) {
      return MissionVisualizationState::Preflight;
    }
    if (state_ == State::MissionComplete) {
      return MissionVisualizationState::Complete;
    }
    if (state_ == State::Failed) {
      return MissionVisualizationState::Failed;
    }
    return MissionVisualizationState::Running;
  }

  void publish_mission_visualization(bool force = false)
  {
    const auto display_state = visualization_state();
    const bool state_changed =
      last_visualized_goal_index_ != current_goal_index_ ||
      last_visualized_visited_goals_ != visited_goals_ ||
      last_visualized_state_ != display_state;
    const auto steady_now = std::chrono::steady_clock::now();
    const bool periodic_update_due = !last_visualization_publish_time_ ||
      std::chrono::duration<double>(
      steady_now - *last_visualization_publish_time_).count() >=
      1.0 / visualization_update_frequency_;
    if (!force && !state_changed && !periodic_update_due)
    {
      return;
    }
    const builtin_interfaces::msg::Time stamp = now();
    if (goals_.empty()) {
      visualization_msgs::msg::MarkerArray clear;
      visualization_msgs::msg::Marker delete_all;
      delete_all.action = visualization_msgs::msg::Marker::DELETEALL;
      clear.markers.push_back(delete_all);
      goal_markers_publisher_->publish(clear);
      last_visualized_state_ = display_state;
      last_visualization_publish_time_ = steady_now;
      return;
    }
    goal_markers_publisher_->publish(make_goal_markers(
      goals_, current_goal_index_, visited_goals_, display_state, frame_id_, stamp,
      visualization_actual_speed_, visualization_reference_speed_,
      trajectory_parameters_.nominal_speed));

    geometry_msgs::msg::PoseStamped current_goal_pose;
    current_goal_pose.header.frame_id = frame_id_;
    current_goal_pose.header.stamp = stamp;
    const auto & goal = goals_[std::min(current_goal_index_, goals_.size() - 1U)];
    current_goal_pose.pose.position.x = goal.position.x();
    current_goal_pose.pose.position.y = goal.position.y();
    current_goal_pose.pose.position.z = goal.position.z();
    current_goal_pose.pose.orientation.z = std::sin(0.5 * goal.yaw);
    current_goal_pose.pose.orientation.w = std::cos(0.5 * goal.yaw);
    current_goal_pose_publisher_->publish(current_goal_pose);

    last_visualized_goal_index_ = current_goal_index_;
    last_visualized_visited_goals_ = visited_goals_;
    last_visualized_state_ = display_state;
    last_visualization_publish_time_ = steady_now;
  }

  void clear_planning_visualization_paths()
  {
    const nav_msgs::msg::Path empty_path = make_path({});
    planned_path_publisher_->publish(empty_path);
    simplified_path_publisher_->publish(empty_path);
    reference_path_publisher_->publish(empty_path);
  }

  void fail(const std::string & reason)
  {
    failure_reason_ = reason;
    state_ = State::Failed;
    stable_duration_ = 0.0;
    clear_planning_visualization_paths();
    RCLCPP_ERROR(get_logger(), "multi-goal static avoidance failed: %s", reason.c_str());
  }

  void start_planning(const Eigen::Vector3d & start)
  {
    hold_position_ = start;
    if (flight_started_ && start.allFinite()) {
      safe_hold_position_ = start;
    }
    const Eigen::Vector3d goal = goals_[current_goal_index_].position;
    const CollisionChecker checker = *navigation_collision_checker_;
    const double resolution = resolution_;
    const std::size_t max_grid_nodes = max_grid_nodes_;
    const PlannedTrajectoryParameters parameters = trajectory_parameters_;
    planning_future_.emplace(
      std::async(
        std::launch::async,
        [checker, resolution, max_grid_nodes, parameters, start, goal]() mutable {
          SegmentPlan result;
          try {
            AStarPlanner planner(checker, resolution, max_grid_nodes);
            result.astar_result = planner.plan(start, goal);
            if (!result.astar_result.success()) {
              result.error = "A* failed for the current ordered goal";
              return result;
            }
            result.trajectory_result =
            PlannedTrajectoryBuilder(checker, parameters).build(result.astar_result.path_world);
            if (!result.trajectory_result.success || !result.trajectory_result.trajectory) {
              result.error = "planned trajectory validation failed for the current ordered goal";
            }
          } catch (const std::exception & error) {
            result.error = error.what();
          }
          return result;
        }));
    state_ = State::PlanningSegment;
    RCLCPP_INFO(
      get_logger(), "planning ordered goal %zu from actual Odom [%.3f, %.3f, %.3f] "
      "to [%.3f, %.3f, %.3f]",
      current_goal_index_, start.x(), start.y(), start.z(), goal.x(), goal.y(), goal.z());
  }

  void accept_plan(SegmentPlan plan)
  {
    if (!plan.error.empty() || !plan.astar_result.success() ||
      !plan.trajectory_result.success || !plan.trajectory_result.trajectory)
    {
      fail(plan.error.empty() ? "segment planning produced an incomplete result" : plan.error);
      return;
    }

    planned_path_publisher_->publish(make_path(plan.astar_result.path_world));
    simplified_path_publisher_->publish(
      make_path(plan.trajectory_result.simplified_path_world));
    trajectory_ = std::move(plan.trajectory_result.trajectory);
    trajectory_total_duration_ = plan.trajectory_result.total_duration;
    std::vector<Eigen::Vector3d> reference_points;
    for (double time = 0.0; time < trajectory_total_duration_;
      time += reference_path_sample_period_)
    {
      reference_points.push_back(trajectory_->sample(time).position_world);
    }
    reference_points.push_back(
      trajectory_->sample(trajectory_total_duration_).position_world);
    reference_path_publisher_->publish(make_path(reference_points));

    trajectory_elapsed_ = 0.0;
    current_segment_ = 0U;
    state_ = State::ExecutingSegment;
    RCLCPP_INFO(
      get_logger(),
      "ordered goal %zu trajectory ready: raw_points=%zu simplified_points=%zu "
      "initial_simplified_points=%zu refinements=%zu duration=%.3f s "
      "velocity_scale=%.2f duration_scale=%.2f max_speed=%.6f m/s "
      "max_acceleration=%.6f m/s^2 raw_length=%.6f m simplified_length=%.6f m "
      "expanded_nodes=%zu",
      current_goal_index_, plan.astar_result.path_world.size(),
      plan.trajectory_result.simplified_path_world.size(),
      plan.trajectory_result.initial_simplified_point_count,
      plan.trajectory_result.refinement_iterations, trajectory_total_duration_,
      plan.trajectory_result.selected_velocity_scale,
      plan.trajectory_result.selected_duration_scale,
      plan.trajectory_result.max_reference_speed,
      plan.trajectory_result.max_reference_acceleration,
      path_length(plan.astar_result.path_world),
      path_length(plan.trajectory_result.simplified_path_world),
      plan.astar_result.expanded_nodes);
  }

  void update()
  {
    const auto steady_now = std::chrono::steady_clock::now();
    const double dt = std::chrono::duration<double>(steady_now - last_update_time_).count();
    last_update_time_ = steady_now;
    current_update_dt_ = dt;
    Eigen::Vector3d odometry_position;
    double speed = 0.0;
    const bool valid_fresh_odometry =
      odometry_is_fresh(steady_now) && valid_odometry(odometry_position, speed);
    if (valid_fresh_odometry && !yaw_initialized_from_odometry_) {
      const auto yaw = quaternion_yaw(latest_odometry_->pose.pose.orientation);
      if (yaw) {
        yaw_generator_->initialize(*yaw);
        yaw_initialized_from_odometry_ = true;
      }
    }
    visualization_actual_speed_ = valid_fresh_odometry ?
      std::optional<double>(speed) : std::nullopt;
    visualization_reference_speed_ = 0.0;
    if (flight_started_ && valid_fresh_odometry && state_ != State::Failed) {
      safe_hold_position_ = odometry_position;
    }

    if (state_ == State::WaitingForPreflightOdometry) {
      if (valid_fresh_odometry) {
        start_preflight(odometry_position);
      } else if (mission_request_time_ &&
        std::chrono::duration<double>(steady_now - *mission_request_time_).count() >=
        interactive_mission_odom_wait_timeout_)
      {
        fail("REJECTED: timed out waiting for fresh Odom before preflight");
      }
    }

    if (state_ == State::PreflightValidation && preflight_future_ &&
      preflight_future_->wait_for(std::chrono::seconds(0)) == std::future_status::ready)
    {
      const PreflightResult result = preflight_future_->get();
      preflight_future_.reset();
      if (!result.success) {
        fail("REJECTED: " + result.message);
      } else if (preflight_requires_takeoff_) {
        state_ = State::CheckingTakeoff;
      } else if (!valid_fresh_odometry) {
        fail("REJECTED: Odom became stale after preflight validation");
      } else {
        RCLCPP_INFO(get_logger(), "preflight passed; planning first goal from airborne Odom");
        start_planning(odometry_position);
      }
    }

    if (state_ == State::WaitingForOdometry && valid_fresh_odometry) {
      initial_position_ = odometry_position;
      takeoff_anchor_ = Eigen::Vector3d(
        odometry_position.x(), odometry_position.y(), takeoff_height_);
      state_ = State::CheckingTakeoff;
    }

    if (state_ == State::CheckingTakeoff) {
      if (takeoff_collision_checker_->segment_in_collision(initial_position_, takeoff_anchor_)) {
        fail("initial position to takeoff anchor is not safe in the original environment");
      } else if (navigation_collision_checker_->point_in_collision(takeoff_anchor_)) {
        fail("takeoff anchor is not valid in the navigation environment");
      } else if (valid_fresh_odometry) {
        hold_position_ = takeoff_anchor_;
        safe_hold_position_ = odometry_position;
        flight_started_ = true;
        stable_duration_ = 0.0;
        state_ = State::TakingOff;
        RCLCPP_INFO(
          get_logger(), "takeoff checked from [%.3f, %.3f, %.3f] to [%.3f, %.3f, %.3f]",
          initial_position_.x(), initial_position_.y(), initial_position_.z(),
          takeoff_anchor_.x(), takeoff_anchor_.y(), takeoff_anchor_.z());
      }
    }

    switch (state_) {
      case State::WaitingForMission:
      case State::WaitingForPreflightOdometry:
        break;
      case State::PreflightValidation:
        if (!preflight_requires_takeoff_ && hold_position_.allFinite()) {
          publish_hold(hold_position_);
        }
        break;
      case State::WaitingForOdometry:
      case State::CheckingTakeoff:
        break;
      case State::TakingOff:
        publish_hold(takeoff_anchor_);
        if (valid_fresh_odometry &&
          (odometry_position - takeoff_anchor_).norm() < takeoff_position_tolerance_ &&
          speed < takeoff_speed_tolerance_)
        {
          stable_duration_ += dt;
        } else {
          stable_duration_ = 0.0;
        }
        if (stable_duration_ >= takeoff_hold_duration_) {
          RCLCPP_INFO(get_logger(), "takeoff stable; starting first ordered goal");
          start_planning(odometry_position);
        }
        break;
      case State::PlanningSegment:
        publish_hold(hold_position_);
        if (planning_future_ &&
          planning_future_->wait_for(std::chrono::seconds(0)) == std::future_status::ready)
        {
          SegmentPlan plan = planning_future_->get();
          planning_future_.reset();
          accept_plan(std::move(plan));
        }
        break;
      case State::ExecutingSegment:
        {
          if (valid_fresh_odometry) {
            trajectory_elapsed_ += dt;
          }
          const auto sample = trajectory_->sample(trajectory_elapsed_);
          visualization_reference_speed_ = sample.velocity_world.norm();
          current_segment_ = sample.segment_index;
          publish_setpoint(
            sample.position_world, sample.velocity_world, sample.acceleration_world,
            goals_[current_goal_index_].yaw);
          if (sample.complete) {
            state_ = State::HoldingGoal;
            stable_duration_ = 0.0;
            hold_position_ = goals_[current_goal_index_].position;
            RCLCPP_INFO(
              get_logger(), "ordered goal %zu trajectory complete; waiting for stable hold",
              current_goal_index_);
          }
          break;
        }
      case State::HoldingGoal:
        publish_hold(goals_[current_goal_index_].position);
        if (valid_fresh_odometry &&
          (odometry_position - goals_[current_goal_index_].position).norm() <
          goal_position_tolerance_ && speed < goal_speed_tolerance_)
        {
          stable_duration_ += dt;
        } else {
          stable_duration_ = 0.0;
        }
        if (stable_duration_ >= goal_hold_duration_) {
          ++visited_goals_;
          RCLCPP_INFO(get_logger(), "ordered goal %zu accepted", current_goal_index_);
          if (current_goal_index_ + 1U < goals_.size()) {
            ++current_goal_index_;
            start_planning(odometry_position);
          } else {
            state_ = State::MissionComplete;
            clear_planning_visualization_paths();
            RCLCPP_INFO(get_logger(), "multi-goal static avoidance mission complete");
          }
        }
        break;
      case State::MissionComplete:
        publish_hold(goals_.back().position);
        break;
      case State::Failed:
        if (const auto command = make_failure_hold_command(
            flight_started_, safe_hold_position_))
        {
          publish_setpoint(
            command->position_world, command->velocity_world,
            command->acceleration_world, yaw_generator_->reference());
        } else if (flight_started_) {
          RCLCPP_FATAL_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "flight failed without a valid safe hold position; refusing an unsafe default target");
        }
        break;
    }
    publish_status();
  }

  std::string frame_id_{"map"};
  std::string goal_source_{"parameters"};
  std::string failure_reason_;
  bool interactive_mode_{false};
  bool mission_ever_accepted_{false};
  bool preflight_requires_takeoff_{false};
  bool flight_started_{false};
  std::uint64_t accepted_draft_revision_{0U};
  double interactive_mission_odom_wait_timeout_{3.0};
  double effective_planning_radius_{0.35};
  double takeoff_height_{1.5};
  double minimum_navigation_altitude_{0.50};
  double odometry_timeout_{0.25};
  double takeoff_position_tolerance_{0.20};
  double takeoff_speed_tolerance_{0.15};
  double takeoff_hold_duration_{1.0};
  double goal_position_tolerance_{0.20};
  double goal_speed_tolerance_{0.15};
  double goal_hold_duration_{1.0};
  double resolution_{0.25};
  double reference_path_sample_period_{0.05};
  double visualization_update_frequency_{5.0};
  double stable_duration_{0.0};
  double trajectory_elapsed_{0.0};
  double trajectory_total_duration_{0.0};
  double current_update_dt_{0.02};
  std::size_t max_grid_nodes_{200000U};
  std::size_t max_goals_{8U};
  std::size_t current_goal_index_{0U};
  std::size_t current_segment_{0U};
  std::size_t visited_goals_{0U};
  bool yaw_initialized_from_odometry_{false};
  State state_{State::WaitingForOdometry};
  PlannedTrajectoryParameters trajectory_parameters_;
  std::vector<MissionGoal> goals_;
  Eigen::Vector3d initial_position_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d takeoff_anchor_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d hold_position_{Eigen::Vector3d::Zero()};
  std::optional<Eigen::Vector3d> safe_hold_position_;
  std::unique_ptr<CollisionChecker> takeoff_collision_checker_;
  std::unique_ptr<CollisionChecker> navigation_collision_checker_;
  std::optional<drone_mission::PiecewiseQuinticTrajectory> trajectory_;
  std::unique_ptr<YawReferenceGenerator> yaw_generator_;
  std::optional<std::future<SegmentPlan>> planning_future_;
  std::optional<std::future<PreflightResult>> preflight_future_;
  std::chrono::steady_clock::time_point last_update_time_;
  std::optional<nav_msgs::msg::Odometry> latest_odometry_;
  std::optional<std::chrono::steady_clock::time_point> latest_odometry_reception_time_;
  std::optional<std::chrono::steady_clock::time_point> mission_request_time_;
  std::optional<double> visualization_actual_speed_;
  double visualization_reference_speed_{0.0};
  std::optional<std::size_t> last_visualized_goal_index_;
  std::optional<std::size_t> last_visualized_visited_goals_;
  std::optional<MissionVisualizationState> last_visualized_state_;
  std::optional<std::chrono::steady_clock::time_point> last_visualization_publish_time_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr planned_path_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr simplified_path_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr reference_path_publisher_;
  rclcpp::Publisher<drone_msgs::msg::TrajectorySetpoint>::SharedPtr setpoint_publisher_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr current_goal_publisher_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr current_segment_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr complete_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr success_publisher_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr visited_goals_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr interactive_active_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr interactive_status_publisher_;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr interactive_revision_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr goal_markers_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr current_goal_pose_publisher_;
  rclcpp::Service<drone_msgs::srv::ExecuteGoalSequence>::SharedPtr execute_service_;
  rclcpp::TimerBase::SharedPtr update_timer_;
};

}  // namespace drone_planning

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<drone_planning::MultiGoalStaticAvoidanceNode>());
  rclcpp::shutdown();
  return 0;
}
