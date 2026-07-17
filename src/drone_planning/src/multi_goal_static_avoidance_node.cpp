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
#include "drone_planning/astar_planner.hpp"
#include "drone_planning/multi_goal_visualization.hpp"
#include "drone_planning/planned_trajectory_builder.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int32.hpp"
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

}  // namespace

class MultiGoalStaticAvoidanceNode : public rclcpp::Node
{
public:
  MultiGoalStaticAvoidanceNode()
  : Node("multi_goal_static_avoidance_node")
  {
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
    goals_ = parse_goals(goal_values);
    const double publish_frequency = declare_parameter<double>("publish_frequency", 50.0);
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

    if (!finite_positive(takeoff_height_) ||
      !std::isfinite(minimum_navigation_altitude_) || minimum_navigation_altitude_ < 0.0 ||
      takeoff_height_ <= minimum_navigation_altitude_ ||
      !finite_positive(publish_frequency) || !finite_positive(odometry_timeout_) ||
      !finite_positive(takeoff_position_tolerance_) ||
      !finite_positive(takeoff_speed_tolerance_) || !finite_positive(takeoff_hold_duration_) ||
      !finite_positive(goal_position_tolerance_) || !finite_positive(goal_speed_tolerance_) ||
      !finite_positive(goal_hold_duration_) || !finite_positive(resolution_) ||
      !finite_positive(reference_path_sample_period_) || trajectory_parameters_.fixed_yaw != 0.0)
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
    publish_mission_visualization();

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
      "multi-goal static avoidance waiting for odometry: goals=%zu takeoff=%.2f m "
      "navigation_floor=%.2f m effective_radius=%.2f m",
      goals_.size(), takeoff_height_, minimum_navigation_altitude_,
      effective_planning_radius_);
  }

private:
  enum class State
  {
    WaitingForOdometry,
    CheckingTakeoff,
    TakingOff,
    PlanningSegment,
    ExecutingSegment,
    HoldingGoal,
    MissionComplete,
    Failed
  };

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
    const Eigen::Vector3d & acceleration, double yaw)
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
    setpoint.yaw = yaw;
    setpoint_publisher_->publish(setpoint);
  }

  void publish_hold(const Eigen::Vector3d & position)
  {
    publish_setpoint(position, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 0.0);
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
    publish_mission_visualization();
  }

  MissionVisualizationState visualization_state() const
  {
    if (state_ == State::MissionComplete) {
      return MissionVisualizationState::Complete;
    }
    if (state_ == State::Failed) {
      return MissionVisualizationState::Failed;
    }
    return MissionVisualizationState::Running;
  }

  void publish_mission_visualization()
  {
    const auto display_state = visualization_state();
    if (last_visualized_goal_index_ == current_goal_index_ &&
      last_visualized_visited_goals_ == visited_goals_ &&
      last_visualized_state_ == display_state)
    {
      return;
    }
    const builtin_interfaces::msg::Time stamp = now();
    goal_markers_publisher_->publish(make_goal_markers(
      goals_, current_goal_index_, visited_goals_, display_state, frame_id_, stamp,
      trajectory_parameters_.nominal_speed));

    geometry_msgs::msg::PoseStamped current_goal_pose;
    current_goal_pose.header.frame_id = frame_id_;
    current_goal_pose.header.stamp = stamp;
    const auto & goal = goals_[std::min(current_goal_index_, goals_.size() - 1U)];
    current_goal_pose.pose.position.x = goal.position.x();
    current_goal_pose.pose.position.y = goal.position.y();
    current_goal_pose.pose.position.z = goal.position.z();
    current_goal_pose.pose.orientation.w = 1.0;
    current_goal_pose_publisher_->publish(current_goal_pose);

    last_visualized_goal_index_ = current_goal_index_;
    last_visualized_visited_goals_ = visited_goals_;
    last_visualized_state_ = display_state;
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
    state_ = State::Failed;
    stable_duration_ = 0.0;
    RCLCPP_ERROR(get_logger(), "multi-goal static avoidance failed: %s", reason.c_str());
  }

  void start_planning(const Eigen::Vector3d & start)
  {
    hold_position_ = start;
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
    Eigen::Vector3d odometry_position;
    double speed = 0.0;
    const bool valid_fresh_odometry =
      odometry_is_fresh(steady_now) && valid_odometry(odometry_position, speed);

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
      } else {
        hold_position_ = takeoff_anchor_;
        stable_duration_ = 0.0;
        state_ = State::TakingOff;
        RCLCPP_INFO(
          get_logger(), "takeoff checked from [%.3f, %.3f, %.3f] to [%.3f, %.3f, %.3f]",
          initial_position_.x(), initial_position_.y(), initial_position_.z(),
          takeoff_anchor_.x(), takeoff_anchor_.y(), takeoff_anchor_.z());
      }
    }

    switch (state_) {
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
          current_segment_ = sample.segment_index;
          publish_setpoint(
            sample.position_world, sample.velocity_world, sample.acceleration_world, sample.yaw);
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
        if (hold_position_.allFinite()) {
          publish_hold(hold_position_);
        }
        break;
    }
    publish_status();
  }

  std::string frame_id_{"map"};
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
  double stable_duration_{0.0};
  double trajectory_elapsed_{0.0};
  double trajectory_total_duration_{0.0};
  std::size_t max_grid_nodes_{200000U};
  std::size_t current_goal_index_{0U};
  std::size_t current_segment_{0U};
  std::size_t visited_goals_{0U};
  State state_{State::WaitingForOdometry};
  PlannedTrajectoryParameters trajectory_parameters_;
  std::vector<MissionGoal> goals_;
  Eigen::Vector3d initial_position_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d takeoff_anchor_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d hold_position_{Eigen::Vector3d::Zero()};
  std::unique_ptr<CollisionChecker> takeoff_collision_checker_;
  std::unique_ptr<CollisionChecker> navigation_collision_checker_;
  std::optional<drone_mission::PiecewiseQuinticTrajectory> trajectory_;
  std::optional<std::future<SegmentPlan>> planning_future_;
  std::chrono::steady_clock::time_point last_update_time_;
  std::optional<nav_msgs::msg::Odometry> latest_odometry_;
  std::optional<std::chrono::steady_clock::time_point> latest_odometry_reception_time_;
  std::optional<std::size_t> last_visualized_goal_index_;
  std::optional<std::size_t> last_visualized_visited_goals_;
  std::optional<MissionVisualizationState> last_visualized_state_;
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
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr goal_markers_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr current_goal_pose_publisher_;
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
