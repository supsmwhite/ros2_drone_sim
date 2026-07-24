#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <future>
#include <iomanip>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include "drone_planning/astar_planner.hpp"
#include "drone_planning/interactive_goal_editor.hpp"
#include "drone_planning/planned_trajectory_builder.hpp"
#include "drone_msgs/srv/execute_goal_sequence.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "interactive_markers/interactive_marker_server.hpp"
#include "interactive_markers/menu_handler.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int32.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "visualization_msgs/msg/interactive_marker_feedback.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace drone_planning
{
namespace
{

constexpr char kMarkerName[] = "goal_candidate";
constexpr char kServerNamespace[] = "/drone/interactive_goals/goal_editor";

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
      throw std::invalid_argument("obstacle centers and positive sizes must be finite");
    }
    obstacles.push_back({center - 0.5 * size, center + 0.5 * size});
  }
  return obstacles;
}

std::string segment_name(std::size_t segment_index)
{
  if (segment_index == 0U) {
    return "START -> P1";
  }
  return "P" + std::to_string(segment_index) + " -> P" +
         std::to_string(segment_index + 1U);
}

struct SequencePlanResult
{
  bool success{false};
  std::string message;
  std::optional<std::size_t> failed_goal_index;
  std::vector<Eigen::Vector3d> preview_points;
  std::uint64_t revision{0U};
};

std::string trajectory_failure_message(
  std::size_t segment_index, TrajectoryFailureReason reason)
{
  const std::string prefix = "REJECTED: segment " + segment_name(segment_index) + " ";
  if (reason == TrajectoryFailureReason::speed_limit ||
    reason == TrajectoryFailureReason::acceleration_limit)
  {
    return prefix + "failed dynamic constraint validation";
  }
  return prefix + "has no valid continuous trajectory (trajectory generation failure)";
}

}  // namespace

class InteractiveGoalEditorNode : public rclcpp::Node
{
public:
  InteractiveGoalEditorNode()
  : Node("interactive_goal_editor_node"),
    editor_(validated_max_goals(declare_parameter<std::int64_t>("max_goals", 8)))
  {
    execution_enabled_ = declare_parameter<bool>("execution_enabled", false);
    frame_id_ = declare_parameter<std::string>("frame_id", "map");
    if (frame_id_ != "map") {
      throw std::invalid_argument("interactive goal editor frame_id must be map");
    }
    const auto workspace_values =
      declare_parameter<std::vector<double>>("workspace", std::vector<double>{});
    const auto obstacle_values =
      declare_parameter<std::vector<double>>("obstacles", std::vector<double>{});
    const double safety_radius = declare_parameter<double>("safety_radius", 0.25);
    const double planning_margin = declare_parameter<double>("planning_margin", 0.10);
    minimum_navigation_altitude_ =
      declare_parameter<double>("minimum_navigation_altitude", 0.50);
    snap_resolution_ = declare_parameter<double>("snap_resolution", 0.05);
    const auto planning_start_values =
      declare_parameter<std::vector<double>>("planning_start", {0.0, 0.0, 1.5});
    resolution_ = declare_parameter<double>("resolution", 0.25);
    const auto max_grid_nodes = declare_parameter<std::int64_t>("max_grid_nodes", 200000);
    preview_sample_period_ = declare_parameter<double>("reference_path_sample_period", 0.05);

    trajectory_parameters_.nominal_speed = declare_parameter<double>("nominal_speed", 0.35);
    trajectory_parameters_.min_segment_duration =
      declare_parameter<double>("min_segment_duration", 2.0);
    trajectory_parameters_.validation_sample_period =
      declare_parameter<double>("validation_sample_period", 0.02);
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
      {1.0, 1.05, 1.10, 1.15, 1.20, 1.25, 1.30, 1.35, 1.40, 1.45, 1.50,
        1.75, 2.0, 3.0, 4.0});
    const auto max_refinements =
      declare_parameter<std::int64_t>("max_refinement_iterations", 8);
    const auto max_insertions =
      declare_parameter<std::int64_t>("max_insertions_per_refinement", 3);
    trajectory_parameters_.fixed_yaw = declare_parameter<double>("fixed_yaw", 0.0);

    if (!std::isfinite(safety_radius) || safety_radius < 0.0 ||
      !std::isfinite(planning_margin) || planning_margin < 0.0 ||
      !std::isfinite(minimum_navigation_altitude_) || minimum_navigation_altitude_ < 0.0 ||
      !finite_positive(snap_resolution_) || !finite_positive(resolution_) ||
      !finite_positive(preview_sample_period_) || max_grid_nodes <= 0 ||
      max_refinements < 0 || max_insertions <= 0 || trajectory_parameters_.fixed_yaw != 0.0)
    {
      throw std::invalid_argument("interactive goal editor scalar parameters are invalid");
    }
    max_grid_nodes_ = static_cast<std::size_t>(max_grid_nodes);
    trajectory_parameters_.max_refinement_iterations =
      static_cast<std::size_t>(max_refinements);
    trajectory_parameters_.max_insertions_per_refinement =
      static_cast<std::size_t>(max_insertions);
    if (planning_start_values.size() != 3U) {
      throw std::invalid_argument("planning_start must contain [x,y,z]");
    }
    planning_start_ = Eigen::Vector3d(
      planning_start_values[0], planning_start_values[1], planning_start_values[2]);

    const AxisAlignedBox original_workspace = parse_workspace(workspace_values);
    const auto obstacles = parse_obstacles(obstacle_values);
    AxisAlignedBox navigation_workspace = original_workspace;
    const double effective_radius = safety_radius + planning_margin;
    navigation_workspace.min_corner.z() = minimum_navigation_altitude_ - effective_radius;
    collision_checker_ = std::make_unique<CollisionChecker>(
      StaticEnvironment(navigation_workspace, obstacles), effective_radius);
    const auto start_validation = validate_goal_candidate(
      planning_start_, *collision_checker_, minimum_navigation_altitude_);
    if (!start_validation.valid) {
      throw std::invalid_argument("planning_start is invalid: " + start_validation.reason);
    }

    const auto qos = rclcpp::QoS(1).transient_local().reliable();
    goal_markers_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/drone/interactive_goals/goal_markers", qos);
    selected_goals_publisher_ = create_publisher<geometry_msgs::msg::PoseArray>(
      "/drone/interactive_goals/selected_goals", qos);
    preview_path_publisher_ = create_publisher<nav_msgs::msg::Path>(
      "/drone/interactive_goals/preview_path", qos);
    status_publisher_ = create_publisher<std_msgs::msg::String>(
      "/drone/interactive_goals/status", qos);
    ready_publisher_ = create_publisher<std_msgs::msg::Bool>(
      "/drone/interactive_goals/ready", qos);
    count_publisher_ = create_publisher<std_msgs::msg::UInt32>(
      "/drone/interactive_goals/count", qos);
    if (execution_enabled_) {
      execute_client_ = create_client<drone_msgs::srv::ExecuteGoalSequence>(
        "/drone/interactive_goals/execute");
      mission_active_subscription_ = create_subscription<std_msgs::msg::Bool>(
        "/drone/interactive_mission/active", qos,
        [this](const std_msgs::msg::Bool::SharedPtr message) {
          execution_active_ = message->data;
          if (execution_active_) {
            mission_submitted_ = true;
            status_override_ = "MISSION EXECUTION ACTIVE";
            rebuild_candidate_marker();
            publish_state();
          }
        });
      mission_status_subscription_ = create_subscription<std_msgs::msg::String>(
        "/drone/interactive_mission/status", qos,
        [this](const std_msgs::msg::String::SharedPtr message) {
          executor_status_ = message->data;
          if (mission_submitted_) {
            status_override_ = executor_status_;
            publish_state();
          }
        });
      mission_revision_subscription_ = create_subscription<std_msgs::msg::UInt64>(
        "/drone/interactive_mission/draft_revision", qos,
        [this](const std_msgs::msg::UInt64::SharedPtr message) {
          executor_revision_ = message->data;
        });
    }

    marker_server_ = std::make_unique<interactive_markers::InteractiveMarkerServer>(
      kServerNamespace, get_node_base_interface(), get_node_clock_interface(),
      get_node_logging_interface(), get_node_topics_interface(), get_node_services_interface());
    configure_menu();
    quick_validate();
    rebuild_candidate_marker();
    publish_state();

    future_timer_ = create_wall_timer(
      std::chrono::milliseconds(50), [this]() {poll_planning_result();});
    RCLCPP_INFO(
      get_logger(),
      "interactive goal editor ready: update_topic=%s/update max_goals=%zu, %s",
      kServerNamespace, editor_.max_goals(),
      execution_enabled_ ? "preview and execution enabled" : "preview only");
  }

private:
  static std::size_t validated_max_goals(std::int64_t value)
  {
    if (value <= 0 ||
      static_cast<std::uint64_t>(value) > std::numeric_limits<std::uint32_t>::max())
    {
      throw std::invalid_argument("max_goals must be positive and fit in UInt32");
    }
    return static_cast<std::size_t>(value);
  }

  CandidateValidation candidate_validation() const
  {
    return validate_goal_candidate(
      editor_.candidate().position, *collision_checker_, minimum_navigation_altitude_);
  }

  void quick_validate()
  {
    editor_.set_candidate_validation(candidate_validation());
  }

  void invalidate_preview_storage()
  {
    preview_points_.clear();
  }

  void configure_menu()
  {
    menu_handler_.insert("Add Goal", [this](const auto &) {add_goal();});
    menu_handler_.insert("Undo Last Goal", [this](const auto &) {undo_goal();});
    menu_handler_.insert("Clear All Goals", [this](const auto &) {clear_goals();});
    const auto height_menu = menu_handler_.insert("Set Height");
    menu_handler_.insert(height_menu, "1.5 m", [this](const auto &) {set_height(1.5);});
    menu_handler_.insert(height_menu, "2.5 m", [this](const auto &) {set_height(2.5);});
    menu_handler_.insert(height_menu, "4.0 m", [this](const auto &) {set_height(4.0);});
    menu_handler_.insert("Validate & Preview", [this](const auto &) {start_validation();});
    if (execution_enabled_) {
      menu_handler_.insert(
        "Execute Validated Mission", [this](const auto &) {execute_validated_mission();});
    }
    menu_handler_.insert("Print Mission YAML", [this](const auto &) {print_yaml();});
    const auto yaw_menu = menu_handler_.insert("Set Yaw");
    const std::vector<std::pair<std::string, double>> yaw_options{
      {"0 deg", 0.0}, {"45 deg", M_PI / 4.0}, {"90 deg", M_PI / 2.0},
      {"135 deg", 3.0 * M_PI / 4.0}, {"180 deg", M_PI},
      {"-135 deg", -3.0 * M_PI / 4.0}, {"-90 deg", -M_PI / 2.0},
      {"-45 deg", -M_PI / 4.0}};
    for (const auto & option : yaw_options) {
      menu_handler_.insert(
        yaw_menu, option.first, [this, yaw = option.second](const auto &) {set_yaw(yaw);});
    }
  }

  bool editor_locked() const
  {
    return execute_request_pending_ || mission_submitted_ || execution_active_;
  }

  bool reject_if_locked(const std::string & action)
  {
    if (!editor_locked()) {
      return false;
    }
    status_override_ = "MISSION EXECUTION ACTIVE: " + action + " REJECTED";
    RCLCPP_WARN(get_logger(), "%s", status_override_->c_str());
    rebuild_candidate_marker();
    publish_state();
    return true;
  }

  void rebuild_candidate_marker()
  {
    if (editor_locked()) {
      marker_server_->erase(kMarkerName);
      marker_server_->applyChanges();
      return;
    }
    auto marker = make_goal_candidate_marker(
      editor_.candidate(), editor_.goals().size() + 1U, editor_.state(),
      editor_.status_message(), frame_id_);
    marker_server_->insert(
      marker, [this](const auto feedback) {process_feedback(feedback);});
    menu_handler_.apply(*marker_server_, kMarkerName);
    marker_server_->applyChanges();
  }

  void process_feedback(
    const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & feedback)
  {
    if (editor_locked()) {
      rebuild_candidate_marker();
      publish_state();
      return;
    }
    status_override_.reset();
    if (feedback->event_type == visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE) {
      const Eigen::Vector3d position(
        feedback->pose.position.x, feedback->pose.position.y, feedback->pose.position.z);
      const auto feedback_yaw = yaw_from_quaternion(feedback->pose.orientation);
      if (!position.allFinite() || !feedback_yaw) {
        status_override_ = "EDIT REJECTED: invalid pose";
        RCLCPP_WARN(get_logger(), "%s", status_override_->c_str());
        rebuild_candidate_marker();
        publish_state();
        return;
      }
      const bool translation_control = feedback->control_name.rfind("move_", 0U) == 0U;
      const double yaw = translation_control ? editor_.candidate().yaw : *feedback_yaw;
      if (!editor_.set_candidate({position, yaw})) {
        status_override_ = "EDIT REJECTED: invalid pose";
        rebuild_candidate_marker();
        publish_state();
        return;
      }
      invalidate_preview_storage();
      rebuild_candidate_marker();
      publish_state();
    } else if (feedback->event_type ==
      visualization_msgs::msg::InteractiveMarkerFeedback::MOUSE_UP)
    {
      const Eigen::Vector3d snapped =
        snap_goal_candidate(editor_.candidate().position, snap_resolution_);
      editor_.set_candidate_position(snapped);
      invalidate_preview_storage();
      quick_validate();
      rebuild_candidate_marker();
      publish_state();
    }
  }

  void add_goal()
  {
    if (reject_if_locked("ADD")) {
      return;
    }
    status_override_.reset();
    std::string reason;
    if (editor_.add_goal(candidate_validation(), reason)) {
      invalidate_preview_storage();
      RCLCPP_INFO(get_logger(), "added P%zu", editor_.goals().size());
    } else {
      RCLCPP_WARN(get_logger(), "Add Goal rejected: %s", reason.c_str());
    }
    rebuild_candidate_marker();
    publish_state();
  }

  void undo_goal()
  {
    if (reject_if_locked("UNDO")) {
      return;
    }
    status_override_.reset();
    if (!editor_.undo_last_goal()) {
      RCLCPP_WARN(get_logger(), "Undo Last Goal ignored: goal list is empty");
    }
    invalidate_preview_storage();
    rebuild_candidate_marker();
    publish_state();
  }

  void clear_goals()
  {
    if (reject_if_locked("CLEAR")) {
      return;
    }
    status_override_.reset();
    editor_.clear_goals();
    invalidate_preview_storage();
    rebuild_candidate_marker();
    publish_state();
  }

  void set_height(double height)
  {
    if (reject_if_locked("SET HEIGHT")) {
      return;
    }
    status_override_.reset();
    Eigen::Vector3d position = editor_.candidate().position;
    position.z() = height;
    editor_.set_candidate_position(position);
    invalidate_preview_storage();
    quick_validate();
    rebuild_candidate_marker();
    publish_state();
  }

  void set_yaw(double yaw)
  {
    if (reject_if_locked("SET YAW")) {
      return;
    }
    status_override_.reset();
    if (!editor_.set_candidate_yaw(yaw)) {
      status_override_ = "SET YAW REJECTED: non-finite yaw";
    }
    invalidate_preview_storage();
    quick_validate();
    rebuild_candidate_marker();
    publish_state();
  }

  void start_validation()
  {
    if (reject_if_locked("VALIDATE")) {
      return;
    }
    status_override_.reset();
    if (planning_future_) {
      RCLCPP_WARN(get_logger(), "Validate & Preview ignored while a planner task is active");
      return;
    }
    const bool candidate_is_last_goal =
      !editor_.goals().empty() &&
      editor_.goals().back().position.isApprox(editor_.candidate().position, 1.0e-9) &&
      std::abs(normalize_angle(editor_.goals().back().yaw - editor_.candidate().yaw)) < 1.0e-9;
    if (!candidate_is_last_goal) {
      std::string reason;
      if (!editor_.add_goal(candidate_validation(), reason)) {
        RCLCPP_WARN(
          get_logger(), "Validate & Preview rejected current candidate: %s", reason.c_str());
        rebuild_candidate_marker();
        publish_state();
        return;
      }
      invalidate_preview_storage();
      RCLCPP_INFO(
        get_logger(), "Validate & Preview included current candidate as P%zu",
        editor_.goals().size());
    }
    std::uint64_t revision = 0U;
    std::vector<InteractiveGoal> goals;
    if (!editor_.begin_validation(revision, goals)) {
      rebuild_candidate_marker();
      publish_state();
      return;
    }
    invalidate_preview_storage();
    rebuild_candidate_marker();
    publish_state();

    const CollisionChecker checker = *collision_checker_;
    const Eigen::Vector3d start = planning_start_;
    const double resolution = resolution_;
    const std::size_t max_grid_nodes = max_grid_nodes_;
    const auto parameters = trajectory_parameters_;
    const double sample_period = preview_sample_period_;
    planning_future_.emplace(
      std::async(
        std::launch::async,
        [checker, start, goals, resolution, max_grid_nodes, parameters,
        sample_period, revision]() mutable {
          SequencePlanResult result;
          result.revision = revision;
          Eigen::Vector3d segment_start = start;
          try {
            for (std::size_t index = 0U; index < goals.size(); ++index) {
              AStarPlanner planner(checker, resolution, max_grid_nodes);
              const auto astar = planner.plan(segment_start, goals[index].position);
              if (!astar.success()) {
                result.message = "REJECTED: segment " + segment_name(index) +
                " A* failure";
                result.failed_goal_index = index;
                return result;
              }
              const auto trajectory_result =
              PlannedTrajectoryBuilder(checker, parameters).build(astar.path_world);
              if (!trajectory_result.success || !trajectory_result.trajectory) {
                result.message = trajectory_failure_message(
                  index, trajectory_result.failure_reason);
                result.failed_goal_index = index;
                return result;
              }
              const auto & trajectory = *trajectory_result.trajectory;
              for (double time = 0.0; time < trajectory.total_duration(); time += sample_period) {
                const auto point = trajectory.sample(time).position_world;
                if (result.preview_points.empty() ||
                !result.preview_points.back().isApprox(point, 1.0e-12))
                {
                  result.preview_points.push_back(point);
                }
              }
              const auto endpoint = trajectory.sample(trajectory.total_duration()).position_world;
              if (result.preview_points.empty() ||
              !result.preview_points.back().isApprox(endpoint, 1.0e-12))
              {
                result.preview_points.push_back(endpoint);
              }
              segment_start = goals[index].position;
            }
            result.success = true;
            result.message = "READY: all " + std::to_string(goals.size()) +
            " ordered segments passed full trajectory validation";
          } catch (const std::exception & error) {
            result.message = std::string("REJECTED: planning exception: ") + error.what();
          }
          return result;
        }));
  }

  void poll_planning_result()
  {
    if (!planning_future_ ||
      planning_future_->wait_for(std::chrono::seconds(0)) != std::future_status::ready)
    {
      return;
    }
    SequencePlanResult result = planning_future_->get();
    planning_future_.reset();
    if (!editor_.accept_validation(
        result.revision, result.success, result.message, result.failed_goal_index))
    {
      RCLCPP_INFO(get_logger(), "discarded stale preview for revision %lu", result.revision);
      return;
    }
    preview_points_ = result.success ? std::move(result.preview_points) :
      std::vector<Eigen::Vector3d>{};
    if (result.success) {
      RCLCPP_INFO(get_logger(), "%s", result.message.c_str());
    } else {
      RCLCPP_WARN(get_logger(), "%s", result.message.c_str());
    }
    rebuild_candidate_marker();
    publish_state();
  }

  void execute_validated_mission()
  {
    if (!execution_enabled_) {
      return;
    }
    if (reject_if_locked("EXECUTE")) {
      return;
    }
    if (!editor_.preview_valid()) {
      status_override_ = "EXECUTE REJECTED: run successful Validate & Preview first";
      RCLCPP_WARN(get_logger(), "%s", status_override_->c_str());
      rebuild_candidate_marker();
      publish_state();
      return;
    }
    if (!execute_client_->service_is_ready()) {
      status_override_ = "EXECUTE REJECTED: mission executor is unavailable";
      RCLCPP_WARN(get_logger(), "%s", status_override_->c_str());
      rebuild_candidate_marker();
      publish_state();
      return;
    }

    auto request = std::make_shared<drone_msgs::srv::ExecuteGoalSequence::Request>();
    request->goals = make_selected_goals(editor_.goals(), frame_id_);
    request->goals.header.stamp = now();
    request->draft_revision = editor_.draft_revision();
    submitted_revision_ = request->draft_revision;
    execute_request_pending_ = true;
    status_override_ = "EXECUTION REQUEST PENDING";
    rebuild_candidate_marker();
    publish_state();

    execute_client_->async_send_request(
      request,
      [this](rclcpp::Client<drone_msgs::srv::ExecuteGoalSequence>::SharedFuture future) {
        execute_request_pending_ = false;
        try {
          const auto response = future.get();
          if (!response->accepted) {
            status_override_ = "EXECUTE REJECTED: " + response->message;
            RCLCPP_WARN(get_logger(), "%s", status_override_->c_str());
          } else {
            mission_submitted_ = true;
            status_override_ = "EXECUTION REQUEST ACCEPTED";
            RCLCPP_INFO(
              get_logger(), "interactive mission revision=%lu accepted by executor",
              submitted_revision_);
          }
        } catch (const std::exception & error) {
          status_override_ = "EXECUTE REJECTED: mission executor request failed";
          RCLCPP_WARN(get_logger(), "%s", status_override_->c_str());
          RCLCPP_DEBUG(get_logger(), "executor request error: %s", error.what());
        }
        rebuild_candidate_marker();
        publish_state();
      });
  }

  void print_yaml()
  {
    if (!editor_.preview_valid()) {
      RCLCPP_WARN(
        get_logger(), "Print Mission YAML rejected: run successful Validate & Preview first");
      return;
    }
    const std::string yaml = format_mission_yaml(editor_.goals());
    RCLCPP_INFO(
      get_logger(), "Validated mission YAML (copy only; no file was changed):\n%s",
      yaml.c_str());
  }

  visualization_msgs::msg::MarkerArray make_goal_markers() const
  {
    auto array = make_interactive_goal_markers(
      editor_.goals(), editor_.state(), editor_.failed_goal_index(), frame_id_);
    if (editor_locked()) {
      array.markers.resize(1U);
      return array;
    }
    const auto stamp = now();
    for (auto & marker : array.markers) {
      marker.header.stamp = stamp;
    }
    return array;
  }

  nav_msgs::msg::Path make_preview_path() const
  {
    nav_msgs::msg::Path path;
    path.header.frame_id = frame_id_;
    path.header.stamp = now();
    if (editor_locked()) {
      return path;
    }
    path.poses.reserve(preview_points_.size());
    for (const auto & point : preview_points_) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = point.x();
      pose.pose.position.y = point.y();
      pose.pose.position.z = point.z();
      pose.pose.orientation.w = 1.0;
      path.poses.push_back(pose);
    }
    return path;
  }

  void publish_state()
  {
    goal_markers_publisher_->publish(make_goal_markers());
    auto goals = make_selected_goals(editor_.goals(), frame_id_);
    goals.header.stamp = now();
    selected_goals_publisher_->publish(goals);
    preview_path_publisher_->publish(make_preview_path());
    std_msgs::msg::String status;
    status.data = status_override_.value_or(editor_.status_message());
    status_publisher_->publish(status);
    std_msgs::msg::Bool ready;
    ready.data = editor_.preview_valid();
    ready_publisher_->publish(ready);
    std_msgs::msg::UInt32 count;
    count.data = static_cast<std::uint32_t>(editor_.goals().size());
    count_publisher_->publish(count);
  }

  InteractiveGoalEditor editor_;
  std::string frame_id_{"map"};
  double minimum_navigation_altitude_{0.50};
  double snap_resolution_{0.05};
  double resolution_{0.25};
  double preview_sample_period_{0.05};
  std::size_t max_grid_nodes_{200000U};
  Eigen::Vector3d planning_start_{0.0, 0.0, 1.5};
  PlannedTrajectoryParameters trajectory_parameters_;
  std::unique_ptr<CollisionChecker> collision_checker_;
  std::vector<Eigen::Vector3d> preview_points_;
  std::optional<std::future<SequencePlanResult>> planning_future_;
  bool execute_request_pending_{false};
  bool execution_enabled_{false};
  bool mission_submitted_{false};
  bool execution_active_{false};
  std::uint64_t submitted_revision_{0U};
  std::uint64_t executor_revision_{0U};
  std::string executor_status_;
  std::optional<std::string> status_override_;

  std::unique_ptr<interactive_markers::InteractiveMarkerServer> marker_server_;
  interactive_markers::MenuHandler menu_handler_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr goal_markers_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr selected_goals_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr preview_path_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_publisher_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr count_publisher_;
  rclcpp::Client<drone_msgs::srv::ExecuteGoalSequence>::SharedPtr execute_client_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mission_active_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mission_status_subscription_;
  rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr mission_revision_subscription_;
  rclcpp::TimerBase::SharedPtr future_timer_;
};

}  // namespace drone_planning

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<drone_planning::InteractiveGoalEditorNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("interactive_goal_editor_node"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
