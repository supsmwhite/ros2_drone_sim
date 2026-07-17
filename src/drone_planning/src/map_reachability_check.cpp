#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "drone_planning/astar_planner.hpp"
#include "rclcpp/rclcpp.hpp"

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
    throw std::invalid_argument("obstacles must contain center and size groups");
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

bool strictly_inside(const Eigen::Vector3d & point, const AxisAlignedBox & box)
{
  return (point.array() > box.min_corner.array()).all() &&
         (point.array() < box.max_corner.array()).all();
}

bool inside_closed(const Eigen::Vector3d & point, const AxisAlignedBox & box)
{
  return (point.array() >= box.min_corner.array()).all() &&
         (point.array() <= box.max_corner.array()).all();
}

std::vector<double> samples(double minimum, double maximum, double spacing)
{
  std::vector<double> values;
  for (double value = minimum; value <= maximum + 1.0e-12; value += spacing) {
    values.push_back(value);
  }
  return values;
}

struct LayerStatistics
{
  std::size_t sampled{0U};
  std::size_t outside_safe_workspace{0U};
  std::size_t inside_inflated_obstacle{0U};
  std::size_t safe{0U};
  std::size_t reachable{0U};
  std::vector<Eigen::Vector3d> unreachable;
};

}  // namespace

class MapReachabilityCheck : public rclcpp::Node
{
public:
  MapReachabilityCheck()
  : Node("map_reachability_check")
  {
    const auto workspace_values =
      declare_parameter<std::vector<double>>("workspace", std::vector<double>{});
    const auto obstacle_values =
      declare_parameter<std::vector<double>>("obstacles", std::vector<double>{});
    const double safety_radius = declare_parameter<double>("safety_radius", 0.25);
    const double planning_margin = declare_parameter<double>("planning_margin", 0.10);
    resolution_ = declare_parameter<double>("resolution", 0.25);
    const auto max_grid_nodes = declare_parameter<std::int64_t>("max_grid_nodes", 200000);
    spacing_ = declare_parameter<double>("sampling_spacing", 1.5);
    heights_ = declare_parameter<std::vector<double>>(
      "sample_heights", {1.5, 2.5, 4.0});
    const auto start_values = declare_parameter<std::vector<double>>(
      "start", {0.0, 0.0, 1.5});
    required_goal_values_ = declare_parameter<std::vector<double>>(
      "required_goals",
      {12.1, 5.5, 1.5, 12.1, 1.1, 1.5, 7.0, 5.0, 4.0, 0.8, 0.7, 2.0});

    if (!std::isfinite(safety_radius) || safety_radius < 0.0 ||
      !std::isfinite(planning_margin) || planning_margin < 0.0 ||
      !std::isfinite(resolution_) || resolution_ <= 0.0 ||
      max_grid_nodes <= 0 || !std::isfinite(spacing_) || spacing_ <= 0.0 ||
      heights_.empty() || start_values.size() != 3U ||
      required_goal_values_.size() % 3U != 0U)
    {
      throw std::invalid_argument("map reachability parameters are invalid");
    }
    for (const double height : heights_) {
      if (!std::isfinite(height)) {
        throw std::invalid_argument("sample heights must be finite");
      }
    }
    start_ = Eigen::Vector3d(start_values[0], start_values[1], start_values[2]);
    environment_ = StaticEnvironment(
      parse_workspace(workspace_values), parse_obstacles(obstacle_values));
    checker_ = std::make_unique<CollisionChecker>(
      environment_, safety_radius + planning_margin);
    planner_ = std::make_unique<AStarPlanner>(
      *checker_, resolution_, static_cast<std::size_t>(max_grid_nodes));
  }

  bool run() const
  {
    if (checker_->point_in_collision(start_)) {
      std::cerr << "reachability_error: start is not planning-safe\n";
      return false;
    }

    bool all_required_reachable = true;
    for (std::size_t offset = 0U; offset < required_goal_values_.size(); offset += 3U) {
      const Eigen::Vector3d goal(
        required_goal_values_[offset], required_goal_values_[offset + 1U],
        required_goal_values_[offset + 2U]);
      const bool safe = !checker_->point_in_collision(goal);
      const auto result = safe ? planner_->plan(start_, goal) : AStarResult{};
      std::cout << std::fixed << std::setprecision(3)
                << "required_goal=" << goal.transpose()
                << " safe=" << std::boolalpha << safe
                << " reachable=" << (safe && result.success())
                << " path_points=" << result.path_world.size()
                << " expanded_nodes=" << result.expanded_nodes << '\n';
      all_required_reachable = all_required_reachable && safe && result.success();
    }

    const auto x_values = samples(
      environment_.workspace().min_corner.x(), environment_.workspace().max_corner.x(), spacing_);
    const auto y_values = samples(
      environment_.workspace().min_corner.y(), environment_.workspace().max_corner.y(), spacing_);
    LayerStatistics total;
    for (const double height : heights_) {
      LayerStatistics layer;
      for (const double x : x_values) {
        for (const double y : y_values) {
          const Eigen::Vector3d goal(x, y, height);
          ++layer.sampled;
          if (!strictly_inside(goal, checker_->safe_workspace())) {
            ++layer.outside_safe_workspace;
            continue;
          }
          bool in_obstacle = false;
          for (const auto & obstacle : checker_->inflated_obstacles()) {
            if (inside_closed(goal, obstacle)) {
              in_obstacle = true;
              break;
            }
          }
          if (in_obstacle) {
            ++layer.inside_inflated_obstacle;
            continue;
          }
          ++layer.safe;
          const auto result = planner_->plan(start_, goal);
          if (result.success()) {
            ++layer.reachable;
          } else {
            layer.unreachable.push_back(goal);
          }
        }
      }
      print_layer(height, layer);
      total.sampled += layer.sampled;
      total.outside_safe_workspace += layer.outside_safe_workspace;
      total.inside_inflated_obstacle += layer.inside_inflated_obstacle;
      total.safe += layer.safe;
      total.reachable += layer.reachable;
      total.unreachable.insert(
        total.unreachable.end(), layer.unreachable.begin(), layer.unreachable.end());
    }
    const double ratio = total.safe == 0U ? 0.0 :
      100.0 * static_cast<double>(total.reachable) / static_cast<double>(total.safe);
    std::cout << std::fixed << std::setprecision(2)
              << "reachability_total sampled=" << total.sampled
              << " outside_safe_workspace=" << total.outside_safe_workspace
              << " inside_inflated_obstacle=" << total.inside_inflated_obstacle
              << " safe=" << total.safe
              << " reachable=" << total.reachable
              << " unreachable=" << total.unreachable.size()
              << " ratio_percent=" << ratio << '\n';
    for (const auto & point : total.unreachable) {
      std::cout << "unreachable_goal=" << point.transpose() << '\n';
    }
    return all_required_reachable && total.safe > 0U && total.unreachable.empty();
  }

private:
  static void print_layer(double height, const LayerStatistics & layer)
  {
    const double ratio = layer.safe == 0U ? 0.0 :
      100.0 * static_cast<double>(layer.reachable) / static_cast<double>(layer.safe);
    std::cout << std::fixed << std::setprecision(2)
              << "reachability_layer z=" << height
              << " sampled=" << layer.sampled
              << " outside_safe_workspace=" << layer.outside_safe_workspace
              << " inside_inflated_obstacle=" << layer.inside_inflated_obstacle
              << " safe=" << layer.safe
              << " reachable=" << layer.reachable
              << " unreachable=" << layer.unreachable.size()
              << " ratio_percent=" << ratio << '\n';
  }

  StaticEnvironment environment_{
    AxisAlignedBox{Eigen::Vector3d::Zero(), Eigen::Vector3d::Ones()}, {}};
  std::unique_ptr<CollisionChecker> checker_;
  std::unique_ptr<AStarPlanner> planner_;
  Eigen::Vector3d start_{Eigen::Vector3d::Zero()};
  std::vector<double> heights_;
  std::vector<double> required_goal_values_;
  double resolution_{0.25};
  double spacing_{1.5};
};

}  // namespace drone_planning

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    const auto checker = std::make_shared<drone_planning::MapReachabilityCheck>();
    const bool success = checker->run();
    rclcpp::shutdown();
    return success ? 0 : 1;
  } catch (const std::exception & error) {
    std::cerr << "map_reachability_check failed: " << error.what() << '\n';
    rclcpp::shutdown();
    return 2;
  }
}
