#include "drone_mission/goal_input.hpp"

#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <stdexcept>

namespace drone_mission
{

std::vector<double> parse_finite_numbers(const std::vector<std::string> & arguments)
{
  std::vector<double> values;
  values.reserve(arguments.size());
  for (const auto & argument : arguments) {
    char * end = nullptr;
    errno = 0;
    const double value = std::strtod(argument.c_str(), &end);
    if (errno == ERANGE || end == argument.c_str() || *end != '\0' || !std::isfinite(value)) {
      throw std::invalid_argument("invalid finite number: '" + argument + "'");
    }
    values.push_back(value);
  }
  return values;
}

std::vector<double> parse_goal_arguments(const std::vector<std::string> & arguments)
{
  if (arguments.empty() || arguments.size() % 4U != 0U) {
    throw std::invalid_argument("goals require one or more x y z yaw groups");
  }
  std::vector<double> values;
  values.reserve(arguments.size());
  for (std::size_t index = 0U; index < arguments.size(); ++index) {
    const bool yaw_argument = index % 4U == 3U;
    const std::string prefix = "yaw=";
    if (yaw_argument && arguments[index].compare(0U, prefix.size(), prefix) == 0) {
      const auto degrees = parse_finite_numbers({arguments[index].substr(prefix.size())});
      values.push_back(degrees.front() * 3.14159265358979323846 / 180.0);
    } else {
      const auto parsed = parse_finite_numbers({arguments[index]});
      values.push_back(parsed.front());
    }
  }
  return values;
}

void validate_constraints(const GoalConstraints & constraints)
{
  const bool finite = std::isfinite(constraints.x_min) && std::isfinite(constraints.x_max) &&
    std::isfinite(constraints.y_min) && std::isfinite(constraints.y_max) &&
    std::isfinite(constraints.z_min) && std::isfinite(constraints.z_max);
  if (!finite || constraints.x_min >= constraints.x_max ||
    constraints.y_min >= constraints.y_max || constraints.z_min >= constraints.z_max)
  {
    throw std::invalid_argument("workspace must contain finite increasing axis bounds");
  }
}

geometry_msgs::msg::Pose make_goal_pose(
  double x, double y, double z, double yaw, const GoalConstraints & constraints)
{
  validate_constraints(constraints);
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) || !std::isfinite(yaw)) {
    throw std::invalid_argument("all goal values must be finite");
  }
  if (x < constraints.x_min || x > constraints.x_max ||
    y < constraints.y_min || y > constraints.y_max ||
    z < constraints.z_min || z > constraints.z_max)
  {
    throw std::out_of_range("goal is outside the configured workspace/height bounds");
  }
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation.z = std::sin(0.5 * yaw);
  pose.orientation.w = std::cos(0.5 * yaw);
  return pose;
}

std::vector<geometry_msgs::msg::Pose> make_goal_poses(
  const std::vector<double> & values, const GoalConstraints & constraints)
{
  if (values.empty() || values.size() % 4U != 0U) {
    throw std::invalid_argument("multi mode requires one or more x y z yaw groups");
  }
  std::vector<geometry_msgs::msg::Pose> poses;
  poses.reserve(values.size() / 4U);
  for (std::size_t offset = 0U; offset < values.size(); offset += 4U) {
    poses.push_back(make_goal_pose(
      values[offset], values[offset + 1U], values[offset + 2U], values[offset + 3U],
      constraints));
  }
  return poses;
}

}  // namespace drone_mission
