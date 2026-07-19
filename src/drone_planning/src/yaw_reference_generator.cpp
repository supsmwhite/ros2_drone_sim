#include "drone_planning/yaw_reference_generator.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace drone_planning
{
namespace
{

constexpr double kTwoPi = 2.0 * 3.14159265358979323846;

double shortest_angle_error(double target, double current)
{
  return std::remainder(target - current, kTwoPi);
}

void validate(const YawReferenceParameters & parameters)
{
  if (!std::isfinite(parameters.fixed_yaw)) {
    throw std::invalid_argument("fixed_yaw must be finite");
  }
  if (!std::isfinite(parameters.tangent_speed_threshold) ||
    parameters.tangent_speed_threshold < 0.0)
  {
    throw std::invalid_argument("tangent_speed_threshold must be finite and non-negative");
  }
  if (!std::isfinite(parameters.terminal_blend_distance) ||
    parameters.terminal_blend_distance <= 0.0)
  {
    throw std::invalid_argument("terminal_blend_distance must be finite and positive");
  }
  if (!std::isfinite(parameters.filter_time_constant) ||
    parameters.filter_time_constant <= 0.0)
  {
    throw std::invalid_argument("yaw_filter_time_constant must be finite and positive");
  }
  if (!std::isfinite(parameters.max_yaw_rate) || parameters.max_yaw_rate <= 0.0) {
    throw std::invalid_argument("max_yaw_rate must be finite and positive");
  }
}

}  // namespace

YawMode parse_yaw_mode(const std::string & value)
{
  if (value == "fixed") {
    return YawMode::Fixed;
  }
  if (value == "path_tangent") {
    return YawMode::PathTangent;
  }
  throw std::invalid_argument("yaw_mode must be fixed or path_tangent");
}

std::string yaw_mode_name(YawMode mode)
{
  return mode == YawMode::Fixed ? "fixed" : "path_tangent";
}

YawReferenceGenerator::YawReferenceGenerator(YawReferenceParameters parameters)
: parameters_(parameters), reference_yaw_(parameters.fixed_yaw),
  tangent_yaw_(parameters.fixed_yaw)
{
  validate(parameters_);
}

void YawReferenceGenerator::initialize(double initial_yaw)
{
  reference_yaw_ = std::isfinite(initial_yaw) ? initial_yaw : parameters_.fixed_yaw;
  tangent_yaw_ = reference_yaw_;
  initialized_ = true;
  have_valid_tangent_ = false;
}

double YawReferenceGenerator::update(
  const Eigen::Vector3d & trajectory_position,
  const Eigen::Vector3d & trajectory_velocity,
  const Eigen::Vector3d & goal_position,
  double terminal_yaw,
  double dt)
{
  if (parameters_.mode == YawMode::Fixed) {
    reference_yaw_ = parameters_.fixed_yaw;
    initialized_ = true;
    return reference_yaw_;
  }
  if (!initialized_) {
    initialize(parameters_.fixed_yaw);
  }
  if (!trajectory_position.allFinite() || !trajectory_velocity.allFinite() ||
    !goal_position.allFinite() || !std::isfinite(terminal_yaw) ||
    !std::isfinite(dt) || dt <= 0.0)
  {
    return reference_yaw_;
  }

  const double horizontal_speed = trajectory_velocity.head<2>().norm();
  if (!std::isfinite(horizontal_speed)) {
    return reference_yaw_;
  }
  if (horizontal_speed >= parameters_.tangent_speed_threshold) {
    const double wrapped_tangent = std::atan2(
      trajectory_velocity.y(), trajectory_velocity.x());
    if (!std::isfinite(wrapped_tangent)) {
      return reference_yaw_;
    }
    const double tangent_base = have_valid_tangent_ ? tangent_yaw_ : reference_yaw_;
    tangent_yaw_ = tangent_base + shortest_angle_error(wrapped_tangent, tangent_base);
    have_valid_tangent_ = true;
  }

  const double remaining_distance = (goal_position - trajectory_position).norm();
  if (!std::isfinite(remaining_distance)) {
    return reference_yaw_;
  }
  const double blend_base = have_valid_tangent_ ? tangent_yaw_ : reference_yaw_;
  const double u = std::clamp(
    (parameters_.terminal_blend_distance - remaining_distance) /
    parameters_.terminal_blend_distance,
    0.0, 1.0);
  const double weight = u * u * (3.0 - 2.0 * u);
  const double raw_target = blend_base +
    weight * shortest_angle_error(terminal_yaw, blend_base);
  const double alpha = 1.0 - std::exp(-dt / parameters_.filter_time_constant);
  const double filtered_delta = alpha * shortest_angle_error(raw_target, reference_yaw_);
  const double limited_delta = std::clamp(
    filtered_delta, -parameters_.max_yaw_rate * dt, parameters_.max_yaw_rate * dt);
  const double candidate = reference_yaw_ + limited_delta;
  if (std::isfinite(candidate)) {
    reference_yaw_ = candidate;
  }
  return reference_yaw_;
}

double YawReferenceGenerator::reference() const
{
  return reference_yaw_;
}

}  // namespace drone_planning
