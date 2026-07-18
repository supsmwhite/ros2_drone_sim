#include "drone_controller/position/horizontal_position_controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace drone_controller
{
namespace
{

constexpr double kHalfPi = 1.57079632679489661923;
constexpr double kTwoPi = 6.28318530717958647692;
constexpr double kMinimumVectorNorm = 1.0e-12;

bool vector_is_finite(const Eigen::Vector2d & value)
{
  return value.array().isFinite().all();
}

HorizontalPositionControllerResult invalid_result()
{
  HorizontalPositionControllerResult result;
  result.valid = false;
  result.saturated = true;
  return result;
}

}  // namespace

HorizontalPositionController::HorizontalPositionController(
  const HorizontalPositionControllerParameters & parameters)
: parameters_(parameters)
{
  if (!vector_is_finite(parameters_.position_kp) ||
    !vector_is_finite(parameters_.velocity_kd) ||
    !vector_is_finite(parameters_.position_ki) ||
    (parameters_.position_kp.array() < 0.0).any() ||
    (parameters_.velocity_kd.array() < 0.0).any() ||
    (parameters_.position_ki.array() < 0.0).any() ||
    !std::isfinite(parameters_.gravity) || parameters_.gravity <= 0.0 ||
    !std::isfinite(parameters_.max_horizontal_acceleration) ||
    parameters_.max_horizontal_acceleration <= 0.0 ||
    !std::isfinite(parameters_.integral_acceleration_limit) ||
    parameters_.integral_acceleration_limit <= 0.0 ||
    parameters_.integral_acceleration_limit >= parameters_.max_horizontal_acceleration ||
    !std::isfinite(parameters_.anti_windup_gain) ||
    parameters_.anti_windup_gain < 0.0 ||
    !std::isfinite(parameters_.integrator_unload_gain) ||
    parameters_.integrator_unload_gain < 0.0 ||
    !std::isfinite(parameters_.integral_capture_radius) ||
    parameters_.integral_capture_radius <= 0.0 ||
    !std::isfinite(parameters_.max_tilt_angle) ||
    parameters_.max_tilt_angle <= 0.0 || parameters_.max_tilt_angle >= kHalfPi)
  {
    throw std::invalid_argument("Invalid horizontal position controller parameters");
  }
}

HorizontalPositionControllerResult HorizontalPositionController::compute(
  const HorizontalPositionControllerInput & input) const
{
  if (!vector_is_finite(input.desired_position_world) ||
    !vector_is_finite(input.desired_velocity_world) ||
    !vector_is_finite(input.desired_acceleration_world) ||
    !vector_is_finite(input.current_position_world) ||
    !vector_is_finite(input.current_velocity_world) ||
    !std::isfinite(input.desired_yaw))
  {
    return invalid_result();
  }

  // Use extended precision for error subtraction and gain multiplication. This keeps
  // extreme but finite double inputs finite until the vector is direction-preservingly
  // scaled into the configured acceleration limit.
  long double raw_acceleration[2]{};
  for (Eigen::Index axis = 0; axis < 2; ++axis) {
    const long double position_error =
      static_cast<long double>(input.desired_position_world[axis]) -
      static_cast<long double>(input.current_position_world[axis]);
    const long double velocity_error =
      static_cast<long double>(input.desired_velocity_world[axis]) -
      static_cast<long double>(input.current_velocity_world[axis]);
    raw_acceleration[axis] =
      static_cast<long double>(parameters_.position_kp[axis]) * position_error +
      static_cast<long double>(parameters_.velocity_kd[axis]) * velocity_error +
      static_cast<long double>(input.desired_acceleration_world[axis]);
  }

  const long double raw_norm = std::hypot(raw_acceleration[0], raw_acceleration[1]);
  if (!std::isfinite(raw_norm)) {
    return invalid_result();
  }

  const double acceleration_by_tilt =
    parameters_.gravity * std::tan(parameters_.max_tilt_angle);
  const double acceleration_limit =
    std::min(parameters_.max_horizontal_acceleration, acceleration_by_tilt);
  if (!std::isfinite(acceleration_limit) || acceleration_limit <= 0.0) {
    return invalid_result();
  }

  HorizontalPositionControllerResult result;
  result.proportional_acceleration_world =
    parameters_.position_kp.cwiseProduct(
    input.desired_position_world - input.current_position_world);
  result.derivative_acceleration_world =
    parameters_.velocity_kd.cwiseProduct(
    input.desired_velocity_world - input.current_velocity_world);
  result.feedforward_acceleration_world = input.desired_acceleration_world;
  result.raw_acceleration_world =
    result.proportional_acceleration_world + result.derivative_acceleration_world +
    result.feedforward_acceleration_world;
  long double scale = 1.0L;
  if (raw_norm > static_cast<long double>(acceleration_limit)) {
    scale = static_cast<long double>(acceleration_limit) / raw_norm;
    result.saturated = true;
  }
  result.desired_acceleration_world = Eigen::Vector2d(
    static_cast<double>(raw_acceleration[0] * scale),
    static_cast<double>(raw_acceleration[1] * scale));
  if (!vector_is_finite(result.desired_acceleration_world)) {
    return invalid_result();
  }

  // Desired thrust direction in map/ENU. With thrust along body +z, its horizontal
  // projection must point in the desired world-frame acceleration direction.
  Eigen::Vector3d body_z_desired(
    result.desired_acceleration_world.x(), result.desired_acceleration_world.y(),
    parameters_.gravity);
  const double body_z_norm = body_z_desired.norm();
  if (!std::isfinite(body_z_norm) || body_z_norm < kMinimumVectorNorm) {
    return invalid_result();
  }
  body_z_desired /= body_z_norm;

  const double yaw = std::remainder(input.desired_yaw, kTwoPi);
  if (!std::isfinite(yaw)) {
    return invalid_result();
  }
  const Eigen::Vector3d heading(std::cos(yaw), std::sin(yaw), 0.0);
  Eigen::Vector3d body_y_desired = body_z_desired.cross(heading);
  const double body_y_norm = body_y_desired.norm();
  if (!std::isfinite(body_y_norm) || body_y_norm < kMinimumVectorNorm) {
    return invalid_result();
  }
  body_y_desired /= body_y_norm;
  Eigen::Vector3d body_x_desired = body_y_desired.cross(body_z_desired);
  const double body_x_norm = body_x_desired.norm();
  if (!std::isfinite(body_x_norm) || body_x_norm < kMinimumVectorNorm) {
    return invalid_result();
  }
  body_x_desired /= body_x_norm;

  Eigen::Matrix3d rotation_body_to_world;
  rotation_body_to_world.col(0) = body_x_desired;
  rotation_body_to_world.col(1) = body_y_desired;
  rotation_body_to_world.col(2) = body_z_desired;
  if (!rotation_body_to_world.array().isFinite().all() ||
    !std::isfinite(rotation_body_to_world.determinant()) ||
    rotation_body_to_world.determinant() <= 0.0)
  {
    return invalid_result();
  }

  result.desired_orientation_body_to_world = Eigen::Quaterniond(rotation_body_to_world);
  const double quaternion_norm = result.desired_orientation_body_to_world.norm();
  if (!std::isfinite(quaternion_norm) || quaternion_norm < kMinimumVectorNorm) {
    return invalid_result();
  }
  result.desired_orientation_body_to_world.normalize();

  // ZYX roll/pitch extraction, used for diagnostics and unit-level sign checks.
  result.desired_roll = std::atan2(
    rotation_body_to_world(2, 1), rotation_body_to_world(2, 2));
  result.desired_pitch = std::atan2(
    -rotation_body_to_world(2, 0),
    std::hypot(rotation_body_to_world(2, 1), rotation_body_to_world(2, 2)));
  if (!std::isfinite(result.desired_roll) || !std::isfinite(result.desired_pitch) ||
    !result.desired_orientation_body_to_world.coeffs().array().isFinite().all())
  {
    return invalid_result();
  }
  return result;
}

HorizontalPositionControllerResult HorizontalPositionController::compute(
  const HorizontalPositionControllerInput & input, const double dt,
  const bool integrator_enabled)
{
  if (!std::isfinite(dt) || dt <= 0.0) {
    return invalid_result();
  }

  if (!parameters_.enable_integral) {
    return compute(input);
  }

  HorizontalPositionControllerInput provisional_input = input;
  provisional_input.desired_acceleration_world += integral_acceleration_world_;
  HorizontalPositionControllerResult provisional =
    static_cast<const HorizontalPositionController &>(*this).compute(provisional_input);
  if (!provisional.valid) {
    return provisional;
  }

  const Eigen::Vector2d position_error =
    input.desired_position_world - input.current_position_world;
  const bool integrator_unloading_active =
    integrator_enabled && unwind_integrator_if_opposing_error(position_error, dt);

  // Re-evaluate saturation after deterministic unloading because the integral
  // contribution used by the provisional result may have changed.
  if (integrator_unloading_active) {
    provisional_input.desired_acceleration_world =
      input.desired_acceleration_world + integral_acceleration_world_;
    provisional = static_cast<const HorizontalPositionController &>(*this).compute(
      provisional_input);
    if (!provisional.valid) {
      return provisional;
    }
  }
  const bool inside_capture_radius =
    position_error.norm() <= parameters_.integral_capture_radius;
  const bool integrate_position_error =
    integrator_enabled && !integrator_unloading_active &&
    inside_capture_radius && !provisional.saturated;
  const Eigen::Vector2d saturation_residual =
    provisional.desired_acceleration_world - provisional.raw_acceleration_world;
  const bool saturation_anti_windup_active =
    integrator_enabled && provisional.saturated &&
    parameters_.anti_windup_gain > 0.0 &&
    saturation_residual.squaredNorm() > 0.0;
  Eigen::Vector2d candidate_integral = integral_acceleration_world_;
  if (integrator_enabled) {
    Eigen::Vector2d integral_derivative = Eigen::Vector2d::Zero();
    if (integrate_position_error) {
      integral_derivative += parameters_.position_ki.cwiseProduct(position_error);
    }
    if (saturation_anti_windup_active) {
      integral_derivative += parameters_.anti_windup_gain * saturation_residual;
    }
    candidate_integral += integral_derivative * dt;
    const double candidate_norm = candidate_integral.norm();
    if (!vector_is_finite(candidate_integral) || !std::isfinite(candidate_norm)) {
      return invalid_result();
    }
    if (candidate_norm > parameters_.integral_acceleration_limit) {
      candidate_integral *= parameters_.integral_acceleration_limit / candidate_norm;
    }
  }

  HorizontalPositionControllerInput final_input = input;
  final_input.desired_acceleration_world += candidate_integral;
  HorizontalPositionControllerResult result =
    static_cast<const HorizontalPositionController &>(*this).compute(final_input);
  if (!result.valid) {
    return result;
  }
  integral_acceleration_world_ = candidate_integral;
  result.integral_acceleration_world = integral_acceleration_world_;
  result.feedforward_acceleration_world = input.desired_acceleration_world;
  result.raw_acceleration_world =
    result.proportional_acceleration_world + result.derivative_acceleration_world +
    result.integral_acceleration_world + result.feedforward_acceleration_world;
  result.integral_enabled = true;
  result.integral_frozen = !integrate_position_error;
  result.saturation_backcalc_active = saturation_anti_windup_active;
  result.integrator_unloading_active = integrator_unloading_active;
  result.anti_windup_active =
    saturation_anti_windup_active || integrator_unloading_active;
  return result;
}

void HorizontalPositionController::reset_integrator()
{
  integral_acceleration_world_.setZero();
}

const Eigen::Vector2d & HorizontalPositionController::integral_acceleration_world() const
{
  return integral_acceleration_world_;
}

bool HorizontalPositionController::unwind_integrator_if_opposing_error(
  const Eigen::Vector2d & position_error, const double dt)
{
  if (!parameters_.enable_integral || !vector_is_finite(position_error) ||
    !std::isfinite(dt) || dt <= 0.0 || parameters_.integrator_unload_gain <= 0.0 ||
    integral_acceleration_world_.squaredNorm() <=
    kMinimumVectorNorm * kMinimumVectorNorm ||
    position_error.dot(integral_acceleration_world_) >= 0.0)
  {
    return false;
  }
  const double scale = std::max(0.0, 1.0 - parameters_.integrator_unload_gain * dt);
  integral_acceleration_world_ *= scale;
  return true;
}

}  // namespace drone_controller
