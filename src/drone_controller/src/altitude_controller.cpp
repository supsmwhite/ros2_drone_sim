#include "drone_controller/altitude/altitude_controller.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

namespace drone_controller
{
namespace
{

constexpr double kMinimumQuaternionScale = 1.0e-12;

bool all_finite(const std::array<double, 5> & values)
{
  return std::all_of(values.begin(), values.end(), [](const double value) {
    return std::isfinite(value);
  });
}

bool normalize_quaternion(Eigen::Quaterniond & quaternion)
{
  if (!quaternion.coeffs().array().isFinite().all()) {
    return false;
  }
  const double scale = quaternion.coeffs().cwiseAbs().maxCoeff();
  if (!std::isfinite(scale) || scale < kMinimumQuaternionScale) {
    return false;
  }
  quaternion.coeffs() /= scale;
  const double norm = quaternion.norm();
  if (!std::isfinite(norm) || norm < kMinimumQuaternionScale) {
    return false;
  }
  quaternion.coeffs() /= norm;
  return quaternion.coeffs().array().isFinite().all();
}

AltitudeControllerResult invalid_result()
{
  AltitudeControllerResult result;
  result.valid = false;
  result.saturated = true;
  return result;
}

}  // namespace

AltitudeController::AltitudeController(const AltitudeControllerParameters & parameters)
: parameters_(parameters)
{
  if (!std::isfinite(parameters_.mass) || parameters_.mass <= 0.0 ||
    !std::isfinite(parameters_.gravity) || parameters_.gravity <= 0.0 ||
    !std::isfinite(parameters_.altitude_kp) || parameters_.altitude_kp < 0.0 ||
    !std::isfinite(parameters_.vertical_velocity_kd) ||
    parameters_.vertical_velocity_kd < 0.0 ||
    !std::isfinite(parameters_.max_upward_acceleration) ||
    parameters_.max_upward_acceleration < 0.0 ||
    !std::isfinite(parameters_.max_downward_acceleration) ||
    parameters_.max_downward_acceleration < 0.0 ||
    !std::isfinite(parameters_.min_collective_thrust) ||
    !std::isfinite(parameters_.max_collective_thrust) ||
    parameters_.min_collective_thrust < 0.0 ||
    parameters_.min_collective_thrust >= parameters_.max_collective_thrust ||
    !std::isfinite(parameters_.min_tilt_cosine) ||
    parameters_.min_tilt_cosine <= 0.0 || parameters_.min_tilt_cosine > 1.0)
  {
    throw std::invalid_argument("Invalid altitude controller parameters");
  }
}

AltitudeControllerResult AltitudeController::compute(const AltitudeControllerInput & input) const
{
  if (!all_finite({input.desired_altitude, input.desired_vertical_velocity,
      input.desired_vertical_acceleration, input.current_altitude,
      input.current_vertical_velocity}))
  {
    return invalid_result();
  }

  Eigen::Quaterniond orientation = input.current_orientation_body_to_world;
  if (!normalize_quaternion(orientation)) {
    return invalid_result();
  }

  const double position_error = input.desired_altitude - input.current_altitude;
  const double velocity_error =
    input.desired_vertical_velocity - input.current_vertical_velocity;
  const double raw_acceleration =
    parameters_.altitude_kp * position_error +
    parameters_.vertical_velocity_kd * velocity_error +
    input.desired_vertical_acceleration;
  if (!std::isfinite(position_error) || !std::isfinite(velocity_error) ||
    !std::isfinite(raw_acceleration))
  {
    return invalid_result();
  }

  AltitudeControllerResult result;
  result.commanded_vertical_acceleration = std::clamp(
    raw_acceleration, -parameters_.max_downward_acceleration,
    parameters_.max_upward_acceleration);
  if (result.commanded_vertical_acceleration != raw_acceleration) {
    result.saturated = true;
  }

  const double vertical_force =
    parameters_.mass * (parameters_.gravity + result.commanded_vertical_acceleration);
  const double cos_tilt = (orientation * Eigen::Vector3d::UnitZ()).z();
  if (!std::isfinite(vertical_force) || !std::isfinite(cos_tilt)) {
    return invalid_result();
  }
  if (cos_tilt <= 0.0) {
    return invalid_result();
  }

  double safe_cos_tilt = cos_tilt;
  if (safe_cos_tilt < parameters_.min_tilt_cosine) {
    safe_cos_tilt = parameters_.min_tilt_cosine;
    result.saturated = true;
  }
  const double raw_thrust = vertical_force / safe_cos_tilt;
  if (!std::isfinite(raw_thrust)) {
    return invalid_result();
  }

  result.collective_thrust = std::clamp(
    raw_thrust, parameters_.min_collective_thrust,
    parameters_.max_collective_thrust);
  if (result.collective_thrust != raw_thrust) {
    result.saturated = true;
  }
  return result;
}

}  // namespace drone_controller
