#include "drone_controller/hover/hover_controller.hpp"

#include <cmath>

namespace drone_controller
{
namespace
{

constexpr double kMinimumQuaternionScale = 1.0e-12;

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

HoverControllerResult invalid_result(HoverControllerResult result)
{
  result.motor_rpm.fill(0.0);
  result.valid = false;
  result.saturated = true;
  return result;
}

}  // namespace

HoverController::HoverController(const HoverControllerParameters & parameters)
: altitude_controller_(parameters.altitude),
  attitude_controller_(parameters.attitude),
  motor_mixer_(parameters.mixer)
{
}

HoverControllerResult HoverController::compute(const HoverControllerInput & input) const
{
  HoverControllerResult result;

  AltitudeControllerInput altitude_input;
  altitude_input.desired_altitude = input.desired_altitude;
  altitude_input.desired_vertical_velocity = input.desired_vertical_velocity;
  altitude_input.desired_vertical_acceleration = input.desired_vertical_acceleration;
  altitude_input.current_altitude = input.current_altitude;
  altitude_input.current_vertical_velocity = input.current_vertical_velocity_world;
  altitude_input.current_orientation_body_to_world = input.current_orientation_body_to_world;
  const AltitudeControllerResult altitude_result = altitude_controller_.compute(altitude_input);
  result.collective_thrust = altitude_result.collective_thrust;
  result.altitude_saturated = altitude_result.saturated;
  result.saturated = altitude_result.saturated;
  if (!altitude_result.valid) {
    return invalid_result(result);
  }

  AttitudeControllerInput attitude_input;
  attitude_input.desired_orientation_body_to_world =
    input.desired_orientation_body_to_world;
  attitude_input.current_orientation_body_to_world =
    input.current_orientation_body_to_world;
  attitude_input.desired_angular_velocity_body = input.desired_angular_velocity_body;
  attitude_input.current_angular_velocity_body = input.current_angular_velocity_body;
  const AttitudeControllerResult attitude_result = attitude_controller_.compute(attitude_input);
  result.torque_body = attitude_result.torque_body;
  result.attitude_saturated = attitude_result.saturated;
  result.saturated = result.saturated || attitude_result.saturated;
  if (!attitude_result.valid) {
    return invalid_result(result);
  }

  const MixerResult mixer_result = motor_mixer_.mix({
      result.collective_thrust, result.torque_body.x(), result.torque_body.y(),
      result.torque_body.z()});
  result.mixer_saturated = mixer_result.saturated;
  result.saturated = result.saturated || mixer_result.saturated;
  if (!mixer_result.valid) {
    return invalid_result(result);
  }
  result.motor_rpm = mixer_result.motor_rpm;
  for (const double rpm : result.motor_rpm) {
    if (!std::isfinite(rpm)) {
      return invalid_result(result);
    }
  }
  return result;
}

bool world_vertical_velocity_from_body(
  const Eigen::Quaterniond & orientation_body_to_world,
  const Eigen::Vector3d & velocity_body,
  double & vertical_velocity_world)
{
  if (!velocity_body.array().isFinite().all()) {
    return false;
  }
  Eigen::Quaterniond orientation = orientation_body_to_world;
  if (!normalize_quaternion(orientation)) {
    return false;
  }
  const Eigen::Vector3d velocity_world = orientation * velocity_body;
  if (!velocity_world.array().isFinite().all()) {
    return false;
  }
  vertical_velocity_world = velocity_world.z();
  return std::isfinite(vertical_velocity_world);
}

bool level_orientation_from_goal_yaw(
  const Eigen::Quaterniond & goal_orientation_body_to_world,
  Eigen::Quaterniond & level_orientation_body_to_world)
{
  Eigen::Quaterniond goal = goal_orientation_body_to_world;
  if (!normalize_quaternion(goal)) {
    return false;
  }
  const Eigen::Matrix3d rotation = goal.toRotationMatrix();
  const double yaw = std::atan2(rotation(1, 0), rotation(0, 0));
  if (!std::isfinite(yaw)) {
    return false;
  }
  level_orientation_body_to_world =
    Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
  return level_orientation_body_to_world.coeffs().array().isFinite().all();
}

}  // namespace drone_controller
