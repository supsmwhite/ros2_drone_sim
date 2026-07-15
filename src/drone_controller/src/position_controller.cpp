#include "drone_controller/position/position_controller.hpp"

#include <cmath>
#include <stdexcept>

namespace drone_controller
{
namespace
{

PositionControllerResult invalid_result(PositionControllerResult result)
{
  result.motor_rpm.fill(0.0);
  result.valid = false;
  result.saturated = true;
  return result;
}

}  // namespace

PositionController::PositionController(const PositionControllerParameters & parameters)
: horizontal_controller_(parameters.horizontal), hover_controller_(parameters.hover)
{
  if (parameters.horizontal.gravity != parameters.hover.altitude.gravity) {
    throw std::invalid_argument(
            "Horizontal and altitude controllers must use the same gravity");
  }
}

PositionControllerResult PositionController::compute(const PositionControllerInput & input) const
{
  PositionControllerResult result;

  HorizontalPositionControllerInput horizontal_input;
  horizontal_input.desired_position_world = input.desired_position_world.head<2>();
  horizontal_input.desired_velocity_world = input.desired_velocity_world.head<2>();
  horizontal_input.desired_acceleration_world = input.desired_acceleration_world.head<2>();
  horizontal_input.current_position_world = input.current_position_world.head<2>();
  horizontal_input.current_velocity_world = input.current_velocity_world.head<2>();
  horizontal_input.desired_yaw = input.desired_yaw;
  const HorizontalPositionControllerResult horizontal_result =
    horizontal_controller_.compute(horizontal_input);
  result.desired_horizontal_acceleration_world =
    horizontal_result.desired_acceleration_world;
  result.desired_roll = horizontal_result.desired_roll;
  result.desired_pitch = horizontal_result.desired_pitch;
  result.desired_orientation_body_to_world =
    horizontal_result.desired_orientation_body_to_world;
  result.horizontal_saturated = horizontal_result.saturated;
  result.saturated = horizontal_result.saturated;
  if (!horizontal_result.valid) {
    return invalid_result(result);
  }

  HoverControllerInput hover_input;
  hover_input.desired_altitude = input.desired_position_world.z();
  hover_input.desired_vertical_velocity = input.desired_velocity_world.z();
  hover_input.desired_vertical_acceleration = input.desired_acceleration_world.z();
  hover_input.desired_orientation_body_to_world =
    horizontal_result.desired_orientation_body_to_world;
  hover_input.current_altitude = input.current_position_world.z();
  hover_input.current_vertical_velocity_world = input.current_velocity_world.z();
  hover_input.current_orientation_body_to_world =
    input.current_orientation_body_to_world;
  hover_input.desired_angular_velocity_body = input.desired_angular_velocity_body;
  hover_input.current_angular_velocity_body = input.current_angular_velocity_body;
  const HoverControllerResult hover_result = hover_controller_.compute(hover_input);
  result.collective_thrust = hover_result.collective_thrust;
  result.torque_body = hover_result.torque_body;
  result.altitude_saturated = hover_result.altitude_saturated;
  result.attitude_saturated = hover_result.attitude_saturated;
  result.mixer_saturated = hover_result.mixer_saturated;
  result.saturated = result.saturated || hover_result.saturated;
  if (!hover_result.valid) {
    return invalid_result(result);
  }

  result.motor_rpm = hover_result.motor_rpm;
  for (const double rpm : result.motor_rpm) {
    if (!std::isfinite(rpm)) {
      return invalid_result(result);
    }
  }
  return result;
}

}  // namespace drone_controller
