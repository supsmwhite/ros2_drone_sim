#include "drone_controller/mixer/motor_mixer.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace drone_controller
{
namespace
{

constexpr double kTwoPi = 6.28318530717958647692;

bool is_positive_finite(const double value)
{
  return std::isfinite(value) && value > 0.0;
}

MixerResult invalid_result()
{
  MixerResult result;
  result.valid = false;
  result.saturated = true;
  return result;
}

}  // namespace

MotorMixer::MotorMixer(const MixerParameters & parameters)
: parameters_(parameters)
{
  if (!is_positive_finite(parameters_.arm_length)) {
    throw std::invalid_argument("arm_length must be finite and greater than zero");
  }
  if (!is_positive_finite(parameters_.thrust_coefficient)) {
    throw std::invalid_argument("thrust_coefficient must be finite and greater than zero");
  }
  if (!is_positive_finite(parameters_.drag_torque_coefficient)) {
    throw std::invalid_argument(
            "drag_torque_coefficient must be finite and greater than zero");
  }
  if (!std::isfinite(parameters_.min_rpm) || !std::isfinite(parameters_.max_rpm) ||
    parameters_.min_rpm < 0.0 || parameters_.min_rpm >= parameters_.max_rpm)
  {
    throw std::invalid_argument("RPM limits must satisfy 0 <= min_rpm < max_rpm");
  }
}

MixerResult MotorMixer::mix(const WrenchCommand & command) const
{
  MixerResult result;
  if (!std::isfinite(command.thrust) || !std::isfinite(command.roll_torque) ||
    !std::isfinite(command.pitch_torque) || !std::isfinite(command.yaw_torque))
  {
    return invalid_result();
  }

  double thrust = command.thrust;
  if (thrust < 0.0) {
    thrust = 0.0;
    result.saturated = true;
  }

  // Invert the X-layout wrench matrix in thrust space. Here a=L/sqrt(2)
  // and b=kM/kF. Motor order and signs exactly match QuadrotorModel.
  const double a = parameters_.arm_length / std::sqrt(2.0);
  const double b = parameters_.drag_torque_coefficient / parameters_.thrust_coefficient;
  const double roll_term = command.roll_torque / a;
  const double pitch_term = command.pitch_torque / a;
  const double yaw_term = command.yaw_torque / b;
  if (!std::isfinite(roll_term) || !std::isfinite(pitch_term) ||
    !std::isfinite(yaw_term))
  {
    return invalid_result();
  }
  std::array<double, 4> motor_thrust{
    0.25 * (thrust + roll_term - pitch_term - yaw_term),
    0.25 * (thrust + roll_term + pitch_term + yaw_term),
    0.25 * (thrust - roll_term + pitch_term - yaw_term),
    0.25 * (thrust - roll_term - pitch_term + yaw_term)};

  for (std::size_t index = 0; index < motor_thrust.size(); ++index) {
    if (!std::isfinite(motor_thrust[index])) {
      return invalid_result();
    }
    if (motor_thrust[index] < 0.0) {
      motor_thrust[index] = 0.0;
      result.saturated = true;
    }

    const double omega = std::sqrt(motor_thrust[index] / parameters_.thrust_coefficient);
    const double rpm = omega * 60.0 / kTwoPi;
    if (!std::isfinite(omega) || !std::isfinite(rpm)) {
      return invalid_result();
    }
    const double clamped_rpm = std::clamp(rpm, parameters_.min_rpm, parameters_.max_rpm);
    if (clamped_rpm != rpm) {
      result.saturated = true;
    }
    result.motor_rpm[index] = clamped_rpm;
  }
  return result;
}

}  // namespace drone_controller
