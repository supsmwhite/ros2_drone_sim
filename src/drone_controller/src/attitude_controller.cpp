#include "drone_controller/attitude/attitude_controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace drone_controller
{
namespace
{

constexpr double kMinimumQuaternionScale = 1.0e-12;
constexpr double kHalfTurnScalarEpsilon = 1.0e-12;

bool vector_is_finite(const Eigen::Vector3d & value)
{
  return value.array().isFinite().all();
}

bool normalize_quaternion(Eigen::Quaterniond & quaternion)
{
  if (!quaternion.coeffs().array().isFinite().all()) {
    return false;
  }

  // Scale first so even very large finite coefficients do not overflow while
  // computing the norm. Eigen stores quaternion coefficients as [x,y,z,w].
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

AttitudeControllerResult invalid_result()
{
  AttitudeControllerResult result;
  result.valid = false;
  return result;
}

}  // namespace

AttitudeController::AttitudeController(const AttitudeControllerParameters & parameters)
: parameters_(parameters)
{
  if (!vector_is_finite(parameters_.attitude_kp) ||
    !vector_is_finite(parameters_.angular_rate_kd) ||
    !vector_is_finite(parameters_.max_torque) ||
    (parameters_.attitude_kp.array() < 0.0).any() ||
    (parameters_.angular_rate_kd.array() < 0.0).any() ||
    (parameters_.max_torque.array() <= 0.0).any())
  {
    throw std::invalid_argument(
            "Attitude gains must be finite and nonnegative; torque limits must be finite and positive");
  }
}

AttitudeControllerResult AttitudeController::compute(const AttitudeControllerInput & input) const
{
  if (!vector_is_finite(input.desired_angular_velocity_body) ||
    !vector_is_finite(input.current_angular_velocity_body))
  {
    return invalid_result();
  }

  Eigen::Quaterniond desired = input.desired_orientation_body_to_world;
  Eigen::Quaterniond current = input.current_orientation_body_to_world;
  if (!normalize_quaternion(desired) || !normalize_quaternion(current)) {
    return invalid_result();
  }

  // q_error maps the desired body orientation relative to the current body.
  // Its vector part therefore has the same base_link roll/pitch/yaw signs used
  // by the dynamics torque vector.
  Eigen::Quaterniond error = current.conjugate() * desired;
  bool negate_error = error.w() < -kHalfTurnScalarEpsilon;
  if (std::abs(error.w()) <= kHalfTurnScalarEpsilon) {
    Eigen::Index dominant_axis = 0;
    error.vec().cwiseAbs().maxCoeff(&dominant_axis);
    negate_error = error.vec()[dominant_axis] < 0.0;
  }
  if (negate_error) {
    error.coeffs() *= -1.0;
  }
  const Eigen::Vector3d attitude_error = 2.0 * error.vec();

  // Positive current rate with zero desired rate produces negative damping.
  Eigen::Vector3d torque =
    parameters_.attitude_kp.cwiseProduct(attitude_error) +
    parameters_.angular_rate_kd.cwiseProduct(
    input.desired_angular_velocity_body - input.current_angular_velocity_body);
  if (!vector_is_finite(torque)) {
    return invalid_result();
  }

  AttitudeControllerResult result;
  for (Eigen::Index axis = 0; axis < torque.size(); ++axis) {
    const double limited = std::clamp(
      torque[axis], -parameters_.max_torque[axis], parameters_.max_torque[axis]);
    if (limited != torque[axis]) {
      result.saturated = true;
    }
    result.torque_body[axis] = limited;
  }
  return result;
}

}  // namespace drone_controller
