#include "drone_dynamics/quadrotor_model.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace drone_dynamics
{
namespace
{

constexpr double kRpmToRadiansPerSecond = 0.10471975511965977;

bool is_positive_finite(const double value)
{
  return std::isfinite(value) && value > 0.0;
}

}  // namespace

QuadrotorModel::QuadrotorModel(const QuadrotorParameters & parameters)
: parameters_(parameters)
{
  validate_parameters();
  reset();
}

void QuadrotorModel::reset()
{
  state_ = QuadrotorState{};
  commanded_motor_angular_velocity_rad_s_.fill(0.0);
  body_wrench_ = BodyWrench{};
  linear_acceleration_world_ = Eigen::Vector3d(0.0, 0.0, -parameters_.gravity);
}

void QuadrotorModel::set_motor_rpm_command(const MotorValues & motor_rpm)
{
  for (std::size_t index = 0; index < motor_rpm.size(); ++index) {
    const double finite_rpm = std::isfinite(motor_rpm[index]) ? motor_rpm[index] : 0.0;
    const double clamped_rpm =
      std::clamp(finite_rpm, parameters_.min_rpm, parameters_.max_rpm);
    commanded_motor_angular_velocity_rad_s_[index] = rpm_to_rad_s(clamped_rpm);
  }
}

void QuadrotorModel::step(const double dt)
{
  if (!is_positive_finite(dt)) {
    throw std::invalid_argument("Dynamics time step must be finite and greater than zero");
  }

  update_motor_response(dt);
  body_wrench_ = calculate_body_wrench();

  const Eigen::Vector3d thrust_body(0.0, 0.0, body_wrench_.thrust);
  const Eigen::Vector3d gravity_world(0.0, 0.0, -parameters_.gravity);
  linear_acceleration_world_ =
    state_.orientation_body_to_world * thrust_body / parameters_.mass + gravity_world;

  const Eigen::Vector3d angular_momentum =
    parameters_.inertia.asDiagonal() * state_.angular_velocity_body;
  const Eigen::Vector3d angular_acceleration = parameters_.inertia.cwiseInverse().asDiagonal() *
    (body_wrench_.torque - state_.angular_velocity_body.cross(angular_momentum));

  state_.velocity_world += linear_acceleration_world_ * dt;
  state_.position_world += state_.velocity_world * dt;
  state_.angular_velocity_body += angular_acceleration * dt;

  const double angular_speed = state_.angular_velocity_body.norm();
  if (angular_speed > 1.0e-12) {
    const Eigen::Quaterniond incremental_rotation(
      Eigen::AngleAxisd(angular_speed * dt, state_.angular_velocity_body / angular_speed));
    state_.orientation_body_to_world =
      state_.orientation_body_to_world * incremental_rotation;
  }
  state_.orientation_body_to_world.normalize();
}

const QuadrotorParameters & QuadrotorModel::parameters() const
{
  return parameters_;
}

const QuadrotorState & QuadrotorModel::state() const
{
  return state_;
}

const BodyWrench & QuadrotorModel::body_wrench() const
{
  return body_wrench_;
}

const Eigen::Vector3d & QuadrotorModel::linear_acceleration_world() const
{
  return linear_acceleration_world_;
}

Eigen::Vector3d QuadrotorModel::specific_force_body() const
{
  return Eigen::Vector3d(0.0, 0.0, body_wrench_.thrust / parameters_.mass);
}

double QuadrotorModel::rpm_to_rad_s(const double rpm)
{
  return rpm * kRpmToRadiansPerSecond;
}

void QuadrotorModel::validate_parameters() const
{
  if (!is_positive_finite(parameters_.mass)) {
    throw std::invalid_argument("mass must be finite and greater than zero");
  }
  if (!(parameters_.inertia.array().isFinite().all()) ||
    !(parameters_.inertia.array() > 0.0).all())
  {
    throw std::invalid_argument("all principal inertia values must be finite and greater than zero");
  }
  if (!is_positive_finite(parameters_.arm_length)) {
    throw std::invalid_argument("arm_length must be finite and greater than zero");
  }
  if (!is_positive_finite(parameters_.thrust_coefficient)) {
    throw std::invalid_argument("thrust_coefficient must be finite and greater than zero");
  }
  if (!is_positive_finite(parameters_.drag_torque_coefficient)) {
    throw std::invalid_argument("drag_torque_coefficient must be finite and greater than zero");
  }
  if (!is_positive_finite(parameters_.motor_time_constant)) {
    throw std::invalid_argument("motor_time_constant must be finite and greater than zero");
  }
  if (!std::isfinite(parameters_.min_rpm) || parameters_.min_rpm < 0.0 ||
    !std::isfinite(parameters_.max_rpm) || parameters_.max_rpm <= parameters_.min_rpm)
  {
    throw std::invalid_argument("RPM limits must be finite and satisfy 0 <= min_rpm < max_rpm");
  }
  if (!is_positive_finite(parameters_.gravity)) {
    throw std::invalid_argument("gravity must be finite and greater than zero");
  }
}

void QuadrotorModel::update_motor_response(const double dt)
{
  const double response_fraction = 1.0 - std::exp(-dt / parameters_.motor_time_constant);
  for (std::size_t index = 0; index < state_.motor_angular_velocity_rad_s.size(); ++index) {
    state_.motor_angular_velocity_rad_s[index] += response_fraction *
      (commanded_motor_angular_velocity_rad_s_[index] -
      state_.motor_angular_velocity_rad_s[index]);
  }
}

BodyWrench QuadrotorModel::calculate_body_wrench() const
{
  std::array<double, 4> thrust{};
  std::array<double, 4> reaction_torque{};
  for (std::size_t index = 0; index < thrust.size(); ++index) {
    const double squared_speed =
      state_.motor_angular_velocity_rad_s[index] *
      state_.motor_angular_velocity_rad_s[index];
    thrust[index] = parameters_.thrust_coefficient * squared_speed;
    reaction_torque[index] = parameters_.drag_torque_coefficient * squared_speed;
  }

  const double moment_arm = parameters_.arm_length / std::sqrt(2.0);
  BodyWrench wrench;
  wrench.thrust = thrust[0] + thrust[1] + thrust[2] + thrust[3];
  wrench.torque.x() = moment_arm * (thrust[0] + thrust[1] - thrust[2] - thrust[3]);
  wrench.torque.y() = moment_arm * (-thrust[0] + thrust[1] + thrust[2] - thrust[3]);
  wrench.torque.z() =
    -reaction_torque[0] + reaction_torque[1] - reaction_torque[2] + reaction_torque[3];
  return wrench;
}

}  // namespace drone_dynamics
