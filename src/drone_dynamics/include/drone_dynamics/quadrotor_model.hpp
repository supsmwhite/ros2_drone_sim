#ifndef DRONE_DYNAMICS__QUADROTOR_MODEL_HPP_
#define DRONE_DYNAMICS__QUADROTOR_MODEL_HPP_

#include <array>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace drone_dynamics
{

struct QuadrotorParameters
{
  double mass{1.0};
  Eigen::Vector3d inertia{0.02, 0.02, 0.04};
  double arm_length{0.20};
  double thrust_coefficient{1.91e-6};
  double drag_torque_coefficient{2.60e-7};
  double motor_time_constant{0.05};
  double min_rpm{0.0};
  double max_rpm{20000.0};
  double gravity{9.80665};
};

struct QuadrotorState
{
  Eigen::Vector3d position_world{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity_world{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond orientation_body_to_world{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d angular_velocity_body{Eigen::Vector3d::Zero()};
  std::array<double, 4> motor_angular_velocity_rad_s{};
};

struct BodyWrench
{
  double thrust{0.0};
  Eigen::Vector3d torque{Eigen::Vector3d::Zero()};
};

class QuadrotorModel
{
public:
  using MotorValues = std::array<double, 4>;

  explicit QuadrotorModel(const QuadrotorParameters & parameters);

  void reset();
  void set_motor_rpm_command(const MotorValues & motor_rpm);
  void step(double dt);

  const QuadrotorParameters & parameters() const;
  const QuadrotorState & state() const;
  const BodyWrench & body_wrench() const;
  const Eigen::Vector3d & linear_acceleration_world() const;
  Eigen::Vector3d specific_force_body() const;

private:
  static double rpm_to_rad_s(double rpm);
  void validate_parameters() const;
  void update_motor_response(double dt);
  BodyWrench calculate_body_wrench() const;

  QuadrotorParameters parameters_;
  QuadrotorState state_;
  MotorValues commanded_motor_angular_velocity_rad_s_{};
  BodyWrench body_wrench_;
  Eigen::Vector3d linear_acceleration_world_{Eigen::Vector3d::Zero()};
};

}  // namespace drone_dynamics

#endif  // DRONE_DYNAMICS__QUADROTOR_MODEL_HPP_
