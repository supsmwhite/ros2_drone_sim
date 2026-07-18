#ifndef DRONE_CONTROLLER__POSITION__POSITION_CONTROLLER_HPP_
#define DRONE_CONTROLLER__POSITION__POSITION_CONTROLLER_HPP_

#include <array>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "drone_controller/hover/hover_controller.hpp"
#include "drone_controller/position/horizontal_position_controller.hpp"

namespace drone_controller
{

struct PositionControllerParameters
{
  HorizontalPositionControllerParameters horizontal;
  HoverControllerParameters hover;
};

struct PositionControllerInput
{
  Eigen::Vector3d desired_position_world{Eigen::Vector3d::Zero()};
  Eigen::Vector3d desired_velocity_world{Eigen::Vector3d::Zero()};
  Eigen::Vector3d desired_acceleration_world{Eigen::Vector3d::Zero()};
  double desired_yaw{0.0};

  Eigen::Vector3d current_position_world{Eigen::Vector3d::Zero()};
  Eigen::Vector3d current_velocity_world{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond current_orientation_body_to_world{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d desired_angular_velocity_body{Eigen::Vector3d::Zero()};
  Eigen::Vector3d current_angular_velocity_body{Eigen::Vector3d::Zero()};
};

struct PositionControllerResult
{
  std::array<double, 4> motor_rpm{};
  std::array<double, 4> unclipped_motor_rpm{};
  Eigen::Vector2d desired_horizontal_acceleration_world{Eigen::Vector2d::Zero()};
  Eigen::Vector2d horizontal_proportional_acceleration_world{Eigen::Vector2d::Zero()};
  Eigen::Vector2d horizontal_derivative_acceleration_world{Eigen::Vector2d::Zero()};
  Eigen::Vector2d horizontal_integral_acceleration_world{Eigen::Vector2d::Zero()};
  Eigen::Vector2d horizontal_feedforward_acceleration_world{Eigen::Vector2d::Zero()};
  Eigen::Vector2d horizontal_raw_acceleration_world{Eigen::Vector2d::Zero()};
  double desired_roll{0.0};
  double desired_pitch{0.0};
  Eigen::Quaterniond desired_orientation_body_to_world{Eigen::Quaterniond::Identity()};
  double collective_thrust{0.0};
  Eigen::Vector3d torque_body{Eigen::Vector3d::Zero()};
  bool valid{true};
  bool saturated{false};
  bool horizontal_saturated{false};
  bool altitude_saturated{false};
  bool attitude_saturated{false};
  bool mixer_saturated{false};
  bool horizontal_integral_enabled{false};
  bool horizontal_integral_frozen{false};
  bool horizontal_saturation_backcalc_active{false};
  bool horizontal_integrator_unloading_active{false};
  bool horizontal_anti_windup_active{false};
};

// ROS-independent composition of horizontal position control and the existing
// altitude/attitude/mixer hover chain.
class PositionController
{
public:
  explicit PositionController(
    const PositionControllerParameters & parameters = PositionControllerParameters{});

  PositionControllerResult compute(const PositionControllerInput & input) const;
  PositionControllerResult compute(
    const PositionControllerInput & input, double dt, bool integrator_enabled);

  void reset_horizontal_integrator();
  const Eigen::Vector2d & horizontal_integral_acceleration_world() const;

private:
  HorizontalPositionController horizontal_controller_;
  HoverController hover_controller_;
};

}  // namespace drone_controller

#endif  // DRONE_CONTROLLER__POSITION__POSITION_CONTROLLER_HPP_
