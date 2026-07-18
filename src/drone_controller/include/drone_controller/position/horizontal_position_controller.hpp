#ifndef DRONE_CONTROLLER__POSITION__HORIZONTAL_POSITION_CONTROLLER_HPP_
#define DRONE_CONTROLLER__POSITION__HORIZONTAL_POSITION_CONTROLLER_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace drone_controller
{

// Horizontal states and accelerations are expressed in map/ENU. Angles use radians.
struct HorizontalPositionControllerParameters
{
  Eigen::Vector2d position_kp{1.0, 1.0};
  Eigen::Vector2d velocity_kd{1.0, 1.0};
  bool enable_integral{false};
  Eigen::Vector2d position_ki{Eigen::Vector2d::Zero()};
  double integral_acceleration_limit{0.35};
  double anti_windup_gain{1.0};
  double integral_capture_radius{0.5};
  double gravity{9.80665};
  double max_horizontal_acceleration{5.0};
  double max_tilt_angle{0.5};
};

struct HorizontalPositionControllerInput
{
  Eigen::Vector2d desired_position_world{Eigen::Vector2d::Zero()};
  Eigen::Vector2d desired_velocity_world{Eigen::Vector2d::Zero()};
  Eigen::Vector2d desired_acceleration_world{Eigen::Vector2d::Zero()};
  Eigen::Vector2d current_position_world{Eigen::Vector2d::Zero()};
  Eigen::Vector2d current_velocity_world{Eigen::Vector2d::Zero()};
  double desired_yaw{0.0};
};

struct HorizontalPositionControllerResult
{
  Eigen::Vector2d proportional_acceleration_world{Eigen::Vector2d::Zero()};
  Eigen::Vector2d derivative_acceleration_world{Eigen::Vector2d::Zero()};
  Eigen::Vector2d integral_acceleration_world{Eigen::Vector2d::Zero()};
  Eigen::Vector2d feedforward_acceleration_world{Eigen::Vector2d::Zero()};
  Eigen::Vector2d raw_acceleration_world{Eigen::Vector2d::Zero()};
  Eigen::Vector2d desired_acceleration_world{Eigen::Vector2d::Zero()};
  double desired_roll{0.0};
  double desired_pitch{0.0};
  Eigen::Quaterniond desired_orientation_body_to_world{Eigen::Quaterniond::Identity()};
  bool valid{true};
  bool saturated{false};
  bool integral_enabled{false};
  bool integral_frozen{false};
  bool anti_windup_active{false};
};

// Horizontal PID-like controller. It converts world-frame x/y errors into a
// bounded world-frame acceleration and a body-to-world attitude with the requested
// yaw heading. The state is an acceleration contribution, not accumulated metres.
// It does not depend on ROS2 and does not command thrust or motors.
class HorizontalPositionController
{
public:
  explicit HorizontalPositionController(
    const HorizontalPositionControllerParameters & parameters =
    HorizontalPositionControllerParameters{});

  HorizontalPositionControllerResult compute(
    const HorizontalPositionControllerInput & input) const;

  HorizontalPositionControllerResult compute(
    const HorizontalPositionControllerInput & input, double dt,
    bool integrator_enabled);

  void reset_integrator();
  const Eigen::Vector2d & integral_acceleration_world() const;
  bool back_calculate_integrator_to_zero(double dt);

private:
  HorizontalPositionControllerParameters parameters_;
  Eigen::Vector2d integral_acceleration_world_{Eigen::Vector2d::Zero()};
};

}  // namespace drone_controller

#endif  // DRONE_CONTROLLER__POSITION__HORIZONTAL_POSITION_CONTROLLER_HPP_
