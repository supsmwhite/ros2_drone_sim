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
  double gravity{9.80665};
  double max_horizontal_acceleration{5.0};
  double max_tilt_angle{0.5};
};

struct HorizontalPositionControllerInput
{
  Eigen::Vector2d desired_position_world{Eigen::Vector2d::Zero()};
  Eigen::Vector2d desired_velocity_world{Eigen::Vector2d::Zero()};
  Eigen::Vector2d current_position_world{Eigen::Vector2d::Zero()};
  Eigen::Vector2d current_velocity_world{Eigen::Vector2d::Zero()};
  double desired_yaw{0.0};
};

struct HorizontalPositionControllerResult
{
  Eigen::Vector2d desired_acceleration_world{Eigen::Vector2d::Zero()};
  double desired_roll{0.0};
  double desired_pitch{0.0};
  Eigen::Quaterniond desired_orientation_body_to_world{Eigen::Quaterniond::Identity()};
  bool valid{true};
  bool saturated{false};
};

// Stateless horizontal PD controller. It converts world-frame x/y errors into a
// bounded world-frame acceleration and a body-to-world attitude with the requested
// yaw heading. It does not depend on ROS2 and does not command thrust or motors.
class HorizontalPositionController
{
public:
  explicit HorizontalPositionController(
    const HorizontalPositionControllerParameters & parameters =
    HorizontalPositionControllerParameters{});

  HorizontalPositionControllerResult compute(
    const HorizontalPositionControllerInput & input) const;

private:
  HorizontalPositionControllerParameters parameters_;
};

}  // namespace drone_controller

#endif  // DRONE_CONTROLLER__POSITION__HORIZONTAL_POSITION_CONTROLLER_HPP_
