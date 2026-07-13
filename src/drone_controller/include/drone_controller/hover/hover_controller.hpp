#ifndef DRONE_CONTROLLER__HOVER__HOVER_CONTROLLER_HPP_
#define DRONE_CONTROLLER__HOVER__HOVER_CONTROLLER_HPP_

#include <array>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "drone_controller/altitude/altitude_controller.hpp"
#include "drone_controller/attitude/attitude_controller.hpp"
#include "drone_controller/mixer/motor_mixer.hpp"

namespace drone_controller
{

struct HoverControllerParameters
{
  AltitudeControllerParameters altitude;
  AttitudeControllerParameters attitude;
  MixerParameters mixer;
};

struct HoverControllerInput
{
  double desired_altitude{0.0};
  double desired_vertical_velocity{0.0};
  double desired_vertical_acceleration{0.0};
  Eigen::Quaterniond desired_orientation_body_to_world{Eigen::Quaterniond::Identity()};
  double current_altitude{0.0};
  double current_vertical_velocity_world{0.0};
  Eigen::Quaterniond current_orientation_body_to_world{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d desired_angular_velocity_body{Eigen::Vector3d::Zero()};
  Eigen::Vector3d current_angular_velocity_body{Eigen::Vector3d::Zero()};
};

struct HoverControllerResult
{
  std::array<double, 4> motor_rpm{};
  double collective_thrust{0.0};
  Eigen::Vector3d torque_body{Eigen::Vector3d::Zero()};
  bool valid{true};
  bool saturated{false};
  bool altitude_saturated{false};
  bool attitude_saturated{false};
  bool mixer_saturated{false};
};

class HoverController
{
public:
  explicit HoverController(
    const HoverControllerParameters & parameters = HoverControllerParameters{});

  HoverControllerResult compute(const HoverControllerInput & input) const;

private:
  AltitudeController altitude_controller_;
  AttitudeController attitude_controller_;
  MotorMixer motor_mixer_;
};

// Pure conversion helpers used by the ROS node and unit tests.
bool world_vertical_velocity_from_body(
  const Eigen::Quaterniond & orientation_body_to_world,
  const Eigen::Vector3d & velocity_body,
  double & vertical_velocity_world);

bool level_orientation_from_goal_yaw(
  const Eigen::Quaterniond & goal_orientation_body_to_world,
  Eigen::Quaterniond & level_orientation_body_to_world);

}  // namespace drone_controller

#endif  // DRONE_CONTROLLER__HOVER__HOVER_CONTROLLER_HPP_
