#ifndef DRONE_CONTROLLER__ATTITUDE__ATTITUDE_CONTROLLER_HPP_
#define DRONE_CONTROLLER__ATTITUDE__ATTITUDE_CONTROLLER_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace drone_controller
{

struct AttitudeControllerParameters
{
  Eigen::Vector3d attitude_kp{4.0, 4.0, 2.0};
  Eigen::Vector3d angular_rate_kd{0.20, 0.20, 0.10};
  Eigen::Vector3d max_torque{1.0, 1.0, 0.5};
};

struct AttitudeControllerInput
{
  // Both quaternions rotate vectors from base_link/body into map/world.
  Eigen::Quaterniond desired_orientation_body_to_world{Eigen::Quaterniond::Identity()};
  Eigen::Quaterniond current_orientation_body_to_world{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d desired_angular_velocity_body{Eigen::Vector3d::Zero()};
  Eigen::Vector3d current_angular_velocity_body{Eigen::Vector3d::Zero()};
};

struct AttitudeControllerResult
{
  // [roll, pitch, yaw] torque expressed in base_link, in N*m.
  Eigen::Vector3d torque_body{Eigen::Vector3d::Zero()};
  bool valid{true};
  bool saturated{false};
};

class AttitudeController
{
public:
  explicit AttitudeController(
    const AttitudeControllerParameters & parameters = AttitudeControllerParameters{});

  AttitudeControllerResult compute(const AttitudeControllerInput & input) const;

private:
  AttitudeControllerParameters parameters_;
};

}  // namespace drone_controller

#endif  // DRONE_CONTROLLER__ATTITUDE__ATTITUDE_CONTROLLER_HPP_
