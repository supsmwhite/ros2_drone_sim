#ifndef DRONE_CONTROLLER__ALTITUDE__ALTITUDE_CONTROLLER_HPP_
#define DRONE_CONTROLLER__ALTITUDE__ALTITUDE_CONTROLLER_HPP_

#include <Eigen/Geometry>

namespace drone_controller
{

struct AltitudeControllerParameters
{
  double mass{1.0};
  double gravity{9.80665};
  double altitude_kp{3.0};
  double vertical_velocity_kd{3.5};
  double max_upward_acceleration{5.0};
  double max_downward_acceleration{5.0};
  double min_collective_thrust{0.0};
  double max_collective_thrust{30.0};
  double min_tilt_cosine{0.5};
};

struct AltitudeControllerInput
{
  double desired_altitude{0.0};
  double desired_vertical_velocity{0.0};
  double desired_vertical_acceleration{0.0};
  double current_altitude{0.0};
  // This is world/map vertical velocity, not Odometry twist.linear.z directly.
  double current_vertical_velocity{0.0};
  Eigen::Quaterniond current_orientation_body_to_world{Eigen::Quaterniond::Identity()};
};

struct AltitudeControllerResult
{
  double collective_thrust{0.0};
  double commanded_vertical_acceleration{0.0};
  bool valid{true};
  bool saturated{false};
};

class AltitudeController
{
public:
  explicit AltitudeController(
    const AltitudeControllerParameters & parameters = AltitudeControllerParameters{});

  AltitudeControllerResult compute(const AltitudeControllerInput & input) const;

private:
  AltitudeControllerParameters parameters_;
};

}  // namespace drone_controller

#endif  // DRONE_CONTROLLER__ALTITUDE__ALTITUDE_CONTROLLER_HPP_
