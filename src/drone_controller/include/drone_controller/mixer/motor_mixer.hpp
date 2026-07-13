#ifndef DRONE_CONTROLLER__MIXER__MOTOR_MIXER_HPP_
#define DRONE_CONTROLLER__MIXER__MOTOR_MIXER_HPP_

#include <array>

namespace drone_controller
{

struct MixerParameters
{
  // These values must remain consistent with the quadrotor dynamics parameters.
  double arm_length{0.20};
  double thrust_coefficient{1.91e-6};
  double drag_torque_coefficient{2.60e-7};
  double min_rpm{0.0};
  double max_rpm{20000.0};
};

struct WrenchCommand
{
  double thrust{0.0};
  double roll_torque{0.0};
  double pitch_torque{0.0};
  double yaw_torque{0.0};
};

struct MixerResult
{
  // Fixed order: [M1 front-left CCW, M2 rear-left CW,
  // M3 rear-right CCW, M4 front-right CW].
  std::array<double, 4> motor_rpm{};
  bool valid{true};
  bool saturated{false};
};

class MotorMixer
{
public:
  explicit MotorMixer(const MixerParameters & parameters = MixerParameters{});

  // Converts a desired body wrench into four target RPM values. Per-motor
  // clipping is intentionally simple: after saturation, the achieved wrench
  // can differ from the request and a future controller must use the flag.
  MixerResult mix(const WrenchCommand & command) const;

private:
  MixerParameters parameters_;
};

}  // namespace drone_controller

#endif  // DRONE_CONTROLLER__MIXER__MOTOR_MIXER_HPP_
