#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "drone_controller/mixer/motor_mixer.hpp"

namespace
{

constexpr double kPi = 3.14159265358979323846;

struct ReconstructedWrench
{
  double thrust;
  double roll;
  double pitch;
  double yaw;
};

ReconstructedWrench reconstruct_wrench(
  const std::array<double, 4> & rpm,
  const drone_controller::MixerParameters & parameters)
{
  std::array<double, 4> force{};
  for (std::size_t i = 0; i < rpm.size(); ++i) {
    const double omega = rpm[i] * 2.0 * kPi / 60.0;
    force[i] = parameters.thrust_coefficient * omega * omega;
  }
  const double a = parameters.arm_length / std::sqrt(2.0);
  const double b = parameters.drag_torque_coefficient / parameters.thrust_coefficient;
  return {
    force[0] + force[1] + force[2] + force[3],
    a * (force[0] + force[1] - force[2] - force[3]),
    a * (-force[0] + force[1] + force[2] - force[3]),
    b * (-force[0] + force[1] - force[2] + force[3])};
}

TEST(MotorMixer, ZeroInputProducesZeroRpm)
{
  const drone_controller::MotorMixer mixer;
  const auto result = mixer.mix({});
  EXPECT_TRUE(result.valid);
  EXPECT_FALSE(result.saturated);
  for (const double rpm : result.motor_rpm) {
    EXPECT_DOUBLE_EQ(rpm, 0.0);
    EXPECT_TRUE(std::isfinite(rpm));
  }
}

TEST(MotorMixer, PureHoverThrustProducesEqualRpm)
{
  const drone_controller::MotorMixer mixer;
  const auto result = mixer.mix({9.80665, 0.0, 0.0, 0.0});
  EXPECT_TRUE(result.valid);
  EXPECT_FALSE(result.saturated);
  for (const double rpm : result.motor_rpm) {
    EXPECT_NEAR(rpm, 10818.9, 0.1);
    EXPECT_NEAR(rpm, result.motor_rpm[0], 1.0e-10);
  }
}

TEST(MotorMixer, PositiveRollRaisesLeftMotors)
{
  const drone_controller::MixerParameters parameters;
  const drone_controller::MotorMixer mixer(parameters);
  const auto baseline = mixer.mix({9.80665, 0.0, 0.0, 0.0});
  const auto result = mixer.mix({9.80665, 0.10, 0.0, 0.0});
  EXPECT_GT(result.motor_rpm[0], baseline.motor_rpm[0]);
  EXPECT_GT(result.motor_rpm[1], baseline.motor_rpm[1]);
  EXPECT_LT(result.motor_rpm[2], baseline.motor_rpm[2]);
  EXPECT_LT(result.motor_rpm[3], baseline.motor_rpm[3]);
  EXPECT_NEAR(result.motor_rpm[0], result.motor_rpm[1], 1.0e-10);
  EXPECT_NEAR(result.motor_rpm[2], result.motor_rpm[3], 1.0e-10);
  const auto wrench = reconstruct_wrench(result.motor_rpm, parameters);
  EXPECT_NEAR(wrench.roll, 0.10, 1.0e-12);
  EXPECT_NEAR(wrench.pitch, 0.0, 1.0e-12);
  EXPECT_NEAR(wrench.yaw, 0.0, 1.0e-12);
}

TEST(MotorMixer, PositivePitchRaisesRearMotors)
{
  const drone_controller::MixerParameters parameters;
  const drone_controller::MotorMixer mixer(parameters);
  const auto baseline = mixer.mix({9.80665, 0.0, 0.0, 0.0});
  const auto result = mixer.mix({9.80665, 0.0, 0.10, 0.0});
  EXPECT_LT(result.motor_rpm[0], baseline.motor_rpm[0]);
  EXPECT_GT(result.motor_rpm[1], baseline.motor_rpm[1]);
  EXPECT_GT(result.motor_rpm[2], baseline.motor_rpm[2]);
  EXPECT_LT(result.motor_rpm[3], baseline.motor_rpm[3]);
  const auto wrench = reconstruct_wrench(result.motor_rpm, parameters);
  EXPECT_NEAR(wrench.roll, 0.0, 1.0e-12);
  EXPECT_NEAR(wrench.pitch, 0.10, 1.0e-12);
  EXPECT_NEAR(wrench.yaw, 0.0, 1.0e-12);
}

TEST(MotorMixer, PositiveYawRaisesClockwiseMotors)
{
  const drone_controller::MixerParameters parameters;
  const drone_controller::MotorMixer mixer(parameters);
  const auto baseline = mixer.mix({9.80665, 0.0, 0.0, 0.0});
  const auto result = mixer.mix({9.80665, 0.0, 0.0, 0.02});
  EXPECT_LT(result.motor_rpm[0], baseline.motor_rpm[0]);
  EXPECT_GT(result.motor_rpm[1], baseline.motor_rpm[1]);
  EXPECT_LT(result.motor_rpm[2], baseline.motor_rpm[2]);
  EXPECT_GT(result.motor_rpm[3], baseline.motor_rpm[3]);
  const auto wrench = reconstruct_wrench(result.motor_rpm, parameters);
  EXPECT_NEAR(wrench.thrust, 9.80665, 1.0e-12);
  EXPECT_NEAR(wrench.yaw, 0.02, 1.0e-12);
}

TEST(MotorMixer, MixedCommandRoundTripsThroughForwardEquations)
{
  const drone_controller::MixerParameters parameters;
  const drone_controller::MotorMixer mixer(parameters);
  const drone_controller::WrenchCommand command{8.0, 0.12, -0.08, 0.015};
  const auto result = mixer.mix(command);
  ASSERT_TRUE(result.valid);
  ASSERT_FALSE(result.saturated);
  const auto wrench = reconstruct_wrench(result.motor_rpm, parameters);
  EXPECT_NEAR(wrench.thrust, command.thrust, 1.0e-12);
  EXPECT_NEAR(wrench.roll, command.roll_torque, 1.0e-12);
  EXPECT_NEAR(wrench.pitch, command.pitch_torque, 1.0e-12);
  EXPECT_NEAR(wrench.yaw, command.yaw_torque, 1.0e-12);
}

TEST(MotorMixer, ImpossibleCommandIsSafelySaturated)
{
  const drone_controller::MotorMixer mixer;
  const auto result = mixer.mix({1.0, 100.0, -100.0, 50.0});
  EXPECT_TRUE(result.valid);
  EXPECT_TRUE(result.saturated);
  for (const double rpm : result.motor_rpm) {
    EXPECT_TRUE(std::isfinite(rpm));
    EXPECT_GE(rpm, 0.0);
    EXPECT_LE(rpm, 20000.0);
  }
}

TEST(MotorMixer, NegativeThrustIsClampedAndMarkedSaturated)
{
  const drone_controller::MotorMixer mixer;
  const auto result = mixer.mix({-1.0, 0.0, 0.0, 0.0});
  EXPECT_TRUE(result.valid);
  EXPECT_TRUE(result.saturated);
  for (const double rpm : result.motor_rpm) {
    EXPECT_DOUBLE_EQ(rpm, 0.0);
  }
}

TEST(MotorMixer, NonFiniteCommandReturnsSafeInvalidResult)
{
  const drone_controller::MotorMixer mixer;
  for (const drone_controller::WrenchCommand command : {
      drone_controller::WrenchCommand{std::numeric_limits<double>::quiet_NaN(), 0, 0, 0},
      drone_controller::WrenchCommand{1, std::numeric_limits<double>::infinity(), 0, 0},
      drone_controller::WrenchCommand{1, 0, -std::numeric_limits<double>::infinity(), 0}})
  {
    const auto result = mixer.mix(command);
    EXPECT_FALSE(result.valid);
    EXPECT_TRUE(result.saturated);
    EXPECT_EQ(result.motor_rpm, (std::array<double, 4>{}));
  }
}

TEST(MotorMixer, InvalidParametersThrow)
{
  drone_controller::MixerParameters parameters;
  parameters.arm_length = -1.0;
  EXPECT_THROW(drone_controller::MotorMixer{parameters}, std::invalid_argument);
  parameters = {};
  parameters.thrust_coefficient = 0.0;
  EXPECT_THROW(drone_controller::MotorMixer{parameters}, std::invalid_argument);
  parameters = {};
  parameters.drag_torque_coefficient = std::numeric_limits<double>::infinity();
  EXPECT_THROW(drone_controller::MotorMixer{parameters}, std::invalid_argument);
  parameters = {};
  parameters.min_rpm = 100.0;
  parameters.max_rpm = 100.0;
  EXPECT_THROW(drone_controller::MotorMixer{parameters}, std::invalid_argument);
}

}  // namespace
