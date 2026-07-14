#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "drone_controller/hover/hover_controller.hpp"

namespace
{

constexpr double kHoverRpm = 10818.9;
constexpr double kTolerance = 1.0e-10;

Eigen::Quaterniond rotation(const Eigen::Vector3d & axis, const double angle)
{
  return Eigen::Quaterniond(Eigen::AngleAxisd(angle, axis));
}

TEST(HoverController, BalancedHoverProducesEqualHoverRpm)
{
  const drone_controller::HoverController controller;
  const auto result = controller.compute({});
  ASSERT_TRUE(result.valid);
  EXPECT_FALSE(result.saturated);
  for (const double rpm : result.motor_rpm) {
    EXPECT_NEAR(rpm, kHoverRpm, 0.1);
    EXPECT_NEAR(rpm, result.motor_rpm[0], 1.0e-10);
  }
}

TEST(HoverController, LowerThanTargetRaisesAllMotors)
{
  const drone_controller::HoverController controller;
  drone_controller::HoverControllerInput input;
  input.desired_altitude = 0.5;
  const auto result = controller.compute(input);
  ASSERT_TRUE(result.valid);
  for (const double rpm : result.motor_rpm) {
    EXPECT_GT(rpm, kHoverRpm);
    EXPECT_NEAR(rpm, result.motor_rpm[0], 1.0e-10);
  }
}

TEST(HoverController, FastUpwardMotionLowersAllMotors)
{
  const drone_controller::HoverController controller;
  drone_controller::HoverControllerInput input;
  input.current_vertical_velocity_world = 0.5;
  const auto result = controller.compute(input);
  ASSERT_TRUE(result.valid);
  for (const double rpm : result.motor_rpm) {
    EXPECT_LT(rpm, kHoverRpm);
  }
}

TEST(HoverController, PositiveCurrentRollCommandsRestoringMotorPattern)
{
  const drone_controller::HoverController controller;
  drone_controller::HoverControllerInput input;
  input.current_orientation_body_to_world = rotation(Eigen::Vector3d::UnitX(), 0.1);
  const auto result = controller.compute(input);
  ASSERT_TRUE(result.valid);
  EXPECT_LT(result.torque_body.x(), 0.0);
  EXPECT_LT(result.motor_rpm[0], result.motor_rpm[2]);
  EXPECT_LT(result.motor_rpm[1], result.motor_rpm[3]);
}

TEST(HoverController, PositiveCurrentPitchCommandsRestoringMotorPattern)
{
  const drone_controller::HoverController controller;
  drone_controller::HoverControllerInput input;
  input.current_orientation_body_to_world = rotation(Eigen::Vector3d::UnitY(), 0.1);
  const auto result = controller.compute(input);
  ASSERT_TRUE(result.valid);
  EXPECT_LT(result.torque_body.y(), 0.0);
  EXPECT_GT(result.motor_rpm[0], result.motor_rpm[1]);
  EXPECT_GT(result.motor_rpm[3], result.motor_rpm[2]);
}

TEST(HoverController, PositiveYawErrorRaisesClockwiseMotors)
{
  const drone_controller::HoverController controller;
  drone_controller::HoverControllerInput input;
  input.desired_orientation_body_to_world = rotation(Eigen::Vector3d::UnitZ(), 0.1);
  const auto result = controller.compute(input);
  ASSERT_TRUE(result.valid);
  EXPECT_GT(result.torque_body.z(), 0.0);
  EXPECT_GT(result.motor_rpm[1], result.motor_rpm[0]);
  EXPECT_GT(result.motor_rpm[3], result.motor_rpm[2]);
}

TEST(HoverController, AltitudeSaturationPropagates)
{
  const drone_controller::HoverController controller;
  drone_controller::HoverControllerInput input;
  input.desired_altitude = 100.0;
  const auto result = controller.compute(input);
  EXPECT_TRUE(result.valid);
  EXPECT_TRUE(result.altitude_saturated);
  EXPECT_TRUE(result.saturated);
  for (const double rpm : result.motor_rpm) {
    EXPECT_TRUE(std::isfinite(rpm));
  }
}

TEST(HoverController, AttitudeSaturationPropagates)
{
  drone_controller::HoverControllerParameters parameters;
  parameters.attitude.max_torque.setConstant(0.01);
  const drone_controller::HoverController controller(parameters);
  drone_controller::HoverControllerInput input;
  input.current_orientation_body_to_world = rotation(Eigen::Vector3d::UnitX(), 0.3);
  const auto result = controller.compute(input);
  EXPECT_TRUE(result.valid);
  EXPECT_TRUE(result.attitude_saturated);
  EXPECT_TRUE(result.saturated);
}

TEST(HoverController, MixerSaturationPropagatesAndBoundsRpm)
{
  drone_controller::HoverControllerParameters parameters;
  parameters.mixer.max_rpm = 10000.0;
  const drone_controller::HoverController controller(parameters);
  const auto result = controller.compute({});
  EXPECT_TRUE(result.valid);
  EXPECT_TRUE(result.mixer_saturated);
  EXPECT_TRUE(result.saturated);
  for (const double rpm : result.motor_rpm) {
    EXPECT_GE(rpm, 0.0);
    EXPECT_LE(rpm, 10000.0);
  }
}

TEST(HoverController, InvalidInputReturnsZeroRpm)
{
  const drone_controller::HoverController controller;
  drone_controller::HoverControllerInput input;
  input.current_orientation_body_to_world.coeffs().setZero();
  auto result = controller.compute(input);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.motor_rpm, (std::array<double, 4>{}));

  input = {};
  input.desired_altitude = std::numeric_limits<double>::infinity();
  result = controller.compute(input);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.motor_rpm, (std::array<double, 4>{}));
}

TEST(HoverConversions, HorizontalBodyZEqualsWorldZ)
{
  double world_vz = 0.0;
  ASSERT_TRUE(drone_controller::world_vertical_velocity_from_body(
      Eigen::Quaterniond::Identity(), Eigen::Vector3d(1.0, 2.0, 3.0), world_vz));
  EXPECT_DOUBLE_EQ(world_vz, 3.0);
}

TEST(HoverConversions, IdentityOrientationPreservesCompleteVelocity)
{
  Eigen::Vector3d velocity_world;
  const Eigen::Vector3d velocity_body(1.0, -2.0, 3.0);
  ASSERT_TRUE(drone_controller::world_velocity_from_body(
      Eigen::Quaterniond::Identity(), velocity_body, velocity_world));
  EXPECT_TRUE(velocity_world.isApprox(velocity_body, kTolerance));
}

TEST(HoverConversions, PitchRotatesBodyXIntoWorldZ)
{
  Eigen::Vector3d velocity_world;
  const Eigen::Quaterniond orientation = rotation(Eigen::Vector3d::UnitY(), 0.5);
  ASSERT_TRUE(drone_controller::world_velocity_from_body(
      orientation, Eigen::Vector3d(2.0, 0.0, 0.0), velocity_world));
  EXPECT_NEAR(velocity_world.x(), 2.0 * std::cos(0.5), kTolerance);
  EXPECT_NEAR(velocity_world.y(), 0.0, kTolerance);
  EXPECT_NEAR(velocity_world.z(), -2.0 * std::sin(0.5), kTolerance);
}

TEST(HoverConversions, YawRotatesBodyXYIntoWorldXY)
{
  Eigen::Vector3d velocity_world;
  const Eigen::Quaterniond orientation =
    rotation(Eigen::Vector3d::UnitZ(), std::acos(-1.0) / 2.0);
  ASSERT_TRUE(drone_controller::world_velocity_from_body(
      orientation, Eigen::Vector3d(2.0, 3.0, 0.0), velocity_world));
  EXPECT_NEAR(velocity_world.x(), -3.0, kTolerance);
  EXPECT_NEAR(velocity_world.y(), 2.0, kTolerance);
  EXPECT_NEAR(velocity_world.z(), 0.0, kTolerance);
}

TEST(HoverConversions, CompleteVelocityRejectsInvalidQuaternion)
{
  Eigen::Quaterniond invalid;
  invalid.coeffs().setZero();
  Eigen::Vector3d velocity_world;
  EXPECT_FALSE(drone_controller::world_velocity_from_body(
      invalid, Eigen::Vector3d::Ones(), velocity_world));
}

TEST(HoverConversions, TiltedVelocityRequiresFullRotation)
{
  double world_vz = 0.0;
  const Eigen::Quaterniond orientation = rotation(Eigen::Vector3d::UnitY(), 0.5);
  const Eigen::Vector3d velocity_body(2.0, 0.0, 0.0);
  ASSERT_TRUE(drone_controller::world_vertical_velocity_from_body(
      orientation, velocity_body, world_vz));
  EXPECT_NEAR(world_vz, -2.0 * std::sin(0.5), 1.0e-12);
  EXPECT_NE(world_vz, velocity_body.z());
}

TEST(HoverConversions, InvalidQuaternionFailsSafely)
{
  Eigen::Quaterniond invalid;
  invalid.coeffs().setZero();
  double world_vz = 123.0;
  EXPECT_FALSE(drone_controller::world_vertical_velocity_from_body(
      invalid, Eigen::Vector3d::Zero(), world_vz));
  Eigen::Quaterniond level;
  EXPECT_FALSE(drone_controller::level_orientation_from_goal_yaw(invalid, level));
}

TEST(HoverConversions, GoalRollPitchAreRemovedButYawIsPreserved)
{
  const Eigen::Quaterniond goal =
    Eigen::AngleAxisd(0.6, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(-0.1, Eigen::Vector3d::UnitX());
  Eigen::Quaterniond level;
  ASSERT_TRUE(drone_controller::level_orientation_from_goal_yaw(goal, level));
  const Eigen::Vector3d body_z_world = level * Eigen::Vector3d::UnitZ();
  EXPECT_NEAR(body_z_world.x(), 0.0, 1.0e-12);
  EXPECT_NEAR(body_z_world.y(), 0.0, 1.0e-12);
  EXPECT_NEAR(body_z_world.z(), 1.0, 1.0e-12);
  EXPECT_NEAR(std::atan2(level.toRotationMatrix()(1, 0), level.toRotationMatrix()(0, 0)),
    0.6, 1.0e-12);
}

}  // namespace
