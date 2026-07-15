#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "drone_controller/position/position_controller.hpp"

namespace
{

constexpr double kTolerance = 1.0e-10;

TEST(PositionController, ZeroHorizontalErrorMatchesBalancedHover)
{
  const drone_controller::PositionController controller;
  const drone_controller::HoverController hover_controller;
  const auto result = controller.compute({});
  const auto hover_result = hover_controller.compute({});

  ASSERT_TRUE(result.valid);
  ASSERT_TRUE(hover_result.valid);
  EXPECT_FALSE(result.saturated);
  EXPECT_NEAR(result.desired_roll, 0.0, kTolerance);
  EXPECT_NEAR(result.desired_pitch, 0.0, kTolerance);
  EXPECT_TRUE(result.desired_orientation_body_to_world.isApprox(
    Eigen::Quaterniond::Identity(), kTolerance));
  for (std::size_t index = 0; index < result.motor_rpm.size(); ++index) {
    EXPECT_NEAR(result.motor_rpm[index], hover_result.motor_rpm[index], kTolerance);
    EXPECT_NEAR(result.motor_rpm[index], result.motor_rpm[0], kTolerance);
  }
}

TEST(PositionController, PositiveXTargetCommandsPitchAndRearMotorIncrease)
{
  const drone_controller::PositionController controller;
  drone_controller::PositionControllerInput input;
  input.desired_position_world.x() = 1.0;
  const auto result = controller.compute(input);

  ASSERT_TRUE(result.valid);
  EXPECT_GT(result.desired_pitch, 0.0);
  EXPECT_GT(result.torque_body.y(), 0.0);
  EXPECT_GT(result.motor_rpm[1], result.motor_rpm[0]);
  EXPECT_GT(result.motor_rpm[2], result.motor_rpm[3]);
}

TEST(PositionController, PositiveYTargetCommandsNegativeRollAndRightMotorIncrease)
{
  const drone_controller::PositionController controller;
  drone_controller::PositionControllerInput input;
  input.desired_position_world.y() = 1.0;
  const auto result = controller.compute(input);

  ASSERT_TRUE(result.valid);
  EXPECT_LT(result.desired_roll, 0.0);
  EXPECT_LT(result.torque_body.x(), 0.0);
  EXPECT_GT(result.motor_rpm[2], result.motor_rpm[1]);
  EXPECT_GT(result.motor_rpm[3], result.motor_rpm[0]);
}

TEST(PositionController, HorizontalVelocityFeedbackOpposesMotion)
{
  const drone_controller::PositionController controller;
  drone_controller::PositionControllerInput input;
  input.current_velocity_world.x() = 1.0;
  input.current_velocity_world.y() = -2.0;
  const auto result = controller.compute(input);

  ASSERT_TRUE(result.valid);
  EXPECT_LT(result.desired_horizontal_acceleration_world.x(), 0.0);
  EXPECT_GT(result.desired_horizontal_acceleration_world.y(), 0.0);
}

TEST(PositionController, DesiredAccelerationFeedsHorizontalAndVerticalControllers)
{
  const drone_controller::PositionController controller;
  const auto baseline = controller.compute({});
  drone_controller::PositionControllerInput input;
  input.desired_acceleration_world = Eigen::Vector3d(0.2, -0.1, 0.5);
  const auto result = controller.compute(input);

  ASSERT_TRUE(baseline.valid);
  ASSERT_TRUE(result.valid);
  EXPECT_TRUE(result.desired_horizontal_acceleration_world.isApprox(
    input.desired_acceleration_world.head<2>(), kTolerance));
  EXPECT_GT(result.collective_thrust, baseline.collective_thrust);
}

TEST(PositionController, HorizontalSaturationPropagates)
{
  drone_controller::PositionControllerParameters parameters;
  parameters.horizontal.max_horizontal_acceleration = 0.5;
  const drone_controller::PositionController controller(parameters);
  drone_controller::PositionControllerInput input;
  input.desired_position_world.x() = 100.0;
  const auto result = controller.compute(input);

  ASSERT_TRUE(result.valid);
  EXPECT_TRUE(result.horizontal_saturated);
  EXPECT_TRUE(result.saturated);
  EXPECT_LE(result.desired_horizontal_acceleration_world.norm(), 0.5 + kTolerance);
}

TEST(PositionController, InvalidHorizontalInputReturnsZeroRpm)
{
  const drone_controller::PositionController controller;
  drone_controller::PositionControllerInput input;
  input.desired_position_world.x() = std::numeric_limits<double>::quiet_NaN();
  const auto result = controller.compute(input);

  EXPECT_FALSE(result.valid);
  EXPECT_TRUE(result.saturated);
  EXPECT_EQ(result.motor_rpm, (std::array<double, 4>{}));
}

TEST(PositionController, HoverFailureReturnsZeroRpm)
{
  const drone_controller::PositionController controller;
  drone_controller::PositionControllerInput input;
  input.current_orientation_body_to_world.coeffs().setZero();
  const auto result = controller.compute(input);

  EXPECT_FALSE(result.valid);
  EXPECT_TRUE(result.saturated);
  EXPECT_EQ(result.motor_rpm, (std::array<double, 4>{}));
}

TEST(PositionController, ZeroXYTargetPreservesExistingAltitudeYawBehavior)
{
  drone_controller::PositionControllerParameters parameters;
  const drone_controller::PositionController controller(parameters);
  const drone_controller::HoverController hover_controller(parameters.hover);
  drone_controller::PositionControllerInput input;
  input.desired_position_world.z() = 0.5;
  input.desired_yaw = 0.2;
  const auto result = controller.compute(input);

  drone_controller::HoverControllerInput hover_input;
  hover_input.desired_altitude = input.desired_position_world.z();
  hover_input.desired_orientation_body_to_world = Eigen::Quaterniond(
    Eigen::AngleAxisd(input.desired_yaw, Eigen::Vector3d::UnitZ()));
  const auto hover_result = hover_controller.compute(hover_input);

  ASSERT_TRUE(result.valid);
  ASSERT_TRUE(hover_result.valid);
  EXPECT_TRUE(result.desired_orientation_body_to_world.isApprox(
    hover_input.desired_orientation_body_to_world, kTolerance));
  for (std::size_t index = 0; index < result.motor_rpm.size(); ++index) {
    EXPECT_NEAR(result.motor_rpm[index], hover_result.motor_rpm[index], kTolerance);
  }
}

TEST(PositionController, MismatchedGravityParametersThrow)
{
  drone_controller::PositionControllerParameters parameters;
  parameters.horizontal.gravity += 0.1;
  EXPECT_THROW(drone_controller::PositionController{parameters}, std::invalid_argument);
}

}  // namespace
