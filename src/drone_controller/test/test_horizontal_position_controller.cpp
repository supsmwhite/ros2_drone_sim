#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "drone_controller/position/horizontal_position_controller.hpp"

namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kTolerance = 1.0e-10;

double yaw_from_rotation(const Eigen::Matrix3d & rotation)
{
  return std::atan2(rotation(1, 0), rotation(0, 0));
}

double tilt_from_rotation(const Eigen::Matrix3d & rotation)
{
  return std::acos(std::clamp(rotation(2, 2), -1.0, 1.0));
}

TEST(HorizontalPositionController, ZeroErrorPreservesYawAndProducesLevelAttitude)
{
  const drone_controller::HorizontalPositionController controller;
  drone_controller::HorizontalPositionControllerInput input;
  input.desired_yaw = 0.7;
  const auto result = controller.compute(input);

  ASSERT_TRUE(result.valid);
  EXPECT_FALSE(result.saturated);
  EXPECT_TRUE(result.desired_acceleration_world.isZero(kTolerance));
  EXPECT_NEAR(result.desired_roll, 0.0, kTolerance);
  EXPECT_NEAR(result.desired_pitch, 0.0, kTolerance);
  EXPECT_NEAR(result.desired_orientation_body_to_world.norm(), 1.0, kTolerance);
  EXPECT_NEAR(yaw_from_rotation(result.desired_orientation_body_to_world.toRotationMatrix()),
    input.desired_yaw, kTolerance);
}

TEST(HorizontalPositionController, PositiveXErrorProducesPositivePitchAtZeroYaw)
{
  const drone_controller::HorizontalPositionController controller;
  drone_controller::HorizontalPositionControllerInput input;
  input.desired_position_world.x() = 1.0;
  const auto result = controller.compute(input);

  ASSERT_TRUE(result.valid);
  EXPECT_GT(result.desired_acceleration_world.x(), 0.0);
  EXPECT_NEAR(result.desired_acceleration_world.y(), 0.0, kTolerance);
  EXPECT_GT(result.desired_pitch, 0.0);
  EXPECT_NEAR(result.desired_roll, 0.0, kTolerance);
}

TEST(HorizontalPositionController, NegativeXErrorProducesNegativePitchAtZeroYaw)
{
  const drone_controller::HorizontalPositionController controller;
  drone_controller::HorizontalPositionControllerInput input;
  input.desired_position_world.x() = -1.0;
  const auto result = controller.compute(input);

  ASSERT_TRUE(result.valid);
  EXPECT_LT(result.desired_acceleration_world.x(), 0.0);
  EXPECT_LT(result.desired_pitch, 0.0);
  EXPECT_NEAR(result.desired_roll, 0.0, kTolerance);
}

TEST(HorizontalPositionController, PositiveYErrorProducesNegativeRollAtZeroYaw)
{
  const drone_controller::HorizontalPositionController controller;
  drone_controller::HorizontalPositionControllerInput input;
  input.desired_position_world.y() = 1.0;
  const auto result = controller.compute(input);

  ASSERT_TRUE(result.valid);
  EXPECT_GT(result.desired_acceleration_world.y(), 0.0);
  EXPECT_LT(result.desired_roll, 0.0);
  EXPECT_NEAR(result.desired_pitch, 0.0, kTolerance);
}

TEST(HorizontalPositionController, NegativeYErrorProducesPositiveRollAtZeroYaw)
{
  const drone_controller::HorizontalPositionController controller;
  drone_controller::HorizontalPositionControllerInput input;
  input.desired_position_world.y() = -1.0;
  const auto result = controller.compute(input);

  ASSERT_TRUE(result.valid);
  EXPECT_LT(result.desired_acceleration_world.y(), 0.0);
  EXPECT_GT(result.desired_roll, 0.0);
  EXPECT_NEAR(result.desired_pitch, 0.0, kTolerance);
}

TEST(HorizontalPositionController, XVelocityFeedbackOpposesCurrentVelocity)
{
  const drone_controller::HorizontalPositionController controller;
  drone_controller::HorizontalPositionControllerInput input;
  input.current_velocity_world.x() = 2.0;
  auto result = controller.compute(input);
  ASSERT_TRUE(result.valid);
  EXPECT_LT(result.desired_acceleration_world.x(), 0.0);

  input.current_velocity_world.x() = -2.0;
  result = controller.compute(input);
  ASSERT_TRUE(result.valid);
  EXPECT_GT(result.desired_acceleration_world.x(), 0.0);
}

TEST(HorizontalPositionController, AccelerationFeedforwardIsAddedAtZeroTrackingError)
{
  const drone_controller::HorizontalPositionController controller;
  drone_controller::HorizontalPositionControllerInput input;
  input.desired_acceleration_world = Eigen::Vector2d(0.3, -0.2);
  const auto result = controller.compute(input);

  ASSERT_TRUE(result.valid);
  EXPECT_FALSE(result.saturated);
  EXPECT_TRUE(result.desired_acceleration_world.isApprox(
    input.desired_acceleration_world, kTolerance));
}

TEST(HorizontalPositionController, AccelerationFeedforwardUsesExistingVectorLimit)
{
  drone_controller::HorizontalPositionControllerParameters parameters;
  parameters.max_horizontal_acceleration = 0.5;
  const drone_controller::HorizontalPositionController controller(parameters);
  drone_controller::HorizontalPositionControllerInput input;
  input.desired_acceleration_world = Eigen::Vector2d(3.0, 4.0);
  const auto result = controller.compute(input);

  ASSERT_TRUE(result.valid);
  EXPECT_TRUE(result.saturated);
  EXPECT_NEAR(result.desired_acceleration_world.norm(), 0.5, kTolerance);
  EXPECT_NEAR(result.desired_acceleration_world.x(), 0.3, kTolerance);
  EXPECT_NEAR(result.desired_acceleration_world.y(), 0.4, kTolerance);
}

TEST(HorizontalPositionController, YVelocityFeedbackOpposesCurrentVelocity)
{
  const drone_controller::HorizontalPositionController controller;
  drone_controller::HorizontalPositionControllerInput input;
  input.current_velocity_world.y() = 2.0;
  auto result = controller.compute(input);
  ASSERT_TRUE(result.valid);
  EXPECT_LT(result.desired_acceleration_world.y(), 0.0);

  input.current_velocity_world.y() = -2.0;
  result = controller.compute(input);
  ASSERT_TRUE(result.valid);
  EXPECT_GT(result.desired_acceleration_world.y(), 0.0);
}

TEST(HorizontalPositionController, NinetyDegreeYawRotatesAccelerationToRoll)
{
  const drone_controller::HorizontalPositionController controller;
  drone_controller::HorizontalPositionControllerInput input;
  input.desired_position_world.x() = 1.0;
  input.desired_yaw = kPi / 2.0;
  const auto result = controller.compute(input);

  ASSERT_TRUE(result.valid);
  EXPECT_GT(result.desired_acceleration_world.x(), 0.0);
  EXPECT_GT(result.desired_roll, 0.0);
  EXPECT_NEAR(result.desired_pitch, 0.0, kTolerance);
}

TEST(HorizontalPositionController, DiagonalErrorKeepsDirectionAndFiniteAttitude)
{
  const drone_controller::HorizontalPositionController controller;
  drone_controller::HorizontalPositionControllerInput input;
  input.desired_position_world = Eigen::Vector2d(1.0, 2.0);
  input.desired_yaw = 0.3;
  const auto result = controller.compute(input);

  ASSERT_TRUE(result.valid);
  EXPECT_FALSE(result.saturated);
  EXPECT_GT(result.desired_acceleration_world.x(), 0.0);
  EXPECT_GT(result.desired_acceleration_world.y(), 0.0);
  EXPECT_NEAR(
    result.desired_acceleration_world.y() / result.desired_acceleration_world.x(), 2.0,
    kTolerance);
  EXPECT_TRUE(std::isfinite(result.desired_roll));
  EXPECT_TRUE(std::isfinite(result.desired_pitch));
  EXPECT_TRUE(result.desired_orientation_body_to_world.coeffs().array().isFinite().all());
}

TEST(HorizontalPositionController, AccelerationMagnitudeLimitPreservesDirection)
{
  drone_controller::HorizontalPositionControllerParameters parameters;
  parameters.max_horizontal_acceleration = 2.0;
  parameters.max_tilt_angle = 0.7;
  const drone_controller::HorizontalPositionController controller(parameters);
  drone_controller::HorizontalPositionControllerInput input;
  input.desired_position_world = Eigen::Vector2d(3.0, 4.0);
  const auto result = controller.compute(input);

  ASSERT_TRUE(result.valid);
  EXPECT_TRUE(result.saturated);
  EXPECT_NEAR(result.desired_acceleration_world.norm(), 2.0, kTolerance);
  EXPECT_NEAR(result.desired_acceleration_world.x(), 1.2, kTolerance);
  EXPECT_NEAR(result.desired_acceleration_world.y(), 1.6, kTolerance);
}

TEST(HorizontalPositionController, TiltLimitBoundsDesiredBodyZ)
{
  drone_controller::HorizontalPositionControllerParameters parameters;
  parameters.max_horizontal_acceleration = 100.0;
  parameters.max_tilt_angle = 0.2;
  const drone_controller::HorizontalPositionController controller(parameters);
  drone_controller::HorizontalPositionControllerInput input;
  input.desired_position_world.x() = 100.0;
  const auto result = controller.compute(input);

  ASSERT_TRUE(result.valid);
  EXPECT_TRUE(result.saturated);
  EXPECT_NEAR(
    result.desired_acceleration_world.norm(),
    parameters.gravity * std::tan(parameters.max_tilt_angle), kTolerance);
  const Eigen::Matrix3d rotation =
    result.desired_orientation_body_to_world.toRotationMatrix();
  EXPECT_LE(tilt_from_rotation(rotation), parameters.max_tilt_angle + kTolerance);
}

TEST(HorizontalPositionController, NonFiniteInputReturnsSafeInvalidResult)
{
  const drone_controller::HorizontalPositionController controller;
  for (int case_index = 0; case_index < 5; ++case_index) {
    drone_controller::HorizontalPositionControllerInput input;
    if (case_index == 0) {
      input.desired_yaw = std::numeric_limits<double>::quiet_NaN();
    } else if (case_index == 1) {
      input.desired_position_world.x() = std::numeric_limits<double>::infinity();
    } else if (case_index == 2) {
      input.desired_velocity_world.y() = -std::numeric_limits<double>::infinity();
    } else if (case_index == 3) {
      input.desired_acceleration_world.x() = std::numeric_limits<double>::quiet_NaN();
    } else {
      input.current_velocity_world.x() = std::numeric_limits<double>::quiet_NaN();
    }
    const auto result = controller.compute(input);
    EXPECT_FALSE(result.valid);
    EXPECT_TRUE(result.saturated);
    EXPECT_TRUE(result.desired_acceleration_world.isZero());
    EXPECT_DOUBLE_EQ(result.desired_roll, 0.0);
    EXPECT_DOUBLE_EQ(result.desired_pitch, 0.0);
    EXPECT_TRUE(result.desired_orientation_body_to_world.isApprox(
      Eigen::Quaterniond::Identity(), kTolerance));
  }
}

TEST(HorizontalPositionController, InvalidParametersThrow)
{
  drone_controller::HorizontalPositionControllerParameters parameters;
  parameters.position_kp.x() = -1.0;
  EXPECT_THROW(
    drone_controller::HorizontalPositionController{parameters}, std::invalid_argument);
  parameters = {};
  parameters.velocity_kd.y() = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(
    drone_controller::HorizontalPositionController{parameters}, std::invalid_argument);
  parameters = {};
  parameters.gravity = 0.0;
  EXPECT_THROW(
    drone_controller::HorizontalPositionController{parameters}, std::invalid_argument);
  parameters = {};
  parameters.gravity = std::numeric_limits<double>::infinity();
  EXPECT_THROW(
    drone_controller::HorizontalPositionController{parameters}, std::invalid_argument);
  parameters = {};
  parameters.max_horizontal_acceleration = -1.0;
  EXPECT_THROW(
    drone_controller::HorizontalPositionController{parameters}, std::invalid_argument);
  parameters = {};
  parameters.max_tilt_angle = 0.0;
  EXPECT_THROW(
    drone_controller::HorizontalPositionController{parameters}, std::invalid_argument);
  parameters = {};
  parameters.max_tilt_angle = kPi / 2.0;
  EXPECT_THROW(
    drone_controller::HorizontalPositionController{parameters}, std::invalid_argument);
}

TEST(HorizontalPositionController, ExtremeFiniteInputSaturatesWithoutNonFiniteOutput)
{
  const drone_controller::HorizontalPositionController controller;
  drone_controller::HorizontalPositionControllerInput input;
  const double maximum = std::numeric_limits<double>::max();
  input.desired_position_world = Eigen::Vector2d(maximum, maximum);
  input.current_position_world = Eigen::Vector2d(-maximum, -maximum);
  input.desired_velocity_world = Eigen::Vector2d(maximum, -maximum);
  input.current_velocity_world = Eigen::Vector2d(-maximum, maximum);
  const auto result = controller.compute(input);

  ASSERT_TRUE(result.valid);
  EXPECT_TRUE(result.saturated);
  EXPECT_TRUE(result.desired_acceleration_world.array().isFinite().all());
  EXPECT_LE(result.desired_acceleration_world.norm(), 5.0 + kTolerance);
  EXPECT_TRUE(std::isfinite(result.desired_roll));
  EXPECT_TRUE(std::isfinite(result.desired_pitch));
  EXPECT_TRUE(result.desired_orientation_body_to_world.coeffs().array().isFinite().all());
}

TEST(HorizontalPositionController, OrientationIsProperAndBodyZMatchesAcceleration)
{
  const drone_controller::HorizontalPositionController controller;
  drone_controller::HorizontalPositionControllerInput input;
  input.desired_position_world = Eigen::Vector2d(1.0, -2.0);
  input.desired_velocity_world = Eigen::Vector2d(0.3, 0.2);
  input.desired_yaw = -0.8;
  const auto result = controller.compute(input);

  ASSERT_TRUE(result.valid);
  const Eigen::Matrix3d rotation =
    result.desired_orientation_body_to_world.toRotationMatrix();
  EXPECT_NEAR(result.desired_orientation_body_to_world.norm(), 1.0, kTolerance);
  EXPECT_TRUE((rotation.transpose() * rotation).isApprox(
    Eigen::Matrix3d::Identity(), kTolerance));
  EXPECT_NEAR(rotation.determinant(), 1.0, kTolerance);

  const Eigen::Vector2d body_z_horizontal = rotation.col(2).head<2>();
  ASSERT_GT(body_z_horizontal.norm(), 0.0);
  EXPECT_GT(body_z_horizontal.dot(result.desired_acceleration_world), 0.0);
  EXPECT_NEAR(
    body_z_horizontal.x() / body_z_horizontal.y(),
    result.desired_acceleration_world.x() / result.desired_acceleration_world.y(),
    kTolerance);
}

}  // namespace
