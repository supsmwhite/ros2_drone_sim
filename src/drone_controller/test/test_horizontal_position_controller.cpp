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

drone_controller::HorizontalPositionControllerParameters integral_parameters()
{
  drone_controller::HorizontalPositionControllerParameters parameters;
  parameters.position_kp = Eigen::Vector2d(0.4, 0.4);
  parameters.velocity_kd = Eigen::Vector2d(1.2, 1.2);
  parameters.enable_integral = true;
  parameters.position_ki = Eigen::Vector2d(0.1, 0.1);
  parameters.integral_acceleration_limit = 0.35;
  parameters.anti_windup_gain = 1.0;
  parameters.integrator_unload_gain = 2.0;
  parameters.integral_capture_radius = 0.5;
  parameters.max_horizontal_acceleration = 0.8;
  parameters.max_tilt_angle = 0.15;
  return parameters;
}

TEST(HorizontalPositionControllerIntegral, ZeroKiIsExactlyPdCompatible)
{
  auto parameters = integral_parameters();
  parameters.position_ki.setZero();
  drone_controller::HorizontalPositionController controller(parameters);
  drone_controller::HorizontalPositionControllerInput input;
  input.desired_position_world = Eigen::Vector2d(0.2, -0.1);
  input.current_velocity_world = Eigen::Vector2d(0.03, -0.04);
  input.desired_acceleration_world = Eigen::Vector2d(0.01, 0.02);
  const auto pd = controller.compute(input);
  const auto pi = controller.compute(input, 0.01, true);
  EXPECT_EQ(pd.desired_acceleration_world.x(), pi.desired_acceleration_world.x());
  EXPECT_EQ(pd.desired_acceleration_world.y(), pi.desired_acceleration_world.y());
  EXPECT_TRUE(controller.integral_acceleration_world().isZero());
}

TEST(HorizontalPositionControllerIntegral, ConstantErrorAccumulatesAcceleration)
{
  drone_controller::HorizontalPositionController controller(integral_parameters());
  drone_controller::HorizontalPositionControllerInput input;
  input.desired_position_world.x() = 0.2;
  for (int index = 0; index < 100; ++index) {
    ASSERT_TRUE(controller.compute(input, 0.01, true).valid);
  }
  EXPECT_NEAR(controller.integral_acceleration_world().x(), 0.02, 1.0e-12);
  EXPECT_NEAR(controller.integral_acceleration_world().y(), 0.0, kTolerance);
}

TEST(HorizontalPositionControllerIntegral, AxisSignsFollowPositionError)
{
  drone_controller::HorizontalPositionController controller(integral_parameters());
  drone_controller::HorizontalPositionControllerInput input;
  input.desired_position_world = Eigen::Vector2d(-0.2, 0.3);
  ASSERT_TRUE(controller.compute(input, 0.1, true).valid);
  EXPECT_LT(controller.integral_acceleration_world().x(), 0.0);
  EXPECT_GT(controller.integral_acceleration_world().y(), 0.0);
}

TEST(HorizontalPositionControllerIntegral, IndependentIntegralVectorLimitIsEnforced)
{
  auto parameters = integral_parameters();
  parameters.position_ki = Eigen::Vector2d(10.0, 10.0);
  parameters.integral_acceleration_limit = 0.25;
  drone_controller::HorizontalPositionController controller(parameters);
  drone_controller::HorizontalPositionControllerInput input;
  input.desired_position_world = Eigen::Vector2d(0.3, 0.3);
  ASSERT_TRUE(controller.compute(input, 1.0, true).valid);
  EXPECT_NEAR(controller.integral_acceleration_world().norm(), 0.25, kTolerance);
}

TEST(HorizontalPositionControllerIntegral, BackCalculationActsDuringSaturation)
{
  auto parameters = integral_parameters();
  parameters.integral_capture_radius = 10.0;
  drone_controller::HorizontalPositionController controller(parameters);
  drone_controller::HorizontalPositionControllerInput input;
  input.desired_position_world.x() = 4.0;
  const auto result = controller.compute(input, 0.1, true);
  ASSERT_TRUE(result.valid);
  EXPECT_TRUE(result.saturated);
  EXPECT_TRUE(result.saturation_backcalc_active);
  EXPECT_FALSE(result.integrator_unloading_active);
  EXPECT_TRUE(result.anti_windup_active);
  EXPECT_LT(controller.integral_acceleration_world().x(), 0.0);
}

TEST(HorizontalPositionControllerIntegral, OpposingErrorUnloadsStoredIntegral)
{
  drone_controller::HorizontalPositionController controller(integral_parameters());
  drone_controller::HorizontalPositionControllerInput input;
  input.desired_position_world.x() = 0.4;
  for (int index = 0; index < 100; ++index) {
    controller.compute(input, 0.01, true);
  }
  const double stored = controller.integral_acceleration_world().x();
  input.desired_position_world.x() = -0.4;
  const auto result = controller.compute(input, 0.01, true);
  EXPECT_TRUE(result.integrator_unloading_active);
  EXPECT_TRUE(result.anti_windup_active);
  EXPECT_GT(controller.integral_acceleration_world().x(), 0.0);
  EXPECT_LT(controller.integral_acceleration_world().norm(), std::abs(stored));
}

TEST(HorizontalPositionControllerIntegral, RecoveringMotionAccumulatesSameSideBias)
{
  drone_controller::HorizontalPositionController controller(integral_parameters());
  drone_controller::HorizontalPositionControllerInput input;
  input.current_position_world.x() = 0.2;
  input.current_velocity_world.x() = -0.1;
  const auto result = controller.compute(input, 0.1, true);
  EXPECT_FALSE(result.integral_frozen);
  EXPECT_LT(controller.integral_acceleration_world().x(), 0.0);
}

TEST(HorizontalPositionControllerIntegral, ExplicitOpposingErrorUnloadingPreservesDirection)
{
  drone_controller::HorizontalPositionController controller(integral_parameters());
  drone_controller::HorizontalPositionControllerInput input;
  input.desired_position_world = Eigen::Vector2d(-0.2, 0.1);
  controller.compute(input, 1.0, true);
  const Eigen::Vector2d stored = controller.integral_acceleration_world();
  ASSERT_GT(stored.norm(), 0.0);
  EXPECT_TRUE(controller.unwind_integrator_if_opposing_error(-stored, 0.1));
  const Eigen::Vector2d unloaded = controller.integral_acceleration_world();
  EXPECT_LT(unloaded.norm(), stored.norm());
  EXPECT_GT(unloaded.dot(stored), 0.0);
  EXPECT_NEAR(unloaded.x() / unloaded.y(), stored.x() / stored.y(), kTolerance);
}

TEST(HorizontalPositionControllerIntegral, SameDirectionErrorDoesNotActivelyUnload)
{
  drone_controller::HorizontalPositionController controller(integral_parameters());
  drone_controller::HorizontalPositionControllerInput input;
  input.desired_position_world.x() = 0.2;
  controller.compute(input, 0.5, true);
  const double stored = controller.integral_acceleration_world().norm();
  const auto result = controller.compute(input, 0.1, true);
  EXPECT_FALSE(result.integrator_unloading_active);
  EXPECT_GT(controller.integral_acceleration_world().norm(), stored);
}

TEST(HorizontalPositionControllerIntegral, ZeroErrorHoldsStoredIntegral)
{
  drone_controller::HorizontalPositionController controller(integral_parameters());
  drone_controller::HorizontalPositionControllerInput input;
  input.desired_position_world.x() = 0.2;
  controller.compute(input, 0.5, true);
  const Eigen::Vector2d stored = controller.integral_acceleration_world();
  input.desired_position_world.setZero();
  const auto result = controller.compute(input, 0.1, true);
  EXPECT_FALSE(result.integrator_unloading_active);
  EXPECT_TRUE(controller.integral_acceleration_world().isApprox(stored, kTolerance));
}

TEST(HorizontalPositionControllerIntegral, ZeroUnloadGainDisablesActiveUnloading)
{
  auto parameters = integral_parameters();
  parameters.integrator_unload_gain = 0.0;
  drone_controller::HorizontalPositionController controller(parameters);
  drone_controller::HorizontalPositionControllerInput input;
  input.desired_position_world.x() = 0.2;
  controller.compute(input, 0.5, true);
  const Eigen::Vector2d stored = controller.integral_acceleration_world();
  input.desired_position_world.x() = -0.2;
  EXPECT_FALSE(controller.unwind_integrator_if_opposing_error(
    input.desired_position_world, 0.1));
  EXPECT_TRUE(controller.integral_acceleration_world().isApprox(stored, kTolerance));
}

TEST(HorizontalPositionControllerIntegral, UnsaturatedOutputDoesNotActivateBackCalculation)
{
  drone_controller::HorizontalPositionController controller(integral_parameters());
  drone_controller::HorizontalPositionControllerInput input;
  input.desired_position_world.x() = 0.1;
  const auto result = controller.compute(input, 0.1, true);
  ASSERT_TRUE(result.valid);
  EXPECT_FALSE(result.saturated);
  EXPECT_FALSE(result.saturation_backcalc_active);
}

TEST(HorizontalPositionControllerIntegral, InvalidExplicitUnloadDoesNotPolluteState)
{
  drone_controller::HorizontalPositionController controller(integral_parameters());
  drone_controller::HorizontalPositionControllerInput input;
  input.desired_position_world.x() = 0.2;
  controller.compute(input, 0.5, true);
  const Eigen::Vector2d stored = controller.integral_acceleration_world();
  Eigen::Vector2d invalid_error(-1.0, std::numeric_limits<double>::quiet_NaN());
  EXPECT_FALSE(controller.unwind_integrator_if_opposing_error(invalid_error, 0.1));
  EXPECT_FALSE(controller.unwind_integrator_if_opposing_error(-stored, 0.0));
  EXPECT_FALSE(controller.unwind_integrator_if_opposing_error(
    -stored, std::numeric_limits<double>::infinity()));
  EXPECT_TRUE(controller.integral_acceleration_world().isApprox(stored, kTolerance));
}

TEST(HorizontalPositionControllerIntegral, ResetClearsState)
{
  drone_controller::HorizontalPositionController controller(integral_parameters());
  drone_controller::HorizontalPositionControllerInput input;
  input.desired_position_world.x() = 0.2;
  controller.compute(input, 0.1, true);
  ASSERT_GT(controller.integral_acceleration_world().norm(), 0.0);
  controller.reset_integrator();
  EXPECT_TRUE(controller.integral_acceleration_world().isZero());
}

TEST(HorizontalPositionControllerIntegral, DisabledCycleFreezesExistingState)
{
  drone_controller::HorizontalPositionController controller(integral_parameters());
  drone_controller::HorizontalPositionControllerInput input;
  input.desired_position_world.x() = 0.2;
  controller.compute(input, 0.1, true);
  const Eigen::Vector2d stored = controller.integral_acceleration_world();
  const auto result = controller.compute(input, 1.0, false);
  EXPECT_TRUE(result.integral_frozen);
  EXPECT_TRUE(controller.integral_acceleration_world().isApprox(stored, kTolerance));
}

TEST(HorizontalPositionControllerIntegral, InvalidInputDoesNotPolluteState)
{
  drone_controller::HorizontalPositionController controller(integral_parameters());
  drone_controller::HorizontalPositionControllerInput input;
  input.desired_position_world.x() = 0.2;
  controller.compute(input, 0.1, true);
  const Eigen::Vector2d stored = controller.integral_acceleration_world();
  input.current_position_world.y() = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(controller.compute(input, 0.1, true).valid);
  EXPECT_TRUE(controller.integral_acceleration_world().isApprox(stored, kTolerance));
}

TEST(HorizontalPositionControllerIntegral, InvalidDtDoesNotPolluteState)
{
  drone_controller::HorizontalPositionController controller(integral_parameters());
  drone_controller::HorizontalPositionControllerInput input;
  input.desired_position_world.x() = 0.2;
  controller.compute(input, 0.1, true);
  const Eigen::Vector2d stored = controller.integral_acceleration_world();
  for (const double dt : {0.0, -0.1, std::numeric_limits<double>::quiet_NaN()}) {
    EXPECT_FALSE(controller.compute(input, dt, true).valid);
    EXPECT_TRUE(controller.integral_acceleration_world().isApprox(stored, kTolerance));
  }
}

TEST(HorizontalPositionControllerIntegral, CaptureRadiusFreezesPositionAccumulation)
{
  drone_controller::HorizontalPositionController controller(integral_parameters());
  drone_controller::HorizontalPositionControllerInput input;
  input.desired_position_world.x() = 0.6;
  const auto result = controller.compute(input, 1.0, true);
  EXPECT_TRUE(result.integral_frozen);
  EXPECT_TRUE(controller.integral_acceleration_world().isZero());
}

TEST(HorizontalPositionControllerIntegral, StableSmallErrorRemainsFiniteAndBounded)
{
  drone_controller::HorizontalPositionController controller(integral_parameters());
  drone_controller::HorizontalPositionControllerInput input;
  input.desired_position_world = Eigen::Vector2d(0.01, -0.01);
  for (int index = 0; index < 10000; ++index) {
    ASSERT_TRUE(controller.compute(input, 0.01, true).valid);
  }
  EXPECT_TRUE(controller.integral_acceleration_world().allFinite());
  EXPECT_LE(controller.integral_acceleration_world().norm(), 0.35 + kTolerance);
}

TEST(HorizontalPositionControllerIntegral, GloballyDisabledIntegralKeepsExactPdBehavior)
{
  auto parameters = integral_parameters();
  parameters.enable_integral = false;
  drone_controller::HorizontalPositionController controller(parameters);
  drone_controller::HorizontalPositionControllerInput input;
  input.desired_position_world = Eigen::Vector2d(0.2, 0.1);
  const auto pd = controller.compute(input);
  const auto disabled = controller.compute(input, 0.2, true);
  EXPECT_EQ(pd.desired_acceleration_world.x(), disabled.desired_acceleration_world.x());
  EXPECT_EQ(pd.desired_acceleration_world.y(), disabled.desired_acceleration_world.y());
  EXPECT_FALSE(disabled.integral_enabled);
}

}  // namespace
