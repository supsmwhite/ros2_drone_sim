#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "drone_controller/attitude/attitude_controller.hpp"

namespace
{

constexpr double kTolerance = 1.0e-12;

Eigen::Quaterniond rotation(const Eigen::Vector3d & axis, const double angle)
{
  return Eigen::Quaterniond(Eigen::AngleAxisd(angle, axis));
}

TEST(AttitudeController, MatchingAttitudeAndRatesProduceZeroTorque)
{
  const drone_controller::AttitudeController controller;
  const auto result = controller.compute({});
  EXPECT_TRUE(result.valid);
  EXPECT_FALSE(result.saturated);
  EXPECT_NEAR(result.torque_body.norm(), 0.0, kTolerance);
}

TEST(AttitudeController, PositiveRollErrorProducesPositiveRollTorque)
{
  const drone_controller::AttitudeController controller;
  drone_controller::AttitudeControllerInput input;
  input.desired_orientation_body_to_world = rotation(Eigen::Vector3d::UnitX(), 0.1);
  const auto result = controller.compute(input);
  EXPECT_TRUE(result.valid);
  EXPECT_GT(result.torque_body.x(), 0.0);
  EXPECT_NEAR(result.torque_body.y(), 0.0, kTolerance);
  EXPECT_NEAR(result.torque_body.z(), 0.0, kTolerance);
}

TEST(AttitudeController, ExistingPositiveRollProducesNegativeRestoringTorque)
{
  const drone_controller::AttitudeController controller;
  drone_controller::AttitudeControllerInput input;
  input.current_orientation_body_to_world = rotation(Eigen::Vector3d::UnitX(), 0.1);
  const auto result = controller.compute(input);
  EXPECT_LT(result.torque_body.x(), 0.0);
  EXPECT_NEAR(result.torque_body.y(), 0.0, kTolerance);
  EXPECT_NEAR(result.torque_body.z(), 0.0, kTolerance);
}

TEST(AttitudeController, PositivePitchAndYawErrorsHaveCorrectSigns)
{
  drone_controller::AttitudeControllerParameters parameters;
  parameters.max_torque.setConstant(10.0);
  const drone_controller::AttitudeController controller(parameters);

  drone_controller::AttitudeControllerInput pitch_input;
  pitch_input.desired_orientation_body_to_world = rotation(Eigen::Vector3d::UnitY(), 0.1);
  const auto pitch = controller.compute(pitch_input);
  EXPECT_GT(pitch.torque_body.y(), 0.0);
  EXPECT_NEAR(pitch.torque_body.x(), 0.0, kTolerance);
  EXPECT_NEAR(pitch.torque_body.z(), 0.0, kTolerance);

  drone_controller::AttitudeControllerInput yaw_input;
  yaw_input.desired_orientation_body_to_world = rotation(Eigen::Vector3d::UnitZ(), 0.1);
  const auto yaw = controller.compute(yaw_input);
  EXPECT_GT(yaw.torque_body.z(), 0.0);
  EXPECT_NEAR(yaw.torque_body.x(), 0.0, kTolerance);
  EXPECT_NEAR(yaw.torque_body.y(), 0.0, kTolerance);
}

TEST(AttitudeController, PositiveCurrentRateProducesNegativeDampingTorque)
{
  const drone_controller::AttitudeController controller;
  drone_controller::AttitudeControllerInput input;
  input.current_angular_velocity_body = Eigen::Vector3d(0.5, 0.4, 0.3);
  const auto result = controller.compute(input);
  EXPECT_LT(result.torque_body.x(), 0.0);
  EXPECT_LT(result.torque_body.y(), 0.0);
  EXPECT_LT(result.torque_body.z(), 0.0);
}

TEST(AttitudeController, QuaternionAndItsNegativeProduceSameTorque)
{
  const drone_controller::AttitudeController controller;
  drone_controller::AttitudeControllerInput positive_input;
  positive_input.desired_orientation_body_to_world =
    rotation(Eigen::Vector3d(1.0, 2.0, 3.0).normalized(), 0.2);
  auto negative_input = positive_input;
  negative_input.desired_orientation_body_to_world.coeffs() *= -1.0;
  const auto positive = controller.compute(positive_input);
  const auto negative = controller.compute(negative_input);
  EXPECT_TRUE(positive.valid);
  EXPECT_TRUE(negative.valid);
  EXPECT_TRUE(positive.torque_body.isApprox(negative.torque_body, kTolerance));
}

TEST(AttitudeController, ValidNonUnitQuaternionsAreNormalized)
{
  const drone_controller::AttitudeController controller;
  drone_controller::AttitudeControllerInput unit_input;
  unit_input.desired_orientation_body_to_world = rotation(Eigen::Vector3d::UnitY(), 0.1);
  auto scaled_input = unit_input;
  scaled_input.desired_orientation_body_to_world.coeffs() *= 5.0;
  scaled_input.current_orientation_body_to_world.coeffs() *= 3.0;
  const auto unit = controller.compute(unit_input);
  const auto scaled = controller.compute(scaled_input);
  EXPECT_TRUE(scaled.valid);
  EXPECT_TRUE(unit.torque_body.isApprox(scaled.torque_body, kTolerance));
}

TEST(AttitudeController, TorqueIsLimitedPerAxis)
{
  drone_controller::AttitudeControllerParameters parameters;
  parameters.attitude_kp.setConstant(100.0);
  parameters.max_torque = Eigen::Vector3d(0.2, 0.3, 0.4);
  const drone_controller::AttitudeController controller(parameters);
  drone_controller::AttitudeControllerInput input;
  input.desired_orientation_body_to_world =
    rotation(Eigen::Vector3d(1.0, -1.0, 1.0).normalized(), 0.5);
  const auto result = controller.compute(input);
  EXPECT_TRUE(result.valid);
  EXPECT_TRUE(result.saturated);
  EXPECT_NEAR(result.torque_body.x(), 0.2, kTolerance);
  EXPECT_NEAR(result.torque_body.y(), -0.3, kTolerance);
  EXPECT_NEAR(result.torque_body.z(), 0.4, kTolerance);
}

TEST(AttitudeController, InvalidInputReturnsZeroTorque)
{
  const drone_controller::AttitudeController controller;
  drone_controller::AttitudeControllerInput input;
  input.desired_orientation_body_to_world.coeffs().setZero();
  auto result = controller.compute(input);
  EXPECT_FALSE(result.valid);
  EXPECT_NEAR(result.torque_body.norm(), 0.0, kTolerance);

  input = {};
  input.current_orientation_body_to_world.x() = std::numeric_limits<double>::quiet_NaN();
  result = controller.compute(input);
  EXPECT_FALSE(result.valid);
  EXPECT_NEAR(result.torque_body.norm(), 0.0, kTolerance);

  input = {};
  input.current_angular_velocity_body.z() = std::numeric_limits<double>::infinity();
  result = controller.compute(input);
  EXPECT_FALSE(result.valid);
  EXPECT_NEAR(result.torque_body.norm(), 0.0, kTolerance);
}

TEST(AttitudeController, InvalidParametersThrow)
{
  drone_controller::AttitudeControllerParameters parameters;
  parameters.attitude_kp.x() = -1.0;
  EXPECT_THROW(drone_controller::AttitudeController{parameters}, std::invalid_argument);
  parameters = {};
  parameters.angular_rate_kd.y() = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(drone_controller::AttitudeController{parameters}, std::invalid_argument);
  parameters = {};
  parameters.max_torque.z() = 0.0;
  EXPECT_THROW(drone_controller::AttitudeController{parameters}, std::invalid_argument);
}

}  // namespace
