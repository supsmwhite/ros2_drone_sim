#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "drone_controller/altitude/altitude_controller.hpp"

namespace
{

constexpr double kTolerance = 1.0e-10;

Eigen::Quaterniond pitch_orientation(const double angle)
{
  return Eigen::Quaterniond(Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitY()));
}

TEST(AltitudeController, BalancedHoverProducesWeight)
{
  const drone_controller::AltitudeController controller;
  const auto result = controller.compute({});
  EXPECT_TRUE(result.valid);
  EXPECT_FALSE(result.saturated);
  EXPECT_NEAR(result.commanded_vertical_acceleration, 0.0, kTolerance);
  EXPECT_NEAR(result.collective_thrust, 9.80665, kTolerance);
}

TEST(AltitudeController, HigherTargetIncreasesThrust)
{
  const drone_controller::AltitudeController controller;
  drone_controller::AltitudeControllerInput input;
  input.desired_altitude = 0.5;
  const auto result = controller.compute(input);
  EXPECT_GT(result.collective_thrust, 9.80665);
}

TEST(AltitudeController, LowerTargetDecreasesButNeverNegatesThrust)
{
  const drone_controller::AltitudeController controller;
  drone_controller::AltitudeControllerInput input;
  input.desired_altitude = -0.5;
  const auto result = controller.compute(input);
  EXPECT_LT(result.collective_thrust, 9.80665);
  EXPECT_GE(result.collective_thrust, 0.0);
}

TEST(AltitudeController, UpwardVelocityProducesBrakingReduction)
{
  const drone_controller::AltitudeController controller;
  drone_controller::AltitudeControllerInput input;
  input.current_vertical_velocity = 0.5;
  const auto result = controller.compute(input);
  EXPECT_LT(result.collective_thrust, 9.80665);
}

TEST(AltitudeController, DownwardVelocityIncreasesThrust)
{
  const drone_controller::AltitudeController controller;
  drone_controller::AltitudeControllerInput input;
  input.current_vertical_velocity = -0.5;
  const auto result = controller.compute(input);
  EXPECT_GT(result.collective_thrust, 9.80665);
}

TEST(AltitudeController, PositiveAccelerationFeedforwardIncreasesThrust)
{
  const drone_controller::AltitudeController controller;
  drone_controller::AltitudeControllerInput input;
  input.desired_vertical_acceleration = 1.0;
  const auto result = controller.compute(input);
  EXPECT_NEAR(result.collective_thrust, 10.80665, kTolerance);
}

TEST(AltitudeController, SixtyDegreeTiltApproximatelyDoublesThrust)
{
  drone_controller::AltitudeControllerParameters parameters;
  parameters.min_tilt_cosine = 0.25;
  const drone_controller::AltitudeController controller(parameters);
  const auto horizontal = controller.compute({});
  drone_controller::AltitudeControllerInput tilted_input;
  tilted_input.current_orientation_body_to_world =
    pitch_orientation(std::acos(-1.0) / 3.0);
  const auto tilted = controller.compute(tilted_input);
  EXPECT_TRUE(tilted.valid);
  EXPECT_FALSE(tilted.saturated);
  EXPECT_NEAR(tilted.collective_thrust, 2.0 * horizontal.collective_thrust, kTolerance);
}

TEST(AltitudeController, NearNinetyDegreeTiltUsesSafeCosine)
{
  const drone_controller::AltitudeController controller;
  drone_controller::AltitudeControllerInput input;
  input.current_orientation_body_to_world = pitch_orientation(std::acos(0.1));
  const auto result = controller.compute(input);
  EXPECT_TRUE(result.valid);
  EXPECT_TRUE(result.saturated);
  EXPECT_TRUE(std::isfinite(result.collective_thrust));
  EXPECT_NEAR(result.collective_thrust, 9.80665 / 0.5, kTolerance);
}

TEST(AltitudeController, InvertedAttitudeReturnsInvalidZeroThrust)
{
  const drone_controller::AltitudeController controller;
  drone_controller::AltitudeControllerInput input;
  input.current_orientation_body_to_world = pitch_orientation(std::acos(-1.0));
  const auto result = controller.compute(input);
  EXPECT_FALSE(result.valid);
  EXPECT_TRUE(result.saturated);
  EXPECT_DOUBLE_EQ(result.collective_thrust, 0.0);
}

TEST(AltitudeController, AccelerationAndThrustAreSaturated)
{
  drone_controller::AltitudeControllerParameters parameters;
  parameters.max_upward_acceleration = 2.0;
  parameters.max_collective_thrust = 10.0;
  const drone_controller::AltitudeController controller(parameters);
  drone_controller::AltitudeControllerInput input;
  input.desired_altitude = 100.0;
  const auto result = controller.compute(input);
  EXPECT_TRUE(result.valid);
  EXPECT_TRUE(result.saturated);
  EXPECT_NEAR(result.commanded_vertical_acceleration, 2.0, kTolerance);
  EXPECT_NEAR(result.collective_thrust, 10.0, kTolerance);
}

TEST(AltitudeController, ValidNonUnitQuaternionIsNormalized)
{
  const drone_controller::AltitudeController controller;
  drone_controller::AltitudeControllerInput unit_input;
  unit_input.current_orientation_body_to_world = pitch_orientation(0.4);
  auto scaled_input = unit_input;
  scaled_input.current_orientation_body_to_world.coeffs() *= 7.0;
  const auto unit = controller.compute(unit_input);
  const auto scaled = controller.compute(scaled_input);
  EXPECT_TRUE(scaled.valid);
  EXPECT_NEAR(scaled.collective_thrust, unit.collective_thrust, kTolerance);
}

TEST(AltitudeController, InvalidInputReturnsSafeResult)
{
  const drone_controller::AltitudeController controller;
  drone_controller::AltitudeControllerInput input;
  input.desired_altitude = std::numeric_limits<double>::quiet_NaN();
  auto result = controller.compute(input);
  EXPECT_FALSE(result.valid);
  EXPECT_TRUE(result.saturated);
  EXPECT_DOUBLE_EQ(result.collective_thrust, 0.0);

  input = {};
  input.current_vertical_velocity = std::numeric_limits<double>::infinity();
  result = controller.compute(input);
  EXPECT_FALSE(result.valid);
  EXPECT_TRUE(result.saturated);

  input = {};
  input.current_orientation_body_to_world.coeffs().setZero();
  result = controller.compute(input);
  EXPECT_FALSE(result.valid);
  EXPECT_TRUE(result.saturated);
  EXPECT_DOUBLE_EQ(result.collective_thrust, 0.0);
}

TEST(AltitudeController, InvalidParametersThrow)
{
  drone_controller::AltitudeControllerParameters parameters;
  parameters.mass = 0.0;
  EXPECT_THROW(drone_controller::AltitudeController{parameters}, std::invalid_argument);
  parameters = {};
  parameters.gravity = std::numeric_limits<double>::infinity();
  EXPECT_THROW(drone_controller::AltitudeController{parameters}, std::invalid_argument);
  parameters = {};
  parameters.altitude_kp = -1.0;
  EXPECT_THROW(drone_controller::AltitudeController{parameters}, std::invalid_argument);
  parameters = {};
  parameters.max_downward_acceleration = -1.0;
  EXPECT_THROW(drone_controller::AltitudeController{parameters}, std::invalid_argument);
  parameters = {};
  parameters.min_collective_thrust = parameters.max_collective_thrust;
  EXPECT_THROW(drone_controller::AltitudeController{parameters}, std::invalid_argument);
  parameters = {};
  parameters.min_tilt_cosine = 1.1;
  EXPECT_THROW(drone_controller::AltitudeController{parameters}, std::invalid_argument);
}

}  // namespace
