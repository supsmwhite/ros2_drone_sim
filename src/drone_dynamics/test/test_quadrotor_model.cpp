#include <array>
#include <cmath>
#include <iostream>
#include <limits>

#include <gtest/gtest.h>

#include "drone_dynamics/quadrotor_model.hpp"

namespace drone_dynamics
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

QuadrotorParameters fast_motor_parameters()
{
  QuadrotorParameters parameters;
  parameters.motor_time_constant = 1.0e-4;
  return parameters;
}

double nominal_hover_rpm(const QuadrotorParameters & parameters)
{
  const double angular_velocity = std::sqrt(
    parameters.mass * parameters.gravity / (4.0 * parameters.thrust_coefficient));
  return angular_velocity * 30.0 / kPi;
}

TEST(QuadrotorModelTest, ZeroRpmFreeFall)
{
  QuadrotorModel model(fast_motor_parameters());
  model.set_motor_rpm_command({0.0, 0.0, 0.0, 0.0});

  constexpr double dt = 0.01;
  for (int step = 0; step < 100; ++step) {
    model.step(dt);
  }

  const QuadrotorState & state = model.state();
  std::cout << "zero_rpm: z=" << state.position_world.z()
            << " vz=" << state.velocity_world.z()
            << " quaternion_norm=" << state.orientation_body_to_world.norm()
            << " angular_rate_norm=" << state.angular_velocity_body.norm() << '\n';

  EXPECT_LT(state.position_world.z(), -4.9);
  EXPECT_NEAR(state.velocity_world.z(), -model.parameters().gravity, 1.0e-10);
  EXPECT_NEAR(state.angular_velocity_body.norm(), 0.0, 1.0e-12);
  EXPECT_NEAR(state.orientation_body_to_world.norm(), 1.0, 1.0e-12);
  EXPECT_NEAR(model.specific_force_body().norm(), 0.0, 1.0e-10);
}

TEST(QuadrotorModelTest, ZeroExternalWrenchPreservesNominalHoverBehavior)
{
  const QuadrotorParameters parameters = fast_motor_parameters();
  QuadrotorModel baseline(parameters);
  QuadrotorModel explicit_zero(parameters);
  const double hover_rpm = nominal_hover_rpm(parameters);
  baseline.set_motor_rpm_command({hover_rpm, hover_rpm, hover_rpm, hover_rpm});
  explicit_zero.set_motor_rpm_command({hover_rpm, hover_rpm, hover_rpm, hover_rpm});
  explicit_zero.set_external_wrench(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

  for (int step = 0; step < 100; ++step) {
    baseline.step(0.01);
    explicit_zero.step(0.01);
  }
  EXPECT_TRUE(baseline.state().position_world.isApprox(
    explicit_zero.state().position_world, 0.0));
  EXPECT_TRUE(baseline.state().velocity_world.isApprox(
    explicit_zero.state().velocity_world, 0.0));
  EXPECT_TRUE(baseline.state().angular_velocity_body.isApprox(
    explicit_zero.state().angular_velocity_body, 0.0));
}

TEST(QuadrotorModelTest, WorldForceProducesMatchingHorizontalAccelerationWithoutAngularAcceleration)
{
  QuadrotorParameters parameters = fast_motor_parameters();
  QuadrotorModel positive(parameters);
  QuadrotorModel negative(parameters);
  const double hover_rpm = nominal_hover_rpm(parameters);
  positive.set_motor_rpm_command({hover_rpm, hover_rpm, hover_rpm, hover_rpm});
  negative.set_motor_rpm_command({hover_rpm, hover_rpm, hover_rpm, hover_rpm});
  positive.set_external_wrench(Eigen::Vector3d(0.8, 0.0, 0.0));
  negative.set_external_wrench(Eigen::Vector3d(-0.8, 0.0, 0.0));

  positive.step(0.01);
  negative.step(0.01);
  EXPECT_NEAR(positive.linear_acceleration_world().x(), 0.8 / parameters.mass, 1.0e-12);
  EXPECT_NEAR(negative.linear_acceleration_world().x(), -0.8 / parameters.mass, 1.0e-12);
  EXPECT_NEAR(positive.state().angular_velocity_body.norm(), 0.0, 1.0e-12);
  EXPECT_NEAR(negative.state().angular_velocity_body.norm(), 0.0, 1.0e-12);
}

TEST(QuadrotorModelTest, NonFiniteExternalWrenchIsRejected)
{
  QuadrotorModel model(fast_motor_parameters());
  EXPECT_THROW(
    model.set_external_wrench(
      Eigen::Vector3d(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0)),
    std::invalid_argument);
  EXPECT_THROW(
    model.set_external_wrench(
      Eigen::Vector3d::Zero(),
      Eigen::Vector3d(0.0, std::numeric_limits<double>::infinity(), 0.0)),
    std::invalid_argument);
}

TEST(QuadrotorModelTest, GroundContactKeepsZeroRpmVehicleStationary)
{
  QuadrotorParameters parameters = fast_motor_parameters();
  parameters.enable_ground_contact = true;
  parameters.ground_z = 0.0;
  QuadrotorModel model(parameters);

  EXPECT_DOUBLE_EQ(model.state().position_world.z(), parameters.ground_z);
  EXPECT_NEAR(model.linear_acceleration_world().norm(), 0.0, 1.0e-12);

  constexpr double dt = 0.01;
  for (int step = 0; step < 100; ++step) {
    model.step(dt);
    EXPECT_GE(model.state().position_world.z(), parameters.ground_z);
  }

  const QuadrotorState & state = model.state();
  std::cout << "ground_static: z=" << state.position_world.z()
            << " vz=" << state.velocity_world.z()
            << " az=" << model.linear_acceleration_world().z()
            << " imu_specific_force_z=" << model.specific_force_body().z() << '\n';

  EXPECT_NEAR(state.position_world.z(), parameters.ground_z, 1.0e-12);
  EXPECT_NEAR(state.velocity_world.z(), 0.0, 1.0e-12);
  EXPECT_NEAR(state.angular_velocity_body.norm(), 0.0, 1.0e-12);
  EXPECT_NEAR(state.orientation_body_to_world.norm(), 1.0, 1.0e-12);
  EXPECT_NEAR(model.linear_acceleration_world().z(), 0.0, 1.0e-12);
  EXPECT_NEAR(model.specific_force_body().z(), parameters.gravity, 1.0e-10);
}

TEST(QuadrotorModelTest, GroundContactAllowsTakeoffAboveHoverRpm)
{
  QuadrotorParameters parameters;
  parameters.enable_ground_contact = true;
  parameters.ground_z = 0.0;
  QuadrotorModel model(parameters);
  model.set_motor_rpm_command({12000.0, 12000.0, 12000.0, 12000.0});

  constexpr double dt = 0.005;
  for (int step = 0; step < 400; ++step) {
    model.step(dt);
    EXPECT_GE(model.state().position_world.z(), parameters.ground_z);
  }

  const QuadrotorState & state = model.state();
  std::cout << "ground_takeoff: z=" << state.position_world.z()
            << " vz=" << state.velocity_world.z()
            << " az=" << model.linear_acceleration_world().z() << '\n';

  EXPECT_GT(state.position_world.z(), parameters.ground_z);
  EXPECT_GT(state.velocity_world.z(), 0.0);
}

TEST(QuadrotorModelTest, AirborneVehicleFallsAndStopsAtConfiguredGround)
{
  QuadrotorParameters parameters;
  parameters.enable_ground_contact = true;
  parameters.ground_z = 0.35;
  parameters.motor_time_constant = 0.01;
  QuadrotorModel model(parameters);

  // 先通过模型自身推力起飞，避免为测试暴露可任意修改内部状态的接口。
  model.set_motor_rpm_command({12000.0, 12000.0, 12000.0, 12000.0});
  constexpr double dt = 0.01;
  for (int step = 0; step < 100; ++step) {
    model.step(dt);
  }
  ASSERT_GT(model.state().position_world.z(), parameters.ground_z);

  // 关闭电机后自由下落；每一步都检查没有穿透配置的非零地面高度。
  model.set_motor_rpm_command({0.0, 0.0, 0.0, 0.0});
  for (int step = 0; step < 800; ++step) {
    model.step(dt);
    EXPECT_GE(model.state().position_world.z(), parameters.ground_z);
  }

  const QuadrotorState & state = model.state();
  std::cout << "ground_landing: ground_z=" << parameters.ground_z
            << " final_z=" << state.position_world.z()
            << " final_vz=" << state.velocity_world.z()
            << " final_az=" << model.linear_acceleration_world().z() << '\n';

  EXPECT_NEAR(state.position_world.z(), parameters.ground_z, 1.0e-12);
  EXPECT_NEAR(state.velocity_world.z(), 0.0, 1.0e-12);
  EXPECT_NEAR(model.linear_acceleration_world().z(), 0.0, 1.0e-12);
  EXPECT_NEAR(model.specific_force_body().z(), parameters.gravity, 1.0e-10);
}

TEST(QuadrotorModelTest, GroundContactDoesNotClearHorizontalVelocity)
{
  QuadrotorParameters parameters;
  parameters.enable_ground_contact = true;
  parameters.ground_z = 0.0;
  parameters.motor_time_constant = 0.01;
  QuadrotorModel model(parameters);
  constexpr double dt = 0.01;

  // 对称推力先让无人机离地。
  model.set_motor_rpm_command({12000.0, 12000.0, 12000.0, 12000.0});
  for (int step = 0; step < 100; ++step) {
    model.step(dt);
  }
  ASSERT_GT(model.state().position_world.z(), parameters.ground_z);

  // 后侧电机较高，短暂正 pitch 后产生非零水平速度。
  model.set_motor_rpm_command({9000.0, 12000.0, 12000.0, 9000.0});
  for (int step = 0; step < 20; ++step) {
    model.step(dt);
  }
  ASSERT_GT(std::abs(model.state().velocity_world.x()), 1.0e-3);

  // 零 RPM 下落，记录第一次接触地面时的水平速度。
  model.set_motor_rpm_command({0.0, 0.0, 0.0, 0.0});
  Eigen::Vector2d horizontal_velocity_at_landing = Eigen::Vector2d::Zero();
  bool landed = false;
  for (int step = 0; step < 1000; ++step) {
    model.step(dt);
    if (model.state().position_world.z() == parameters.ground_z &&
      model.state().velocity_world.z() == 0.0)
    {
      horizontal_velocity_at_landing = model.state().velocity_world.head<2>();
      landed = true;
      break;
    }
  }
  ASSERT_TRUE(landed);
  ASSERT_GT(horizontal_velocity_at_landing.norm(), 1.0e-3);

  // 在地面继续运行；无摩擦地面不得改变 x/y 速度。
  for (int step = 0; step < 100; ++step) {
    model.step(dt);
  }

  std::cout << "ground_horizontal_velocity: landing="
            << horizontal_velocity_at_landing.transpose()
            << " final=" << model.state().velocity_world.head<2>().transpose() << '\n';
  EXPECT_TRUE(model.state().velocity_world.head<2>().isApprox(
    horizontal_velocity_at_landing, 1.0e-10));
}

TEST(QuadrotorModelTest, NonFiniteGroundHeightIsRejected)
{
  QuadrotorParameters parameters;
  parameters.ground_z = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(QuadrotorModel model(parameters), std::invalid_argument);
}

TEST(QuadrotorModelTest, SymmetricThrustHasZeroTorqueAndExpectedVerticalAcceleration)
{
  const QuadrotorParameters parameters = fast_motor_parameters();
  const double hover_rpm = nominal_hover_rpm(parameters);

  QuadrotorModel low_model(parameters);
  low_model.set_motor_rpm_command({0.8 * hover_rpm, 0.8 * hover_rpm,
    0.8 * hover_rpm, 0.8 * hover_rpm});
  low_model.step(0.01);

  QuadrotorModel hover_model(parameters);
  hover_model.set_motor_rpm_command({hover_rpm, hover_rpm, hover_rpm, hover_rpm});
  hover_model.step(0.01);

  QuadrotorModel high_model(parameters);
  high_model.set_motor_rpm_command({1.2 * hover_rpm, 1.2 * hover_rpm,
    1.2 * hover_rpm, 1.2 * hover_rpm});
  high_model.step(0.01);

  std::cout << "symmetric_thrust: hover_rpm=" << hover_rpm
            << " az_low=" << low_model.linear_acceleration_world().z()
            << " az_hover=" << hover_model.linear_acceleration_world().z()
            << " az_high=" << high_model.linear_acceleration_world().z()
            << " hover_torque_norm=" << hover_model.body_wrench().torque.norm() << '\n';

  EXPECT_LT(low_model.linear_acceleration_world().z(), 0.0);
  EXPECT_NEAR(hover_model.linear_acceleration_world().z(), 0.0, 1.0e-10);
  EXPECT_GT(high_model.linear_acceleration_world().z(), 0.0);
  EXPECT_NEAR(hover_model.body_wrench().torque.norm(), 0.0, 1.0e-12);
}

TEST(QuadrotorModelTest, LeftRightAsymmetryProducesPositiveRollOnly)
{
  QuadrotorModel model(fast_motor_parameters());
  model.set_motor_rpm_command({11000.0, 11000.0, 9000.0, 9000.0});
  model.step(0.01);

  const Eigen::Vector3d torque = model.body_wrench().torque;
  std::cout << "roll_response: torque=" << torque.transpose()
            << " angular_velocity=" << model.state().angular_velocity_body.transpose() << '\n';

  EXPECT_GT(torque.x(), 0.0);
  EXPECT_NEAR(torque.y(), 0.0, 1.0e-12);
  EXPECT_NEAR(torque.z(), 0.0, 1.0e-12);
  EXPECT_GT(model.state().angular_velocity_body.x(), 0.0);
}

TEST(QuadrotorModelTest, RearFrontAsymmetryProducesPositivePitchOnly)
{
  QuadrotorModel model(fast_motor_parameters());
  model.set_motor_rpm_command({9000.0, 11000.0, 11000.0, 9000.0});
  model.step(0.01);

  const Eigen::Vector3d torque = model.body_wrench().torque;
  std::cout << "pitch_response: torque=" << torque.transpose()
            << " angular_velocity=" << model.state().angular_velocity_body.transpose() << '\n';

  EXPECT_NEAR(torque.x(), 0.0, 1.0e-12);
  EXPECT_GT(torque.y(), 0.0);
  EXPECT_NEAR(torque.z(), 0.0, 1.0e-12);
  EXPECT_GT(model.state().angular_velocity_body.y(), 0.0);
}

TEST(QuadrotorModelTest, CwCcWAsymmetryProducesYawAtConstantTotalThrust)
{
  constexpr double baseline_rpm = 10000.0;
  constexpr double ccw_rpm = 9000.0;
  const double cw_rpm = std::sqrt(2.0 * baseline_rpm * baseline_rpm - ccw_rpm * ccw_rpm);

  QuadrotorModel baseline_model(fast_motor_parameters());
  baseline_model.set_motor_rpm_command(
    {baseline_rpm, baseline_rpm, baseline_rpm, baseline_rpm});
  baseline_model.step(0.01);

  QuadrotorModel yaw_model(fast_motor_parameters());
  yaw_model.set_motor_rpm_command({ccw_rpm, cw_rpm, ccw_rpm, cw_rpm});
  yaw_model.step(0.01);

  const Eigen::Vector3d torque = yaw_model.body_wrench().torque;
  std::cout << "yaw_response: ccw_rpm=" << ccw_rpm
            << " cw_rpm=" << cw_rpm
            << " torque=" << torque.transpose()
            << " thrust_delta="
            << yaw_model.body_wrench().thrust - baseline_model.body_wrench().thrust << '\n';

  EXPECT_NEAR(torque.x(), 0.0, 1.0e-12);
  EXPECT_NEAR(torque.y(), 0.0, 1.0e-12);
  EXPECT_GT(torque.z(), 0.0);
  EXPECT_NEAR(yaw_model.body_wrench().thrust, baseline_model.body_wrench().thrust, 1.0e-10);
}

TEST(QuadrotorModelTest, MotorResponseUsesRpmLimitsAndStableFirstOrderUpdate)
{
  QuadrotorParameters parameters;
  parameters.motor_time_constant = 0.05;
  parameters.max_rpm = 15000.0;
  QuadrotorModel model(parameters);
  model.set_motor_rpm_command({-100.0, 20000.0, 0.0, 0.0});
  model.step(parameters.motor_time_constant);

  const auto & motor_speed = model.state().motor_angular_velocity_rad_s;
  const double expected_max_motor_speed =
    15000.0 * kPi / 30.0 * (1.0 - std::exp(-1.0));
  std::cout << "motor_response: m1_rad_s=" << motor_speed[0]
            << " m2_rad_s=" << motor_speed[1]
            << " expected_m2_rad_s=" << expected_max_motor_speed << '\n';

  EXPECT_DOUBLE_EQ(motor_speed[0], 0.0);
  EXPECT_NEAR(motor_speed[1], expected_max_motor_speed, 1.0e-10);
}

}  // namespace
}  // namespace drone_dynamics
