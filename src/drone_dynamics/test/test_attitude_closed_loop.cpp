#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <limits>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "drone_controller/attitude/attitude_controller.hpp"
#include "drone_controller/mixer/motor_mixer.hpp"
#include "drone_dynamics/quadrotor_model.hpp"

namespace
{

constexpr double kDynamicsDt = 0.005;
constexpr int kControlDivider = 2;
constexpr double kWarmupSeconds = 2.0;
constexpr double kRunSeconds = 20.0;
constexpr double kSteadyWindowSeconds = 5.0;
constexpr double kRadiansPerSecondToRpm = 60.0 / (2.0 * 3.14159265358979323846);

struct AttitudeClosedLoopMetrics
{
  double max_commanded_axis_attitude{0.0};
  double max_commanded_axis_rate{0.0};
  double max_cross_axis_attitude{0.0};
  double max_yaw{0.0};
  double max_torque{0.0};
  double final_attitude_error{0.0};
  double final_body_rate{0.0};
  double steady_max_attitude_error{0.0};
  double steady_max_body_rate{0.0};
  double minimum_command_rpm{std::numeric_limits<double>::infinity()};
  double maximum_command_rpm{0.0};
  double minimum_actual_rpm{std::numeric_limits<double>::infinity()};
  double maximum_actual_rpm{0.0};
  double attitude_saturation_duration{0.0};
  double mixer_saturation_duration{0.0};
  double steady_attitude_saturation_duration{0.0};
  double steady_mixer_saturation_duration{0.0};
  bool finite{true};
};

Eigen::Vector3d roll_pitch_yaw(const Eigen::Quaterniond & orientation)
{
  const Eigen::Matrix3d rotation = orientation.toRotationMatrix();
  return Eigen::Vector3d(
    std::atan2(rotation(2, 1), rotation(2, 2)),
    std::asin(std::clamp(-rotation(2, 0), -1.0, 1.0)),
    std::atan2(rotation(1, 0), rotation(0, 0)));
}

bool metrics_meet_acceptance(const AttitudeClosedLoopMetrics & metrics)
{
  return metrics.finite &&
         metrics.max_commanded_axis_attitude < 0.15 &&
         metrics.max_commanded_axis_rate < 0.50 &&
         metrics.max_cross_axis_attitude < 0.02 &&
         metrics.max_yaw < 0.02 &&
         metrics.steady_max_attitude_error < 0.003 &&
         metrics.steady_max_body_rate < 0.02 &&
         metrics.steady_attitude_saturation_duration == 0.0 &&
         metrics.steady_mixer_saturation_duration == 0.0 &&
         metrics.minimum_command_rpm > 0.0 &&
         metrics.maximum_command_rpm < 20000.0;
}

AttitudeClosedLoopMetrics run_closed_loop(
  const double desired_roll, const double desired_pitch, const double angular_rate_kd)
{
  drone_dynamics::QuadrotorParameters dynamics_parameters;
  dynamics_parameters.enable_ground_contact = true;
  drone_dynamics::QuadrotorModel model(dynamics_parameters);

  drone_controller::AttitudeControllerParameters attitude_parameters;
  attitude_parameters.attitude_kp = Eigen::Vector3d(4.0, 4.0, 1.0);
  attitude_parameters.angular_rate_kd = Eigen::Vector3d(angular_rate_kd, angular_rate_kd, 0.40);
  attitude_parameters.max_torque = Eigen::Vector3d(1.0, 1.0, 0.20);
  const drone_controller::AttitudeController attitude_controller(attitude_parameters);
  const drone_controller::MotorMixer mixer;

  const Eigen::Quaterniond desired_orientation =
    Eigen::Quaterniond(Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ())) *
    Eigen::Quaterniond(Eigen::AngleAxisd(desired_pitch, Eigen::Vector3d::UnitY())) *
    Eigen::Quaterniond(Eigen::AngleAxisd(desired_roll, Eigen::Vector3d::UnitX()));
  const int warmup_steps = static_cast<int>(kWarmupSeconds / kDynamicsDt);
  const int run_steps = static_cast<int>(kRunSeconds / kDynamicsDt);
  const int steady_start_step =
    run_steps - static_cast<int>(kSteadyWindowSeconds / kDynamicsDt);
  std::array<double, 4> rpm_command{};
  AttitudeClosedLoopMetrics metrics;

  for (int step = -warmup_steps; step < run_steps; ++step) {
    const bool warmup = step < 0;
    const bool update_control = (step + warmup_steps) % kControlDivider == 0;
    if (update_control) {
      drone_controller::AttitudeControllerInput input;
      input.desired_orientation_body_to_world =
        warmup ? Eigen::Quaterniond::Identity() : desired_orientation;
      input.current_orientation_body_to_world = model.state().orientation_body_to_world;
      input.current_angular_velocity_body = model.state().angular_velocity_body;
      const auto attitude_result = attitude_controller.compute(input);

      const double tilt_cosine = std::max(
        0.5, (model.state().orientation_body_to_world * Eigen::Vector3d::UnitZ()).z());
      drone_controller::WrenchCommand wrench;
      wrench.thrust = dynamics_parameters.mass * dynamics_parameters.gravity / tilt_cosine;
      wrench.roll_torque = attitude_result.torque_body.x();
      wrench.pitch_torque = attitude_result.torque_body.y();
      wrench.yaw_torque = attitude_result.torque_body.z();
      const auto mixer_result = mixer.mix(wrench);
      rpm_command = mixer_result.motor_rpm;
      model.set_motor_rpm_command(rpm_command);

      if (!warmup) {
        const double control_dt = kDynamicsDt * kControlDivider;
        metrics.attitude_saturation_duration += attitude_result.saturated ? control_dt : 0.0;
        metrics.mixer_saturation_duration += mixer_result.saturated ? control_dt : 0.0;
        if (step >= steady_start_step) {
          metrics.steady_attitude_saturation_duration +=
            attitude_result.saturated ? control_dt : 0.0;
          metrics.steady_mixer_saturation_duration += mixer_result.saturated ? control_dt : 0.0;
        }
        metrics.max_torque = std::max(metrics.max_torque, attitude_result.torque_body.norm());
        for (const double rpm : rpm_command) {
          metrics.minimum_command_rpm = std::min(metrics.minimum_command_rpm, rpm);
          metrics.maximum_command_rpm = std::max(metrics.maximum_command_rpm, rpm);
        }
        metrics.finite = metrics.finite && attitude_result.valid && mixer_result.valid &&
          attitude_result.torque_body.array().isFinite().all();
      }
    }

    model.step(kDynamicsDt);
    if (warmup) {
      continue;
    }

    const auto & state = model.state();
    const Eigen::Vector3d rpy = roll_pitch_yaw(state.orientation_body_to_world);
    const int commanded_axis = std::abs(desired_roll) > 0.0 ? 0 : 1;
    const int cross_axis = commanded_axis == 0 ? 1 : 0;
    const double desired_axis = commanded_axis == 0 ? desired_roll : desired_pitch;
    const double attitude_error = std::abs(rpy[commanded_axis] - desired_axis);
    const double body_rate = std::abs(state.angular_velocity_body[commanded_axis]);

    metrics.max_commanded_axis_attitude =
      std::max(metrics.max_commanded_axis_attitude, std::abs(rpy[commanded_axis]));
    metrics.max_commanded_axis_rate = std::max(metrics.max_commanded_axis_rate, body_rate);
    metrics.max_cross_axis_attitude =
      std::max(metrics.max_cross_axis_attitude, std::abs(rpy[cross_axis]));
    metrics.max_yaw = std::max(metrics.max_yaw, std::abs(rpy.z()));
    metrics.final_attitude_error = attitude_error;
    metrics.final_body_rate = body_rate;
    if (step >= steady_start_step) {
      metrics.steady_max_attitude_error =
        std::max(metrics.steady_max_attitude_error, attitude_error);
      metrics.steady_max_body_rate = std::max(metrics.steady_max_body_rate, body_rate);
    }
    for (const double speed : state.motor_angular_velocity_rad_s) {
      const double actual_rpm = speed * kRadiansPerSecondToRpm;
      metrics.minimum_actual_rpm = std::min(metrics.minimum_actual_rpm, actual_rpm);
      metrics.maximum_actual_rpm = std::max(metrics.maximum_actual_rpm, actual_rpm);
    }
    metrics.finite = metrics.finite && rpy.array().isFinite().all() &&
      state.angular_velocity_body.array().isFinite().all() &&
      state.orientation_body_to_world.coeffs().array().isFinite().all();
  }
  return metrics;
}

void print_metrics(
  const double kd, const double roll, const double pitch,
  const AttitudeClosedLoopMetrics & metrics)
{
  std::cout << "attitude_closed_loop kd=" << kd << " target=[" << roll << "," << pitch
            << "] max_attitude=" << metrics.max_commanded_axis_attitude
            << " max_rate=" << metrics.max_commanded_axis_rate
            << " steady_error=" << metrics.steady_max_attitude_error
            << " steady_rate=" << metrics.steady_max_body_rate
            << " cross=" << metrics.max_cross_axis_attitude
            << " yaw=" << metrics.max_yaw
            << " command_rpm=[" << metrics.minimum_command_rpm << ","
            << metrics.maximum_command_rpm << "] actual_rpm=["
            << metrics.minimum_actual_rpm << "," << metrics.maximum_actual_rpm << "]"
            << " saturation=[" << metrics.attitude_saturation_duration << ","
            << metrics.mixer_saturation_duration << "] accepted="
            << (metrics_meet_acceptance(metrics) ? "true" : "false") << '\n';
}

void expect_stable_closed_loop(
  const double desired_roll, const double desired_pitch,
  const AttitudeClosedLoopMetrics & metrics)
{
  print_metrics(0.35, desired_roll, desired_pitch, metrics);
  EXPECT_TRUE(metrics.finite);
  EXPECT_LT(metrics.max_commanded_axis_attitude, 0.15);
  EXPECT_LT(metrics.max_commanded_axis_rate, 0.50);
  EXPECT_LT(metrics.max_cross_axis_attitude, 0.02);
  EXPECT_LT(metrics.max_yaw, 0.02);
  EXPECT_LT(metrics.steady_max_attitude_error, 0.003);
  EXPECT_LT(metrics.steady_max_body_rate, 0.02);
  EXPECT_DOUBLE_EQ(metrics.steady_attitude_saturation_duration, 0.0);
  EXPECT_DOUBLE_EQ(metrics.steady_mixer_saturation_duration, 0.0);
  EXPECT_GT(metrics.minimum_command_rpm, 0.0);
  EXPECT_LT(metrics.maximum_command_rpm, 20000.0);
}

TEST(AttitudeClosedLoop, ExistingDampingLacksStabilityMargin)
{
  const auto metrics = run_closed_loop(0.02, 0.0, 0.20);
  print_metrics(0.20, 0.02, 0.0, metrics);
  EXPECT_TRUE(metrics.finite);
  EXPECT_FALSE(metrics_meet_acceptance(metrics));
  EXPECT_GT(metrics.steady_max_attitude_error, 0.003);
  EXPECT_GT(metrics.steady_max_body_rate, 0.02);
}

TEST(AttitudeClosedLoop, PositiveRollIsStableForTwentySeconds)
{
  const auto metrics = run_closed_loop(0.02, 0.0, 0.35);
  expect_stable_closed_loop(0.02, 0.0, metrics);
}

TEST(AttitudeClosedLoop, NegativeRollIsStableForTwentySeconds)
{
  const auto metrics = run_closed_loop(-0.02, 0.0, 0.35);
  expect_stable_closed_loop(-0.02, 0.0, metrics);
}

TEST(AttitudeClosedLoop, PositivePitchIsStableForTwentySeconds)
{
  const auto metrics = run_closed_loop(0.0, 0.02, 0.35);
  expect_stable_closed_loop(0.0, 0.02, metrics);
}

TEST(AttitudeClosedLoop, NegativePitchIsStableForTwentySeconds)
{
  const auto metrics = run_closed_loop(0.0, -0.02, 0.35);
  expect_stable_closed_loop(0.0, -0.02, metrics);
}

TEST(AttitudeClosedLoop, LevelAttitudeIsStableForTwentySeconds)
{
  const auto metrics = run_closed_loop(0.0, 0.0, 0.35);
  expect_stable_closed_loop(0.0, 0.0, metrics);
}

}  // namespace
