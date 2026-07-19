#include <cmath>
#include <limits>
#include <stdexcept>

#include <gtest/gtest.h>

#include "drone_planning/yaw_reference_generator.hpp"

namespace drone_planning
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

YawReferenceParameters fast_parameters()
{
  YawReferenceParameters parameters;
  parameters.mode = YawMode::PathTangent;
  parameters.filter_time_constant = 0.01;
  parameters.max_yaw_rate = 100.0;
  return parameters;
}

double update_many(
  YawReferenceGenerator & generator, const Eigen::Vector3d & position,
  const Eigen::Vector3d & velocity, const Eigen::Vector3d & goal,
  double terminal_yaw, int count = 20, double dt = 0.1)
{
  double result = generator.reference();
  for (int index = 0; index < count; ++index) {
    result = generator.update(position, velocity, goal, terminal_yaw, dt);
  }
  return result;
}

TEST(YawReferenceGenerator, RejectsUnknownMode)
{
  EXPECT_EQ(parse_yaw_mode("fixed"), YawMode::Fixed);
  EXPECT_EQ(parse_yaw_mode("path_tangent"), YawMode::PathTangent);
  EXPECT_THROW(parse_yaw_mode("target_facing"), std::invalid_argument);
}

TEST(YawReferenceGenerator, RejectsInvalidParameters)
{
  auto parameters = fast_parameters();
  parameters.max_yaw_rate = 0.0;
  EXPECT_THROW(YawReferenceGenerator generator(parameters), std::invalid_argument);
  parameters = fast_parameters();
  parameters.terminal_blend_distance = std::numeric_limits<double>::infinity();
  EXPECT_THROW(YawReferenceGenerator generator(parameters), std::invalid_argument);
}

TEST(YawReferenceGenerator, FixedModeAlwaysReturnsConfiguredYaw)
{
  YawReferenceParameters parameters;
  parameters.fixed_yaw = 0.7;
  YawReferenceGenerator generator(parameters);
  generator.initialize(-1.2);
  EXPECT_DOUBLE_EQ(generator.update(
    Eigen::Vector3d::Zero(), Eigen::Vector3d(1.0, 0.0, 0.0),
    Eigen::Vector3d::Ones(), -2.0, 0.02), 0.7);
  EXPECT_DOUBLE_EQ(generator.update(
    Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN()),
    Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity()),
    Eigen::Vector3d::Zero(), std::numeric_limits<double>::quiet_NaN(), -1.0), 0.7);
}

class TangentDirectionTest : public ::testing::TestWithParam<std::pair<Eigen::Vector3d, double>> {};

TEST_P(TangentDirectionTest, FollowsHorizontalVelocityDirection)
{
  YawReferenceGenerator generator(fast_parameters());
  generator.initialize(0.0);
  const auto & [velocity, expected] = GetParam();
  const double result = update_many(
    generator, Eigen::Vector3d::Zero(), velocity, Eigen::Vector3d(10.0, 0.0, 0.0), 0.0);
  EXPECT_NEAR(std::remainder(result - expected, 2.0 * kPi), 0.0, 1.0e-6);
}

INSTANTIATE_TEST_SUITE_P(
  CardinalDirections, TangentDirectionTest,
  ::testing::Values(
    std::make_pair(Eigen::Vector3d(1.0, 0.0, 0.0), 0.0),
    std::make_pair(Eigen::Vector3d(0.0, 1.0, 0.0), 0.5 * kPi),
    std::make_pair(Eigen::Vector3d(-1.0, 0.0, 0.0), kPi),
    std::make_pair(Eigen::Vector3d(0.0, -1.0, 0.0), -0.5 * kPi)));

TEST(YawReferenceGenerator, LowHorizontalSpeedHoldsLastValidTangent)
{
  YawReferenceGenerator generator(fast_parameters());
  generator.initialize(0.3);
  const Eigen::Vector3d position(0.0, 0.0, 0.0);
  const Eigen::Vector3d goal(10.0, 0.0, 0.0);
  const double moving = update_many(generator, position, Eigen::Vector3d(0.0, 1.0, 0.0), goal, 0.0);
  const double held = update_many(generator, position, Eigen::Vector3d(0.01, 0.01, 3.0), goal, 0.0);
  EXPECT_NEAR(held, moving, 1.0e-9);
}

TEST(YawReferenceGenerator, VerticalTakeoffKeepsInitializationYaw)
{
  YawReferenceGenerator generator(fast_parameters());
  generator.initialize(-0.8);
  EXPECT_NEAR(update_many(
    generator, Eigen::Vector3d::Zero(), Eigen::Vector3d(0.0, 0.0, 1.0),
    Eigen::Vector3d(5.0, 0.0, 2.0), 1.0), -0.8, 1.0e-12);
}

TEST(YawReferenceGenerator, ThresholdIsInclusive)
{
  auto parameters = fast_parameters();
  parameters.tangent_speed_threshold = 0.1;
  YawReferenceGenerator generator(parameters);
  generator.initialize(0.0);
  const double result = update_many(
    generator, Eigen::Vector3d::Zero(), Eigen::Vector3d(0.0, 0.1, 5.0),
    Eigen::Vector3d(10.0, 0.0, 0.0), 0.0);
  EXPECT_NEAR(result, 0.5 * kPi, 1.0e-6);
}

TEST(YawReferenceGenerator, NonFiniteInitializationFallsBackToFixedYaw)
{
  auto parameters = fast_parameters();
  parameters.fixed_yaw = -0.25;
  YawReferenceGenerator generator(parameters);
  generator.initialize(std::numeric_limits<double>::quiet_NaN());
  EXPECT_DOUBLE_EQ(generator.reference(), -0.25);
}

TEST(YawReferenceGenerator, RecoversSmoothlyFromLowSpeed)
{
  auto parameters = fast_parameters();
  parameters.max_yaw_rate = 0.8;
  YawReferenceGenerator generator(parameters);
  generator.initialize(0.0);
  const double held = generator.update(
    Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d(5.0, 0.0, 0.0), 0.0, 0.1);
  const double resumed = generator.update(
    Eigen::Vector3d::Zero(), Eigen::Vector3d(0.0, 1.0, 0.0),
    Eigen::Vector3d(5.0, 0.0, 0.0), 0.0, 0.1);
  EXPECT_DOUBLE_EQ(held, 0.0);
  EXPECT_GT(resumed, held);
  EXPECT_LE(resumed - held, 0.08 + 1.0e-12);
}

TEST(YawReferenceGenerator, PositivePiBoundaryUsesShortDirection)
{
  YawReferenceGenerator generator(fast_parameters());
  generator.initialize(179.0 * kPi / 180.0);
  const double result = update_many(
    generator, Eigen::Vector3d::Zero(),
    Eigen::Vector3d(std::cos(-179.0 * kPi / 180.0), std::sin(-179.0 * kPi / 180.0), 0.0),
    Eigen::Vector3d(10.0, 0.0, 0.0), 0.0);
  EXPECT_NEAR(result, 181.0 * kPi / 180.0, 1.0e-6);
}

TEST(YawReferenceGenerator, NegativePiBoundaryUsesShortDirection)
{
  YawReferenceGenerator generator(fast_parameters());
  generator.initialize(-179.0 * kPi / 180.0);
  const double result = update_many(
    generator, Eigen::Vector3d::Zero(),
    Eigen::Vector3d(std::cos(179.0 * kPi / 180.0), std::sin(179.0 * kPi / 180.0), 0.0),
    Eigen::Vector3d(10.0, 0.0, 0.0), 0.0);
  EXPECT_NEAR(result, -181.0 * kPi / 180.0, 1.0e-6);
}

TEST(YawReferenceGenerator, RepeatedBoundaryUpdatesHaveNoTwoPiJump)
{
  YawReferenceGenerator generator(fast_parameters());
  generator.initialize(179.0 * kPi / 180.0);
  double previous = generator.reference();
  for (int index = 0; index < 20; ++index) {
    const double angle = (index % 2 == 0 ? -179.0 : 179.0) * kPi / 180.0;
    const double current = generator.update(
      Eigen::Vector3d::Zero(), Eigen::Vector3d(std::cos(angle), std::sin(angle), 0.0),
      Eigen::Vector3d(10.0, 0.0, 0.0), 0.0, 0.1);
    EXPECT_LT(std::abs(current - previous), 0.1);
    previous = current;
  }
}

TEST(YawReferenceGenerator, TerminalBlendIsSmoothAndConverges)
{
  YawReferenceGenerator generator(fast_parameters());
  generator.initialize(0.0);
  const Eigen::Vector3d velocity(1.0, 0.0, 0.0);
  const Eigen::Vector3d goal(1.0, 0.0, 0.0);
  const double outside = update_many(generator, Eigen::Vector3d::Zero(), velocity, goal, 0.5 * kPi);
  const double entering = update_many(
    generator, Eigen::Vector3d(0.3, 0.0, 0.0), velocity, goal, 0.5 * kPi);
  const double nearer = update_many(
    generator, Eigen::Vector3d(0.8, 0.0, 0.0), velocity, goal, 0.5 * kPi);
  const double arrived = update_many(generator, goal, Eigen::Vector3d::Zero(), goal, 0.5 * kPi);
  EXPECT_NEAR(outside, 0.0, 1.0e-6);
  EXPECT_GT(entering, outside);
  EXPECT_GT(nearer, entering);
  EXPECT_NEAR(arrived, 0.5 * kPi, 1.0e-6);
}

TEST(YawReferenceGenerator, TerminalBlendCrossesPiByShortestAngle)
{
  YawReferenceGenerator generator(fast_parameters());
  generator.initialize(179.0 * kPi / 180.0);
  const double result = update_many(
    generator, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
    Eigen::Vector3d::Zero(), -179.0 * kPi / 180.0);
  EXPECT_NEAR(result, 181.0 * kPi / 180.0, 1.0e-6);
}

TEST(YawReferenceGenerator, TerminalYawWorksBeforeAnyValidTangent)
{
  YawReferenceGenerator generator(fast_parameters());
  generator.initialize(0.2);
  const double result = update_many(
    generator, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
    Eigen::Vector3d::Zero(), -0.7);
  EXPECT_NEAR(result, -0.7, 1.0e-6);
}

TEST(YawReferenceGenerator, FilterTimeConstantPreventsSingleStepTargetJump)
{
  auto parameters = fast_parameters();
  parameters.filter_time_constant = 0.3;
  YawReferenceGenerator generator(parameters);
  generator.initialize(0.0);
  const double result = generator.update(
    Eigen::Vector3d::Zero(), Eigen::Vector3d(0.0, 1.0, 0.0),
    Eigen::Vector3d(10.0, 0.0, 0.0), 0.0, 0.02);
  EXPECT_GT(result, 0.0);
  EXPECT_LT(result, 0.5 * kPi);
}

TEST(YawReferenceGenerator, NonFiniteInputsUseFiniteSafeFallback)
{
  YawReferenceGenerator generator(fast_parameters());
  generator.initialize(0.4);
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_DOUBLE_EQ(generator.update(
    Eigen::Vector3d(nan, 0.0, 0.0), Eigen::Vector3d::Zero(),
    Eigen::Vector3d::Zero(), 0.0, 0.1), 0.4);
  EXPECT_DOUBLE_EQ(generator.update(
    Eigen::Vector3d::Zero(), Eigen::Vector3d(nan, 0.0, 0.0),
    Eigen::Vector3d::Zero(), 0.0, 0.1), 0.4);
  EXPECT_DOUBLE_EQ(generator.update(
    Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
    Eigen::Vector3d::Zero(), nan, 0.1), 0.4);
  EXPECT_DOUBLE_EQ(generator.update(
    Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
    Eigen::Vector3d::Zero(), 0.0, nan), 0.4);
  EXPECT_TRUE(std::isfinite(generator.reference()));
}

TEST(YawReferenceGenerator, EveryUpdateObeysYawRateLimit)
{
  auto parameters = fast_parameters();
  parameters.max_yaw_rate = 0.8;
  YawReferenceGenerator generator(parameters);
  generator.initialize(0.0);
  double previous = generator.reference();
  for (int index = 0; index < 100; ++index) {
    const double current = generator.update(
      Eigen::Vector3d::Zero(), Eigen::Vector3d(0.0, -1.0, 0.0),
      Eigen::Vector3d(10.0, 0.0, 0.0), 0.0, 0.02);
    EXPECT_TRUE(std::isfinite(current));
    EXPECT_LE(std::abs(current - previous), 0.8 * 0.02 + 1.0e-12);
    previous = current;
  }
}

}  // namespace
}  // namespace drone_planning
