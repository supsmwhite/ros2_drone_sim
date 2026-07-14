#include "drone_controller/hover/hover_controller.hpp"

#include <cmath>

namespace drone_controller
{
namespace
{

constexpr double kMinimumQuaternionScale = 1.0e-12;

// 与 AltitudeController/AttitudeController 里同名函数逻辑一致：先按最大分量缩放
// 避免数值溢出，再按模长归一化，任何一步非法都返回 false。
// HoverController 自己也需要处理四元数（两个转换函数里），故在此处重复实现一份。
bool normalize_quaternion(Eigen::Quaterniond & quaternion)
{
  if (!quaternion.coeffs().array().isFinite().all()) {
    return false;
  }
  const double scale = quaternion.coeffs().cwiseAbs().maxCoeff();
  if (!std::isfinite(scale) || scale < kMinimumQuaternionScale) {
    return false;
  }
  quaternion.coeffs() /= scale;
  const double norm = quaternion.norm();
  if (!std::isfinite(norm) || norm < kMinimumQuaternionScale) {
    return false;
  }
  quaternion.coeffs() /= norm;
  return quaternion.coeffs().array().isFinite().all();
}

// 统一失败处理：在传入的部分结果基础上（保留已经算出的分项饱和标志供调试），
// 强制将 motor_rpm 清零、valid 置 false、saturated 置 true，
// 确保下游（节点）不会将一个无效结果当作安全 RPM 发给电机。
HoverControllerResult invalid_result(HoverControllerResult result)
{
  result.motor_rpm.fill(0.0);
  result.valid = false;
  result.saturated = true;
  return result;
}

}  // namespace

HoverController::HoverController(const HoverControllerParameters & parameters)
: altitude_controller_(parameters.altitude),
  attitude_controller_(parameters.attitude),
  motor_mixer_(parameters.mixer)
{
}

HoverControllerResult HoverController::compute(const HoverControllerInput & input) const
{
  HoverControllerResult result;

  // ===== 第 1 环：高度控制 =====
  // 只抽取 HoverControllerInput 中与高度相关的字段，注意
  // current_vertical_velocity 必须传入世界系的 current_vertical_velocity_world。
  AltitudeControllerInput altitude_input;
  altitude_input.desired_altitude = input.desired_altitude;
  altitude_input.desired_vertical_velocity = input.desired_vertical_velocity;
  altitude_input.desired_vertical_acceleration = input.desired_vertical_acceleration;
  altitude_input.current_altitude = input.current_altitude;
  altitude_input.current_vertical_velocity = input.current_vertical_velocity_world;
  altitude_input.current_orientation_body_to_world = input.current_orientation_body_to_world;
  const AltitudeControllerResult altitude_result = altitude_controller_.compute(altitude_input);
  result.collective_thrust = altitude_result.collective_thrust;
  result.altitude_saturated = altitude_result.saturated;
  result.saturated = altitude_result.saturated;
  // 高度环失败则立即短路，不再调用后续姿态/Mixer，直接返回全零 RPM。
  if (!altitude_result.valid) {
    return invalid_result(result);
  }

  // ===== 第 2 环：姿态控制 =====
  // desired_orientation_body_to_world 由调用方传入（目前是通过
  // level_orientation_from_goal_yaw 从目标 yaw 生成的水平姿态）。
  AttitudeControllerInput attitude_input;
  attitude_input.desired_orientation_body_to_world =
    input.desired_orientation_body_to_world;
  attitude_input.current_orientation_body_to_world =
    input.current_orientation_body_to_world;
  attitude_input.desired_angular_velocity_body = input.desired_angular_velocity_body;
  attitude_input.current_angular_velocity_body = input.current_angular_velocity_body;
  const AttitudeControllerResult attitude_result = attitude_controller_.compute(attitude_input);
  result.torque_body = attitude_result.torque_body;
  result.attitude_saturated = attitude_result.saturated;
  // 注意这里用 || 累加，而不是直接赋值：一旦之前高度环已经饱和，
  // 总体 saturated 必须继续保持 true，不能被本环的未饱和状态覆盖。
  result.saturated = result.saturated || attitude_result.saturated;
  // 姿态环失败同样立即短路，不再调用 Mixer。
  if (!attitude_result.valid) {
    return invalid_result(result);
  }

  // ===== 第 3 环：Motor Mixer =====
  // 把前两环的输出（总推力 + 三轴力矩）拼成 WrenchCommand 传给 Mixer。
  const MixerResult mixer_result = motor_mixer_.mix({
      result.collective_thrust, result.torque_body.x(), result.torque_body.y(),
      result.torque_body.z()});
  result.mixer_saturated = mixer_result.saturated;
  result.saturated = result.saturated || mixer_result.saturated;
  if (!mixer_result.valid) {
    return invalid_result(result);
  }
  result.motor_rpm = mixer_result.motor_rpm;
  // 最后一道保险：即使 Mixer 自认 valid，也再次确认四个 RPM 均为有限值，
  // 避免任何未预期的浮点异常泄露到 /drone/motor_rpm_cmd。
  for (const double rpm : result.motor_rpm) {
    if (!std::isfinite(rpm)) {
      return invalid_result(result);
    }
  }
  return result;
}

// 把 Odom 中机体系（base_link）的线速度旋转到世界系（map），取其 z 分量，
// 供 AltitudeController 作为 current_vertical_velocity 使用。
// 公式：velocity_world = orientation_body_to_world * velocity_body。
bool world_vertical_velocity_from_body(
  const Eigen::Quaterniond & orientation_body_to_world,
  const Eigen::Vector3d & velocity_body,
  double & vertical_velocity_world)
{
  if (!velocity_body.array().isFinite().all()) {
    return false;
  }
  Eigen::Quaterniond orientation = orientation_body_to_world;
  if (!normalize_quaternion(orientation)) {
    return false;
  }
  const Eigen::Vector3d velocity_world = orientation * velocity_body;
  if (!velocity_world.array().isFinite().all()) {
    return false;
  }
  vertical_velocity_world = velocity_world.z();
  return std::isfinite(vertical_velocity_world);
}

// 从目标四元数中只提取 yaw，强制生成 roll=pitch=0 的“水平期望姿态”，
// 这就是当前控制器忽略目标 roll/pitch、只使用 yaw 的具体实现。
bool level_orientation_from_goal_yaw(
  const Eigen::Quaterniond & goal_orientation_body_to_world,
  Eigen::Quaterniond & level_orientation_body_to_world)
{
  Eigen::Quaterniond goal = goal_orientation_body_to_world;
  if (!normalize_quaternion(goal)) {
    return false;
  }
  const Eigen::Matrix3d rotation = goal.toRotationMatrix();
  // 从旋转矩阵前两列取 yaw：yaw = atan2(R10, R00)，是标准的 ZYX 欧拉角提取方式。
  const double yaw = std::atan2(rotation(1, 0), rotation(0, 0));
  if (!std::isfinite(yaw)) {
    return false;
  }
  // 只绕世界系 z 轴旋转 yaw 角度，roll/pitch 都为 0，得到水平期望姿态。
  level_orientation_body_to_world =
    Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
  return level_orientation_body_to_world.coeffs().array().isFinite().all();
}

}  // namespace drone_controller
