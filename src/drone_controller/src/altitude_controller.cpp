#include "drone_controller/altitude/altitude_controller.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

namespace drone_controller
{
namespace
{

constexpr double kMinimumQuaternionScale = 1.0e-12;

// 一次性检查一组 double 是否全部有限（非 NaN/Inf），用于输入合法性校验。
bool all_finite(const std::array<double, 5> & values)
{
  return std::all_of(values.begin(), values.end(), [](const double value) {
    return std::isfinite(value);
  });
}

// 对四元数做安全归一化：先按最大分量缩放避免数值溢出，再按模长归一化。
// 任何一步出现非有限值或接近零模长都判定为非法，返回 false。
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

// 统一的失败返回：valid=false 告知调用方（HoverController/节点）本次结果不可用，
// saturated=true 是保守取值，避免调用方误以为“未饱和”而信任一个无效的推力。
AltitudeControllerResult invalid_result()
{
  AltitudeControllerResult result;
  result.valid = false;
  result.saturated = true;
  return result;
}

}  // namespace

AltitudeController::AltitudeController(const AltitudeControllerParameters & parameters)
: parameters_(parameters)
{
  if (!std::isfinite(parameters_.mass) || parameters_.mass <= 0.0 ||
    !std::isfinite(parameters_.gravity) || parameters_.gravity <= 0.0 ||
    !std::isfinite(parameters_.altitude_kp) || parameters_.altitude_kp < 0.0 ||
    !std::isfinite(parameters_.vertical_velocity_kd) ||
    parameters_.vertical_velocity_kd < 0.0 ||
    !std::isfinite(parameters_.max_upward_acceleration) ||
    parameters_.max_upward_acceleration < 0.0 ||
    !std::isfinite(parameters_.max_downward_acceleration) ||
    parameters_.max_downward_acceleration < 0.0 ||
    !std::isfinite(parameters_.min_collective_thrust) ||
    !std::isfinite(parameters_.max_collective_thrust) ||
    parameters_.min_collective_thrust < 0.0 ||
    parameters_.min_collective_thrust >= parameters_.max_collective_thrust ||
    !std::isfinite(parameters_.min_tilt_cosine) ||
    parameters_.min_tilt_cosine <= 0.0 || parameters_.min_tilt_cosine > 1.0)
  {
    throw std::invalid_argument("Invalid altitude controller parameters");
  }
}

AltitudeControllerResult AltitudeController::compute(const AltitudeControllerInput & input) const
{
  // 第 0 步：输入合法性检查。任何一个输入是 NaN/Inf 都直接判为无效，
  // 避免非法值污染后续的算术运算。
  if (!all_finite({input.desired_altitude, input.desired_vertical_velocity,
      input.desired_vertical_acceleration, input.current_altitude,
      input.current_vertical_velocity}))
  {
    return invalid_result();
  }

  // 姿态四元数只用于取倾角余弦，这里先做安全归一化，非法姿态直接判为无效。
  Eigen::Quaterniond orientation = input.current_orientation_body_to_world;
  if (!normalize_quaternion(orientation)) {
    return invalid_result();
  }

  // 第 1 步：世界系高度/速度 PD + 前馈，得到“期望的世界系垂直加速度”。
  // e_z：位置误差；e_vz：速度误差；raw_acceleration = Kp*e_z + Kd*e_vz + a_ff。
  const double position_error = input.desired_altitude - input.current_altitude;
  const double velocity_error =
    input.desired_vertical_velocity - input.current_vertical_velocity;
  const double raw_acceleration =
    parameters_.altitude_kp * position_error +
    parameters_.vertical_velocity_kd * velocity_error +
    input.desired_vertical_acceleration;
  if (!std::isfinite(position_error) || !std::isfinite(velocity_error) ||
    !std::isfinite(raw_acceleration))
  {
    return invalid_result();
  }

  // 第 2 步：对加速度指令限幅（防止过大加速度请求），并记录是否发生了饱和。
  AltitudeControllerResult result;
  result.commanded_vertical_acceleration = std::clamp(
    raw_acceleration, -parameters_.max_downward_acceleration,
    parameters_.max_upward_acceleration);
  if (result.commanded_vertical_acceleration != raw_acceleration) {
    result.saturated = true;
  }

  // 第 3 步：把“期望垂直加速度”换算成“期望世界系垂直合力”，
  // F = m * (g + a_z)：g 补偿重力，a_z 是上面算出的加速度指令。
  const double vertical_force =
    parameters_.mass * (parameters_.gravity + result.commanded_vertical_acceleration);
  // cos_tilt：机体 z 轴（推力方向）在世界系 z 方向上的投影分量，
  // 即 R(q)*[0,0,1]^T 的 z 分量，用来衡量当前倾斜程度（水平时=1，倾斜时<1）。
  const double cos_tilt = (orientation * Eigen::Vector3d::UnitZ()).z();
  if (!std::isfinite(vertical_force) || !std::isfinite(cos_tilt)) {
    return invalid_result();
  }
  // 倾角>=90°（cos_tilt<=0）意味着推力方向已经没有向上分量甚至朝下，
  // 此时无法通过“除以 cos_tilt”得到合理推力，直接判为无效而不是硬算。
  if (cos_tilt <= 0.0) {
    return invalid_result();
  }

  // 第 4 步：倾角余弦下限保护。cos_tilt 越小，vertical_force/cos_tilt 越容易暴涨，
  // 因此当倾角过大（cos_tilt 低于 min_tilt_cosine）时，钳制到 min_tilt_cosine 并标记饱和，
  // 用可控的推力换取“大倾角下垂直力略微不足”的折中。
  double safe_cos_tilt = cos_tilt;
  if (safe_cos_tilt < parameters_.min_tilt_cosine) {
    safe_cos_tilt = parameters_.min_tilt_cosine;
    result.saturated = true;
  }
  // 第 5 步：倾斜补偿——总推力沿机体 z 轴，只有 cos_tilt 比例的分量作用在世界系垂直方向，
  // 所以要施加的总推力必须是“世界系所需垂直力”除以 cos_tilt 才能抵消倾斜带来的损失。
  const double raw_thrust = vertical_force / safe_cos_tilt;
  if (!std::isfinite(raw_thrust)) {
    return invalid_result();
  }

  // 第 6 步：最终推力限幅到电机物理可实现范围，并记录是否饱和。
  result.collective_thrust = std::clamp(
    raw_thrust, parameters_.min_collective_thrust,
    parameters_.max_collective_thrust);
  if (result.collective_thrust != raw_thrust) {
    result.saturated = true;
  }
  return result;
}

}  // namespace drone_controller
