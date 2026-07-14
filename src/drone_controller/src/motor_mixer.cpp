#include "drone_controller/mixer/motor_mixer.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace drone_controller
{
namespace
{

constexpr double kTwoPi = 6.28318530717958647692;

// 检查一个值是否既有限又严格大于零，用于校验几何/物理系数参数。
bool is_positive_finite(const double value)
{
  return std::isfinite(value) && value > 0.0;
}

// 统一的失败返回：四电机 RPM 保持默认零向量，valid=false 告知调用方
// （HoverController/节点）本次结果不可用，saturated=true 是保守取值。
MixerResult invalid_result()
{
  MixerResult result;
  result.valid = false;
  result.saturated = true;
  return result;
}

}  // namespace

MotorMixer::MotorMixer(const MixerParameters & parameters)
: parameters_(parameters)
{
  if (!is_positive_finite(parameters_.arm_length)) {
    throw std::invalid_argument("arm_length must be finite and greater than zero");
  }
  if (!is_positive_finite(parameters_.thrust_coefficient)) {
    throw std::invalid_argument("thrust_coefficient must be finite and greater than zero");
  }
  if (!is_positive_finite(parameters_.drag_torque_coefficient)) {
    throw std::invalid_argument(
            "drag_torque_coefficient must be finite and greater than zero");
  }
  if (!std::isfinite(parameters_.min_rpm) || !std::isfinite(parameters_.max_rpm) ||
    parameters_.min_rpm < 0.0 || parameters_.min_rpm >= parameters_.max_rpm)
  {
    throw std::invalid_argument("RPM limits must satisfy 0 <= min_rpm < max_rpm");
  }
}

MixerResult MotorMixer::mix(const WrenchCommand & command) const
{
  // 第 0 步：输入合法性检查，任何一项 NaN/Inf 直接判为无效。
  MixerResult result;
  if (!std::isfinite(command.thrust) || !std::isfinite(command.roll_torque) ||
    !std::isfinite(command.pitch_torque) || !std::isfinite(command.yaw_torque))
  {
    return invalid_result();
  }

  // 第 1 步：总推力不允许为负（物理上无意义），负值钳零并标记饱和。
  double thrust = command.thrust;
  if (thrust < 0.0) {
    thrust = 0.0;
    result.saturated = true;
  }

  // Invert the X-layout wrench matrix in thrust space. Here a=L/sqrt(2)
  // and b=kM/kF. Motor order and signs exactly match QuadrotorModel.
  // a：X 型布局下电机到机体 x/y 轴的等效力臂（机臂长度投影到 45°对角线上）；
  // b：反扇矩与推力的比值，用于把 yaw 力矩换算成同量纲的推力单位。
  const double a = parameters_.arm_length / std::sqrt(2.0);
  const double b = parameters_.drag_torque_coefficient / parameters_.thrust_coefficient;
  // 先把三轴力矩除以对应系数，转换为与总推力同量纲的“推力差分量”，
  // 之后只需线性加减即可得到单电机推力，不需要矩阵求逆。
  const double roll_term = command.roll_torque / a;
  const double pitch_term = command.pitch_torque / a;
  const double yaw_term = command.yaw_torque / b;
  if (!std::isfinite(roll_term) || !std::isfinite(pitch_term) ||
    !std::isfinite(yaw_term))
  {
    return invalid_result();
  }
  // 第 2 步：X 型布局 Wrench 矩阵的解析逆。四行分别对应 M1~M4，
  // 符号组合与 AI_CONTEXT 中的 tau_x/tau_y/tau_z 推导公式一致：
  // M1/M3 为 CCW（产生负 yaw 反扇矩），M2/M4 为 CW（产生正 yaw 反扇矩）。
  std::array<double, 4> motor_thrust{
    0.25 * (thrust + roll_term - pitch_term - yaw_term),
    0.25 * (thrust + roll_term + pitch_term + yaw_term),
    0.25 * (thrust - roll_term + pitch_term - yaw_term),
    0.25 * (thrust - roll_term - pitch_term + yaw_term)};

  for (std::size_t index = 0; index < motor_thrust.size(); ++index) {
    if (!std::isfinite(motor_thrust[index])) {
      return invalid_result();
    }
    // 单电机解得的推力也可能为负（例如力矩请求过大且总推力过小时），
    // 同样钳零并标记饱和，保证下面 sqrt 不会对负数开方。
    if (motor_thrust[index] < 0.0) {
      motor_thrust[index] = 0.0;
      result.saturated = true;
    }

    // 第 3 步：由单电机推力反推转速：F=k_F*omega^2 → omega=sqrt(F/k_F)，
    // 再从 rad/s 换算成 RPM（乘 60/2pi），与 QuadrotorModel 内部单位约定相反。
    const double omega = std::sqrt(motor_thrust[index] / parameters_.thrust_coefficient);
    const double rpm = omega * 60.0 / kTwoPi;
    if (!std::isfinite(omega) || !std::isfinite(rpm)) {
      return invalid_result();
    }
    // 第 4 步：逐电机限幅到物理 RPM 范围。注意限幅后实际 Wrench 可能与请求不同，
    // 调用方必须通过 saturated 标志感知这种偏差，本函数不会回馈实际 Wrench。
    const double clamped_rpm = std::clamp(rpm, parameters_.min_rpm, parameters_.max_rpm);
    if (clamped_rpm != rpm) {
      result.saturated = true;
    }
    result.motor_rpm[index] = clamped_rpm;
  }
  return result;
}

}  // namespace drone_controller
