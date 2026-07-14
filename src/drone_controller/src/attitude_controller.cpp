#include "drone_controller/attitude/attitude_controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace drone_controller
{
namespace
{

constexpr double kMinimumQuaternionScale = 1.0e-12;
// q.w() 的绝对值小于此阈值时，认为当前误差旋转恰好接近 180°（“半圈”），
// 需要走下面的“确定性符号”特殊分支，而不是用 q.w() 的符号判断最短路径。
constexpr double kHalfTurnScalarEpsilon = 1.0e-12;

// 逐元素检查 3 维向量是否全部有限（非 NaN/Inf）。
bool vector_is_finite(const Eigen::Vector3d & value)
{
  return value.array().isFinite().all();
}

bool normalize_quaternion(Eigen::Quaterniond & quaternion)
{
  if (!quaternion.coeffs().array().isFinite().all()) {
    return false;
  }

  // Scale first so even very large finite coefficients do not overflow while
  // computing the norm. Eigen stores quaternion coefficients as [x,y,z,w].
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

// 统一的失败返回：torque_body 保持默认零向量，valid=false 告知调用方
// （HoverController/节点）本次力矩不可用，不应被下发给电机。
AttitudeControllerResult invalid_result()
{
  AttitudeControllerResult result;
  result.valid = false;
  return result;
}

}  // namespace

AttitudeController::AttitudeController(const AttitudeControllerParameters & parameters)
: parameters_(parameters)
{
  if (!vector_is_finite(parameters_.attitude_kp) ||
    !vector_is_finite(parameters_.angular_rate_kd) ||
    !vector_is_finite(parameters_.max_torque) ||
    (parameters_.attitude_kp.array() < 0.0).any() ||
    (parameters_.angular_rate_kd.array() < 0.0).any() ||
    (parameters_.max_torque.array() <= 0.0).any())
  {
    throw std::invalid_argument(
            "Attitude gains must be finite and nonnegative; torque limits must be finite and positive");
  }
}

AttitudeControllerResult AttitudeController::compute(const AttitudeControllerInput & input) const
{
  // 第 0 步：角速度输入合法性检查，防止 NaN/Inf 污染后续运算。
  if (!vector_is_finite(input.desired_angular_velocity_body) ||
    !vector_is_finite(input.current_angular_velocity_body))
  {
    return invalid_result();
  }

  // 两个姿态四元数都要安全归一化，非法姿态直接判为无效。
  Eigen::Quaterniond desired = input.desired_orientation_body_to_world;
  Eigen::Quaterniond current = input.current_orientation_body_to_world;
  if (!normalize_quaternion(desired) || !normalize_quaternion(current)) {
    return invalid_result();
  }

  // 第 1 步：计算姿态误差四元数。
  // q_error maps the desired body orientation relative to the current body.
  // Its vector part therefore has the same base_link roll/pitch/yaw signs used
  // by the dynamics torque vector.
  // current.conjugate() 相当于“世界->当前机体”的逆旋转，乘以 desired（机体->世界）
  // 后得到的 q_error 表示“从当前姿态转到期望姿态”需要的旋转增量，且其向量部分
  // 天然落在 base_link 坐标系下，与力矩向量的符号约定一致。
  Eigen::Quaterniond error = current.conjugate() * desired;

  // 四元数存在 q 和 -q 表示同一旋转的“双重覆盖”问题：若不处理，q_error 可能
  // 代表一个大于 180° 的多余旋转（本该转 10°却按 350°的反方向算误差）。
  // 规则：q_error.w() 是旋转角一半的余弦，w()<0 说明旋转角超过180°，
  // 此时取反四元数（等价旋转但角度换成 360°减去原角度）即可得到最短路径。
  bool negate_error = error.w() < -kHalfTurnScalarEpsilon;
  if (std::abs(error.w()) <= kHalfTurnScalarEpsilon) {
    // 恰好等于180°的边界情况：w()≈0，符号判断失效（正负浮点误差都可能出现）。
    // 此时改用向量部分绝对值最大的分量的符号，作为确定性的统一规则，
    // 避免因浮点噪声在多次调用之间正负号抖动，导致控制输出不连续。
    Eigen::Index dominant_axis = 0;
    error.vec().cwiseAbs().maxCoeff(&dominant_axis);
    negate_error = error.vec()[dominant_axis] < 0.0;
  }
  if (negate_error) {
    error.coeffs() *= -1.0;
  }
  // 小角度近似：单位四元数向量部分 vec() ≈ (旋转轴)*sin(角度/2)，
  // 乘以 2 后可近似当作“姿态误差角度向量”[roll_err, pitch_err, yaw_err]，
  // 供下面的比例项直接使用。
  const Eigen::Vector3d attitude_error = 2.0 * error.vec();

  // 第 2 步：PD 合成 —— 比例项跟踪姿态误差，微分项跟踪角速度误差（阻尼）。
  // Positive current rate with zero desired rate produces negative damping.
  // 注：desired-current 为零时（期望角速度为0，当前角速度非零），阻尼项为负，
  // 起到“抑制当前旋转”的作用，这是符合直觉的阻尼效果而非笔误。
  Eigen::Vector3d torque =
    parameters_.attitude_kp.cwiseProduct(attitude_error) +
    parameters_.angular_rate_kd.cwiseProduct(
    input.desired_angular_velocity_body - input.current_angular_velocity_body);
  if (!vector_is_finite(torque)) {
    return invalid_result();
  }

  // 第 3 步：逐轴（roll/pitch/yaw）独立限幅，分别记录是否触发饱和。
  AttitudeControllerResult result;
  for (Eigen::Index axis = 0; axis < torque.size(); ++axis) {
    const double limited = std::clamp(
      torque[axis], -parameters_.max_torque[axis], parameters_.max_torque[axis]);
    if (limited != torque[axis]) {
      result.saturated = true;
    }
    result.torque_body[axis] = limited;
  }
  return result;
}

}  // namespace drone_controller
