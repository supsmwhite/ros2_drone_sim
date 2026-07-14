#ifndef DRONE_CONTROLLER__MIXER__MOTOR_MIXER_HPP_
#define DRONE_CONTROLLER__MIXER__MOTOR_MIXER_HPP_

#include <array>

namespace drone_controller
{

// MotorMixer 是控制链路的最后一环：把总推力 + 三轴力矩，解算成
// 四个电机各自的目标 RPM，交给 quadrotor_dynamics_node 执行。
struct MixerParameters
{
  // These values must remain consistent with the quadrotor dynamics parameters.
  // 以下物理参数必须与 QuadrotorModel/dynamics.yaml 中的同名参数保持一致，
  // 否则 Mixer 反解出的 RPM 和实际动力学响应会不匹配。
  double arm_length{0.20};              // 机臂长度 L (m)，X 型布局下等效力臂 a=L/sqrt(2)。
  double thrust_coefficient{1.91e-6};   // 推力系数 k_F：F=k_F*omega^2 (N/(rad/s)^2)。
  double drag_torque_coefficient{2.60e-7}; // 反扭矩系数 k_M：Q=k_M*omega^2 (N*m/(rad/s)^2)。
  double min_rpm{0.0};                  // 电机 RPM 下限，用于最终限幅。
  double max_rpm{20000.0};              // 电机 RPM 上限，用于最终限幅。
};

// 上游（HoverController）请求的机体系合力/合力矩，即“期望施加给机体的总 Wrench”。
struct WrenchCommand
{
  double thrust{0.0};       // 总推力 T (N)，沿机体 z 轴。
  double roll_torque{0.0};  // 绕机体 x 轴的力矩 tau_x (N*m)。
  double pitch_torque{0.0}; // 绕机体 y 轴的力矩 tau_y (N*m)。
  double yaw_torque{0.0};   // 绕机体 z 轴的力矩 tau_z (N*m)。
};

struct MixerResult
{
  // Fixed order: [M1 front-left CCW, M2 rear-left CW,
  // M3 rear-right CCW, M4 front-right CW].
  // 固定电机顺序，必须与 drone_msgs/MotorRPM.msg 及 QuadrotorModel 完全一致，
  // 不允许在本文件内单独调整编号或符号。
  std::array<double, 4> motor_rpm{};
  bool valid{true};       // 输入非法（NaN/Inf 等）时为 false，此时 motor_rpm 无意义。
  bool saturated{false};  // 推力为负、单电机推力为负或 RPM 超出范围被限幅时为 true。
};

class MotorMixer
{
public:
  // 构造时校验参数：几何/物理系数必须为正有限，RPM 范围必须满足 0<=min<max。
  explicit MotorMixer(const MixerParameters & parameters = MixerParameters{});

  // Converts a desired body wrench into four target RPM values. Per-motor
  // clipping is intentionally simple: after saturation, the achieved wrench
  // can differ from the request and a future controller must use the flag.
  // 核心接口：无状态纯函数，输入合力/合力矩 -> 输出四电机 RPM，
  // 便于单元测试逐条构造 Wrench 用例并核对反解结果。
  MixerResult mix(const WrenchCommand & command) const;

private:
  MixerParameters parameters_;
};

}  // namespace drone_controller

#endif  // DRONE_CONTROLLER__MIXER__MOTOR_MIXER_HPP_
