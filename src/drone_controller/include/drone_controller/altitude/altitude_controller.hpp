#ifndef DRONE_CONTROLLER__ALTITUDE__ALTITUDE_CONTROLLER_HPP_
#define DRONE_CONTROLLER__ALTITUDE__ALTITUDE_CONTROLLER_HPP_

#include <Eigen/Geometry>

namespace drone_controller
{

// 高度控制器只输出一个标量总推力（collective thrust），不产生任何姿态/力矩指令。
// roll/pitch/yaw 由 AttitudeController 单独负责，两者的结果最终一起交给 MotorMixer。
struct AltitudeControllerParameters
{
  double mass{1.0};                        // 机体质量 (kg)，用于力=质量*加速度换算。
  double gravity{9.80665};                 // 重力加速度 (m/s^2)，用于重力补偿前馈。
  double altitude_kp{3.0};                 // 位置误差 e_z 的比例增益。
  double vertical_velocity_kd{3.5};        // 速度误差 e_vz 的微分（阻尼）增益。
  double max_upward_acceleration{5.0};     // 允许的最大向上指令加速度 (m/s^2)，用于限幅。
  double max_downward_acceleration{5.0};   // 允许的最大向下指令加速度 (m/s^2)，用于限幅。
  double min_collective_thrust{0.0};       // 输出总推力下限 (N)，防止负推力等非物理值。
  double max_collective_thrust{30.0};      // 输出总推力上限 (N)，对应电机/RPM 物理能力。
  double min_tilt_cosine{0.5};             // 倾角余弦下限，避免大倾角时除以过小的值导致推力暴涨。
};

struct AltitudeControllerInput
{
  double desired_altitude{0.0};              // 目标世界系高度 z_desired (m)。
  double desired_vertical_velocity{0.0};     // 目标世界系垂直速度 vz_desired (m/s)，通常为0（悬停/定高）。
  double desired_vertical_acceleration{0.0}; // 前馈加速度 a_ff (m/s^2)，通常为0，为未来轨迹跟踪预留。
  double current_altitude{0.0};              // 当前世界系高度 z_current (m)，来自 Odom pose.position.z。
  // This is world/map vertical velocity, not Odometry twist.linear.z directly.
  // 注意：这里必须是世界系 vz，Odom 的 twist.linear 是机体系，调用方需先做
  // velocity_world = orientation_body_to_world * velocity_body 转换后再传入。
  double current_vertical_velocity{0.0};
  // 当前姿态四元数（body-to-world），仅用于计算倾角余弦 cos_tilt 做推力补偿，
  // 不用于产生任何姿态指令。
  Eigen::Quaterniond current_orientation_body_to_world{Eigen::Quaterniond::Identity()};
};

struct AltitudeControllerResult
{
  double collective_thrust{0.0};               // 最终输出：沿机体 z 轴方向所需的总推力 (N)。
  double commanded_vertical_acceleration{0.0};  // 中间量：PD+前馈算出、限幅后的世界系垂直加速度指令，供调试/测试观察。
  bool valid{true};       // 输入非法（NaN/Inf、倾角<=0 等）或计算失败时为 false，此时 collective_thrust 无意义。
  bool saturated{false};  // 加速度限幅、倾角余弦限幅或推力限幅中任一项触发时为 true。
};

class AltitudeController
{
public:
  // parameters 在构造时做合法性检查（有限、符号、范围），非法参数直接抛异常，
  // 避免带着错误参数运行整个控制回路。
  explicit AltitudeController(
    const AltitudeControllerParameters & parameters = AltitudeControllerParameters{});

  // 核心接口：无状态纯函数，输入结构体 -> 输出结构体，不修改任何成员状态，
  // 可在任意频率下重复调用，也便于单元测试逐条构造输入用例。
  AltitudeControllerResult compute(const AltitudeControllerInput & input) const;

private:
  AltitudeControllerParameters parameters_;
};

}  // namespace drone_controller

#endif  // DRONE_CONTROLLER__ALTITUDE__ALTITUDE_CONTROLLER_HPP_
