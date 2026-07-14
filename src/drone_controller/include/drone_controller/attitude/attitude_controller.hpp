#ifndef DRONE_CONTROLLER__ATTITUDE__ATTITUDE_CONTROLLER_HPP_
#define DRONE_CONTROLLER__ATTITUDE__ATTITUDE_CONTROLLER_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace drone_controller
{

// 姿态控制器只输出机体系三轴力矩，不涉及总推力（那是 AltitudeController 的职责）。
// 两者的结果最终一起交给 MotorMixer 解算成四电机 RPM。
struct AttitudeControllerParameters
{
  Eigen::Vector3d attitude_kp{4.0, 4.0, 2.0};      // [roll,pitch,yaw] 姿态误差的比例增益。
  Eigen::Vector3d angular_rate_kd{0.20, 0.20, 0.10}; // [roll,pitch,yaw] 角速度误差（阻尼）增益。
  Eigen::Vector3d max_torque{1.0, 1.0, 0.5};       // [roll,pitch,yaw] 输出力矩上限 (N*m)，逐轴限幅。
};

struct AttitudeControllerInput
{
  // Both quaternions rotate vectors from base_link/body into map/world.
  // 两个四元数都是“机体->世界”的旋转（与 QuadrotorModel 的
  // orientation_body_to_world 约定一致），用于计算姿态误差。
  Eigen::Quaterniond desired_orientation_body_to_world{Eigen::Quaterniond::Identity()};
  Eigen::Quaterniond current_orientation_body_to_world{Eigen::Quaterniond::Identity()};
  // 期望/当前机体系角速度 (rad/s)，用于角速度阻尼项，通常 desired 为 0（不主动旋转）。
  Eigen::Vector3d desired_angular_velocity_body{Eigen::Vector3d::Zero()};
  Eigen::Vector3d current_angular_velocity_body{Eigen::Vector3d::Zero()};
};

struct AttitudeControllerResult
{
  // [roll, pitch, yaw] torque expressed in base_link, in N*m.
  // 最终输出：机体系三轴力矩，直接对应 Mixer 的 WrenchCommand 里的
  // roll_torque/pitch_torque/yaw_torque。
  Eigen::Vector3d torque_body{Eigen::Vector3d::Zero()};
  bool valid{true};       // 输入非法（NaN/Inf、四元数无法归一化等）时为 false。
  bool saturated{false};  // 任意一轴力矩被 max_torque 限幅时为 true。
};

class AttitudeController
{
public:
  // 构造时校验参数：增益必须非负有限，力矩上限必须为正有限，否则直接抛异常。
  explicit AttitudeController(
    const AttitudeControllerParameters & parameters = AttitudeControllerParameters{});

  // 核心接口：无状态纯函数，输入姿态/角速度 -> 输出三轴力矩，便于单元测试逐条构造用例。
  AttitudeControllerResult compute(const AttitudeControllerInput & input) const;

private:
  AttitudeControllerParameters parameters_;
};

}  // namespace drone_controller

#endif  // DRONE_CONTROLLER__ATTITUDE__ATTITUDE_CONTROLLER_HPP_
