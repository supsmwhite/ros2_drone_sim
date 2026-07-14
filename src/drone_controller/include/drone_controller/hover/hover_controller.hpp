#ifndef DRONE_CONTROLLER__HOVER__HOVER_CONTROLLER_HPP_
#define DRONE_CONTROLLER__HOVER__HOVER_CONTROLLER_HPP_

#include <array>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "drone_controller/altitude/altitude_controller.hpp"
#include "drone_controller/attitude/attitude_controller.hpp"
#include "drone_controller/mixer/motor_mixer.hpp"

namespace drone_controller
{

// 汇总三个子控制器各自的参数结构体，构造 HoverController 时一次性传入。
struct HoverControllerParameters
{
  AltitudeControllerParameters altitude;
  AttitudeControllerParameters attitude;
  MixerParameters mixer;
};

// HoverController 的统一输入：整合了高度、姿态、角速度三方面的目标值和当前状态，
// 由调用方（目前是 position_controller_node）从 /drone/goal 和 /drone/odom 组装。
struct HoverControllerInput
{
  double desired_altitude{0.0};              // 目标世界系高度 (m)。
  double desired_vertical_velocity{0.0};     // 目标世界系垂直速度 (m/s)，通常为0。
  double desired_vertical_acceleration{0.0}; // 前馈垂直加速度 (m/s^2)，通常为0。
  // 期望姿态（body-to-world）。当前节点只用目标 yaw 生成水平姿态，
  // 见 level_orientation_from_goal_yaw()。
  Eigen::Quaterniond desired_orientation_body_to_world{Eigen::Quaterniond::Identity()};
  double current_altitude{0.0};                 // 当前世界系高度 (m)，来自 Odom。
  double current_vertical_velocity_world{0.0};  // 当前世界系垂直速度 (m/s)，
                                                 // 必须先用 world_vertical_velocity_from_body() 转换，
                                                 // 不能直接用 Odom 的机体系 twist.linear.z。
  Eigen::Quaterniond current_orientation_body_to_world{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d desired_angular_velocity_body{Eigen::Vector3d::Zero()}; // 期望机体角速度，通常为0。
  Eigen::Vector3d current_angular_velocity_body{Eigen::Vector3d::Zero()}; // 当前机体角速度，来自 Odom。
};

// HoverController 的统一输出：既有最终的四电机 RPM，也保留每个子控制器的
// 中间结果和饱和状态，便于日志/测试定位是哪一级触发了限幅或失败。
struct HoverControllerResult
{
  std::array<double, 4> motor_rpm{};             // 最终交给 /drone/motor_rpm_cmd 的四电机目标转速。
  double collective_thrust{0.0};                 // AltitudeController 的输出，供调试观察。
  Eigen::Vector3d torque_body{Eigen::Vector3d::Zero()}; // AttitudeController 的输出，供调试观察。
  bool valid{true};        // 三个子控制器中任一个 invalid，则整体为 false，此时 motor_rpm 全零。
  bool saturated{false};   // altitude/attitude/mixer 三者任一饱和时为 true，用于综合判断。
  bool altitude_saturated{false};
  bool attitude_saturated{false};
  bool mixer_saturated{false};
};

class HoverController
{
public:
  // 用一份 HoverControllerParameters 分别构造三个子控制器，参数校验交给各自构造函数完成。
  explicit HoverController(
    const HoverControllerParameters & parameters = HoverControllerParameters{});

  // 核心接口：按 高度 -> 姿态 -> Mixer 的顺序依次调用三个子控制器，
  // 任一环节失败立即短路返回全零 RPM；成功则汇总饱和状态并返回四电机 RPM。
  HoverControllerResult compute(const HoverControllerInput & input) const;

private:
  AltitudeController altitude_controller_;
  AttitudeController attitude_controller_;
  MotorMixer motor_mixer_;
};

// Pure conversion helpers used by the ROS node and unit tests.
// 两个纯函数，是 ROS 层数据与本包算法输入之间的“适配器”，不依赖 ROS2，可独立测试。

// 把 Odom 里机体系（base_link）的线速度旋转到世界系（map），取其 z 分量，
// 因为 Odom 约定 twist.linear 表达在 base_link 中，不能直接当作世界系 vz 使用。
bool world_vertical_velocity_from_body(
  const Eigen::Quaterniond & orientation_body_to_world,
  const Eigen::Vector3d & velocity_body,
  double & vertical_velocity_world);

// 从目标姿态四元数中只提取 yaw 分量，roll/pitch 强制置零，
// 生成“水平期望姿态”。这是当前控制器忽略目标 roll/pitch 的具体实现方式。
bool level_orientation_from_goal_yaw(
  const Eigen::Quaterniond & goal_orientation_body_to_world,
  Eigen::Quaterniond & level_orientation_body_to_world);

}  // namespace drone_controller

#endif  // DRONE_CONTROLLER__HOVER__HOVER_CONTROLLER_HPP_
