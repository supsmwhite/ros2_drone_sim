#ifndef DRONE_DYNAMICS__QUADROTOR_MODEL_HPP_
#define DRONE_DYNAMICS__QUADROTOR_MODEL_HPP_

#include <array>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace drone_dynamics
{

// 四旋翼刚体和电机模型使用的固定参数，全部采用 SI 单位。
// 节点启动时从 dynamics.yaml 读取同名 ROS2 参数，再用该结构体构造模型。
struct QuadrotorParameters
{
  // 无人机总质量 m，单位 kg。
  double mass{1.0};

  // 机体系三个主轴的转动惯量，依次对应 x、y、z，单位 kg*m^2。
  // 当前假设 base_link 的三个轴与刚体惯量主轴重合。
  Eigen::Vector3d inertia{0.02, 0.02, 0.04};

  // 质心到单个电机旋转轴的距离 l，单位 m。
  // X 型布局中，机臂位于机体前后轴和左右轴之间的 45 度方向。
  double arm_length{0.20};

  // 推力系数，单位 N/(rad/s)^2，用于由电机角速度平方计算推力。
  double thrust_coefficient{1.91e-6};

  // 旋翼反扭矩系数，单位 N*m/(rad/s)^2，用于由角速度平方计算反扭矩。
  double drag_torque_coefficient{2.60e-7};

  // 电机一阶响应时间常数，单位 s；数值越小，实际转速追随命令越快。
  double motor_time_constant{0.05};

  // 外部 RPM 指令的允许范围。指令先被限制到该区间，再转换为 rad/s。
  double min_rpm{0.0};
  double max_rpm{20000.0};

  // 重力加速度大小 g，单位 m/s^2；重力向量在 ENU 世界系中为 [0,0,-g]。
  double gravity{9.80665};

  // 集总机体空气动力学。平动系数在机体系逐轴作用；linear_drag 的单位为
  // N/(m/s)，quadratic_drag 的单位为 N/(m/s)^2。angular_damping 的单位为
  // N*m/(rad/s)。它们不是旋翼气动辨识参数。
  bool enable_aerodynamic_drag{false};
  Eigen::Vector3d linear_drag{Eigen::Vector3d::Zero()};
  Eigen::Vector3d quadratic_drag{Eigen::Vector3d::Zero()};
  Eigen::Vector3d angular_damping{Eigen::Vector3d::Zero()};

  // 是否启用只约束世界系 z 方向的简化刚性地面。
  // 默认关闭，保证纯模型默认仍可执行不受地面影响的自由落体测试。
  bool enable_ground_contact{false};

  // 水平地面在世界坐标系中的 z 高度，单位 m。
  double ground_z{0.0};
};

// 模型积分过程中需要持续保存的完整状态。
// world 表示 map/ENU 世界坐标系，body 表示 base_link/FLU 机体坐标系。
struct QuadrotorState
{
  // 质心在世界系中的位置 p_world=[x,y,z]，单位 m。
  Eigen::Vector3d position_world{Eigen::Vector3d::Zero()};

  // 质心在世界系中的线速度 v_world，单位 m/s。
  Eigen::Vector3d velocity_world{Eigen::Vector3d::Zero()};

  // 将机体系向量旋转到世界系的单位四元数。
  // 默认单位四元数表示机体与世界系方向完全对齐。
  Eigen::Quaterniond orientation_body_to_world{Eigen::Quaterniond::Identity()};

  // 机体绕 base_link x/y/z 轴的角速度 [p,q,r]，单位 rad/s。
  Eigen::Vector3d angular_velocity_body{Eigen::Vector3d::Zero()};

  // 四个电机经过一阶响应后的“实际”角速度，单位 rad/s。
  // 数组顺序固定为 [M1前左CCW, M2后左CW, M3后右CCW, M4前右CW]。
  std::array<double, 4> motor_angular_velocity_rad_s{};
};

// 四个旋翼共同作用在机体质心上的合力/合力矩，均在 base_link 中表达。
struct BodyWrench
{
  // 沿机体系 +z 的合推力大小 T=F1+F2+F3+F4，单位 N。
  // 因当前模型不含其他机体系外力，所以只需要保存一个标量。
  double thrust{0.0};

  // 绕机体系 x/y/z 轴的 [roll, pitch, yaw] 合力矩，单位 N*m。
  Eigen::Vector3d torque{Eigen::Vector3d::Zero()};
};

// 与 ROS2 无关的纯四旋翼动力学模型。
//
// 输入：四个电机目标 RPM。
// 状态：位置、速度、姿态、角速度、四个电机实际转速。
// 每次调用 step()，模型依次更新电机实际转速、机体受力、线运动、角运动和姿态。
//
// 该类不负责 Topic、定时器、参数服务器和消息转换，因此可以独立做 GTest。
class QuadrotorModel
{
public:
  // 所有四电机数组统一使用 [M1, M2, M3, M4] 顺序。
  using MotorValues = std::array<double, 4>;

  // 保存并检查模型参数，然后把所有状态重置为初始值。
  explicit QuadrotorModel(const QuadrotorParameters & parameters);

  // 恢复到速度/角速度为 0、单位姿态、零电机转速的初始状态；地面开启时
  // 初始 z 使用 ground_z，关闭时初始位置为世界原点。
  void reset();

  // 设置一个有限、物理合法的初始刚体状态，主要供可重复的离线实验和纯模型
  // 测试使用。四元数会归一化；电机角速度仍须位于配置的物理范围。
  void set_state(const QuadrotorState & state);

  // 设置四电机外部 RPM 指令。函数内部负责非法值处理、上下限和单位转换；
  // 它只更新目标值，实际电机转速由后续 step() 中的一阶响应逐渐逼近。
  void set_motor_rpm_command(const MotorValues & motor_rpm);

  // 设置作用在质心上的外部扰动。force 使用 map/world 坐标系，torque 使用
  // base_link/body 坐标系；所有分量必须有限。ROS 适配层可进一步限制支持范围。
  void set_external_wrench(
    const Eigen::Vector3d & external_force_world,
    const Eigen::Vector3d & external_torque_body = Eigen::Vector3d::Zero());

  // 使用固定时间步长 dt（单位 s）将完整动力学状态向前推进一步。
  void step(double dt);

  // 以下只读接口供 ROS2 节点发布消息以及测试程序检查中间结果。
  const QuadrotorParameters & parameters() const;
  const QuadrotorState & state() const;
  const BodyWrench & body_wrench() const;

  // 最近一步的空气阻力（世界系）和角阻尼力矩（机体系）。开关关闭或对应
  // 速度为零时严格返回零，便于纯模型测试和运行诊断。
  const Eigen::Vector3d & aerodynamic_drag_force_world() const;
  const Eigen::Vector3d & aerodynamic_damping_torque_body() const;

  // 只计算给定状态下的集总气动力，不修改模型状态。
  Eigen::Vector3d calculate_aerodynamic_drag_force_world(
    const Eigen::Vector3d & velocity_world,
    const Eigen::Quaterniond & orientation_body_to_world) const;
  Eigen::Vector3d calculate_aerodynamic_damping_torque_body(
    const Eigen::Vector3d & angular_velocity_body) const;

  // 最近一次 step() 计算出的世界系质心线加速度，单位 m/s^2，包含重力。
  const Eigen::Vector3d & linear_acceleration_world() const;

  // 理想 IMU 在机体系测得的比力，单位 m/s^2，不包含重力自由落体项。
  Eigen::Vector3d specific_force_body() const;

private:
  // 基础输入和参数辅助函数。
  static double rpm_to_rad_s(double rpm);
  void validate_parameters() const;

  // 根据一阶电机模型更新四个实际角速度。
  void update_motor_response(double dt);

  // 根据实际电机角速度计算当前机体系合推力及三轴合力矩。
  BodyWrench calculate_body_wrench() const;

  // 当简化地面启用时，只限制世界系 z 位置和向下速度。
  void apply_ground_contact_constraint();

  // 构造后保持不变的物理参数。
  QuadrotorParameters parameters_;

  // 会随每个积分步更新的刚体与电机状态。
  QuadrotorState state_;

  // 由外部 RPM 指令转换得到的目标电机角速度，单位 rad/s；
  // 与 state_ 中的“实际角速度”分开保存，才能表示电机响应滞后。
  MotorValues commanded_motor_angular_velocity_rad_s_{};

  Eigen::Vector3d external_force_world_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d external_torque_body_{Eigen::Vector3d::Zero()};

  // 最近一个积分步的中间/输出量，缓存后可直接用于消息发布和测试。
  BodyWrench body_wrench_;
  Eigen::Vector3d aerodynamic_drag_force_world_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d aerodynamic_damping_torque_body_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d linear_acceleration_world_{Eigen::Vector3d::Zero()};
};

}  // namespace drone_dynamics

#endif  // DRONE_DYNAMICS__QUADROTOR_MODEL_HPP_
