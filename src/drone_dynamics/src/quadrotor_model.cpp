#include "drone_dynamics/quadrotor_model.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace drone_dynamics
{
namespace
{

// 将外部使用的 RPM 转换为模型内部使用的 rad/s。
constexpr double kRpmToRadiansPerSecond = 0.10471975511965977;

// 输入：一个待检查的数值。
// 计算：判断它既不是 NaN/Inf，又严格大于 0。
// 输出：参数可用于质量、惯量、时间常数等必须为正的物理量时返回 true。
bool is_positive_finite(const double value)
{
  return std::isfinite(value) && value > 0.0;
}

}  // namespace

// 输入：节点从参数文件读取后组成的全部物理参数。
// 读取：传入的 parameters。
// 计算：检查每个参数是否合法。
// 修改：保存 parameters_，并把模型状态初始化为静止、零转速状态。
// 物理意义：建立一个参数确定、初始状态确定的四旋翼模型。
QuadrotorModel::QuadrotorModel(const QuadrotorParameters & parameters)
: parameters_(parameters)
{
  validate_parameters();
  reset();
}

// 输入：无。
// 读取：parameters_ 中的重力大小。
// 计算：无动力学积分。
// 修改：清零位置、速度、角速度、电机指令和机体力矩；姿态恢复为单位四元数。
// 物理意义：无人机回到世界原点、机体与世界系对齐、电机停转的初始状态。
void QuadrotorModel::reset()
{
  state_ = QuadrotorState{};
  commanded_motor_angular_velocity_rad_s_.fill(0.0);
  body_wrench_ = BodyWrench{};

  // 此时旋翼没有推力，所以初始世界系线加速度只有向下的重力。
  linear_acceleration_world_ = Eigen::Vector3d(0.0, 0.0, -parameters_.gravity);
}

// 输入：四个电机目标转速，单位 RPM，顺序固定为 M1、M2、M3、M4。
// 读取：parameters_ 中的最小和最大 RPM。
// 计算：处理 NaN/Inf，执行 RPM 限幅，再把 RPM 转为 rad/s。
// 修改：commanded_motor_angular_velocity_rad_s_，不直接修改电机实际转速。
// 物理意义：记录驾驶员或控制器希望电机达到的目标转速。
void QuadrotorModel::set_motor_rpm_command(const MotorValues & motor_rpm)
{
  for (std::size_t index = 0; index < motor_rpm.size(); ++index) {
    // 异常输入按 0 RPM 处理，防止 NaN 进入后续积分。
    const double finite_rpm = std::isfinite(motor_rpm[index]) ? motor_rpm[index] : 0.0;

    // 电机命令不能超过参数规定的物理范围。
    const double clamped_rpm =
      std::clamp(finite_rpm, parameters_.min_rpm, parameters_.max_rpm);

    // ROS2 消息使用 RPM，动力学模型内部统一使用 rad/s。
    commanded_motor_angular_velocity_rad_s_[index] = rpm_to_rad_s(clamped_rpm);
  }
}

// 输入：本次仿真向前推进的时间 dt，单位 s。
// 读取：模型参数、当前完整状态、四电机目标转速。
// 计算：电机响应、推力和力矩、线加速度、角加速度以及姿态增量。
// 修改：电机实际转速、位置、速度、姿态、角速度和最近一次力/加速度结果。
// 物理意义：把四旋翼从“当前时刻状态”推进到“dt 秒后的状态”。
void QuadrotorModel::step(const double dt)
{
  // 第 1 步：检查 dt 是否有效。
  if (!is_positive_finite(dt)) {
    throw std::invalid_argument("Dynamics time step must be finite and greater than zero");
  }

  // 第 2 步：根据目标转速和电机时间常数，更新四个电机的实际转速。
  update_motor_response(dt);

  // 第 3 步：用四个实际转速计算总推力以及 roll、pitch、yaw 三轴力矩。
  body_wrench_ = calculate_body_wrench();

  // 第 4 步：机体总推力原本沿 base_link 的 +z。
  const Eigen::Vector3d thrust_body(0.0, 0.0, body_wrench_.thrust);

  // 第 5 步：把机体推力旋转到 map 世界系，加上世界系向下的重力，
  // 再除以质量，得到质心的世界系线加速度。
  const Eigen::Vector3d gravity_world(0.0, 0.0, -parameters_.gravity);
  linear_acceleration_world_ =
    state_.orientation_body_to_world * thrust_body / parameters_.mass + gravity_world;

  // 第 6 步：用当前机体系角速度和转动惯量计算角动量。
  const Eigen::Vector3d angular_momentum =
    parameters_.inertia.asDiagonal() * state_.angular_velocity_body;

  // 第 7 步：根据三轴力矩、转动惯量和刚体耦合项计算机体系角加速度。
  const Eigen::Vector3d angular_acceleration = parameters_.inertia.cwiseInverse().asDiagonal() *
    (body_wrench_.torque - state_.angular_velocity_body.cross(angular_momentum));

  // 第 8 步：用线加速度更新世界系速度，再用新速度更新世界系位置。
  state_.velocity_world += linear_acceleration_world_ * dt;
  state_.position_world += state_.velocity_world * dt;

  // 第 9 步：用角加速度更新机体系角速度。
  state_.angular_velocity_body += angular_acceleration * dt;

  // 第 10 步：把角速度在 dt 内形成的转动写成“旋转轴 + 旋转角”。
  const double angular_speed = state_.angular_velocity_body.norm();
  if (angular_speed > 1.0e-12) {
    const Eigen::Quaterniond incremental_rotation(
      Eigen::AngleAxisd(angular_speed * dt, state_.angular_velocity_body / angular_speed));

    // 角速度在机体系表达，因此增量旋转右乘到当前姿态，得到新姿态。
    state_.orientation_body_to_world =
      state_.orientation_body_to_world * incremental_rotation;
  }

  // 第 11 步：消除浮点累计误差，保证姿态四元数长度始终为 1。
  state_.orientation_body_to_world.normalize();
}

// 输入：无。读取：parameters_。修改：无。
// 输出：当前模型使用的只读物理参数。
const QuadrotorParameters & QuadrotorModel::parameters() const
{
  return parameters_;
}

// 输入：无。读取：state_。修改：无。
// 输出：当前时刻的只读位置、速度、姿态、角速度和电机实际转速。
const QuadrotorState & QuadrotorModel::state() const
{
  return state_;
}

// 输入：无。读取：body_wrench_。修改：无。
// 输出：最近一次 step() 算出的机体系总推力和三轴力矩。
const BodyWrench & QuadrotorModel::body_wrench() const
{
  return body_wrench_;
}

// 输入：无。读取：linear_acceleration_world_。修改：无。
// 输出：最近一次 step() 算出的世界系线加速度，其中包含重力。
const Eigen::Vector3d & QuadrotorModel::linear_acceleration_world() const
{
  return linear_acceleration_world_;
}

// 输入：无。
// 读取：最近一次总推力和无人机质量。
// 计算：总推力除以质量，方向为机体系 +z。
// 修改：无。
// 物理意义：返回理想 IMU 加速度计测到的机体系比力；自由落体时为 0。
Eigen::Vector3d QuadrotorModel::specific_force_body() const
{
  return Eigen::Vector3d(0.0, 0.0, body_wrench_.thrust / parameters_.mass);
}

// 输入：单个电机转速，单位 RPM。
// 计算：把“每分钟转数”转换成“每秒弧度数”。
// 修改：无。
// 输出：同一转速对应的 rad/s。
double QuadrotorModel::rpm_to_rad_s(const double rpm)
{
  return rpm * kRpmToRadiansPerSecond;
}

// 输入：无。
// 读取：parameters_ 的所有物理参数。
// 计算：检查有限性、正数要求和 RPM 上下限关系。
// 修改：无；发现错误时抛出异常并阻止模型运行。
// 物理意义：避免零质量、负惯量等没有物理意义的参数进入动力学计算。
void QuadrotorModel::validate_parameters() const
{
  if (!is_positive_finite(parameters_.mass)) {
    throw std::invalid_argument("mass must be finite and greater than zero");
  }
  if (!(parameters_.inertia.array().isFinite().all()) ||
    !(parameters_.inertia.array() > 0.0).all())
  {
    throw std::invalid_argument("all principal inertia values must be finite and greater than zero");
  }
  if (!is_positive_finite(parameters_.arm_length)) {
    throw std::invalid_argument("arm_length must be finite and greater than zero");
  }
  if (!is_positive_finite(parameters_.thrust_coefficient)) {
    throw std::invalid_argument("thrust_coefficient must be finite and greater than zero");
  }
  if (!is_positive_finite(parameters_.drag_torque_coefficient)) {
    throw std::invalid_argument("drag_torque_coefficient must be finite and greater than zero");
  }
  if (!is_positive_finite(parameters_.motor_time_constant)) {
    throw std::invalid_argument("motor_time_constant must be finite and greater than zero");
  }
  if (!std::isfinite(parameters_.min_rpm) || parameters_.min_rpm < 0.0 ||
    !std::isfinite(parameters_.max_rpm) || parameters_.max_rpm <= parameters_.min_rpm)
  {
    throw std::invalid_argument("RPM limits must be finite and satisfy 0 <= min_rpm < max_rpm");
  }
  if (!is_positive_finite(parameters_.gravity)) {
    throw std::invalid_argument("gravity must be finite and greater than zero");
  }
}

// 输入：仿真步长 dt。
// 读取：目标电机转速、电机实际转速和电机时间常数。
// 计算：求出本时间步内电机能够完成的响应比例。
// 修改：state_ 中四个电机的实际角速度。
// 物理意义：电机不能瞬间达到命令值，而是以一阶响应逐渐逼近目标值。
void QuadrotorModel::update_motor_response(const double dt)
{
  const double response_fraction = 1.0 - std::exp(-dt / parameters_.motor_time_constant);
  for (std::size_t index = 0; index < state_.motor_angular_velocity_rad_s.size(); ++index) {
    // 实际转速向目标转速移动 response_fraction 比例。
    state_.motor_angular_velocity_rad_s[index] += response_fraction *
      (commanded_motor_angular_velocity_rad_s_[index] -
      state_.motor_angular_velocity_rad_s[index]);
  }
}

// 输入：无。
// 读取：四个电机实际角速度、推力/反扭矩系数和机臂长度。
// 计算：每个电机推力、每个旋翼反扭矩、总推力以及三轴合力矩。
// 修改：无。
// 输出：当前时刻作用在 base_link 质心上的 BodyWrench。
// 物理意义：把四个独立电机的作用合成为刚体平动和转动所需的输入。
BodyWrench QuadrotorModel::calculate_body_wrench() const
{
  std::array<double, 4> thrust{};
  std::array<double, 4> reaction_torque{};

  // 第 1 步：实际转速平方后乘相应系数，得到每个电机的推力和反扭矩大小。
  for (std::size_t index = 0; index < thrust.size(); ++index) {
    const double squared_speed =
      state_.motor_angular_velocity_rad_s[index] *
      state_.motor_angular_velocity_rad_s[index];
    thrust[index] = parameters_.thrust_coefficient * squared_speed;
    reaction_torque[index] = parameters_.drag_torque_coefficient * squared_speed;
  }

  // 第 2 步：X 型布局中，机臂在 x、y 方向的有效长度相同。
  const double moment_arm = parameters_.arm_length / std::sqrt(2.0);
  BodyWrench wrench;

  // 第 3 步：四个向上推力相加，得到沿机体系 +z 的总推力。
  wrench.thrust = thrust[0] + thrust[1] + thrust[2] + thrust[3];

  // 第 4 步：左侧 M1/M2 增推产生正 roll；右侧 M3/M4 产生负 roll。
  wrench.torque.x() = moment_arm * (thrust[0] + thrust[1] - thrust[2] - thrust[3]);

  // 第 5 步：后侧 M2/M3 增推产生正 pitch；前侧 M1/M4 产生负 pitch。
  wrench.torque.y() = moment_arm * (-thrust[0] + thrust[1] + thrust[2] - thrust[3]);

  // 第 6 步：M1/M3 为 CCW，对机体产生负 yaw；M2/M4 为 CW，产生正 yaw。
  wrench.torque.z() =
    -reaction_torque[0] + reaction_torque[1] - reaction_torque[2] + reaction_torque[3];
  return wrench;
}

}  // namespace drone_dynamics
