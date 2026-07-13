#include <chrono>
#include <cmath>
#include <cstddef>
#include <memory>
#include <stdexcept>

#include "drone_dynamics/quadrotor_model.hpp"
#include "drone_msgs/msg/motor_rpm.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace drone_dynamics
{

// QuadrotorDynamicsNode 是 ROS2 与纯算法模型之间的适配层：
// 1. 从 ROS2 参数服务器读取模型参数；
// 2. 将 MotorRPM 消息转换为模型输入；
// 3. 用固定步长定时调用 QuadrotorModel::step()；
// 4. 将模型状态转换为 Odom、IMU、Path 和 TF。
// 具体动力学公式全部放在 QuadrotorModel 中，节点不重复实现物理计算。
class QuadrotorDynamicsNode : public rclcpp::Node
{
public:
  // 输入：ROS2 参数文件、/drone/motor_rpm_cmd Topic，以及墙钟定时器事件。
  // 读取：全部动力学参数、仿真频率和 Path 配置。
  // 计算：创建模型、计算固定步长和名义悬停 RPM。
  // 修改：创建订阅器、发布器、TF 广播器和仿真定时器。
  // 物理意义：把纯动力学模型接入 ROS2 通信和周期运行环境。
  QuadrotorDynamicsNode()
  : Node("quadrotor_dynamics_node")
  {
    // dynamics.yaml 中的值会覆盖 declare_parameter() 提供的默认值。
    // QuadrotorParameters 只包含物理模型参数；发布频率和 Path 设置属于节点参数。
    const QuadrotorParameters parameters = declare_model_parameters();
    const double simulation_frequency =
      declare_parameter<double>("simulation_frequency", 200.0);
    path_publish_divider_ = declare_parameter<int>("path_publish_divider", 10);
    path_max_points_ = declare_parameter<int>("path_max_points", 2000);
    enable_motor_command_timeout_ =
      declare_parameter<bool>("enable_motor_command_timeout", true);
    motor_command_timeout_ = declare_parameter<double>("motor_command_timeout", 0.30);

    if (!std::isfinite(simulation_frequency) || simulation_frequency <= 0.0) {
      throw std::invalid_argument("simulation_frequency must be finite and greater than zero");
    }
    if (path_publish_divider_ <= 0) {
      throw std::invalid_argument("path_publish_divider must be greater than zero");
    }
    if (path_max_points_ <= 0) {
      throw std::invalid_argument("path_max_points must be greater than zero");
    }
    if (!std::isfinite(motor_command_timeout_) || motor_command_timeout_ <= 0.0) {
      throw std::invalid_argument(
              "motor_command_timeout must be finite and greater than zero");
    }

    // 模型使用固定仿真步长 dt=1/f，而不是直接使用两次回调之间有抖动的墙钟差值。
    // 例如 200 Hz 对应 dt=0.005 s。
    fixed_time_step_ = 1.0 / simulation_frequency;
    model_ = std::make_unique<QuadrotorModel>(parameters);

    // MotorRPM 字段顺序已经显式包含电机编号、位置和旋转方向；这里保持
    // [M1 前左, M2 后左, M3 后右, M4 前右] 的顺序传入算法模型。
    motor_rpm_subscription_ = create_subscription<drone_msgs::msg::MotorRPM>(
      "/drone/motor_rpm_cmd", 10,
      [this](const drone_msgs::msg::MotorRPM::SharedPtr message) {
        model_->set_motor_rpm_command({
          message->m1_front_left_ccw_rpm,
          message->m2_rear_left_cw_rpm,
          message->m3_rear_right_ccw_rpm,
          message->m4_front_right_cw_rpm});
        last_motor_command_time_ = std::chrono::steady_clock::now();
        has_received_motor_command_ = true;
        if (motor_command_timed_out_) {
          motor_command_timed_out_ = false;
          RCLCPP_INFO(get_logger(), "Fresh MotorRPM command received; watchdog recovered");
        }
      });

    // 三个状态 Topic 使用深度为 10 的普通 ROS2 QoS。TF 使用专用广播器发布到 /tf。
    odometry_publisher_ = create_publisher<nav_msgs::msg::Odometry>("/drone/odom", 10);
    imu_publisher_ = create_publisher<sensor_msgs::msg::Imu>("/drone/imu", 10);
    path_publisher_ = create_publisher<nav_msgs::msg::Path>("/drone/path", 10);
    transform_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // create_wall_timer 需要 chrono duration；将 double 秒转换成纳秒周期。
    const auto timer_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(fixed_time_step_));
    simulation_timer_ = create_wall_timer(timer_period, [this]() {simulation_step();});

    // 计算四个电机共同承担重力时，每个电机所需的名义稳态转速。
    // 此值只是推力平衡点，不会消除启动阶段已经产生的位置或速度误差。
    const double hover_angular_velocity = std::sqrt(
      parameters.mass * parameters.gravity / (4.0 * parameters.thrust_coefficient));
    const double hover_rpm = hover_angular_velocity * 30.0 / 3.14159265358979323846;
    RCLCPP_INFO(
      get_logger(),
      "Dynamics started at %.1f Hz; nominal steady-state hover RPM is %.1f; "
      "ground contact is %s at z=%.3f m; motor command watchdog is %s (%.3f s)",
      simulation_frequency, hover_rpm,
      parameters.enable_ground_contact ? "enabled" : "disabled", parameters.ground_z,
      enable_motor_command_timeout_ ? "enabled" : "disabled", motor_command_timeout_);
  }

private:
  // 输入：ROS2 参数服务器中与模型同名的参数。
  // 读取：QuadrotorParameters 自带的默认值，或 dynamics.yaml 的覆盖值。
  // 计算：无动力学计算，只完成参数收集。
  // 修改：ROS2 节点的已声明参数集合。
  // 输出：可直接用于构造 QuadrotorModel 的参数对象。
  QuadrotorParameters declare_model_parameters()
  {
    // 先构造带默认值的参数对象，再逐项向 ROS2 声明同名参数。
    // 单位分别为 kg、kg*m^2、m、N/(rad/s)^2、N*m/(rad/s)^2、s、RPM、m/s^2。
    QuadrotorParameters parameters;
    parameters.mass = declare_parameter<double>("mass", parameters.mass);
    parameters.inertia.x() = declare_parameter<double>("inertia_xx", parameters.inertia.x());
    parameters.inertia.y() = declare_parameter<double>("inertia_yy", parameters.inertia.y());
    parameters.inertia.z() = declare_parameter<double>("inertia_zz", parameters.inertia.z());
    parameters.arm_length = declare_parameter<double>("arm_length", parameters.arm_length);
    parameters.thrust_coefficient =
      declare_parameter<double>("thrust_coefficient", parameters.thrust_coefficient);
    parameters.drag_torque_coefficient = declare_parameter<double>(
      "drag_torque_coefficient", parameters.drag_torque_coefficient);
    parameters.motor_time_constant =
      declare_parameter<double>("motor_time_constant", parameters.motor_time_constant);
    parameters.min_rpm = declare_parameter<double>("min_rpm", parameters.min_rpm);
    parameters.max_rpm = declare_parameter<double>("max_rpm", parameters.max_rpm);
    parameters.gravity = declare_parameter<double>("gravity", parameters.gravity);
    parameters.enable_ground_contact =
      declare_parameter<bool>("enable_ground_contact", parameters.enable_ground_contact);
    parameters.ground_z = declare_parameter<double>("ground_z", parameters.ground_z);
    return parameters;
  }

  // 输入：一次墙钟定时器事件。
  // 读取：fixed_time_step_、当前模型状态和 Path 发布分频设置。
  // 计算：让模型前进一步，并生成统一时间戳。
  // 修改：模型完整状态、Odom/IMU/TF 输出以及按较低频率更新的 Path。
  // 物理意义：这是 ROS2 节点每一个仿真周期的总流程。
  void simulation_step()
  {
    check_motor_command_timeout();

    // 每次定时器回调只推进一个固定 dt。Odom、IMU 和 TF 每个仿真步都发布，
    // 因而名义频率与 simulation_frequency 相同。
    model_->step(fixed_time_step_);
    const rclcpp::Time stamp = now();
    publish_odometry(stamp);
    publish_imu(stamp);
    publish_transform(stamp);

    // Path 消息包含历史位姿，体积会随时间增长，因此降低发布/记录频率。
    // 默认 divider=10，在 200 Hz 仿真下每 10 步记录一次，即 20 Hz。
    ++simulation_step_count_;
    if (simulation_step_count_ % static_cast<std::size_t>(path_publish_divider_) == 0U) {
      publish_path(stamp);
    }
  }

  // 使用不受 ROS 时间跳变影响的单调时钟检查命令新鲜度。超时只修改模型的
  // 目标 RPM；实际电机转速仍由 QuadrotorModel 的一阶响应自然衰减。
  void check_motor_command_timeout()
  {
    if (!enable_motor_command_timeout_ || !has_received_motor_command_) {
      return;
    }

    const double command_age = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - last_motor_command_time_).count();
    if (command_age <= motor_command_timeout_) {
      return;
    }

    if (!motor_command_timed_out_) {
      model_->set_motor_rpm_command({0.0, 0.0, 0.0, 0.0});
      motor_command_timed_out_ = true;
      motor_command_timeout_started_ = std::chrono::steady_clock::now();
      RCLCPP_WARN(
        get_logger(),
        "MotorRPM command timed out after %.3f s; target RPM set to zero",
        command_age);
      return;
    }

    const double timeout_duration = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - motor_command_timeout_started_).count();
    if (timeout_duration < 5.0) {
      return;
    }

    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "MotorRPM command remains timed out; target RPM is zero");
  }

  // 输入：本次发布使用的 ROS2 时间戳。
  // 读取：模型中的世界系位置/速度、姿态和机体系角速度。
  // 计算：把世界系线速度转换为 base_link 机体系线速度。
  // 修改：不修改模型，只发布 /drone/odom。
  // 物理意义：向其他节点提供无人机相对 map 的位姿和运动状态。
  void publish_odometry(const rclcpp::Time & stamp)
  {
    const QuadrotorState & state = model_->state();
    nav_msgs::msg::Odometry message;

    // pose 表示 base_link 在 map 中的位置和姿态：
    // position_world 直接是 ENU 世界系坐标；orientation_body_to_world
    // 是将 base_link(FLU) 向量旋转到 map(ENU) 的单位四元数。
    message.header.stamp = stamp;
    message.header.frame_id = "map";
    message.child_frame_id = "base_link";
    message.pose.pose.position.x = state.position_world.x();
    message.pose.pose.position.y = state.position_world.y();
    message.pose.pose.position.z = state.position_world.z();
    message.pose.pose.orientation.x = state.orientation_body_to_world.x();
    message.pose.pose.orientation.y = state.orientation_body_to_world.y();
    message.pose.pose.orientation.z = state.orientation_body_to_world.z();
    message.pose.pose.orientation.w = state.orientation_body_to_world.w();

    // 模型内部保存的是世界系线速度 v_world。nav_msgs/Odometry 的 twist
    // 按 child_frame_id=base_link 表达，因此使用姿态的逆旋转把它转换到机体系。
    // 角速度在模型内本来就是机体系，无需转换。
    const Eigen::Vector3d velocity_body =
      state.orientation_body_to_world.conjugate() * state.velocity_world;
    message.twist.twist.linear.x = velocity_body.x();
    message.twist.twist.linear.y = velocity_body.y();
    message.twist.twist.linear.z = velocity_body.z();
    message.twist.twist.angular.x = state.angular_velocity_body.x();
    message.twist.twist.angular.y = state.angular_velocity_body.y();
    message.twist.twist.angular.z = state.angular_velocity_body.z();
    odometry_publisher_->publish(message);
  }

  // 输入：本次发布使用的 ROS2 时间戳。
  // 读取：模型姿态、机体系角速度和机体系比力。
  // 计算：只进行 Eigen 到 ROS2 消息字段的转换。
  // 修改：不修改模型，只发布 /drone/imu。
  // 物理意义：模拟安装在质心、坐标轴与 base_link 对齐的理想 IMU。
  void publish_imu(const rclcpp::Time & stamp)
  {
    const QuadrotorState & state = model_->state();
    // specific_force 是理想加速度计在机体系测得的比力。
    // 它不等于世界系 a_world；自由落体时其值为 0，水平悬停时 z 约为 +g。
    const Eigen::Vector3d specific_force = model_->specific_force_body();
    sensor_msgs::msg::Imu message;

    // 假设 IMU 安装在质心、坐标轴与 base_link 完全重合，且暂不模拟噪声和偏置。
    message.header.stamp = stamp;
    message.header.frame_id = "base_link";
    message.orientation.x = state.orientation_body_to_world.x();
    message.orientation.y = state.orientation_body_to_world.y();
    message.orientation.z = state.orientation_body_to_world.z();
    message.orientation.w = state.orientation_body_to_world.w();
    message.angular_velocity.x = state.angular_velocity_body.x();
    message.angular_velocity.y = state.angular_velocity_body.y();
    message.angular_velocity.z = state.angular_velocity_body.z();
    message.linear_acceleration.x = specific_force.x();
    message.linear_acceleration.y = specific_force.y();
    message.linear_acceleration.z = specific_force.z();
    imu_publisher_->publish(message);
  }

  // 输入：本次发布使用的 ROS2 时间戳。
  // 读取：模型的世界系位置和机体到世界的姿态。
  // 计算：组装 map 到 base_link 的刚体变换。
  // 修改：不修改模型，只通过 /tf 广播变换。
  // 物理意义：让 RViz 和其他 TF 使用者知道无人机在世界中的位置和朝向。
  void publish_transform(const rclcpp::Time & stamp)
  {
    const QuadrotorState & state = model_->state();
    geometry_msgs::msg::TransformStamped transform;

    // TF map -> base_link 与 Odom pose 使用同一组位置和姿态，保证可视化与
    // /drone/odom 不会出现坐标不一致。
    transform.header.stamp = stamp;
    transform.header.frame_id = "map";
    transform.child_frame_id = "base_link";
    transform.transform.translation.x = state.position_world.x();
    transform.transform.translation.y = state.position_world.y();
    transform.transform.translation.z = state.position_world.z();
    transform.transform.rotation.x = state.orientation_body_to_world.x();
    transform.transform.rotation.y = state.orientation_body_to_world.y();
    transform.transform.rotation.z = state.orientation_body_to_world.z();
    transform.transform.rotation.w = state.orientation_body_to_world.w();
    transform_broadcaster_->sendTransform(transform);
  }

  // 输入：本次发布使用的 ROS2 时间戳。
  // 读取：当前世界系位置、姿态和已有 Path 历史。
  // 计算：把当前位姿追加到轨迹，并在超限时删除最老点。
  // 修改：path_ 历史缓存，并发布 /drone/path。
  // 物理意义：保存无人机最近一段时间经过的空间轨迹。
  void publish_path(const rclcpp::Time & stamp)
  {
    const QuadrotorState & state = model_->state();
    geometry_msgs::msg::PoseStamped pose;

    // Path 中每个 PoseStamped 都在 map 坐标系表达。
    pose.header.stamp = stamp;
    pose.header.frame_id = "map";
    pose.pose.position.x = state.position_world.x();
    pose.pose.position.y = state.position_world.y();
    pose.pose.position.z = state.position_world.z();
    pose.pose.orientation.x = state.orientation_body_to_world.x();
    pose.pose.orientation.y = state.orientation_body_to_world.y();
    pose.pose.orientation.z = state.orientation_body_to_world.z();
    pose.pose.orientation.w = state.orientation_body_to_world.w();

    path_.header = pose.header;
    path_.poses.push_back(pose);

    // 限制历史点数，避免长时间运行时 Path 无限占用内存。
    // 超过上限后删除最老的点，只保留最近 path_max_points_ 个位姿。
    if (path_.poses.size() > static_cast<std::size_t>(path_max_points_)) {
      path_.poses.erase(path_.poses.begin());
    }
    path_publisher_->publish(path_);
  }

  // 算法模型及仿真/轨迹配置。
  std::unique_ptr<QuadrotorModel> model_;
  double fixed_time_step_{0.005};
  int path_publish_divider_{10};
  int path_max_points_{2000};
  std::size_t simulation_step_count_{0};
  nav_msgs::msg::Path path_;
  bool enable_motor_command_timeout_{true};
  double motor_command_timeout_{0.30};
  bool has_received_motor_command_{false};
  bool motor_command_timed_out_{false};
  std::chrono::steady_clock::time_point last_motor_command_time_{};
  std::chrono::steady_clock::time_point motor_command_timeout_started_{};

  // ROS2 通信对象。用成员变量持有 shared_ptr，保证订阅、发布器和定时器
  // 在节点整个生命周期内持续有效。
  rclcpp::Subscription<drone_msgs::msg::MotorRPM>::SharedPtr motor_rpm_subscription_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;
  rclcpp::TimerBase::SharedPtr simulation_timer_;
};

}  // namespace drone_dynamics

int main(int argc, char * argv[])
{
  // 输入：命令行中的 ROS2 参数和重映射选项。
  // 计算：初始化 ROS2，创建动力学节点并持续处理订阅和定时器回调。
  // 修改：建立本进程的 ROS2 通信资源；Ctrl-C 后统一清理。
  // 物理意义：无；这是节点进程的程序入口和生命周期管理。
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<drone_dynamics::QuadrotorDynamicsNode>());
  rclcpp::shutdown();
  return 0;
}
