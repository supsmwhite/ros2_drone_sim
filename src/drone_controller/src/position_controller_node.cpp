#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "drone_controller/hover/hover_controller.hpp"
#include "drone_msgs/msg/motor_rpm.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

namespace drone_controller
{

// 这个节点是本包中唯一一个依赖 rclcpp 的类，相当于给 HoverController
// 包上一层“ROS2 话题适配器”：订阅话题 -> 组装 HoverControllerInput ->
// 调用 HoverController::compute() -> 把 HoverControllerResult 拆分发布到话题。
class PositionControllerNode : public rclcpp::Node
{
public:
  PositionControllerNode()
  : Node("position_controller_node")
  {
    // 节点自身的三个循环参数：控制频率、Odom 超时时长、支持的目标 frame。
    const double control_frequency = declare_parameter<double>("control_frequency", 100.0);
    odometry_timeout_ = declare_parameter<double>("odometry_timeout", 0.2);
    supported_goal_frame_ = declare_parameter<std::string>("supported_goal_frame", "map");
    if (!std::isfinite(control_frequency) || control_frequency <= 0.0 ||
      !std::isfinite(odometry_timeout_) || odometry_timeout_ <= 0.0 ||
      supported_goal_frame_.empty())
    {
      throw std::invalid_argument("Invalid controller loop parameters");
    }

    // 把所有控制参数读入后，一次性构造 HoverController（又会依次构造
    // AltitudeController/AttitudeController/MotorMixer，各自在构造函数里校验参数）。
    hover_controller_ = std::make_unique<HoverController>(read_hover_parameters());
    // 订阅 /drone/goal：回调只做最简单的缓存，不在回调里做任何计算，
    // 真正的处理集中在 control_step() 中按固定频率执行。
    goal_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/drone/goal", 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr message) {
        latest_goal_ = *message;
      });
    // 订阅 /drone/odom：除了缓存消息本身，还记录接收时刻，用于后续判断
    // Odom 是否过期（watchdog 的一部分）。
    odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      "/drone/odom", 10,
      [this](const nav_msgs::msg::Odometry::SharedPtr message) {
        latest_odometry_ = *message;
        latest_odometry_reception_time_ = now();
      });
    motor_rpm_publisher_ =
      create_publisher<drone_msgs::msg::MotorRPM>("/drone/motor_rpm_cmd", 10);

    // 根据 control_frequency 创建定时器，每个周期调用一次 control_step()，
    // 这是整个控制环的驱动源（默认 100 Hz，与动力学 200 Hz 独立）。
    const auto period = std::chrono::duration<double>(1.0 / control_frequency);
    control_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() {control_step();});

    RCLCPP_INFO(
      get_logger(),
      "Altitude-hover controller started at %.1f Hz (odom timeout %.3f s); waiting for a map goal",
      control_frequency, odometry_timeout_);
  }

private:
  // 把 HoverControllerParameters 里三个子结构体的每个字段都声明成 ROS2 参数，
  // 默认值取自头文件定义的结构体默认值，允许 YAML/命令行覆盖。
  // 这里只是单纯的参数搜集，不包含任何算法逻辑。
  HoverControllerParameters read_hover_parameters()
  {
    HoverControllerParameters parameters;
    parameters.altitude.mass = declare_parameter<double>("mass", parameters.altitude.mass);
    parameters.altitude.gravity =
      declare_parameter<double>("gravity", parameters.altitude.gravity);
    parameters.altitude.altitude_kp =
      declare_parameter<double>("altitude_kp", parameters.altitude.altitude_kp);
    parameters.altitude.vertical_velocity_kd = declare_parameter<double>(
      "vertical_velocity_kd", parameters.altitude.vertical_velocity_kd);
    parameters.altitude.max_upward_acceleration = declare_parameter<double>(
      "max_upward_acceleration", parameters.altitude.max_upward_acceleration);
    parameters.altitude.max_downward_acceleration = declare_parameter<double>(
      "max_downward_acceleration", parameters.altitude.max_downward_acceleration);
    parameters.altitude.min_collective_thrust = declare_parameter<double>(
      "min_collective_thrust", parameters.altitude.min_collective_thrust);
    parameters.altitude.max_collective_thrust = declare_parameter<double>(
      "max_collective_thrust", parameters.altitude.max_collective_thrust);
    parameters.altitude.min_tilt_cosine = declare_parameter<double>(
      "min_tilt_cosine", parameters.altitude.min_tilt_cosine);

    parameters.attitude.attitude_kp = Eigen::Vector3d(
      declare_parameter<double>("attitude_kp_roll", parameters.attitude.attitude_kp.x()),
      declare_parameter<double>("attitude_kp_pitch", parameters.attitude.attitude_kp.y()),
      declare_parameter<double>("attitude_kp_yaw", parameters.attitude.attitude_kp.z()));
    parameters.attitude.angular_rate_kd = Eigen::Vector3d(
      declare_parameter<double>(
        "angular_rate_kd_roll", parameters.attitude.angular_rate_kd.x()),
      declare_parameter<double>(
        "angular_rate_kd_pitch", parameters.attitude.angular_rate_kd.y()),
      declare_parameter<double>(
        "angular_rate_kd_yaw", parameters.attitude.angular_rate_kd.z()));
    parameters.attitude.max_torque = Eigen::Vector3d(
      declare_parameter<double>("max_torque_roll", parameters.attitude.max_torque.x()),
      declare_parameter<double>("max_torque_pitch", parameters.attitude.max_torque.y()),
      declare_parameter<double>("max_torque_yaw", parameters.attitude.max_torque.z()));

    parameters.mixer.arm_length =
      declare_parameter<double>("arm_length", parameters.mixer.arm_length);
    parameters.mixer.thrust_coefficient = declare_parameter<double>(
      "thrust_coefficient", parameters.mixer.thrust_coefficient);
    parameters.mixer.drag_torque_coefficient = declare_parameter<double>(
      "drag_torque_coefficient", parameters.mixer.drag_torque_coefficient);
    parameters.mixer.min_rpm =
      declare_parameter<double>("min_rpm", parameters.mixer.min_rpm);
    parameters.mixer.max_rpm =
      declare_parameter<double>("max_rpm", parameters.mixer.max_rpm);
    return parameters;
  }

  // 安全处理失败分支的统一出口：发布一条默认构造的 MotorRPM（各字段均为 0）。
  void publish_zero_rpm()
  {
    motor_rpm_publisher_->publish(drone_msgs::msg::MotorRPM{});
  }

  // 每个控制周期（默认 100 Hz）执行一次，是本节点的核心。
  // 整体结构是一条“安全检查链”：任一环不满足就立即 publish_zero_rpm() 并 return，
  // 只有全部通过才会真正调用 HoverController::compute()。
  void control_step()
  {
    // 检查 1：尚未收到任何目标。
    if (!latest_goal_) {
      publish_zero_rpm();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting for /drone/goal; RPM=0");
      return;
    }
    // 检查 2：尚未收到任何 Odom。
    if (!latest_odometry_ || !latest_odometry_reception_time_) {
      publish_zero_rpm();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting for /drone/odom; RPM=0");
      return;
    }
    // 检查 3：最新 Odom 已经过期（与动力学侧的 MotorRPM watchdog 相对应，
    // 但这里检查的是控制器读到的 Odom 是否新鲜，两者方向相反）。
    const double odometry_age = (now() - *latest_odometry_reception_time_).seconds();
    if (!std::isfinite(odometry_age) || odometry_age > odometry_timeout_) {
      publish_zero_rpm();
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000, "Odometry stale (%.3f s); RPM=0", odometry_age);
      return;
    }

    // 检查 4：目标 frame 合法性。空字符串按 map 处理，非空且不等于 supported_goal_frame_
    // （默认 "map"）时拒绝执行。
    const std::string & frame = latest_goal_->header.frame_id;
    if (!frame.empty() && frame != supported_goal_frame_) {
      publish_zero_rpm();
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000, "Unsupported goal frame '%s'; RPM=0", frame.c_str());
      return;
    }

    // ===== 以下开始把 ROS 消息转换为算法层需要的 Eigen 类型 =====
    const auto & goal_pose = latest_goal_->pose;
    const auto & odometry = *latest_odometry_;
    const auto & current_pose = odometry.pose.pose;
    // 注意 Eigen::Quaterniond 的构造参数顺序是 (w,x,y,z)，
    // 与 ROS 消息里的 orientation 字段顺序不同，这里手动对齐。
    const Eigen::Quaterniond goal_orientation(
      goal_pose.orientation.w, goal_pose.orientation.x,
      goal_pose.orientation.y, goal_pose.orientation.z);
    const Eigen::Quaterniond current_orientation(
      current_pose.orientation.w, current_pose.orientation.x,
      current_pose.orientation.y, current_pose.orientation.z);
    // 调用 hover_controller.hpp 里的纯函数：只从目标四元数提取 yaw，
    // 生成 roll=pitch=0 的期望姿态，这就是目前忽略目标 x/y 后仍能使用的
    // “只控 z+yaw”策略具体落地处。
    Eigen::Quaterniond desired_level_orientation;
    if (!level_orientation_from_goal_yaw(goal_orientation, desired_level_orientation)) {
      publish_zero_rpm();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Invalid goal quaternion; RPM=0");
      return;
    }

    // 检查 5：把 Odom 机体系（base_link）的 twist.linear 旋转到世界系，
    // 取其 z 分量作为世界系垂直速度。这一步对应 AI_CONTEXT 里特别强调的
    // “Odom twist 在机体系，不能直接当世界系 vz 用”警告。
    const Eigen::Vector3d velocity_body(
      odometry.twist.twist.linear.x, odometry.twist.twist.linear.y,
      odometry.twist.twist.linear.z);
    double vertical_velocity_world = 0.0;
    if (!world_vertical_velocity_from_body(
        current_orientation, velocity_body, vertical_velocity_world))
    {
      publish_zero_rpm();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Invalid odometry state; RPM=0");
      return;
    }

    // ===== 组装 HoverControllerInput，这里是 ROS 数据真正注入算法层的入口 =====
    // 目标 x/y（goal_pose.position.x/y）在这里被明确忽略，不赋值给任何字段，
    // 这就是 README/AI_CONTEXT 中反复提到的当前限制的具体代码体现。
    HoverControllerInput input;
    input.desired_altitude = goal_pose.position.z;
    input.desired_orientation_body_to_world = desired_level_orientation;
    input.current_altitude = current_pose.position.z;
    input.current_vertical_velocity_world = vertical_velocity_world;
    input.current_orientation_body_to_world = current_orientation;
    input.current_angular_velocity_body = Eigen::Vector3d(
      odometry.twist.twist.angular.x, odometry.twist.twist.angular.y,
      odometry.twist.twist.angular.z);
    // 真正的控制计算全部发生在这一行：依次调用 AltitudeController、
    // AttitudeController、MotorMixer，即之前分析过的整条链路。
    const HoverControllerResult result = hover_controller_->compute(input);
    if (!result.valid) {
      publish_zero_rpm();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Invalid control result; RPM=0");
      return;
    }

    // ===== 把 HoverControllerResult 四个 RPM 按固定编号回填到 MotorRPM 消息 =====
    // 字段命名与 AI_CONTEXT 中 M1~M4 的固定编号/旋向约定一致，不允许错位。
    drone_msgs::msg::MotorRPM message;
    message.m1_front_left_ccw_rpm = result.motor_rpm[0];
    message.m2_rear_left_cw_rpm = result.motor_rpm[1];
    message.m3_rear_right_ccw_rpm = result.motor_rpm[2];
    message.m4_front_right_cw_rpm = result.motor_rpm[3];
    motor_rpm_publisher_->publish(message);
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "hover target_z=%.3f current_z=%.3f world_vz=%.3f thrust=%.3f "
      "rpm=[%.1f, %.1f, %.1f, %.1f] saturated=%s",
      input.desired_altitude, input.current_altitude, input.current_vertical_velocity_world,
      result.collective_thrust, result.motor_rpm[0], result.motor_rpm[1], result.motor_rpm[2],
      result.motor_rpm[3], result.saturated ? "true" : "false");
  }

  std::unique_ptr<HoverController> hover_controller_;
  double odometry_timeout_{0.2};
  std::string supported_goal_frame_{"map"};
  std::optional<geometry_msgs::msg::PoseStamped> latest_goal_;
  std::optional<nav_msgs::msg::Odometry> latest_odometry_;
  std::optional<rclcpp::Time> latest_odometry_reception_time_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  rclcpp::Publisher<drone_msgs::msg::MotorRPM>::SharedPtr motor_rpm_publisher_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

}  // namespace drone_controller

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<drone_controller::PositionControllerNode>());
  rclcpp::shutdown();
  return 0;
}
