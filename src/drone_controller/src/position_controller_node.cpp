#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "drone_controller/position/position_controller.hpp"
#include "drone_msgs/msg/motor_rpm.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

namespace drone_controller
{

// ROS2 adapter: validates Goal/Odom, converts body velocity to world velocity,
// calls the ROS-independent PositionController, and publishes motor RPM.
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

    position_controller_ = std::make_unique<PositionController>(read_position_parameters());
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
      "3D position controller started at %.1f Hz (odom timeout %.3f s); waiting for a map goal",
      control_frequency, odometry_timeout_);
  }

private:
  PositionControllerParameters read_position_parameters()
  {
    PositionControllerParameters parameters;
    const double gravity =
      declare_parameter<double>("gravity", parameters.hover.altitude.gravity);
    parameters.horizontal.gravity = gravity;
    parameters.hover.altitude.gravity = gravity;
    parameters.horizontal.position_kp = Eigen::Vector2d(
      declare_parameter<double>(
        "horizontal_position_kp_x", parameters.horizontal.position_kp.x()),
      declare_parameter<double>(
        "horizontal_position_kp_y", parameters.horizontal.position_kp.y()));
    parameters.horizontal.velocity_kd = Eigen::Vector2d(
      declare_parameter<double>(
        "horizontal_velocity_kd_x", parameters.horizontal.velocity_kd.x()),
      declare_parameter<double>(
        "horizontal_velocity_kd_y", parameters.horizontal.velocity_kd.y()));
    parameters.horizontal.max_horizontal_acceleration = declare_parameter<double>(
      "max_horizontal_acceleration", parameters.horizontal.max_horizontal_acceleration);
    parameters.horizontal.max_tilt_angle =
      declare_parameter<double>("max_tilt_angle", parameters.horizontal.max_tilt_angle);

    parameters.hover.altitude.mass =
      declare_parameter<double>("mass", parameters.hover.altitude.mass);
    parameters.hover.altitude.altitude_kp = declare_parameter<double>(
      "altitude_kp", parameters.hover.altitude.altitude_kp);
    parameters.hover.altitude.vertical_velocity_kd = declare_parameter<double>(
      "vertical_velocity_kd", parameters.hover.altitude.vertical_velocity_kd);
    parameters.hover.altitude.max_upward_acceleration = declare_parameter<double>(
      "max_upward_acceleration", parameters.hover.altitude.max_upward_acceleration);
    parameters.hover.altitude.max_downward_acceleration = declare_parameter<double>(
      "max_downward_acceleration", parameters.hover.altitude.max_downward_acceleration);
    parameters.hover.altitude.min_collective_thrust = declare_parameter<double>(
      "min_collective_thrust", parameters.hover.altitude.min_collective_thrust);
    parameters.hover.altitude.max_collective_thrust = declare_parameter<double>(
      "max_collective_thrust", parameters.hover.altitude.max_collective_thrust);
    parameters.hover.altitude.min_tilt_cosine = declare_parameter<double>(
      "min_tilt_cosine", parameters.hover.altitude.min_tilt_cosine);

    parameters.hover.attitude.attitude_kp = Eigen::Vector3d(
      declare_parameter<double>(
        "attitude_kp_roll", parameters.hover.attitude.attitude_kp.x()),
      declare_parameter<double>(
        "attitude_kp_pitch", parameters.hover.attitude.attitude_kp.y()),
      declare_parameter<double>(
        "attitude_kp_yaw", parameters.hover.attitude.attitude_kp.z()));
    parameters.hover.attitude.angular_rate_kd = Eigen::Vector3d(
      declare_parameter<double>(
        "angular_rate_kd_roll", parameters.hover.attitude.angular_rate_kd.x()),
      declare_parameter<double>(
        "angular_rate_kd_pitch", parameters.hover.attitude.angular_rate_kd.y()),
      declare_parameter<double>(
        "angular_rate_kd_yaw", parameters.hover.attitude.angular_rate_kd.z()));
    parameters.hover.attitude.max_torque = Eigen::Vector3d(
      declare_parameter<double>(
        "max_torque_roll", parameters.hover.attitude.max_torque.x()),
      declare_parameter<double>(
        "max_torque_pitch", parameters.hover.attitude.max_torque.y()),
      declare_parameter<double>(
        "max_torque_yaw", parameters.hover.attitude.max_torque.z()));

    parameters.hover.mixer.arm_length =
      declare_parameter<double>("arm_length", parameters.hover.mixer.arm_length);
    parameters.hover.mixer.thrust_coefficient = declare_parameter<double>(
      "thrust_coefficient", parameters.hover.mixer.thrust_coefficient);
    parameters.hover.mixer.drag_torque_coefficient = declare_parameter<double>(
      "drag_torque_coefficient", parameters.hover.mixer.drag_torque_coefficient);
    parameters.hover.mixer.min_rpm =
      declare_parameter<double>("min_rpm", parameters.hover.mixer.min_rpm);
    parameters.hover.mixer.max_rpm =
      declare_parameter<double>("max_rpm", parameters.hover.mixer.max_rpm);
    return parameters;
  }

  // 安全处理失败分支的统一出口：发布一条默认构造的 MotorRPM（各字段均为 0）。
  void publish_zero_rpm()
  {
    motor_rpm_publisher_->publish(drone_msgs::msg::MotorRPM{});
  }

  // 每个控制周期（默认 100 Hz）执行一次，是本节点的核心。
  // 整体结构是一条“安全检查链”：任一环不满足就立即 publish_zero_rpm() 并 return，
  // 只有全部通过才会真正调用 PositionController::compute()。
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
    // Reuse the validated goal-quaternion helper, then pass only its yaw to the
    // position controller. Goal roll/pitch remain intentionally ignored.
    Eigen::Quaterniond desired_level_orientation;
    if (!level_orientation_from_goal_yaw(goal_orientation, desired_level_orientation)) {
      publish_zero_rpm();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Invalid goal quaternion; RPM=0");
      return;
    }
    const Eigen::Matrix3d desired_level_rotation =
      desired_level_orientation.toRotationMatrix();
    const double desired_yaw =
      std::atan2(desired_level_rotation(1, 0), desired_level_rotation(0, 0));
    if (!std::isfinite(desired_yaw)) {
      publish_zero_rpm();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Invalid goal yaw; RPM=0");
      return;
    }

    // Convert the complete Odom linear velocity from base_link to map before
    // using any x/y/z component in position feedback.
    const Eigen::Vector3d velocity_body(
      odometry.twist.twist.linear.x, odometry.twist.twist.linear.y,
      odometry.twist.twist.linear.z);
    Eigen::Vector3d velocity_world;
    if (!world_velocity_from_body(current_orientation, velocity_body, velocity_world))
    {
      publish_zero_rpm();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Invalid odometry state; RPM=0");
      return;
    }

    PositionControllerInput input;
    input.desired_position_world = Eigen::Vector3d(
      goal_pose.position.x, goal_pose.position.y, goal_pose.position.z);
    input.desired_yaw = desired_yaw;
    input.current_position_world = Eigen::Vector3d(
      current_pose.position.x, current_pose.position.y, current_pose.position.z);
    input.current_velocity_world = velocity_world;
    input.current_orientation_body_to_world = current_orientation;
    input.current_angular_velocity_body = Eigen::Vector3d(
      odometry.twist.twist.angular.x, odometry.twist.twist.angular.y,
      odometry.twist.twist.angular.z);
    const PositionControllerResult result = position_controller_->compute(input);
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
      "position target=[%.3f, %.3f, %.3f] current=[%.3f, %.3f, %.3f] "
      "world_v=[%.3f, %.3f, %.3f] a_xy=[%.3f, %.3f] thrust=%.3f "
      "rpm=[%.1f, %.1f, %.1f, %.1f] saturated=%s",
      input.desired_position_world.x(), input.desired_position_world.y(),
      input.desired_position_world.z(), input.current_position_world.x(),
      input.current_position_world.y(), input.current_position_world.z(),
      input.current_velocity_world.x(), input.current_velocity_world.y(),
      input.current_velocity_world.z(), result.desired_horizontal_acceleration_world.x(),
      result.desired_horizontal_acceleration_world.y(), result.collective_thrust,
      result.motor_rpm[0], result.motor_rpm[1], result.motor_rpm[2], result.motor_rpm[3],
      result.saturated ? "true" : "false");
  }

  std::unique_ptr<PositionController> position_controller_;
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
