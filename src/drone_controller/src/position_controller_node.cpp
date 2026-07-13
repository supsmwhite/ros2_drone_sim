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

class PositionControllerNode : public rclcpp::Node
{
public:
  PositionControllerNode()
  : Node("position_controller_node")
  {
    const double control_frequency = declare_parameter<double>("control_frequency", 100.0);
    odometry_timeout_ = declare_parameter<double>("odometry_timeout", 0.2);
    supported_goal_frame_ = declare_parameter<std::string>("supported_goal_frame", "map");
    if (!std::isfinite(control_frequency) || control_frequency <= 0.0 ||
      !std::isfinite(odometry_timeout_) || odometry_timeout_ <= 0.0 ||
      supported_goal_frame_.empty())
    {
      throw std::invalid_argument("Invalid controller loop parameters");
    }

    hover_controller_ = std::make_unique<HoverController>(read_hover_parameters());
    goal_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/drone/goal", 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr message) {
        latest_goal_ = *message;
      });
    odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      "/drone/odom", 10,
      [this](const nav_msgs::msg::Odometry::SharedPtr message) {
        latest_odometry_ = *message;
        latest_odometry_reception_time_ = now();
      });
    motor_rpm_publisher_ =
      create_publisher<drone_msgs::msg::MotorRPM>("/drone/motor_rpm_cmd", 10);

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

  void publish_zero_rpm()
  {
    motor_rpm_publisher_->publish(drone_msgs::msg::MotorRPM{});
  }

  void control_step()
  {
    if (!latest_goal_) {
      publish_zero_rpm();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting for /drone/goal; RPM=0");
      return;
    }
    if (!latest_odometry_ || !latest_odometry_reception_time_) {
      publish_zero_rpm();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting for /drone/odom; RPM=0");
      return;
    }
    const double odometry_age = (now() - *latest_odometry_reception_time_).seconds();
    if (!std::isfinite(odometry_age) || odometry_age > odometry_timeout_) {
      publish_zero_rpm();
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000, "Odometry stale (%.3f s); RPM=0", odometry_age);
      return;
    }

    const std::string & frame = latest_goal_->header.frame_id;
    if (!frame.empty() && frame != supported_goal_frame_) {
      publish_zero_rpm();
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000, "Unsupported goal frame '%s'; RPM=0", frame.c_str());
      return;
    }

    const auto & goal_pose = latest_goal_->pose;
    const auto & odometry = *latest_odometry_;
    const auto & current_pose = odometry.pose.pose;
    const Eigen::Quaterniond goal_orientation(
      goal_pose.orientation.w, goal_pose.orientation.x,
      goal_pose.orientation.y, goal_pose.orientation.z);
    const Eigen::Quaterniond current_orientation(
      current_pose.orientation.w, current_pose.orientation.x,
      current_pose.orientation.y, current_pose.orientation.z);
    Eigen::Quaterniond desired_level_orientation;
    if (!level_orientation_from_goal_yaw(goal_orientation, desired_level_orientation)) {
      publish_zero_rpm();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Invalid goal quaternion; RPM=0");
      return;
    }

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

    HoverControllerInput input;
    input.desired_altitude = goal_pose.position.z;
    input.desired_orientation_body_to_world = desired_level_orientation;
    input.current_altitude = current_pose.position.z;
    input.current_vertical_velocity_world = vertical_velocity_world;
    input.current_orientation_body_to_world = current_orientation;
    input.current_angular_velocity_body = Eigen::Vector3d(
      odometry.twist.twist.angular.x, odometry.twist.twist.angular.y,
      odometry.twist.twist.angular.z);
    const HoverControllerResult result = hover_controller_->compute(input);
    if (!result.valid) {
      publish_zero_rpm();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Invalid control result; RPM=0");
      return;
    }

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
