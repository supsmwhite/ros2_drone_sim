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

class QuadrotorDynamicsNode : public rclcpp::Node
{
public:
  QuadrotorDynamicsNode()
  : Node("quadrotor_dynamics_node")
  {
    const QuadrotorParameters parameters = declare_model_parameters();
    const double simulation_frequency =
      declare_parameter<double>("simulation_frequency", 200.0);
    path_publish_divider_ = declare_parameter<int>("path_publish_divider", 10);
    path_max_points_ = declare_parameter<int>("path_max_points", 2000);

    if (!std::isfinite(simulation_frequency) || simulation_frequency <= 0.0) {
      throw std::invalid_argument("simulation_frequency must be finite and greater than zero");
    }
    if (path_publish_divider_ <= 0) {
      throw std::invalid_argument("path_publish_divider must be greater than zero");
    }
    if (path_max_points_ <= 0) {
      throw std::invalid_argument("path_max_points must be greater than zero");
    }

    fixed_time_step_ = 1.0 / simulation_frequency;
    model_ = std::make_unique<QuadrotorModel>(parameters);

    motor_rpm_subscription_ = create_subscription<drone_msgs::msg::MotorRPM>(
      "/drone/motor_rpm_cmd", 10,
      [this](const drone_msgs::msg::MotorRPM::SharedPtr message) {
        model_->set_motor_rpm_command({
          message->m1_front_left_ccw_rpm,
          message->m2_rear_left_cw_rpm,
          message->m3_rear_right_ccw_rpm,
          message->m4_front_right_cw_rpm});
      });

    odometry_publisher_ = create_publisher<nav_msgs::msg::Odometry>("/drone/odom", 10);
    imu_publisher_ = create_publisher<sensor_msgs::msg::Imu>("/drone/imu", 10);
    path_publisher_ = create_publisher<nav_msgs::msg::Path>("/drone/path", 10);
    transform_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    const auto timer_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(fixed_time_step_));
    simulation_timer_ = create_wall_timer(timer_period, [this]() {simulation_step();});

    const double hover_angular_velocity = std::sqrt(
      parameters.mass * parameters.gravity / (4.0 * parameters.thrust_coefficient));
    const double hover_rpm = hover_angular_velocity * 30.0 / 3.14159265358979323846;
    RCLCPP_INFO(
      get_logger(),
      "Dynamics started at %.1f Hz; nominal steady-state hover RPM is %.1f",
      simulation_frequency, hover_rpm);
  }

private:
  QuadrotorParameters declare_model_parameters()
  {
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
    return parameters;
  }

  void simulation_step()
  {
    model_->step(fixed_time_step_);
    const rclcpp::Time stamp = now();
    publish_odometry(stamp);
    publish_imu(stamp);
    publish_transform(stamp);

    ++simulation_step_count_;
    if (simulation_step_count_ % static_cast<std::size_t>(path_publish_divider_) == 0U) {
      publish_path(stamp);
    }
  }

  void publish_odometry(const rclcpp::Time & stamp)
  {
    const QuadrotorState & state = model_->state();
    nav_msgs::msg::Odometry message;
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

  void publish_imu(const rclcpp::Time & stamp)
  {
    const QuadrotorState & state = model_->state();
    const Eigen::Vector3d specific_force = model_->specific_force_body();
    sensor_msgs::msg::Imu message;
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

  void publish_transform(const rclcpp::Time & stamp)
  {
    const QuadrotorState & state = model_->state();
    geometry_msgs::msg::TransformStamped transform;
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

  void publish_path(const rclcpp::Time & stamp)
  {
    const QuadrotorState & state = model_->state();
    geometry_msgs::msg::PoseStamped pose;
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
    if (path_.poses.size() > static_cast<std::size_t>(path_max_points_)) {
      path_.poses.erase(path_.poses.begin());
    }
    path_publisher_->publish(path_);
  }

  std::unique_ptr<QuadrotorModel> model_;
  double fixed_time_step_{0.005};
  int path_publish_divider_{10};
  int path_max_points_{2000};
  std::size_t simulation_step_count_{0};
  nav_msgs::msg::Path path_;

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
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<drone_dynamics::QuadrotorDynamicsNode>());
  rclcpp::shutdown();
  return 0;
}
