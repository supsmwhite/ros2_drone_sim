#include <memory>

#include "drone_controller/position/position_controller.hpp"
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
    goal_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/drone/goal", 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr message) {
        controller_.set_goal(*message);
      });

    odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      "/drone/odom", 10,
      [this](const nav_msgs::msg::Odometry::SharedPtr message) {
        controller_.set_odometry(*message);
      });

    motor_rpm_publisher_ =
      create_publisher<drone_msgs::msg::MotorRPM>("/drone/motor_rpm_cmd", 10);

    RCLCPP_INFO(
      get_logger(),
      "Controller node skeleton started; control laws and motor command publication are not implemented.");
  }

private:
  PositionController controller_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  rclcpp::Publisher<drone_msgs::msg::MotorRPM>::SharedPtr motor_rpm_publisher_;
};

}  // namespace drone_controller

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<drone_controller::PositionControllerNode>());
  rclcpp::shutdown();
  return 0;
}
