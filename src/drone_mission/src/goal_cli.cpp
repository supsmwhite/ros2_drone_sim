#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "drone_mission/goal_input.hpp"
#include "drone_msgs/srv/execute_goal_sequence.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{

constexpr auto kDiscoveryTimeout = std::chrono::seconds(10);

void print_usage(const char * program)
{
  std::cout << "Usage:\n  " << program << " single x y z yaw_rad|yaw=degrees\n  " << program
            << " multi x1 y1 z1 yaw_rad|yaw=degrees [x2 y2 z2 yaw2 ...]\n"
            << "Examples: yaw=30, yaw=60, yaw=90. Plain numeric yaw remains radians.\n"
            << "All goals must be finite and inside the configured workspace.\n";
}

drone_mission::GoalConstraints constraints_from_node(rclcpp::Node & node)
{
  const auto workspace = node.declare_parameter<std::vector<double>>(
    "workspace", {-1.0, 14.5, -2.5, 7.0, -0.5, 5.0});
  const double minimum_altitude = node.declare_parameter<double>(
    "minimum_navigation_altitude", 0.0);
  if (workspace.size() != 6U) {
    throw std::invalid_argument("workspace must be [xmin,xmax,ymin,ymax,zmin,zmax]");
  }
  drone_mission::GoalConstraints constraints{
    workspace[0], workspace[1], workspace[2], workspace[3],
    std::max(workspace[4], minimum_altitude), workspace[5]};
  drone_mission::validate_constraints(constraints);
  return constraints;
}

int run_sequence(
  const std::shared_ptr<rclcpp::Node> & node,
  const std::vector<geometry_msgs::msg::Pose> & poses)
{
  auto client = node->create_client<drone_msgs::srv::ExecuteGoalSequence>(
    "/drone/mission/execute");
  if (!client->wait_for_service(kDiscoveryTimeout)) {
    std::cerr << "Error: /drone/mission/execute is unavailable after 10 seconds.\n";
    return 3;
  }
  auto request = std::make_shared<drone_msgs::srv::ExecuteGoalSequence::Request>();
  request->goals.header.stamp = node->now();
  request->goals.header.frame_id = "map";
  request->goals.poses = poses;
  request->draft_revision = 0U;
  auto future = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, future, kDiscoveryTimeout) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    std::cerr << "Error: mission service call timed out.\n";
    return 4;
  }
  const auto response = future.get();
  std::cout << (response->accepted ? "Mission accepted: " : "Mission rejected: ")
            << response->message << '\n';
  return response->accepted ? 0 : 5;
}

}  // namespace

int main(int argc, char * argv[])
{
  if (argc == 2 && (std::string(argv[1]) == "--help" || std::string(argv[1]) == "-h")) {
    print_usage(argv[0]);
    return 0;
  }
  rclcpp::init(argc, argv);
  const auto arguments_without_ros = rclcpp::remove_ros_arguments(argc, argv);
  if (arguments_without_ros.size() < 2U) {
    print_usage(argv[0]);
    rclcpp::shutdown();
    return 2;
  }
  const std::string mode(arguments_without_ros[1]);
  const std::size_t value_count = arguments_without_ros.size() - 2U;
  if ((mode == "single" && value_count != 4) ||
    (mode == "multi" && (value_count == 0 || value_count % 4 != 0)) ||
    (mode != "single" && mode != "multi"))
  {
    std::cerr << "Error: invalid mode or argument count.\n";
    print_usage(argv[0]);
    rclcpp::shutdown();
    return 2;
  }

  int result = 1;
  try {
    auto node = std::make_shared<rclcpp::Node>("goal_cli");
    const auto constraints = constraints_from_node(*node);
    std::vector<std::string> arguments;
    for (std::size_t index = 2U; index < arguments_without_ros.size(); ++index) {
      arguments.push_back(arguments_without_ros[index]);
    }
    const auto values = drone_mission::parse_goal_arguments(arguments);
    const auto poses = drone_mission::make_goal_poses(values, constraints);
    result = run_sequence(node, poses);
  } catch (const std::exception & error) {
    std::cerr << "Error: " << error.what() << '\n';
    result = 2;
  }
  rclcpp::shutdown();
  return result;
}
