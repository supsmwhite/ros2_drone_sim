cmake_minimum_required(VERSION 3.8)
project(drone_mission)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(drone_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_library(goal_input src/goal_input.cpp)
target_include_directories(goal_input PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_options(goal_input PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(goal_input geometry_msgs)

add_library(goal_visualization src/goal_visualization.cpp)
target_include_directories(goal_visualization PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_options(goal_visualization PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(goal_visualization geometry_msgs visualization_msgs)

add_library(waypoint_manager src/waypoint_manager.cpp)
target_include_directories(waypoint_manager
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(waypoint_manager PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(waypoint_manager Eigen3::Eigen)

add_library(piecewise_quintic_trajectory src/piecewise_quintic_trajectory.cpp)
target_include_directories(piecewise_quintic_trajectory
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(piecewise_quintic_trajectory PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(piecewise_quintic_trajectory Eigen3::Eigen)

add_executable(waypoint_manager_node src/waypoint_manager_node.cpp)
target_compile_options(waypoint_manager_node PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(
  waypoint_manager_node drone_msgs geometry_msgs nav_msgs rclcpp std_msgs)
target_link_libraries(waypoint_manager_node waypoint_manager Eigen3::Eigen)

add_executable(goal_cli src/goal_cli.cpp)
target_compile_options(goal_cli PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(goal_cli drone_msgs geometry_msgs rclcpp)
target_link_libraries(goal_cli goal_input)

add_executable(goal_visualizer_node src/goal_visualizer_node.cpp)
target_compile_options(goal_visualizer_node PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(
  goal_visualizer_node geometry_msgs rclcpp std_msgs visualization_msgs)
target_link_libraries(goal_visualizer_node goal_visualization)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_waypoint_manager test/test_waypoint_manager.cpp)
  target_compile_options(test_waypoint_manager PRIVATE -Wall -Wextra -Wpedantic)
  target_link_libraries(test_waypoint_manager waypoint_manager Eigen3::Eigen)

  ament_add_gtest(test_goal_input test/test_goal_input.cpp)
  target_compile_options(test_goal_input PRIVATE -Wall -Wextra -Wpedantic)
  target_link_libraries(test_goal_input goal_input)

  ament_add_gtest(test_goal_visualization test/test_goal_visualization.cpp)
  target_compile_options(test_goal_visualization PRIVATE -Wall -Wextra -Wpedantic)
  target_link_libraries(test_goal_visualization goal_visualization)

  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_waypoint_service.py TIMEOUT 30)
  add_launch_test(test/test_goal_visualizer_node.py TIMEOUT 30)

  ament_add_gtest(
    test_piecewise_quintic_trajectory test/test_piecewise_quintic_trajectory.cpp)
  target_compile_options(
    test_piecewise_quintic_trajectory PRIVATE -Wall -Wextra -Wpedantic)
  target_link_libraries(
    test_piecewise_quintic_trajectory piecewise_quintic_trajectory Eigen3::Eigen)
endif()

install(DIRECTORY include/ DESTINATION include)
install(
  TARGETS goal_input goal_visualization waypoint_manager piecewise_quintic_trajectory
    waypoint_manager_node goal_cli goal_visualizer_node
  EXPORT export_waypoint_manager
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(export_waypoint_manager HAS_LIBRARY_TARGET)
ament_export_dependencies(eigen3_cmake_module Eigen3 geometry_msgs nav_msgs visualization_msgs)
ament_package()
