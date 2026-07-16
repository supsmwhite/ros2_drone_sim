cmake_minimum_required(VERSION 3.8)
project(drone_planning)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(drone_mission REQUIRED)
find_package(drone_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_library(static_environment src/static_environment.cpp)
target_include_directories(static_environment
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(static_environment PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(static_environment Eigen3::Eigen)

add_library(collision_checker src/collision_checker.cpp)
target_include_directories(collision_checker
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(collision_checker PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(collision_checker static_environment Eigen3::Eigen)

add_library(astar_planner src/astar_planner.cpp)
target_include_directories(astar_planner
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(astar_planner PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(astar_planner collision_checker Eigen3::Eigen)

add_library(path_simplifier src/path_simplifier.cpp)
target_include_directories(path_simplifier
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(path_simplifier PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(path_simplifier collision_checker Eigen3::Eigen)

add_library(planned_trajectory_builder src/planned_trajectory_builder.cpp)
target_include_directories(planned_trajectory_builder
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(planned_trajectory_builder PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(
  planned_trajectory_builder path_simplifier collision_checker
    drone_mission::piecewise_quintic_trajectory Eigen3::Eigen)

add_executable(static_environment_node src/static_environment_node.cpp)
target_compile_options(static_environment_node PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(
  static_environment_node nav_msgs rclcpp std_msgs visualization_msgs)
target_link_libraries(static_environment_node collision_checker Eigen3::Eigen)

add_executable(astar_planner_node src/astar_planner_node.cpp)
target_compile_options(astar_planner_node PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(
  astar_planner_node geometry_msgs nav_msgs rclcpp std_msgs)
target_link_libraries(astar_planner_node astar_planner Eigen3::Eigen)

add_executable(planned_trajectory_node src/planned_trajectory_node.cpp)
target_compile_options(planned_trajectory_node PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(
  planned_trajectory_node drone_msgs geometry_msgs nav_msgs rclcpp std_msgs)
target_link_libraries(
  planned_trajectory_node planned_trajectory_builder Eigen3::Eigen)

add_executable(
  multi_goal_static_avoidance_node src/multi_goal_static_avoidance_node.cpp)
target_compile_options(
  multi_goal_static_avoidance_node PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(
  multi_goal_static_avoidance_node
  drone_msgs geometry_msgs nav_msgs rclcpp std_msgs)
target_link_libraries(
  multi_goal_static_avoidance_node
  planned_trajectory_builder astar_planner Eigen3::Eigen)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_collision_checker test/test_collision_checker.cpp)
  target_compile_options(test_collision_checker PRIVATE -Wall -Wextra -Wpedantic)
  target_link_libraries(test_collision_checker collision_checker Eigen3::Eigen)

  ament_add_gtest(test_astar_planner test/test_astar_planner.cpp)
  target_compile_options(test_astar_planner PRIVATE -Wall -Wextra -Wpedantic)
  target_link_libraries(test_astar_planner astar_planner Eigen3::Eigen)

  ament_add_gtest(test_path_simplifier test/test_path_simplifier.cpp)
  target_compile_options(test_path_simplifier PRIVATE -Wall -Wextra -Wpedantic)
  target_link_libraries(test_path_simplifier path_simplifier Eigen3::Eigen)

  ament_add_gtest(
    test_planned_trajectory_builder test/test_planned_trajectory_builder.cpp)
  target_compile_options(
    test_planned_trajectory_builder PRIVATE -Wall -Wextra -Wpedantic)
  target_link_libraries(
    test_planned_trajectory_builder planned_trajectory_builder astar_planner Eigen3::Eigen)
endif()

install(DIRECTORY include/ DESTINATION include)
install(
  TARGETS static_environment collision_checker astar_planner path_simplifier
    planned_trajectory_builder
    static_environment_node astar_planner_node planned_trajectory_node
    multi_goal_static_avoidance_node
  EXPORT export_drone_planning
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(export_drone_planning HAS_LIBRARY_TARGET)
ament_export_dependencies(drone_mission drone_msgs eigen3_cmake_module Eigen3)
ament_package()
