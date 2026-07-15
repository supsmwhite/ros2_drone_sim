cmake_minimum_required(VERSION 3.8)
project(drone_dynamics)

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
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(quadrotor_model src/quadrotor_model.cpp)
target_include_directories(quadrotor_model
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(quadrotor_model PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(quadrotor_model Eigen3::Eigen)

add_executable(quadrotor_dynamics_node src/quadrotor_dynamics_node.cpp)
target_compile_options(quadrotor_dynamics_node PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(quadrotor_dynamics_node
  drone_msgs
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  tf2
  tf2_ros
)
target_link_libraries(quadrotor_dynamics_node quadrotor_model Eigen3::Eigen)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(drone_controller REQUIRED)

  ament_add_gtest(test_quadrotor_model test/test_quadrotor_model.cpp)
  target_compile_options(test_quadrotor_model PRIVATE -Wall -Wextra -Wpedantic)
  target_link_libraries(test_quadrotor_model quadrotor_model Eigen3::Eigen)

  ament_add_gtest(test_attitude_closed_loop test/test_attitude_closed_loop.cpp)
  target_compile_options(test_attitude_closed_loop PRIVATE -Wall -Wextra -Wpedantic)
  target_link_libraries(
    test_attitude_closed_loop
    quadrotor_model
    drone_controller::attitude_controller
    drone_controller::motor_mixer
    Eigen3::Eigen)
endif()

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS quadrotor_model quadrotor_dynamics_node
  EXPORT export_quadrotor_model
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(export_quadrotor_model HAS_LIBRARY_TARGET)
ament_export_dependencies(eigen3_cmake_module Eigen3)

ament_package()
