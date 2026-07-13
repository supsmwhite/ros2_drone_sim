cmake_minimum_required(VERSION 3.8)
project(drone_controller)

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

add_library(motor_mixer src/motor_mixer.cpp)
target_include_directories(motor_mixer
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(motor_mixer PRIVATE -Wall -Wextra -Wpedantic)

add_library(attitude_controller src/attitude_controller.cpp)
target_include_directories(attitude_controller
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(attitude_controller PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(attitude_controller Eigen3::Eigen)

add_library(altitude_controller src/altitude_controller.cpp)
target_include_directories(altitude_controller
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(altitude_controller PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(altitude_controller Eigen3::Eigen)

add_library(hover_controller src/hover_controller.cpp)
target_include_directories(hover_controller
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(hover_controller PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(
  hover_controller PUBLIC altitude_controller attitude_controller motor_mixer Eigen3::Eigen)

add_executable(position_controller_node src/position_controller_node.cpp)
target_compile_options(position_controller_node PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(position_controller_node
  drone_msgs
  geometry_msgs
  nav_msgs
  rclcpp
)
target_link_libraries(position_controller_node hover_controller Eigen3::Eigen)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS motor_mixer attitude_controller altitude_controller hover_controller
    position_controller_node
  EXPORT export_controller_algorithms
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(export_controller_algorithms HAS_LIBRARY_TARGET)
ament_export_dependencies(eigen3_cmake_module Eigen3 geometry_msgs nav_msgs)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_motor_mixer test/test_motor_mixer.cpp)
  target_link_libraries(test_motor_mixer motor_mixer)
  target_compile_options(test_motor_mixer PRIVATE -Wall -Wextra -Wpedantic)

  ament_add_gtest(test_attitude_controller test/test_attitude_controller.cpp)
  target_link_libraries(test_attitude_controller attitude_controller Eigen3::Eigen)
  target_compile_options(test_attitude_controller PRIVATE -Wall -Wextra -Wpedantic)

  ament_add_gtest(test_altitude_controller test/test_altitude_controller.cpp)
  target_link_libraries(test_altitude_controller altitude_controller Eigen3::Eigen)
  target_compile_options(test_altitude_controller PRIVATE -Wall -Wextra -Wpedantic)

  ament_add_gtest(test_hover_controller test/test_hover_controller.cpp)
  target_link_libraries(test_hover_controller hover_controller Eigen3::Eigen)
  target_compile_options(test_hover_controller PRIVATE -Wall -Wextra -Wpedantic)
endif()

ament_package()
