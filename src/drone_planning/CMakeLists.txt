cmake_minimum_required(VERSION 3.8)
project(drone_planning)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
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

add_executable(static_environment_node src/static_environment_node.cpp)
target_compile_options(static_environment_node PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(
  static_environment_node nav_msgs rclcpp std_msgs visualization_msgs)
target_link_libraries(static_environment_node collision_checker Eigen3::Eigen)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_collision_checker test/test_collision_checker.cpp)
  target_compile_options(test_collision_checker PRIVATE -Wall -Wextra -Wpedantic)
  target_link_libraries(test_collision_checker collision_checker Eigen3::Eigen)
endif()

install(DIRECTORY include/ DESTINATION include)
install(
  TARGETS static_environment collision_checker static_environment_node
  EXPORT export_drone_planning
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(export_drone_planning HAS_LIBRARY_TARGET)
ament_export_dependencies(eigen3_cmake_module Eigen3)
ament_package()
