cmake_minimum_required(VERSION 3.8)
project(drone_mission)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_library(waypoint_manager src/waypoint_manager.cpp)
target_include_directories(waypoint_manager
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(waypoint_manager PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(waypoint_manager Eigen3::Eigen)

add_executable(waypoint_manager_node src/waypoint_manager_node.cpp)
target_compile_options(waypoint_manager_node PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(waypoint_manager_node geometry_msgs nav_msgs rclcpp std_msgs)
target_link_libraries(waypoint_manager_node waypoint_manager Eigen3::Eigen)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_waypoint_manager test/test_waypoint_manager.cpp)
  target_compile_options(test_waypoint_manager PRIVATE -Wall -Wextra -Wpedantic)
  target_link_libraries(test_waypoint_manager waypoint_manager Eigen3::Eigen)
endif()

install(DIRECTORY include/ DESTINATION include)
install(
  TARGETS waypoint_manager waypoint_manager_node
  EXPORT export_waypoint_manager
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(export_waypoint_manager HAS_LIBRARY_TARGET)
ament_export_dependencies(eigen3_cmake_module Eigen3)
ament_package()
