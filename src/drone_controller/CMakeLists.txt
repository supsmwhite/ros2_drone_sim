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
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_library(controller_algorithms src/position_controller.cpp)
target_include_directories(controller_algorithms
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(controller_algorithms PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(controller_algorithms geometry_msgs nav_msgs)

add_library(motor_mixer src/motor_mixer.cpp)
target_include_directories(motor_mixer
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(motor_mixer PRIVATE -Wall -Wextra -Wpedantic)

add_executable(position_controller_node src/position_controller_node.cpp)
target_compile_options(position_controller_node PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(position_controller_node
  drone_msgs
  geometry_msgs
  nav_msgs
  rclcpp
)
target_link_libraries(position_controller_node controller_algorithms)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS controller_algorithms motor_mixer position_controller_node
  EXPORT export_controller_algorithms
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(export_controller_algorithms HAS_LIBRARY_TARGET)
ament_export_dependencies(geometry_msgs nav_msgs)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_motor_mixer test/test_motor_mixer.cpp)
  target_link_libraries(test_motor_mixer motor_mixer)
  target_compile_options(test_motor_mixer PRIVATE -Wall -Wextra -Wpedantic)
endif()

ament_package()
