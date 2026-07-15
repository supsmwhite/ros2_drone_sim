cmake_minimum_required(VERSION 3.8)
project(drone_bringup)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)

  add_launch_test(
    test/test_single_goal_e2e.py
    TIMEOUT 70
  )
endif()

install(
  DIRECTORY launch config rviz urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
