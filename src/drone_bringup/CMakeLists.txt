cmake_minimum_required(VERSION 3.8)
project(drone_bringup)

find_package(ament_cmake REQUIRED)
find_package(drone_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(disturbance_demo_node src/disturbance_demo_node.cpp)
ament_target_dependencies(
  disturbance_demo_node
  drone_msgs
  geometry_msgs
  nav_msgs
  rclcpp
  visualization_msgs
)

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(
    test_physical_parameter_consistency
    test/test_physical_parameter_consistency.py
  )

  ament_add_pytest_test(
    test_launch_structure
    test/test_launch_structure.py
  )

  ament_add_pytest_test(
    test_assessment_launch_structure
    test/test_assessment_launch_structure.py
  )

  add_launch_test(
    test/test_single_goal_e2e.py
    TIMEOUT 70
  )

  add_launch_test(
    test/test_assessment_basic_single_e2e.py
    TIMEOUT 90
  )

  add_launch_test(
    test/test_assessment_basic_multi_e2e.py
    TIMEOUT 110
  )

  add_launch_test(
    test/test_waypoint_mission_e2e.py
    TIMEOUT 120
  )

  add_launch_test(
    test/test_trajectory_mission_e2e.py
    TIMEOUT 100
  )

  add_launch_test(
    test/test_static_environment_e2e.py
    TIMEOUT 40
  )

  add_launch_test(
    test/test_astar_planner_e2e.py
    TIMEOUT 40
  )

  add_launch_test(
    test/test_planned_trajectory_e2e.py
    TIMEOUT 40
  )

  add_launch_test(
    test/test_static_avoidance_e2e.py
    TIMEOUT 80
  )

  add_launch_test(
    test/test_multi_goal_static_avoidance_e2e.py
    TIMEOUT 200
  )

  add_launch_test(
    test/test_interactive_goal_editor_e2e.py
    TIMEOUT 120
  )

  add_launch_test(
    test/test_interactive_mission_service.py
    TIMEOUT 40
  )

  add_launch_test(
    test/test_interactive_goal_navigation_e2e.py
    TIMEOUT 130
  )

  add_launch_test(
    test/test_interactive_preflight_failure.py
    TIMEOUT 40
  )

  add_launch_test(
    test/test_external_wrench.py
    TIMEOUT 40
  )

  add_launch_test(
    test/test_horizontal_integral_node.py
    TIMEOUT 40
  )

  add_launch_test(
    test/test_disturbance_demo_node.py
    TIMEOUT 30
  )

  add_launch_test(
    test/test_disturbance_visual_launch.py
    TIMEOUT 30
  )

  add_launch_test(
    test/test_assessment_disturbance_launch.py
    TIMEOUT 40
  )
endif()

install(
  TARGETS disturbance_demo_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config rviz urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
