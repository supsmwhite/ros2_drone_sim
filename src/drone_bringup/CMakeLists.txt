cmake_minimum_required(VERSION 3.8)
project(drone_bringup)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)

  add_launch_test(
    test/test_single_goal_e2e.py
    TIMEOUT 70
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
endif()

install(
  DIRECTORY launch config rviz urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
