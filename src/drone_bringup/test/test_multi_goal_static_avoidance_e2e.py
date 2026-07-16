#!/usr/bin/env python3

import math
import os
import time
import unittest

os.environ['ROS_DOMAIN_ID'] = '113'

from ament_index_python.packages import get_package_share_directory
from drone_msgs.msg import MotorRPM, TrajectorySetpoint
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
import launch_testing.actions
import launch_testing.markers
from nav_msgs.msg import Odometry, Path
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, UInt32


TARGETS = (
    (3.6, 3.1, 1.5),
    (7.85, 3.1, 2.5),
    (9.85, 0.6, 3.25),
    (12.1, 0.85, 1.5),
)
TAKEOFF_ANCHOR = (0.0, 0.0, 1.5)
NAVIGATION_FLOOR = 0.50
DISCOVERY_TIMEOUT = 8.0
MISSION_TIMEOUT = 105.0
POST_COMPLETE_OBSERVATION = 3.0
BASE_INFLATED_OBSTACLES = (
    ((1.95, -2.75, -0.25), (3.25, 1.75, 4.95)),
    ((3.95, 1.55, -0.25), (5.25, 6.75, 4.95)),
    ((6.05, -1.05, -0.25), (7.35, 2.65, 4.95)),
    ((8.25, 0.75, -0.25), (9.55, 6.75, 4.95)),
    ((10.05, -1.75, -0.25), (11.35, 0.05, 4.95)),
    ((10.05, 1.45, -0.25), (11.35, 4.45, 4.95)),
)


def norm3(values):
    return math.sqrt(sum(value * value for value in values))


def distance_to_box(point, lower, upper):
    return norm3(tuple(
        max(lower[index] - point[index], 0.0, point[index] - upper[index])
        for index in range(3)
    ))


def inside_closed_box(point, lower, upper):
    return all(lower[index] <= point[index] <= upper[index] for index in range(3))


def segment_intersects_closed_box(start, end, lower, upper):
    interval_min = 0.0
    interval_max = 1.0
    for index in range(3):
        delta = end[index] - start[index]
        if delta == 0.0:
            if start[index] < lower[index] or start[index] > upper[index]:
                return False
            continue
        axis_min = (lower[index] - start[index]) / delta
        axis_max = (upper[index] - start[index]) / delta
        if axis_min > axis_max:
            axis_min, axis_max = axis_max, axis_min
        interval_min = max(interval_min, axis_min)
        interval_max = min(interval_max, axis_max)
        if interval_min > interval_max:
            return False
    return True


def path_points(message):
    return [
        (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z)
        for pose in message.poses
    ]


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    bringup_share = get_package_share_directory('drone_bringup')
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            bringup_share, 'launch', 'multi_goal_static_avoidance_sim.launch.py')),
        launch_arguments={
            'use_rviz': 'false',
            'mission_config': os.path.join(
                bringup_share, 'config', 'multi_goal_mission_test.yaml'),
        }.items(),
    )
    return launch.LaunchDescription([
        simulation,
        launch_testing.actions.ReadyToTest(),
    ])


class TestMultiGoalStaticAvoidanceEndToEnd(unittest.TestCase):

    def test_ordered_goals_execute_safely(self, proc_output):
        rclpy.init()
        node = rclpy.create_node('multi_goal_static_avoidance_e2e_test')
        test_start = time.monotonic()
        latest_position = None
        previous_odom_position = None
        latest_speed = math.inf
        latest_setpoint = None
        current_goal_index = None
        latest_visited = None
        mission_complete_time = None
        first_path_time = None
        last_odom_time = None
        last_setpoint_time = None
        last_rpm_time = None
        observed_goal_indices = []
        observed_visited_counts = []
        planned_paths = []
        simplified_paths = []
        reference_paths = []
        planning_start_errors = []
        goal_acceptance_errors = []
        goal_acceptance_speeds = []
        maximum_tracking_errors = [0.0] * len(TARGETS)
        minimum_sampled_clearance = math.inf
        odom_samples = 0
        setpoint_samples = 0
        rpm_samples = 0
        collision_samples = 0
        odom_after_complete = 0
        setpoints_after_complete = 0
        rpm_after_complete = 0
        takeoff_anchor_setpoints = 0
        success_seen = False
        health_errors = []

        def check_path(message, name):
            if message.header.frame_id != 'map':
                health_errors.append(f'{name} frame is not map')
                return []
            points = path_points(message)
            if len(points) < 2:
                health_errors.append(f'{name} contains fewer than two points')
                return points
            if not all(math.isfinite(value) for point in points for value in point):
                health_errors.append(f'{name} contains non-finite points')
            if any(point[2] < NAVIGATION_FLOOR - 1.0e-9 for point in points):
                health_errors.append(f'{name} went below the 0.50 m navigation floor')
            return points

        def on_planned_path(message):
            nonlocal first_path_time
            points = check_path(message, 'planned path')
            if not points:
                return
            path_index = len(planned_paths)
            planned_paths.append(points)
            if first_path_time is None:
                first_path_time = time.monotonic()
            if path_index >= len(TARGETS):
                health_errors.append('more planned paths than ordered goals')
                return
            if latest_position is None:
                health_errors.append('planned path arrived without actual Odom')
                return
            planning_start_errors.append(norm3(tuple(
                points[0][axis] - latest_position[axis] for axis in range(3))))
            if norm3(tuple(
                    points[-1][axis] - TARGETS[path_index][axis]
                    for axis in range(3))) > 1.0e-9:
                health_errors.append(
                    f'planned path {path_index} did not end at its ordered goal')
            if path_index == 0:
                takeoff_error = norm3(tuple(
                    latest_position[axis] - TAKEOFF_ANCHOR[axis]
                    for axis in range(3)))
                if takeoff_error >= 0.20 or latest_speed >= 0.15:
                    health_errors.append(
                        'first A* segment started before takeoff was stable: '
                        f'error={takeoff_error}, speed={latest_speed}')

        def on_simplified_path(message):
            points = check_path(message, 'simplified path')
            if points:
                simplified_paths.append(points)

        def on_reference_path(message):
            points = check_path(message, 'reference path')
            if points:
                reference_paths.append(points)

        def on_goal_index(message):
            nonlocal current_goal_index
            value = int(message.data)
            current_goal_index = value
            if not observed_goal_indices or observed_goal_indices[-1] != value:
                observed_goal_indices.append(value)
                if observed_goal_indices != list(range(len(observed_goal_indices))):
                    health_errors.append(
                        f'goal index skipped or regressed: {observed_goal_indices}')

        def on_visited(message):
            nonlocal latest_visited
            value = int(message.data)
            latest_visited = value
            if not observed_visited_counts or observed_visited_counts[-1] != value:
                observed_visited_counts.append(value)
                expected = list(range(value + 1))
                if observed_visited_counts != expected:
                    health_errors.append(
                        f'visited goal count skipped or regressed: {observed_visited_counts}')
                if value > 0 and latest_position is not None:
                    target = TARGETS[value - 1]
                    goal_acceptance_errors.append(norm3(tuple(
                        latest_position[axis] - target[axis] for axis in range(3))))
                    goal_acceptance_speeds.append(latest_speed)

        def on_complete(message):
            nonlocal mission_complete_time
            if message.data and mission_complete_time is None:
                mission_complete_time = time.monotonic()
            elif not message.data and mission_complete_time is not None:
                health_errors.append('mission complete state regressed to false')

        def on_success(message):
            nonlocal success_seen
            success_seen = True
            if not message.data:
                health_errors.append('multi-goal mission success became false')

        def on_collision(message):
            nonlocal collision_samples
            collision_samples += 1
            if message.data:
                health_errors.append('/drone/environment/in_collision became true')

        def on_setpoint(message):
            nonlocal latest_setpoint, last_setpoint_time, setpoint_samples
            nonlocal setpoints_after_complete, takeoff_anchor_setpoints
            values = (
                message.position.x, message.position.y, message.position.z,
                message.velocity.x, message.velocity.y, message.velocity.z,
                message.acceleration.x, message.acceleration.y,
                message.acceleration.z, message.yaw,
            )
            if message.header.frame_id != 'map':
                health_errors.append('trajectory setpoint frame is not map')
                return
            if not all(math.isfinite(value) for value in values):
                health_errors.append('non-finite trajectory setpoint')
                return
            if first_path_time is None:
                takeoff_position = values[0:3]
                takeoff_setpoint_error = norm3(tuple(
                    takeoff_position[axis] - TAKEOFF_ANCHOR[axis]
                    for axis in range(3)))
                if takeoff_setpoint_error < 1.0e-9:
                    takeoff_anchor_setpoints += 1
                elif takeoff_setpoint_error >= 0.05:
                    health_errors.append(
                        'pre-planning setpoint left the stable takeoff neighborhood')
                if norm3(values[3:6]) > 1.0e-9 or norm3(values[6:9]) > 1.0e-9:
                    health_errors.append('takeoff setpoint velocity or acceleration was non-zero')
            latest_setpoint = message
            last_setpoint_time = time.monotonic()
            setpoint_samples += 1
            if mission_complete_time is not None:
                setpoints_after_complete += 1

        def on_odometry(message):
            nonlocal latest_position, previous_odom_position, latest_speed
            nonlocal last_odom_time, odom_samples, odom_after_complete
            nonlocal minimum_sampled_clearance
            pose = message.pose.pose
            twist = message.twist.twist
            values = (
                pose.position.x, pose.position.y, pose.position.z,
                pose.orientation.x, pose.orientation.y,
                pose.orientation.z, pose.orientation.w,
                twist.linear.x, twist.linear.y, twist.linear.z,
                twist.angular.x, twist.angular.y, twist.angular.z,
            )
            if not all(math.isfinite(value) for value in values):
                previous_odom_position = None
                health_errors.append('non-finite odometry')
                return
            position = values[0:3]
            latest_position = position
            latest_speed = norm3(values[7:10])
            last_odom_time = time.monotonic()
            odom_samples += 1
            if mission_complete_time is not None:
                odom_after_complete += 1
            for lower, upper in BASE_INFLATED_OBSTACLES:
                minimum_sampled_clearance = min(
                    minimum_sampled_clearance,
                    distance_to_box(position, lower, upper))
                if inside_closed_box(position, lower, upper):
                    health_errors.append(
                        f'actual Odom entered a base-inflated obstacle at {position}')
                if (previous_odom_position is not None and
                        segment_intersects_closed_box(
                            previous_odom_position, position, lower, upper)):
                    health_errors.append(
                        'actual Odom segment intersected a base-inflated obstacle: '
                        f'{previous_odom_position} -> {position}')
            if (current_goal_index is not None and current_goal_index < len(TARGETS) and
                    len(planned_paths) > current_goal_index and
                    latest_setpoint is not None and mission_complete_time is None):
                reference = (
                    latest_setpoint.position.x,
                    latest_setpoint.position.y,
                    latest_setpoint.position.z,
                )
                maximum_tracking_errors[current_goal_index] = max(
                    maximum_tracking_errors[current_goal_index],
                    norm3(tuple(
                        position[axis] - reference[axis] for axis in range(3))))
            previous_odom_position = position

        def on_motor_rpm(message):
            nonlocal last_rpm_time, rpm_samples, rpm_after_complete
            values = (
                message.m1_front_left_ccw_rpm,
                message.m2_rear_left_cw_rpm,
                message.m3_rear_right_ccw_rpm,
                message.m4_front_right_cw_rpm,
            )
            if not all(math.isfinite(value) for value in values):
                health_errors.append('non-finite motor RPM command')
                return
            last_rpm_time = time.monotonic()
            rpm_samples += 1
            if mission_complete_time is not None:
                rpm_after_complete += 1

        latched_qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        subscriptions = [
            node.create_subscription(
                Path, '/drone/planned_path', on_planned_path, latched_qos),
            node.create_subscription(
                Path, '/drone/simplified_path', on_simplified_path, latched_qos),
            node.create_subscription(
                Path, '/drone/reference_path', on_reference_path, latched_qos),
            node.create_subscription(
                UInt32, '/drone/multi_goal/current_goal_index', on_goal_index, 10),
            node.create_subscription(
                UInt32, '/drone/multi_goal/visited_goals', on_visited, 10),
            node.create_subscription(
                Bool, '/drone/multi_goal/complete', on_complete, 10),
            node.create_subscription(
                Bool, '/drone/multi_goal/success', on_success, 10),
            node.create_subscription(
                Bool, '/drone/environment/in_collision', on_collision, 10),
            node.create_subscription(
                TrajectorySetpoint, '/drone/trajectory_setpoint', on_setpoint, 10),
            node.create_subscription(Odometry, '/drone/odom', on_odometry, 10),
            node.create_subscription(
                MotorRPM, '/drone/motor_rpm_cmd', on_motor_rpm, 10),
        ]
        required_nodes = {
            'quadrotor_dynamics_node',
            'position_controller_node',
            'robot_state_publisher',
            'static_environment_node',
            'multi_goal_static_avoidance_node',
        }

        try:
            discovery_deadline = time.monotonic() + DISCOVERY_TIMEOUT
            while time.monotonic() < discovery_deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
                if (required_nodes.issubset(set(node.get_node_names())) and
                        latest_position is not None and latest_setpoint is not None and
                        current_goal_index is not None and latest_visited is not None and
                        success_seen and rpm_samples > 0 and collision_samples > 0):
                    break
            else:
                self.fail(
                    'multi-goal ROS graph was not ready: '
                    f'nodes={sorted(node.get_node_names())}, odom={odom_samples}, '
                    f'setpoints={setpoint_samples}, rpm={rpm_samples}, '
                    f'collision={collision_samples}, success={success_seen}')

            self.assertEqual(node.count_publishers('/drone/planned_path'), 1)
            self.assertEqual(node.count_publishers('/drone/simplified_path'), 1)
            self.assertEqual(node.count_publishers('/drone/reference_path'), 1)
            self.assertEqual(node.count_publishers('/drone/trajectory_setpoint'), 1)
            self.assertNotIn('astar_planner_node', node.get_node_names())
            self.assertNotIn('planned_trajectory_node', node.get_node_names())
            self.assertNotIn('trajectory_mission_node', node.get_node_names())
            self.assertNotIn('waypoint_manager_node', node.get_node_names())

            mission_deadline = test_start + MISSION_TIMEOUT
            while time.monotonic() < mission_deadline and mission_complete_time is None:
                rclpy.spin_once(node, timeout_sec=0.01)
                now = time.monotonic()
                if health_errors:
                    self.fail(health_errors[0])
                if last_odom_time is not None and now - last_odom_time > 1.0:
                    self.fail('/drone/odom stopped')
                if last_setpoint_time is not None and now - last_setpoint_time > 1.0:
                    self.fail('/drone/trajectory_setpoint stopped')
                if last_rpm_time is not None and now - last_rpm_time > 1.0:
                    self.fail('/drone/motor_rpm_cmd stopped')
                if not required_nodes.issubset(set(node.get_node_names())):
                    self.fail(f'a required node exited: {sorted(node.get_node_names())}')

            self.assertIsNotNone(
                mission_complete_time,
                f'mission did not complete: goals={observed_goal_indices}, '
                f'visited={observed_visited_counts}, position={latest_position}')

            observation_deadline = mission_complete_time + POST_COMPLETE_OBSERVATION
            while time.monotonic() < observation_deadline:
                rclpy.spin_once(node, timeout_sec=0.01)
                if health_errors:
                    self.fail(health_errors[0])
                self.assertTrue(required_nodes.issubset(set(node.get_node_names())))

            final_position_error = norm3(tuple(
                latest_position[axis] - TARGETS[-1][axis] for axis in range(3)))
            final_setpoint_position = (
                latest_setpoint.position.x,
                latest_setpoint.position.y,
                latest_setpoint.position.z,
            )
            final_setpoint_velocity = (
                latest_setpoint.velocity.x,
                latest_setpoint.velocity.y,
                latest_setpoint.velocity.z,
            )
            final_setpoint_acceleration = (
                latest_setpoint.acceleration.x,
                latest_setpoint.acceleration.y,
                latest_setpoint.acceleration.z,
            )
            controller_output = b''.join(event.text for event in proc_output)
            saturation_true_count = controller_output.count(b'saturated=true')
            summary = (
                'multi_goal_static_avoidance_e2e: '
                f'mission_complete_time={mission_complete_time - test_start:.3f}s '
                f'goal_indices={observed_goal_indices} '
                f'visited_counts={observed_visited_counts} '
                f'paths=(raw={len(planned_paths)}, simplified={len(simplified_paths)}, '
                f'reference={len(reference_paths)}) '
                f'planning_start_errors={[round(v, 6) for v in planning_start_errors]} '
                f'goal_acceptance_errors={[round(v, 6) for v in goal_acceptance_errors]} '
                f'goal_acceptance_speeds={[round(v, 6) for v in goal_acceptance_speeds]} '
                f'max_tracking_errors={[round(v, 6) for v in maximum_tracking_errors]} '
                f'minimum_clearance={minimum_sampled_clearance:.6f}m '
                f'controller_saturated_true_logs={saturation_true_count} '
                f'final_error={final_position_error:.6f}m '
                f'final_speed={latest_speed:.6f}m/s '
                f'samples=(odom={odom_samples}, setpoint={setpoint_samples}, rpm={rpm_samples}, '
                f'collision={collision_samples}) post_complete=(odom={odom_after_complete}, '
                f'setpoint={setpoints_after_complete}, rpm={rpm_after_complete})'
            )
            print(summary, flush=True)

            self.assertEqual(observed_goal_indices, [0, 1, 2, 3], summary)
            self.assertEqual(observed_visited_counts, [0, 1, 2, 3, 4], summary)
            self.assertEqual(len(planned_paths), 4, summary)
            self.assertEqual(len(simplified_paths), 4, summary)
            self.assertEqual(len(reference_paths), 4, summary)
            self.assertEqual(len(planning_start_errors), 4, summary)
            self.assertTrue(all(value < 0.05 for value in planning_start_errors), summary)
            self.assertGreater(takeoff_anchor_setpoints, 100, summary)
            self.assertEqual(len(goal_acceptance_errors), 4, summary)
            self.assertTrue(all(value < 0.20 for value in goal_acceptance_errors), summary)
            self.assertTrue(all(value < 0.15 for value in goal_acceptance_speeds), summary)
            self.assertTrue(all(value < 0.10 for value in maximum_tracking_errors), summary)
            self.assertGreater(minimum_sampled_clearance, 0.05, summary)
            self.assertEqual(saturation_true_count, 0, summary)
            self.assertEqual(latest_setpoint.header.frame_id, 'map', summary)
            self.assertLess(norm3(tuple(
                final_setpoint_position[axis] - TARGETS[-1][axis]
                for axis in range(3))), 1.0e-9, summary)
            self.assertLess(norm3(final_setpoint_velocity), 1.0e-9, summary)
            self.assertLess(norm3(final_setpoint_acceleration), 1.0e-9, summary)
            self.assertLess(final_position_error, 0.20, summary)
            self.assertLess(latest_speed, 0.15, summary)
            self.assertGreater(odom_after_complete, 300, summary)
            self.assertGreater(setpoints_after_complete, 100, summary)
            self.assertGreater(rpm_after_complete, 200, summary)
            self.assertFalse(health_errors, '; '.join(health_errors))
        finally:
            for subscription in subscriptions:
                node.destroy_subscription(subscription)
            node.destroy_node()
            rclpy.shutdown()


@launch_testing.post_shutdown_test()
class TestMultiGoalStaticAvoidanceShutdown(unittest.TestCase):

    def test_processes_exit_cleanly(self, proc_info):
        process_names = proc_info.process_names()
        for expected in (
                'quadrotor_dynamics_node',
                'position_controller_node',
                'robot_state_publisher',
                'static_environment_node',
                'multi_goal_static_avoidance_node'):
            self.assertTrue(
                any(expected in name for name in process_names),
                f'{expected} was not launched: {process_names}')
        launch_testing.asserts.assertExitCodes(proc_info)
