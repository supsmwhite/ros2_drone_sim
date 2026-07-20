#!/usr/bin/env python3

import math
import os
import time
import unittest

os.environ['ROS_DOMAIN_ID'] = '117'

from ament_index_python.packages import get_package_share_directory
from drone_msgs.msg import MotorRPM, TrajectorySetpoint
from geometry_msgs.msg import PoseArray
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
from std_msgs.msg import Bool, String, UInt32
from visualization_msgs.msg import InteractiveMarkerFeedback


TARGETS = ((3.5, 1.0, 2.5), (5.5, 1.0, 4.0), (7.0, 5.0, 4.0))
EXPECTED_YAWS = (math.pi / 2.0, math.pi, -math.pi / 2.0)
GOAL_POSITION_TOLERANCE = 0.20
GOAL_SPEED_TOLERANCE = 0.15
GOAL_YAW_TOLERANCE = 0.10
GOAL_ANGULAR_SPEED_TOLERANCE = 0.20
BASE_INFLATED_OBSTACLES = (
    ((1.95, -2.75, -0.25), (3.25, 1.75, 4.95)),
    ((3.95, 1.55, -0.25), (5.25, 6.75, 4.95)),
    ((6.05, -1.05, -0.25), (7.35, 2.65, 4.95)),
    ((8.25, 0.75, -0.25), (9.55, 6.75, 4.95)),
    ((10.75, -1.75, -0.25), (12.05, 0.05, 4.95)),
    ((10.75, 1.45, -0.25), (12.05, 4.45, 4.95)),
)


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    bringup = get_package_share_directory('drone_bringup')
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            bringup, 'launch', 'assessment_navigation_sim.launch.py')),
        launch_arguments={
            'use_rviz': 'false', 'scenario': 'obstacle_field',
            'yaw_mode': 'path_tangent'}.items(),
    )
    return launch.LaunchDescription([
        simulation,
        launch_testing.actions.ReadyToTest(),
    ])


def norm3(values):
    return math.sqrt(sum(value * value for value in values))


def distance_to_box(point, lower, upper):
    return norm3(tuple(
        max(lower[index] - point[index], 0.0, point[index] - upper[index])
        for index in range(3)))


def segment_intersects_box(start, end, lower, upper):
    low, high = 0.0, 1.0
    for axis in range(3):
        delta = end[axis] - start[axis]
        if delta == 0.0:
            if start[axis] < lower[axis] or start[axis] > upper[axis]:
                return False
            continue
        first = (lower[axis] - start[axis]) / delta
        second = (upper[axis] - start[axis]) / delta
        if first > second:
            first, second = second, first
        low, high = max(low, first), min(high, second)
        if low > high:
            return False
    return True


class TestInteractiveGoalNavigationEndToEnd(unittest.TestCase):

    def test_validated_snapshot_executes_and_editor_stays_locked(self, proc_output):
        rclpy.init()
        node = rclpy.create_node('interactive_goal_navigation_e2e_test')
        latched = QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE)
        latest = {}
        goal_indices = []
        visited_counts = []
        health_errors = []
        previous_position = None
        maximum_tracking_error = 0.0
        minimum_clearance = math.inf
        maximum_rpm = 0.0
        nonzero_rpm_before_execute = False
        accepted_time = None
        mission_complete_time = None
        success_false_after_accept = False
        previous_yaw_reference = None
        previous_yaw_stamp = None
        maximum_yaw_jump = 0.0
        maximum_yaw_reference_rate = 0.0
        accepted_actual_yaws = []
        goal_position_errors = []
        goal_speeds = []
        goal_yaw_errors = []
        goal_angular_speeds = []
        goal_yaw_reference_errors = [math.inf, math.inf, math.inf]
        non_finite_count = 0

        def on_odom(message):
            nonlocal previous_position, maximum_tracking_error, minimum_clearance
            nonlocal non_finite_count
            position = (message.pose.pose.position.x, message.pose.pose.position.y,
                        message.pose.pose.position.z)
            velocity = (message.twist.twist.linear.x, message.twist.twist.linear.y,
                        message.twist.twist.linear.z)
            angular_velocity = (
                message.twist.twist.angular.x,
                message.twist.twist.angular.y,
                message.twist.twist.angular.z)
            if not all(math.isfinite(value) for value in
                       position + velocity + angular_velocity):
                health_errors.append('non-finite Odom')
                non_finite_count += 1
            orientation = message.pose.pose.orientation
            yaw = math.atan2(
                2.0 * (orientation.w * orientation.z +
                       orientation.x * orientation.y),
                1.0 - 2.0 * (orientation.y * orientation.y +
                             orientation.z * orientation.z))
            if math.isfinite(yaw):
                latest['actual_yaw'] = yaw
            else:
                non_finite_count += 1
            for lower, upper in BASE_INFLATED_OBSTACLES:
                minimum_clearance = min(
                    minimum_clearance, distance_to_box(position, lower, upper))
                if previous_position and segment_intersects_box(
                        previous_position, position, lower, upper):
                    health_errors.append('actual Odom segment collision')
            previous_position = position
            latest['position'] = position
            latest['speed'] = norm3(velocity)
            latest['angular_speed'] = norm3(angular_velocity)
            # The takeoff setpoint intentionally steps from the ground to the
            # configured navigation altitude.  Tracking error is an execution
            # metric, so exclude that pre-navigation climb.
            if (latest.get('mission_status', '').startswith('EXECUTING') and
                    'setpoint' in latest):
                maximum_tracking_error = max(
                    maximum_tracking_error,
                    math.dist(position, latest['setpoint']))

        def on_setpoint(message):
            nonlocal previous_yaw_reference, previous_yaw_stamp
            nonlocal maximum_yaw_jump, maximum_yaw_reference_rate
            nonlocal non_finite_count
            latest['setpoint'] = (
                message.position.x, message.position.y, message.position.z)
            stamp = message.header.stamp.sec + 1.0e-9 * message.header.stamp.nanosec
            if not math.isfinite(message.yaw):
                non_finite_count += 1
                return
            if previous_yaw_reference is not None:
                jump = abs(math.atan2(
                    math.sin(message.yaw - previous_yaw_reference),
                    math.cos(message.yaw - previous_yaw_reference)))
                maximum_yaw_jump = max(maximum_yaw_jump, jump)
                delta_time = stamp - previous_yaw_stamp
                if delta_time > 1.0e-6:
                    maximum_yaw_reference_rate = max(
                        maximum_yaw_reference_rate, jump / delta_time)
            previous_yaw_reference = message.yaw
            previous_yaw_stamp = stamp
            if 'goal_index' in latest:
                expected = EXPECTED_YAWS[latest['goal_index']]
                error = abs(math.atan2(
                    math.sin(message.yaw - expected),
                    math.cos(message.yaw - expected)))
                goal_yaw_reference_errors[latest['goal_index']] = min(
                    goal_yaw_reference_errors[latest['goal_index']], error)

        def on_rpm(message):
            nonlocal maximum_rpm, nonzero_rpm_before_execute
            values = (
                message.m1_front_left_ccw_rpm,
                message.m2_rear_left_cw_rpm,
                message.m3_rear_right_ccw_rpm,
                message.m4_front_right_cw_rpm,
            )
            if not all(math.isfinite(value) for value in values):
                health_errors.append('non-finite RPM')
            maximum_rpm = max(maximum_rpm, *values)
            if accepted_time is None and max(values) > 1.0e-6:
                nonzero_rpm_before_execute = True

        def on_goal_index(message):
            latest['goal_index'] = message.data
            if not goal_indices or goal_indices[-1] != message.data:
                goal_indices.append(message.data)

        def on_visited(message):
            if not visited_counts or visited_counts[-1] != message.data:
                visited_counts.append(message.data)
                if message.data > 0:
                    accepted_index = message.data - 1
                    expected = EXPECTED_YAWS[accepted_index]
                    accepted_actual_yaws.append(latest['actual_yaw'])
                    goal_position_errors.append(math.dist(
                        latest['position'], TARGETS[accepted_index]))
                    goal_speeds.append(latest['speed'])
                    goal_yaw_errors.append(abs(math.atan2(
                        math.sin(latest['actual_yaw'] - expected),
                        math.cos(latest['actual_yaw'] - expected))))
                    goal_angular_speeds.append(latest['angular_speed'])

        def on_success(message):
            nonlocal success_false_after_accept
            if accepted_time is not None and not message.data:
                success_false_after_accept = True

        def on_complete(message):
            nonlocal mission_complete_time
            latest['complete'] = message.data
            if message.data and mission_complete_time is None:
                mission_complete_time = time.monotonic()

        subscriptions = [
            node.create_subscription(Odometry, '/drone/odom', on_odom, 20),
            node.create_subscription(TrajectorySetpoint, '/drone/trajectory_setpoint',
                                     on_setpoint, 20),
            node.create_subscription(MotorRPM, '/drone/motor_rpm_cmd', on_rpm, 20),
            node.create_subscription(UInt32, '/drone/multi_goal/current_goal_index',
                                     on_goal_index, 20),
            node.create_subscription(UInt32, '/drone/multi_goal/visited_goals',
                                     on_visited, 20),
            node.create_subscription(Bool, '/drone/multi_goal/success', on_success, 20),
            node.create_subscription(Bool, '/drone/multi_goal/complete', on_complete, 20),
            node.create_subscription(Bool, '/drone/interactive_mission/active',
                                     lambda msg: latest.__setitem__('active', msg.data), latched),
            node.create_subscription(String, '/drone/interactive_mission/status',
                                     lambda msg: latest.__setitem__('mission_status', msg.data), latched),
            node.create_subscription(String, '/drone/interactive_goals/status',
                                     lambda msg: latest.__setitem__('editor_status', msg.data), latched),
            node.create_subscription(Bool, '/drone/interactive_goals/ready',
                                     lambda msg: latest.__setitem__('ready', msg.data), latched),
            node.create_subscription(UInt32, '/drone/interactive_goals/count',
                                     lambda msg: latest.__setitem__('count', msg.data), latched),
            node.create_subscription(PoseArray, '/drone/interactive_goals/selected_goals',
                                     lambda msg: latest.__setitem__('goals', msg), latched),
            node.create_subscription(Path, '/drone/interactive_goals/preview_path',
                                     lambda msg: latest.__setitem__('preview', msg), latched),
            node.create_subscription(Path, '/drone/planned_path',
                                     lambda msg: latest.__setitem__('planned', msg), latched),
            node.create_subscription(Path, '/drone/simplified_path',
                                     lambda msg: latest.__setitem__('simplified', msg), latched),
            node.create_subscription(Path, '/drone/reference_path',
                                     lambda msg: latest.__setitem__('reference', msg), latched),
            node.create_subscription(Path, '/drone/path',
                                     lambda msg: latest.__setitem__('actual_path', msg), 10),
        ]
        feedback_publisher = node.create_publisher(
            InteractiveMarkerFeedback,
            '/drone/interactive_goals/goal_editor/feedback', 10)

        def spin_until(predicate, timeout, description):
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.02)
                if predicate():
                    return
            self.fail(f'timed out waiting for {description}; latest={latest}')

        def feedback(event_type, point=(0.0, 0.0, 1.5), menu_entry=0,
                     yaw=0.0, control_name=None):
            message = InteractiveMarkerFeedback()
            message.header.frame_id = 'map'
            message.client_id = 'navigation_e2e'
            message.marker_name = 'goal_candidate'
            message.control_name = control_name or (
                'menu' if menu_entry else 'move_xy')
            message.event_type = event_type
            message.menu_entry_id = menu_entry
            message.pose.position.x, message.pose.position.y, message.pose.position.z = point
            message.pose.orientation.z = math.sin(0.5 * yaw)
            message.pose.orientation.w = math.cos(0.5 * yaw)
            feedback_publisher.publish(message)

        def set_candidate(point):
            feedback(InteractiveMarkerFeedback.POSE_UPDATE, point)
            end = time.monotonic() + 0.12
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.02)
            feedback(InteractiveMarkerFeedback.MOUSE_UP, point)

        def menu(entry):
            feedback(InteractiveMarkerFeedback.MENU_SELECT, menu_entry=entry)

        def set_yaw(point, yaw):
            feedback(InteractiveMarkerFeedback.POSE_UPDATE, point, yaw=yaw,
                     control_name='rotate_z')
            end = time.monotonic() + 0.12
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.02)

        try:
            spin_until(
                lambda: feedback_publisher.get_subscription_count() > 0 and
                latest.get('mission_status') == 'WAITING FOR VALIDATED MISSION' and
                latest.get('active') is False and latest.get('count') == 0 and
                'position' in latest,
                10.0, 'idle editor and executor')
            idle_deadline = time.monotonic() + 2.0
            while time.monotonic() < idle_deadline:
                rclpy.spin_once(node, timeout_sec=0.02)
            self.assertFalse(nonzero_rpm_before_execute)
            self.assertLess(abs(latest['position'][2]), 0.02)

            menu(9)  # Execute before READY.
            spin_until(
                lambda: latest.get('editor_status', '').startswith('EXECUTE REJECTED:'),
                3.0, 'READY gate rejection')
            self.assertFalse(latest.get('active', False))

            for index, target in enumerate(TARGETS[:2], start=1):
                set_candidate(target)
                set_yaw(target, EXPECTED_YAWS[index - 1])
                menu(1)
                spin_until(lambda: latest.get('count') == index, 3.0, f'goal {index}')
            # The final candidate is confirmed by Validate & Preview itself.
            set_candidate(TARGETS[2])
            set_yaw(TARGETS[2], EXPECTED_YAWS[2])
            menu(8)  # Validate & Preview.
            spin_until(
                lambda: latest.get('ready') is True and
                latest.get('count') == 3 and
                len(latest.get('preview', Path()).poses) > 2,
                35.0, 'validated preview')
            snapshot = [
                (pose.position.x, pose.position.y, pose.position.z)
                for pose in latest['goals'].poses]
            self.assertEqual(snapshot, list(TARGETS))
            selected_yaws = [
                math.atan2(
                    2.0 * pose.orientation.w * pose.orientation.z,
                    1.0 - 2.0 * pose.orientation.z * pose.orientation.z)
                for pose in latest['goals'].poses]
            for actual, expected in zip(selected_yaws, EXPECTED_YAWS):
                self.assertAlmostEqual(actual, expected, places=6)

            menu(9)  # Execute Validated Mission.
            spin_until(lambda: latest.get('active') is True, 8.0, 'execution active')
            accepted_time = time.monotonic()
            self.assertEqual(latest.get('count'), 3)
            spin_until(
                lambda: len(latest.get('preview', Path()).poses) == 0,
                3.0, 'editor preview hidden')

            # Locked editor must ignore drag and every mutating menu action.
            set_candidate((0.8, 0.7, 2.0))
            for entry in (1, 2, 3, 8, 9):
                menu(entry)
            lock_deadline = time.monotonic() + 1.0
            while time.monotonic() < lock_deadline:
                rclpy.spin_once(node, timeout_sec=0.02)
            self.assertEqual(latest.get('count'), 3)
            locked_snapshot = [
                (pose.position.x, pose.position.y, pose.position.z)
                for pose in latest['goals'].poses]
            self.assertEqual(locked_snapshot, snapshot)

            spin_until(lambda: mission_complete_time is not None, 95.0, 'mission complete')
            post_deadline = time.monotonic() + 2.0
            while time.monotonic() < post_deadline:
                rclpy.spin_once(node, timeout_sec=0.02)

            self.assertEqual(goal_indices[:3], [0, 1, 2])
            self.assertEqual(visited_counts, [0, 1, 2, 3])
            self.assertFalse(success_false_after_accept)
            self.assertFalse(latest.get('active', True))
            self.assertEqual(latest.get('mission_status'), 'MISSION COMPLETE')
            self.assertGreater(len(latest.get('actual_path', Path()).poses), 10)
            self.assertEqual(len(latest.get('planned', Path()).poses), 0)
            self.assertEqual(len(latest.get('simplified', Path()).poses), 0)
            self.assertEqual(len(latest.get('reference', Path()).poses), 0)
            self.assertFalse(health_errors, health_errors)
            self.assertLess(maximum_tracking_error, 0.05)
            self.assertGreaterEqual(minimum_clearance, 0.085)
            self.assertLess(math.dist(latest['position'], TARGETS[-1]), 0.05)
            self.assertLess(latest['speed'], 0.03)
            self.assertEqual(len(accepted_actual_yaws), 3)
            self.assertEqual(len(goal_position_errors), 3)
            self.assertEqual(len(goal_speeds), 3)
            self.assertEqual(len(goal_yaw_errors), 3)
            self.assertEqual(len(goal_angular_speeds), 3)
            for error in goal_position_errors:
                self.assertLess(error, GOAL_POSITION_TOLERANCE)
            for speed in goal_speeds:
                self.assertLess(speed, GOAL_SPEED_TOLERANCE)
            for error in goal_yaw_errors:
                self.assertLess(error, GOAL_YAW_TOLERANCE)
            for speed in goal_angular_speeds:
                self.assertLess(speed, GOAL_ANGULAR_SPEED_TOLERANCE)

            output = b''.join(event.text for event in proc_output)
            self.assertIn(b'preview and execution enabled', output)
            saturation_count = output.count(b'saturated=true')
            self.assertEqual(saturation_count, 0)
            print(
                'interactive_goal_navigation_e2e: '
                f'task_time={mission_complete_time - accepted_time:.3f}s '
                f'goals={TARGETS} max_tracking_error={maximum_tracking_error:.6f}m '
                f'minimum_clearance={minimum_clearance:.6f}m '
                f'maximum_rpm={maximum_rpm:.1f} final_error='
                f'{math.dist(latest["position"], TARGETS[-1]):.6f}m '
                f'final_speed={latest["speed"]:.6f}m/s '
                f'accepted_actual_yaws={accepted_actual_yaws} '
                f'goal_position_errors={goal_position_errors} '
                f'goal_speeds={goal_speeds} '
                f'goal_yaw_errors={goal_yaw_errors} '
                f'goal_angular_speeds={goal_angular_speeds} '
                f'goal_yaw_reference_errors={goal_yaw_reference_errors} '
                f'maximum_yaw_jump={maximum_yaw_jump:.6f}rad '
                f'maximum_yaw_reference_rate={maximum_yaw_reference_rate:.6f}rad/s '
                f'collision_count={len([error for error in health_errors if "collision" in error])} '
                f'non_finite_count={non_finite_count} '
                f'saturation_count={saturation_count}')
        finally:
            for subscription in subscriptions:
                node.destroy_subscription(subscription)
            node.destroy_publisher(feedback_publisher)
            node.destroy_node()
            rclpy.shutdown()


@launch_testing.post_shutdown_test()
class TestInteractiveGoalNavigationShutdown(unittest.TestCase):

    def test_processes_exit_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
