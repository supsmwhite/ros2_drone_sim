#!/usr/bin/env python3

import math
import os
import time
import unittest

os.environ['ROS_DOMAIN_ID'] = '133'

from ament_index_python.packages import get_package_share_directory
from drone_msgs.msg import ControllerDiagnostics, MotorRPM
from drone_msgs.srv import ExecuteGoalSequence
from geometry_msgs.msg import Pose
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
from std_msgs.msg import Bool, String
from visualization_msgs.msg import MarkerArray


START = (0.0, 0.0, 1.5)
TARGET = (8.5, 0.0, 1.5)
# Effective AABBs include the unchanged 0.25 m safety radius plus 0.10 m
# planning margin.  Base-inflated boxes are used for actual vehicle clearance.
RAW_BOXES = (
    ((2.6, -3.0, 0.0), (3.4, 0.3, 4.7)),
    ((2.6, 2.1, 0.0), (3.4, 3.0, 4.7)),
    ((5.6, -3.0, 0.0), (6.4, -1.9, 4.7)),
    ((5.6, -0.1, 0.0), (6.4, 3.0, 4.7)),
    ((7.7, 1.85, 0.0), (8.7, 2.85, 4.7)),
)


def inflate(box, amount):
    lower, upper = box
    return (tuple(value - amount for value in lower),
            tuple(value + amount for value in upper))


PLANNING_BOXES = tuple(inflate(box, 0.35) for box in RAW_BOXES)
BASE_BOXES = tuple(inflate(box, 0.25) for box in RAW_BOXES)


def point(message):
    return (message.x, message.y, message.z)


def path_points(message):
    return [point(pose.pose.position) for pose in message.poses]


def path_length(points):
    return sum(math.dist(a, b) for a, b in zip(points, points[1:]))


def distance_to_box(value, box):
    lower, upper = box
    return math.sqrt(sum(
        max(lower[i] - value[i], 0.0, value[i] - upper[i]) ** 2
        for i in range(3)))


def segment_intersects_box(start, end, box):
    lower, upper = box
    low, high = 0.0, 1.0
    for axis in range(3):
        delta = end[axis] - start[axis]
        if abs(delta) < 1.0e-12:
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


def path_is_safe(points, boxes):
    return all(
        not segment_intersects_box(a, b, box)
        for a, b in zip(points, points[1:]) for box in boxes)


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    launch_file = os.path.join(
        get_package_share_directory('drone_bringup'), 'launch',
        'assessment_navigation_sim.launch.py')
    return launch.LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_file),
            launch_arguments={
                'scenario': 'narrow_passage', 'yaw_mode': 'path_tangent',
                'use_rviz': 'false'}.items()),
        launch_testing.actions.ReadyToTest(),
    ])


class TestAssessmentNavigationNarrowPassage(unittest.TestCase):

    def test_forced_safe_s_route_completes(self, proc_output):
        rclpy.init()
        node = rclpy.create_node('assessment_navigation_narrow_e2e_test')
        latched = QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE)
        latest = {}
        saved_paths = {'planned': [], 'simplified': [], 'reference': [], 'actual': []}
        collision_count = 0
        saturation_count = 0
        health_errors = []
        minimum_clearance = math.inf

        def save_path(name, message):
            points = path_points(message)
            if len(points) > len(saved_paths[name]):
                saved_paths[name] = points

        def on_odom(message):
            nonlocal minimum_clearance
            values = (
                message.pose.pose.position.x, message.pose.pose.position.y,
                message.pose.pose.position.z, message.twist.twist.linear.x,
                message.twist.twist.linear.y, message.twist.twist.linear.z,
                message.twist.twist.angular.x, message.twist.twist.angular.y,
                message.twist.twist.angular.z)
            if not all(math.isfinite(value) for value in values):
                health_errors.append('non-finite Odom')
                return
            latest['position'] = values[:3]
            latest['speed'] = math.sqrt(sum(value * value for value in values[3:6]))
            minimum_clearance = min(
                minimum_clearance,
                *(distance_to_box(values[:3], box) for box in BASE_BOXES))

        def on_collision(message):
            nonlocal collision_count
            collision_count += int(message.data)

        def on_rpm(message):
            values = (
                message.m1_front_left_ccw_rpm, message.m2_rear_left_cw_rpm,
                message.m3_rear_right_ccw_rpm, message.m4_front_right_cw_rpm)
            if not all(math.isfinite(value) and 0.0 <= value <= 20000.0
                       for value in values):
                health_errors.append('invalid RPM')

        def on_diagnostics(message):
            nonlocal saturation_count
            if (message.horizontal_saturated or message.altitude_saturated or
                    message.attitude_saturated or message.mixer_saturated):
                saturation_count += 1

        subscriptions = [
            node.create_subscription(Odometry, '/drone/odom', on_odom, 20),
            node.create_subscription(MotorRPM, '/drone/motor_rpm_cmd', on_rpm, 20),
            node.create_subscription(
                ControllerDiagnostics, '/drone/controller/diagnostics',
                on_diagnostics, 20),
            node.create_subscription(
                Bool, '/drone/environment/in_collision', on_collision, 20),
            node.create_subscription(
                MarkerArray, '/drone/environment/markers',
                lambda msg: latest.__setitem__('markers', msg), latched),
            node.create_subscription(
                Path, '/drone/planned_path', lambda msg: save_path('planned', msg), latched),
            node.create_subscription(
                Path, '/drone/simplified_path',
                lambda msg: save_path('simplified', msg), latched),
            node.create_subscription(
                Path, '/drone/reference_path',
                lambda msg: save_path('reference', msg), latched),
            node.create_subscription(
                Path, '/drone/path', lambda msg: save_path('actual', msg), 10),
            node.create_subscription(
                Bool, '/drone/multi_goal/complete',
                lambda msg: latest.__setitem__('complete', msg.data), 20),
            node.create_subscription(
                Bool, '/drone/multi_goal/success',
                lambda msg: latest.__setitem__('success', msg.data), 20),
            node.create_subscription(
                String, '/drone/interactive_mission/status',
                lambda msg: latest.__setitem__('status', msg.data), latched),
        ]
        client = node.create_client(
            ExecuteGoalSequence, '/drone/interactive_goals/execute')

        def spin_until(predicate, timeout, description):
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.02)
                if health_errors:
                    self.fail(health_errors[0])
                if predicate():
                    return
            self.fail(f'timed out waiting for {description}; latest={latest}')

        try:
            self.assertTrue(client.wait_for_service(timeout_sec=8.0))
            spin_until(
                lambda: 'position' in latest and 'markers' in latest and
                latest.get('status') == 'WAITING FOR VALIDATED MISSION',
                8.0, 'narrow-passage graph')
            self.assertEqual(len(latest['markers'].markers), 11)
            self.assertNotEqual(len(latest['markers'].markers), 13)
            self.assertTrue(any(
                segment_intersects_box(START, TARGET, box) for box in RAW_BOXES))

            request = ExecuteGoalSequence.Request()
            request.goals.header.frame_id = 'map'
            goal = Pose()
            goal.position.x, goal.position.y, goal.position.z = TARGET
            goal.orientation.w = 1.0
            request.goals.poses = [goal]
            request.draft_revision = 1
            future = client.call_async(request)
            spin_until(future.done, 5.0, 'mission request')
            self.assertTrue(future.result().accepted, future.result().message)
            spin_until(lambda: len(saved_paths['reference']) > 10, 25.0, 'safe paths')
            spin_until(lambda: latest.get('complete') is True, 75.0, 'mission complete')
            self.assertTrue(latest.get('success'))

            planned = saved_paths['planned']
            simplified = saved_paths['simplified']
            reference = saved_paths['reference']
            actual = saved_paths['actual']
            for name, values in (
                    ('planned', planned), ('simplified', simplified),
                    ('reference', reference)):
                self.assertGreater(len(values), 1, name)
                self.assertTrue(path_is_safe(values, PLANNING_BOXES), name)
            straight_distance = math.dist(START, TARGET)
            planned_length = path_length(planned)
            maximum_deviation = max(abs(value[1]) for value in planned)
            self.assertGreater(planned_length, straight_distance + 1.0)
            self.assertGreater(maximum_deviation, 0.75)
            self.assertTrue(any(2.25 <= p[0] <= 3.75 and 0.65 < p[1] < 1.75
                                for p in planned))
            self.assertTrue(any(5.25 <= p[0] <= 6.75 and -1.55 < p[1] < -0.45
                                for p in planned))
            self.assertTrue(any(2.4 <= p[0] <= 3.6 and 0.3 < p[1] < 2.1
                                for p in actual))
            self.assertTrue(any(5.4 <= p[0] <= 6.6 and -1.9 < p[1] < -0.1
                                for p in actual))
            final_error = math.dist(latest['position'], TARGET)
            self.assertLess(final_error, 0.05)
            self.assertLess(latest['speed'], 0.03)
            self.assertGreater(minimum_clearance, 0.0)
            self.assertEqual(collision_count, 0)
            self.assertEqual(saturation_count, 0)
            output = b''.join(event.text for event in proc_output)
            self.assertIn(b'static environment started with 5 obstacles', output)
            print(
                'assessment_navigation_narrow_e2e: '
                f'straight_distance={straight_distance:.6f} '
                f'planned_length={planned_length:.6f} '
                f'maximum_deviation={maximum_deviation:.6f} '
                f'minimum_clearance={minimum_clearance:.6f} '
                f'final_error={final_error:.6f}', flush=True)
        finally:
            for subscription in subscriptions:
                node.destroy_subscription(subscription)
            node.destroy_client(client)
            node.destroy_node()
            rclpy.shutdown()


@launch_testing.post_shutdown_test()
class TestAssessmentNavigationNarrowShutdown(unittest.TestCase):

    def test_processes_exit_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
