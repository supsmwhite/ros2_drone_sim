#!/usr/bin/env python3

import math
import os
import time
import unittest

os.environ['ROS_DOMAIN_ID'] = '98'

from ament_index_python.packages import get_package_share_directory
import launch
import launch_testing
import launch_testing.actions
import launch_testing.markers
from launch_ros.actions import Node
from nav_msgs.msg import Path
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float64, UInt32


START = (0.0, 0.0, 1.5)
GOAL = (8.0, 5.0, 1.5)
VELOCITY_SCALE_CANDIDATES = (1.0, 0.75, 0.5, 0.25, 0.0)
EFFECTIVE_INFLATED_OBSTACLES = [
    ((1.75, -0.85, -0.35), (3.25, 2.85, 3.35)),
    ((5.25, 2.15, -0.35), (6.75, 5.85, 3.35)),
]


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    bringup_share = get_package_share_directory('drone_bringup')
    environment_parameters = os.path.join(
        bringup_share, 'config', 'environment.yaml')
    astar_parameters = os.path.join(
        bringup_share, 'config', 'astar.yaml')
    trajectory_parameters = os.path.join(
        bringup_share, 'config', 'planned_trajectory.yaml')
    environment_node = Node(
        package='drone_planning',
        executable='static_environment_node',
        name='static_environment_node',
        output='screen',
        parameters=[environment_parameters],
    )
    planner_node = Node(
        package='drone_planning',
        executable='astar_planner_node',
        name='astar_planner_node',
        output='screen',
        parameters=[environment_parameters, astar_parameters],
    )
    trajectory_node = Node(
        package='drone_planning',
        executable='planned_trajectory_node',
        name='planned_trajectory_node',
        output='screen',
        parameters=[
            environment_parameters, astar_parameters, trajectory_parameters],
    )
    return launch.LaunchDescription([
        environment_node,
        planner_node,
        trajectory_node,
        launch_testing.actions.ReadyToTest(),
    ])


def segment_intersects_closed_box(start, end, box_min, box_max):
    enter = 0.0
    exit_time = 1.0
    for axis in range(3):
        direction = end[axis] - start[axis]
        if direction == 0.0:
            if start[axis] < box_min[axis] or start[axis] > box_max[axis]:
                return False
            continue
        first = (box_min[axis] - start[axis]) / direction
        second = (box_max[axis] - start[axis]) / direction
        if first > second:
            first, second = second, first
        enter = max(enter, first)
        exit_time = min(exit_time, second)
        if enter > exit_time:
            return False
    return True


def path_points(message):
    return [(
        pose.pose.position.x,
        pose.pose.position.y,
        pose.pose.position.z,
    ) for pose in message.poses]


def assert_path_finite_and_matches(test_case, message):
    test_case.assertEqual(message.header.frame_id, 'map')
    points = path_points(message)
    test_case.assertGreaterEqual(len(points), 2)
    for point in points:
        test_case.assertTrue(all(math.isfinite(value) for value in point))
    for actual, expected in zip(points[0], START):
        test_case.assertAlmostEqual(actual, expected, places=12)
    for actual, expected in zip(points[-1], GOAL):
        test_case.assertAlmostEqual(actual, expected, places=12)
    return points


def assert_segments_safe(test_case, points, description):
    for index, (start, end) in enumerate(zip(points, points[1:])):
        for box_min, box_max in EFFECTIVE_INFLATED_OBSTACLES:
            test_case.assertFalse(
                segment_intersects_closed_box(start, end, box_min, box_max),
                f'{description} segment {index} intersects effective obstacle: '
                f'{start} -> {end}',
            )


class TestPlannedTrajectoryEndToEnd(unittest.TestCase):

    def test_raw_simplified_and_reference_paths_are_independently_safe(self):
        rclpy.init()
        node = rclpy.create_node('planned_trajectory_e2e_test')
        messages = {}
        result_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        subscriptions = [
            node.create_subscription(
                Path, '/drone/planned_path',
                lambda message: messages.__setitem__('raw', message), result_qos),
            node.create_subscription(
                Path, '/drone/simplified_path',
                lambda message: messages.__setitem__('simplified', message), result_qos),
            node.create_subscription(
                Path, '/drone/reference_path',
                lambda message: messages.__setitem__('reference', message), result_qos),
            node.create_subscription(
                Bool, '/drone/trajectory_generation/success',
                lambda message: messages.__setitem__('success', message), result_qos),
            node.create_subscription(
                UInt32, '/drone/trajectory_generation/simplified_waypoints',
                lambda message: messages.__setitem__('count', message), result_qos),
            node.create_subscription(
                Float64, '/drone/trajectory_generation/selected_velocity_scale',
                lambda message: messages.__setitem__('scale', message), result_qos),
            node.create_subscription(
                Float64, '/drone/trajectory_generation/duration',
                lambda message: messages.__setitem__('duration', message), result_qos),
        ]

        try:
            required_nodes = {
                'static_environment_node',
                'astar_planner_node',
                'planned_trajectory_node',
            }
            required_messages = {
                'raw', 'simplified', 'reference', 'success',
                'count', 'scale', 'duration',
            }
            deadline = time.monotonic() + 12.0
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
                if (required_messages.issubset(messages) and
                        required_nodes.issubset(set(node.get_node_names()))):
                    break
            else:
                self.fail(
                    f'planned trajectory results not received; '
                    f'nodes={node.get_node_names()}, messages={messages.keys()}')

            self.assertTrue(messages['success'].data)
            raw = assert_path_finite_and_matches(self, messages['raw'])
            simplified = assert_path_finite_and_matches(
                self, messages['simplified'])
            reference = assert_path_finite_and_matches(
                self, messages['reference'])
            self.assertGreater(len(raw), len(simplified))
            self.assertGreater(len(reference), len(simplified))
            self.assertEqual(messages['count'].data, len(simplified))
            self.assertIn(messages['scale'].data, VELOCITY_SCALE_CANDIDATES)
            self.assertTrue(math.isfinite(messages['duration'].data))
            self.assertGreater(messages['duration'].data, 0.0)
            assert_segments_safe(self, simplified, 'simplified path')
            assert_segments_safe(self, reference, 'reference path')
            topic_names = {
                name for name, _types in node.get_topic_names_and_types()}
            self.assertNotIn('/drone/trajectory_setpoint', topic_names)

            raw_length = sum(math.dist(a, b) for a, b in zip(raw, raw[1:]))
            simplified_length = sum(
                math.dist(a, b) for a, b in zip(simplified, simplified[1:]))
            print(
                'planned_trajectory_e2e: '
                f'raw_points={len(raw)} simplified_points={len(simplified)} '
                f'reference_points={len(reference)} raw_length={raw_length:.6f} '
                f'simplified_length={simplified_length:.6f} '
                f'velocity_scale={messages["scale"].data:.2f} '
                f'duration={messages["duration"].data:.6f} collision=false',
                flush=True,
            )
        finally:
            for subscription in subscriptions:
                node.destroy_subscription(subscription)
            node.destroy_node()
            rclpy.shutdown()


@launch_testing.post_shutdown_test()
class TestPlannedTrajectoryShutdown(unittest.TestCase):

    def test_processes_exit_cleanly(self, proc_info):
        process_names = proc_info.process_names()
        for expected in (
                'static_environment_node',
                'astar_planner_node',
                'planned_trajectory_node'):
            self.assertTrue(
                any(expected in name for name in process_names),
                f'{expected} was not launched: {process_names}')
        launch_testing.asserts.assertExitCodes(proc_info)
