#!/usr/bin/env python3

import math
import os
import time
import unittest

os.environ['ROS_DOMAIN_ID'] = '97'

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
from std_msgs.msg import Bool, UInt32


START = (0.0, 0.0, 1.5)
GOAL = (12.0, 2.7, 1.5)
RESOLUTION = 0.25
EFFECTIVE_INFLATED_OBSTACLES = [
    ((2.05, -2.85, -0.35), (3.55, 1.85, 5.05)),
    ((5.45, 0.65, -0.35), (6.95, 6.85, 5.05)),
    ((5.45, -2.85, -0.35), (6.95, -0.85, 5.05)),
    ((8.85, -2.85, -0.35), (10.35, 1.85, 5.05)),
    ((8.85, 3.65, -0.35), (10.35, 6.85, 5.05)),
]


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    bringup_share = get_package_share_directory('drone_bringup')
    environment_parameters = os.path.join(
        bringup_share, 'config', 'environment.yaml')
    astar_parameters = os.path.join(
        bringup_share, 'config', 'astar.yaml')
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
    return launch.LaunchDescription([
        environment_node,
        planner_node,
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


class TestAStarPlannerEndToEnd(unittest.TestCase):

    def test_default_environment_path_is_independently_safe(self):
        rclpy.init()
        node = rclpy.create_node('astar_planner_e2e_test')
        path_message = None
        success_message = None
        expanded_nodes_message = None
        result_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        def on_path(message):
            nonlocal path_message
            path_message = message

        def on_success(message):
            nonlocal success_message
            success_message = message

        def on_expanded_nodes(message):
            nonlocal expanded_nodes_message
            expanded_nodes_message = message

        path_subscription = node.create_subscription(
            Path, '/drone/planned_path', on_path, result_qos)
        success_subscription = node.create_subscription(
            Bool, '/drone/planning/success', on_success, result_qos)
        expanded_subscription = node.create_subscription(
            UInt32, '/drone/planning/expanded_nodes',
            on_expanded_nodes, result_qos)

        try:
            deadline = time.monotonic() + 10.0
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
                nodes = node.get_node_names()
                if (path_message is not None and
                        success_message is not None and
                        expanded_nodes_message is not None and
                        'static_environment_node' in nodes and
                        'astar_planner_node' in nodes):
                    break
            else:
                self.fail(
                    f'planning result not received; nodes={node.get_node_names()}, '
                    f'path={path_message}, success={success_message}, '
                    f'expanded={expanded_nodes_message}')

            self.assertTrue(success_message.data)
            self.assertGreater(expanded_nodes_message.data, 0)
            self.assertEqual(path_message.header.frame_id, 'map')
            self.assertGreater(len(path_message.poses), 2)

            points = [(
                pose.pose.position.x,
                pose.pose.position.y,
                pose.pose.position.z,
            ) for pose in path_message.poses]
            for pose, point in zip(path_message.poses, points):
                self.assertTrue(all(math.isfinite(value) for value in point))
                orientation = pose.pose.orientation
                orientation_norm = math.sqrt(
                    orientation.x ** 2 + orientation.y ** 2 +
                    orientation.z ** 2 + orientation.w ** 2)
                self.assertAlmostEqual(orientation_norm, 1.0, places=12)
            for actual, expected in zip(points[0], START):
                self.assertAlmostEqual(actual, expected, places=12)
            for actual, expected in zip(points[-1], GOAL):
                self.assertAlmostEqual(actual, expected, places=12)

            path_length = 0.0
            for index, (segment_start, segment_end) in enumerate(
                    zip(points, points[1:])):
                step = math.dist(segment_start, segment_end)
                path_length += step
                if index != 0 and index != len(points) - 2:
                    self.assertLessEqual(
                        step, math.sqrt(3.0) * RESOLUTION + 1.0e-9)
                for box_min, box_max in EFFECTIVE_INFLATED_OBSTACLES:
                    self.assertFalse(
                        segment_intersects_closed_box(
                            segment_start, segment_end, box_min, box_max),
                        f'path segment {index} intersects inflated obstacle: '
                        f'{segment_start} -> {segment_end}')

            direct_distance = math.dist(START, GOAL)
            self.assertGreater(path_length, direct_distance)
            max_height = max(point[2] for point in points)
            print(
                'astar_planner_e2e: '
                f'path_nodes={len(points)} path_length={path_length:.6f} '
                f'expanded_nodes={expanded_nodes_message.data} '
                f'max_height={max_height:.6f}',
                flush=True,
            )
        finally:
            node.destroy_subscription(path_subscription)
            node.destroy_subscription(success_subscription)
            node.destroy_subscription(expanded_subscription)
            node.destroy_node()
            rclpy.shutdown()


@launch_testing.post_shutdown_test()
class TestAStarPlannerShutdown(unittest.TestCase):

    def test_processes_exit_cleanly(self, proc_info):
        process_names = proc_info.process_names()
        self.assertTrue(any(
            'static_environment_node' in name for name in process_names))
        self.assertTrue(any(
            'astar_planner_node' in name for name in process_names))
        launch_testing.asserts.assertExitCodes(proc_info)
