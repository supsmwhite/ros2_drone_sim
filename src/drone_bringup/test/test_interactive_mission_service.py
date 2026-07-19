#!/usr/bin/env python3

import math
import os
import time
import unittest

os.environ['ROS_DOMAIN_ID'] = '116'

from ament_index_python.packages import get_package_share_directory
from drone_msgs.srv import ExecuteGoalSequence
from geometry_msgs.msg import Pose
import launch
import launch_testing
import launch_testing.actions
import launch_testing.markers
from launch_ros.actions import Node
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    bringup = get_package_share_directory('drone_bringup')
    executor = Node(
        package='drone_planning',
        executable='multi_goal_static_avoidance_node',
        name='multi_goal_static_avoidance_node',
        output='screen',
        parameters=[
            os.path.join(bringup, 'config', 'environment.yaml'),
            os.path.join(bringup, 'config', 'astar.yaml'),
            os.path.join(bringup, 'config', 'planned_trajectory.yaml'),
            os.path.join(bringup, 'config', 'interactive_goal_executor.yaml'),
            {'interactive_mission_odom_wait_timeout': 1.0,
             'yaw_mode': 'path_tangent'},
        ],
    )
    return launch.LaunchDescription([
        executor,
        launch_testing.actions.ReadyToTest(),
    ])


def pose(x, y, z, yaw=0.0):
    result = Pose()
    result.position.x = x
    result.position.y = y
    result.position.z = z
    result.orientation.z = math.sin(0.5 * yaw)
    result.orientation.w = math.cos(0.5 * yaw)
    return result


class TestInteractiveMissionService(unittest.TestCase):

    def test_request_validation_waiting_and_duplicate_rejection(self):
        rclpy.init()
        node = rclpy.create_node('interactive_mission_service_test')
        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        latest = {}
        subscriptions = [
            node.create_subscription(
                Bool, '/drone/interactive_mission/active',
                lambda message: latest.__setitem__('active', message.data), qos),
            node.create_subscription(
                String, '/drone/interactive_mission/status',
                lambda message: latest.__setitem__('status', message.data), qos),
            node.create_subscription(
                Bool, '/drone/multi_goal/complete',
                lambda message: latest.__setitem__('complete', message.data), 10),
            node.create_subscription(
                Bool, '/drone/multi_goal/success',
                lambda message: latest.__setitem__('success', message.data), 10),
        ]
        client = node.create_client(
            ExecuteGoalSequence, '/drone/interactive_goals/execute')

        def spin_until(predicate, timeout, description):
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
                if predicate():
                    return
            self.fail(f'timed out waiting for {description}; latest={latest}')

        def call(poses, frame='map', revision=1):
            request = ExecuteGoalSequence.Request()
            request.goals.header.frame_id = frame
            request.goals.poses = poses
            request.draft_revision = revision
            future = client.call_async(request)
            spin_until(future.done, 4.0, 'service response')
            return future.result()

        try:
            self.assertTrue(client.wait_for_service(timeout_sec=8.0))
            spin_until(
                lambda: latest.get('status') == 'WAITING FOR VALIDATED MISSION' and
                latest.get('active') is False and latest.get('complete') is False,
                4.0, 'interactive waiting state')

            invalid_requests = [
                ([], 'map', 'empty'),
                ([pose(0.8, 0.7, 2.0)], 'odom', 'frame_id'),
                ([pose(float('nan'), 0.7, 2.0)], 'map', 'non-finite'),
                ([pose(0.8, 0.7, 0.3)], 'map', 'navigation floor'),
                ([pose(20.0, 0.7, 2.0)], 'map', 'safe workspace'),
                ([pose(2.6, -0.5, 1.5)], 'map', 'inflated obstacle'),
                ([pose(0.8, 0.7, 2.0)] * 9, 'map', 'max_goals'),
            ]
            invalid_orientation = pose(0.8, 0.7, 2.0)
            invalid_orientation.orientation.w = 0.0
            invalid_requests.append(
                ([invalid_orientation], 'map', 'invalid orientation'))
            for poses, frame, reason in invalid_requests:
                response = call(poses, frame)
                self.assertFalse(response.accepted)
                self.assertIn('REJECTED:', response.message)
                self.assertIn(reason, response.message)

            response = call([
                pose(3.5, 1.0, 2.5, 0.2),
                pose(5.5, 1.0, 4.0),
                pose(7.0, 5.0, 4.0),
            ], revision=42)
            self.assertTrue(response.accepted)
            self.assertIn('preflight validation started', response.message)
            spin_until(lambda: latest.get('active') is True, 3.0, 'active mission')

            duplicate = call([pose(0.8, 0.7, 2.0)], revision=43)
            self.assertFalse(duplicate.accepted)
            self.assertEqual(duplicate.message, 'REJECTED: another mission is active')

            spin_until(
                lambda: latest.get('active') is False and
                latest.get('success') is False and
                latest.get('status', '').startswith('MISSION FAILED: REJECTED:'),
                4.0, 'fresh Odom timeout rejection')
        finally:
            for subscription in subscriptions:
                node.destroy_subscription(subscription)
            node.destroy_client(client)
            node.destroy_node()
            rclpy.shutdown()


@launch_testing.post_shutdown_test()
class TestInteractiveMissionServiceShutdown(unittest.TestCase):

    def test_process_exits_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
