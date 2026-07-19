#!/usr/bin/env python3

import os
import time
import unittest

os.environ['ROS_DOMAIN_ID'] = '119'

from drone_msgs.srv import ExecuteGoalSequence
from geometry_msgs.msg import Pose, PoseArray
import launch
from launch_ros.actions import Node
import launch_testing
import launch_testing.actions
import launch_testing.markers
import pytest
import rclpy
from std_msgs.msg import UInt32


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    manager = Node(
        package='drone_mission', executable='waypoint_manager_node',
        parameters=[{
            'start_with_configured_waypoints': False,
            'workspace': [-1.0, 4.0, -2.0, 3.0, -0.5, 5.0],
            'minimum_navigation_altitude': 0.5,
        }], output='screen')
    return launch.LaunchDescription([manager, launch_testing.actions.ReadyToTest()])


def pose(x, y, z, yaw_half=0.0):
    result = Pose()
    result.position.x, result.position.y, result.position.z = x, y, z
    result.orientation.z = __import__('math').sin(yaw_half)
    result.orientation.w = __import__('math').cos(yaw_half)
    return result


class TestWaypointService(unittest.TestCase):

    def test_accept_reset_and_active_rejection(self):
        rclpy.init()
        node = rclpy.create_node('waypoint_service_test')
        client = node.create_client(ExecuteGoalSequence, '/drone/mission/execute')
        indices = []
        subscription = node.create_subscription(
            UInt32, '/drone/mission/current_waypoint_index',
            lambda message: indices.append(message.data), 10)
        try:
            self.assertTrue(client.wait_for_service(timeout_sec=8.0))

            invalid = ExecuteGoalSequence.Request()
            invalid.goals.header.frame_id = 'map'
            invalid.goals.poses = [pose(0.0, 0.0, 0.2)]
            rejected = client.call_async(invalid)
            rclpy.spin_until_future_complete(node, rejected, timeout_sec=5.0)
            self.assertFalse(rejected.result().accepted)
            self.assertTrue(
                'workspace' in rejected.result().message or
                'height' in rejected.result().message,
                rejected.result().message)

            request = ExecuteGoalSequence.Request()
            request.goals = PoseArray()
            request.goals.header.frame_id = 'map'
            request.goals.poses = [pose(0.0, 0.0, 1.5), pose(2.0, 1.0, 1.5, 0.5)]
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
            self.assertTrue(future.result().accepted, future.result().message)
            self.assertIn('state reset', future.result().message)

            second = client.call_async(request)
            rclpy.spin_until_future_complete(node, second, timeout_sec=5.0)
            self.assertFalse(second.result().accepted)
            self.assertIn('active', second.result().message)

            deadline = time.monotonic() + 2.0
            while time.monotonic() < deadline and 0 not in indices:
                rclpy.spin_once(node, timeout_sec=0.05)
            self.assertIn(0, indices)

            status_count = len(indices)
            deadline = time.monotonic() + 0.4
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
            self.assertEqual(len(indices), status_count)
        finally:
            node.destroy_subscription(subscription)
            node.destroy_node()
            rclpy.shutdown()
