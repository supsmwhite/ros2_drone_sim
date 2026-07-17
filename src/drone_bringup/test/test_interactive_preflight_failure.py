#!/usr/bin/env python3

import os
import time
import unittest

os.environ['ROS_DOMAIN_ID'] = '118'

from ament_index_python.packages import get_package_share_directory
from drone_msgs.msg import TrajectorySetpoint
from drone_msgs.srv import ExecuteGoalSequence
from geometry_msgs.msg import Pose
import launch
import launch_testing
import launch_testing.actions
import launch_testing.markers
from launch_ros.actions import Node
from nav_msgs.msg import Odometry
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
            {
                # Test-only closed wall: both endpoints are geometrically legal,
                # but no complete START -> P1 route exists.
                'workspace': [-1.0, 5.0, -2.5, 2.5, -0.5, 5.0],
                'obstacles': [2.0, 0.0, 2.35, 0.8, 5.0, 4.7],
                'interactive_mission_odom_wait_timeout': 2.0,
            },
        ],
    )
    return launch.LaunchDescription([
        executor,
        launch_testing.actions.ReadyToTest(),
    ])


class TestInteractivePreflightFailure(unittest.TestCase):

    def test_no_path_rejection_publishes_no_flight_setpoint(self):
        rclpy.init()
        node = rclpy.create_node('interactive_preflight_failure_test')
        qos = QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE)
        latest = {}
        setpoint_count = 0

        def on_setpoint(_message):
            nonlocal setpoint_count
            setpoint_count += 1

        subscriptions = [
            node.create_subscription(
                Bool, '/drone/interactive_mission/active',
                lambda msg: latest.__setitem__('active', msg.data), qos),
            node.create_subscription(
                Bool, '/drone/multi_goal/success',
                lambda msg: latest.__setitem__('success', msg.data), 10),
            node.create_subscription(
                String, '/drone/interactive_mission/status',
                lambda msg: latest.__setitem__('status', msg.data), qos),
            node.create_subscription(
                TrajectorySetpoint, '/drone/trajectory_setpoint', on_setpoint, 10),
        ]
        odom_publisher = node.create_publisher(Odometry, '/drone/odom', 10)
        client = node.create_client(
            ExecuteGoalSequence, '/drone/interactive_goals/execute')

        def spin_until(predicate, timeout, description, publish_odom=False):
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                if publish_odom:
                    odom = Odometry()
                    odom.header.frame_id = 'map'
                    odom.header.stamp = node.get_clock().now().to_msg()
                    odom.pose.pose.orientation.w = 1.0
                    odom_publisher.publish(odom)
                rclpy.spin_once(node, timeout_sec=0.05)
                if predicate():
                    return
            self.fail(f'timed out waiting for {description}; latest={latest}')

        try:
            self.assertTrue(client.wait_for_service(timeout_sec=8.0))
            spin_until(lambda: latest.get('active') is False, 3.0, 'waiting state')
            warmup_deadline = time.monotonic() + 0.4
            while time.monotonic() < warmup_deadline:
                odom = Odometry()
                odom.header.frame_id = 'map'
                odom.header.stamp = node.get_clock().now().to_msg()
                odom.pose.pose.orientation.w = 1.0
                odom_publisher.publish(odom)
                rclpy.spin_once(node, timeout_sec=0.05)

            request = ExecuteGoalSequence.Request()
            request.goals.header.frame_id = 'map'
            goal = Pose()
            request.goals.poses = [goal]
            goal.position.x = 4.0
            goal.position.y = 0.0
            goal.position.z = 1.5
            goal.orientation.w = 1.0
            request.draft_revision = 7
            future = client.call_async(request)
            spin_until(future.done, 4.0, 'service response', publish_odom=True)
            self.assertTrue(future.result().accepted)
            spin_until(
                lambda: latest.get('active') is False and
                latest.get('success') is False and
                latest.get('status', '').startswith(
                    'MISSION FAILED: REJECTED: preflight segment'),
                5.0, 'preflight no-path rejection', publish_odom=True)
            self.assertEqual(setpoint_count, 0)
            self.assertIn('START -> P1 A* failed', latest['status'])
        finally:
            for subscription in subscriptions:
                node.destroy_subscription(subscription)
            node.destroy_publisher(odom_publisher)
            node.destroy_client(client)
            node.destroy_node()
            rclpy.shutdown()


@launch_testing.post_shutdown_test()
class TestInteractivePreflightFailureShutdown(unittest.TestCase):

    def test_process_exits_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
