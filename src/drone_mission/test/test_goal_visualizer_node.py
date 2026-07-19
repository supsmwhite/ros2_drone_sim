#!/usr/bin/env python3

import os
import time
import unittest

os.environ['ROS_DOMAIN_ID'] = '120'

from geometry_msgs.msg import Pose, PoseArray, PoseStamped
import launch
from launch_ros.actions import Node
import launch_testing
import launch_testing.actions
import launch_testing.markers
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, UInt32
from visualization_msgs.msg import Marker, MarkerArray


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    visualizer = Node(
        package='drone_mission', executable='goal_visualizer_node',
        output='screen')
    return launch.LaunchDescription([
        visualizer,
        launch_testing.actions.ReadyToTest(),
    ])


def marker_labels(message):
    return {
        marker.text for marker in message.markers
        if marker.type == Marker.TEXT_VIEW_FACING
    }


class TestGoalVisualizerNode(unittest.TestCase):

    def test_single_and_mission_marker_updates(self):
        rclpy.init()
        node = rclpy.create_node('goal_visualizer_node_test')
        state_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE)
        goal_publisher = node.create_publisher(PoseStamped, '/drone/goal', 10)
        goals_publisher = node.create_publisher(
            PoseArray, '/drone/mission/goals', state_qos)
        index_publisher = node.create_publisher(
            UInt32, '/drone/mission/current_waypoint_index', state_qos)
        complete_publisher = node.create_publisher(
            Bool, '/drone/mission/complete', state_qos)
        received = []
        marker_subscription = node.create_subscription(
            MarkerArray, '/drone/mission/goal_markers',
            lambda message: received.append(message), state_qos)

        def wait_for_labels(expected):
            deadline = time.monotonic() + 5.0
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
                if received and expected.issubset(marker_labels(received[-1])):
                    return
            self.fail(
                f'expected marker labels {expected}, got '
                f'{marker_labels(received[-1]) if received else set()}')

        try:
            deadline = time.monotonic() + 5.0
            while (goal_publisher.get_subscription_count() == 0 and
                   time.monotonic() < deadline):
                rclpy.spin_once(node, timeout_sec=0.05)
            self.assertGreater(goal_publisher.get_subscription_count(), 0)

            goal = PoseStamped()
            goal.header.frame_id = 'map'
            goal.pose.position.z = 1.5
            goal.pose.orientation.w = 1.0
            goal_publisher.publish(goal)
            wait_for_labels({'GOAL CURRENT'})

            goals = PoseArray()
            goals.header.frame_id = 'map'
            first = Pose()
            first.position.z = 1.5
            first.orientation.w = 1.0
            second = Pose()
            second.position.x = 2.0
            second.position.z = 1.5
            second.orientation.w = 1.0
            goals.poses = [first, second]
            goals_publisher.publish(goals)
            index_publisher.publish(UInt32(data=0))
            complete_publisher.publish(Bool(data=False))
            wait_for_labels({'P1 CURRENT', 'P2'})

            index_publisher.publish(UInt32(data=1))
            wait_for_labels({'P1 DONE', 'P2 CURRENT'})

            complete_publisher.publish(Bool(data=True))
            wait_for_labels({'P1 DONE', 'P2 DONE'})
        finally:
            node.destroy_subscription(marker_subscription)
            node.destroy_node()
            rclpy.shutdown()
