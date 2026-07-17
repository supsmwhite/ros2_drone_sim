#!/usr/bin/env python3

import os
import time
import unittest

os.environ['ROS_DOMAIN_ID'] = '115'

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseArray
import launch
import launch_testing
import launch_testing.actions
import launch_testing.markers
from launch_ros.actions import Node
from nav_msgs.msg import Path
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String, UInt32
from visualization_msgs.msg import InteractiveMarkerFeedback


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    bringup_share = get_package_share_directory('drone_bringup')
    editor_node = Node(
        package='drone_planning',
        executable='interactive_goal_editor_node',
        name='interactive_goal_editor_node',
        output='screen',
        parameters=[
            os.path.join(bringup_share, 'config', 'environment.yaml'),
            os.path.join(bringup_share, 'config', 'astar.yaml'),
            os.path.join(bringup_share, 'config', 'planned_trajectory.yaml'),
            os.path.join(
                bringup_share, 'config', 'interactive_goal_editor.yaml'),
        ],
    )
    return launch.LaunchDescription([
        editor_node,
        launch_testing.actions.ReadyToTest(),
    ])


class TestInteractiveGoalEditorEndToEnd(unittest.TestCase):

    def test_rviz_uses_humble_interactive_marker_namespace(self):
        bringup_share = get_package_share_directory('drone_bringup')
        rviz_config = os.path.join(
            bringup_share, 'rviz', 'drone_sim.rviz')
        with open(rviz_config, encoding='utf-8') as config_file:
            config = config_file.read()
        self.assertIn(
            'Interactive Markers Namespace: '
            '/drone/interactive_goals/goal_editor',
            config,
        )
        self.assertNotIn(
            'Update Topic: /drone/interactive_goals/goal_editor/update',
            config,
        )

    def test_editor_topics_validation_and_read_only_contract(self):
        rclpy.init()
        node = rclpy.create_node('interactive_goal_editor_e2e_test')
        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        latest = {}

        subscriptions = [
            node.create_subscription(
                PoseArray, '/drone/interactive_goals/selected_goals',
                lambda message: latest.__setitem__('goals', message), qos),
            node.create_subscription(
                Path, '/drone/interactive_goals/preview_path',
                lambda message: latest.__setitem__('path', message), qos),
            node.create_subscription(
                String, '/drone/interactive_goals/status',
                lambda message: latest.__setitem__('status', message), qos),
            node.create_subscription(
                Bool, '/drone/interactive_goals/ready',
                lambda message: latest.__setitem__('ready', message), qos),
            node.create_subscription(
                UInt32, '/drone/interactive_goals/count',
                lambda message: latest.__setitem__('count', message), qos),
        ]
        feedback_publisher = node.create_publisher(
            InteractiveMarkerFeedback,
            '/drone/interactive_goals/goal_editor/feedback', 10)

        def spin_until(predicate, timeout, description):
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
                if predicate():
                    return
            self.fail(f'timed out waiting for {description}; latest={latest}')

        def feedback(event_type, x=0.0, y=0.0, z=1.5, menu_entry=0):
            message = InteractiveMarkerFeedback()
            message.header.frame_id = 'map'
            message.client_id = 'launch_test'
            message.marker_name = 'goal_candidate'
            message.control_name = 'menu' if menu_entry else 'move_xy'
            message.event_type = event_type
            message.menu_entry_id = menu_entry
            message.pose.position.x = x
            message.pose.position.y = y
            message.pose.position.z = z
            message.pose.orientation.w = 1.0
            feedback_publisher.publish(message)

        def set_candidate(x, y, z):
            feedback(InteractiveMarkerFeedback.POSE_UPDATE, x, y, z)
            end = time.monotonic() + 0.15
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.02)
            feedback(InteractiveMarkerFeedback.MOUSE_UP, x, y, z)

        def select_menu(entry_id):
            feedback(InteractiveMarkerFeedback.MENU_SELECT, menu_entry=entry_id)

        def add_goal(index, point):
            set_candidate(*point)
            select_menu(1)  # Add Goal is the first menu entry.
            spin_until(
                lambda: latest.get('count') is not None and
                latest['count'].data == index,
                3.0, f'goal count {index}')

        try:
            spin_until(
                lambda: all(key in latest for key in
                            ('goals', 'path', 'status', 'ready', 'count')) and
                feedback_publisher.get_subscription_count() > 0,
                8.0, 'initial transient-local editor state')
            self.assertEqual(latest['goals'].header.frame_id, 'map')
            self.assertEqual(len(latest['goals'].poses), 0)
            self.assertEqual(latest['count'].data, 0)
            self.assertEqual(len(latest['path'].poses), 0)
            self.assertFalse(latest['ready'].data)

            topic_names = dict(node.get_topic_names_and_types())
            self.assertIn(
                '/drone/interactive_goals/goal_editor/update', topic_names)
            self.assertNotIn('/drone/trajectory_setpoint', topic_names)
            self.assertNotIn('/drone/motor_rpm_cmd', topic_names)

            # An obstacle-interior point is rejected and reports the exact reason.
            set_candidate(2.6, -0.5, 1.5)
            select_menu(1)
            spin_until(
                lambda: latest.get('status') is not None and
                'INSIDE PLANNING-INFLATED OBSTACLE' in latest['status'].data,
                3.0, 'explicit obstacle rejection')
            self.assertEqual(latest['count'].data, 0)

            legal_three = [
                (13.2, 5.5, 1.5),
                (7.0, 5.0, 4.0),
                (0.8, 0.7, 2.0),
            ]
            for index, point in enumerate(legal_three, start=1):
                add_goal(index, point)
            select_menu(8)  # Validate & Preview follows the height submenu.
            spin_until(
                lambda: latest.get('ready') is not None and latest['ready'].data,
                45.0, 'three-goal full continuous preview')
            self.assertEqual(latest['count'].data, 3)
            self.assertGreater(len(latest['path'].poses), 2)
            self.assertIn('READY', latest['status'].data)

            # Editing after READY invalidates and clears the latched preview immediately.
            set_candidate(0.85, 0.75, 2.0)
            spin_until(
                lambda: not latest['ready'].data and len(latest['path'].poses) == 0,
                3.0, 'preview invalidation after editing')

            # Five confirmed targets are accepted without a hard-coded count of three.
            select_menu(3)  # Clear All Goals.
            spin_until(lambda: latest['count'].data == 0, 3.0, 'clear all')
            legal_five = [
                (0.8, 0.7, 2.0),
                (3.5, 1.0, 2.5),
                (5.5, 1.0, 4.0),
                (7.0, 5.0, 4.0),
                (0.8, 0.7, 2.0),
            ]
            for index, point in enumerate(legal_five, start=1):
                add_goal(index, point)
            self.assertEqual(len(latest['goals'].poses), 5)
            self.assertEqual(latest['count'].data, 5)
            select_menu(8)
            spin_until(
                lambda: latest.get('status') is not None and
                (latest['status'].data.startswith('READY:') or
                 latest['status'].data.startswith('REJECTED:')),
                45.0, 'explicit five-goal planning outcome')
            self.assertIn(
                latest['status'].data.split(':', maxsplit=1)[0],
                ('READY', 'REJECTED'))
        finally:
            for subscription in subscriptions:
                node.destroy_subscription(subscription)
            node.destroy_publisher(feedback_publisher)
            node.destroy_node()
            rclpy.shutdown()


@launch_testing.post_shutdown_test()
class TestInteractiveGoalEditorShutdown(unittest.TestCase):

    def test_process_exits_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
