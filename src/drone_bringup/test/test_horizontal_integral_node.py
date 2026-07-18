#!/usr/bin/env python3

import math
import os
import time
import unittest

os.environ['ROS_DOMAIN_ID'] = '92'

from drone_msgs.msg import ControllerDiagnostics, MotorRPM, TrajectorySetpoint
from geometry_msgs.msg import PoseStamped
import launch
import launch_testing
import launch_testing.actions
import launch_testing.markers
from launch_ros.actions import Node
from nav_msgs.msg import Odometry
import pytest
import rclpy


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    common_parameters = {
        'control_frequency': 100.0,
        'odometry_timeout': 0.2,
        'horizontal_position_kp_x': 0.4,
        'horizontal_position_kp_y': 0.4,
        'horizontal_velocity_kd_x': 1.2,
        'horizontal_velocity_kd_y': 1.2,
        'max_horizontal_acceleration': 0.8,
        'max_tilt_angle': 0.15,
        'horizontal_position_ki_x': 0.15,
        'horizontal_position_ki_y': 0.15,
        'horizontal_integral_acceleration_limit': 0.35,
        'horizontal_anti_windup_gain': 2.0,
        'horizontal_integrator_unload_gain': 2.0,
        'horizontal_integral_capture_radius': 0.5,
        'horizontal_integral_reset_distance': 1.0,
    }
    controller = Node(
        package='drone_controller', executable='position_controller_node',
        name='position_controller_node', output='screen', parameters=[{
            **common_parameters,
            'setpoint_source': 'pose_goal',
            'enable_horizontal_integral': True,
        }])
    trajectory_controller = Node(
        package='drone_controller', executable='position_controller_node',
        name='trajectory_position_controller_node', output='screen', parameters=[{
            **common_parameters,
            'setpoint_source': 'trajectory',
            'enable_horizontal_integral': True,
        }], remappings=[
            ('/drone/trajectory_setpoint', '/trajectory/drone/trajectory_setpoint'),
            ('/drone/odom', '/trajectory/drone/odom'),
            ('/drone/motor_rpm_cmd', '/trajectory/drone/motor_rpm_cmd'),
            ('/drone/controller/diagnostics', '/trajectory/drone/controller/diagnostics'),
        ])
    disabled_controller = Node(
        package='drone_controller', executable='position_controller_node',
        name='disabled_integral_position_controller_node', output='screen', parameters=[{
            **common_parameters,
            'setpoint_source': 'pose_goal',
            'enable_horizontal_integral': False,
        }], remappings=[
            ('/drone/goal', '/disabled/drone/goal'),
            ('/drone/odom', '/disabled/drone/odom'),
            ('/drone/motor_rpm_cmd', '/disabled/drone/motor_rpm_cmd'),
            ('/drone/controller/diagnostics', '/disabled/drone/controller/diagnostics'),
        ])
    return launch.LaunchDescription([
        controller,
        trajectory_controller,
        disabled_controller,
        launch_testing.actions.ReadyToTest(),
    ])


class TestHorizontalIntegralNode(unittest.TestCase):

    def test_continuous_trajectory_does_not_reset_and_disabled_integral_stays_zero(self):
        rclpy.init()
        node = rclpy.create_node('horizontal_integral_node_modes_test')
        trajectory_publisher = node.create_publisher(
            TrajectorySetpoint, '/trajectory/drone/trajectory_setpoint', 10)
        trajectory_odom_publisher = node.create_publisher(
            Odometry, '/trajectory/drone/odom', 10)
        disabled_goal_publisher = node.create_publisher(
            PoseStamped, '/disabled/drone/goal', 10)
        disabled_odom_publisher = node.create_publisher(
            Odometry, '/disabled/drone/odom', 10)
        trajectory_diagnostics = []
        disabled_diagnostics = []
        subscriptions = [
            node.create_subscription(
                ControllerDiagnostics, '/trajectory/drone/controller/diagnostics',
                trajectory_diagnostics.append, 20),
            node.create_subscription(
                ControllerDiagnostics, '/disabled/drone/controller/diagnostics',
                disabled_diagnostics.append, 20),
        ]
        try:
            goal = PoseStamped()
            goal.header.frame_id = 'map'
            goal.pose.position.x = 0.1
            goal.pose.position.z = 1.5
            goal.pose.orientation.w = 1.0
            for _ in range(80):
                stamp = node.get_clock().now().to_msg()
                trajectory = TrajectorySetpoint()
                trajectory.header.stamp = stamp
                trajectory.header.frame_id = 'map'
                trajectory.position.x = 0.1
                trajectory.position.z = 1.5
                trajectory_publisher.publish(trajectory)
                odometry = Odometry()
                odometry.header.stamp = stamp
                odometry.pose.pose.position.z = 1.5
                odometry.pose.pose.orientation.w = 1.0
                trajectory_odom_publisher.publish(odometry)
                goal.header.stamp = stamp
                disabled_goal_publisher.publish(goal)
                disabled_odom_publisher.publish(odometry)
                rclpy.spin_once(node, timeout_sec=0.01)
            self.assertTrue(trajectory_diagnostics)
            self.assertGreater(trajectory_diagnostics[-1].horizontal_i_acceleration_x, 0.0)
            self.assertFalse(any(
                message.horizontal_integral_reset
                for message in trajectory_diagnostics[1:]))
            self.assertTrue(disabled_diagnostics)
            self.assertTrue(all(
                not message.horizontal_integral_enabled and
                message.horizontal_i_acceleration_x == 0.0 and
                message.horizontal_i_acceleration_y == 0.0
                for message in disabled_diagnostics))
        finally:
            for subscription in subscriptions:
                node.destroy_subscription(subscription)
            node.destroy_node()
            rclpy.shutdown()

    def test_integrator_lifecycle_and_safe_output(self):
        rclpy.init()
        node = rclpy.create_node('horizontal_integral_node_test')
        goal_publisher = node.create_publisher(PoseStamped, '/drone/goal', 10)
        odometry_publisher = node.create_publisher(Odometry, '/drone/odom', 10)
        diagnostics = []
        rpm_messages = []
        subscriptions = [
            node.create_subscription(
                ControllerDiagnostics, '/drone/controller/diagnostics',
                diagnostics.append, 20),
            node.create_subscription(MotorRPM, '/drone/motor_rpm_cmd', rpm_messages.append, 20),
        ]

        def spin_for(duration):
            deadline = time.monotonic() + duration
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.02)

        def publish_goal(x, z=1.5, valid=True):
            message = PoseStamped()
            message.header.stamp = node.get_clock().now().to_msg()
            message.header.frame_id = 'map'
            message.pose.position.x = x
            message.pose.position.z = z
            message.pose.orientation.w = 1.0 if valid else 0.0
            goal_publisher.publish(message)

        def publish_odometry(x, z, vx=0.0):
            message = Odometry()
            message.header.stamp = node.get_clock().now().to_msg()
            message.header.frame_id = 'map'
            message.child_frame_id = 'base_link'
            message.pose.pose.position.x = x
            message.pose.pose.position.z = z
            message.pose.pose.orientation.w = 1.0
            message.twist.twist.linear.x = vx
            odometry_publisher.publish(message)

        def rpm_values(message):
            return (message.m1_front_left_ccw_rpm, message.m2_rear_left_cw_rpm,
                    message.m3_rear_right_ccw_rpm, message.m4_front_right_cw_rpm)

        try:
            spin_for(0.3)
            self.assertTrue(rpm_messages)
            self.assertTrue(all(value == 0.0 for value in rpm_values(rpm_messages[-1])))

            publish_goal(0.1)
            for _ in range(20):
                publish_odometry(0.0, 0.0)
                spin_for(0.01)
            self.assertTrue(diagnostics)
            self.assertTrue(diagnostics[-1].horizontal_integral_enabled)
            self.assertTrue(diagnostics[-1].horizontal_integral_frozen)
            self.assertAlmostEqual(diagnostics[-1].horizontal_i_acceleration_x, 0.0)

            for _ in range(60):
                publish_odometry(0.0, 1.5)
                spin_for(0.01)
            self.assertGreater(diagnostics[-1].horizontal_i_acceleration_x, 0.0)

            normal_motion_start = len(diagnostics)
            for index in range(20):
                publish_odometry(0.0, 1.5, vx=0.3 if index % 2 else -0.3)
                spin_for(0.01)
            self.assertFalse(any(
                message.horizontal_integrator_unloading_active
                for message in diagnostics[normal_motion_start:]))

            backcalc_start = len(diagnostics)
            for _ in range(10):
                publish_odometry(0.0, 1.5, vx=1.0)
                spin_for(0.01)
            self.assertTrue(any(
                message.horizontal_saturation_backcalc_active
                for message in diagnostics[backcalc_start:]))

            unload_start = len(diagnostics)
            for _ in range(10):
                publish_odometry(0.2, 1.5)
                spin_for(0.01)
            unloading = [
                message for message in diagnostics[unload_start:]
                if message.horizontal_integrator_unloading_active]
            self.assertTrue(unloading)
            self.assertTrue(all(message.horizontal_anti_windup_active for message in unloading))
            self.assertTrue(all(not message.horizontal_saturation_backcalc_active
                                for message in unloading))

            rpm_messages.clear()
            spin_for(0.3)
            self.assertTrue(rpm_messages)
            self.assertTrue(all(value == 0.0 for value in rpm_values(rpm_messages[-1])))
            for _ in range(10):
                publish_odometry(0.0, 1.5)
                spin_for(0.01)
            self.assertTrue(math.isfinite(diagnostics[-1].horizontal_i_acceleration_x))

            diagnostic_count_before_jump = len(diagnostics)
            publish_goal(2.0)
            for _ in range(10):
                publish_odometry(0.0, 1.5)
                spin_for(0.01)
            self.assertTrue(any(
                message.horizontal_integral_reset
                for message in diagnostics[diagnostic_count_before_jump:]))
            self.assertAlmostEqual(diagnostics[-1].horizontal_i_acceleration_x, 0.0)

            publish_goal(0.0, valid=False)
            rpm_messages.clear()
            for _ in range(10):
                publish_odometry(0.0, 1.5)
                spin_for(0.01)
            self.assertTrue(rpm_messages)
            self.assertTrue(all(math.isfinite(value) and value == 0.0
                                for value in rpm_values(rpm_messages[-1])))
        finally:
            for subscription in subscriptions:
                node.destroy_subscription(subscription)
            node.destroy_node()
            rclpy.shutdown()
