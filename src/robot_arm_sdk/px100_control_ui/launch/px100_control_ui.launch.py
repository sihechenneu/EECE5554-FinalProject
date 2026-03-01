#!/usr/bin/env python3
"""Launch the PincherX 100 Control UI node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_name', default_value='px100', description='Namespace/robot name (must match arm driver)'),
        DeclareLaunchArgument('ui_port', default_value='8080', description='Port for the web UI'),
        Node(
            package='px100_control_ui',
            executable='ui_node',
            name='px100_control_ui',
            output='screen',
            parameters=[{
                'robot_name': LaunchConfiguration('robot_name'),
                'ui_port': LaunchConfiguration('ui_port'),
            }],
        ),
    ])
