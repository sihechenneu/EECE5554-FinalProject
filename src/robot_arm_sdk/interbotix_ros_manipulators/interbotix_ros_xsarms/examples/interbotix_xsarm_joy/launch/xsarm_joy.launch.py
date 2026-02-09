#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_joy = get_package_share_directory('interbotix_xsarm_joy')
    pkg_control = get_package_share_directory('interbotix_xsarm_control')
    mode_configs = os.path.join(pkg_joy, 'config', 'modes.yaml')
    robot_model = LaunchConfiguration('robot_model', default='')
    robot_name = LaunchConfiguration('robot_name', default=robot_model)
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    threshold = LaunchConfiguration('threshold', default='0.75')
    controller = LaunchConfiguration('controller', default='ps4')
    launch_driver = LaunchConfiguration('launch_driver', default='true')
    use_sim = LaunchConfiguration('use_sim', default='false')
    base_link_frame = LaunchConfiguration('base_link_frame', default='base_link')

    control_launch = os.path.join(pkg_control, 'launch', 'xsarm_control.launch.py')
    return LaunchDescription([
        DeclareLaunchArgument('robot_model', default_value=''),
        DeclareLaunchArgument('robot_name', default_value=robot_model),
        DeclareLaunchArgument('base_link_frame', default_value='base_link'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('mode_configs', default_value=mode_configs),
        DeclareLaunchArgument('threshold', default_value='0.75'),
        DeclareLaunchArgument('controller', default_value='ps4'),
        DeclareLaunchArgument('launch_driver', default_value='true'),
        DeclareLaunchArgument('use_sim', default_value='false'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(control_launch),
            condition=IfCondition(launch_driver),
            launch_arguments={
                'robot_model': robot_model,
                'robot_name': robot_name,
                'base_link_frame': base_link_frame,
                'use_rviz': use_rviz,
                'mode_configs': mode_configs,
                'use_sim': use_sim,
            }.items(),
        ),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            namespace=robot_name,
            output='screen',
            parameters=[{'dev': '/dev/input/js0'}],
            remappings=[('joy', 'commands/joy_raw')],
        ),
        Node(
            package='interbotix_xsarm_joy',
            executable='xsarm_joy',
            name='xsarm_joy',
            namespace=robot_name,
            output='screen',
            parameters=[{'threshold': threshold}, {'controller': controller}],
        ),
        Node(
            package='interbotix_xsarm_joy',
            executable='xsarm_robot',
            name='xsarm_robot',
            namespace=robot_name,
            output='screen',
            parameters=[{'robot_model': robot_model}],
        ),
    ])
