#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_xs_sdk(context, *args, **kwargs):
    robot_model = context.perform_substitution(LaunchConfiguration('robot_model', default='wx200'))
    robot_name = context.launch_configurations.get('robot_name', robot_model)
    load_configs = context.perform_substitution(LaunchConfiguration('load_configs', default='true'))
    use_sim = context.perform_substitution(LaunchConfiguration('use_sim', default='false')).lower() == 'true'
    robot_port = context.perform_substitution(LaunchConfiguration('robot_port', default=''))
    pkg_control = get_package_share_directory('interbotix_xsarm_control')
    motor_configs_path = os.path.join(pkg_control, 'config', robot_model + '.yaml')
    mode_configs_path = os.path.join(pkg_control, 'config', 'modes.yaml')
    executable = 'xs_sdk_sim' if use_sim else 'xs_sdk'
    params = [
        {'motor_configs': motor_configs_path},
        {'mode_configs': mode_configs_path},
        {'load_configs': load_configs},  # string 'true'/'false'; xs_sdk reads as string
    ]
    if robot_port and robot_port != '':
        params.append({'port': robot_port})
    return [
        Node(
            package='interbotix_xs_sdk',
            executable=executable,
            name='xs_sdk',
            namespace=robot_name,
            output='screen',
            parameters=params,
        ),
    ]


def generate_launch_description():
    pkg_control = get_package_share_directory('interbotix_xsarm_control')
    pkg_descriptions = get_package_share_directory('interbotix_xsarm_descriptions')

    robot_model = LaunchConfiguration('robot_model', default='wx200')
    robot_name = LaunchConfiguration('robot_name', default=robot_model)
    base_link_frame = LaunchConfiguration('base_link_frame', default='base_link')
    show_ar_tag = LaunchConfiguration('show_ar_tag', default='false')
    show_gripper_bar = LaunchConfiguration('show_gripper_bar', default='true')
    show_gripper_fingers = LaunchConfiguration('show_gripper_fingers', default='true')
    use_world_frame = LaunchConfiguration('use_world_frame', default='true')
    external_urdf_loc = LaunchConfiguration('external_urdf_loc', default='')
    use_rviz = LaunchConfiguration('use_rviz', default='true')

    description_launch = os.path.join(pkg_descriptions, 'launch', 'xsarm_description.launch.py')

    return LaunchDescription([
        DeclareLaunchArgument('robot_model', default_value='wx200'),
        DeclareLaunchArgument('robot_name', default_value=LaunchConfiguration('robot_model')),
        DeclareLaunchArgument('base_link_frame', default_value='base_link'),
        DeclareLaunchArgument('show_ar_tag', default_value='false'),
        DeclareLaunchArgument('show_gripper_bar', default_value='true'),
        DeclareLaunchArgument('show_gripper_fingers', default_value='true'),
        DeclareLaunchArgument('use_world_frame', default_value='true'),
        DeclareLaunchArgument('external_urdf_loc', default_value=''),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('load_configs', default_value='true'),
        DeclareLaunchArgument('use_sim', default_value='false'),
        DeclareLaunchArgument('robot_port', default_value='', description='Serial port for the arm (e.g. /dev/ttyUSB0). Overrides port in motor config.'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(description_launch),
            launch_arguments={
                'robot_model': robot_model,
                'robot_name': robot_name,
                'base_link_frame': base_link_frame,
                'show_ar_tag': show_ar_tag,
                'show_gripper_bar': show_gripper_bar,
                'show_gripper_fingers': show_gripper_fingers,
                'use_world_frame': use_world_frame,
                'external_urdf_loc': external_urdf_loc,
                'use_rviz': use_rviz,
            }.items(),
        ),
        OpaqueFunction(function=launch_xs_sdk),
    ])
