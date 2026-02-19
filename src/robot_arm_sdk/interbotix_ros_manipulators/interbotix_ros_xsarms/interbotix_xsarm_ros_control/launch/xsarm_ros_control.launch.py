#!/usr/bin/env python3
"""Launch xsarm with ros2_control (controller_manager + xs_hardware_interface)."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_ros_control(context, *args, **kwargs):
    robot_model = context.perform_substitution(LaunchConfiguration('robot_model', default='wx200'))
    robot_name = context.launch_configurations.get('robot_name', robot_model)
    dof = context.launch_configurations.get('dof', '5')
    pkg = get_package_share_directory('interbotix_xsarm_ros_control')
    controllers_file = os.path.join(pkg, 'config', dof + 'dof_controllers.yaml')
    hardware_file = os.path.join(pkg, 'config', 'hardware.yaml')
    return [
        Node(
            package='controller_manager',
            executable='controller_manager',
            name='controller_manager',
            namespace=robot_name,
            output='screen',
            parameters=[controllers_file, hardware_file],
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            name='controller_spawner',
            namespace=robot_name,
            arguments=['arm_controller', 'gripper_controller', '--param-file', controllers_file],
            output='screen',
        ),
        Node(
            package='interbotix_xs_ros_control',
            executable='xs_hardware_interface',
            name='xs_hardware_interface',
            namespace=robot_name,
            output='screen',
        ),
    ]


def generate_launch_description():
    pkg_control = get_package_share_directory('interbotix_xsarm_control')
    control_launch = os.path.join(pkg_control, 'launch', 'xsarm_control.launch.py')

    robot_model = LaunchConfiguration('robot_model', default='wx200')
    robot_name = LaunchConfiguration('robot_name', default=robot_model)
    base_link_frame = LaunchConfiguration('base_link_frame', default='base_link')
    show_ar_tag = LaunchConfiguration('show_ar_tag', default='false')
    use_world_frame = LaunchConfiguration('use_world_frame', default='true')
    external_urdf_loc = LaunchConfiguration('external_urdf_loc', default='')
    use_rviz = LaunchConfiguration('use_rviz', default='false')
    dof = LaunchConfiguration('dof', default='5')

    return LaunchDescription([
        DeclareLaunchArgument('robot_model', default_value='wx200'),
        DeclareLaunchArgument('robot_name', default_value=LaunchConfiguration('robot_model')),
        DeclareLaunchArgument('base_link_frame', default_value='base_link'),
        DeclareLaunchArgument('show_ar_tag', default_value='false'),
        DeclareLaunchArgument('use_world_frame', default_value='true'),
        DeclareLaunchArgument('external_urdf_loc', default_value=''),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument('dof', default_value='5'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(control_launch),
            launch_arguments={
                'robot_model': robot_model,
                'robot_name': robot_name,
                'base_link_frame': base_link_frame,
                'show_ar_tag': show_ar_tag,
                'use_world_frame': use_world_frame,
                'external_urdf_loc': external_urdf_loc,
                'use_rviz': use_rviz,
            }.items(),
        ),
        OpaqueFunction(function=launch_ros_control),
    ])
