#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_uxarm_nodes(context, *args, **kwargs):
    robot_model = context.perform_substitution(LaunchConfiguration('robot_model', default='uxarm6'))
    robot_name = context.launch_configurations.get('robot_name', robot_model)
    robot_ip = context.launch_configurations.get('robot_ip', '')
    pkg_control = get_package_share_directory('interbotix_uxarm_control')
    config_file = os.path.join(pkg_control, 'config', robot_model + '.yaml')
    return [
        Node(
            package='xarm_api',
            executable='xarm_driver_node',
            name='xarm_driver_node',
            namespace=robot_name,
            output='screen',
            parameters=[{'xarm_robot_ip': robot_ip}, config_file],
            remappings=[('joint_states', 'arm/joint_states')],
        ),
        # gripper_pub node omitted until interbotix_uxarm_control C++ and ux_modules are ported
    ]


def generate_launch_description():
    pkg_control = get_package_share_directory('interbotix_uxarm_control')
    pkg_descriptions = get_package_share_directory('interbotix_uxarm_descriptions')
    description_launch = os.path.join(pkg_descriptions, 'launch', 'uxarm_description.launch.py')

    robot_model = LaunchConfiguration('robot_model', default='uxarm6')
    robot_name = LaunchConfiguration('robot_name', default=robot_model)
    base_link_frame = LaunchConfiguration('base_link_frame', default='base_link')
    use_gripper = LaunchConfiguration('use_gripper', default='false')
    show_gripper = LaunchConfiguration('show_gripper', default=use_gripper)
    use_world_frame = LaunchConfiguration('use_world_frame', default='true')
    external_urdf_loc = LaunchConfiguration('external_urdf_loc', default='')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    robot_ip = LaunchConfiguration('robot_ip', default='')

    return LaunchDescription([
        DeclareLaunchArgument('robot_model', default_value='uxarm6'),
        DeclareLaunchArgument('robot_name', default_value=LaunchConfiguration('robot_model')),
        DeclareLaunchArgument('base_link_frame', default_value='base_link'),
        DeclareLaunchArgument('use_gripper', default_value='false'),
        DeclareLaunchArgument('show_gripper', default_value=LaunchConfiguration('use_gripper')),
        DeclareLaunchArgument('use_world_frame', default_value='true'),
        DeclareLaunchArgument('external_urdf_loc', default_value=''),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('robot_ip', default_value=''),
        DeclareLaunchArgument('gripper_pub_freq', default_value='10'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(description_launch),
            launch_arguments={
                'robot_model': robot_model,
                'robot_name': robot_name,
                'base_link_frame': base_link_frame,
                'show_gripper': show_gripper,
                'use_world_frame': use_world_frame,
                'external_urdf_loc': external_urdf_loc,
                'use_rviz': use_rviz,
                'use_joint_pub': 'true',
                'rate': '10',
            }.items(),
        ),
        OpaqueFunction(function=launch_uxarm_nodes),
    ])
