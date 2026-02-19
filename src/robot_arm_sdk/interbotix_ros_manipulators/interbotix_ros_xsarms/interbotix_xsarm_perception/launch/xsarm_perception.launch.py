#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_perception = get_package_share_directory('interbotix_xsarm_perception')
    pkg_control = get_package_share_directory('interbotix_xsarm_control')

    robot_model = LaunchConfiguration('robot_model', default='wx200')
    robot_name = LaunchConfiguration('robot_name', default=robot_model)
    base_link_frame = LaunchConfiguration('base_link_frame', default='base_link')
    show_gripper_bar = LaunchConfiguration('show_gripper_bar', default='true')
    show_gripper_fingers = LaunchConfiguration('show_gripper_fingers', default='true')
    external_urdf_loc = LaunchConfiguration('external_urdf_loc', default='')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    load_configs = LaunchConfiguration('load_configs', default='true')
    filter_params = LaunchConfiguration('filter_params', default=os.path.join(pkg_perception, 'config', 'filter_params.yaml'))
    rvizconfig = LaunchConfiguration('rvizconfig', default=os.path.join(pkg_perception, 'rviz', 'xsarm_perception.rviz'))

    control_launch = os.path.join(pkg_control, 'launch', 'xsarm_control.launch.py')

    return LaunchDescription([
        DeclareLaunchArgument('robot_model', default_value='wx200'),
        DeclareLaunchArgument('robot_name', default_value=LaunchConfiguration('robot_model')),
        DeclareLaunchArgument('base_link_frame', default_value='base_link'),
        DeclareLaunchArgument('show_gripper_bar', default_value='true'),
        DeclareLaunchArgument('show_gripper_fingers', default_value='true'),
        DeclareLaunchArgument('external_urdf_loc', default_value=''),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('load_configs', default_value='true'),
        DeclareLaunchArgument('filter_params', default_value=os.path.join(pkg_perception, 'config', 'filter_params.yaml')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(control_launch),
            launch_arguments={
                'robot_model': robot_model,
                'robot_name': robot_name,
                'base_link_frame': base_link_frame,
                'show_ar_tag': 'true',
                'show_gripper_bar': show_gripper_bar,
                'show_gripper_fingers': show_gripper_fingers,
                'use_world_frame': 'false',
                'external_urdf_loc': external_urdf_loc,
                'use_rviz': 'false',
            }.items(),
        ),
        # realsense2_camera, pc_filter, armtag, static_transform_pub from interbotix_perception_modules
        # IncludeLaunchDescription(...) when those packages provide ROS2 launch files.
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            namespace=robot_name,
            arguments=['-d', rvizconfig],
            condition=IfCondition(use_rviz),
        ),
    ])
