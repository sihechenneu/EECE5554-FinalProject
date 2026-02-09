#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('interbotix_uxarm_descriptions')

    robot_model = LaunchConfiguration('robot_model', default='uxarm6')
    robot_name = LaunchConfiguration('robot_name', default=LaunchConfiguration('robot_model'))
    base_link_frame = LaunchConfiguration('base_link_frame', default='base_link')
    show_gripper = LaunchConfiguration('show_gripper', default='true')
    use_world_frame = LaunchConfiguration('use_world_frame', default='true')
    external_urdf_loc = LaunchConfiguration('external_urdf_loc', default='')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    load_gazebo_configs = LaunchConfiguration('load_gazebo_configs', default='false')
    use_joint_pub = LaunchConfiguration('use_joint_pub', default='false')
    use_joint_pub_gui = LaunchConfiguration('use_joint_pub_gui', default='false')
    rate = LaunchConfiguration('rate', default='10')
    rvizconfig = LaunchConfiguration('rvizconfig', default=os.path.join(pkg_share, 'rviz', 'uxarm_description.rviz'))

    robot_description = Command([
        'xacro ', pkg_share, '/urdf/', robot_model, '.urdf.xacro',
        ' robot_name:=', robot_name,
        ' base_link_frame:=', base_link_frame,
        ' show_gripper:=', show_gripper,
        ' use_world_frame:=', use_world_frame,
        ' external_urdf_loc:=', external_urdf_loc,
        ' load_gazebo_configs:=', load_gazebo_configs,
    ])

    return LaunchDescription([
        DeclareLaunchArgument('robot_model', default_value='uxarm6'),
        DeclareLaunchArgument('robot_name', default_value=LaunchConfiguration('robot_model')),
        DeclareLaunchArgument('base_link_frame', default_value='base_link'),
        DeclareLaunchArgument('show_gripper', default_value='true'),
        DeclareLaunchArgument('use_world_frame', default_value='true'),
        DeclareLaunchArgument('external_urdf_loc', default_value=''),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('load_gazebo_configs', default_value='false'),
        DeclareLaunchArgument('use_joint_pub', default_value='false'),
        DeclareLaunchArgument('use_joint_pub_gui', default_value='false'),
        DeclareLaunchArgument('rate', default_value='10'),
        DeclareLaunchArgument('rvizconfig', default_value=os.path.join(pkg_share, 'rviz', 'uxarm_description.rviz')),

        Node(
            condition=IfCondition(use_joint_pub),
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            namespace=robot_name,
            parameters=[{'rate': rate}],
        ),
        Node(
            condition=IfCondition(use_joint_pub_gui),
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            namespace=robot_name,
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=robot_name,
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            condition=IfCondition(use_rviz),
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            namespace=robot_name,
            arguments=['-d', rvizconfig],
        ),
    ])
