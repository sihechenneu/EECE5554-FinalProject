#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('interbotix_xsarm_descriptions')
    rvizconfig = LaunchConfiguration('rvizconfig', default=os.path.join(pkg_share, 'rviz', 'xsarm_description.rviz'))
    robot_name = LaunchConfiguration('robot_name', default='')

    return LaunchDescription([
        DeclareLaunchArgument('robot_name', default_value='', description='Robot namespace'),
        DeclareLaunchArgument('rvizconfig', default_value=os.path.join(pkg_share, 'rviz', 'xsarm_description.rviz')),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            namespace=robot_name,
            arguments=['-d', rvizconfig],
        ),
    ])
