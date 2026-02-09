#!/usr/bin/env python3
"""Launch xsarm in Gazebo (description + empty world + spawn). Controller loading depends on gazebo_ros/gazebo_ros_control."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_gazebo = get_package_share_directory('interbotix_xsarm_gazebo')
    pkg_descriptions = get_package_share_directory('interbotix_xsarm_descriptions')

    robot_model = LaunchConfiguration('robot_model', default='wx200')
    robot_name = LaunchConfiguration('robot_name', default=robot_model)
    base_link_frame = LaunchConfiguration('base_link_frame', default='base_link')
    show_ar_tag = LaunchConfiguration('show_ar_tag', default='false')
    show_gripper_bar = LaunchConfiguration('show_gripper_bar', default='true')
    show_gripper_fingers = LaunchConfiguration('show_gripper_fingers', default='true')
    use_world_frame = LaunchConfiguration('use_world_frame', default='true')
    external_urdf_loc = LaunchConfiguration('external_urdf_loc', default='')
    use_rviz = LaunchConfiguration('use_rviz', default='false')
    world_name = LaunchConfiguration('world_name', default=os.path.join(pkg_gazebo, 'worlds', 'xsarm_gazebo.world'))
    gui = LaunchConfiguration('gui', default='true')
    paused = LaunchConfiguration('paused', default='true')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_trajectory_controllers = LaunchConfiguration('use_trajectory_controllers', default='false')
    use_position_controllers = LaunchConfiguration('use_position_controllers', default='false')

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
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument('world_name', default_value=os.path.join(pkg_gazebo, 'worlds', 'xsarm_gazebo.world')),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('paused', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('use_trajectory_controllers', default_value='false'),
        DeclareLaunchArgument('use_position_controllers', default_value='false'),
        SetEnvironmentVariable(name='GAZEBO_RESOURCE_PATH', value=pkg_gazebo),
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
                'load_gazebo_configs': 'true',
            }.items(),
        ),
        # Gazebo empty_world and spawn_entity are typically from gazebo_ros; include if available.
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')),
        #     launch_arguments={'world': world_name, 'gui': gui, 'paused': paused}.items(),
        # ),
        # Node(
        #     package='gazebo_ros',
        #     executable='spawn_entity',
        #     name='urdf_spawner',
        #     namespace=robot_name,
        #     arguments=['-urdf', '-model', robot_model, '-param', 'robot_description'],
        # ),
    ])
