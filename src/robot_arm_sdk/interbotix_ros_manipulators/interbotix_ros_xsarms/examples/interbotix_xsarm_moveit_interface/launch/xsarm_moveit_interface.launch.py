#!/usr/bin/env python3
"""
ROS 2 launch: description + MoveIt 2 move_group + moveit_interface for xsarm.
Use this so /<robot_name>/moveit_plan is available (e.g. for move_to_point.sh).

Requires: MoveIt 2 installed (sudo apt install ros-jazzy-moveit).
"""
import os
import subprocess
import tempfile
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _run_xacro(pkg_path, xacro_file, **kwargs):
    args = ['xacro', os.path.join(pkg_path, xacro_file)]
    for k, v in kwargs.items():
        args.append('{}:={}'.format(k, v))
    return subprocess.check_output(args, text=True)


def _launch_move_group_and_interface(context, *args, **kwargs):
    robot_model = context.perform_substitution(LaunchConfiguration('robot_model', default='px100'))
    robot_name = context.perform_substitution(LaunchConfiguration('robot_name', default='px100'))
    pkg_descriptions = get_package_share_directory('interbotix_xsarm_descriptions')
    pkg_moveit = get_package_share_directory('interbotix_xsarm_moveit')
    base_link_frame = context.perform_substitution(LaunchConfiguration('base_link_frame', default='base_link'))

    robot_description = _run_xacro(
        pkg_descriptions, 'urdf/' + robot_model + '.urdf.xacro',
        robot_name=robot_name, base_link_frame=base_link_frame,
        show_ar_tag='false', show_gripper_bar='true', show_gripper_fingers='true',
        use_world_frame='true', external_urdf_loc='', load_gazebo_configs='false')
    robot_description_semantic = _run_xacro(
        pkg_moveit, 'config/srdf/' + robot_model + '.srdf.xacro',
        robot_name=robot_name, base_link_frame=base_link_frame,
        show_ar_tag='false', external_srdf_loc='')

    joint_limits_path = os.path.join(pkg_moveit, 'config', 'joint_limits', '5dof_joint_limits.yaml')
    kinematics_path = os.path.join(pkg_moveit, 'config', 'kinematics.yaml')
    ompl_path = os.path.join(pkg_moveit, 'config', 'ompl_planning.yaml')
    fake_controllers_path = os.path.join(pkg_moveit, 'config', 'fake_controllers', '5dof_controllers.yaml')

    with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as f:
        f.write('robot_description: |\n')
        for line in robot_description.splitlines():
            f.write('  ' + line + '\n')
        f.write('robot_description_semantic: |\n')
        for line in robot_description_semantic.splitlines():
            f.write('  ' + line + '\n')
        params_file = f.name

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        name='move_group',
        namespace=robot_name,
        output='screen',
        parameters=[
            params_file,
            joint_limits_path,
            kinematics_path,
            ompl_path,
            fake_controllers_path,
            {'use_sim_time': False},
        ],
    )
    moveit_interface_node = Node(
        package='interbotix_moveit_interface',
        executable='moveit_interface',
        name='moveit_interface',
        namespace=robot_name,
        output='screen',
    )
    return [move_group_node, moveit_interface_node]


def generate_launch_description():
    pkg_descriptions = get_package_share_directory('interbotix_xsarm_descriptions')
    pkg_control = get_package_share_directory('interbotix_xsarm_control')
    description_launch = os.path.join(pkg_descriptions, 'launch', 'xsarm_description.launch.py')
    control_launch = os.path.join(pkg_control, 'launch', 'xsarm_control.launch.py')

    robot_model = LaunchConfiguration('robot_model', default='px100')
    robot_name = LaunchConfiguration('robot_name', default=LaunchConfiguration('robot_model'))
    use_sim_control = LaunchConfiguration('use_sim_control', default='true')

    return LaunchDescription([
        DeclareLaunchArgument('robot_model', default_value='px100'),
        DeclareLaunchArgument('robot_name', default_value=LaunchConfiguration('robot_model')),
        DeclareLaunchArgument('base_link_frame', default_value='base_link'),
        DeclareLaunchArgument('use_sim_control', default_value='true',
                              description='If true, include xsarm_control with use_sim (xs_sdk_sim).'),
        DeclareLaunchArgument('dof', default_value='5'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(control_launch),
            condition=IfCondition(use_sim_control),
            launch_arguments={
                'robot_model': robot_model,
                'robot_name': robot_name,
                'use_rviz': 'false',
                'use_sim': 'true',
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(description_launch),
            condition=UnlessCondition(use_sim_control),
            launch_arguments={
                'robot_model': robot_model,
                'robot_name': robot_name,
                'use_rviz': 'false',
            }.items(),
        ),
        OpaqueFunction(function=_launch_move_group_and_interface),
    ])
