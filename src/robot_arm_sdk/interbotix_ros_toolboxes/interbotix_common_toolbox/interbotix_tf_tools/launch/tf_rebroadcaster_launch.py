#!/usr/bin/env python3

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('interbotix_tf_tools')
    default_config = os.path.join(pkg_share, 'config', 'tf_rebroadcaster.yaml')

    topic_to_arg = DeclareLaunchArgument(
        'topic_to',
        default_value='',
        description='Topic to publish rebroadcast TFs to'
    )
    topic_from_arg = DeclareLaunchArgument(
        'topic_from',
        default_value='',
        description='Topic to subscribe to for incoming TFs'
    )
    tf_rebroadcaster_config_arg = DeclareLaunchArgument(
        'tf_rebroadcaster_config',
        default_value=default_config,
        description='Path to tf_rebroadcaster config YAML'
    )
    use_incoming_time_arg = DeclareLaunchArgument(
        'use_incoming_time',
        default_value='true',
        description='Use timestamp from incoming TF messages'
    )

    tf_rebroadcaster_node = Node(
        package='interbotix_tf_tools',
        executable='tf_rebroadcaster',
        name='tf_rebroadcaster',
        output='screen',
        parameters=[{
            'topic_to': LaunchConfiguration('topic_to'),
            'topic_from': LaunchConfiguration('topic_from'),
            'filepath_config': LaunchConfiguration('tf_rebroadcaster_config'),
            'use_incoming_time': LaunchConfiguration('use_incoming_time'),
        }]
    )

    return LaunchDescription([
        topic_to_arg,
        topic_from_arg,
        tf_rebroadcaster_config_arg,
        use_incoming_time_arg,
        tf_rebroadcaster_node,
    ])
