#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='spatial_calculator',
            executable='spatial_calculator',
            name='spatial_calculator',
            output='screen',
            parameters=[{
                # Depth image published by OAK-D stereo pipeline
                'depth_topic': '/oakd/stereo/image_raw',
                # Camera intrinsics for the aligned depth frame
                # NOTE: use stereo/camera_info, not rgb/camera_info —
                # depthai-ros suppresses rgb camera_info when i_publish_topic: false
                'info_topic': '/oakd/stereo/camera_info',
                # On-device YOLO spatial detections (i_nn_type: spatial in oakd_yolo.yaml)
                'yolo_topic': '/oakd/nn/spatial_detections',
                'yolo_type': 'Detection3DArray',
                'output_topic': '/yolo/spatial_detections',
            }],
        ),
    ])
