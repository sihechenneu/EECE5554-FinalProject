#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory("trash_detection_2d")
    config_dir = os.path.join(pkg_dir, "config")
    
    return LaunchDescription([
        DeclareLaunchArgument("image_topic", default_value="/camera/color/image_raw"),
        DeclareLaunchArgument("inference_topic", default_value="/yolo_trash/inference"),
        DeclareLaunchArgument("weights", default_value=""),
        DeclareLaunchArgument("conf", default_value="0.35"),
        DeclareLaunchArgument("iou", default_value="0.45"),
        DeclareLaunchArgument("imgsz", default_value="640"),
        DeclareLaunchArgument("frame_id", default_value="camera_color_optical_frame"),
        Node(
            package="trash_detection_2d",
            executable="yolo_trash_subscriber",
            name="yolo_trash_subscriber",
            output="screen",
            parameters=[{
                "image_topic": LaunchConfiguration("image_topic"),
                "inference_topic": LaunchConfiguration("inference_topic"),
                "weights": LaunchConfiguration("weights"),
                "conf": LaunchConfiguration("conf"),
                "iou": LaunchConfiguration("iou"),
                "imgsz": LaunchConfiguration("imgsz"),
                "frame_id": LaunchConfiguration("frame_id"),
                "nn_config_path": os.path.join(config_dir, "yolov8.json")
            }],
        ),
    ])
