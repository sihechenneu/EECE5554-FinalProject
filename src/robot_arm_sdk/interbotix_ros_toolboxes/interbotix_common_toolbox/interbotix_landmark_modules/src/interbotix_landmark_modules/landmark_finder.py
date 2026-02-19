#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TransformStamped
from interbotix_common_modules.angle_manipulation import quaternion_is_valid
from interbotix_landmark_modules.landmark import Landmark, LandmarkCollection

try:
    from interbotix_perception_modules.apriltag import InterbotixAprilTagInterface as AprilTag
    HAS_APRILTAG = True
except ImportError:
    HAS_APRILTAG = False


def main(args=None):
    rclpy.init(args=args)
    node = Node('landmark_finder')

    if not HAS_APRILTAG:
        node.get_logger().error(
            'landmark_finder requires interbotix_perception_modules (ROS 2). '
            'Ensure it is converted and built.')
        rclpy.shutdown()
        return

    node.declare_parameter('apriltag_ns', 'apriltag')
    node.declare_parameter('landmark_ns', 'landmarks')
    try:
        pkg_path = get_package_share_directory('interbotix_landmark_modules')
    except Exception:
        pkg_path = ''
    default_config = pkg_path + '/landmarks/landmarks.yaml' if pkg_path else 'landmarks.yaml'
    node.declare_parameter('landmark_config', default_config)
    node.declare_parameter('obs_frame', 'camera_color_optical_frame')
    node.declare_parameter('fixed_frame', 'landmarks')

    lm_config_filepath = node.get_parameter('landmark_config').value
    obs_frame = node.get_parameter('obs_frame').value
    fixed_frame = node.get_parameter('fixed_frame').value
    apriltag_ns = node.get_parameter('apriltag_ns').value

    landmarks = LandmarkCollection(
        landmarks={},
        obs_frame=obs_frame,
        fixed_frame=fixed_frame,
        ros_on=True,
        node=node)
    landmarks.load(lm_config_filepath)
    valid_tags = [l.get_id() for l in landmarks.data.values()]
    apriltag = AprilTag(apriltag_ns=apriltag_ns, init_node=False)
    apriltag.set_valid_tags(valid_tags)

    def timer_callback():
        poses_wrt_cam, tag_ids = apriltag.find_pose_id()
        if len(tag_ids) > 0:
            for i in range(len(tag_ids)):
                if quaternion_is_valid(poses_wrt_cam[i].orientation):
                    landmarks.data[tag_ids[i]].set_tf_wrt_cam(poses_wrt_cam[i])
                    landmarks.data[tag_ids[i]].update_tfs(
                        parent_old=obs_frame,
                        parent_new=fixed_frame)
            landmarks.pub_tfs(tag_ids)
            rclpy.spin_once(node, timeout_sec=1.0)
            landmarks.pub_markers(tag_ids)

    timer = node.create_timer(2.0, timer_callback)
    node.get_logger().info('Initialized Landmark Finder')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
