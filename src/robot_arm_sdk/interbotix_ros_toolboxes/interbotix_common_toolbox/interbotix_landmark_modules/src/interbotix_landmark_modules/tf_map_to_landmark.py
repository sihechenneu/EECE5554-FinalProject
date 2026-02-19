#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import TransformStamped


def main(args=None):
    rclpy.init(args=args)
    node = Node('tf_map_to_landmark')
    node.declare_parameter('original_frame', 'map')
    node.declare_parameter('fixed_frame', 'landmarks')
    original_frame = node.get_parameter('original_frame').value
    fixed_frame = node.get_parameter('fixed_frame').value

    pub = node.create_publisher(TransformStamped, 'static_transforms', 10)

    tf = TransformStamped()
    tf.header.stamp = node.get_clock().now().to_msg()
    tf.header.frame_id = original_frame
    tf.child_frame_id = fixed_frame
    tf.transform.translation.x = 0.0
    tf.transform.translation.y = 0.0
    tf.transform.translation.z = 0.0
    tf.transform.rotation.x = 0.0
    tf.transform.rotation.y = 0.0
    tf.transform.rotation.z = 0.0
    tf.transform.rotation.w = 1.0

    pub.publish(tf)
    node.get_logger().info(
        '[tf_map_to_landmark] Broadcasted static transformation between "%s" and "%s".'
        % (original_frame, fixed_frame))

    rclpy.spin_once(node, timeout_sec=100.0)
    node.get_logger().info('[tf_map_to_landmark] Shutting down...')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
