#!/usr/bin/env python3
"""
ROS2 Jazzy node: subscribe to RGB image topic (e.g. from bag or camera),
run YOLO trash detection, publish vision_msgs/Detection2DArray.
"""

import sys
from pathlib import Path

# Ensure 2D-detection modules are importable when run via ros2 run (installed or from source)
_SCRIPT_DIR = Path(__file__).resolve().parent
if _SCRIPT_DIR not in sys.path:
    sys.path.insert(0, str(_SCRIPT_DIR))

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from geometry_msgs.msg import Pose2D
from cv_bridge import CvBridge

from detector import run_detection


class TrashDetectionNode(Node):
    """Subscribes to RGB image, runs YOLO trash detection, publishes Detection2DArray."""

    def __init__(self):
        super().__init__("trash_detection_node")
        self.declare_parameter("image_topic", "/camera/color/image_raw")
        self.declare_parameter("detections_topic", "/trash_detections")
        self.declare_parameter("image_out_topic", "")  # optional annotated image
        self.declare_parameter("weights", "")
        self.declare_parameter("conf", 0.35)
        self.declare_parameter("iou", 0.45)
        self.declare_parameter("imgsz", 640)
        self.declare_parameter("device", "")
        self.declare_parameter("frame_id", "camera_color_optical_frame")

        image_topic = self.get_parameter("image_topic").value
        detections_topic = self.get_parameter("detections_topic").value
        image_out_topic = self.get_parameter("image_out_topic").value
        weights = self.get_parameter("weights").value or None
        self.conf = self.get_parameter("conf").value
        self.iou = self.get_parameter("iou").value
        self.imgsz = self.get_parameter("imgsz").value
        self.device = self.get_parameter("device").value or None
        self.frame_id = self.get_parameter("frame_id").value

        self._bridge = CvBridge()
        self._model = None  # cache YOLO model after first run

        self._sub = self.create_subscription(
            Image,
            image_topic,
            self._image_cb,
            10,
        )
        self._pub = self.create_publisher(Detection2DArray, detections_topic, 10)
        self._pub_image = (
            self.create_publisher(Image, image_out_topic, 10)
            if image_out_topic
            else None
        )

        self.get_logger().info(
            f"Trash detection node: sub {image_topic} -> pub {detections_topic}"
        )

    def _image_cb(self, msg: Image):
        try:
            cv_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"cv_bridge error: {e}")
            return

        detections, model = run_detection(
            cv_image,
            weights=self.get_parameter("weights").value or None,
            conf=self.conf,
            iou=self.iou,
            imgsz=self.imgsz,
            device=self.device,
            model=self._model,
        )
        if self._model is None and model is not None:
            self._model = model

        out = Detection2DArray()
        out.header = msg.header
        if out.header.frame_id == "":
            out.header.frame_id = self.frame_id

        for d in detections:
            det = Detection2D()
            det.header = out.header
            x1, y1, x2, y2 = d["bbox_xyxy"]
            det.bbox.center.x = (x1 + x2) / 2.0
            det.bbox.center.y = (y1 + y2) / 2.0
            det.bbox.center.theta = 0.0
            det.bbox.size_x = float(x2 - x1)
            det.bbox.size_y = float(y2 - y1)
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = str(d["class_id"])
            hyp.hypothesis.score = d["confidence"]
            det.results.append(hyp)
            out.detections.append(det)

        self._pub.publish(out)

        if self._pub_image is not None and model is not None:
            try:
                import cv2
                ann = cv_image.copy()
                for d in detections:
                    x1, y1, x2, y2 = [int(round(x)) for x in d["bbox_xyxy"]]
                    cv2.rectangle(ann, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    label = f"{d['class_name']} {d['confidence']:.2f}"
                    cv2.putText(
                        ann, label, (x1, y1 - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1,
                    )
                out_msg = self._bridge.cv2_to_imgmsg(ann, encoding="bgr8")
                out_msg.header = msg.header
                self._pub_image.publish(out_msg)
            except Exception as e:
                self.get_logger().warn(f"annotated image publish error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = TrashDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
