import argparse
import sys
from pathlib import Path

if Path(__file__).resolve().parent not in sys.path:
    sys.path.insert(0, str(Path(__file__).resolve().parent))

from detector import DEFAULT_WEIGHTS, run_detection


def _cli():
    p = argparse.ArgumentParser()
    p.add_argument("--source", required=True, help="Image path, video path, or 0 for webcam")
    p.add_argument("--weights", default=str(DEFAULT_WEIGHTS))
    p.add_argument("--conf", type=float, default=0.35)
    p.add_argument("--iou", type=float, default=0.45)
    p.add_argument("--imgsz", type=int, default=640)
    p.add_argument("--show", action="store_true")
    p.add_argument("--device", default="")
    args = p.parse_args()

    from ultralytics import YOLO
    model = YOLO(args.weights)
    source = int(args.source) if args.source.isdigit() else args.source
    stream = isinstance(source, int) or Path(str(source)).suffix.lower() in (".mp4", ".avi", ".mov", ".mkv")
    for r in model.predict(source=source, conf=args.conf, iou=args.iou, imgsz=args.imgsz,
                           show=args.show, save=True, device=args.device or None, stream=stream):
        if r.boxes is not None and len(r.boxes):
            for box in r.boxes:
                print(model.names[int(box.cls[0])], float(box.conf[0]), box.xyxy[0].tolist())


def _ros_node(args=None):
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
    from cv_bridge import CvBridge

    class TrashDetectionNode(Node):
        def __init__(self):
            super().__init__("trash_detection_node")
            self.declare_parameter("image_topic", "/camera/color/image_raw")
            self.declare_parameter("detections_topic", "/trash_detections")
            self.declare_parameter("weights", "")
            self.declare_parameter("conf", 0.35)
            self.declare_parameter("frame_id", "camera_color_optical_frame")
            topic = self.get_parameter("image_topic").value
            out_topic = self.get_parameter("detections_topic").value
            self._weights = self.get_parameter("weights").value or None
            self._conf = self.get_parameter("conf").value
            self._frame_id = self.get_parameter("frame_id").value
            self._model = None
            self._bridge = CvBridge()
            self._sub = self.create_subscription(Image, topic, self._cb, 10)
            self._pub = self.create_publisher(Detection2DArray, out_topic, 10)

        def _cb(self, msg):
            try:
                img = self._bridge.imgmsg_to_cv2(msg, "bgr8")
            except Exception as e:
                self.get_logger().warn(str(e))
                return
            dets, model = run_detection(img, weights=self._weights, conf=self._conf, model=self._model)
            if self._model is None and model is not None:
                self._model = model
            out = Detection2DArray()
            out.header = msg.header
            out.header.frame_id = out.header.frame_id or self._frame_id
            for d in dets:
                det = Detection2D()
                det.header = out.header
                x1, y1, x2, y2 = d["bbox_xyxy"]
                det.bbox.center.x = (x1 + x2) / 2.0
                det.bbox.center.y = (y1 + y2) / 2.0
                det.bbox.size_x = float(x2 - x1)
                det.bbox.size_y = float(y2 - y1)
                h = ObjectHypothesisWithPose()
                h.hypothesis.class_id = str(d["class_id"])
                h.hypothesis.score = d["confidence"]
                det.results.append(h)
                out.detections.append(det)
            self._pub.publish(out)

    rclpy.init(args=args)
    node = TrashDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


def main():
    if "--ros" in sys.argv:
        sys.argv.remove("--ros")
        _ros_node()
    else:
        _cli()


if __name__ == "__main__":
    main()