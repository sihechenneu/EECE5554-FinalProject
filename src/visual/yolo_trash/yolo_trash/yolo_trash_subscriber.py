import time
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from yolov8_msgs.msg import Yolov8Inference, InferenceResult

from yolo_trash.detector import run_detection


class YoloTrashSubscriber(Node):
    def __init__(self):
        super().__init__("yolo_trash_subscriber")
        self.declare_parameter("image_topic", "/oakd/rgb/preview/image_raw")
        self.declare_parameter("inference_topic", "/yolo_trash/inference")
        self.declare_parameter("weights", "")
        self.declare_parameter("conf", 0.35)
        self.declare_parameter("iou", 0.45)
        self.declare_parameter("imgsz", 640)
        self.declare_parameter("device", "")
        self.declare_parameter("frame_id", "camera_color_optical_frame")

        image_topic = self.get_parameter("image_topic").value
        inference_topic = self.get_parameter("inference_topic").value
        self._weights = self.get_parameter("weights").value or None
        self._conf = self.get_parameter("conf").value
        self._iou = self.get_parameter("iou").value
        self._imgsz = self.get_parameter("imgsz").value
        self._device = self.get_parameter("device").value or None
        self._frame_id = self.get_parameter("frame_id").value

        self._bridge = CvBridge()
        self._model = None
        
        # Variables for thread decoupling
        self._latest_msg = None
        self._lock = threading.Lock()
        
        self._sub = self.create_subscription(Image, image_topic, self._image_cb, 10)
        self._pub = self.create_publisher(Yolov8Inference, inference_topic, 10)
        
        # Start background inference thread
        self._inference_thread = threading.Thread(target=self._inference_loop, daemon=True)
        self._inference_thread.start()

    def _image_cb(self, msg):
        # Cache the newest frame and immediately return (takes ~0.001s)
        with self._lock:
            self._latest_msg = msg

    def _inference_loop(self):
        while rclpy.ok():
            msg = None
            with self._lock:
                if self._latest_msg is not None:
                    msg = self._latest_msg
                    self._latest_msg = None  # Consume the message

            if msg is None:
                time.sleep(0.01)
                continue

            try:
                cv_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            except Exception as e:
                self.get_logger().warn(str(e))
                continue

            detections, model = run_detection(
                cv_image,
                weights=self._weights,
                conf=self._conf,
                iou=self._iou,
                imgsz=self._imgsz,
                device=self._device,
                model=self._model,
            )
            
            if self._model is None and model is not None:
                self._model = model
                
            out = Yolov8Inference()
            out.header = msg.header
            if not out.header.frame_id:
                out.header.frame_id = self._frame_id
                
            for d in detections:
                res = InferenceResult()
                res.class_name = d["class_name"]
                res.coordinates = list(map(float, d["bbox_xyxy"]))
                out.yolov8_inference.append(res)
                
            self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = YoloTrashSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
