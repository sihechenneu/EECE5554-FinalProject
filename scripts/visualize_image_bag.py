#!/usr/bin/env python3
"""
ROS2 node to visualize multiple image topics at once (RGB, grayscale, and stereo/depth). Use while playing a bag:
  ros2 bag play <path>   # in another terminal
  python visualize_rgb_topics.py --topics /oakd/left/image_raw /oakd/right/image_raw /oakd/rgb/image_raw /oakd/stereo/image_raw

Stereo topic (/oakd/stereo/image_raw): 16-bit depth is shown as JET colormap (clipped 0--stereo-max-depth mm);
8-bit is shown as grayscale. Same method as oakd_visualizer.py.

Displays all topics in a single window (grid). Press [Q] to quit.
Requirements: rclpy, sensor_msgs, numpy, opencv-python
"""

import argparse
import sys

import numpy as np
import cv2


def image_msg_to_bgr(msg, topic: str = "", rgb_topic_as_rgb: bool = False, stereo_max_depth_mm: float = 2000.0):
    """Convert sensor_msgs/Image to BGR numpy array for display. Returns None if unsupported.
    - Grayscale (left/right): MONO8 -> BGR grayscale.
    - RGB topic: by default treated as BGR; use rgb_topic_as_rgb=True if red/blue swapped.
    - Stereo topic (name contains 'stereo'): 16-bit depth -> clip to stereo_max_depth_mm, scale, JET colormap;
      8-bit -> grayscale. Same as oakd_visualizer.py.
    """
    try:
        from sensor_msgs.msg import Image
    except ImportError:
        print("Source ROS2: source /opt/ros/jazzy/setup.bash", file=sys.stderr)
        sys.exit(1)
    if not isinstance(msg, Image):
        return None
    h, w = msg.height, msg.width
    enc = msg.encoding.upper().replace(" ", "")
    is_stereo_topic = "stereo" in topic.lower()

    if enc in ("8UC1", "8U_C1", "MONO8"):
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w)
        return cv2.cvtColor(arr, cv2.COLOR_GRAY2BGR)

    if enc in ("16UC1", "16U_C1") and is_stereo_topic:
        arr = np.frombuffer(msg.data, dtype=np.uint16).reshape(h, w)
        arr = np.clip(arr, 0, int(stereo_max_depth_mm))
        arr_8u = np.uint8((arr.astype(np.float32) / float(stereo_max_depth_mm)) * 255.0)
        return cv2.applyColorMap(arr_8u, cv2.COLORMAP_JET)

    if enc in ("8UC3", "8U_C3", "RGB8", "BGR8"):
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 3)
        is_rgb_topic = "rgb" in topic.lower()
        if is_rgb_topic:
            if rgb_topic_as_rgb:
                arr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
        elif "BGR" not in msg.encoding:
            arr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
        return arr
    return None


def short_name(topic: str, max_len: int = 28) -> str:
    """Shorten topic for overlay label (e.g. /oakd/left/image_raw -> left/image_raw)."""
    if len(topic) <= max_len:
        return topic
    parts = topic.rsplit("/", 2)
    if len(parts) >= 2:
        return parts[-2] + "/" + parts[-1]
    return topic[-max_len:]


def main():
    parser = argparse.ArgumentParser(description="Visualize multiple RGB image topics at once (e.g. while playing a bag).")
    parser.add_argument("--topics", "-t", type=str, nargs="+", required=True,
                        help="Image topics, e.g. /oakd/left/image_raw /oakd/right/image_raw /oakd/rgb/image_raw")
    parser.add_argument("--max-size", type=int, default=400, help="Max width/height per cell (default 400)")
    parser.add_argument("--rgb-as-rgb", action="store_true",
                        help="Treat the rgb topic as RGB (convert to BGR for display). Use if rgb still has red/blue swapped.")
    parser.add_argument("--stereo-max-depth", type=float, default=2000.0,
                        help="For stereo topic: max depth in mm for colormap (default 2000). Clipped above this.")
    parser.add_argument("--yolo-topic", type=str, default="",
                        help="Optional. Topic name for yolov8_msgs/Yolov8Inference to draw bounding boxes.")
    args = parser.parse_args()

    try:
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import Image
    except ImportError:
        print("rclpy not found. Source ROS2: source /opt/ros/jazzy/setup.bash", file=sys.stderr)
        sys.exit(1)

    topics = [t.strip() for t in args.topics if t.strip()]
    if not topics:
        print("Provide at least one topic via --topics.", file=sys.stderr)
        sys.exit(1)

    # latest[topic] = (bgr_array, timestamp_ns)
    latest = {t: None for t in topics}
    latest_yolo = None
    rgb_as_rgb = getattr(args, "rgb_as_rgb", False)
    stereo_max_depth_mm = getattr(args, "stereo_max_depth", 2000.0)
    yolo_topic = getattr(args, "yolo_topic", "")
    
    YoloMsgClass = None
    if yolo_topic:
        try:
            from yolov8_msgs.msg import Yolov8Inference
            YoloMsgClass = Yolov8Inference
        except ImportError:
            pass

        try:
            from vision_msgs.msg import Detection3DArray
            YoloMsgClass3D = Detection3DArray
            if YoloMsgClass is None or "spatial" in yolo_topic.lower() or "vision" in yolo_topic.lower():
                YoloMsgClass = YoloMsgClass3D
        except ImportError:
            pass

        if YoloMsgClass is None:
            print("Message types for yolo_topic not found. Make sure vision_msgs is installed.", file=sys.stderr)
            sys.exit(1)

    class MultiImageSubscriber(Node):
        def __init__(self):
            super().__init__("visualize_rgb_topics_node")
            for topic in topics:
                self.create_subscription(Image, topic, lambda msg, t=topic: self.callback(msg, t), 1)
            
            if yolo_topic:
                self.create_subscription(YoloMsgClass, yolo_topic, self.yolo_callback, 1)

        def yolo_callback(self, msg):
            nonlocal latest_yolo
            latest_yolo = msg

        def callback(self, msg, topic):
            bgr = image_msg_to_bgr(msg, topic, rgb_topic_as_rgb=rgb_as_rgb, stereo_max_depth_mm=stereo_max_depth_mm)
            if bgr is not None:
                stamp = getattr(msg.header, "stamp", None)
                ns = 0
                if stamp is not None:
                    ns = getattr(stamp, "sec", 0) * 10**9 + getattr(stamp, "nanosec", 0)
                latest[topic] = (bgr, ns)

    rclpy.init()
    node = MultiImageSubscriber()
    print(f"Subscribed to {len(topics)} topic(s). Play the bag: ros2 bag play <path>")
    print("Press [Q] to quit.")

    max_side = args.max_size
    win_name = "RGB topics [Q] quit"

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.03)
            ready = [t for t in topics if latest[t] is not None]
            if not ready:
                key = cv2.waitKey(50)
                if key != -1 and (key & 0xFF) == ord("q"):
                    break
                continue

            cell_size = max_side
            cells = []
            for topic in topics:
                if latest[topic] is None:
                    cells.append(np.zeros((cell_size, cell_size, 3), dtype=np.uint8))
                    continue
                bgr_orig, _ = latest[topic]
                bgr = bgr_orig.copy()
                
                # Draw YOLO bounding boxes on the original resolution before resize
                if yolo_topic and latest_yolo is not None:
                    if hasattr(latest_yolo, 'yolov8_inference'):
                        for det in latest_yolo.yolov8_inference:
                            x1, y1, x2, y2 = map(int, det.coordinates)
                            cv2.rectangle(bgr, (x1, y1), (x2, y2), (0, 0, 255), 2)
                            cv2.putText(bgr, det.class_name, (x1, max(15, y1 - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                    elif hasattr(latest_yolo, 'detections'):
                        for det in latest_yolo.detections:
                            class_id = ""
                            if len(det.results) > 0:
                                class_id = getattr(det.results[0].hypothesis, 'class_id', "")
                                if not class_id and hasattr(det.results[0], 'id'):
                                    class_id = str(det.results[0].id)
                            
                            cx = det.bbox.center.position.x
                            cy = det.bbox.center.position.y
                            sx = det.bbox.size.x
                            sy = det.bbox.size.y
                            
                            x1 = cx - sx / 2
                            y1 = cy - sy / 2
                            x2 = cx + sx / 2
                            y2 = cy + sy / 2
                            
                            h_img, w_img = bgr.shape[:2]
                            if sx <= 1.05 and sy <= 1.05 and cx <= 1.05 and cy <= 1.05:
                                x1 *= w_img
                                y1 *= h_img
                                x2 *= w_img
                                y2 *= h_img
                            else:
                                # Fix Geometry Miscalculation:
                                # The VPU publishes boxes in the stretched square NN space (e.g., 320x320).
                                # The source image is rectangular (e.g., 320w x 240h).
                                # To map it perfectly back, we un-stretch the coordinates!
                                # Assuming the width is 1:1 (320 -> 320), the NN scale is the image width.
                                nn_scale = max(w_img, h_img) 
                                x1 = (x1 / nn_scale) * w_img
                                x2 = (x2 / nn_scale) * w_img
                                y1 = (y1 / nn_scale) * h_img
                                y2 = (y2 / nn_scale) * h_img
                            
                            x1, y1, x2, y2 = map(int, [x1, y1, x2, y2])

                            cv2.rectangle(bgr, (x1, y1), (x2, y2), (0, 255, 255), 2)
                            cv2.putText(bgr, class_id, (x1, max(15, y1 - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                        
                h, w = bgr.shape[:2]
                if max(h, w) > cell_size:
                    scale = cell_size / max(h, w)
                    bgr = cv2.resize(bgr, (int(w * scale), int(h * scale)), interpolation=cv2.INTER_AREA)
                    h, w = bgr.shape[:2]
                # Pad to cell_size x cell_size (center)
                pad_h = (cell_size - h) // 2
                pad_w = (cell_size - w) // 2
                padded = np.zeros((cell_size, cell_size, 3), dtype=np.uint8)
                padded[pad_h : pad_h + h, pad_w : pad_w + w] = bgr
                label = short_name(topic)
                cv2.putText(padded, label, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cells.append(padded)

            row = np.hstack(cells)
            cv2.imshow(win_name, row)
            key = cv2.waitKey(1)
            if key != -1 and (key & 0xFF) == ord("q"):
                break
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()
    print("Done.")


if __name__ == "__main__":
    main()
