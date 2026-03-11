#!/usr/bin/env python3
"""
ROS2 node: subscribe to an image topic, run Depth Anything V2 (ONNX), publish monocular depth.

Usage:
  # Terminal 1: play bag or run camera
  ros2 bag play <path> --read-ahead-queue-size 100

  # Terminal 2: run this node (default: /oakd/left/image_raw)
  python3 scripts/depth_anything_v2_ros2.py --topic /oakd/left/image_raw --model /path/to/depth_anything_v2_vitb.onnx

  # Custom output topics. Window shows input (left) and depth colormap (right). Press [Q] to quit.
  python3 scripts/depth_anything_v2_ros2.py --topic /oakd/rgb/image_raw --out-depth /depth/image_raw --out-viz /depth/colormap

Model: Download ONNX from https://github.com/fabio-sim/Depth-Anything-ONNX/releases
  (e.g. depth_anything_v2_vitb.onnx, input 1x3x518x518 -> output 1x518x518).
  Or use dynamic-shape model; H,W must be divisible by 14.

Requirements: rclpy, sensor_msgs, cv_bridge, numpy, opencv-python, onnxruntime
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import cv2

# ImageNet normalization for Depth Anything V2
IMAGENET_MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32)
IMAGENET_STD = np.array([0.229, 0.224, 0.225], dtype=np.float32)

# Default input size for fixed-shape ONNX (Depth-Anything-ONNX 518x518)
DEFAULT_INPUT_HEIGHT = 518
DEFAULT_INPUT_WIDTH = 518


def image_msg_to_bgr(msg) -> np.ndarray | None:
    """Convert sensor_msgs/Image to BGR numpy (H, W, 3). Returns None if unsupported."""
    try:
        from sensor_msgs.msg import Image
    except ImportError:
        return None
    if not hasattr(msg, "encoding"):
        return None
    h, w = msg.height, msg.width
    enc = (getattr(msg, "encoding", None) or "").upper().replace(" ", "")
    if enc in ("8UC1", "8U_C1", "MONO8"):
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w)
        return cv2.cvtColor(arr, cv2.COLOR_GRAY2BGR)
    if enc in ("8UC3", "8U_C3", "BGR8"):
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 3)
        if enc != "BGR8":
            arr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
        return arr
    if enc == "RGB8":
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 3)
        return cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
    return None


def preprocess_bgr_for_onnx(
    bgr: np.ndarray,
    height: int = DEFAULT_INPUT_HEIGHT,
    width: int = DEFAULT_INPUT_WIDTH,
) -> np.ndarray:
    """Resize BGR to (height, width), normalize ImageNet, return NCHW float32."""
    resized = cv2.resize(bgr, (width, height), interpolation=cv2.INTER_LINEAR)
    rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
    x = (rgb.astype(np.float32) / 255.0 - IMAGENET_MEAN) / IMAGENET_STD
    return x.transpose(2, 0, 1)[np.newaxis, ...]  # (1, 3, H, W)


def run_inference_onnx(
    session,
    bgr: np.ndarray,
    input_height: int,
    input_width: int,
    input_name: str | None,
    output_name: str | None,
) -> np.ndarray:
    """Run ONNX session; return depth map (H, W) same spatial size as model output."""
    x = preprocess_bgr_for_onnx(bgr, input_height, input_width)
    if input_name is None:
        input_name = session.get_inputs()[0].name
    if output_name is None:
        output_name = session.get_outputs()[0].name
    out = session.run([output_name], {input_name: x})[0]
    # out shape: (1, H, W) -> (H, W)
    depth = np.squeeze(out, axis=0)
    return depth.astype(np.float32)


def depth_to_colormap(depth: np.ndarray, max_percentile: float = 98.0) -> np.ndarray:
    """Convert float depth to BGR colormap (JET) for visualization."""
    valid = np.isfinite(depth) & (depth > 0)
    if not np.any(valid):
        return cv2.applyColorMap(np.zeros(depth.shape, dtype=np.uint8), cv2.COLORMAP_JET)
    vmax = np.percentile(depth[valid], max_percentile)
    vmin = max(0, np.min(depth[valid]))
    depth_clip = np.clip(depth, vmin, vmax)
    depth_norm = np.uint8(255 * (depth_clip - vmin) / max(vmax - vmin, 1e-6))
    return cv2.applyColorMap(depth_norm, cv2.COLORMAP_JET)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Depth Anything V2 from ROS2 image topic (ONNX)."
    )
    parser.add_argument(
        "--topic", "-t",
        type=str,
        default="/oakd/left/image_raw",
        help="Input image topic (default: /oakd/left/image_raw)",
    )
    parser.add_argument(
        "--model", "-m",
        type=str,
        default=None,
        help="Path to Depth Anything V2 ONNX model (e.g. depth_anything_v2_vitb.onnx)",
    )
    parser.add_argument(
        "--out-depth",
        type=str,
        default="/depth_anything_v2/depth",
        help="Output depth topic (32FC1, default: /depth_anything_v2/depth)",
    )
    parser.add_argument(
        "--input-size",
        type=int,
        nargs=2,
        default=[DEFAULT_INPUT_HEIGHT, DEFAULT_INPUT_WIDTH],
        metavar=("H", "W"),
        help=f"ONNX input height and width (default: {DEFAULT_INPUT_HEIGHT} {DEFAULT_INPUT_WIDTH}; use multiple of 14 for dynamic)",
    )
    parser.add_argument(
        "--max-size",
        type=int,
        default=0,
        help="If set, resize input image so max side is this (reduces lag; 0 = no resize)",
    )
    parser.add_argument(
        "--no-viz",
        action="store_true",
        help="Disable OpenCV window visualization",
    )
    parser.add_argument(
        "--qos-depth",
        type=int,
        default=1,
        help="Subscription QoS depth (default 1 to avoid queue buildup with bag playback)",
    )
    # Spatial Calculator Integration Args
    parser.add_argument("--info-topic", default="/oakd/rgb/camera_info", help="Camera Info topic for intrinsics")
    parser.add_argument("--yolo-topic", default="", help="Optional: YOLO detections topic to calculate 3D spatial points")
    parser.add_argument("--yolo-type", default="Detection2DArray", choices=["Detection2DArray", "Detection3DArray", "Yolov8Inference"])
    parser.add_argument("--out-spatial", default="/yolo/spatial_detections", help="Output Spatial Detections topic")
    parser.add_argument("--stereo-topic", default="", help="Optional: Stereo depth topic to dynamically calibrate the scale factor")
    parser.add_argument("--stereo-max-bad-ratio", type=float, default=0.3, help="Max ratio of bad stereo pixels to consider the depth trusted (default 0.3)")
    args = parser.parse_args()

    try:
        import onnxruntime as ort
    except ImportError:
        print("onnxruntime not found. Install: pip install onnxruntime", file=sys.stderr)
        sys.exit(1)

    try:
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import Image, CameraInfo
        from cv_bridge import CvBridge
        from std_msgs.msg import Header
        from vision_msgs.msg import Detection3DArray, Detection3D, ObjectHypothesisWithPose, Detection2DArray
        import time # Added for profiling latency
    except ImportError as e:
        print("ROS2/cv_bridge not found. Source your workspace and install: pip install opencv-python", file=sys.stderr)
        sys.exit(1)

    model_path = args.model
    if not model_path or not Path(model_path).exists():
        print(
            "Missing or invalid --model. Download ONNX from:\n"
            "  https://github.com/fabio-sim/Depth-Anything-ONNX/releases\n"
            "  e.g. depth_anything_v2_vitb.onnx",
            file=sys.stderr,
        )
        sys.exit(1)

    input_h, input_w = args.input_size[0], args.input_size[1]
    session = ort.InferenceSession(
        model_path,
        providers=["CUDAExecutionProvider", "CPUExecutionProvider"],
    )
    in_spec = session.get_inputs()[0]
    # Allow fixed (e.g. 518,518) or dynamic
    if in_spec.shape[2] is not None and in_spec.shape[3] is not None:
        input_h, input_w = int(in_spec.shape[2]), int(in_spec.shape[3])
    input_name = in_spec.name
    output_name = session.get_outputs()[0].name

    bridge = CvBridge()

    class DepthAnythingV2Node(Node):
        def __init__(self) -> None:
            super().__init__("depth_anything_v2_node")
            self._pub_depth = self.create_publisher(Image, args.out_depth, 1)
            self._sub = self.create_subscription(
                Image,
                args.topic,
                self._callback,
                args.qos_depth,
            )
            self._bridge = bridge
            self._session = session
            self._input_h = input_h
            self._input_w = input_w
            self._input_name = input_name
            self._output_name = output_name
            self._max_size = args.max_size
            self._header: Header | None = None
            self._header: Header | None = None
            self._latest_depth_viz: np.ndarray | None = None
            
            # Spatial Calculator states
            self.camera_info = None
            self.latest_depth_map = None # Clean raw float depth (from AI)
            self.latest_stereo_depth = None # Clean raw float depth (from Stereo)
            self._stereo_max_bad_ratio = args.stereo_max_bad_ratio
            
            # --------------------------------------------------------------------------
            # HARDCODED CALIBRATION LOOKUP TABLE (LUT) FOR DA2 -> METRIC DEPTH
            # --------------------------------------------------------------------------
            # Format: (DA2_Output_Value, Real_Distance_In_Meters)
            # 
            # Step 1: Place an object at exactly 1.0m. Run script. Look at the debug text shown
            #         above the bounding box. The SECOND number is the DA2 Output Value.
            # Step 2: Write that DA2 number down next to `1.0`.
            # Step 3: Move object to 0.5m, 0.3m, 0.1m and record those DA2 values too.
            # Step 4: Add them to this list below!
            # --------------------------------------------------------------------------
            # IMPORTANT: The DA2 values MUST be sorted from smallest to largest! (Because 
            # DA2 values get larger as objects get closer to the camera).
            self.da2_lut = [
                (0.0, 1.197),
                (5.0, 1.483),
            ]
            self._lut_x = np.array([p[0] for p in self.da2_lut])
            self._lut_y = np.array([p[1] for p in self.da2_lut])
            
            self.yolo_boxes = []
            
            # Profiling logic
            self._profile_stats = {
                'count': 0,
                'cv_bridge': 0.0,
                'pre_resize': 0.0,
                'onnx': 0.0,
                'post_resize': 0.0,
                'publish': 0.0,
                'total_fps': 0.0
            }
            self._last_frame_time = None
            
            if args.info_topic and args.yolo_topic:
                self.create_subscription(CameraInfo, args.info_topic, self.info_cb, 1)
                self.pub_spatial = self.create_publisher(Detection3DArray, args.out_spatial, 1)
                
                if args.stereo_topic:
                    self.create_subscription(Image, args.stereo_topic, self.stereo_cb, 1)
                
                # We subscribe to ONLY the specified YOLO format
                if args.yolo_type == 'Yolov8Inference':
                    try:
                        from yolov8_msgs.msg import Yolov8Inference
                        self.create_subscription(Yolov8Inference, args.yolo_topic, self.yolo_cb, 1)
                    except ImportError as e:
                        self.get_logger().error(f"Cannot import Yolov8Inference: {e}")
                elif args.yolo_type == 'Detection3DArray':
                    self.create_subscription(Detection3DArray, args.yolo_topic, self.vision3d_cb, 1)
                else:
                    self.create_subscription(Detection2DArray, args.yolo_topic, self.vision2d_cb, 1)
            
            self.get_logger().info(
                f"Subscribed to {args.topic}, publishing depth to {args.out_depth}"
            )

        def _callback(self, msg: Image) -> None:
            t_start = time.perf_counter()
            if self._last_frame_time is not None:
                self._profile_stats['total_fps'] += (1.0 / (t_start - self._last_frame_time))
            self._last_frame_time = t_start
            
            bgr = image_msg_to_bgr(msg)
            if bgr is None:
                try:
                    bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
                except Exception:
                    self.get_logger().warn("Unsupported image encoding: %s" % getattr(msg, "encoding", ""))
                    return
            t_bridge = time.perf_counter()
            self._profile_stats['cv_bridge'] += (t_bridge - t_start)
            
            if self._max_size > 0:
                h, w = bgr.shape[:2]
                if max(h, w) > self._max_size:
                    scale = self._max_size / max(h, w)
                    bgr = cv2.resize(bgr, (int(w * scale), int(h * scale)), interpolation=cv2.INTER_AREA)
            t_pre = time.perf_counter()
            self._profile_stats['pre_resize'] += (t_pre - t_bridge)
            
            depth = run_inference_onnx(
                self._session,
                bgr,
                self._input_h,
                self._input_w,
                self._input_name,
                self._output_name,
            )
            t_onnx = time.perf_counter()
            self._profile_stats['onnx'] += (t_onnx - t_pre)
            
            # Resize depth back to input image size for publishing
            h_out, w_out = bgr.shape[:2]
            if depth.shape[0] != h_out or depth.shape[1] != w_out:
                depth = cv2.resize(
                    depth,
                    (w_out, h_out),
                    interpolation=cv2.INTER_LINEAR,
                )
            t_post = time.perf_counter()
            self._profile_stats['post_resize'] += (t_post - t_onnx)
            
            self._header = msg.header
            depth_msg = self._bridge.cv2_to_imgmsg(depth.astype(np.float32), encoding="32FC1")
            depth_msg.header = msg.header
            self._pub_depth.publish(depth_msg)
            
            # Save raw depth map for bounding box processing
            self.latest_depth_map = depth
            
            t_pub = time.perf_counter()
            self._profile_stats['publish'] += (t_pub - t_post)
            self._profile_stats['count'] += 1
            
            if self._profile_stats['count'] == 10:
                count = self._profile_stats['count']
                fps = self._profile_stats['total_fps'] / count
                cb = self._profile_stats['cv_bridge'] / count * 1000.0
                pr = self._profile_stats['pre_resize'] / count * 1000.0
                onnx = self._profile_stats['onnx'] / count * 1000.0
                po = self._profile_stats['post_resize'] / count * 1000.0
                pub = self._profile_stats['publish'] / count * 1000.0
                total = cb + pr + onnx + po + pub
                
                self.get_logger().info(
                    f"Profile (10 frames): FPS={fps:.1f} | ONNX={onnx:.1f}ms | Pub={pub:.1f}ms | Bridge={cb:.1f}ms | Resizing={pr+po:.1f}ms | Total={total:.1f}ms"
                )
                
                # Reset counters
                for k in self._profile_stats:
                    self._profile_stats[k] = 0.0

        def stereo_cb(self, msg: Image) -> None:
            enc = (getattr(msg, "encoding", None) or "").upper().replace(" ", "")
            if enc in ("16UC1", "16U_C1", "MONO16", "TYPE_16UC1"):
                arr = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
                self.latest_stereo_depth = arr.astype(np.float32) / 1000.0  # Convert mm to meters
            elif enc in ("32FC1", "TYPE_32FC1"):
                self.latest_stereo_depth = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)

        def info_cb(self, msg: CameraInfo) -> None:
            self.camera_info = msg

        def process_and_publish_spatial(self, boxes, header):
            if self.latest_depth_map is None or self.camera_info is None:
                return
                
            out_msg = Detection3DArray()
            out_msg.header = header
            
            fx = self.camera_info.k[0]
            fy = self.camera_info.k[4]
            cx = self.camera_info.k[2]
            cy = self.camera_info.k[5]
            
            depth_h, depth_w = self.latest_depth_map.shape
            nn_scale = max(depth_w, depth_h)
            
            for b in boxes:
                is_normalized = b.get('is_normalized', False)
                if is_normalized:
                    x1 = int(b['x1'] * depth_w)
                    x2 = int(b['x2'] * depth_w)
                    y1 = int(b['y1'] * depth_h)
                    y2 = int(b['y2'] * depth_h)
                else:
                    s = b.get('nn_size', nn_scale)
                    if s == 0:
                        s = 1.0
                    x1 = int((b['x1'] / s) * depth_w)
                    x2 = int((b['x2'] / s) * depth_w)
                    y1 = int((b['y1'] / s) * depth_h)
                    y2 = int((b['y2'] / s) * depth_h)
                
                x1 = max(0, min(depth_w - 1, x1))
                x2 = max(0, min(depth_w - 1, x2))
                y1 = max(0, min(depth_h - 1, y1))
                y2 = max(0, min(depth_h - 1, y2))
                
                if x2 <= x1 or y2 <= y1:
                    continue
                    
                # Count the valid DA2 floating point output pixels in the box
                ai_depth_roi = self.latest_depth_map[y1:y2, x1:x2]
                valid_ai_depths = ai_depth_roi[ai_depth_roi > 0]
                if len(valid_ai_depths) == 0:
                    continue
                median_ai_depth = float(np.median(valid_ai_depths))
                
                # Compute DA2 distance using 1D Linear Interpolation across the Lookup Table
                z_da2 = float(np.interp(median_ai_depth, self._lut_x, self._lut_y))
                
                # 2. Extract Stereo depth if available
                median_stereo = None
                stereo_trust_ratio = 1.0  # 1.0 means 100% bad data by default
                
                if self.latest_stereo_depth is not None:
                    # Map box to stereo depth map's dimensions
                    stereo_h, stereo_w = self.latest_stereo_depth.shape
                    sx1 = int((x1 / depth_w) * stereo_w)
                    sx2 = int((x2 / depth_w) * stereo_w)
                    sy1 = int((y1 / depth_h) * stereo_h)
                    sy2 = int((y2 / depth_h) * stereo_h)
                    
                    sx1 = max(0, min(stereo_w - 1, sx1))
                    sx2 = max(0, min(stereo_w - 1, sx2))
                    sy1 = max(0, min(stereo_h - 1, sy1))
                    sy2 = max(0, min(stereo_h - 1, sy2))
                    
                    if sx2 > sx1 and sy2 > sy1:
                        stereo_roi = self.latest_stereo_depth[sy1:sy2, sx1:sx2]
                        total_pixels = stereo_roi.size
                        if total_pixels > 0:
                            valid_stereo = stereo_roi[stereo_roi > 0]
                            # Count ratio of invalid pixels (0 or extremely large noise > 10m)
                            bad_pixels = total_pixels - len(valid_stereo) + len(valid_stereo[valid_stereo > 10.0])
                            stereo_trust_ratio = bad_pixels / total_pixels
                            
                            if len(valid_stereo) > 0:
                                median_stereo = float(np.median(valid_stereo))
                            
                # 3. Apply the threshold logic based cleanly on Stereo depth
                z_m = z_da2
                
                if median_stereo is not None:
                    # Trust stereo only if it has enough valid pixels
                    # If the bad data ratio explodes (>30% noise/zeros), we use the DA2 relative depth
                    if stereo_trust_ratio <= self._stereo_max_bad_ratio:
                        
                        # Use stereo depth for the actual spatial coordinate only when > 30cm
                        if median_stereo >= 0.3:
                            z_m = median_stereo
                                            
                bcx = (x1 + x2) / 2.0
                bcy = (y1 + y2) / 2.0
                
                x_m = (bcx - cx) * z_m / fx
                y_m = (bcy - cy) * z_m / fy
                
                
                det3d = Detection3D()
                hyp = ObjectHypothesisWithPose()
                base_class = str(b.get('class_id', ''))
                
                # Format: "class_name|ai_depth|stereo_depth|interpolated_z|stereo_bad_ratio"
                stereo_str = f"{median_stereo:.2f}" if median_stereo is not None else "N/A"
                bad_pct = int(stereo_trust_ratio * 100)
                hyp.hypothesis.class_id = f"{base_class}|{median_ai_depth:.2f}|{stereo_str}|{z_da2:.2f}|{bad_pct}"
                
                hyp.hypothesis.score = float(b.get('score', 1.0))
                
                hyp.pose.pose.position.x = float(x_m)
                hyp.pose.pose.position.y = float(y_m)
                hyp.pose.pose.position.z = float(z_m)
                det3d.results.append(hyp)
                
                det3d.bbox.center.position.x = float(b['orig_cx'])
                det3d.bbox.center.position.y = float(b['orig_cy'])
                det3d.bbox.size.x = float(b['orig_sx'])
                det3d.bbox.size.y = float(b['orig_sy'])
                
                out_msg.detections.append(det3d)
                
            self.pub_spatial.publish(out_msg)

        def yolo_cb(self, msg) -> None:
            boxes = []
            for det in getattr(msg, 'yolov8_inference', []):
                x1, y1, x2, y2 = det.coordinates
                cx = (x1 + x2) / 2.0
                cy = (y1 + y2) / 2.0
                sx = x2 - x1
                sy = y2 - y1
                boxes.append({
                    'class_id': det.class_name,
                    'x1': float(x1), 'y1': float(y1), 'x2': float(x2), 'y2': float(y2),
                    'orig_cx': cx, 'orig_cy': cy, 'orig_sx': sx, 'orig_sy': sy,
                    'is_normalized': False, 'nn_size': 1.0
                })
            if boxes:
                self.process_and_publish_spatial(boxes, msg.header)

        def vision2d_cb(self, msg) -> None:
            boxes = []
            for det in msg.detections:
                if not det.results:
                    continue
                id_str = getattr(det.results[0].hypothesis, 'class_id', "")
                if not id_str and hasattr(det.results[0], 'id'):
                    id_str = str(det.results[0].id)
                cx = det.bbox.center.position.x
                cy = det.bbox.center.position.y
                sx = det.bbox.size_x
                sy = det.bbox.size_y
                is_norm = (sx <= 1.05 and sy <= 1.05 and cx <= 1.05 and cy <= 1.05)
                boxes.append({
                    'class_id': id_str,
                    'x1': cx - sx/2, 'y1': cy - sy/2, 'x2': cx + sx/2, 'y2': cy + sy/2,
                    'orig_cx': cx, 'orig_cy': cy, 'orig_sx': sx, 'orig_sy': sy,
                    'is_normalized': is_norm
                })
            if boxes:
                self.process_and_publish_spatial(boxes, msg.header)
                
        def vision3d_cb(self, msg) -> None:
            boxes = []
            for det in msg.detections:
                if not det.results:
                    continue
                id_str = getattr(det.results[0].hypothesis, 'class_id', "")
                if not id_str and hasattr(det.results[0], 'id'):
                    id_str = str(det.results[0].id)
                cx = det.bbox.center.position.x
                cy = det.bbox.center.position.y
                sx = det.bbox.size.x
                sy = det.bbox.size.y
                is_norm = (sx <= 1.05 and sy <= 1.05 and cx <= 1.05 and cy <= 1.05)
                boxes.append({
                    'class_id': id_str,
                    'x1': cx - sx/2, 'y1': cy - sy/2, 'x2': cx + sx/2, 'y2': cy + sy/2,
                    'orig_cx': cx, 'orig_cy': cy, 'orig_sx': sx, 'orig_sy': sy,
                    'score': det.results[0].hypothesis.score,
                    'is_normalized': is_norm
                })
            if boxes:
                self.process_and_publish_spatial(boxes, msg.header)

    rclpy.init()
    node = DepthAnythingV2Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
