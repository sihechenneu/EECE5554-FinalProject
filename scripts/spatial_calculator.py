#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection3DArray, Detection3D, ObjectHypothesisWithPose, Detection2DArray
import numpy as np
import argparse
import sys

class SpatialCalculatorNode(Node):
    def __init__(self, depth_topic, info_topic, yolo_topic, output_topic, **kwargs):
        super().__init__('spatial_calculator')
        self.latest_depth = None
        self.latest_depth_encoding = None
        self.camera_info = None

        self.create_subscription(Image, depth_topic, self.depth_cb, 1)
        self.create_subscription(CameraInfo, info_topic, self.info_cb, 1)
        self.pub = self.create_publisher(Detection3DArray, output_topic, 1)
        
        # We subscribe to ONLY the specified YOLO format to avoid ROS 2 incompatible type errors
        if kwargs.get('yolo_type', 'Detection2DArray') == 'Yolov8Inference':
            try:
                from yolov8_msgs.msg import Yolov8Inference
                self.create_subscription(Yolov8Inference, yolo_topic, self.yolo_cb, 1)
            except ImportError as e:
                self.get_logger().error(f"Cannot import Yolov8Inference: {e}")
        elif kwargs.get('yolo_type') == 'Detection3DArray':
            self.create_subscription(Detection3DArray, yolo_topic, self.vision3d_cb, 1)
        else:
            self.create_subscription(Detection2DArray, yolo_topic, self.vision2d_cb, 1)
        
        self.get_logger().info(f"Spatial calculator started.")
        self.get_logger().info(f" - Listening to YOLO: {yolo_topic}")
        self.get_logger().info(f" - Listening to Depth: {depth_topic}")
        self.get_logger().info(f" - Listening to Camera Info: {info_topic}")
        self.get_logger().info(f" - Publishing Spatial 3D Detections: {output_topic}")

    def depth_cb(self, msg):
        self.latest_depth_encoding = msg.encoding
        if msg.encoding in ("16UC1", "16U_C1", "MONO16", "TYPE_16UC1"):
            self.latest_depth = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
        elif msg.encoding in ("32FC1", "TYPE_32FC1"):
            self.latest_depth = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
        else:
            self.latest_depth = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width)

    def info_cb(self, msg):
        self.camera_info = msg

    def process_and_publish(self, boxes, header):
        if self.latest_depth is None or self.camera_info is None:
            return
            
        out_msg = Detection3DArray()
        out_msg.header = header
        
        fx = self.camera_info.k[0]
        fy = self.camera_info.k[4]
        cx = self.camera_info.k[2]
        cy = self.camera_info.k[5]
        
        depth_h, depth_w = self.latest_depth.shape
        nn_scale = max(depth_w, depth_h)
        
        for b in boxes:
            is_normalized = b.get('is_normalized', False)
            if is_normalized:
                x1_norm = b['x1']
                x2_norm = b['x2']
                y1_norm = b['y1']
                y2_norm = b['y2']
                x1 = int(x1_norm * depth_w)
                x2 = int(x2_norm * depth_w)
                y1 = int(y1_norm * depth_h)
                y2 = int(y2_norm * depth_h)
            else:
                s = b.get('nn_size', nn_scale)
                if s == 0:
                    s = 1.0 # fallback
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
                
            depth_roi = self.latest_depth[y1:y2, x1:x2]
            valid_depths = depth_roi[depth_roi > 0]
            
            if len(valid_depths) == 0:
                continue
                
            median_depth = float(np.median(valid_depths))
            if self.latest_depth_encoding in ("16UC1", "16U_C1", "MONO16", "TYPE_16UC1"):
                z_m = median_depth / 1000.0
            elif self.latest_depth_encoding in ("8UC1", "8U_C1", "MONO8"):
                z_m = median_depth / 255.0 * 2.0  # approximate scaling
            else:
                z_m = median_depth 
                
            bcx = (x1 + x2) / 2.0
            bcy = (y1 + y2) / 2.0
            
            x_m = (bcx - cx) * z_m / fx
            y_m = (bcy - cy) * z_m / fy
            
            det3d = Detection3D()
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = b['class_id']
            hyp.hypothesis.score = b.get('score', 1.0)
            
            # Store calculated 3D coordinates in pose.position
            hyp.pose.pose.position.x = x_m
            hyp.pose.pose.position.y = y_m
            hyp.pose.pose.position.z = z_m
            det3d.results.append(hyp)
            
            # Save the 2D bounding box info inside the Detection3D format exactly as visualize_image_bag expects:
            det3d.bbox.center.position.x = float(b['orig_cx'])
            det3d.bbox.center.position.y = float(b['orig_cy'])
            det3d.bbox.size.x = float(b['orig_sx'])
            det3d.bbox.size.y = float(b['orig_sy'])
            
            out_msg.detections.append(det3d)
            
        self.pub.publish(out_msg)

    def yolo_cb(self, msg):
        boxes = []
        for det in msg.yolov8_inference:
            x1, y1, x2, y2 = det.coordinates
            cx = (x1 + x2) / 2.0
            cy = (y1 + y2) / 2.0
            sx = x2 - x1
            sy = y2 - y1
            boxes.append({
                'class_id': det.class_name,
                'x1': float(x1),
                'y1': float(y1),
                'x2': float(x2),
                'y2': float(y2),
                'orig_cx': cx,
                'orig_cy': cy,
                'orig_sx': sx,
                'orig_sy': sy,
                'is_normalized': False,
                'nn_size': 1.0
            })
        if boxes:
            self.process_and_publish(boxes, msg.header)

    def vision2d_cb(self, msg):
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
                'x1': cx - sx/2,
                'y1': cy - sy/2,
                'x2': cx + sx/2,
                'y2': cy + sy/2,
                'orig_cx': cx,
                'orig_cy': cy,
                'orig_sx': sx,
                'orig_sy': sy,
                'is_normalized': is_norm
            })
        if boxes:
            self.process_and_publish(boxes, msg.header)
            
    def vision3d_cb(self, msg):
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
                'x1': cx - sx/2,
                'y1': cy - sy/2,
                'x2': cx + sx/2,
                'y2': cy + sy/2,
                'orig_cx': cx,
                'orig_cy': cy,
                'orig_sx': sx,
                'orig_sy': sy,
                'score': det.results[0].hypothesis.score,
                'is_normalized': is_norm
            })
        if boxes:
            self.process_and_publish(boxes, msg.header)

def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--depth-topic", default="/oakd/stereo/image_raw")
    parser.add_argument("--info-topic", default="/oakd/rgb/camera_info")
    parser.add_argument("--yolo-topic", default="/yolo/detections")
    parser.add_argument("--yolo-type", default="Detection2DArray", choices=["Detection2DArray", "Detection3DArray", "Yolov8Inference"])
    parser.add_argument("--output-topic", default="/yolo/spatial_detections")
    parsed_args, sys_args = parser.parse_known_args()
    
    rclpy.init(args=sys_args)
    node = SpatialCalculatorNode(
        depth_topic=parsed_args.depth_topic,
        info_topic=parsed_args.info_topic,
        yolo_topic=parsed_args.yolo_topic,
        output_topic=parsed_args.output_topic,
        yolo_type=parsed_args.yolo_type
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
