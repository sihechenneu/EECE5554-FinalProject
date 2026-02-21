# 2D trash detection

YOLO-based trash detection. Two ways to run:

**1. Manual (CLI)** — image, video, or webcam:
```bash
python inference.py --source /path/to/image_or_video
python inference.py --source 0   # webcam
```

**2. ROS2 node** — subscribe to RGB images, publish `vision_msgs/Detection2DArray`:
```bash
colcon build --packages-select trash_detection_2d && source install/setup.bash
ros2 run trash_detection_2d trash_detection_node
```
Remap image topic if needed: `--ros-args -p image_topic:=/your/image/topic`

Train with `train.py` (dataset via `dataset.py`). Weights default: `runs/detect/taco_yolov8/weights/best.pt`.
