# yolo_trash — 2D trash detection

YOLO-based trash detection for images, video, and ROS 2. The package builds as **`trash_detection_2d`**.

**Workspace (ROS 2 Jazzy):** Build from the workspace root (e.g. `EECE5554-FinalProject`). Use **`--merge-install`** so `trash_detection_2d` and `yolov8_msgs` share one prefix and `ros2 run` finds the package. Then source the top-level install in every terminal where you run ROS commands:

```bash
cd /path/to/EECE5554-FinalProject   # workspace root
# If you already have an install/ from a previous non-merge build, remove it first:
#   rm -rf install build
colcon build --packages-select yolov8_msgs trash_detection_2d --merge-install
source install/setup.bash
```

---

## 1. CLI inference (image, video, webcam)

Uses `yolo_trash.detector` and optional `--weights` (default: `yolo_trash/best.pt`).

```bash
cd src/visual/yolo_trash 
python -m yolo_trash.inference --source /path/to/image_or_video
python -m yolo_trash.inference --source 0 --show   # webcam
```

Or after building the workspace and sourcing `install/setup.bash`:

```bash
ros2 run trash_detection_2d yolo_trash_inference -- --source /path/to/image_or_video
```

---

## 2. ROS 2 detection node (live camera)

Subscribes to an RGB image topic and publishes **`yolov8_msgs/Yolov8Inference`** (header + list of `InferenceResult`: `class_name`, `coordinates` bbox).

**Build and run** (from workspace root; use `--merge-install` and source `install/setup.bash`):

```bash
colcon build --packages-select yolov8_msgs trash_detection_2d --merge-install
source install/setup.bash
ros2 run trash_detection_2d yolo_trash_subscriber
```

The node subscribes to the image topic and publishes detections; it runs independently (no camera required to start). Remap topics or pass `weights` if needed.

**Launch file** (with remappable args):

```bash
ros2 launch trash_detection_2d detection_node.launch.py
```

Launch arguments:

| Argument           | Default                         | Description                    |
|--------------------|---------------------------------|--------------------------------|
| `image_topic`      | `/camera/color/image_raw`       | Input `sensor_msgs/Image`      |
| `inference_topic`  | `/yolo_trash/inference`         | Output `Yolov8Inference`       |
| `weights`          | `""` (uses detector default)    | Path to `.pt` model            |
| `conf`             | `0.35`                          | Confidence threshold           |
| `iou`              | `0.45`                          | NMS IoU threshold              |
| `imgsz`            | `640`                           | Input size                     |
| `frame_id`         | `camera_color_optical_frame`    | Header frame_id if missing     |

Example with remaps:

```bash
ros2 run trash_detection_2d yolo_trash_subscriber --ros-args \
  -p image_topic:=/my_camera/image -p inference_topic:=/trash/detections
```

---

## 3. Bag replay publisher

Publishes images from a ROS 2 bag so the detection node can run offline.

```bash
ros2 run trash_detection_2d yolo_trash_publisher --ros-args \
  -p bag_path:=/path/to/bag -p image_topic_in_bag:=/camera/color/image_raw \
  -p image_topic_out:=/camera/color/image_raw -p rate:=10.0
```

Parameters: `bag_path` (required), `image_topic_in_bag`, `image_topic_out`, `storage_id` (default `mcap`), `rate` (Hz).

---

## Training

- **`train.py`** — Trains YOLOv8 (default base: `yolov8n.pt`) on the TACO-derived YOLO dataset.  
  Default data: `datasets/taco_yolo/data.yaml`; output: `runs/detect/taco_yolov8/` (e.g. `weights/best.pt`).

- **`dataset.py`** — Builds `datasets/taco_yolo` from TACO (clone + annotations). Converts COCO-style annotations to YOLO format and writes `data.yaml` with class names.


Use the produced `best.pt` as `--weights` for the subscriber or CLI.