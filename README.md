# EECE5554-FinalProject: Autonomous Trash Collection System (ATCS)

An end-to-end autonomous mobile manipulation system designed to detect, localize, and collect indoor trash items (cans, bottles, paper balls, etc.) using a **TurtleBot 4** platform.

## 🚀 Project Overview

This project integrates real-time computer vision, SLAM, and robotic manipulation to maintain cleanliness in indoor environments. Developed as part of the **Northeastern University CS5180/EECE5554** curriculum.

### Key Features

* **On-Device Perception:** YOLOv8n object detection running natively on the OAK-D VPU for zero-latency RGB-Depth synchronization.
* **3D Localization:** Fuses 2D bounding boxes with stereo disparity to estimate metric 3D coordinates.
* **Autonomous Navigation:** LiDAR-based SLAM using `slam_toolbox` and Nav2 behavior trees for collision-free exploration.
* **Dexterous Manipulation:** Custom pick-and-place pipeline for the 4-DOF PincherX 100 arm.
* **Robust State Machine:** A comprehensive behavior tree managing states from *Exploring* to *Return-to-Dock*.

---

## 🛠 Hardware Stack

* **Mobile Base:** iRobot Create 3 (TurtleBot 4)
* **Manipulator:** Trossen Robotics PincherX 100 (PX100)
* **Primary Sensor:** Luxonis OAK-D Stereo Camera
* **Lidar:** RPLiDAR A1
* **Compute:** Raspberry Pi 4 (Onboard) + Laptop (Fallback GPU compute)
* **Custom Parts:** 3D-printed upper deck, camera mounts, and gripper extensions.

---

## 💻 Software Architecture

The system is built on **ROS 2 Jazzy**.

### Component Breakdown

1. **`oakd_driver_node`**: Handles VPU-based YOLO inference and stereo depth streaming.
2. **`yolo_3d_detector_node`**: Projects 2D detections into 3D space using camera intrinsics and median depth filtering.
3. **`slam_toolbox`**: Provides graph-based SLAM for real-time mapping and localization.
4. **`mission_state_machine`**: Orchestrates high-level logic: `Explore` → `Approach` → `Pick Up` → `Deposit`.
5. **`px100_arm_controller`**: Manages analytical Inverse Kinematics (IK) and joint trajectories for the arm.

---

## 📊 Performance Targets

| Metric | Goal |
| --- | --- |
| **Detection Accuracy** | $\geq 70\%$ mAP50 (Level 2) / $\geq 90\%$ (Level 3) |
| **3D Localization Error** | $\pm 3$ cm within manipulation range |
| **Grasp Success Rate** | $\geq 60\%$ (overall) |
| **Navigation Accuracy** | $< 10$ cm translational error |
| **Cycle Time** | 10–20 minutes per room |

---

## 📂 Repository Structure

```text
├── core/                   # Mission state machine and callback logic
├── scripts/                # Training (MaskablePPO) and ROS 2 nodes
├── hardware/               # STL files for 3D-printed mounts
├── models/                 # Trained YOLOv8 weights (.pt and .blob)
├── launch/                 # Unified system launch files
└── config/                 # Nav2 and SLAM parameters

```

---

## ⚙️ Installation & Setup

1. **Clone the Repo:**
```bash
git clone https://github.com/your-username/trash-collection-system.git

```


2. **Install Dependencies:**
```bash
# Ensure ROS 2 Jazzy is installed
sudo apt install ros-jazzy-slam-toolbox ros-jazzy-navigation2 ros-jazzy-depthai-ros
pip install -r requirements.txt

```


3. **Permissions:**
Ensure you have access to the OAK-D and USB serial ports:
```bash
sudo usermod -a -G dialout $USER

```



---

## 📝 Team

* **Yanyue Wang** - Northeastern University
* **Wenyuan Sheng** - Northeastern University
* **Sihe Chen** - Northeastern University

---

## ⚖️ License

This project is licensed under the MIT License - see the [LICENSE](https://www.google.com/search?q=LICENSE) file for details.

**Would you like me to add a section on how to run the Fallback mode with Depth Anything V2 or specific instructions for the Conda environment setup?**