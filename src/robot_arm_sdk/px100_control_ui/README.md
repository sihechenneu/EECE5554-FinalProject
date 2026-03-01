# PincherX 100 Control UI

Web UI to control the PincherX 100 robot arm in ROS 2 Jazzy. Control all joints (waist, shoulder, elbow, wrist_angle, gripper), toggle torque, and use preset poses.

## Prerequisites

- ROS 2 Jazzy
- PincherX 100 arm driver running (e.g. `interbotix_xsarm_control` with `robot_model:=px100`)
- **Flask** (required for the web UI). Install one of:
  - **System (recommended on Ubuntu/Debian):** `sudo apt install python3-flask`
  - Or in a venv: `pip install flask`

## Build

From your workspace root:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select px100_control_ui --symlink-install
source install/setup.bash
```

If you have not built the rest of the workspace (e.g. `interbotix_xs_msgs`), build dependencies first:

```bash
colcon build --packages-up-to px100_control_ui --symlink-install
source install/setup.bash
```

## Run

1. **Start the robot arm** (in one terminal), e.g.:

   ```bash
   ros2 launch interbotix_xsarm_control xsarm_control.launch.py robot_model:=px100
   ```

2. **Start the UI** (in another terminal):

   ```bash
   source install/setup.bash
   ros2 launch px100_control_ui px100_control_ui.launch.py
   ```

   Or run the node directly with optional parameters:

   ```bash
   ros2 run px100_control_ui ui_node --ros-args -p robot_name:=px100 -p ui_port:=8080
   ```

3. **Open the UI** in a browser: **http://localhost:8080**

## UI features

- **Connection**: Status and torque enable toggle (arm + gripper).
- **Joint control**: Sliders for waist, shoulder, elbow, wrist_angle, gripper (values in radians). Use **Move arm** to send all arm joints; **Move gripper** to send the gripper only.
- **Preset poses**: **Sleep**, **Home**, **Ready** — set sliders and send commands in one click.

## Parameters

- `robot_name` (string, default: `px100`): Must match the namespace of the arm driver (topic/service prefix).
- `ui_port` (int, default: `8080`): Port for the web server.

## Topics / services used

The UI node connects to the arm under the namespace given by `robot_name` (e.g. `px100`):

- Subscribes: `/<robot_name>/joint_states`
- Publishes: `/<robot_name>/commands/joint_single`, `/<robot_name>/commands/joint_group`
- Services: `/<robot_name>/torque_enable`, `/<robot_name>/get_robot_info`
