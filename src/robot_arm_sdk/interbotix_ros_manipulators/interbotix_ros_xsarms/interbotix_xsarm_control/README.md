# interbotix_xsarm_control

[![View Documentation](https://docs.trossenrobotics.com/docs_button.svg)](https://docs.trossenrobotics.com/interbotix_xsarms_docs/ros1_packages/arm_control.html)

## Overview

This package contains the configuration and launch files necessary to easily start the Interbotix Arm platform. This includes launching the **xs_sdk** node responsible for driving the Dynamixel motors on the robot and loading the URDF to the `robot_description` parameter. Essentially, this package is what all 'downstream' ROS packages should reference to get the robot up and running.

## Validating Without Hardware (Simulation)

You can test the control stack and RViz panel **without a physical robot** by using the built-in software simulator:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch interbotix_xsarm_control xsarm_control.launch.py robot_model:=px100 use_sim:=true
```

- **`use_sim:=true`** runs **xs_sdk_sim** instead of the real **xs_sdk** driver. The sim node publishes joint states and responds to the same topics and services, so the Interbotix Control Panel in RViz and any other nodes (e.g. MoveIt, joystick) work as with the real arm.
- In RViz you can add the **Interbotix Control Panel** (Panels → Add New Panel) and drive the robot in the 3D view; the model will follow commanded positions.
- This does **not** use Gazebo—it is a lightweight “RViz-only” sim for validating the port and your code.

For **Gazebo** simulation (physics, sensors), use the `interbotix_xsarm_gazebo` package and launch Gazebo plus the arm with `use_sim:=true` for the control stack (see that package’s README when available).

## Physical robot (real hardware)

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch interbotix_xsarm_control xsarm_control.launch.py robot_model:=px100
```

If the arm is on a different serial port than the config default (`/dev/ttyDXL`), pass it explicitly:

```bash
ros2 launch interbotix_xsarm_control xsarm_control.launch.py robot_model:=px100 robot_port:=/dev/ttyUSB0
```

**"Failed transmit instruction packet!" / "Can't find DYNAMIXEL ID"** usually means:

1. **Wrong port** – Check which port the arm is on: `ls /dev/ttyUSB* /dev/ttyACM*` (unplug the arm, run again; the one that disappears is the arm). Use that device with `robot_port:=/dev/ttyUSB0` (or the correct name). Install the Interbotix udev rules so `/dev/ttyDXL` exists and has correct permissions: copy `interbotix_xs_sdk/99-interbotix-udev.rules` into `/etc/udev/rules.d/`, then `sudo udevadm control --reload-rules && sudo udevadm trigger`, and replug the USB.
2. **Arm not powered** – Ensure the arm's power supply is on.
3. **Wrong cable** – Use the USB cable/adapter that came with the arm (e.g. U2D2); standard USB cables may not expose the correct serial device.
