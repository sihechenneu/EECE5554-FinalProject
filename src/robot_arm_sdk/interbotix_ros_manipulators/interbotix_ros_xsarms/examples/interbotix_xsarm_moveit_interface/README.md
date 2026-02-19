# interbotix_xsarm_moveit_interface

[![docs](https://docs.trossenrobotics.com/docs_button.svg)](https://docs.trossenrobotics.com/interbotix_xsarms_docs/ros1_packages/moveit_interface_and_api.html)

## Overview

This package contains a small API modeled after the [Move Group C++ Interface Tutorial](https://github.com/ros-planning/moveit_tutorials/blob/kinetic-devel/doc/move_group_interface/src/move_group_interface_tutorial.cpp) that allows a user to command desired end-effector poses to an Interbotix arm. It is not meant to be all-encompassing but rather should be viewed as a starting point for someone interested in creating their own MoveIt interface to interact with an arm. The package also contains a small GUI that can be used to pose the end-effector.

Finally, this package also contains a modified version of the [Move Group Python Interface Tutorial](https://github.com/ros-planning/moveit_tutorials/blob/kinetic-devel/doc/move_group_python_interface/scripts/move_group_python_interface_tutorial.py) script that can be used as a guide for those users who would like to interface with an Interbotix robot via the MoveIt Commander Python module.

## ROS 2: Enabling `/px100/moveit_plan` (move to 3D point)

The `moveit_plan` service is provided by the **moveit_interface** node, which runs together with **MoveIt 2 move_group**. To have `/px100/moveit_plan` available (e.g. for `move_to_point.sh` or `ros2 service call`):

1. **Install MoveIt 2** (if not already installed). If you see `package 'moveit_ros_move_group' not found`, run:
   ```bash
   sudo apt install ros-jazzy-moveit
   ```
   Then run `source /opt/ros/jazzy/setup.bash` (and your `install/setup.bash`) again.

2. **Run the combined launch** (control + sim + MoveIt + moveit_interface):
   ```bash
   source install/setup.bash
   ros2 launch interbotix_xsarm_moveit_interface xsarm_moveit_interface.launch.py robot_model:=px100
   ```
   This starts the arm description, xs_sdk_sim, MoveIt 2 move_group, and moveit_interface. The service will be at `/px100/moveit_plan`.

3. **Then** in another terminal you can move to a 3D point:
   ```bash
   ros2 run interbotix_moveit_interface move_to_point.sh 0.2 0.0 0.2
   ```

To run only MoveIt + moveit_interface (e.g. you already have control + RViz running), use `use_sim_control:=false` and ensure the robot description and joint states are being published (e.g. from your existing control launch).
