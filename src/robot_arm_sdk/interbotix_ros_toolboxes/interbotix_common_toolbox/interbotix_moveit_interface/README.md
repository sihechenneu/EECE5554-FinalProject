# interbotix_moveit_interface

## Overview
This package contains a small API modeled after the [Move Group C++ Interface Tutorial](https://github.com/ros-planning/moveit_tutorials/blob/482dc9db944c785870274c35223b4d06f2f0bc90/doc/move_group_interface/src/move_group_interface_tutorial.cpp) that allows a user to command desired end-effector poses to an Interbotix arm. It is not meant to be all-encompassing but rather should be viewed as a starting point for someone interested in creating their own MoveIt interface to interact with an arm. The package also contains a small GUI that can be used to pose the end-effector.

Finally, this package also contains a modified version of the [Move Group Python Interface Tutorial](https://github.com/ros-planning/moveit_tutorials/blob/482dc9db944c785870274c35223b4d06f2f0bc90/doc/move_group_python_interface/scripts/move_group_python_interface_tutorial.py) script that can be used as a guide for those users who would like to interface with an Interbotix robot via the MoveIt Commander Python module.

## Nodes
The *interbotix_moveit_interface* nodes are described below:
- **moveit_interface** - a small C++ API that makes it easier for a user to command custom poses to the end-effector of an Interbotix arm; it uses MoveIt's planner behind the scenes to generate desired joint trajectories
- **moveit_interface_gui** - a GUI (modeled after the one in the *joint_state_publisher* package) that allows a user to enter in desired end-effector poses via text fields or sliders; it uses the **moveit_interface** API to plan and execute trajectories
- **moveit_python_interface** - a modified version of the script used in the [Move Group Python Interface](http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/move_group_python_interface/move_group_python_interface_tutorial.html) tutorial that is meant to work with an Interbotix arm; just press 'Enter' in the terminal to walk through the different steps

## Usage
This package is not meant to be used by itself but with any robot platform that contains an arm (like a standalone arm or a mobile manipulator). Refer to the example ROS packages by those robot platforms to see more info on how this package is used. These nodes are not located there to avoid code duplicity.

### Command-line: move end-effector to a 3D point (base frame)

With MoveIt and the **moveit_interface** node running (same namespace as the robot, e.g. `px100`):

**Using the helper script (X Y Z in meters, base at 0,0,0):**
```bash
ros2 run interbotix_moveit_interface move_to_point.sh 0.2 0.0 0.2
# optional 4th argument: robot namespace (default px100)
ros2 run interbotix_moveit_interface move_to_point.sh 0.15 -0.05 0.25 wx200
```

**Using raw `ros2 service call` (plan then execute):**
```bash
# 1) Plan to position (cmd=2). Replace x,y,z and namespace.
ros2 service call /px100/moveit_plan interbotix_moveit_interface/srv/MoveItPlan \
  "{cmd: 2, ee_pose: {position: {x: 0.2, y: 0.0, z: 0.2}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"

# 2) Execute the plan (cmd=4)
ros2 service call /px100/moveit_plan interbotix_moveit_interface/srv/MoveItPlan \
  "{cmd: 4, ee_pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```

`cmd`: 1=plan full pose, 2=plan position only, 3=plan orientation only, 4=execute.
