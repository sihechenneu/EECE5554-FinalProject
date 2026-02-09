#!/bin/bash
# Move the arm end-effector to a 3D point (in the robot base frame) using the moveit_plan service.
# Requires: MoveIt 2 and moveit_interface node running (e.g. under the same robot namespace).
#
# Usage: move_to_point.sh X Y Z [ROBOT_NAMESPACE]
#   X Y Z  - target position in meters (base frame; base is 0,0,0)
#   ROBOT_NAMESPACE - optional, default: px100
#
# Example: move_to_point.sh 0.2 0.0 0.2
# Example: move_to_point.sh 0.15 -0.05 0.25 wx200

set -e
X="${1:?Usage: $0 X Y Z [ROBOT_NAMESPACE]}"
Y="${2:?Usage: $0 X Y Z [ROBOT_NAMESPACE]}"
Z="${3:?Usage: $0 X Y Z [ROBOT_NAMESPACE]}"
NS="${4:-px100}"

SVC="/${NS}/moveit_plan"

# Check service exists
if ! ros2 service list | grep -q "^${SVC}$"; then
  echo "Service ${SVC} not found. Start MoveIt and moveit_interface first (e.g. with robot_name:=${NS})."
  exit 1
fi

# Step 1: Plan to position (cmd=2 = CMD_PLAN_POSITION)
echo "Planning to position ($X, $Y, $Z) ..."
RES=$(ros2 service call "${SVC}" interbotix_moveit_interface/srv/MoveItPlan \
  "{cmd: 2, ee_pose: {position: {x: ${X}, y: ${Y}, z: ${Z}}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}")
if ! echo "$RES" | grep -q "success: true"; then
  echo "Plan failed: $RES"
  exit 1
fi
echo "Plan succeeded."

# Step 2: Execute (cmd=4 = CMD_EXECUTE)
echo "Executing trajectory ..."
RES=$(ros2 service call "${SVC}" interbotix_moveit_interface/srv/MoveItPlan \
  "{cmd: 4, ee_pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}")
if ! echo "$RES" | grep -q "success: true"; then
  echo "Execute failed: $RES"
  exit 1
fi
echo "Done."
