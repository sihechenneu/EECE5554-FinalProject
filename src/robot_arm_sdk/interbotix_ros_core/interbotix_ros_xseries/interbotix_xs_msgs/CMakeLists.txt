cmake_minimum_required(VERSION 3.8)
project(interbotix_xs_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/JointGroupCommand.msg"
  "msg/JointSingleCommand.msg"
  "msg/JointTrajectoryCommand.msg"
  "msg/JointTemps.msg"
  "msg/ArmJoy.msg"
  "msg/HexJoy.msg"
  "msg/LocobotJoy.msg"
  "msg/TurretJoy.msg"
  "srv/Reboot.srv"
  "srv/RobotInfo.srv"
  "srv/MotorGains.srv"
  "srv/TorqueEnable.srv"
  "srv/OperatingModes.srv"
  "srv/RegisterValues.srv"
  DEPENDENCIES std_msgs trajectory_msgs geometry_msgs
)

ament_package()
