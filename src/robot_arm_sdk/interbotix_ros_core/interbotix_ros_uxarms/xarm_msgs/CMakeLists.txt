cmake_minimum_required(VERSION 3.8)
project(xarm_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotMsg.msg"
  "msg/IOState.msg"
  "srv/Move.srv"
  "srv/SetAxis.srv"
  "srv/SetInt16.srv"
  "srv/TCPOffset.srv"
  "srv/SetDigitalIO.srv"
  "srv/GetDigitalIO.srv"
  "srv/GetAnalogIO.srv"
  "srv/ClearErr.srv"
  "srv/GetErr.srv"
  "srv/GripperConfig.srv"
  "srv/GripperMove.srv"
  "srv/GripperState.srv"
  "srv/SetLoad.srv"
  "srv/SetToolModbus.srv"
  "srv/ConfigToolModbus.srv"
  "srv/GetControllerDigitalIO.srv"
  DEPENDENCIES std_msgs
)

ament_package()
