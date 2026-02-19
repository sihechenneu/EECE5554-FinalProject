cmake_minimum_required(VERSION 3.8)
project(interbotix_uxarm_control)
find_package(ament_cmake REQUIRED)
# gripper_pub C++ node omitted until interbotix_ux_modules is ported to ROS2
install(DIRECTORY config launch images DESTINATION share/${PROJECT_NAME})
ament_package()
