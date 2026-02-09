cmake_minimum_required(VERSION 3.8)
project(interbotix_xs_ros_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(interbotix_xs_msgs REQUIRED)

# Standalone bridge node: subscribes to joint_states and forwards commands from a control loop.
# For full ros2_control integration (controller_manager loading this as a plugin), a separate
# hardware_interface plugin would be needed; this node provides the same behavior as the
# legacy ROS1 node when used with the spawner that loads position controllers.
add_executable(xs_hardware_interface
  src/xs_hardware_interface.cpp
  src/xs_hardware_interface_obj.cpp
)
target_include_directories(xs_hardware_interface PRIVATE include)
ament_target_dependencies(xs_hardware_interface
  rclcpp
  sensor_msgs
  interbotix_xs_msgs
)
install(TARGETS xs_hardware_interface
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(rclcpp sensor_msgs interbotix_xs_msgs)
ament_package()
