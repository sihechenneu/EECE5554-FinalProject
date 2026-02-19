cmake_minimum_required(VERSION 3.8)
project(interbotix_ux_ros_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(xarm_msgs REQUIRED)
find_package(xarm_api REQUIRED)

add_executable(ux_hardware_interface
  src/ux_hardware_interface.cpp
  src/ux_hardware_interface_obj.cpp
)
target_include_directories(ux_hardware_interface PRIVATE include)
ament_target_dependencies(ux_hardware_interface
  rclcpp
  sensor_msgs
  xarm_msgs
  xarm_api
)
target_link_libraries(ux_hardware_interface xarm_api::xarm_ros_client xarm_api::xarm_api)
install(TARGETS ux_hardware_interface
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(rclcpp sensor_msgs xarm_msgs xarm_api)
ament_package()
