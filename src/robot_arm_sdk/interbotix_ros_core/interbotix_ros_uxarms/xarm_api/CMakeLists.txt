cmake_minimum_required(VERSION 3.8)
project(xarm_api)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(xarm_msgs REQUIRED)

set(XARM_API_DIR src/xarm)

include_directories(include)

# Core xarm library (no ROS)
add_library(xarm_api
  ${XARM_API_DIR}/connect.cc
  ${XARM_API_DIR}/report_data.cc
  ${XARM_API_DIR}/common/crc16.cc
  ${XARM_API_DIR}/common/queue_memcpy.cc
  ${XARM_API_DIR}/debug/debug_print.cc
  ${XARM_API_DIR}/instruction/uxbus_cmd.cc
  ${XARM_API_DIR}/instruction/uxbus_cmd_ser.cc
  ${XARM_API_DIR}/instruction/uxbus_cmd_tcp.cc
  ${XARM_API_DIR}/linux/network.cc
  ${XARM_API_DIR}/linux/thread.cc
  ${XARM_API_DIR}/port/serial.cc
  ${XARM_API_DIR}/port/socket.cc
)
# xarm_api is pure C++ library, no ROS

# ROS 2 client library
add_library(xarm_ros_client src/xarm_ros_client.cpp)
target_include_directories(xarm_ros_client PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(xarm_ros_client rclcpp std_msgs sensor_msgs xarm_msgs)
target_link_libraries(xarm_ros_client xarm_api)

# Main driver node
add_executable(xarm_driver_node src/xarm_driver_node.cpp src/xarm_driver.cpp)
target_include_directories(xarm_driver_node PRIVATE include)
ament_target_dependencies(xarm_driver_node rclcpp std_msgs sensor_msgs xarm_msgs)
target_link_libraries(xarm_driver_node xarm_api)

install(TARGETS xarm_api xarm_ros_client
  EXPORT xarm_api_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS xarm_driver_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
)

ament_export_include_directories(include)
ament_export_targets(xarm_api_targets)
ament_export_dependencies(rclcpp std_msgs sensor_msgs xarm_msgs)
ament_package()
