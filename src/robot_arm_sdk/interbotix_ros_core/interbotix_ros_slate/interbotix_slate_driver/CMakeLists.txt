cmake_minimum_required(VERSION 3.8)
project(interbotix_slate_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

set(serial_driver "chassis_driver")

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(interbotix_slate_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(
  include
)

add_library(slate_base
  src/slate_base.cpp
  src/base_driver.cpp
)
target_link_libraries(slate_base
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/lib${serial_driver}.so
)
ament_target_dependencies(slate_base
  geometry_msgs
  interbotix_slate_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  std_srvs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

add_executable(slate_base_node
  src/slate_base_node.cpp
)
ament_target_dependencies(slate_base_node
  geometry_msgs
  interbotix_slate_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  std_srvs
  tf2
  tf2_ros
  tf2_geometry_msgs
)
target_link_libraries(slate_base_node slate_base)

add_custom_command(
  OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/lib/lib${serial_driver}.so
  COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_CURRENT_BINARY_DIR}/lib
  COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_CURRENT_SOURCE_DIR}/lib/lib${serial_driver}.so ${CMAKE_CURRENT_BINARY_DIR}/lib/
)
add_custom_target(COPY_CHASSIS_LIB ALL DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/lib/lib${serial_driver}.so slate_base_node)

install(TARGETS
  slate_base
  slate_base_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(FILES
  lib/lib${serial_driver}.so
  DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
