cmake_minimum_required(VERSION 3.8)
# Silence CMake policy warnings from PCL/FLANN find_package (set before project)
if(POLICY CMP0144)
  cmake_policy(SET CMP0144 NEW)
endif()
if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

project(interbotix_perception_modules)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_srvs REQUIRED)
# Use our FindFLANN wrapper (sets CMP0144) so PCL's FindFLANN does not emit dev warning
list(PREPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)

# Interfaces (msg/srv)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ClusterInfo.msg"
  "srv/ClusterInfoArray.srv"
  "srv/FilterParams.srv"
  "srv/SnapPicture.srv"
  DEPENDENCIES geometry_msgs std_msgs
)

# C++ pointcloud filter node
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
add_executable(perception_pipeline src/perception_pipeline.cpp)
add_dependencies(perception_pipeline ${cpp_typesupport_target})
ament_target_dependencies(perception_pipeline
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  visualization_msgs
  std_srvs
  pcl_conversions
)
target_include_directories(perception_pipeline PRIVATE
  include
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(perception_pipeline ${PCL_LIBRARIES} "${cpp_typesupport_target}")

# Install
install(TARGETS perception_pipeline
  DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS
  scripts/armtag_tuner_gui
  scripts/pointcloud_tuner_gui
  scripts/static_trans_pub
  scripts/picture_snapper
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY src/interbotix_perception_modules/
  DESTINATION lib/python3.12/site-packages
  FILES_MATCHING PATTERN "*.py")
install(DIRECTORY config/ launch/ images/
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
  PATTERN "*.py" EXCLUDE)

ament_package()
