cmake_minimum_required(VERSION 3.8)
project(interbotix_moveit_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/MoveItPlan.srv"
  DEPENDENCIES geometry_msgs std_msgs
)

# C++ node only when MoveIt 2 is available
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface QUIET)
find_package(moveit_visual_tools QUIET)
if(moveit_ros_planning_interface_FOUND AND moveit_visual_tools_FOUND)
  rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
  add_executable(moveit_interface src/moveit_interface.cpp src/moveit_interface_obj.cpp)
  add_dependencies(moveit_interface ${cpp_typesupport_target})
  target_include_directories(moveit_interface PRIVATE
    include
    ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
  )
  ament_target_dependencies(moveit_interface
    rclcpp
    geometry_msgs
    std_msgs
    moveit_ros_planning_interface
    moveit_visual_tools
  )
  target_link_libraries(moveit_interface "${cpp_typesupport_target}")
  install(TARGETS moveit_interface DESTINATION lib/${PROJECT_NAME})
endif()

install(PROGRAMS scripts/moveit_interface_gui scripts/moveit_python_interface scripts/move_to_point.sh
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY images/ DESTINATION share/${PROJECT_NAME}/images OPTIONAL)

ament_package()
