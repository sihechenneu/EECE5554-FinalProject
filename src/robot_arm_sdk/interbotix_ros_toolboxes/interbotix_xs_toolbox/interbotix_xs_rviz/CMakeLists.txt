cmake_minimum_required(VERSION 3.8)
project(interbotix_xs_rviz)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(interbotix_xs_msgs REQUIRED)
find_package(rviz_common REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(pluginlib REQUIRED)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

qt5_wrap_ui(UIC_FILES src/ui/interbotix_control_panel.ui)

add_library(${PROJECT_NAME} SHARED
  src/interbotix_control_panel.cpp
  include/interbotix_xs_rviz/interbotix_control_panel.hpp
  ${UIC_FILES}
)
target_include_directories(${PROJECT_NAME} PRIVATE
  include
  ${CMAKE_CURRENT_BINARY_DIR}
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  interbotix_xs_msgs
  rviz_common
  pluginlib
)
target_link_libraries(${PROJECT_NAME} Qt5::Widgets)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Register plugin with ament index so rviz2 can discover it (plugin_category must match rviz_common)
pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(rviz_common interbotix_xs_msgs pluginlib)
ament_package()
