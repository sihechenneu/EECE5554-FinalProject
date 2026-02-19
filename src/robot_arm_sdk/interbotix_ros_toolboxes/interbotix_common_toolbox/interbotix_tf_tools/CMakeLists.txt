cmake_minimum_required(VERSION 3.8)
project(interbotix_tf_tools)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
find_path(YAML_CPP_INCLUDE_DIR NAMES yaml-cpp/yaml.h PATHS ${YAML_CPP_INCLUDE_DIRS})
find_library(YAML_CPP_LIBRARY NAMES yaml-cpp PATHS ${YAML_CPP_LIBRARY_DIRS})

if(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")
  add_compile_definitions(HAVE_NEW_YAMLCPP)
endif()

include_directories(include ${YAML_CPP_INCLUDE_DIRS})

add_executable(tf_rebroadcaster src/tf_rebroadcaster.cpp)
ament_target_dependencies(tf_rebroadcaster
  geometry_msgs
  rclcpp
  tf2_msgs
  tf2_ros
)
target_link_libraries(tf_rebroadcaster ${YAML_CPP_LIBRARIES})

install(TARGETS tf_rebroadcaster
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
