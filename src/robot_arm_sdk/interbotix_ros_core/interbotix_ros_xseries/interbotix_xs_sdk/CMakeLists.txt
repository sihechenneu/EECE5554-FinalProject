cmake_minimum_required(VERSION 3.8)
project(interbotix_xs_sdk)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(interbotix_xs_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(dynamixel_workbench_toolbox REQUIRED)
find_package(dynamixel_sdk REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
find_path(YAML_CPP_INCLUDE_DIR NAMES yaml.h PATHS ${YAML_CPP_INCLUDE_DIRS})
find_library(YAML_CPP_LIBRARY NAMES yaml-cpp PATHS ${YAML_CPP_LIBRARY_DIRS})
if(NOT YAML_CPP_LIBRARY)
  find_library(YAML_CPP_LIBRARY NAMES yaml-cpp)
endif()

if(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")
  add_definitions(-DHAVE_NEW_YAMLCPP)
endif()

include_directories(include ${YAML_CPP_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/xs_sdk_obj.cpp)
target_link_libraries(${PROJECT_NAME} ${YAML_CPP_LIBRARY})
ament_target_dependencies(${PROJECT_NAME} rclcpp interbotix_xs_msgs sensor_msgs trajectory_msgs urdf dynamixel_workbench_toolbox dynamixel_sdk)

add_executable(xs_sdk src/xs_sdk.cpp)
target_link_libraries(xs_sdk ${PROJECT_NAME} ${YAML_CPP_LIBRARY})
ament_target_dependencies(xs_sdk rclcpp interbotix_xs_msgs dynamixel_workbench_toolbox dynamixel_sdk)
# Ensure dynamixel_sdk appears after static toolbox on link line (satisfies undefined refs)
target_link_libraries(xs_sdk dynamixel_sdk::dynamixel_sdk)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS xs_sdk
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
)
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS scripts/xs_sdk_sim
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(rclcpp interbotix_xs_msgs sensor_msgs urdf dynamixel_workbench_toolbox)

ament_package()
