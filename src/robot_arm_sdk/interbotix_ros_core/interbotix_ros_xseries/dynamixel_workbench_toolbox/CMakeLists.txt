cmake_minimum_required(VERSION 3.8)
project(dynamixel_workbench_toolbox)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)
find_package(dynamixel_sdk REQUIRED)

include_directories(include)

add_library(dynamixel_workbench_toolbox
  src/dynamixel_workbench_toolbox/dynamixel_item.cpp
  src/dynamixel_workbench_toolbox/dynamixel_driver.cpp
  src/dynamixel_workbench_toolbox/dynamixel_tool.cpp
  src/dynamixel_workbench_toolbox/dynamixel_workbench.cpp
)
ament_target_dependencies(dynamixel_workbench_toolbox dynamixel_sdk)

install(TARGETS dynamixel_workbench_toolbox
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_dependencies(dynamixel_sdk)
ament_export_libraries(dynamixel_workbench_toolbox)

ament_package()
