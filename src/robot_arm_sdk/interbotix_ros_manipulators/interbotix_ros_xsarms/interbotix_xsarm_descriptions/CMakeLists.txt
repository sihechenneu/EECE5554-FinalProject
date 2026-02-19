cmake_minimum_required(VERSION 3.8)
project(interbotix_xsarm_descriptions)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch meshes rviz urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
