cmake_minimum_required(VERSION 3.8)
project(interbotix_uxarm_moveit)
find_package(ament_cmake REQUIRED)
install(DIRECTORY config launch images DESTINATION share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch" EXCLUDE)
ament_package()
