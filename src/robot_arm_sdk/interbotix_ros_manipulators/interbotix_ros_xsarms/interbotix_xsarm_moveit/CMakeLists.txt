cmake_minimum_required(VERSION 3.8)
project(interbotix_xsarm_moveit)
find_package(ament_cmake REQUIRED)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch" EXCLUDE)
ament_package()
