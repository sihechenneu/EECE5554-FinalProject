cmake_minimum_required(VERSION 3.8)
project(interbotix_common_modules)

find_package(ament_cmake_python REQUIRED)
find_package(ament_cmake REQUIRED)

ament_python_install_package(${PROJECT_NAME}
  PACKAGE_DIR src/${PROJECT_NAME}
)

install(DIRECTORY
  resource
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
