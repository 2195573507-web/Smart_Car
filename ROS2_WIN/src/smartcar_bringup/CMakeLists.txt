cmake_minimum_required(VERSION 3.8)
project(smartcar_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config rviz scripts
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE)

ament_package()
