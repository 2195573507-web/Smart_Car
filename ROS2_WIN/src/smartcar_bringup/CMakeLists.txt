cmake_minimum_required(VERSION 3.8)
project(smartcar_bringup)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_goal_confirmation_core test/test_goal_confirmation_core.py)
endif()

install(DIRECTORY launch config rviz scripts
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE)

install(PROGRAMS
  scripts/goal_confirmation.py
  scripts/goal_confirmation_core.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
