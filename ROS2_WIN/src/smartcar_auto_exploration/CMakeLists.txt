cmake_minimum_required(VERSION 3.8)
project(smartcar_auto_exploration)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_frontier test/test_frontier.py)
  ament_add_pytest_test(test_exploration_core test/test_exploration_core.py)
  ament_add_pytest_test(test_ros_adapter_startup test/test_ros_adapter_startup.py)
endif()

install(DIRECTORY smartcar_auto_exploration
  DESTINATION lib/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE)
install(PROGRAMS scripts/auto_exploration.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
