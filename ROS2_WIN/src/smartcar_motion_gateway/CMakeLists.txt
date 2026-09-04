cmake_minimum_required(VERSION 3.8)
project(smartcar_motion_gateway)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(smartcar_state_bridge REQUIRED)

add_library(smartcar_motion_gateway_core
  src/motion_protocol.cpp
  src/health_gate.cpp)
target_include_directories(smartcar_motion_gateway_core PUBLIC include)
ament_target_dependencies(smartcar_motion_gateway_core
  diagnostic_msgs geometry_msgs nav_msgs rclcpp sensor_msgs std_msgs std_srvs tf2_msgs)

add_library(smartcar_motion_gateway_runtime src/gateway_node.cpp)
target_link_libraries(smartcar_motion_gateway_runtime smartcar_motion_gateway_core)
ament_target_dependencies(smartcar_motion_gateway_runtime
  diagnostic_msgs geometry_msgs nav_msgs rclcpp sensor_msgs std_msgs std_srvs tf2_ros tf2_msgs)
target_link_libraries(smartcar_motion_gateway_runtime
  smartcar_state_bridge::smartcar_state_bridge_core)

add_executable(smartcar_motion_gateway_node src/main.cpp)
target_link_libraries(smartcar_motion_gateway_node smartcar_motion_gateway_runtime)
ament_target_dependencies(smartcar_motion_gateway_node rclcpp)

install(TARGETS smartcar_motion_gateway_core smartcar_motion_gateway_runtime
  smartcar_motion_gateway_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_motion_protocol test/test_motion_protocol.cpp)
  if(TARGET test_motion_protocol)
    target_link_libraries(test_motion_protocol smartcar_motion_gateway_core)
    target_include_directories(test_motion_protocol PRIVATE include)
    set_tests_properties(test_motion_protocol PROPERTIES
      ENVIRONMENT "ROS_MOTION_CONTROL_GOLDEN_VECTORS=/ws/protocol/ros-motion-control-v1-golden-vectors.json")
  endif()
  ament_add_gtest(test_health_gate test/test_health_gate.cpp)
  if(TARGET test_health_gate)
    target_link_libraries(test_health_gate smartcar_motion_gateway_core)
    target_include_directories(test_health_gate PRIVATE include)
  endif()
  ament_add_gtest(test_gateway_node test/test_gateway_node.cpp)
  if(TARGET test_gateway_node)
    target_link_libraries(test_gateway_node smartcar_motion_gateway_runtime)
    ament_target_dependencies(test_gateway_node geometry_msgs nav_msgs rclcpp std_srvs tf2_msgs)
    target_include_directories(test_gateway_node PRIVATE
      include ${CMAKE_CURRENT_SOURCE_DIR}/../smartcar_state_bridge/test)
  endif()
endif()

ament_export_dependencies(diagnostic_msgs geometry_msgs nav_msgs rclcpp
  sensor_msgs std_msgs std_srvs tf2_ros tf2_msgs smartcar_state_bridge)
ament_package()
