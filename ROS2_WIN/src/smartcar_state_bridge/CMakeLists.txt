cmake_minimum_required(VERSION 3.8)
project(smartcar_state_bridge)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)

# The Humble binary also exports the controller plugin, which currently has a
# missing librsl.so dependency in the lean runtime image.  Resolve only its
# public header and compile the upstream Odometry source subset below; this
# keeps the read-only adapter independent of the controller/plugin library.
find_path(DIFF_DRIVE_CONTROLLER_INCLUDE_DIR
  NAMES diff_drive_controller/odometry.hpp
  PATH_SUFFIXES include/diff_drive_controller
  REQUIRED)

add_library(smartcar_state_bridge_core STATIC
  src/chassis_state.cpp
  src/odom_message.cpp
  src/srp_v4_chassis.cpp
  src/state_adapter.cpp
  src/telemetry_decoder.cpp
  src/wheel_odom.cpp
  third_party/diff_drive_odometry/odometry.cpp
)
target_include_directories(smartcar_state_bridge_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${DIFF_DRIVE_CONTROLLER_INCLUDE_DIR})
ament_target_dependencies(smartcar_state_bridge_core geometry_msgs nav_msgs rclcpp)

add_executable(smartcar_state_bridge_node src/state_node.cpp)
target_include_directories(smartcar_state_bridge_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include)
ament_target_dependencies(smartcar_state_bridge_node
  diagnostic_msgs geometry_msgs nav_msgs rclcpp tf2_ros)
target_link_libraries(smartcar_state_bridge_node smartcar_state_bridge_core)

install(TARGETS smartcar_state_bridge_core
  EXPORT smartcar_state_bridge_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)
install(TARGETS smartcar_state_bridge_node
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY third_party/diff_drive_odometry
  DESTINATION share/${PROJECT_NAME}/third_party)
install(EXPORT smartcar_state_bridge_targets
  FILE smartcar_state_bridge_targets.cmake
  NAMESPACE smartcar_state_bridge::
  DESTINATION share/${PROJECT_NAME}/cmake)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_telemetry_decoder test/test_telemetry_decoder.cpp)
  if(TARGET test_telemetry_decoder)
    target_include_directories(test_telemetry_decoder PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include)
    target_link_libraries(test_telemetry_decoder smartcar_state_bridge_core)
  endif()

  ament_add_gtest(test_wheel_odom test/test_wheel_odom.cpp)
  if(TARGET test_wheel_odom)
    target_include_directories(test_wheel_odom PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include)
    target_link_libraries(test_wheel_odom smartcar_state_bridge_core)
  endif()

  ament_add_gtest(test_state_adapter test/test_state_adapter.cpp)
  if(TARGET test_state_adapter)
    target_include_directories(test_state_adapter PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include)
    target_link_libraries(test_state_adapter smartcar_state_bridge_core)
  endif()

  ament_add_gtest(test_srp_v4_chassis test/test_srp_v4_chassis.cpp)
  if(TARGET test_srp_v4_chassis)
    target_include_directories(test_srp_v4_chassis PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include
      ${CMAKE_CURRENT_SOURCE_DIR}/test)
    target_link_libraries(test_srp_v4_chassis smartcar_state_bridge_core)
  endif()

  ament_add_gtest(test_chassis_state test/test_chassis_state.cpp)
  if(TARGET test_chassis_state)
    target_include_directories(test_chassis_state PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include
      ${CMAKE_CURRENT_SOURCE_DIR}/test)
    target_link_libraries(test_chassis_state smartcar_state_bridge_core)
  endif()

  ament_add_gtest(test_odom_message test/test_odom_message.cpp)
  if(TARGET test_odom_message)
    target_include_directories(test_odom_message PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include)
    target_link_libraries(test_odom_message smartcar_state_bridge_core)
  endif()
endif()

ament_export_targets(smartcar_state_bridge_targets HAS_LIBRARY_TARGET)
ament_export_dependencies(diagnostic_msgs geometry_msgs nav_msgs rclcpp tf2_ros)
ament_package()
