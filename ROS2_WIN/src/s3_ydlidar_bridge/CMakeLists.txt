cmake_minimum_required(VERSION 3.8)
project(s3_ydlidar_bridge)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Threads REQUIRED)

set(YDLIDAR_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/third_party/ydlidar_sdk)
set(YDLIDAR_GENERATED_DIR ${CMAKE_CURRENT_BINARY_DIR}/generated)
file(MAKE_DIRECTORY ${YDLIDAR_GENERATED_DIR})
set(YDLIDAR_SDK_VERSION_MAJOR 1)
set(YDLIDAR_SDK_VERSION_MINOR 0)
set(YDLIDAR_SDK_VERSION_PATCH 6)
set(YDLIDAR_SDK_VERSION
    ${YDLIDAR_SDK_VERSION_MAJOR}.${YDLIDAR_SDK_VERSION_MINOR}.${YDLIDAR_SDK_VERSION_PATCH})
configure_file(${YDLIDAR_ROOT}/ydlidar_config.h.in
               ${YDLIDAR_GENERATED_DIR}/ydlidar_config.h @ONLY)

set(YDLIDAR_SDK_SOURCES
  ${YDLIDAR_ROOT}/core/base/timer.cpp
  ${YDLIDAR_ROOT}/core/common/ydlidar_def.cpp
  ${YDLIDAR_ROOT}/core/network/ActiveSocket.cpp
  ${YDLIDAR_ROOT}/core/network/SimpleSocket.cpp
  ${YDLIDAR_ROOT}/core/serial/serial.cpp
  ${YDLIDAR_ROOT}/core/serial/impl/unix/unix_serial.cpp
  ${YDLIDAR_ROOT}/core/serial/impl/unix/list_ports_linux.cpp
  ${YDLIDAR_ROOT}/core/serial/impl/unix/lock.c
  ${YDLIDAR_ROOT}/src/ydlidar_driver.cpp
)

add_library(ydlidar_sdk STATIC ${YDLIDAR_SDK_SOURCES})
target_include_directories(ydlidar_sdk PUBLIC
  ${YDLIDAR_ROOT}
  ${YDLIDAR_ROOT}/core
  ${YDLIDAR_ROOT}/src
  ${YDLIDAR_ROOT}/core/serial/impl/unix
  ${YDLIDAR_GENERATED_DIR}
)
target_link_libraries(ydlidar_sdk PUBLIC Threads::Threads)
if(UNIX AND NOT APPLE)
  # The official parser uses atan()/other libm symbols on Linux.
  target_link_libraries(ydlidar_sdk PUBLIC m)
endif()

add_library(s3_ydlidar_bridge_core STATIC
  src/framing.cpp
  src/transport.cpp
  src/official_decoder.cpp
  src/scan_mapper.cpp
)
target_include_directories(s3_ydlidar_bridge_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${YDLIDAR_ROOT}>
  $<BUILD_INTERFACE:${YDLIDAR_ROOT}/core>
  $<BUILD_INTERFACE:${YDLIDAR_ROOT}/src>
  $<BUILD_INTERFACE:${YDLIDAR_GENERATED_DIR}>
)
ament_target_dependencies(s3_ydlidar_bridge_core
  builtin_interfaces diagnostic_msgs rclcpp sensor_msgs)
target_link_libraries(s3_ydlidar_bridge_core ydlidar_sdk)

add_executable(s3_ydlidar_bridge_node src/main.cpp src/bridge_node.cpp)
target_include_directories(s3_ydlidar_bridge_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include)
ament_target_dependencies(s3_ydlidar_bridge_node diagnostic_msgs rclcpp sensor_msgs)
target_link_libraries(s3_ydlidar_bridge_node s3_ydlidar_bridge_core)

install(TARGETS s3_ydlidar_bridge_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/
  DESTINATION include)
install(DIRECTORY config launch rviz rqt
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  ament_add_gtest(test_framing test/test_framing.cpp)
  if(TARGET test_framing)
    target_include_directories(test_framing PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include)
    target_link_libraries(test_framing s3_ydlidar_bridge_core)
  endif()

  ament_add_gtest(test_official_decoder test/test_official_decoder.cpp)
  if(TARGET test_official_decoder)
    target_include_directories(test_official_decoder PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include
      ${YDLIDAR_ROOT}/core
      ${YDLIDAR_ROOT}/src)
    target_link_libraries(test_official_decoder s3_ydlidar_bridge_core)
  endif()

  ament_add_gtest(test_scan_mapper test/test_scan_mapper.cpp)
  if(TARGET test_scan_mapper)
    target_include_directories(test_scan_mapper PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include
      ${YDLIDAR_ROOT}/core
      ${YDLIDAR_ROOT}/src)
    target_link_libraries(test_scan_mapper s3_ydlidar_bridge_core)
  endif()

  ament_add_gtest(test_transport test/test_transport.cpp)
  if(TARGET test_transport)
    target_include_directories(test_transport PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include)
    target_link_libraries(test_transport s3_ydlidar_bridge_core)
  endif()

  ament_add_gtest(test_s3_protocol test/test_s3_protocol.cpp)
  if(TARGET test_s3_protocol)
    target_include_directories(test_s3_protocol PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include
      ${YDLIDAR_ROOT}/core
      ${YDLIDAR_ROOT}/src)
    target_link_libraries(test_s3_protocol s3_ydlidar_bridge_core)
  endif()

  ament_add_gtest(test_tcp_server test/test_tcp_server.cpp)
  if(TARGET test_tcp_server)
    target_include_directories(test_tcp_server PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include)
    target_link_libraries(test_tcp_server s3_ydlidar_bridge_core)
  endif()
endif()

ament_export_dependencies(builtin_interfaces diagnostic_msgs rclcpp sensor_msgs)
ament_package()
