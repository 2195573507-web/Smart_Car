idf_component_register(
    SRCS
        "main.c"
        "radar/radar_parser.c"
        "radar/radar_frame_fifo.c"
        "radar/radar_telemetry_queue.c"
        "radar/radar_telemetry_observability.c"
        "radar/radar_uart.c"
        "radar/radar_uplink_protocol.c"
        "radar/radar_uplink_tx.c"
        "radar/radar_uplink.c"
    INCLUDE_DIRS "." "../../Common/SmartCarDebug"
    REQUIRES freertos log nvs_flash esp_driver_gpio esp_driver_ledc esp_driver_uart
             esp_event esp_netif esp_wifi lwip
             stm_uart smartcar_protocol smartcar_service s3_ble smartcar_log radar_control
             ros_motion_gateway smartcar_wifi_sta
)

set(_smartcar_telemetry_identity_sources
    "${CMAKE_CURRENT_LIST_DIR}/radar/radar_uplink.c"
    "${CMAKE_CURRENT_LIST_DIR}/radar/radar_telemetry_queue.c"
    "${CMAKE_CURRENT_LIST_DIR}/radar/radar_telemetry_queue.h"
    "${CMAKE_CURRENT_LIST_DIR}/radar/radar_telemetry_observability.c"
    "${CMAKE_CURRENT_LIST_DIR}/radar/radar_telemetry_observability.h")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
             ${_smartcar_telemetry_identity_sources})
set(_smartcar_telemetry_source_manifest "")
foreach(_smartcar_telemetry_source IN LISTS _smartcar_telemetry_identity_sources)
    file(SHA256 "${_smartcar_telemetry_source}" _smartcar_telemetry_file_sha)
    string(APPEND _smartcar_telemetry_source_manifest
           "${_smartcar_telemetry_file_sha}")
endforeach()
string(SHA256 _smartcar_telemetry_source_sha
       "${_smartcar_telemetry_source_manifest}")
string(SUBSTRING "${_smartcar_telemetry_source_sha}" 0 8
       _smartcar_telemetry_source_sha8)
target_compile_definitions(${COMPONENT_LIB} PRIVATE
    SMARTCAR_TELEMETRY_SOURCE_SHA8="${_smartcar_telemetry_source_sha8}")
message(STATUS "S3 telemetry source SHA8: ${_smartcar_telemetry_source_sha8}")

if(SMARTCAR_BMI323_DEBUG_ONLY)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE SMARTCAR_BMI323_DEBUG_ONLY=1)
else()
    target_compile_definitions(${COMPONENT_LIB} PRIVATE SMARTCAR_BMI323_DEBUG_ONLY=0)
endif()
