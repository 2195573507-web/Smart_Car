idf_component_register(
    SRCS
        "main.c"
        "radar/radar_parser.c"
        "radar/radar_frame_fifo.c"
        "radar/radar_uart.c"
        "radar/radar_uplink_protocol.c"
        "radar/radar_uplink_tx.c"
        "radar/radar_uplink.c"
    INCLUDE_DIRS "."
    REQUIRES freertos log nvs_flash esp_driver_gpio esp_driver_ledc esp_driver_uart
             esp_event esp_netif esp_wifi lwip
             stm_uart smartcar_protocol smartcar_service s3_ble smartcar_log radar_control
)

if(SMARTCAR_BMI323_DEBUG_ONLY)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE SMARTCAR_BMI323_DEBUG_ONLY=1)
else()
    target_compile_definitions(${COMPONENT_LIB} PRIVATE SMARTCAR_BMI323_DEBUG_ONLY=0)
endif()
