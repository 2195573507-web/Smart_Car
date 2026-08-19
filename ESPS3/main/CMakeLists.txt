idf_component_register(
    SRCS
        "main.c"
        "radar/radar_uart.c"
    INCLUDE_DIRS "."
    REQUIRES freertos log nvs_flash esp_driver_gpio esp_driver_ledc
             stm_uart smartcar_protocol smartcar_service s3_ble smartcar_log radar_control
)

if(SMARTCAR_BMI323_DEBUG_ONLY)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE SMARTCAR_BMI323_DEBUG_ONLY=1)
else()
    target_compile_definitions(${COMPONENT_LIB} PRIVATE SMARTCAR_BMI323_DEBUG_ONLY=0)
endif()
