idf_component_register(
    SRCS "radar_calibration_manager.c" "command_bridge.c" "log_bridge.c"
    INCLUDE_DIRS "include"
    REQUIRES stm_uart smartcar_protocol smartcar_log s3_ble radar_control freertos log
)

if(SMARTCAR_BMI323_DEBUG_ONLY)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE SMARTCAR_BMI323_DEBUG_ONLY=1)
else()
    target_compile_definitions(${COMPONENT_LIB} PRIVATE SMARTCAR_BMI323_DEBUG_ONLY=0)
endif()
