idf_component_register(
    SRCS "command_bridge.c" "radar_calibration_manager.c" "log_bridge.c"
    INCLUDE_DIRS "include" "../../../Common/SmartCarDebug"
    REQUIRES stm_uart smartcar_protocol smartcar_log s3_ble radar_control freertos log
)

if(SMARTCAR_BMI323_DEBUG_ONLY)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE SMARTCAR_BMI323_DEBUG_ONLY=1)
else()
    target_compile_definitions(${COMPONENT_LIB} PRIVATE SMARTCAR_BMI323_DEBUG_ONLY=0)
endif()
