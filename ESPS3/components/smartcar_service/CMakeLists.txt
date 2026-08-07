idf_component_register(
    SRCS "radar_calibration_manager.c" "command_bridge.c" "log_bridge.c"
    INCLUDE_DIRS "include"
    REQUIRES stm_uart smartcar_protocol smartcar_log s3_ble radar_control freertos log
)
