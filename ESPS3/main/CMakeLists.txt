idf_component_register(
    SRCS
        "main.c"
        "radar/radar_uart.c"
    INCLUDE_DIRS "."
    REQUIRES freertos log nvs_flash esp_driver_gpio esp_driver_ledc
             stm_uart smartcar_protocol smartcar_service s3_ble smartcar_log radar_control
)
