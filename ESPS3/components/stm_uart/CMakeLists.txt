idf_component_register(
    SRCS "stm_uart.c"
    INCLUDE_DIRS "include" "../../../Common/SRP/include"
    REQUIRES esp_driver_gpio esp_driver_uart freertos log smartcar_protocol
    PRIV_REQUIRES esp_timer
)
