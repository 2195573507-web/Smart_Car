idf_component_register(
    SRCS
        "main.c"
        "radar/radar_uart.c"
        "radar/radar_parser.c"
    INCLUDE_DIRS "." "radar"
    REQUIRES esp_driver_uart esp_driver_ledc esp_driver_gpio freertos log
)
