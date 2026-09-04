idf_component_register(
    SRCS "ros_motion_protocol.c" "ros_motion_state.c" "ros_motion_stream.c"
         "ros_motion_gateway.c"
    INCLUDE_DIRS "include"
    REQUIRES freertos log lwip mbedtls smartcar_service smartcar_wifi_sta
)
