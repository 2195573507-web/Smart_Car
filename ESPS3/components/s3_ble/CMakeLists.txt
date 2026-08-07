idf_component_register(
    SRCS "s3_ble.c"
    INCLUDE_DIRS "include"
    REQUIRES bt freertos log smartcar_log
)
