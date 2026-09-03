idf_component_register(
    SRCS "s3_ble.c" "s3_ble_log_tx.c"
    INCLUDE_DIRS "include"
    REQUIRES bt freertos log smartcar_log
)
