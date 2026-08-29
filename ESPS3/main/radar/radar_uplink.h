#ifndef S3_RADAR_UPLINK_H
#define S3_RADAR_UPLINK_H

#include <stdbool.h>

#include "esp_err.h"

/* Starts the optional Wi-Fi STA and low-priority radar uplink task. */
esp_err_t radar_uplink_init(void);
bool radar_uplink_is_running(void);

#endif
