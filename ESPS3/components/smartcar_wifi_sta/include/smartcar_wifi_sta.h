#ifndef SMARTCAR_WIFI_STA_H
#define SMARTCAR_WIFI_STA_H

#include <stdbool.h>

#include "esp_err.h"

/* Shared owner for the single S3 Wi-Fi STA used by radar uplink and ROS motion. */

/**
 * @brief Initialize and start the only SmartCar Wi-Fi STA instance.
 *
 * The SSID and password come exclusively from local ESP-IDF configuration.
 * This function never logs credentials or address configuration. It is
 * idempotent after a successful start and owns all ESP Wi-Fi setup calls.
 */
esp_err_t smartcar_wifi_sta_start(void);

/** @brief Return whether the shared STA currently has an IPv4 lease. */
bool smartcar_wifi_sta_is_connected(void);

#endif /* SMARTCAR_WIFI_STA_H */
