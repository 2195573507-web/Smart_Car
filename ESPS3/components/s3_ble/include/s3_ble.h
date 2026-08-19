#ifndef S3_BLE_H
#define S3_BLE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "smartcar_log.h"

/** Initialize the SmartCar_S3 Bluedroid GATT server and start advertising. */
esp_err_t s3_ble_init(void);

/** Send a notification only when GATT and TX Notify are both ready. */
esp_err_t s3_ble_notify_send(const uint8_t *data, uint16_t len);

/** Notify a complete independent log frame through FFE3 only. */
esp_err_t s3_ble_log_notify_send(const uint8_t *data, uint16_t len);

/** Build and notify one S3-originated FFE3 log record. */
esp_err_t s3_ble_log_emit(smartcar_log_level_t level, const char *text);

/* Unified S3-originated diagnostics. Records are delivered through FFE3. */
esp_err_t s3_log_info(const char *text);
esp_err_t s3_log_warn(const char *text);
esp_err_t s3_log_error(const char *text);

/** Return true only when the connected client has enabled TX Notify. */
bool s3_ble_is_ready(void);
bool s3_ble_is_log_ready(void);
uint32_t s3_ble_get_notify_fail_count(void);

typedef void (*s3_ble_ready_callback_t)(void *context);

/** Register a callback for the connected + Notify-enabled edge. */
esp_err_t s3_ble_set_ready_callback(s3_ble_ready_callback_t callback,
                                    void *context);

/* Compatibility wrapper; new callers must use s3_ble_notify_send(). */
esp_err_t s3_ble_send(const uint8_t *data, uint16_t len);

/**
 * Register the consumer for raw bytes written to the RX characteristic.
 *
 * The callback runs in the Bluedroid GATT event context. It must copy the
 * bytes before returning, avoid blocking, and only hand work to a task/queue;
 * the input pointer is not valid after the callback returns.
 */
typedef void (*s3_ble_rx_callback_t)(const uint8_t *data, size_t len, void *context);

/**
 * Register the sole FFE1 write consumer.
 *
 * Registration only selects the transport handoff target. The receiver must
 * remain valid for the life of the BLE service and must not parse frames or
 * control hardware from the callback context.
 */
esp_err_t s3_ble_register_rx_callback(s3_ble_rx_callback_t callback,
                                      void *context);

/* Compatibility wrapper for callers using the original callback API. */
esp_err_t s3_ble_set_rx_callback(s3_ble_rx_callback_t callback, void *context);

#endif /* S3_BLE_H */
