#ifndef S3_RADAR_UPLINK_TX_H
#define S3_RADAR_UPLINK_TX_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define RADAR_UPLINK_TX_MAX_SEND_CALLS 16U

typedef int (*radar_uplink_send_fn_t)(void *context,
                                      const uint8_t *data,
                                      size_t length);

typedef enum {
    RADAR_UPLINK_TX_COMPLETE = 0,
    RADAR_UPLINK_TX_WAIT,
    RADAR_UPLINK_TX_FAILED
} radar_uplink_tx_result_t;

typedef struct {
    size_t offset;
    uint32_t retry_count;
    bool wrote_partial;
} radar_uplink_tx_state_t;

void radar_uplink_tx_reset(radar_uplink_tx_state_t *state);

/* Advance one packet without blocking; transient backpressure preserves offset. */
radar_uplink_tx_result_t radar_uplink_tx_send(
    radar_uplink_tx_state_t *state,
    const uint8_t *data,
    size_t length,
    radar_uplink_send_fn_t send_fn,
    void *context);

#endif
