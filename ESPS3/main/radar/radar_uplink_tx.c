#include "radar_uplink_tx.h"

#include <errno.h>

void radar_uplink_tx_reset(radar_uplink_tx_state_t *state)
{
    if (state != NULL) {
        state->offset = 0U;
        state->retry_count = 0U;
        state->wrote_partial = false;
    }
}

radar_uplink_tx_result_t radar_uplink_tx_send(
    radar_uplink_tx_state_t *state,
    const uint8_t *data,
    size_t length,
    radar_uplink_send_fn_t send_fn,
    void *context)
{
    if (state == NULL || data == NULL || length == 0U || send_fn == NULL ||
        state->offset > length) {
        return RADAR_UPLINK_TX_FAILED;
    }

    state->wrote_partial = false;
    size_t send_calls = 0U;
    while (state->offset < length &&
           send_calls < RADAR_UPLINK_TX_MAX_SEND_CALLS) {
        ++send_calls;
        const size_t remaining = length - state->offset;
        const int written = send_fn(context, &data[state->offset], remaining);
        if (written > 0) {
            if ((size_t)written > remaining) {
                return RADAR_UPLINK_TX_FAILED;
            }
            if ((size_t)written < remaining) {
                state->wrote_partial = true;
            }
            state->offset += (size_t)written;
            continue;
        }
        if (written < 0 && (errno == EAGAIN || errno == EWOULDBLOCK ||
                            errno == EINTR)) {
            ++state->retry_count;
            return RADAR_UPLINK_TX_WAIT;
        }
        return RADAR_UPLINK_TX_FAILED;
    }
    if (state->offset < length) {
        ++state->retry_count;
        return RADAR_UPLINK_TX_WAIT;
    }
    return RADAR_UPLINK_TX_COMPLETE;
}
