#include <assert.h>
#include <errno.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "radar_uplink_tx.h"

typedef struct {
    const int *results;
    size_t result_count;
    size_t result_index;
    uint8_t received[32];
    size_t received_length;
} mock_sender_t;

static int mock_send(void *context, const uint8_t *data, size_t length)
{
    mock_sender_t *sender = context;
    assert(sender->result_index < sender->result_count);
    const int result = sender->results[sender->result_index++];
    if (result < 0) {
        errno = EAGAIN;
        return result;
    }
    assert((size_t)result <= length);
    assert(sender->received_length + (size_t)result <= sizeof(sender->received));
    memcpy(&sender->received[sender->received_length], data, (size_t)result);
    sender->received_length += (size_t)result;
    return result;
}

static void test_partial_write_and_eagain_preserve_packet_offset(void)
{
    static const int results[] = {3, -1, 7};
    static const uint8_t packet[] = {0U, 1U, 2U, 3U, 4U,
                                     5U, 6U, 7U, 8U, 9U};
    mock_sender_t sender = {
        .results = results,
        .result_count = sizeof(results) / sizeof(results[0]),
    };
    radar_uplink_tx_state_t state;
    radar_uplink_tx_reset(&state);

    assert(radar_uplink_tx_send(&state,
                                packet,
                                sizeof(packet),
                                mock_send,
                                &sender) == RADAR_UPLINK_TX_WAIT);
    assert(state.offset == 3U);
    assert(state.retry_count == 1U);
    assert(state.wrote_partial);
    assert(sender.received_length == 3U);

    assert(radar_uplink_tx_send(&state,
                                packet,
                                sizeof(packet),
                                mock_send,
                                &sender) == RADAR_UPLINK_TX_COMPLETE);
    assert(state.offset == sizeof(packet));
    assert(state.retry_count == 1U);
    assert(state.wrote_partial == false);
    assert(sender.received_length == sizeof(packet));
    assert(memcmp(sender.received, packet, sizeof(packet)) == 0);
}

static int failing_send(void *context, const uint8_t *data, size_t length)
{
    (void)context;
    (void)data;
    (void)length;
    errno = EPIPE;
    return -1;
}

static void test_permanent_send_failure_is_reported(void)
{
    static const uint8_t packet[] = {1U};
    radar_uplink_tx_state_t state;
    radar_uplink_tx_reset(&state);

    assert(radar_uplink_tx_send(&state,
                                packet,
                                sizeof(packet),
                                failing_send,
                                NULL) == RADAR_UPLINK_TX_FAILED);
    assert(state.offset == 0U);
}

static int one_byte_send(void *context, const uint8_t *data, size_t length)
{
    size_t *calls = context;
    (void)data;
    assert(length > 0U);
    ++*calls;
    return 1;
}

static void test_send_call_budget_is_bounded(void)
{
    static const uint8_t packet[32] = {0};
    size_t calls = 0U;
    radar_uplink_tx_state_t state;
    radar_uplink_tx_reset(&state);

    assert(radar_uplink_tx_send(&state,
                                packet,
                                sizeof(packet),
                                one_byte_send,
                                &calls) == RADAR_UPLINK_TX_WAIT);
    assert(calls == RADAR_UPLINK_TX_MAX_SEND_CALLS);
    assert(state.offset == RADAR_UPLINK_TX_MAX_SEND_CALLS);
    assert(state.retry_count == 1U);

    assert(radar_uplink_tx_send(&state,
                                packet,
                                sizeof(packet),
                                one_byte_send,
                                &calls) == RADAR_UPLINK_TX_COMPLETE);
    assert(calls == sizeof(packet));
    assert(state.offset == sizeof(packet));
}

int main(void)
{
    test_partial_write_and_eagain_preserve_packet_offset();
    test_permanent_send_failure_is_reported();
    test_send_call_budget_is_bounded();
    return 0;
}
