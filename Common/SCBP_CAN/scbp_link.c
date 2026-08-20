#include "scbp_link.h"

#include <string.h>

static uint8_t valid_local_node(uint8_t node)
{
    return node == SCBP_NODE_STM32H757 || node == SCBP_NODE_ESP32_S3;
}

static uint16_t link_health_max(const scbp_link_t *link)
{
    return link->tec > link->rec ? link->tec : link->rec;
}

static scbp_link_state_t link_state_from_counters(const scbp_link_t *link)
{
    const uint16_t maximum = link_health_max(link);
    if (maximum >= UINT16_C(256)) {
        return SCBP_LINK_BUS_OFF;
    }
    if (maximum >= UINT16_C(128)) {
        return SCBP_LINK_PASSIVE;
    }
    if (maximum >= UINT16_C(32)) {
        return SCBP_LINK_WARNING;
    }
    return SCBP_LINK_ACTIVE;
}

static uint16_t counter_add8(uint16_t value)
{
    return value > UINT16_MAX - UINT16_C(8) ? UINT16_MAX :
           (uint16_t)(value + UINT16_C(8));
}

static void counter_decrement(uint16_t *value)
{
    if (value != NULL && *value != 0U) {
        --(*value);
    }
}

static void link_release_pending(scbp_link_t *link, scbp_link_tx_result_t result,
                                 uint8_t status_code)
{
    for (uint8_t index = 0U; index < SCBP_LINK_PENDING_SLOTS; ++index) {
        scbp_link_pending_t *pending = &link->pending[index];
        if (pending->in_use != 0U) {
            scbp_link_tx_callback_t callback = pending->callback;
            void *callback_context = pending->callback_context;
            pending->in_use = 0U;
            if (callback != NULL) {
                callback(result, status_code, callback_context);
            }
        }
    }
}

static void link_refresh_state(scbp_link_t *link)
{
    const scbp_link_state_t next = link_state_from_counters(link);
    if (link->state == SCBP_LINK_BUS_OFF) {
        return;
    }
    if (next != SCBP_LINK_BUS_OFF) {
        link->state = next;
        return;
    }

    link->state = SCBP_LINK_BUS_OFF;
    link_release_pending(link, SCBP_LINK_TX_BUS_OFF, SCBP_FAST_RESP_TIMEOUT);
    if (link->config.on_bus_off != NULL) {
        link->config.on_bus_off(link->config.context);
    }
}

static void link_add_tec(scbp_link_t *link)
{
    link->tec = counter_add8(link->tec);
    link_refresh_state(link);
}

static void link_add_rec(scbp_link_t *link)
{
    link->rec = counter_add8(link->rec);
    link_refresh_state(link);
}

static int link_transport_send(scbp_link_t *link, const uint8_t *data,
                               uint16_t length)
{
    if (link->config.transport_send == NULL ||
        link->config.transport_send(data, length, link->config.context) != 0) {
        link_add_tec(link);
        return -1;
    }
    return 0;
}

static scbp_link_pending_t *link_find_free_pending(scbp_link_t *link)
{
    for (uint8_t index = 0U; index < SCBP_LINK_PENDING_SLOTS; ++index) {
        if (link->pending[index].in_use == 0U) {
            return &link->pending[index];
        }
    }
    return NULL;
}

static void write_u16_le(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)(value & UINT16_C(0x00FF));
    data[1] = (uint8_t)(value >> 8U);
}

static uint16_t read_u16_le(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8U);
}

void scbp_link_init(scbp_link_t *link, const scbp_link_config_t *config)
{
    if (link == NULL) {
        return;
    }
    (void)memset(link, 0, sizeof(*link));
    if (config != NULL) {
        link->config = *config;
    }
    if (link->config.ack_timeout_ms == 0U) {
        link->config.ack_timeout_ms = SCBP_LINK_ACK_TIMEOUT_MS;
    }
    if (link->config.max_retries == 0U) {
        link->config.max_retries = SCBP_LINK_MAX_RETRIES;
    }
    link->state = SCBP_LINK_ACTIVE;
}

int scbp_link_send(scbp_link_t *link, uint8_t priority, uint8_t destination,
                   uint16_t message_id, uint8_t flags, const uint8_t *payload,
                   uint8_t length, uint32_t now_ms,
                   scbp_link_tx_callback_t callback, void *callback_context)
{
    scbp_can_frame_t frame;
    scbp_link_pending_t *pending = NULL;
    uint8_t frame_bytes[SCBP_CAN_MAX_FRAME_SIZE];
    uint16_t frame_length = 0U;

    if (link == NULL || valid_local_node(link->config.local_node) == 0U ||
        priority > SCBP_CAN_PRIORITY_DEBUG ||
        destination < SCBP_NODE_STM32H757 || destination > SCBP_NODE_BROADCAST ||
        message_id > SCBP_CAN_MESSAGE_MASK ||
        (payload == NULL && length != 0U) ||
        (flags & SCBP_CAN_FLAG_RESERVED_MASK) != 0U) {
        return -1;
    }
    if (link->state == SCBP_LINK_BUS_OFF) {
        if (callback != NULL) {
            callback(SCBP_LINK_TX_BUS_OFF, SCBP_FAST_RESP_TIMEOUT, callback_context);
        }
        return -2;
    }
    if ((flags & SCBP_CAN_FLAG_ACK_REQUIRED) != 0U) {
        pending = link_find_free_pending(link);
        if (pending == NULL) {
            return -3;
        }
    }

    frame.can_id = SCBP_CAN_ID(priority, link->config.local_node, destination,
                               message_id);
    frame.flags = flags;
    frame.sequence = link->next_sequence;
    frame.length = length;
    frame.payload = payload;
    link->next_sequence = (uint8_t)(link->next_sequence + 1U);
    if (scbp_can_encode(&frame, frame_bytes, sizeof(frame_bytes), &frame_length) != 0) {
        return -4;
    }

    if (pending != NULL) {
        (void)memset(pending, 0, sizeof(*pending));
        pending->in_use = 1U;
        pending->sequence = frame.sequence;
        pending->can_id = frame.can_id;
        pending->frame_length = frame_length;
        pending->last_tx_ms = now_ms;
        pending->callback = callback;
        pending->callback_context = callback_context;
        (void)memcpy(pending->frame_bytes, frame_bytes, frame_length);
    }

    if (link_transport_send(link, frame_bytes, frame_length) != 0) {
        if (pending != NULL && pending->in_use != 0U) {
            pending->in_use = 0U;
            if (callback != NULL) {
                callback(SCBP_LINK_TX_TRANSPORT_FAILURE, SCBP_FAST_RESP_TIMEOUT,
                         callback_context);
            }
        } else if (pending == NULL && callback != NULL) {
            callback(SCBP_LINK_TX_TRANSPORT_FAILURE, SCBP_FAST_RESP_TIMEOUT,
                     callback_context);
        }
        return -5;
    }
    return 0;
}

int scbp_link_send_fast_response(scbp_link_t *link, uint8_t priority,
                                 uint8_t destination, uint8_t is_error,
                                 uint16_t ack_can_id, uint8_t ack_sequence,
                                 uint8_t status_code, uint32_t now_ms)
{
    uint8_t payload[sizeof(scbp_fast_resp_payload_t)];
    const uint16_t message_id = is_error != 0U ? SCBP_MSG_ID_ERROR : SCBP_MSG_ID_ACK;
    const uint8_t flags = is_error != 0U ? SCBP_CAN_FLAG_IS_ERROR :
                                            SCBP_CAN_FLAG_IS_ACK;

    write_u16_le(payload, ack_can_id);
    payload[2] = ack_sequence;
    payload[3] = status_code;
    return scbp_link_send(link, priority, destination, message_id, flags, payload,
                          (uint8_t)sizeof(payload), now_ms, NULL, NULL);
}

void scbp_link_receive(scbp_link_t *link, const scbp_can_frame_t *frame)
{
    const uint16_t message_id = frame == NULL ? 0U : SCBP_CAN_ID_MESSAGE(frame->can_id);
    const uint8_t response = frame == NULL ? 0U :
        (uint8_t)((frame->flags & (SCBP_CAN_FLAG_IS_ACK | SCBP_CAN_FLAG_IS_ERROR)) != 0U);

    if (link == NULL || frame == NULL) {
        return;
    }
    counter_decrement(&link->rec);
    link_refresh_state(link);
    if (response != 0U &&
        (message_id == SCBP_MSG_ID_ACK || message_id == SCBP_MSG_ID_ERROR) &&
        frame->length == sizeof(scbp_fast_resp_payload_t) && frame->payload != NULL) {
        const uint16_t acknowledged_can_id = read_u16_le(frame->payload);
        const uint8_t acknowledged_sequence = frame->payload[2];
        const uint8_t status_code = frame->payload[3];
        for (uint8_t index = 0U; index < SCBP_LINK_PENDING_SLOTS; ++index) {
            scbp_link_pending_t *pending = &link->pending[index];
            if (pending->in_use != 0U && pending->can_id == acknowledged_can_id &&
                pending->sequence == acknowledged_sequence) {
                scbp_link_tx_callback_t callback = pending->callback;
                void *callback_context = pending->callback_context;
                pending->in_use = 0U;
                if (message_id == SCBP_MSG_ID_ACK &&
                    (frame->flags & SCBP_CAN_FLAG_IS_ACK) != 0U &&
                    status_code == SCBP_FAST_RESP_OK) {
                    counter_decrement(&link->tec);
                    link_refresh_state(link);
                    if (callback != NULL) {
                        callback(SCBP_LINK_TX_OK, status_code, callback_context);
                    }
                } else if (callback != NULL) {
                    callback(SCBP_LINK_TX_REMOTE_ERROR, status_code, callback_context);
                }
                return;
            }
        }
        return;
    }
    if (link->config.on_frame != NULL) {
        link->config.on_frame(frame, link->config.context);
    }
}

void scbp_link_report_parser_error(scbp_link_t *link, scbp_parser_error_t error)
{
    if (link == NULL) {
        return;
    }
    if (error == SCBP_PARSER_ERROR_HCS || error == SCBP_PARSER_ERROR_FCS) {
        link_add_rec(link);
    }
}

void scbp_link_tick(scbp_link_t *link, uint32_t now_ms)
{
    if (link == NULL || link->state == SCBP_LINK_BUS_OFF) {
        return;
    }
    for (uint8_t index = 0U; index < SCBP_LINK_PENDING_SLOTS; ++index) {
        scbp_link_pending_t *pending = &link->pending[index];
        if (pending->in_use == 0U ||
            (uint32_t)(now_ms - pending->last_tx_ms) < link->config.ack_timeout_ms) {
            continue;
        }
        if (pending->retry_count < link->config.max_retries) {
            ++pending->retry_count;
            pending->last_tx_ms = now_ms;
            if (link_transport_send(link, pending->frame_bytes, pending->frame_length) != 0) {
                if (pending->in_use != 0U) {
                    scbp_link_tx_callback_t callback = pending->callback;
                    void *callback_context = pending->callback_context;
                    pending->in_use = 0U;
                    if (callback != NULL) {
                        callback(SCBP_LINK_TX_TRANSPORT_FAILURE,
                                 SCBP_FAST_RESP_TIMEOUT, callback_context);
                    }
                }
            }
            continue;
        }

        {
            scbp_link_tx_callback_t callback = pending->callback;
            void *callback_context = pending->callback_context;

            pending->in_use = 0U;
            link_add_tec(link);
            if (callback != NULL) {
                callback(link->state == SCBP_LINK_BUS_OFF ? SCBP_LINK_TX_BUS_OFF :
                                                          SCBP_LINK_TX_TIMEOUT,
                         SCBP_FAST_RESP_TIMEOUT, callback_context);
            }
        }
    }
}

void scbp_link_recover(scbp_link_t *link)
{
    if (link == NULL) {
        return;
    }
    link_release_pending(link, SCBP_LINK_TX_BUS_OFF, SCBP_FAST_RESP_TIMEOUT);
    link->tec = 0U;
    link->rec = 0U;
    link->state = SCBP_LINK_ACTIVE;
}

uint16_t scbp_link_get_tec(const scbp_link_t *link)
{
    return link == NULL ? 0U : link->tec;
}

uint16_t scbp_link_get_rec(const scbp_link_t *link)
{
    return link == NULL ? 0U : link->rec;
}

scbp_link_state_t scbp_link_get_state(const scbp_link_t *link)
{
    return link == NULL ? SCBP_LINK_BUS_OFF : link->state;
}
