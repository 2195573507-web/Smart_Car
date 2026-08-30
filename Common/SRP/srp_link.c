#include "srp_link.h"

#include <string.h>

static void update_state(srp_link_t *link)
{
    const uint16_t score = link->tec > link->rec ? link->tec : link->rec;

    link->state = score >= 256U ? SRP_LINK_BUS_OFF
                                : score >= 128U ? SRP_LINK_PASSIVE
                                : score >= 32U ? SRP_LINK_WARNING
                                               : SRP_LINK_ACTIVE;
}

static void add_rec(srp_link_t *link, uint16_t value)
{
    link->rec = link->rec > UINT16_MAX - value ? UINT16_MAX
                                               : (uint16_t)(link->rec + value);
    update_state(link);
}

static void add_tec(srp_link_t *link, uint16_t value)
{
    link->tec = link->tec > UINT16_MAX - value ? UINT16_MAX
                                               : (uint16_t)(link->tec + value);
    update_state(link);
}

static void reduce_rec(srp_link_t *link)
{
    if (link->rec != 0U) {
        --link->rec;
        update_state(link);
    }
}

static void reduce_tec(srp_link_t *link)
{
    if (link->tec != 0U) {
        --link->tec;
        update_state(link);
    }
}

static srp_link_pending_t *find_pending(srp_link_t *link, uint8_t type,
                                        uint8_t sequence)
{
    for (size_t index = 0U; index < SRP_LINK_PENDING_SLOTS; ++index) {
        srp_link_pending_t *pending = &link->pending[index];
        if (pending->in_use != 0U && pending->type == type &&
            pending->sequence == sequence) {
            return pending;
        }
    }
    return NULL;
}

static srp_link_pending_t *reserve_pending(srp_link_t *link)
{
    for (size_t index = 0U; index < SRP_LINK_PENDING_SLOTS; ++index) {
        if (link->pending[index].in_use == 0U) {
            return &link->pending[index];
        }
    }
    return NULL;
}

void srp_link_init(srp_link_t *link, const srp_link_config_t *config)
{
    if (link == NULL) {
        return;
    }
    (void)memset(link, 0, sizeof(*link));
    if (config != NULL) {
        link->config = *config;
    }
    if (link->config.ack_timeout_ms == 0U) {
        link->config.ack_timeout_ms = SRP_LINK_ACK_TIMEOUT_MS;
    }
    if (link->config.max_retries == 0U) {
        link->config.max_retries = SRP_LINK_MAX_RETRIES;
    }
    link->state = SRP_LINK_ACTIVE;
}

int srp_link_send(srp_link_t *link, uint8_t priority, uint8_t destination,
                  uint16_t type, uint8_t flags, const uint8_t *payload,
                  uint16_t length, uint32_t now_ms,
                  srp_link_tx_callback_t callback, void *callback_context)
{
    srp_frame_t frame;
    srp_link_pending_t *pending = NULL;
    uint8_t *frame_bytes;
    uint16_t encoded_length = 0U;
    uint8_t sequence;
    int result;

    (void)destination;
    if (link == NULL || link->config.transport_send == NULL || type > UINT8_MAX ||
        length > SRP_MAX_PAYLOAD || (flags & SRP_FLAG_RESERVED_MASK) != 0U) {
        return -1;
    }
    if ((flags & SRP_FLAG_ACK_REQUIRED) != 0U) {
        pending = reserve_pending(link);
        if (pending == NULL) {
            return -2;
        }
    }
    sequence = link->next_sequence++;
    frame.priority = priority;
    frame.type = (uint8_t)type;
    frame.sequence = sequence;
    frame.flags = flags;
    frame.length = length;
    frame.payload = payload;
    frame_bytes = pending != NULL ? pending->frame_bytes : link->tx_scratch;
    result = srp_encode_frame(&frame, frame_bytes, SRP_MAX_FRAME_SIZE,
                              &encoded_length);
    if (result != SRP_CODEC_OK) {
        return -3;
    }
    if (pending != NULL) {
        pending->in_use = 1U;
        pending->retry_count = 0U;
        pending->type = (uint8_t)type;
        pending->sequence = sequence;
        pending->frame_length = encoded_length;
        pending->last_tx_ms = now_ms;
        pending->callback = callback;
        pending->callback_context = callback_context;
        result = link->config.transport_send(frame_bytes, encoded_length,
                                              link->config.context);
    } else {
        result = link->config.transport_send(frame_bytes, encoded_length,
                                             link->config.context);
    }
    if (result != 0) {
        if (pending != NULL) {
            pending->in_use = 0U;
        }
        add_tec(link, 8U);
        return -4;
    }
    return 0;
}

int srp_link_send_fast_response(srp_link_t *link, uint8_t priority,
                                uint8_t destination, uint8_t is_error,
                                uint16_t ack_type, uint8_t ack_sequence,
                                uint8_t status_code, uint32_t now_ms)
{
    uint8_t payload[SRP_PAYLOAD_FAST_RESPONSE_SIZE] = {
        (uint8_t)ack_type, 0U, ack_sequence, status_code};
    const uint8_t type = is_error != 0U ? SRP_MSG_ID_ERROR : SRP_MSG_ID_ACK;

    (void)now_ms;
    return srp_link_send(link, priority, destination, type,
                         (uint8_t)(is_error != 0U ? SRP_FLAG_ERROR : SRP_FLAG_ACK),
                         payload, sizeof(payload), now_ms, NULL, NULL);
}

void srp_link_cancel_message(srp_link_t *link, uint16_t type)
{
    if (link == NULL) {
        return;
    }
    for (size_t index = 0U; index < SRP_LINK_PENDING_SLOTS; ++index) {
        if (link->pending[index].in_use != 0U &&
            link->pending[index].type == (uint8_t)type) {
            link->pending[index].in_use = 0U;
        }
    }
}

void srp_link_receive(srp_link_t *link, const srp_frame_t *frame)
{
    srp_frame_t local;
    srp_link_pending_t *pending;
    uint8_t ack_type;
    uint8_t ack_sequence;
    uint8_t status_code;

    if (link == NULL || frame == NULL) {
        return;
    }
    reduce_rec(link);
    if (frame->type == SRP_MSG_ID_ACK || frame->type == SRP_MSG_ID_ERROR) {
        if (frame->length < SRP_PAYLOAD_FAST_RESPONSE_SIZE || frame->payload == NULL) {
            add_rec(link, 8U);
            return;
        }
        ack_type = frame->payload[0];
        ack_sequence = frame->payload[2];
        status_code = frame->payload[3];
        pending = find_pending(link, ack_type, ack_sequence);
        if (pending == NULL) {
            return;
        }
        pending->in_use = 0U;
        if (frame->type == SRP_MSG_ID_ACK && status_code == SRP_FAST_RESP_OK) {
            reduce_tec(link);
            if (pending->callback != NULL) {
                pending->callback(SRP_LINK_TX_OK, status_code,
                                  pending->callback_context);
            }
        } else {
            if (pending->callback != NULL) {
                pending->callback(SRP_LINK_TX_REMOTE_ERROR, status_code,
                                  pending->callback_context);
            }
        }
        return;
    }
    local = *frame;
    if (link->config.on_frame != NULL) {
        link->config.on_frame(&local, link->config.context);
    }
}

void srp_link_report_parser_error(srp_link_t *link, srp_parser_error_t error)
{
    if (link == NULL) {
        return;
    }
    if (error == SRP_PARSER_ERROR_CRC || error == SRP_PARSER_ERROR_EOF) {
        add_rec(link, 8U);
    } else {
        add_rec(link, 1U);
    }
}

void srp_link_tick(srp_link_t *link, uint32_t now_ms)
{
    if (link == NULL || link->config.transport_send == NULL) {
        return;
    }
    for (size_t index = 0U; index < SRP_LINK_PENDING_SLOTS; ++index) {
        srp_link_pending_t *pending = &link->pending[index];
        if (pending->in_use == 0U ||
            (uint32_t)(now_ms - pending->last_tx_ms) < link->config.ack_timeout_ms) {
            continue;
        }
        if (pending->retry_count < link->config.max_retries) {
            if (link->config.transport_send(pending->frame_bytes,
                                             pending->frame_length,
                                             link->config.context) == 0) {
                ++pending->retry_count;
                pending->last_tx_ms = now_ms;
            } else {
                add_tec(link, 8U);
            }
        } else {
            pending->in_use = 0U;
            add_tec(link, 8U);
            if (pending->callback != NULL) {
                pending->callback(SRP_LINK_TX_TIMEOUT, SRP_FAST_RESP_TIMEOUT,
                                  pending->callback_context);
            }
        }
    }
    if (link->state == SRP_LINK_BUS_OFF && link->config.on_bus_off != NULL) {
        link->config.on_bus_off(link->config.context);
    }
}

void srp_link_recover(srp_link_t *link)
{
    if (link == NULL) {
        return;
    }
    (void)memset(link->pending, 0, sizeof(link->pending));
    link->tec = 0U;
    link->rec = 0U;
    link->state = SRP_LINK_ACTIVE;
}

uint16_t srp_link_get_tec(const srp_link_t *link)
{
    return link == NULL ? 0U : link->tec;
}

uint16_t srp_link_get_rec(const srp_link_t *link)
{
    return link == NULL ? 0U : link->rec;
}

srp_link_state_t srp_link_get_state(const srp_link_t *link)
{
    return link == NULL ? SRP_LINK_BUS_OFF : link->state;
}
