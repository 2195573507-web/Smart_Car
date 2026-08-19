#include "sc_frame.h"

#include <string.h>

typedef struct {
    uint8_t valid;
    uint16_t msg_id;
    uint8_t seq;
    uint8_t destination;
} scbp_ack_context_t;

typedef struct {
    uint8_t valid;
    uint16_t msg_id;
    uint8_t seq;
    uint8_t destination;
    uint8_t legacy_type;
    uint8_t legacy_payload0;
} scbp_pending_tx_t;

static uint8_t s_tx_sequence;
static scbp_ack_context_t s_ack_context;
static scbp_pending_tx_t s_pending_tx;

uint16_t scbp_crc16(const uint8_t *data, size_t length)
{
    uint16_t crc = UINT16_C(0xFFFF);

    if (data == NULL && length != 0U) {
        return 0U;
    }
    for (size_t index = 0U; index < length; ++index) {
        crc ^= data[index];
        for (uint8_t bit = 0U; bit < 8U; ++bit) {
            crc = (crc & 1U) != 0U ? (uint16_t)((crc >> 1U) ^ UINT16_C(0xA001))
                                   : (uint16_t)(crc >> 1U);
        }
    }
    return crc;
}

uint16_t sc_frame_crc16(const uint8_t *data, size_t length)
{
    return scbp_crc16(data, length);
}

uint8_t scbp_next_tx_sequence(void)
{
#if defined(__GNUC__)
    return __atomic_fetch_add(&s_tx_sequence, 1U, __ATOMIC_RELAXED);
#else
    const uint8_t sequence = s_tx_sequence;
    s_tx_sequence = (uint8_t)(s_tx_sequence + 1U);
    return sequence;
#endif
}

uint8_t scbp_message_priority(uint16_t msg_id)
{
    switch (msg_id) {
    case SCBP_MSG_ID_ACK:
    case SCBP_MSG_ID_BOOT_READY:
    case SCBP_MSG_ID_MOTOR_CONTROL:
    case SCBP_MSG_ID_PWM_SET:
    case SCBP_MSG_ID_PARAM_SET:
    case SCBP_MSG_ID_RADAR_CONTROL:
    case SCBP_MSG_ID_RADAR_PWM_READY:
    case SCBP_MSG_ID_CAL_START:
    case SCBP_MSG_ID_CAL_EVENT:
    case SCBP_MSG_ID_ATTITUDE:
        return SCBP_PRIORITY_REALTIME;
    case SCBP_MSG_ID_LOG:
    case SCBP_MSG_ID_ERROR:
        return SCBP_PRIORITY_DEBUG;
    default:
        return SCBP_PRIORITY_NORMAL;
    }
}

uint8_t scbp_message_flags(uint16_t msg_id)
{
    switch (msg_id) {
    case SCBP_MSG_ID_ACK:
        return SCBP_FLAG_ACK_FRAME;
    case SCBP_MSG_ID_ERROR:
        return SCBP_FLAG_ERROR_FRAME;
    case SCBP_MSG_ID_MOTOR_CONTROL:
    case SCBP_MSG_ID_PWM_SET:
    case SCBP_MSG_ID_PARAM_SET:
    case SCBP_MSG_ID_RADAR_CONTROL:
    case SCBP_MSG_ID_RADAR_PWM_READY:
    case SCBP_MSG_ID_CAL_START:
    case SCBP_MSG_ID_CAL_EVENT:
        return SCBP_FLAG_ACK_REQUIRED;
    case SCBP_MSG_ID_IMU_STATUS:
    case SCBP_MSG_ID_ATTITUDE:
    case SCBP_MSG_ID_IMU_CAL_STATUS:
    case SCBP_MSG_ID_IMU_BIAS:
    case SCBP_MSG_ID_VIBRATION_STATUS:
    case SCBP_MSG_ID_IMU_CAL_RESULT:
    case SCBP_MSG_ID_IMU_VIBRATION_PROFILE:
    case SCBP_MSG_ID_IMU_TELEMETRY:
    case SCBP_MSG_ID_DUAL_IMU_STATUS:
    case SCBP_MSG_ID_RADAR_STATUS:
    case SCBP_MSG_ID_LOG:
        return SCBP_FLAG_STREAM_DATA;
    default:
        return 0U;
    }
}

int scbp_legacy_type_to_msg_id(uint8_t legacy_type, uint16_t *msg_id)
{
    if (msg_id == NULL) {
        return -1;
    }

    switch (legacy_type) {
    case SC_TYPE_PING: *msg_id = SCBP_MSG_ID_PING; return 0;
    case SC_TYPE_PONG: *msg_id = SCBP_MSG_ID_PONG; return 0;
    case SC_TYPE_ACK: *msg_id = SCBP_MSG_ID_ACK; return 0;
    case SC_TYPE_PWM_READY:
    case SC_TYPE_RADAR_PWM_READY:
        *msg_id = SCBP_MSG_ID_RADAR_PWM_READY;
        return 0;
    case SC_TYPE_IMU_CAL_BIAS: *msg_id = SCBP_MSG_ID_IMU_BIAS; return 0;
    case SC_TYPE_RADAR_PWM_ACK:
    case SC_TYPE_CAL_EVENT_ACK:
        *msg_id = SCBP_MSG_ID_ACK;
        return 0;
    case SC_TYPE_CAL_EVENT: *msg_id = SCBP_MSG_ID_CAL_EVENT; return 0;
    case SC_TYPE_STM_BOOT_READY: *msg_id = SCBP_MSG_ID_BOOT_READY; return 0;
    case SC_TYPE_IMU_STATUS: *msg_id = SCBP_MSG_ID_IMU_STATUS; return 0;
    case SC_TYPE_ATTITUDE: *msg_id = SCBP_MSG_ID_ATTITUDE; return 0;
    case SC_TYPE_IMU_CAL_STATUS: *msg_id = SCBP_MSG_ID_IMU_CAL_STATUS; return 0;
    case SC_TYPE_RADAR_STATUS: *msg_id = SCBP_MSG_ID_RADAR_STATUS; return 0;
    case SC_TYPE_RADAR_VIBRATION_STATUS:
        *msg_id = SCBP_MSG_ID_VIBRATION_STATUS;
        return 0;
    case SC_TYPE_IMU_CAL_RESULT:
        *msg_id = SCBP_MSG_ID_IMU_CAL_RESULT;
        return 0;
    case SC_TYPE_IMU_VIBRATION_PROFILE:
        *msg_id = SCBP_MSG_ID_IMU_VIBRATION_PROFILE;
        return 0;
    case SC_TYPE_IMU_TELEMETRY:
        *msg_id = SCBP_MSG_ID_IMU_TELEMETRY;
        return 0;
    case SC_TYPE_DUAL_IMU_STATUS:
        *msg_id = SCBP_MSG_ID_DUAL_IMU_STATUS;
        return 0;
    case SC_TYPE_LOG: *msg_id = SCBP_MSG_ID_LOG; return 0;
    default:
        return -2;
    }
}

void scbp_ack_context_set(uint16_t acknowledged_msg_id, uint8_t acknowledged_seq,
                          uint8_t destination)
{
    s_ack_context.valid = 1U;
    s_ack_context.msg_id = acknowledged_msg_id;
    s_ack_context.seq = acknowledged_seq;
    s_ack_context.destination = destination;
}

void scbp_pending_tx_clear(void)
{
    s_pending_tx.valid = 0U;
}

int scbp_pending_tx_match_ack(const scbp_frame_t *ack,
                              uint8_t *legacy_type,
                              uint8_t *legacy_payload0,
                              uint8_t *result)
{
    if (ack == NULL || legacy_type == NULL || legacy_payload0 == NULL ||
        result == NULL || ack->msg_id != SCBP_MSG_ID_ACK ||
        ack->flags != SCBP_FLAG_ACK_FRAME ||
        ack->src != s_pending_tx.destination ||
        ack->dst != SCBP_LOCAL_NODE_ID ||
        ack->length != SCBP_ACK_PAYLOAD_LENGTH || ack->payload == NULL ||
        s_pending_tx.valid == 0U ||
        s_pending_tx.msg_id !=
            ((uint16_t)ack->payload[0] |
             ((uint16_t)ack->payload[1] << 8U)) ||
        s_pending_tx.seq != ack->payload[2] ||
        ack->payload[3] > SCBP_ACK_RESULT_FAILED ||
        (ack->payload[3] == SCBP_ACK_RESULT_OK &&
         ack->payload[4] != SCBP_ERROR_OK)) {
        return 0;
    }

    *legacy_type = s_pending_tx.legacy_type;
    *legacy_payload0 = s_pending_tx.legacy_payload0;
    *result = ack->payload[3];
    s_pending_tx.valid = 0U;
    return 1;
}

int scbp_frame_encode(const scbp_frame_t *frame, uint8_t *out, size_t capacity,
                      uint16_t *out_length)
{
    uint16_t crc;

    if (frame == NULL || out == NULL || out_length == NULL ||
        (frame->payload == NULL && frame->length != 0U)) {
        return -1;
    }
    if (frame->version != SC_FRAME_VERSION ||
        frame->priority > SCBP_PRIORITY_DEBUG ||
        (frame->flags & SCBP_FLAG_RESERVED_MASK) != 0U ||
        frame->length > SC_FRAME_MAX_PAYLOAD ||
        capacity < (size_t)SC_FRAME_OVERHEAD + frame->length) {
        return -2;
    }

    out[0] = SC_FRAME_HEADER_0;
    out[1] = SC_FRAME_HEADER_1;
    out[2] = frame->version;
    out[3] = frame->priority;
    out[4] = frame->src;
    out[5] = frame->dst;
    out[6] = (uint8_t)(frame->msg_id & UINT16_C(0x00FF));
    out[7] = (uint8_t)(frame->msg_id >> 8U);
    out[8] = frame->seq;
    out[9] = frame->flags;
    out[10] = (uint8_t)(frame->length & UINT16_C(0x00FF));
    out[11] = (uint8_t)(frame->length >> 8U);
    if (frame->length != 0U) {
        memcpy(&out[12], frame->payload, frame->length);
    }
    crc = scbp_crc16(&out[2], (size_t)SCBP_CRC_HEADER_SIZE + frame->length);
    out[12U + frame->length] = (uint8_t)(crc & UINT16_C(0x00FF));
    out[13U + frame->length] = (uint8_t)(crc >> 8U);
    *out_length = (uint16_t)(SC_FRAME_OVERHEAD + frame->length);
    return 0;
}

int scbp_frame_decode(const uint8_t *frame, size_t length, scbp_frame_t *view)
{
    uint16_t payload_length;
    uint16_t received_crc;

    if (frame == NULL || view == NULL || length < SC_FRAME_OVERHEAD) {
        return -1;
    }
    if (frame[0] != SC_FRAME_HEADER_0 || frame[1] != SC_FRAME_HEADER_1) {
        return SC_FRAME_ERROR_AA55_FAIL;
    }
    if (frame[2] != SC_FRAME_VERSION) {
        return SC_FRAME_ERROR_VERSION_FAIL;
    }
    if (frame[3] > SCBP_PRIORITY_DEBUG) {
        return SC_FRAME_ERROR_PRIORITY_FAIL;
    }
    if ((frame[9] & SCBP_FLAG_RESERVED_MASK) != 0U) {
        return SC_FRAME_ERROR_FLAGS_FAIL;
    }
    payload_length = (uint16_t)frame[10] | ((uint16_t)frame[11] << 8U);
    if (payload_length > SC_FRAME_MAX_PAYLOAD ||
        length != (size_t)SC_FRAME_OVERHEAD + payload_length) {
        return SC_FRAME_ERROR_LEN_FAIL;
    }
    received_crc = (uint16_t)frame[12U + payload_length] |
                   ((uint16_t)frame[13U + payload_length] << 8U);
    if (received_crc != scbp_crc16(&frame[2],
                                   (size_t)SCBP_CRC_HEADER_SIZE + payload_length)) {
        return SC_FRAME_ERROR_CRC_FAIL;
    }

    view->version = frame[2];
    view->priority = frame[3];
    view->src = frame[4];
    view->dst = frame[5];
    view->msg_id = (uint16_t)frame[6] | ((uint16_t)frame[7] << 8U);
    view->seq = frame[8];
    view->flags = frame[9];
    view->length = payload_length;
    view->payload = &frame[12];
    view->crc = received_crc;
    view->sequence_status = SCBP_SEQUENCE_FIRST;
    return 0;
}

int sc_frame_encode(uint8_t legacy_type, const uint8_t *payload, uint16_t length,
                    uint8_t *out, size_t capacity, uint16_t *out_length)
{
    scbp_frame_t frame;
    uint16_t msg_id;
    uint8_t ack_payload[SCBP_ACK_PAYLOAD_LENGTH];
    int result;

    if (out == NULL || out_length == NULL || (payload == NULL && length != 0U)) {
        return -1;
    }

    if (legacy_type == SC_TYPE_RADAR_PWM_ACK ||
        legacy_type == SC_TYPE_CAL_EVENT_ACK) {
        if (length != 2U || s_ack_context.valid == 0U) {
            return -3;
        }
        ack_payload[0] = (uint8_t)(s_ack_context.msg_id & UINT16_C(0x00FF));
        ack_payload[1] = (uint8_t)(s_ack_context.msg_id >> 8U);
        ack_payload[2] = s_ack_context.seq;
        ack_payload[3] = payload[1] == 0U ? SCBP_ACK_RESULT_OK
                                           : SCBP_ACK_RESULT_FAILED;
        ack_payload[4] = payload[1] == 0U ? SCBP_ERROR_OK : SCBP_ERROR_NOT_READY;
        frame.version = SC_FRAME_VERSION;
        frame.priority = SCBP_PRIORITY_REALTIME;
        frame.src = SCBP_LOCAL_NODE_ID;
        frame.dst = s_ack_context.destination;
        frame.msg_id = SCBP_MSG_ID_ACK;
        frame.seq = scbp_next_tx_sequence();
        frame.flags = SCBP_FLAG_ACK_FRAME;
        frame.length = SCBP_ACK_PAYLOAD_LENGTH;
        frame.payload = ack_payload;
        frame.crc = 0U;
        frame.sequence_status = SCBP_SEQUENCE_FIRST;
        return scbp_frame_encode(&frame, out, capacity, out_length);
    }

    result = scbp_legacy_type_to_msg_id(legacy_type, &msg_id);
    if (result != 0) {
        return result;
    }
    frame.version = SC_FRAME_VERSION;
    frame.priority = scbp_message_priority(msg_id);
    frame.src = SCBP_LOCAL_NODE_ID;
    frame.dst = SCBP_DEFAULT_DESTINATION;
    frame.msg_id = msg_id;
    frame.seq = scbp_next_tx_sequence();
    frame.flags = scbp_message_flags(msg_id);
    frame.length = length;
    frame.payload = payload;
    frame.crc = 0U;
    frame.sequence_status = SCBP_SEQUENCE_FIRST;
    result = scbp_frame_encode(&frame, out, capacity, out_length);
    if (result == 0 &&
        (legacy_type == SC_TYPE_RADAR_PWM_READY ||
         legacy_type == SC_TYPE_CAL_EVENT)) {
        s_pending_tx.valid = 1U;
        s_pending_tx.msg_id = msg_id;
        s_pending_tx.seq = frame.seq;
        s_pending_tx.destination = frame.dst;
        s_pending_tx.legacy_type = legacy_type;
        s_pending_tx.legacy_payload0 = length != 0U ? payload[0] : 0U;
    }
    return result;
}

int sc_frame_decode(const uint8_t *frame, size_t length, sc_frame_view_t *view)
{
    return scbp_frame_decode(frame, length, view);
}

static void parser_reset(sc_frame_parser_t *parser)
{
    parser->length = 0U;
    parser->expected_length = 0U;
}

static void parser_discard_and_seek_aa(sc_frame_parser_t *parser)
{
    uint16_t index = parser->length;

    while (index > 1U) {
        --index;
        if (parser->bytes[index] == SC_FRAME_HEADER_0) {
            parser->bytes[0] = SC_FRAME_HEADER_0;
            parser->length = 1U;
            parser->expected_length = 0U;
            return;
        }
    }
    parser_reset(parser);
}

static uint8_t parser_sequence_status(sc_frame_parser_t *parser, uint8_t src,
                                      uint8_t sequence)
{
    const uint8_t mask = (uint8_t)(1U << (src & 7U));
    const uint8_t byte_index = (uint8_t)(src >> 3U);
    uint8_t previous;
    uint8_t delta;

    if ((parser->sequence_seen[byte_index] & mask) == 0U) {
        parser->sequence_seen[byte_index] |= mask;
        parser->sequence_last[src] = sequence;
        return SCBP_SEQUENCE_FIRST;
    }
    previous = parser->sequence_last[src];
    delta = (uint8_t)(sequence - previous);
    if (delta == 1U) {
        parser->sequence_last[src] = sequence;
        return SCBP_SEQUENCE_IN_ORDER;
    }
    if (delta == 0U) {
        return SCBP_SEQUENCE_DUPLICATE;
    }
    if (delta < UINT8_C(128)) {
        parser->sequence_last[src] = sequence;
        return SCBP_SEQUENCE_GAP;
    }
    return SCBP_SEQUENCE_OUT_OF_ORDER;
}

void sc_frame_parser_init(sc_frame_parser_t *parser, sc_frame_callback_t callback,
                          sc_frame_error_callback_t error_callback, void *context)
{
    if (parser == NULL) {
        return;
    }
    memset(parser, 0, sizeof(*parser));
    parser->callback = callback;
    parser->error_callback = error_callback;
    parser->context = context;
}

size_t sc_frame_parser_feed(sc_frame_parser_t *parser, const uint8_t *data,
                            size_t length)
{
    size_t frames = 0U;

    if (parser == NULL || (data == NULL && length != 0U)) {
        return 0U;
    }
    for (size_t index = 0U; index < length; ++index) {
        const uint8_t byte = data[index];

        if (parser->length == 0U) {
            if (byte == SC_FRAME_HEADER_0) {
                parser->bytes[parser->length++] = byte;
            }
            continue;
        }
        if (parser->length == 1U && byte != SC_FRAME_HEADER_1) {
            if (parser->error_callback != NULL) {
                parser->error_callback(SC_FRAME_ERROR_AA55_FAIL, parser->bytes,
                                       parser->length, parser->context);
            }
            parser->length = byte == SC_FRAME_HEADER_0 ? 1U : 0U;
            if (parser->length != 0U) {
                parser->bytes[0] = byte;
            }
            continue;
        }
        if (parser->length >= sizeof(parser->bytes)) {
            parser_discard_and_seek_aa(parser);
            continue;
        }
        parser->bytes[parser->length++] = byte;
        if (parser->length == 12U) {
            const uint16_t payload_length = (uint16_t)parser->bytes[10] |
                                            ((uint16_t)parser->bytes[11] << 8U);
            if (payload_length > SC_FRAME_MAX_PAYLOAD) {
                if (parser->error_callback != NULL) {
                    parser->error_callback(SC_FRAME_ERROR_LEN_FAIL, parser->bytes,
                                           parser->length, parser->context);
                }
                parser_discard_and_seek_aa(parser);
                continue;
            }
            parser->expected_length = (uint16_t)(SC_FRAME_OVERHEAD + payload_length);
        }
        if (parser->expected_length != 0U &&
            parser->length == parser->expected_length) {
            sc_frame_view_t view;
            const int status = scbp_frame_decode(parser->bytes, parser->length,
                                                 &view);

            ++parser->frame_index;
            if (status == 0) {
                view.sequence_status = parser_sequence_status(parser, view.src,
                                                               view.seq);
                if (parser->callback != NULL) {
                    parser->callback(&view, parser->context);
                }
                ++frames;
                parser_reset(parser);
            } else {
                if (parser->error_callback != NULL) {
                    parser->error_callback(status, parser->bytes, parser->length,
                                           parser->context);
                }
                parser_discard_and_seek_aa(parser);
            }
        }
    }
    return frames;
}
