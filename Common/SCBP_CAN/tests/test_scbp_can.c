#include <stdio.h>
#include <string.h>

#include "scbp_crc.h"
#include "scbp_link.h"
#include "scbp_wire.h"

#define TEST_ASSERT(condition) do { \
    if (!(condition)) { \
        (void)fprintf(stderr, "assertion failed: %s at %s:%d\n", #condition, __FILE__, __LINE__); \
        return 1; \
    } \
} while (0)

typedef struct {
    unsigned int frames;
    unsigned int errors;
    scbp_parser_error_t last_error;
    scbp_can_frame_t last_frame;
    uint8_t payload[SCBP_CAN_MAX_PAYLOAD];
} parser_observer_t;

typedef struct {
    unsigned int sends;
    unsigned int callbacks;
    unsigned int bus_offs;
    scbp_link_tx_result_t result;
    uint8_t status;
    uint8_t last_tx[SCBP_CAN_MAX_FRAME_SIZE];
    uint16_t last_tx_length;
} link_observer_t;

static void write_u16_le(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)(value & UINT16_C(0x00FF));
    data[1] = (uint8_t)(value >> 8U);
}

static void parser_on_frame(const scbp_can_frame_t *frame, void *context)
{
    parser_observer_t *observer = context;
    ++observer->frames;
    observer->last_frame = *frame;
    if (frame->length != 0U) {
        (void)memcpy(observer->payload, frame->payload, frame->length);
        observer->last_frame.payload = observer->payload;
    }
}

static void parser_on_error(scbp_parser_error_t error, const uint8_t *data,
                            size_t length, void *context)
{
    parser_observer_t *observer = context;
    (void)data;
    (void)length;
    ++observer->errors;
    observer->last_error = error;
}

static int link_send(const uint8_t *data, uint16_t length, void *context)
{
    link_observer_t *observer = context;
    ++observer->sends;
    observer->last_tx_length = length;
    (void)memcpy(observer->last_tx, data, length);
    return 0;
}

static void link_complete(scbp_link_tx_result_t result, uint8_t status, void *context)
{
    link_observer_t *observer = context;
    ++observer->callbacks;
    observer->result = result;
    observer->status = status;
}

static void link_bus_off(void *context)
{
    link_observer_t *observer = context;
    ++observer->bus_offs;
}

static int make_frame(uint8_t sequence, const uint8_t *payload, uint8_t payload_length,
                      uint8_t *wire, uint16_t *wire_length)
{
    const scbp_can_frame_t frame = {
        .can_id = SCBP_CAN_ID(SCBP_CAN_PRIORITY_REALTIME, SCBP_NODE_STM32H757,
                              SCBP_NODE_ESP32_S3, SCBP_MSG_ID_ATTITUDE),
        .flags = SCBP_CAN_FLAG_STREAM_DATA,
        .sequence = sequence,
        .length = payload_length,
        .payload = payload,
    };
    return scbp_can_encode(&frame, wire, SCBP_CAN_MAX_FRAME_SIZE, wire_length);
}

static int test_crc_and_codec(void)
{
    static const uint8_t vector[] = "123456789";
    static const uint8_t payload[] = {0x01U, 0x02U, 0x03U, 0x04U};
    uint8_t wire[SCBP_CAN_MAX_FRAME_SIZE];
    uint16_t wire_length = 0U;
    scbp_can_frame_t view;

    TEST_ASSERT(scbp_crc8_itu(vector, sizeof(vector) - 1U) == UINT8_C(0xF4));
    TEST_ASSERT(scbp_crc16_modbus(vector, sizeof(vector) - 1U) == UINT16_C(0x4B37));
    TEST_ASSERT(make_frame(UINT8_C(0x2A), payload, sizeof(payload), wire,
                           &wire_length) == 0);
    TEST_ASSERT(wire_length == SCBP_CAN_HEADER_SIZE + sizeof(payload) +
                                   SCBP_CAN_TRAILER_SIZE);
    TEST_ASSERT(scbp_can_decode(wire, wire_length, &view) == 0);
    TEST_ASSERT(view.sequence == UINT8_C(0x2A));
    TEST_ASSERT(view.length == sizeof(payload));
    TEST_ASSERT(memcmp(view.payload, payload, sizeof(payload)) == 0);
    return 0;
}

static int test_parser_fragment_and_resync(void)
{
    static const uint8_t payload[] = {0xA1U, 0xB2U, 0xC3U, 0xD4U, 0xE5U};
    uint8_t frame[SCBP_CAN_MAX_FRAME_SIZE];
    uint8_t corrupted[SCBP_CAN_MAX_FRAME_SIZE];
    uint8_t joined[SCBP_CAN_MAX_FRAME_SIZE * 2U];
    uint16_t frame_length = 0U;
    parser_observer_t observer = {0};
    scbp_parser_t parser;

    TEST_ASSERT(make_frame(1U, payload, sizeof(payload), frame, &frame_length) == 0);
    scbp_parser_init(&parser, parser_on_frame, parser_on_error, &observer);
    for (uint16_t index = 0U; index < frame_length; ++index) {
        (void)scbp_parser_feed(&parser, &frame[index], 1U);
    }
    TEST_ASSERT(observer.frames == 1U);
    TEST_ASSERT(memcmp(observer.payload, payload, sizeof(payload)) == 0);

    scbp_parser_init(&parser, parser_on_frame, parser_on_error, &observer);
    (void)memcpy(joined, frame, frame_length);
    (void)memcpy(&joined[frame_length], frame, frame_length);
    TEST_ASSERT(scbp_parser_feed(&parser, joined, (size_t)frame_length * 2U) == 2U);
    TEST_ASSERT(observer.frames == 3U);

    (void)memcpy(corrupted, frame, frame_length);
    corrupted[6] ^= UINT8_C(0x80);
    scbp_parser_init(&parser, parser_on_frame, parser_on_error, &observer);
    (void)memcpy(joined, corrupted, frame_length);
    (void)memcpy(&joined[frame_length], frame, frame_length);
    TEST_ASSERT(scbp_parser_feed(&parser, joined, (size_t)frame_length * 2U) == 1U);
    TEST_ASSERT(observer.last_error == SCBP_PARSER_ERROR_HCS);

    /* A valid frame can begin inside the rejected header. HCS rejection must
     * replay that suffix, rather than trusting LEN or dropping the new SOF. */
    scbp_parser_init(&parser, parser_on_frame, parser_on_error, &observer);
    joined[0] = SCBP_CAN_SOF0;
    joined[1] = SCBP_CAN_SOF1;
    joined[2] = UINT8_C(0x34);
    joined[3] = UINT8_C(0x12);
    (void)memcpy(&joined[4], frame, frame_length);
    TEST_ASSERT(scbp_parser_feed(&parser, joined, (size_t)frame_length + 4U) == 1U);
    TEST_ASSERT(observer.last_error == SCBP_PARSER_ERROR_HCS);

    (void)memcpy(corrupted, frame, frame_length);
    corrupted[SCBP_CAN_HEADER_SIZE] ^= UINT8_C(0x01);
    scbp_parser_init(&parser, parser_on_frame, parser_on_error, &observer);
    (void)memcpy(joined, corrupted, frame_length);
    (void)memcpy(&joined[frame_length], frame, frame_length);
    TEST_ASSERT(scbp_parser_feed(&parser, joined, (size_t)frame_length * 2U) == 1U);
    TEST_ASSERT(observer.last_error == SCBP_PARSER_ERROR_FCS);
    return 0;
}

static int test_explicit_float_wire_codec(void)
{
    const float source[4] = {1.25f, -2.5f, 0.0f, 800.0f};
    float decoded[4] = {0.0f};
    uint8_t bytes[SCBP_PAYLOAD_WHEEL_SPEED_CMD_SIZE];

    scbp_wire_write_f32_array_le(bytes, source, 4U);
    TEST_ASSERT(bytes[0] == 0x00U && bytes[1] == 0x00U &&
                bytes[2] == 0xA0U && bytes[3] == 0x3FU);
    TEST_ASSERT(scbp_wire_read_f32_array_le(bytes, sizeof(bytes), decoded, 4U));
    for (size_t index = 0U; index < 4U; ++index) {
        TEST_ASSERT(decoded[index] == source[index]);
    }
    bytes[0] = 0x00U;
    bytes[1] = 0x00U;
    bytes[2] = 0x80U;
    bytes[3] = 0x7FU;
    TEST_ASSERT(!scbp_wire_read_f32_array_le(bytes, sizeof(bytes), decoded, 1U));
    TEST_ASSERT(!scbp_wire_read_f32_array_le(bytes, sizeof(bytes) - 1U, decoded, 4U));
    return 0;
}

static int test_pid_params_payload_wire_contract(void)
{
    const float source[4] = {1.10f, 0.06f, 0.00f, 800.0f};
    float decoded[4] = {0.0f};
    uint8_t bytes[SCBP_PAYLOAD_PID_PARAMS_SIZE];

    TEST_ASSERT(SCBP_MSG_ID_PID_PARAMS_CMD == UINT16_C(0x111));
    scbp_wire_write_f32_array_le(bytes, source, 4U);
    TEST_ASSERT(bytes[0] == 0xCDU && bytes[1] == 0xCCU &&
                bytes[2] == 0x8CU && bytes[3] == 0x3FU);
    TEST_ASSERT(bytes[12] == 0x00U && bytes[13] == 0x00U &&
                bytes[14] == 0x48U && bytes[15] == 0x44U);
    TEST_ASSERT(scbp_wire_read_f32_array_le(bytes, sizeof(bytes), decoded, 4U));
    for (size_t index = 0U; index < 4U; ++index) {
        TEST_ASSERT(decoded[index] == source[index]);
    }
    TEST_ASSERT(!scbp_wire_read_f32_array_le(bytes,
                                             SCBP_PAYLOAD_PID_PARAMS_SIZE - 1U,
                                             decoded, 4U));
    return 0;
}

static int test_link_ack_retry_and_bus_off(void)
{
    static const uint8_t payload[] = {0x44U};
    link_observer_t observer = {0};
    scbp_link_config_t config = {
        .local_node = SCBP_NODE_STM32H757,
        .transport_send = link_send,
        .on_bus_off = link_bus_off,
        .context = &observer,
    };
    scbp_link_t link;
    scbp_can_frame_t request;
    scbp_can_frame_t response;
    uint8_t response_payload[sizeof(scbp_fast_resp_payload_t)];
    uint8_t response_wire[SCBP_CAN_MAX_FRAME_SIZE];
    uint16_t response_length = 0U;

    scbp_link_init(&link, &config);
    TEST_ASSERT(scbp_link_send(&link, SCBP_CAN_PRIORITY_REALTIME,
                               SCBP_NODE_ESP32_S3, SCBP_MSG_ID_RADAR_PWM_READY,
                               SCBP_CAN_FLAG_ACK_REQUIRED, payload, sizeof(payload),
                               0U, link_complete, &observer) == 0);
    TEST_ASSERT(observer.sends == 1U);
    TEST_ASSERT(scbp_can_decode(observer.last_tx, observer.last_tx_length, &request) == 0);

    write_u16_le(response_payload, request.can_id);
    response_payload[2] = request.sequence;
    response_payload[3] = SCBP_FAST_RESP_OK;
    response.can_id = SCBP_CAN_ID(SCBP_CAN_PRIORITY_REALTIME, SCBP_NODE_ESP32_S3,
                                  SCBP_NODE_STM32H757, SCBP_MSG_ID_ACK);
    response.flags = SCBP_CAN_FLAG_IS_ACK;
    response.sequence = 9U;
    response.length = sizeof(response_payload);
    response.payload = response_payload;
    TEST_ASSERT(scbp_can_encode(&response, response_wire, sizeof(response_wire),
                                &response_length) == 0);
    TEST_ASSERT(scbp_can_decode(response_wire, response_length, &response) == 0);
    scbp_link_receive(&link, &response);
    TEST_ASSERT(observer.callbacks == 1U);
    TEST_ASSERT(observer.result == SCBP_LINK_TX_OK);

    (void)memset(&observer, 0, sizeof(observer));
    config.context = &observer;
    scbp_link_init(&link, &config);
    TEST_ASSERT(scbp_link_send(&link, SCBP_CAN_PRIORITY_NORMAL, SCBP_NODE_ESP32_S3,
                               SCBP_MSG_ID_CAL_EVENT, SCBP_CAN_FLAG_ACK_REQUIRED,
                               payload, sizeof(payload), 0U, link_complete,
                               &observer) == 0);
    scbp_link_tick(&link, 500U);
    scbp_link_tick(&link, 1000U);
    scbp_link_tick(&link, 1500U);
    scbp_link_tick(&link, 2000U);
    TEST_ASSERT(observer.sends == 4U);
    TEST_ASSERT(observer.callbacks == 1U);
    TEST_ASSERT(observer.result == SCBP_LINK_TX_TIMEOUT);
    TEST_ASSERT(scbp_link_get_tec(&link) == 8U);

    for (unsigned int index = 0U; index < 32U; ++index) {
        scbp_link_report_parser_error(&link, SCBP_PARSER_ERROR_HCS);
    }
    TEST_ASSERT(scbp_link_get_state(&link) == SCBP_LINK_BUS_OFF);
    TEST_ASSERT(observer.bus_offs == 1U);
    scbp_link_recover(&link);
    TEST_ASSERT(scbp_link_get_state(&link) == SCBP_LINK_ACTIVE);
    TEST_ASSERT(scbp_link_get_tec(&link) == 0U);
    TEST_ASSERT(scbp_link_get_rec(&link) == 0U);
    return 0;
}

int main(void)
{
    if (test_crc_and_codec() != 0 || test_explicit_float_wire_codec() != 0 ||
        test_pid_params_payload_wire_contract() != 0 ||
        test_parser_fragment_and_resync() != 0 ||
        test_link_ack_retry_and_bus_off() != 0) {
        return 1;
    }
    (void)puts("SCBP-CAN host tests passed");
    return 0;
}
