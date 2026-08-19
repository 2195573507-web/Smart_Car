#include "s3_service.h"

#include <stdio.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

#include "imu_boot_manager.h"
#include "imu_filter.h"
#include "log_service.h"
#include "sc_frame.h"
#include "uart_link.h"

#define S3_SERVICE_STACK_WORDS 384U
#define S3_SERVICE_PRIORITY (tskIDLE_PRIORITY + 2U)
#define S3_SERVICE_RX_TIMEOUT_MS UINT32_C(2000)
#define S3_SERVICE_RADAR_STATUS_TIMEOUT_MS UINT32_C(3000)
#define S3_SERVICE_STACK_MONITOR_PERIOD_MS UINT32_C(5000)

static sc_frame_parser_t s_parser;
static uint32_t s_crc_error_count;
static uint32_t s_last_rx_time;
static uint8_t s_link_stale_logged;
static uint8_t radar_pwm_percent;
static uint32_t radar_status_last_rx_ms;
static uint8_t radar_status_received;
static uint8_t radar_status_stale;
static TaskHandle_t s_s3_service_task_handle;
static UBaseType_t s_s3_service_min_free_words = S3_SERVICE_STACK_WORDS;

/* These buffers are used only by the single s3_service/parser call chain.
 * Keeping them out of the task frame leaves room for snprintf/HAL internals. */
static uint8_t s_step_bytes[64];
static char s_frame_line[96];
static char s_cal_event_line[SMARTCAR_LOG_MAX_PAYLOAD + 1U];
static char s_error_line[256];
static uint8_t s_attitude_v3_payload[SC_ATTITUDE_PAYLOAD_LENGTH];

static HAL_StatusTypeDef s3_service_send_v3(uint16_t msg_id, uint8_t priority,
                                             uint8_t flags, uint8_t destination,
                                             const uint8_t *payload,
                                             uint16_t length)
{
    uint8_t frame_bytes[SC_FRAME_MAX_SIZE];
    uint16_t frame_length = 0U;
    const scbp_frame_t frame = {
        .version = SC_FRAME_VERSION,
        .priority = priority,
        .src = SCBP_LOCAL_NODE_ID,
        .dst = destination,
        .msg_id = msg_id,
        .seq = scbp_next_tx_sequence(),
        .flags = flags,
        .length = length,
        .payload = payload,
        .crc = 0U,
        .sequence_status = SCBP_SEQUENCE_FIRST,
    };

    if (scbp_frame_encode(&frame, frame_bytes, sizeof(frame_bytes),
                          &frame_length) != 0) {
        return HAL_ERROR;
    }
    return uart_link_send(frame_bytes, frame_length);
}

static void s3_service_send_protocol_error(const scbp_frame_t *request,
                                           uint8_t error_code)
{
    uint8_t payload[SCBP_ERROR_PAYLOAD_LENGTH];

    if (request == NULL || request->src == SCBP_NODE_BROADCAST ||
        request->msg_id == SCBP_MSG_ID_ERROR) {
        return;
    }
    payload[0] = SCBP_LOCAL_NODE_ID;
    payload[1] = error_code;
    payload[2] = (uint8_t)(request->msg_id & UINT16_C(0x00FF));
    payload[3] = (uint8_t)(request->msg_id >> 8U);
    payload[4] = request->seq;
    (void)s3_service_send_v3(SCBP_MSG_ID_ERROR, SCBP_PRIORITY_DEBUG,
                              SCBP_FLAG_ERROR_FRAME, request->src, payload,
                              (uint16_t)sizeof(payload));
}

static void s3_service_handle_radar_status(const sc_frame_view_t *frame)
{
    if (frame == NULL || frame->length != 2U || frame->payload == NULL) {
        /* A malformed status must not replace the retained PWM, but it must
         * leave the runtime filter on the safe default until a valid status
         * arrives. */
        imu_filter_set_radar_pwm(0U);
        return;
    }

    /* The online byte is part of the existing RADAR_STATUS contract. Invalid
     * status does not replace the retained PWM, but it forces the safe filter
     * alpha until a valid status arrives. */
    if (frame->payload[0] != 1U || frame->payload[1] > 100U) {
        imu_filter_set_radar_pwm(0U);
        return;
    }
    radar_pwm_percent = frame->payload[1];
    radar_status_last_rx_ms = HAL_GetTick();
    radar_status_received = 1U;
    radar_status_stale = 0U;
    imu_filter_set_radar_pwm(radar_pwm_percent);
}

static void diag(const char *text)
{
    if (text != NULL) LOG_INFO(text);
}

static void s3_service_log_stack(void)
{
    char line[80];
    const UBaseType_t free_words = s_s3_service_task_handle == NULL
                                        ? 0U
                                        : uxTaskGetStackHighWaterMark(
                                              s_s3_service_task_handle);

    if (free_words < s_s3_service_min_free_words) {
        s_s3_service_min_free_words = free_words;
    }

    (void)snprintf(line, sizeof(line),
                   "[S3_TASK_STACK]\r\nfree_words=%lu\r\n"
                   "min_free_words=%lu\r\nstack_size_words=%lu\r\n",
                   (unsigned long)free_words,
                   (unsigned long)s_s3_service_min_free_words,
                   (unsigned long)S3_SERVICE_STACK_WORDS);
    LOG_INFO(line);
}

void s3_service_send_boot_frame(uint8_t type, const uint8_t *payload,
                                uint16_t length)
{
    uint8_t frame[SC_FRAME_MAX_SIZE];
    uint16_t frame_length = 0U;
    const uint8_t event_id =
        (payload != NULL && length != 0U) ? payload[0] : UINT8_C(0xFF);
    const int encode_result =
        sc_frame_encode(type, payload, length, frame, sizeof(frame),
                        &frame_length);

    if (encode_result != 0) {
        if (type == SC_TYPE_CAL_EVENT) {
            (void)snprintf(s_cal_event_line, sizeof(s_cal_event_line),
                           "CAL_EVENT_ENCODE_FAIL id=%u result=%d\r\n",
                           (unsigned)event_id, encode_result);
            LOG_ERROR(s_cal_event_line);
        }
        return;
    }

    const HAL_StatusTypeDef status = uart_link_send(frame, frame_length);
    if (type == SC_TYPE_CAL_EVENT) {
        if (status == HAL_OK) {
            (void)snprintf(s_cal_event_line, sizeof(s_cal_event_line),
                           "CAL_EVENT_TX id=%u seq=%u result=%d\r\n",
                           (unsigned)event_id,
                           (unsigned)frame[8],
                           (int)status);
            LOG_INFO(s_cal_event_line);
        } else {
            (void)snprintf(s_cal_event_line, sizeof(s_cal_event_line),
                           "CAL_EVENT_UART_FAIL id=%u seq=%u result=%d\r\n",
                           (unsigned)event_id,
                           (unsigned)frame[8],
                           (int)status);
            LOG_ERROR(s_cal_event_line);
        }
    } else if (type == SC_TYPE_RADAR_PWM_ACK) {
        char line[64];
        (void)snprintf(line, sizeof(line),
                       "RADAR_PWM_ACK TX\r\nspeed=%u\r\nresult=%u\r\n",
                       (unsigned)(length == 2U ? payload[0] : 0U),
                       (unsigned)(length == 2U ? payload[1] : 1U));
        if (status == HAL_OK) {
            LOG_INFO(line);
        } else {
            LOG_ERROR(line);
        }
    }
}

void s3_service_send_telemetry_frame(uint8_t type, const uint8_t *payload,
                                     uint16_t length)
{
    if (type == SC_TYPE_ATTITUDE) {
        if (payload == NULL) {
            return;
        }
        if (length == SC_LEGACY_ATTITUDE_PAYLOAD_LENGTH) {
            const uint32_t timestamp_ms = HAL_GetTick();

            /* Keep the pre-V3 producer accepted, but put it on the same
             * active wire contract before encoding. */
            for (uint8_t index = 0U; index < 24U; ++index) {
                s_attitude_v3_payload[index] = payload[index];
            }
            s_attitude_v3_payload[24] =
                (uint8_t)(timestamp_ms & UINT32_C(0xFF));
            s_attitude_v3_payload[25] =
                (uint8_t)((timestamp_ms >> 8U) & UINT32_C(0xFF));
            s_attitude_v3_payload[26] =
                (uint8_t)((timestamp_ms >> 16U) & UINT32_C(0xFF));
            s_attitude_v3_payload[27] = (uint8_t)(timestamp_ms >> 24U);
            s_attitude_v3_payload[28] = payload[25]; /* source */
            s_attitude_v3_payload[29] = payload[24]; /* valid */
            payload = s_attitude_v3_payload;
            length = SC_ATTITUDE_PAYLOAD_LENGTH;
        }
        if (length != SC_ATTITUDE_PAYLOAD_LENGTH) {
            return;
        }
        (void)s3_service_send_v3(SCBP_MSG_ID_ATTITUDE,
                                  SCBP_PRIORITY_REALTIME,
                                  SCBP_FLAG_STREAM_DATA,
                                  SCBP_DEFAULT_DESTINATION,
                                  payload, length);
        return;
    }
    s3_service_send_boot_frame(type, payload, length);
}

void s3_service_send_dual_attitude(const uint8_t *payload, uint16_t length)
{
    if (payload == NULL || length != SC_DUAL_ATTITUDE_PAYLOAD_LENGTH ||
        payload[0] != SC_DUAL_ATTITUDE_SCHEMA || payload[2] != 0U ||
        payload[3] != 0U) {
        return;
    }
    (void)s3_service_send_v3(SCBP_MSG_ID_ATTITUDE, SCBP_PRIORITY_REALTIME,
                             SCBP_FLAG_STREAM_DATA, SCBP_DEFAULT_DESTINATION,
                             payload, length);
}

static void s3_service_on_frame(const sc_frame_view_t *frame, void *context)
{
    (void)context;
    if (frame == NULL) {
        return;
    }
    s_last_rx_time = uart_link_get_last_rx_time();
    s_link_stale_logged = 0U;
    if (frame->dst != SCBP_LOCAL_NODE_ID &&
        frame->dst != SCBP_NODE_BROADCAST) {
        return;
    }
    if (frame->sequence_status == SCBP_SEQUENCE_GAP ||
        frame->sequence_status == SCBP_SEQUENCE_DUPLICATE ||
        frame->sequence_status == SCBP_SEQUENCE_OUT_OF_ORDER) {
        (void)snprintf(s_frame_line, sizeof(s_frame_line),
                       "SCBP_SEQ src=0x%02X seq=%u state=%u\r\n",
                       (unsigned)frame->src, (unsigned)frame->seq,
                       (unsigned)frame->sequence_status);
        LOG_WARN(s_frame_line);
    }
    if (frame->msg_id == SCBP_MSG_ID_RADAR_STATUS) {
        /* Radar status is a periodic control-state frame; keep it off the
         * generic per-frame diagnostic path. */
        s3_service_handle_radar_status(frame);
        return;
    }
    (void)snprintf(s_frame_line, sizeof(s_frame_line),
                   "SCBP_RX msg=0x%04X src=0x%02X dst=0x%02X seq=%u len=%u\r\n",
                   (unsigned)frame->msg_id, (unsigned)frame->src,
                   (unsigned)frame->dst, (unsigned)frame->seq,
                   (unsigned)frame->length);
    LOG_INFO(s_frame_line);
    /* PONG remains a parse-compatible frame, but no longer drives a
     * periodic heartbeat. Link freshness is based on last_rx_time instead. */
    if (frame->msg_id == SCBP_MSG_ID_PONG) {
        diag("PONG_RX");
        return;
    }
    if (frame->msg_id == SCBP_MSG_ID_RADAR_PWM_READY &&
        frame->length == 1U && frame->src == SCBP_NODE_ESP32_S3 &&
        frame->flags == SCBP_FLAG_ACK_REQUIRED) {
        (void)snprintf(s_frame_line, sizeof(s_frame_line),
                       "RADAR_PWM_READY RX\r\nspeed=%u\r\n",
                       (unsigned)frame->payload[0]);
        LOG_INFO(s_frame_line);
        scbp_ack_context_set(frame->msg_id, frame->seq, frame->src);
        imu_boot_manager_on_radar_pwm_ready(frame->payload[0]);
        return;
    }
    if (frame->msg_id == SCBP_MSG_ID_CAL_EVENT &&
        frame->src == SCBP_NODE_ESP32_S3 &&
        frame->flags == SCBP_FLAG_ACK_REQUIRED) {
        uint8_t ack[2];

        if (frame->length != 1U ||
            frame->payload[0] != SC_CAL_EVENT_COMPLETE) {
            s3_service_send_protocol_error(frame,
                                           frame->length != 1U
                                               ? SCBP_ERROR_INVALID_LENGTH
                                               : SCBP_ERROR_PARAM);
            return;
        }
        (void)snprintf(s_frame_line, sizeof(s_frame_line),
                       "RADAR_CAL_COMPLETE RX id=%u\r\n",
                       (unsigned)frame->payload[0]);
        LOG_INFO(s_frame_line);
        /* The S3 owns radar PWM completion. This status notification must
         * still be ACKed even if STM32 has already progressed into VERIFY. */
        ack[0] = frame->payload[0];
        ack[1] = CAL_ACK_OK;
        scbp_ack_context_set(frame->msg_id, frame->seq, frame->src);
        s3_service_send_boot_frame(SC_TYPE_CAL_EVENT_ACK, ack,
                                   (uint16_t)sizeof(ack));
        return;
    }
    if (frame->msg_id == SCBP_MSG_ID_ACK) {
        uint8_t legacy_type;
        uint8_t legacy_payload0;
        uint8_t result;

        if (frame->length != SCBP_ACK_PAYLOAD_LENGTH) {
            s3_service_send_protocol_error(frame, SCBP_ERROR_INVALID_LENGTH);
            return;
        }
        if (scbp_pending_tx_match_ack(frame, &legacy_type, &legacy_payload0,
                                      &result) != 0 &&
            legacy_type == SC_TYPE_CAL_EVENT) {
            imu_boot_manager_on_cal_event_ack(
                legacy_payload0,
                result == SCBP_ACK_RESULT_OK ? CAL_ACK_OK : CAL_ACK_ERROR);
        } else {
            LOG_WARN("SCBP_ACK rejected; pending transaction unchanged\r\n");
        }
        return;
    }
    if (frame->msg_id == SCBP_MSG_ID_ERROR) {
        return;
    }
    s3_service_send_protocol_error(frame, SCBP_ERROR_UNKNOWN_MSG);
}

static void s3_service_on_error(int error, const uint8_t *data, size_t length,
                                void *context)
{
    const char *reason = "UNKNOWN";
    (void)context;
    switch (error) {
    case SC_FRAME_ERROR_AA55_FAIL: reason = "AA55_FAIL"; break;
    case SC_FRAME_ERROR_VERSION_FAIL: reason = "VERSION_FAIL"; break;
    case SC_FRAME_ERROR_LEN_FAIL: reason = "LEN_FAIL"; break;
    case SC_FRAME_ERROR_CRC_FAIL:
    {
        uart_link_stats_t uart_stats = {0};
        uint16_t msg_id = 0U;
        uint16_t len_field = 0U;
        uint16_t rx_crc = 0U;
        uint16_t calc_crc = 0U;
        const uint16_t parser_buffer_len =
            length > sizeof(s_parser.bytes) ? (uint16_t)sizeof(s_parser.bytes)
                                            : (uint16_t)length;

        reason = "CRC_FAIL";
        ++s_crc_error_count;
        if (data != NULL && length >= 12U) {
            msg_id = (uint16_t)data[6] | ((uint16_t)data[7] << 8U);
            len_field = (uint16_t)data[10] | ((uint16_t)data[11] << 8U);
            if (length >= (size_t)SC_FRAME_OVERHEAD + len_field) {
                rx_crc = (uint16_t)data[12U + len_field] |
                         ((uint16_t)data[13U + len_field] << 8U);
                calc_crc = scbp_crc16(&data[2],
                                      (size_t)SCBP_CRC_HEADER_SIZE + len_field);
            }
        }
        uart_link_get_stats(&uart_stats);
        (void)snprintf(s_error_line, sizeof(s_error_line),
                       "CRC_FAIL msg=0x%04X len_field=%u rx_crc=0x%04X "
                       "calc_crc=0x%04X frame_index=%lu buffer_len=%u "
                       "buffer_expected=%u buffer_capacity=%u buffered=%u "
                       "rx_overflow=%lu rx_drop=%lu\r\n",
                       (unsigned)msg_id, (unsigned)len_field, (unsigned)rx_crc,
                       (unsigned)calc_crc, (unsigned long)s_parser.frame_index,
                       (unsigned)parser_buffer_len,
                       (unsigned)s_parser.expected_length,
                       (unsigned)uart_stats.rx_buffer_capacity,
                       (unsigned)uart_stats.rx_buffered,
                       (unsigned long)uart_stats.uart_rx_overflow,
                       (unsigned long)uart_stats.uart_rx_drop);
        LOG_ERROR(s_error_line);
        return;
    }
    case SC_FRAME_ERROR_PRIORITY_FAIL: reason = "PRIORITY_FAIL"; break;
    case SC_FRAME_ERROR_FLAGS_FAIL: reason = "FLAGS_FAIL"; break;
    default: break;
    }
    (void)snprintf(s_error_line, sizeof(s_error_line),
                   "FRAME ERROR: %s len=%u\r\n", reason,
                   (unsigned)length);
    if (error == SC_FRAME_ERROR_CRC_FAIL) {
        LOG_ERROR(s_error_line);
    } else {
        LOG_WARN(s_error_line);
    }
}

void s3_service_init(void)
{
    sc_frame_parser_init(&s_parser, s3_service_on_frame, s3_service_on_error, NULL);
    s_crc_error_count = 0U;
    s_last_rx_time = 0U;
    s_link_stale_logged = 0U;
    radar_pwm_percent = 0U;
    radar_status_last_rx_ms = 0U;
    radar_status_received = 0U;
    radar_status_stale = 0U;
    imu_filter_set_radar_pwm(0U);
    s_s3_service_min_free_words = S3_SERVICE_STACK_WORDS;
    imu_boot_manager_set_transport(s3_service_send_boot_frame);
}

void s3_service_step(void)
{
    const size_t length = uart_link_read(s_step_bytes, sizeof(s_step_bytes));
    const uint32_t latest_rx_time = uart_link_get_last_rx_time();
    if (length != 0U) {
        (void)sc_frame_parser_feed(&s_parser, s_step_bytes, length);
    }

    if (latest_rx_time != 0U && latest_rx_time != s_last_rx_time) {
        s_last_rx_time = latest_rx_time;
        s_link_stale_logged = 0U;
    }
    if (s_last_rx_time != 0U &&
        (uint32_t)(HAL_GetTick() - s_last_rx_time) >=
            S3_SERVICE_RX_TIMEOUT_MS &&
        s_link_stale_logged == 0U) {
        LOG_WARN("UART_LINK_STALE last_rx_time exceeded timeout\r\n");
        s_link_stale_logged = 1U;
    }
    if (radar_status_received != 0U && radar_status_stale == 0U &&
        (uint32_t)(HAL_GetTick() - radar_status_last_rx_ms) >=
            S3_SERVICE_RADAR_STATUS_TIMEOUT_MS) {
        /* Keep radar_pwm_percent and the selected alpha unchanged across a
         * transport gap; the next valid frame can update them immediately. */
        radar_status_stale = 1U;
    }
}

void s3_service_task(void *argument)
{
    uint32_t last_stack_monitor_ms;

    (void)argument;
    s3_service_init();
    last_stack_monitor_ms = HAL_GetTick();
    for (;;) {
        s3_service_step();
        if ((uint32_t)(HAL_GetTick() - last_stack_monitor_ms) >=
            S3_SERVICE_STACK_MONITOR_PERIOD_MS) {
            last_stack_monitor_ms = HAL_GetTick();
            s3_service_log_stack();
        }
        vTaskDelay(pdMS_TO_TICKS(1U));
    }
}

void s3_service_start(void)
{
    /* Install the transport before the scheduler so the IMU task can publish
     * STM_BOOT_READY on its first WAIT_RADAR_ZERO transition. */
    imu_boot_manager_set_transport(s3_service_send_boot_frame);
    const BaseType_t result =
        xTaskCreate(s3_service_task, "s3_service", S3_SERVICE_STACK_WORDS,
                    NULL, S3_SERVICE_PRIORITY, &s_s3_service_task_handle);
    char line[64];
    (void)snprintf(line, sizeof(line), "s3_service_task start result=%ld\r\n",
                   (long)result);
    if (result == pdPASS) {
        LOG_INFO(line);
    } else {
        LOG_ERROR(line);
    }
}
