#include "s3_service.h"

#include <stdio.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

#include "imu_boot_manager.h"
#include "log_service.h"
#include "sc_frame.h"
#include "uart_link.h"

#define S3_SERVICE_STACK_WORDS 384U
#define S3_SERVICE_PRIORITY (tskIDLE_PRIORITY + 2U)
#define S3_SERVICE_RX_TIMEOUT_MS UINT32_C(2000)
#define S3_SERVICE_STACK_MONITOR_PERIOD_MS UINT32_C(5000)

static sc_frame_parser_t s_parser;
static uint32_t s_crc_error_count;
static uint32_t s_last_rx_time;
static uint8_t s_link_stale_logged;
static TaskHandle_t s_s3_service_task_handle;
static UBaseType_t s_s3_service_min_free_words = S3_SERVICE_STACK_WORDS;

/* These buffers are used only by the single s3_service/parser call chain.
 * Keeping them out of the task frame leaves room for snprintf/HAL internals. */
static uint8_t s_step_bytes[64];
static char s_frame_line[96];
static char s_error_line[256];

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

    if (sc_frame_encode(type, payload, length, frame, sizeof(frame),
                        &frame_length) == 0) {
        const HAL_StatusTypeDef status = uart_link_send(frame, frame_length);
        if (type == SC_TYPE_RADAR_PWM_ACK) {
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
}

static void s3_service_on_frame(const sc_frame_view_t *frame, void *context)
{
    (void)context;
    s_last_rx_time = uart_link_get_last_rx_time();
    s_link_stale_logged = 0U;
    (void)snprintf(s_frame_line, sizeof(s_frame_line),
                   "RX FRAME\r\ntype=0x%02X\r\nlen=%u\r\n",
                   (unsigned)frame->type, (unsigned)frame->length);
    LOG_INFO(s_frame_line);
    /* PONG remains a parse-compatible frame, but no longer drives a
     * periodic heartbeat. Link freshness is based on last_rx_time instead. */
    if (frame->type == SC_TYPE_PONG) {
        diag("PONG_RX");
        return;
    }
    if (frame->type == SC_TYPE_RADAR_PWM_READY && frame->length == 1U) {
        (void)snprintf(s_frame_line, sizeof(s_frame_line),
                       "RADAR_PWM_READY RX\r\nspeed=%u\r\n",
                       (unsigned)frame->payload[0]);
        LOG_INFO(s_frame_line);
        imu_boot_manager_on_radar_pwm_ready(frame->payload[0]);
        return;
    }
    if (frame->type == SC_TYPE_CAL_EVENT_ACK && frame->length == 2U) {
        imu_boot_manager_on_cal_event_ack(frame->payload[0],
                                          frame->payload[1]);
    }
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
        uint8_t type = 0U;
        uint16_t len_field = 0U;
        uint16_t rx_crc = 0U;
        uint16_t calc_crc = 0U;
        const uint16_t parser_buffer_len =
            length > sizeof(s_parser.bytes) ? (uint16_t)sizeof(s_parser.bytes)
                                            : (uint16_t)length;

        reason = "CRC_FAIL";
        ++s_crc_error_count;
        if (data != NULL && length >= 6U) {
            type = data[3];
            len_field = (uint16_t)data[4] | ((uint16_t)data[5] << 8U);
            if (length >= (size_t)SC_FRAME_OVERHEAD + len_field) {
                rx_crc = (uint16_t)data[6U + len_field] |
                         ((uint16_t)data[7U + len_field] << 8U);
                calc_crc = sc_frame_crc16(&data[2], 4U + len_field);
            }
        }
        uart_link_get_stats(&uart_stats);
        (void)snprintf(s_error_line, sizeof(s_error_line),
                       "CRC_FAIL type=0x%02X len_field=%u rx_crc=0x%04X "
                       "calc_crc=0x%04X frame_index=%lu buffer_len=%u "
                       "buffer_expected=%u buffer_capacity=%u buffered=%u "
                       "rx_overflow=%lu rx_drop=%lu\r\n",
                       (unsigned)type, (unsigned)len_field, (unsigned)rx_crc,
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
