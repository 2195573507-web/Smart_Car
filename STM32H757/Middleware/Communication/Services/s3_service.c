#include "s3_service.h"

#include <stdbool.h>
#include <string.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include "imu_boot_manager.h"
#include "log_service.h"
#include "motor_board_task.h"
#include "scbp_link.h"
#include "scbp_parser.h"
#include "scbp_wire.h"
#include "uart_link.h"

#define S3_SERVICE_STACK_WORDS UINT16_C(512)
#define S3_SERVICE_PRIORITY (tskIDLE_PRIORITY + 2U)
#define S3_SERVICE_RX_TIMEOUT_MS UINT32_C(2000)
#define S3_SERVICE_BUS_OFF_RECOVERY_MS UINT32_C(100)
#define S3_SERVICE_LINK_LOCK_MS UINT32_C(20)
#define S3_SERVICE_STACK_MONITOR_PERIOD_MS UINT32_C(5000)
#define S3_SERVICE_S3_FRAME_TIMEOUT_MS UINT32_C(3000)

static scbp_parser_t s_parser;
static scbp_link_t s_link;
static SemaphoreHandle_t s_link_mutex;
static TaskHandle_t s_s3_service_task_handle;
static UBaseType_t s_s3_service_min_free_words = S3_SERVICE_STACK_WORDS;
static uint8_t s_initialized;
static uint8_t s_bus_off_recovery_pending;
static uint32_t s_bus_off_recovery_at_ms;
static uint32_t s_last_rx_time;
static uint8_t s_link_stale_logged;
static uint32_t s_parser_errors;
static uint8_t s_step_bytes[128U];
static char s_line[128U];
static uint32_t s_last_s3_frame_ms;
static uint8_t s_link_timeout_applied;

static uint8_t s3_service_link_lock(void)
{
    return s_link_mutex != NULL &&
           xSemaphoreTake(s_link_mutex, pdMS_TO_TICKS(S3_SERVICE_LINK_LOCK_MS)) == pdTRUE;
}

static void s3_service_link_unlock(void)
{
    if (s_link_mutex != NULL) {
        (void)xSemaphoreGive(s_link_mutex);
    }
}

static int s3_service_transport_send(const uint8_t *data, uint16_t length,
                                     void *context)
{
    (void)context;
    return uart_link_send(data, length) == HAL_OK ? 0 : -1;
}

static void s3_service_on_bus_off(void *context)
{
    (void)context;
    s_link_timeout_applied = 1U;
    if (!motor_board_force_stop()) {
        LOG_ERROR("SCBP_CAN BUS_OFF forced stop PWM queue drop\r\n");
    }
    s_bus_off_recovery_pending = 1U;
    s_bus_off_recovery_at_ms = HAL_GetTick() + S3_SERVICE_BUS_OFF_RECOVERY_MS;
    uart_link_recover();
    LOG_ERROR("SCBP_CAN BUS_OFF; UART2 recovery requested\r\n");
}

static int s3_service_send_locked(uint8_t priority, uint16_t message_id,
                                  uint8_t flags, const uint8_t *payload,
                                  uint8_t length)
{
    return scbp_link_send(&s_link, priority, SCBP_NODE_ESP32_S3, message_id,
                          flags, payload, length, HAL_GetTick(), NULL, NULL);
}

static void s3_service_send_response_locked(const scbp_can_frame_t *request,
                                            uint8_t is_error,
                                            uint8_t status_code)
{
    const uint8_t source = request == NULL ? 0U : SCBP_CAN_ID_SOURCE(request->can_id);

    if (request == NULL || source == SCBP_NODE_BROADCAST) {
        return;
    }
    (void)scbp_link_send_fast_response(
        &s_link, SCBP_CAN_PRIORITY_REALTIME, source, is_error,
        request->can_id, request->sequence, status_code, HAL_GetTick());
}

static void s3_service_on_frame(const scbp_can_frame_t *frame, void *context)
{
    const uint16_t message_id = frame == NULL ? 0U : SCBP_CAN_ID_MESSAGE(frame->can_id);
    const uint8_t source = frame == NULL ? 0U : SCBP_CAN_ID_SOURCE(frame->can_id);
    const uint8_t destination = frame == NULL ? 0U : SCBP_CAN_ID_DESTINATION(frame->can_id);
    const uint8_t ack_required = frame == NULL ? 0U :
        (uint8_t)((frame->flags & SCBP_CAN_FLAG_ACK_REQUIRED) != 0U);

    (void)context;
    if (frame == NULL || source != SCBP_NODE_ESP32_S3 ||
        (destination != SCBP_NODE_STM32H757 && destination != SCBP_NODE_BROADCAST)) {
        return;
    }
    if (scbp_link_get_state(&s_link) == SCBP_LINK_BUS_OFF) {
        return;
    }
    if (message_id == SCBP_MSG_ID_RADAR_PWM_READY) {
        uint8_t admitted = 0U;
        if (frame->length == SCBP_PAYLOAD_RADAR_PWM_READY_SIZE &&
            frame->payload != NULL) {
            admitted = imu_boot_manager_on_radar_pwm_ready(frame->payload[0]);
        }
        if (ack_required != 0U) {
            s3_service_send_response_locked(
                frame, admitted == 0U, admitted != 0U ? SCBP_FAST_RESP_OK :
                                                   SCBP_FAST_RESP_BUSY);
        }
        return;
    }

    if (message_id == SCBP_MSG_ID_PID_PARAMS_CMD) {
        float params[4];
        const bool valid = frame->length == SCBP_PAYLOAD_PID_PARAMS_SIZE &&
                           frame->payload != NULL &&
                           scbp_wire_read_f32_array_le(frame->payload,
                                                       frame->length,
                                                       params, 4U) &&
                           motor_board_update_pid_params(params[0], params[1],
                                                         params[2], params[3]);
        if (valid) {
            (void)snprintf(s_line, sizeof(s_line),
                           "[PID_CONFIG] Updated: Kp=%.3f, Ki=%.3f, Kd=%.3f, Accel=%.1f\r\n",
                           (double)params[0], (double)params[1],
                           (double)params[2], (double)params[3]);
            LOG_INFO(s_line);
        }
        if (ack_required != 0U) {
            s3_service_send_response_locked(
                frame, valid ? 0U : 1U,
                valid ? SCBP_FAST_RESP_OK : SCBP_FAST_RESP_INVALID_PARAM);
        }
        return;
    }

    if (message_id == SCBP_MSG_ID_WHEEL_SPEED_CMD) {
        float speeds[4];
        const bool valid = frame->length == SCBP_PAYLOAD_WHEEL_SPEED_CMD_SIZE &&
                           frame->payload != NULL &&
                           scbp_wire_read_f32_array_le(frame->payload,
                                                       frame->length,
                                                       speeds, 4U) &&
                           motor_board_set_target_wheel_speeds(speeds);
        if (ack_required != 0U) {
            s3_service_send_response_locked(
                frame, valid ? 0U : 1U,
                valid ? SCBP_FAST_RESP_OK : SCBP_FAST_RESP_INVALID_PARAM);
        }
        return;
    }

    if (ack_required != 0U) {
        s3_service_send_response_locked(frame, 1U, SCBP_FAST_RESP_INVALID_PARAM);
    }
}

static void s3_service_on_parsed_frame(const scbp_can_frame_t *frame, void *context)
{
    const uint8_t source = frame == NULL ? 0U : SCBP_CAN_ID_SOURCE(frame->can_id);
    const uint8_t destination = frame == NULL ? 0U : SCBP_CAN_ID_DESTINATION(frame->can_id);

    (void)context;
    if (frame != NULL && source == SCBP_NODE_ESP32_S3 &&
        (destination == SCBP_NODE_STM32H757 || destination == SCBP_NODE_BROADCAST)) {
        if (scbp_link_get_state(&s_link) != SCBP_LINK_BUS_OFF) {
            s_last_s3_frame_ms = HAL_GetTick();
            s_link_timeout_applied = 0U;
        }
    }
    scbp_link_receive(&s_link, frame);
}

static void s3_service_on_parser_error(scbp_parser_error_t error,
                                       const uint8_t *data, size_t length,
                                       void *context)
{
    const char *reason = "UNKNOWN";
    (void)data;
    (void)context;

    ++s_parser_errors;
    scbp_link_report_parser_error(&s_link, error);
    switch (error) {
    case SCBP_PARSER_ERROR_HCS: reason = "HCS"; break;
    case SCBP_PARSER_ERROR_FCS: reason = "FCS"; break;
    case SCBP_PARSER_ERROR_EOF: reason = "EOF"; break;
    case SCBP_PARSER_ERROR_FLAGS: reason = "FLAGS"; break;
    case SCBP_PARSER_ERROR_NODE: reason = "NODE"; break;
    default: break;
    }
    (void)snprintf(s_line, sizeof(s_line),
                   "SCBP_CAN_RX_ERROR reason=%s bytes=%lu rec=%u tec=%u\r\n",
                   reason, (unsigned long)length,
                   (unsigned)scbp_link_get_rec(&s_link),
                   (unsigned)scbp_link_get_tec(&s_link));
    LOG_WARN(s_line);
}

static void s3_service_log_stack(void)
{
    const UBaseType_t free_words = s_s3_service_task_handle == NULL
                                        ? 0U
                                        : uxTaskGetStackHighWaterMark(
                                              s_s3_service_task_handle);

    if (free_words < s_s3_service_min_free_words) {
        s_s3_service_min_free_words = free_words;
    }
    (void)snprintf(s_line, sizeof(s_line),
                   "[S3_TASK_STACK] free_words=%lu min_free_words=%lu\r\n",
                   (unsigned long)free_words,
                   (unsigned long)s_s3_service_min_free_words);
    LOG_INFO(s_line);
}

void s3_service_init(void)
{
    const scbp_link_config_t config = {
        .local_node = SCBP_NODE_STM32H757,
        .ack_timeout_ms = SCBP_LINK_ACK_TIMEOUT_MS,
        .max_retries = SCBP_LINK_MAX_RETRIES,
        .transport_send = s3_service_transport_send,
        .on_frame = s3_service_on_frame,
        .on_bus_off = s3_service_on_bus_off,
        .context = NULL,
    };

    if (s_initialized != 0U) {
        return;
    }
    s_link_mutex = xSemaphoreCreateMutex();
    if (s_link_mutex == NULL) {
        LOG_ERROR("SCBP_CAN link mutex allocation failed\r\n");
        return;
    }
    scbp_link_init(&s_link, &config);
    scbp_parser_init(&s_parser, s3_service_on_parsed_frame, s3_service_on_parser_error,
                     NULL);
    s_bus_off_recovery_pending = 0U;
    s_bus_off_recovery_at_ms = 0U;
    s_last_rx_time = 0U;
    s_last_s3_frame_ms = HAL_GetTick();
    s_link_timeout_applied = 0U;
    s_link_stale_logged = 0U;
    s_parser_errors = 0U;
    s_s3_service_min_free_words = S3_SERVICE_STACK_WORDS;
    s_initialized = 1U;
    imu_boot_manager_set_transport(s3_service_send_boot_message);
}

int s3_service_send_message(uint8_t priority, uint16_t message_id, uint8_t flags,
                            const uint8_t *payload, uint8_t length)
{
    int result;

    if (s_initialized == 0U || s3_service_link_lock() == 0U) {
        return -1;
    }
    result = s3_service_send_locked(priority, message_id, flags, payload, length);
    s3_service_link_unlock();
    return result;
}

void s3_service_send_boot_message(uint16_t message_id, uint8_t flags,
                                  const uint8_t *payload, uint8_t length)
{
    (void)s3_service_send_message(SCBP_CAN_PRIORITY_REALTIME, message_id, flags,
                                  payload, length);
}

void s3_service_send_imu_telemetry(const uint8_t *payload, uint8_t length)
{
    if (payload == NULL || length != SCBP_PAYLOAD_IMU_TELEMETRY_SIZE) {
        return;
    }
    (void)s3_service_send_message(SCBP_CAN_PRIORITY_NORMAL,
                                  SCBP_MSG_ID_IMU_TELEMETRY,
                                  SCBP_CAN_FLAG_STREAM_DATA, payload, length);
}

void s3_service_send_dual_attitude(const uint8_t *payload, uint8_t length)
{
    if (payload == NULL || length != SCBP_PAYLOAD_DUAL_AHRS_SIZE ||
        payload[0] != SCBP_DUAL_AHRS_SCHEMA || payload[2] != 0U ||
        payload[3] != 0U) {
        return;
    }
    (void)s3_service_send_message(SCBP_CAN_PRIORITY_REALTIME,
                                  SCBP_MSG_ID_ATTITUDE,
                                  SCBP_CAN_FLAG_STREAM_DATA, payload, length);
}

int s3_service_send_log(const uint8_t *payload, uint8_t length)
{
    return s3_service_send_message(SCBP_CAN_PRIORITY_DEBUG, SCBP_MSG_ID_LOG,
                                   SCBP_CAN_FLAG_STREAM_DATA, payload, length);
}

void s3_service_step(void)
{
    const size_t length = uart_link_read(s_step_bytes, sizeof(s_step_bytes));
    const uint32_t now_ms = HAL_GetTick();
    const uint32_t latest_rx_time = uart_link_get_last_rx_time();

    if (s_initialized == 0U) {
        return;
    }
    if (s_link_timeout_applied == 0U &&
        (uint32_t)(now_ms - s_last_s3_frame_ms) >=
            S3_SERVICE_S3_FRAME_TIMEOUT_MS) {
        if (motor_board_force_stop()) {
            s_link_timeout_applied = 1U;
            LOG_WARN("SCBP S3 link timeout; PID reset and PWM stopped\r\n");
        }
    }
    if (s3_service_link_lock() != 0U) {
        if (length != 0U) {
            (void)scbp_parser_feed(&s_parser, s_step_bytes, length);
        }
        scbp_link_tick(&s_link, now_ms);
        if (s_bus_off_recovery_pending != 0U &&
            (uint32_t)(now_ms - s_bus_off_recovery_at_ms) < UINT32_C(0x80000000)) {
            scbp_link_recover(&s_link);
            s_bus_off_recovery_pending = 0U;
            LOG_INFO("SCBP_CAN link recovered after UART2 reset\r\n");
        }
        s3_service_link_unlock();
    }

    if (latest_rx_time != 0U && latest_rx_time != s_last_rx_time) {
        s_last_rx_time = latest_rx_time;
        s_link_stale_logged = 0U;
    }
    if (s_last_rx_time != 0U &&
        (uint32_t)(now_ms - s_last_rx_time) >= S3_SERVICE_RX_TIMEOUT_MS &&
        s_link_stale_logged == 0U) {
        LOG_WARN("SCBP_CAN UART2 receive stale\r\n");
        s_link_stale_logged = 1U;
    }
}

void s3_service_task(void *argument)
{
    uint32_t last_stack_monitor_ms;

    (void)argument;
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
    s3_service_init();
    if (s_initialized == 0U || s_s3_service_task_handle != NULL) {
        return;
    }
    if (xTaskCreate(s3_service_task, "s3_service", S3_SERVICE_STACK_WORDS,
                    NULL, S3_SERVICE_PRIORITY, &s_s3_service_task_handle) != pdPASS) {
        s_s3_service_task_handle = NULL;
        LOG_ERROR("s3_service task creation failed\r\n");
    }
}
