#include "ros_motion_gateway.h"
#include "ros_motion_stream.h"

#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <string.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>
#include <netdb.h>

#include "esp_log.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "mbedtls/md.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "smartcar_service.h"
#include "smartcar_wifi_sta.h"

#ifndef CONFIG_ROS_MOTION_GATEWAY_ENABLED
#define CONFIG_ROS_MOTION_GATEWAY_ENABLED 0
#endif
#ifndef CONFIG_ROS_MOTION_GATEWAY_HOST
#define CONFIG_ROS_MOTION_GATEWAY_HOST ""
#endif
#ifndef CONFIG_ROS_MOTION_GATEWAY_PSK
#define CONFIG_ROS_MOTION_GATEWAY_PSK ""
#endif
#ifndef CONFIG_ROS_MOTION_LINEAR_LIMIT_MM_S
#define CONFIG_ROS_MOTION_LINEAR_LIMIT_MM_S ROS_MOTION_DEFAULT_LINEAR_LIMIT_MM_S
#endif
#ifndef CONFIG_ROS_MOTION_ANGULAR_LIMIT_MRAD_S
#define CONFIG_ROS_MOTION_ANGULAR_LIMIT_MRAD_S ROS_MOTION_DEFAULT_ANGULAR_LIMIT_MRAD_S
#endif

#define ROS_MOTION_TASK_STACK 8192U
#define ROS_MOTION_TASK_PRIORITY 7U
#define ROS_MOTION_CONNECT_TIMEOUT_MS 500U
#define ROS_MOTION_RX_TIMEOUT_MS 25U
#define ROS_MOTION_TX_TIMEOUT_MS 20U
#define ROS_MOTION_TELEMETRY_MAX_SIZE 128U
#define ROS_MOTION_DIAG_LOG_INTERVAL_MS UINT64_C(2000)

static const char *TAG = "ROS_MOTION";

#if CONFIG_ROS_MOTION_GATEWAY_ENABLED
static TaskHandle_t s_task;
static bool s_initialized;
static bool s_service_bound;
static int s_socket = -1;
static uint32_t s_tx_sequence;
static ros_motion_state_machine_t s_state;
static ros_motion_stream_t s_stream;
static uint8_t s_telemetry[ROS_MOTION_TELEMETRY_MAX_SIZE];
static uint16_t s_telemetry_length;
static uint16_t s_telemetry_message_id;
static volatile bool s_telemetry_pending;
static volatile bool s_force_stop;
static portMUX_TYPE s_telemetry_lock = portMUX_INITIALIZER_UNLOCKED;

typedef enum {
    ROS_MOTION_CONNECT_STAGE_NONE = 0U,
    ROS_MOTION_CONNECT_STAGE_RESOLVE,
    ROS_MOTION_CONNECT_STAGE_SOCKET,
    ROS_MOTION_CONNECT_STAGE_NONBLOCK,
    ROS_MOTION_CONNECT_STAGE_CONNECT,
    ROS_MOTION_CONNECT_STAGE_SELECT,
    ROS_MOTION_CONNECT_STAGE_SOCKET_ERROR
} ros_motion_connect_stage_t;

/* Cumulative, secret-free diagnostics. The telemetry sink and ROS task run
 * independently, so every counter is accessed with GCC atomics. */
typedef struct {
    uint32_t connect_attempts;
    uint32_t connect_successes;
    uint32_t connect_failures;
    uint32_t last_connect_stage;
    int32_t last_connect_error;
    uint32_t hello_sent;
    uint32_t hello_send_failures;
    uint32_t hello_acknowledged;
    uint32_t peer_closed;
    uint32_t receive_failures;
    uint32_t protocol_failures;
    uint32_t last_protocol_error;
    uint32_t rejected_frames;
    uint32_t last_reject_code;
    uint32_t chassis_state_slots;
    uint32_t chassis_state_sent;
    uint32_t chassis_state_send_failures;
} ros_motion_diagnostics_t;

static ros_motion_diagnostics_t s_diagnostics;

static uint64_t now_ms(void)
{
    return (uint64_t)esp_timer_get_time() / UINT64_C(1000);
}

static void diagnostic_increment(uint32_t *counter)
{
    (void)__atomic_fetch_add(counter, 1U, __ATOMIC_RELAXED);
}

static void diagnostic_store_u32(uint32_t *value, uint32_t next)
{
    __atomic_store_n(value, next, __ATOMIC_RELAXED);
}

static void diagnostic_store_i32(int32_t *value, int32_t next)
{
    __atomic_store_n(value, next, __ATOMIC_RELAXED);
}

static uint32_t diagnostic_load_u32(const uint32_t *value)
{
    return __atomic_load_n(value, __ATOMIC_RELAXED);
}

static int32_t diagnostic_load_i32(const int32_t *value)
{
    return __atomic_load_n(value, __ATOMIC_RELAXED);
}

/** Emit a compact state snapshot without host, PSK, session, or payload data. */
static void diagnostic_log_snapshot(void)
{
    static uint64_t last_log_ms;
    const uint64_t timestamp_ms = now_ms();

    if (timestamp_ms - last_log_ms < ROS_MOTION_DIAG_LOG_INTERVAL_MS) {
        return;
    }
    last_log_ms = timestamp_ms;
    ESP_LOGI(TAG,
             "DIAG socket=%u state=%u conn=%" PRIu32 "/%" PRIu32 "/%" PRIu32
             " last_conn=%" PRIu32 "/%" PRId32
             " hello=%" PRIu32 "/%" PRIu32 "/%" PRIu32
             " rx=%" PRIu32 "/%" PRIu32 " proto=%" PRIu32 "/%" PRIu32
             " reject=%" PRIu32 "/%" PRIu32
             " state15=%" PRIu32 "/%" PRIu32 "/%" PRIu32,
             s_socket >= 0 ? 1U : 0U, (unsigned)s_state.state,
             diagnostic_load_u32(&s_diagnostics.connect_attempts),
             diagnostic_load_u32(&s_diagnostics.connect_successes),
             diagnostic_load_u32(&s_diagnostics.connect_failures),
             diagnostic_load_u32(&s_diagnostics.last_connect_stage),
             diagnostic_load_i32(&s_diagnostics.last_connect_error),
             diagnostic_load_u32(&s_diagnostics.hello_sent),
             diagnostic_load_u32(&s_diagnostics.hello_send_failures),
             diagnostic_load_u32(&s_diagnostics.hello_acknowledged),
             diagnostic_load_u32(&s_diagnostics.peer_closed),
             diagnostic_load_u32(&s_diagnostics.receive_failures),
             diagnostic_load_u32(&s_diagnostics.protocol_failures),
             diagnostic_load_u32(&s_diagnostics.last_protocol_error),
             diagnostic_load_u32(&s_diagnostics.rejected_frames),
             diagnostic_load_u32(&s_diagnostics.last_reject_code),
             diagnostic_load_u32(&s_diagnostics.chassis_state_slots),
             diagnostic_load_u32(&s_diagnostics.chassis_state_sent),
             diagnostic_load_u32(&s_diagnostics.chassis_state_send_failures));
}

static uint32_t next_tx_sequence(void)
{
    return ++s_tx_sequence;
}

static uint32_t new_session_id(void)
{
    uint32_t session_id;

    do {
        session_id = esp_random();
    } while (session_id == 0U);
    return session_id;
}

static bool auth_tag(const uint8_t *data, size_t length,
                     uint8_t tag[ROS_MOTION_AUTH_TAG_SIZE], void *context)
{
    const mbedtls_md_info_t *info = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);
    const char *psk = context == NULL ? CONFIG_ROS_MOTION_GATEWAY_PSK : context;
    unsigned char digest[32];

    if (info == NULL || psk == NULL || psk[0] == '\0' ||
        mbedtls_md_hmac(info, (const unsigned char *)psk, strlen(psk), data,
                        length, digest) != 0) {
        return false;
    }
    memcpy(tag, digest, ROS_MOTION_AUTH_TAG_SIZE);
    return true;
}

static bool wifi_connected(void)
{
    return smartcar_wifi_sta_is_connected();
}

static void close_socket(void)
{
    if (s_socket >= 0) {
        shutdown(s_socket, SHUT_RDWR);
        close(s_socket);
        s_socket = -1;
    }
    ros_motion_stream_init(&s_stream);
    ros_motion_state_revoke(&s_state);
    __atomic_store_n(&s_force_stop, false, __ATOMIC_RELEASE);
    (void)smartcar_service_ros_motion_stop();
}

static int connect_endpoint(uint32_t *failure_stage, int32_t *failure_error)
{
    struct addrinfo hints = {0};
    struct addrinfo *result = NULL;
    struct timeval timeout = {
        .tv_sec = ROS_MOTION_CONNECT_TIMEOUT_MS / 1000U,
        .tv_usec = (ROS_MOTION_CONNECT_TIMEOUT_MS % 1000U) * 1000U
    };
    int fd;
    int status;

    if (failure_stage == NULL || failure_error == NULL) {
        return -1;
    }
    *failure_stage = ROS_MOTION_CONNECT_STAGE_NONE;
    *failure_error = 0;

    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_protocol = IPPROTO_TCP;
    status = getaddrinfo(CONFIG_ROS_MOTION_GATEWAY_HOST, "8766", &hints, &result);
    if (status != 0 || result == NULL) {
        *failure_stage = ROS_MOTION_CONNECT_STAGE_RESOLVE;
        *failure_error = status;
        return -1;
    }
    fd = socket(result->ai_family, result->ai_socktype, result->ai_protocol);
    if (fd < 0) {
        *failure_stage = ROS_MOTION_CONNECT_STAGE_SOCKET;
        *failure_error = errno;
        freeaddrinfo(result);
        return -1;
    }
    const int flags = fcntl(fd, F_GETFL, 0);
    if (flags < 0 || fcntl(fd, F_SETFL, flags | O_NONBLOCK) < 0) {
        *failure_stage = ROS_MOTION_CONNECT_STAGE_NONBLOCK;
        *failure_error = errno;
        close(fd);
        freeaddrinfo(result);
        return -1;
    }
    status = connect(fd, result->ai_addr, result->ai_addrlen);
    if (status != 0 && errno != EINPROGRESS) {
        *failure_stage = ROS_MOTION_CONNECT_STAGE_CONNECT;
        *failure_error = errno;
        close(fd);
        freeaddrinfo(result);
        return -1;
    }
    if (status != 0) {
        fd_set write_fds;
        FD_ZERO(&write_fds);
        FD_SET(fd, &write_fds);
        status = select(fd + 1, NULL, &write_fds, NULL, &timeout);
        if (status <= 0 || !FD_ISSET(fd, &write_fds)) {
            *failure_stage = ROS_MOTION_CONNECT_STAGE_SELECT;
            *failure_error = status == 0 ? ETIMEDOUT : errno;
            close(fd);
            freeaddrinfo(result);
            return -1;
        }
        int socket_error = 0;
        socklen_t error_length = sizeof(socket_error);
        if (getsockopt(fd, SOL_SOCKET, SO_ERROR, &socket_error, &error_length) != 0 ||
            socket_error != 0) {
            *failure_stage = ROS_MOTION_CONNECT_STAGE_SOCKET_ERROR;
            *failure_error = socket_error != 0 ? socket_error : errno;
            close(fd);
            freeaddrinfo(result);
            return -1;
        }
    }
    freeaddrinfo(result);
    return fd;
}

static int send_all(const uint8_t *data, size_t length)
{
    size_t offset = 0U;
    const uint64_t deadline_ms = now_ms() + ROS_MOTION_TX_TIMEOUT_MS;

    while (offset < length) {
        const int written = (int)send(s_socket, &data[offset], length - offset, 0);
        if (written > 0) {
            offset += (size_t)written;
        } else if (written < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            if (now_ms() >= deadline_ms) {
                return -1;
            }
            vTaskDelay(pdMS_TO_TICKS(1U));
        } else {
            return -1;
        }
    }
    return 0;
}

static int send_frame(uint8_t type, uint32_t sequence, uint32_t lease_id,
                      uint16_t ttl_ms, const uint8_t *payload,
                      uint16_t payload_length)
{
    uint8_t output[ROS_MOTION_MAX_FRAME_SIZE];
    size_t output_length = 0U;
    const ros_motion_frame_view_t frame = {
        .version = ROS_MOTION_PROTOCOL_VERSION,
        .type = type,
        .flags = ROS_MOTION_FLAG_AUTH_PRESENT,
        .payload_length = payload_length,
        .session_id = s_state.session_id,
        .sequence = sequence,
        .lease_id = lease_id,
        .ttl_ms = ttl_ms,
        .payload = payload
    };
    if (ros_motion_encode(&frame, auth_tag, (void *)CONFIG_ROS_MOTION_GATEWAY_PSK,
                          output, sizeof(output), &output_length) != ROS_MOTION_OK) {
        return -1;
    }
    return send_all(output, output_length);
}

static bool send_hello(void)
{
    if (send_frame(ROS_MOTION_TYPE_HELLO, next_tx_sequence(), 0U, 0U,
                   NULL, 0U) != 0) {
        diagnostic_increment(&s_diagnostics.hello_send_failures);
        return false;
    }
    diagnostic_increment(&s_diagnostics.hello_sent);
    return true;
}

static void send_error(uint8_t error_code)
{
    (void)send_frame(ROS_MOTION_TYPE_ERROR, next_tx_sequence(), 0U, 0U,
                     &error_code, 1U);
}

static void request_safe_stop(void)
{
    __atomic_store_n(&s_force_stop, true, __ATOMIC_RELEASE);
    (void)smartcar_service_ros_motion_stop();
}

static void reject_frame(uint8_t error_code)
{
    diagnostic_increment(&s_diagnostics.rejected_frames);
    diagnostic_store_u32(&s_diagnostics.last_reject_code, error_code);
    request_safe_stop();
    send_error(error_code);
    close_socket();
}

static void send_lease_response(uint8_t status)
{
    uint8_t payload[3] = {status, (uint8_t)ROS_MOTION_DEFAULT_LEASE_MS,
                          (uint8_t)(ROS_MOTION_DEFAULT_LEASE_MS >> 8U)};
    (void)send_frame(ROS_MOTION_TYPE_LEASE_RESPONSE, next_tx_sequence(),
                     s_state.lease_id, ROS_MOTION_DEFAULT_LEASE_MS, payload,
                     sizeof(payload));
}

static void send_status(uint8_t status)
{
    (void)send_frame(ROS_MOTION_TYPE_STATUS, next_tx_sequence(),
                     s_state.lease_id, 0U, &status, 1U);
}

static void send_telemetry(void)
{
    uint8_t payload[1U + ROS_MOTION_TELEMETRY_MAX_SIZE];
    uint16_t length;
    bool pending;

    if (s_socket < 0) {
        return;
    }
    portENTER_CRITICAL(&s_telemetry_lock);
    pending = __atomic_exchange_n(&s_telemetry_pending, false, __ATOMIC_ACQ_REL);
    if (pending) {
        length = s_telemetry_length;
        payload[0] = (uint8_t)s_telemetry_message_id;
        memcpy(&payload[1], s_telemetry, length);
    }
    portEXIT_CRITICAL(&s_telemetry_lock);
    if (!pending) {
        return;
    }
    if (send_frame(ROS_MOTION_TYPE_STATUS, next_tx_sequence(),
                   s_state.lease_id, 0U, payload,
                   (uint16_t)(length + 1U)) == 0) {
        diagnostic_increment(&s_diagnostics.chassis_state_sent);
    } else {
        diagnostic_increment(&s_diagnostics.chassis_state_send_failures);
    }
}

static bool inbound_payload_is_valid(const ros_motion_frame_view_t *frame)
{
    if (frame == NULL) {
        return false;
    }
    switch (frame->type) {
    case ROS_MOTION_TYPE_HELLO_ACK:
    case ROS_MOTION_TYPE_LEASE_REQUEST:
    case ROS_MOTION_TYPE_STOP:
    case ROS_MOTION_TYPE_HEARTBEAT:
        return frame->payload_length == 0U;
    case ROS_MOTION_TYPE_MOTION_CMD:
        return frame->payload_length == 8U;
    default:
        return false;
    }
}

static void process_frame(const ros_motion_frame_view_t *frame)
{
    if (frame == NULL || frame->session_id != s_state.session_id) {
        reject_frame(1U);
        return;
    }
    if (!inbound_payload_is_valid(frame)) {
        reject_frame(4U);
        return;
    }
    switch (frame->type) {
    case ROS_MOTION_TYPE_HELLO_ACK:
        if (!ros_motion_state_hello_ack(&s_state, frame->session_id,
                                        frame->sequence)) {
            reject_frame(2U);
        } else {
            diagnostic_increment(&s_diagnostics.hello_acknowledged);
            send_status((uint8_t)ROS_MOTION_STATE_READY);
        }
        break;
    case ROS_MOTION_TYPE_LEASE_REQUEST:
        if (!ros_motion_state_request_lease(&s_state, frame->session_id,
                                            frame->lease_id, frame->sequence,
                                            frame->ttl_ms, now_ms())) {
            reject_frame(3U);
        } else {
            if (smartcar_service_ros_motion_set_lease(true) != ESP_OK) {
                reject_frame(3U);
            } else {
                send_lease_response(0U);
            }
        }
        break;
    case ROS_MOTION_TYPE_MOTION_CMD: {
        ros_motion_command_t command = {0};
        if (!ros_motion_read_f32_le(&frame->payload[0], &command.linear_m_s) ||
            !ros_motion_read_f32_le(&frame->payload[4], &command.angular_rad_s)) {
            reject_frame(4U);
            break;
        }
        command.session_id = frame->session_id;
        command.lease_id = frame->lease_id;
        command.sequence = frame->sequence;
        command.ttl_ms = frame->ttl_ms;
        if (!ros_motion_state_accept_command(
                &s_state, &command, now_ms(), CONFIG_ROS_MOTION_LINEAR_LIMIT_MM_S,
                CONFIG_ROS_MOTION_ANGULAR_LIMIT_MRAD_S)) {
            reject_frame(5U);
            break;
        }
        smartcar_service_ros_motion_command_t service_command = {
            .linear_m_s = command.linear_m_s,
            .angular_rad_s = command.angular_rad_s,
            .session_id = command.session_id,
            .lease_id = command.lease_id,
            .sequence = command.sequence,
            .ttl_ms = command.ttl_ms
        };
        if (smartcar_service_ros_motion_submit(&service_command) != ESP_OK) {
            reject_frame(5U);
        }
        break;
    }
    case ROS_MOTION_TYPE_STOP:
        if (!ros_motion_state_accept_stop(&s_state, frame->session_id,
                                          frame->lease_id, frame->sequence,
                                          now_ms())) {
            reject_frame(6U);
        } else {
            /* STOP revokes the lease; a fresh TCP handshake is required before
             * any new lease or command can be accepted. */
            close_socket();
        }
        break;
    case ROS_MOTION_TYPE_HEARTBEAT:
        if (!ros_motion_state_accept_heartbeat(&s_state, frame->session_id,
                                               frame->lease_id, frame->sequence,
                                               now_ms())) {
            reject_frame(7U);
        }
        break;
    default:
        reject_frame(8U);
        break;
    }
}

static void stream_error(int status, void *context)
{
    (void)context;
    diagnostic_increment(&s_diagnostics.protocol_failures);
    diagnostic_store_u32(&s_diagnostics.last_protocol_error,
                         (uint32_t)(status < 0 ? -status : status));
    reject_frame((uint8_t)(status < 0 ? -status : status));
}

static void stream_frame(const ros_motion_frame_view_t *frame, void *context)
{
    (void)context;
    process_frame(frame);
}

static void drain_rx(void)
{
    uint8_t buffer[64];
    const int count = (int)recv(s_socket, buffer, sizeof(buffer), 0);
    if (count == 0) {
        diagnostic_increment(&s_diagnostics.peer_closed);
        close_socket();
        return;
    }
    if (count < 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
            diagnostic_increment(&s_diagnostics.receive_failures);
            close_socket();
        }
        return;
    }
    (void)ros_motion_stream_feed(&s_stream, buffer, (size_t)count,
                                 auth_tag,
                                 (void *)CONFIG_ROS_MOTION_GATEWAY_PSK,
                                 stream_frame, stream_error, NULL);
}

static bool stop_if_required(void)
{
    if (!__atomic_exchange_n(&s_force_stop, false, __ATOMIC_ACQ_REL) &&
        !ros_motion_state_expired(&s_state, now_ms())) {
        return false;
    }
    (void)smartcar_service_ros_motion_stop();
    send_status((uint8_t)ROS_MOTION_STATE_FAULT);
    close_socket();
    return true;
}

static void ros_motion_task(void *context)
{
    (void)context;
    for (;;) {
        if (s_socket < 0) {
            if (wifi_connected()) {
                uint32_t failure_stage = ROS_MOTION_CONNECT_STAGE_NONE;
                int32_t failure_error = 0;

                diagnostic_increment(&s_diagnostics.connect_attempts);
                s_socket = connect_endpoint(&failure_stage, &failure_error);
                if (s_socket >= 0) {
                    diagnostic_increment(&s_diagnostics.connect_successes);
                    diagnostic_store_u32(&s_diagnostics.last_connect_stage,
                                         ROS_MOTION_CONNECT_STAGE_NONE);
                    diagnostic_store_i32(&s_diagnostics.last_connect_error, 0);
                    s_tx_sequence = 0U;
                    s_state.session_id = new_session_id();
                    ros_motion_state_connected(&s_state, s_state.session_id);
                    ros_motion_stream_init(&s_stream);
                    (void)send_hello();
                } else {
                    diagnostic_increment(&s_diagnostics.connect_failures);
                    diagnostic_store_u32(&s_diagnostics.last_connect_stage,
                                         failure_stage);
                    diagnostic_store_i32(&s_diagnostics.last_connect_error,
                                         failure_error);
                }
            }
            if (s_socket < 0) {
                diagnostic_log_snapshot();
                vTaskDelay(pdMS_TO_TICKS(100U));
                continue;
            }
        }
        if (stop_if_required()) {
            continue;
        }
        fd_set read_fds;
        struct timeval timeout = {.tv_sec = 0, .tv_usec = ROS_MOTION_RX_TIMEOUT_MS * 1000};
        FD_ZERO(&read_fds);
        FD_SET(s_socket, &read_fds);
        if (select(s_socket + 1, &read_fds, NULL, NULL, &timeout) > 0 &&
            FD_ISSET(s_socket, &read_fds)) {
            drain_rx();
        }
        if (stop_if_required()) {
            continue;
        }
        send_telemetry();
        if (stop_if_required()) {
            continue;
        }
        diagnostic_log_snapshot();
        vTaskDelay(pdMS_TO_TICKS(ROS_MOTION_WATCHDOG_PERIOD_MS));
    }
}
#endif

esp_err_t ros_motion_gateway_bind_service(void)
{
#if !CONFIG_ROS_MOTION_GATEWAY_ENABLED
    return ESP_OK;
#else
    esp_err_t ret;

    if (s_initialized || s_service_bound) {
        return ESP_ERR_INVALID_STATE;
    }
    ret = smartcar_service_add_telemetry_sink(ros_motion_gateway_telemetry_sink,
                                              NULL);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = smartcar_service_set_ros_safety_callback(ros_motion_gateway_on_safety_stop,
                                                   NULL);
    if (ret != ESP_OK) {
        return ret;
    }
    s_service_bound = true;
    return ESP_OK;
#endif
}

esp_err_t ros_motion_gateway_init(void)
{
#if !CONFIG_ROS_MOTION_GATEWAY_ENABLED
    ESP_LOGI(TAG, "DISABLED");
    return ESP_OK;
#else
    if (s_initialized || !s_service_bound) {
        return ESP_ERR_INVALID_STATE;
    }
    if (CONFIG_ROS_MOTION_GATEWAY_HOST[0] == '\0' ||
        CONFIG_ROS_MOTION_GATEWAY_PSK[0] == '\0') {
        ESP_LOGE(TAG, "CONFIG INCOMPLETE; DISABLED");
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t ret = smartcar_wifi_sta_start();
    if (ret != ESP_OK) {
        return ret;
    }
    ros_motion_state_init(&s_state);
    s_initialized = true;
    if (xTaskCreate(ros_motion_task, "ros_motion", ROS_MOTION_TASK_STACK, NULL,
                    ROS_MOTION_TASK_PRIORITY, &s_task) != pdPASS) {
        s_initialized = false;
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "READY TCP control port=%u", ROS_MOTION_TCP_PORT);
    return ESP_OK;
#endif
}

bool ros_motion_gateway_is_running(void)
{
#if !CONFIG_ROS_MOTION_GATEWAY_ENABLED
    return false;
#else
    return s_initialized && s_task != NULL;
#endif
}

bool ros_motion_gateway_telemetry_sink(uint16_t message_id,
                                       const uint8_t *encoded_frame,
                                       uint16_t encoded_length,
                                       uint32_t ingress_timestamp_ms,
                                       void *context)
{
#if !CONFIG_ROS_MOTION_GATEWAY_ENABLED
    (void)message_id;
    (void)encoded_frame;
    (void)encoded_length;
    (void)ingress_timestamp_ms;
    (void)context;
    return false;
#else
    (void)context;
    if (!s_initialized || message_id != 0x15U || encoded_frame == NULL ||
        encoded_length == 0U || encoded_length > sizeof(s_telemetry)) {
        return false;
    }
    portENTER_CRITICAL(&s_telemetry_lock);
    memcpy(s_telemetry, encoded_frame, encoded_length);
    s_telemetry_length = encoded_length;
    s_telemetry_message_id = message_id;
    __atomic_store_n(&s_telemetry_pending, true, __ATOMIC_RELEASE);
    portEXIT_CRITICAL(&s_telemetry_lock);
    diagnostic_increment(&s_diagnostics.chassis_state_slots);
    return true;
#endif
}

void ros_motion_gateway_on_safety_stop(void *context)
{
    (void)context;
#if CONFIG_ROS_MOTION_GATEWAY_ENABLED
    __atomic_store_n(&s_force_stop, true, __ATOMIC_RELEASE);
#endif
}
