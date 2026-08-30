#include "radar_uplink.h"

#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <netdb.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <sys/time.h>
#include <unistd.h>

#include "esp_event.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "lwip/inet.h"
#include "radar_uplink_protocol.h"
#include "radar_uplink_tx.h"
#include "radar_telemetry_queue.h"
#include "radar_uart.h"
#include "s3_ble.h"
#include "smartcar_service.h"

#include "sdkconfig.h"
#if CONFIG_SMARTCAR_RADAR_UPLINK_ENABLED
#include "radar_wifi_credentials.h"
#endif

static const char *TAG = "RADAR_UPLINK";

#define RADAR_UPLINK_WIFI_CONNECTED BIT0
#define RADAR_UPLINK_WIFI_ROTATE BIT1
#define RADAR_UPLINK_TASK_STACK_SIZE 6144U
#define RADAR_UPLINK_TASK_PRIORITY 4U
/* Notifications wake the sender; this timeout still services Wi-Fi state. */
#define RADAR_UPLINK_WAIT_MS 100U
/* Drain a bounded burst after a TCP stall without holding the UART FIFO lock. */
#define RADAR_UPLINK_BURST_MAX_FRAMES 4U
#define RADAR_UPLINK_STATS_INTERVAL_MS 1000U
#define RADAR_UPLINK_CONNECT_TIMEOUT_MS 500U
#define RADAR_UPLINK_RETRY_INITIAL_MS 500U
#define RADAR_UPLINK_RETRY_MAX_MS 10000U
#define RADAR_UPLINK_RETRY_YIELD_TICKS 1U
#define RADAR_UPLINK_WIFI_SSID_MAX_LEN 32U
#define RADAR_UPLINK_WIFI_PASSWORD_MAX_LEN 64U
#define RADAR_UPLINK_TELEMETRY_WHEEL_FIFO_DEPTH 32U
/* Candidate operational limits; tune only after a real capture. */
#define RADAR_UPLINK_MAX_RADAR_DEQUEUE_AGE_MS 500U
#define RADAR_UPLINK_MAX_TELEMETRY_AGE_MS 1000U
#define RADAR_UPLINK_TELEMETRY_MUTEX_WAIT_TICKS 0U

#if CONFIG_SMARTCAR_RADAR_UPLINK_ENABLED
static void radar_uplink_ble_log(smartcar_log_level_t level,
                                 const char *format,
                                 ...)
{
    char message[SMARTCAR_LOG_MAX_PAYLOAD + 1U];
    va_list args;
    int written;

    if (format == NULL) {
        return;
    }

    va_start(args, format);
    written = vsnprintf(message, sizeof(message), format, args);
    va_end(args);
    if (written <= 0 || (size_t)written >= sizeof(message)) {
        return;
    }

    switch (level) {
    case SMARTCAR_LOG_LEVEL_INFO:
        (void)s3_log_info(message);
        break;
    case SMARTCAR_LOG_LEVEL_WARN:
        (void)s3_log_warn(message);
        break;
    case SMARTCAR_LOG_LEVEL_ERROR:
        (void)s3_log_error(message);
        break;
    default:
        break;
    }
}
#endif

#if CONFIG_SMARTCAR_RADAR_UPLINK_ENABLED
static EventGroupHandle_t s_wifi_events;
static TaskHandle_t s_uplink_task;
static bool s_initialized;
static bool s_wifi_started;
static int s_socket = -1;
static size_t s_wifi_credential_index;
static radar_telemetry_queue_t s_telemetry_queue;
static radar_telemetry_entry_t *s_telemetry_wheel_entries;
static radar_telemetry_entry_t s_telemetry_attitude_entry;
static radar_telemetry_entry_t s_telemetry_imu_entries[RADAR_TELEMETRY_QUEUE_IMU_SLOT_COUNT];
static SemaphoreHandle_t s_telemetry_mutex;
static StaticSemaphore_t s_telemetry_mutex_storage;
static bool s_telemetry_queue_ready;
static volatile uint32_t s_telemetry_lock_drops;
typedef struct {
    uint32_t successful_frames;
    uint64_t successful_bytes;
    uint32_t send_failures;
    uint32_t send_timeouts;
    uint32_t partial_writes;
    uint32_t pending_retries;
    uint32_t connect_failures;
    uint32_t reconnects;
    uint32_t resync_discarded_frames;
    uint32_t encode_failures;
    uint32_t radar_stale_drops;
    uint32_t radar_sequence_gaps;
    uint32_t telemetry_stale_drops;
    uint32_t telemetry_encode_failures;
    uint32_t telemetry_sent_frames;
    uint64_t telemetry_sent_bytes;
    uint32_t last_telemetry_sequence;
    uint32_t telemetry_lock_drops;
    uint32_t last_sent_sequence;
    uint32_t max_dequeue_age_ms;
    uint32_t last_report_ms;
    bool report_timestamp_valid;
} radar_uplink_stats_t;

static void notify_uplink_task(void)
{
    if (s_uplink_task != NULL) {
        xTaskNotifyGive(s_uplink_task);
    }
}

static void telemetry_queue_release(void)
{
    /* The service setter is only legal before service init, which is the
     * only lifecycle in which this rollback helper is called. */
    (void)smartcar_service_set_telemetry_sink(NULL, NULL);
    s_telemetry_queue_ready = false;
    s_telemetry_mutex = NULL;
    if (s_telemetry_wheel_entries != NULL) {
        heap_caps_free(s_telemetry_wheel_entries);
        s_telemetry_wheel_entries = NULL;
    }
}

static esp_err_t telemetry_queue_prepare(void)
{
    const size_t storage_bytes = RADAR_UPLINK_TELEMETRY_WHEEL_FIFO_DEPTH *
                                 sizeof(*s_telemetry_wheel_entries);
    const radar_telemetry_queue_storage_t storage = {
        .wheel_entries = NULL,
        .wheel_capacity = RADAR_UPLINK_TELEMETRY_WHEEL_FIFO_DEPTH,
        .attitude_entry = &s_telemetry_attitude_entry,
        .imu_entries = s_telemetry_imu_entries,
    };

    s_telemetry_wheel_entries = heap_caps_calloc(
        RADAR_UPLINK_TELEMETRY_WHEEL_FIFO_DEPTH,
        sizeof(*s_telemetry_wheel_entries),
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (s_telemetry_wheel_entries == NULL) {
        ESP_LOGE(TAG, "TELEMETRY FIFO PSRAM ALLOC FAILED entries=%u bytes=%u",
                 (unsigned)RADAR_UPLINK_TELEMETRY_WHEEL_FIFO_DEPTH,
                 (unsigned)storage_bytes);
        return ESP_ERR_NO_MEM;
    }

    radar_telemetry_queue_storage_t mutable_storage = storage;
    mutable_storage.wheel_entries = s_telemetry_wheel_entries;
    if (!radar_telemetry_queue_init(&s_telemetry_queue, &mutable_storage)) {
        telemetry_queue_release();
        return ESP_ERR_INVALID_STATE;
    }
    s_telemetry_mutex = xSemaphoreCreateMutexStatic(&s_telemetry_mutex_storage);
    if (s_telemetry_mutex == NULL) {
        telemetry_queue_release();
        return ESP_ERR_NO_MEM;
    }
    s_telemetry_queue_ready = true;
    ESP_LOGI(TAG, "TELEMETRY FIFO READY entries=%u bytes=%u",
             (unsigned)RADAR_UPLINK_TELEMETRY_WHEEL_FIFO_DEPTH,
             (unsigned)storage_bytes);
    return ESP_OK;
}

static bool radar_uplink_telemetry_sink(uint16_t message_id,
                                        const uint8_t *encoded_frame,
                                        uint16_t encoded_length,
                                        uint32_t ingress_timestamp_ms,
                                        void *context)
{
    bool queued = false;

    if (context != &s_telemetry_queue || !s_telemetry_queue_ready ||
        s_telemetry_mutex == NULL ||
        xSemaphoreTake(s_telemetry_mutex,
                       RADAR_UPLINK_TELEMETRY_MUTEX_WAIT_TICKS) != pdTRUE) {
        ++s_telemetry_lock_drops;
        return false;
    }
    queued = radar_telemetry_queue_push(&s_telemetry_queue,
                                        message_id,
                                        encoded_frame,
                                        encoded_length,
                                        ingress_timestamp_ms);
    (void)xSemaphoreGive(s_telemetry_mutex);
    if (queued) {
        notify_uplink_task();
    }
    return queued;
}

static bool radar_uplink_pop_telemetry(radar_telemetry_entry_t *entry)
{
    bool popped = false;

    if (entry == NULL || !s_telemetry_queue_ready || s_telemetry_mutex == NULL ||
        xSemaphoreTake(s_telemetry_mutex,
                       RADAR_UPLINK_TELEMETRY_MUTEX_WAIT_TICKS) != pdTRUE) {
        ++s_telemetry_lock_drops;
        return false;
    }
    popped = radar_telemetry_queue_pop(&s_telemetry_queue, entry);
    (void)xSemaphoreGive(s_telemetry_mutex);
    return popped;
}

static void radar_uplink_get_telemetry_stats(
    radar_telemetry_queue_stats_t *stats)
{
    if (stats == NULL) {
        return;
    }
    (void)memset(stats, 0, sizeof(*stats));
    if (!s_telemetry_queue_ready || s_telemetry_mutex == NULL ||
        xSemaphoreTake(s_telemetry_mutex,
                       RADAR_UPLINK_TELEMETRY_MUTEX_WAIT_TICKS) != pdTRUE) {
        ++s_telemetry_lock_drops;
        return;
    }
    radar_telemetry_queue_get_stats(&s_telemetry_queue, stats);
    (void)xSemaphoreGive(s_telemetry_mutex);
}

static const radar_wifi_credential_t *current_wifi_credential(void)
{
    return &radar_wifi_credentials[s_wifi_credential_index];
}

static void advance_wifi_credential(void)
{
    s_wifi_credential_index =
        (s_wifi_credential_index + 1U) % RADAR_WIFI_CREDENTIAL_COUNT;
}

static esp_err_t validate_wifi_credentials(void)
{
    if (RADAR_WIFI_CREDENTIAL_COUNT == 0U) {
        ESP_LOGE(TAG, "WIFI CREDENTIAL LIST EMPTY");
        return ESP_ERR_INVALID_ARG;
    }

    for (size_t index = 0U; index < RADAR_WIFI_CREDENTIAL_COUNT; ++index) {
        const radar_wifi_credential_t *credential = &radar_wifi_credentials[index];
        if (credential->ssid == NULL || credential->password == NULL ||
            credential->ssid[0] == '\0' || credential->password[0] == '\0') {
            ESP_LOGE(TAG, "WIFI CREDENTIAL %u INCOMPLETE", (unsigned)(index + 1U));
            return ESP_ERR_INVALID_ARG;
        }
        if (strlen(credential->ssid) > RADAR_UPLINK_WIFI_SSID_MAX_LEN ||
            strlen(credential->password) > RADAR_UPLINK_WIFI_PASSWORD_MAX_LEN) {
            ESP_LOGE(TAG, "WIFI CREDENTIAL %u FIELD TOO LONG", (unsigned)(index + 1U));
            return ESP_ERR_INVALID_ARG;
        }
    }
    return ESP_OK;
}

static esp_err_t apply_wifi_credential(size_t index)
{
    if (index >= RADAR_WIFI_CREDENTIAL_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }

    const radar_wifi_credential_t *credential = &radar_wifi_credentials[index];
    wifi_config_t config = {0};
    const size_t ssid_length = strlen(credential->ssid);
    const size_t password_length = strlen(credential->password);

    memcpy(config.sta.ssid, credential->ssid, ssid_length);
    memcpy(config.sta.password, credential->password, password_length);
    return esp_wifi_set_config(WIFI_IF_STA, &config);
}

static void close_socket(void)
{
    if (s_socket >= 0) {
        shutdown(s_socket, SHUT_RDWR);
        close(s_socket);
        s_socket = -1;
    }
}

static void wifi_event_handler(void *arg,
                               esp_event_base_t event_base,
                               int32_t event_id,
                               void *event_data)
{
    (void)arg;
    (void)event_data;
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        xEventGroupClearBits(s_wifi_events, RADAR_UPLINK_WIFI_CONNECTED);
        xEventGroupSetBits(s_wifi_events, RADAR_UPLINK_WIFI_ROTATE);
        notify_uplink_task();
    }
}

static void ip_event_handler(void *arg,
                             esp_event_base_t event_base,
                             int32_t event_id,
                             void *event_data)
{
    (void)arg;
    (void)event_data;
    if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        xEventGroupSetBits(s_wifi_events, RADAR_UPLINK_WIFI_CONNECTED);
        notify_uplink_task();
    }
}

static int connect_endpoint(void)
{
    struct addrinfo hints = {0};
    struct addrinfo *result = NULL;
    struct timeval timeout = {
        .tv_sec = RADAR_UPLINK_CONNECT_TIMEOUT_MS / 1000U,
        .tv_usec = (RADAR_UPLINK_CONNECT_TIMEOUT_MS % 1000U) * 1000U,
    };
    int socket_fd;

    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_protocol = IPPROTO_TCP;
    if (getaddrinfo(CONFIG_SMARTCAR_RADAR_UPLINK_HOST,
                    CONFIG_SMARTCAR_RADAR_UPLINK_PORT_STRING,
                    &hints,
                    &result) != 0 || result == NULL) {
        return -1;
    }

    socket_fd = socket(result->ai_family, result->ai_socktype, result->ai_protocol);
    if (socket_fd < 0) {
        freeaddrinfo(result);
        return -1;
    }

    const int original_flags = fcntl(socket_fd, F_GETFL, 0);
    if (original_flags < 0 || fcntl(socket_fd, F_SETFL, original_flags | O_NONBLOCK) < 0) {
        close(socket_fd);
        freeaddrinfo(result);
        return -1;
    }

    int connect_result = connect(socket_fd, result->ai_addr, result->ai_addrlen);
    if (connect_result != 0 && errno != EINPROGRESS) {
        close(socket_fd);
        freeaddrinfo(result);
        return -1;
    }
    if (connect_result != 0) {
        fd_set write_fds;
        FD_ZERO(&write_fds);
        FD_SET(socket_fd, &write_fds);
        int select_result;
        do {
            select_result = select(socket_fd + 1, NULL, &write_fds, NULL, &timeout);
        } while (select_result < 0 && errno == EINTR);
        if (select_result <= 0 || !FD_ISSET(socket_fd, &write_fds)) {
            close(socket_fd);
            freeaddrinfo(result);
            return -1;
        }

        int socket_error = 0;
        socklen_t socket_error_length = sizeof(socket_error);
        if (getsockopt(socket_fd,
                       SOL_SOCKET,
                       SO_ERROR,
                       &socket_error,
                       &socket_error_length) != 0 || socket_error != 0) {
            close(socket_fd);
            freeaddrinfo(result);
            return -1;
        }
    }

    freeaddrinfo(result);
    return socket_fd;
}

static int radar_socket_send(void *context, const uint8_t *data, size_t length)
{
    (void)context;
    return (int)send(s_socket, data, length, 0);
}

static bool radar_uplink_frame_is_zero_packet(const uint8_t *frame, size_t length)
{
    return frame != NULL && length > 2U && (frame[2] & 0x01U) != 0U;
}

static void radar_uplink_reset_after_disconnect(
    bool *pending_packet,
    radar_uplink_tx_state_t *tx_state,
    bool *waiting_for_zero_packet,
    radar_uplink_stats_t *stats)
{
    if (pending_packet == NULL || tx_state == NULL ||
        waiting_for_zero_packet == NULL || stats == NULL) {
        return;
    }
    if (*pending_packet) {
        ++stats->resync_discarded_frames;
    }
    *pending_packet = false;
    radar_uplink_tx_reset(tx_state);
    *waiting_for_zero_packet = true;
}

static void radar_uplink_log_stats(radar_uplink_stats_t *stats)
{
    const uint32_t now_ms = (uint32_t)esp_log_timestamp();
    radar_telemetry_queue_stats_t telemetry_stats;
    const UBaseType_t stack_hwm = uxTaskGetStackHighWaterMark(NULL);

    if (stats == NULL ||
        (stats->report_timestamp_valid &&
         (uint32_t)(now_ms - stats->last_report_ms) < RADAR_UPLINK_STATS_INTERVAL_MS)) {
        return;
    }

    stats->last_report_ms = now_ms;
    stats->report_timestamp_valid = true;
    stats->telemetry_lock_drops = s_telemetry_lock_drops;
    radar_uplink_get_telemetry_stats(&telemetry_stats);
    ESP_LOGI(TAG,
             "RADAR_UPLINK_STATS sent=%" PRIu32 " bytes=%" PRIu64
             " send_fail=%" PRIu32 " send_timeout=%" PRIu32
             " partial=%" PRIu32 " pending_retry=%" PRIu32
             " connect_fail=%" PRIu32 " reconnect=%" PRIu32
             " sync_drop=%" PRIu32 " encode_fail=%" PRIu32
             " stale_drop=%" PRIu32 " seq_gap=%" PRIu32
             " telem_sent=%" PRIu32 " telem_bytes=%" PRIu64
             " telem_stale=%" PRIu32 " telem_encode=%" PRIu32
             " telem_q=%u telem_q_drop=%" PRIu32 " telem_q_reject=%" PRIu32
             " last_seq=%" PRIu32 " telem_seq=%" PRIu32
             " max_age_ms=%" PRIu32 " stack_hwm=%u",
             stats->successful_frames,
             stats->successful_bytes,
             stats->send_failures,
             stats->send_timeouts,
             stats->partial_writes,
             stats->pending_retries,
             stats->connect_failures,
             stats->reconnects,
             stats->resync_discarded_frames,
             stats->encode_failures,
             stats->radar_stale_drops,
             stats->radar_sequence_gaps,
             stats->telemetry_sent_frames,
             stats->telemetry_sent_bytes,
             stats->telemetry_stale_drops,
             stats->telemetry_encode_failures,
             (unsigned)telemetry_stats.depth,
             telemetry_stats.wheel.dropped,
             telemetry_stats.rejected,
             stats->last_sent_sequence,
             stats->last_telemetry_sequence,
             stats->max_dequeue_age_ms,
             (unsigned)stack_hwm);

    radar_uplink_ble_log(SMARTCAR_LOG_LEVEL_INFO,
                         "UPLINK_STATS sent=%" PRIu32 " fail=%" PRIu32
                         " timeout=%" PRIu32 " conn_fail=%" PRIu32
                         " reconn=%" PRIu32 " seq=%" PRIu32
                         " age=%" PRIu32 " sync=%" PRIu32
                         " t_sent=%" PRIu32 " t_stale=%" PRIu32
                         " t_q=%u",
                         stats->successful_frames,
                         stats->send_failures,
                         stats->send_timeouts,
                         stats->connect_failures,
                         stats->reconnects,
                         stats->last_sent_sequence,
                         stats->max_dequeue_age_ms,
                         stats->resync_discarded_frames,
                         stats->telemetry_sent_frames,
                         stats->telemetry_stale_drops,
                         (unsigned)telemetry_stats.depth);
    stats->max_dequeue_age_ms = 0U;
}

typedef enum {
    RADAR_UPLINK_PENDING_NONE = 0,
    RADAR_UPLINK_PENDING_RADAR,
    RADAR_UPLINK_PENDING_TELEMETRY,
} radar_uplink_pending_kind_t;

static bool radar_uplink_prepare_telemetry_packet(
    radar_telemetry_entry_t *entry,
    uint8_t *packet,
    size_t packet_capacity,
    size_t *packet_length,
    uint32_t *uplink_sequence,
    radar_uplink_stats_t *stats)
{
    if (entry == NULL || packet == NULL || packet_length == NULL ||
        uplink_sequence == NULL || stats == NULL) {
        return false;
    }

    for (;;) {
        if (!radar_uplink_pop_telemetry(entry)) {
            return false;
        }

        const uint32_t now_ms = (uint32_t)esp_log_timestamp();
        const uint32_t age_ms = (uint32_t)(now_ms - entry->ingress_timestamp_ms);
        if (age_ms > RADAR_UPLINK_MAX_TELEMETRY_AGE_MS) {
            ++stats->telemetry_stale_drops;
            continue;
        }

        uint32_t sequence = *uplink_sequence + 1U;
        if (sequence == 0U) {
            sequence = 1U;
        }
        const radar_uplink_status_t status = radar_uplink_encode_envelope(
            entry->data,
            entry->length,
            RADAR_UPLINK_MESSAGE_SRP_TELEMETRY_EXPERIMENTAL,
            0U,
            (uint32_t)CONFIG_SMARTCAR_RADAR_UPLINK_DEVICE_ID,
            (uint32_t)CONFIG_SMARTCAR_RADAR_UPLINK_STREAM_ID,
            sequence,
            entry->ingress_timestamp_ms,
            packet,
            packet_capacity,
            packet_length);
        if (status != RADAR_UPLINK_OK) {
            ++stats->telemetry_encode_failures;
            continue;
        }

        *uplink_sequence = sequence;
        return true;
    }
}

static bool radar_uplink_prepare_radar_packet(
    uint8_t *frame,
    size_t frame_capacity,
    size_t *frame_length,
    uint32_t *frame_sequence,
    uint32_t *frame_timestamp_ms,
    uint32_t *dequeue_age_ms,
    uint8_t *packet,
    size_t packet_capacity,
    size_t *packet_length,
    bool *waiting_for_zero_packet,
    bool *radar_sequence_valid,
    uint32_t *last_radar_sequence,
    bool *zero_packet,
    uint32_t *uplink_sequence,
    radar_uplink_stats_t *stats)
{
    if (frame == NULL || frame_length == NULL || frame_sequence == NULL ||
        frame_timestamp_ms == NULL || dequeue_age_ms == NULL || packet == NULL ||
        packet_length == NULL || waiting_for_zero_packet == NULL ||
        radar_sequence_valid == NULL || last_radar_sequence == NULL ||
        zero_packet == NULL || uplink_sequence == NULL || stats == NULL) {
        return false;
    }

    while (radar_uart_pop_frame(frame,
                                frame_capacity,
                                frame_length,
                                frame_sequence,
                                frame_timestamp_ms,
                                dequeue_age_ms)) {
        if (*frame_sequence == 0U) {
            ++stats->encode_failures;
            *waiting_for_zero_packet = true;
            *radar_sequence_valid = false;
            continue;
        }
        if (*dequeue_age_ms > RADAR_UPLINK_MAX_RADAR_DEQUEUE_AGE_MS) {
            ++stats->radar_stale_drops;
            *waiting_for_zero_packet = true;
            *radar_sequence_valid = false;
            continue;
        }

        const bool is_zero = radar_uplink_frame_is_zero_packet(frame,
                                                                *frame_length);
        if (!*waiting_for_zero_packet && *radar_sequence_valid &&
            *frame_sequence != *last_radar_sequence + 1U) {
            ++stats->radar_sequence_gaps;
            *waiting_for_zero_packet = true;
            *radar_sequence_valid = false;
            if (!is_zero) {
                ++stats->resync_discarded_frames;
                continue;
            }
        }
        if (*waiting_for_zero_packet && !is_zero) {
            ++stats->resync_discarded_frames;
            continue;
        }

        uint32_t packet_sequence = *uplink_sequence + 1U;
        if (packet_sequence == 0U) {
            packet_sequence = 1U;
        }
        if (radar_uplink_encode_frame(
                frame,
                *frame_length,
                (uint32_t)CONFIG_SMARTCAR_RADAR_UPLINK_DEVICE_ID,
                (uint32_t)CONFIG_SMARTCAR_RADAR_UPLINK_STREAM_ID,
                packet_sequence,
                *frame_timestamp_ms,
                packet,
                packet_capacity,
                packet_length) != RADAR_UPLINK_OK) {
            ++stats->encode_failures;
            *waiting_for_zero_packet = true;
            *radar_sequence_valid = false;
            continue;
        }

        *uplink_sequence = packet_sequence;
        *zero_packet = is_zero;
        return true;
    }
    return false;
}

static void radar_uplink_task(void *context)
{
    (void)context;
    uint8_t frame[RADAR_PARSER_MAX_FRAME_SIZE];
    uint8_t packet[RADAR_UPLINK_MAX_PACKET_SIZE];
    size_t frame_length = 0U;
    size_t packet_length = 0U;
    uint32_t radar_frame_sequence = 0U;
    uint32_t uplink_sequence = 0U;
    uint32_t timestamp_ms = 0U;
    uint32_t pending_sequence = 0U;
    uint32_t pending_radar_frame_sequence = 0U;
    uint32_t retry_ms = RADAR_UPLINK_RETRY_INITIAL_MS;
    uint32_t wifi_retry_ms = RADAR_UPLINK_RETRY_INITIAL_MS;
    bool wifi_connected_logged = false;
    bool pending_packet = false;
    bool pending_zero_packet = false;
    bool waiting_for_zero_packet = true;
    bool radar_sequence_valid = false;
    uint32_t last_radar_sequence = 0U;
    bool telemetry_turn = false;
    bool credential_rotated = false;
    radar_uplink_pending_kind_t pending_kind = RADAR_UPLINK_PENDING_NONE;
    radar_telemetry_entry_t telemetry_entry;
    radar_uplink_tx_state_t tx_state = {0};
    radar_uplink_stats_t stats = {0};

    for (;;) {
        (void)ulTaskNotifyTake(pdFALSE, pdMS_TO_TICKS(RADAR_UPLINK_WAIT_MS));
        credential_rotated = false;
        EventBits_t bits = xEventGroupGetBits(s_wifi_events);
        if ((xEventGroupClearBits(s_wifi_events, RADAR_UPLINK_WIFI_ROTATE) &
             RADAR_UPLINK_WIFI_ROTATE) != 0U) {
            advance_wifi_credential();
            credential_rotated = true;
            ESP_LOGW(TAG,
                     "WIFI DISCONNECTED; NEXT SSID=%s",
                     current_wifi_credential()->ssid);
            radar_uplink_ble_log(SMARTCAR_LOG_LEVEL_WARN,
                                 "WIFI DISCONNECTED; NEXT SSID=%s",
                                 current_wifi_credential()->ssid);
        }
        if ((bits & RADAR_UPLINK_WIFI_CONNECTED) == 0U) {
            wifi_connected_logged = false;
            close_socket();
            radar_uplink_reset_after_disconnect(&pending_packet,
                                                &tx_state,
                                                &waiting_for_zero_packet,
                                                &stats);
            pending_zero_packet = false;
            pending_kind = RADAR_UPLINK_PENDING_NONE;
            radar_sequence_valid = false;
            telemetry_turn = false;
            if (s_wifi_started) {
                if (credential_rotated) {
                    esp_err_t config_ret = apply_wifi_credential(s_wifi_credential_index);
                    if (config_ret != ESP_OK) {
                        ESP_LOGW(TAG,
                                 "WIFI CONFIG FAILED SSID=%s err=%s",
                                 current_wifi_credential()->ssid,
                                 esp_err_to_name(config_ret));
                        radar_uplink_ble_log(SMARTCAR_LOG_LEVEL_WARN,
                                             "WIFI CONFIG FAILED SSID=%s err=%s",
                                             current_wifi_credential()->ssid,
                                             esp_err_to_name(config_ret));
                        advance_wifi_credential();
                        vTaskDelay(pdMS_TO_TICKS(wifi_retry_ms));
                        continue;
                    }
                }
                (void)esp_wifi_connect();
                vTaskDelay(pdMS_TO_TICKS(wifi_retry_ms));
                wifi_retry_ms = wifi_retry_ms < RADAR_UPLINK_RETRY_MAX_MS / 2U
                                    ? wifi_retry_ms * 2U
                                    : RADAR_UPLINK_RETRY_MAX_MS;
            }
            continue;
        }
        if (!wifi_connected_logged) {
            radar_uplink_ble_log(SMARTCAR_LOG_LEVEL_INFO, "WIFI CONNECTED");
            wifi_connected_logged = true;
        }
        wifi_retry_ms = RADAR_UPLINK_RETRY_INITIAL_MS;

        if (s_socket < 0) {
            s_socket = connect_endpoint();
            if (s_socket < 0) {
                ++stats.connect_failures;
                radar_uplink_log_stats(&stats);
                ESP_LOGW(TAG, "TCP CONNECT FAILED");
                radar_uplink_ble_log(SMARTCAR_LOG_LEVEL_WARN,
                                     "TCP CONNECT FAILED");
                vTaskDelay(pdMS_TO_TICKS(retry_ms));
                retry_ms = retry_ms < RADAR_UPLINK_RETRY_MAX_MS / 2U
                               ? retry_ms * 2U
                               : RADAR_UPLINK_RETRY_MAX_MS;
                continue;
            }
            retry_ms = RADAR_UPLINK_RETRY_INITIAL_MS;
            ++stats.reconnects;
            waiting_for_zero_packet = true;
            radar_sequence_valid = false;
            telemetry_turn = false;
            ESP_LOGI(TAG, "TCP CONNECTED");
            radar_uplink_ble_log(SMARTCAR_LOG_LEVEL_INFO, "TCP CONNECTED");
        }

        bool send_waited = false;
        for (uint32_t sent_in_burst = 0U;
             sent_in_burst < RADAR_UPLINK_BURST_MAX_FRAMES;
             ++sent_in_burst) {
            uint32_t dequeue_age_ms = 0U;
            if (!pending_packet) {
                bool packet_ready = false;
                /* A reconnect must establish the radar ring boundary first;
                 * once synchronized, alternate classes to avoid starvation. */
                const bool prefer_telemetry = telemetry_turn &&
                                               !waiting_for_zero_packet;
                if (prefer_telemetry) {
                    packet_ready = radar_uplink_prepare_telemetry_packet(
                        &telemetry_entry,
                        packet,
                        sizeof(packet),
                        &packet_length,
                        &uplink_sequence,
                        &stats);
                    if (packet_ready) {
                        pending_kind = RADAR_UPLINK_PENDING_TELEMETRY;
                        pending_sequence = uplink_sequence;
                    }
                }
                if (!packet_ready) {
                    packet_ready = radar_uplink_prepare_radar_packet(
                        frame,
                        sizeof(frame),
                        &frame_length,
                        &radar_frame_sequence,
                        &timestamp_ms,
                        &dequeue_age_ms,
                        packet,
                        sizeof(packet),
                        &packet_length,
                        &waiting_for_zero_packet,
                        &radar_sequence_valid,
                        &last_radar_sequence,
                        &pending_zero_packet,
                        &uplink_sequence,
                        &stats);
                    if (packet_ready) {
                        pending_kind = RADAR_UPLINK_PENDING_RADAR;
                        pending_sequence = uplink_sequence;
                        pending_radar_frame_sequence = radar_frame_sequence;
                        if (dequeue_age_ms > stats.max_dequeue_age_ms) {
                            stats.max_dequeue_age_ms = dequeue_age_ms;
                        }
                    }
                }
                if (!packet_ready && !prefer_telemetry) {
                    packet_ready = radar_uplink_prepare_telemetry_packet(
                        &telemetry_entry,
                        packet,
                        sizeof(packet),
                        &packet_length,
                        &uplink_sequence,
                        &stats);
                    if (packet_ready) {
                        pending_kind = RADAR_UPLINK_PENDING_TELEMETRY;
                        pending_sequence = uplink_sequence;
                    }
                }
                if (!packet_ready) {
                    break;
                }
                radar_uplink_tx_reset(&tx_state);
                pending_packet = true;
            }

            const radar_uplink_tx_result_t send_result =
                radar_uplink_tx_send(&tx_state,
                                     packet,
                                     packet_length,
                                     radar_socket_send,
                                     NULL);
            if (tx_state.wrote_partial) {
                ++stats.partial_writes;
            }
            if (send_result == RADAR_UPLINK_TX_WAIT) {
                ++stats.send_timeouts;
                ++stats.pending_retries;
                send_waited = true;
                break;
            }
            if (send_result == RADAR_UPLINK_TX_FAILED) {
                const int send_errno = errno;
                ++stats.send_failures;
                ESP_LOGW(TAG, "TCP SEND FAILED errno=%d", send_errno);
                radar_uplink_ble_log(SMARTCAR_LOG_LEVEL_WARN,
                                     "TCP SEND FAILED errno=%d", send_errno);
                close_socket();
                radar_uplink_reset_after_disconnect(&pending_packet,
                                                    &tx_state,
                                                    &waiting_for_zero_packet,
                                                    &stats);
                pending_zero_packet = false;
                pending_kind = RADAR_UPLINK_PENDING_NONE;
                radar_sequence_valid = false;
                telemetry_turn = false;
                break;
            }

            pending_packet = false;
            radar_uplink_tx_reset(&tx_state);
            ++stats.successful_frames;
            stats.successful_bytes += packet_length;
            if (pending_kind == RADAR_UPLINK_PENDING_RADAR) {
                stats.last_sent_sequence = pending_sequence;
                last_radar_sequence = pending_radar_frame_sequence;
                radar_sequence_valid = true;
                if (pending_zero_packet) {
                    waiting_for_zero_packet = false;
                }
                telemetry_turn = true;
            } else if (pending_kind == RADAR_UPLINK_PENDING_TELEMETRY) {
                ++stats.telemetry_sent_frames;
                stats.telemetry_sent_bytes += packet_length;
                stats.last_telemetry_sequence = pending_sequence;
                telemetry_turn = false;
            }
            pending_zero_packet = false;
            pending_kind = RADAR_UPLINK_PENDING_NONE;
        }
        radar_uplink_log_stats(&stats);
        if (send_waited) {
            vTaskDelay(RADAR_UPLINK_RETRY_YIELD_TICKS);
        }
    }
}
#endif

esp_err_t radar_uplink_init(void)
{
#if !CONFIG_SMARTCAR_RADAR_UPLINK_ENABLED
    ESP_LOGI(TAG, "DISABLED");
    return ESP_OK;
#else
    if (s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (validate_wifi_credentials() != ESP_OK ||
        CONFIG_SMARTCAR_RADAR_UPLINK_HOST[0] == '\0' ||
        CONFIG_SMARTCAR_RADAR_UPLINK_PORT_STRING[0] == '\0') {
        ESP_LOGE(TAG, "CONFIG INCOMPLETE; DISABLED");
        radar_uplink_ble_log(SMARTCAR_LOG_LEVEL_ERROR, "CONFIG INCOMPLETE; DISABLED");
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = esp_netif_init();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        radar_uplink_ble_log(SMARTCAR_LOG_LEVEL_ERROR,
                             "NETIF INIT FAILED err=%s", esp_err_to_name(ret));
        return ret;
    }
    ret = esp_event_loop_create_default();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        radar_uplink_ble_log(SMARTCAR_LOG_LEVEL_ERROR,
                             "EVENT LOOP INIT FAILED err=%s", esp_err_to_name(ret));
        return ret;
    }
    esp_netif_t *netif = esp_netif_create_default_wifi_sta();
    if (netif == NULL) {
        radar_uplink_ble_log(SMARTCAR_LOG_LEVEL_ERROR, "WIFI NETIF CREATE FAILED");
        return ESP_ERR_NO_MEM;
    }
    const wifi_init_config_t wifi_init = WIFI_INIT_CONFIG_DEFAULT();
    ret = esp_wifi_init(&wifi_init);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        radar_uplink_ble_log(SMARTCAR_LOG_LEVEL_ERROR,
                             "WIFI INIT FAILED err=%s", esp_err_to_name(ret));
        return ret;
    }

    s_wifi_events = xEventGroupCreate();
    if (s_wifi_events == NULL) {
        radar_uplink_ble_log(SMARTCAR_LOG_LEVEL_ERROR, "WIFI EVENT GROUP FAILED");
        return ESP_ERR_NO_MEM;
    }
    ret = esp_event_handler_register(WIFI_EVENT,
                                     ESP_EVENT_ANY_ID,
                                     &wifi_event_handler,
                                     NULL);
    if (ret == ESP_OK) {
        ret = esp_event_handler_register(IP_EVENT,
                                         IP_EVENT_STA_GOT_IP,
                                         &ip_event_handler,
                                         NULL);
    }
    if (ret == ESP_OK) {
        ret = esp_wifi_set_mode(WIFI_MODE_STA);
    }
    if (ret == ESP_OK) {
        ret = apply_wifi_credential(s_wifi_credential_index);
    }
    if (ret == ESP_OK) {
        ret = esp_wifi_start();
    }
    if (ret != ESP_OK) {
        radar_uplink_ble_log(SMARTCAR_LOG_LEVEL_ERROR,
                             "WIFI SETUP FAILED err=%s", esp_err_to_name(ret));
        return ret;
    }
    s_wifi_started = true;
    ret = esp_wifi_connect();
    if (ret != ESP_OK) {
        /* The worker keeps the retry/backoff path alive for transient Wi-Fi
         * state errors instead of failing the rest of S3 startup. */
        ESP_LOGW(TAG, "WIFI CONNECT START DEFERRED: %s", esp_err_to_name(ret));
        radar_uplink_ble_log(SMARTCAR_LOG_LEVEL_WARN,
                             "WIFI CONNECT START DEFERRED: %s", esp_err_to_name(ret));
    }

    s_telemetry_lock_drops = 0U;
    ret = telemetry_queue_prepare();
    if (ret != ESP_OK) {
        radar_uplink_ble_log(SMARTCAR_LOG_LEVEL_ERROR,
                             "TELEMETRY FIFO INIT FAILED err=%s",
                             esp_err_to_name(ret));
        return ret;
    }
    ret = smartcar_service_set_telemetry_sink(radar_uplink_telemetry_sink,
                                               &s_telemetry_queue);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "TELEMETRY SINK REGISTER FAILED: %s", esp_err_to_name(ret));
        telemetry_queue_release();
        return ret;
    }

    if (xTaskCreate(radar_uplink_task,
                   "radar_uplink",
                   RADAR_UPLINK_TASK_STACK_SIZE,
                   NULL,
                   RADAR_UPLINK_TASK_PRIORITY,
                   &s_uplink_task) != pdPASS) {
        radar_uplink_ble_log(SMARTCAR_LOG_LEVEL_ERROR, "UPLINK TASK CREATE FAILED");
        telemetry_queue_release();
        return ESP_ERR_NO_MEM;
    }
    radar_uart_set_frame_notification_task(s_uplink_task);
    s_initialized = true;
    ESP_LOGI(TAG, "READY TCP uplink");
    radar_uplink_ble_log(SMARTCAR_LOG_LEVEL_INFO, "READY TCP uplink");
    return ESP_OK;
#endif
}

bool radar_uplink_is_running(void)
{
#if !CONFIG_SMARTCAR_RADAR_UPLINK_ENABLED
    return false;
#else
    return s_initialized && s_wifi_started && s_uplink_task != NULL;
#endif
}
