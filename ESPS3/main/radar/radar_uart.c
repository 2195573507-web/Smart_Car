#include "radar_uart.h"

#include <stdio.h>
#include <string.h>

#include "driver/ledc.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "radar_frame_fifo.h"
#include "s3_ble.h"

static const char *TAG = "RADAR";

#define RADAR_UART_RAW_LOG_ENABLED 0U
#define RADAR_UART_HEX_LOG_BYTES 32U
#define RADAR_BLE_RAW_UART_LOG_ENABLED 0U
#define RADAR_BLE_HEX_LOG_BYTES 20U
#define RADAR_BLE_LOG_BUFFER_SIZE 96U
#define RADAR_BLE_LOG_PERIOD_MS 200U
#define RADAR_UART_HEX_LOG_PERIOD_MS 200U
#define RADAR_PARSER_STATS_LOG_PERIOD_MS 1000U
#define RADAR_FRAME_FIFO_MUTEX_WAIT_TICKS 1U

static TaskHandle_t s_uart_task;
static QueueHandle_t s_uart_event_queue;
static bool s_uart_ready;
static bool s_pwm_ready;
static uint8_t s_read_buffer[RADAR_UART_READ_BUFFER_SIZE];
#if RADAR_UART_RAW_LOG_ENABLED
static char s_hex_buffer[RADAR_UART_HEX_LOG_BYTES * 3U];
static uint32_t s_last_ble_log_ms;
static bool s_ble_log_timestamp_valid;
static uint32_t s_last_hex_log_ms;
static bool s_hex_log_timestamp_valid;
#endif
static char s_ble_log_buffer[RADAR_BLE_LOG_BUFFER_SIZE];
static uint32_t s_last_stats_log_ms;
static bool s_stats_log_timestamp_valid;
static radar_parser_t s_parser;
static radar_frame_fifo_t s_frame_fifo;
static radar_frame_fifo_entry_t *s_frame_fifo_entries;
static TaskHandle_t s_uplink_notification_task;
static uint32_t s_frame_sequence;
static size_t s_last_frame_length;
static uint32_t s_last_frame_sequence;
static uint32_t s_last_frame_timestamp_ms;
static uint32_t s_frame_lock_drop_count;
static uint32_t s_uart_overflow_count;
static SemaphoreHandle_t s_frame_fifo_mutex;

static bool radar_uart_handle_event(const uart_event_t *event)
{
    if (event == NULL) {
        return false;
    }

    if (event->type != UART_FIFO_OVF && event->type != UART_BUFFER_FULL) {
        return false;
    }

    ++s_uart_overflow_count;
    (void)uart_flush_input(RADAR_UART_PORT);
    (void)xQueueReset(s_uart_event_queue);
    radar_parser_reset_stream(&s_parser);
    ESP_LOGW(TAG, "RADAR_UART_OVERFLOW type=%d count=%lu",
             (int)event->type, (unsigned long)s_uart_overflow_count);
    return true;
}

static void radar_uart_frame_callback(const uint8_t *data,
                                      size_t length,
                                      void *context)
{
    (void)context;
    if (data == NULL || length == 0U || length > RADAR_PARSER_MAX_FRAME_SIZE) {
        return;
    }

    const uint32_t timestamp_ms = (uint32_t)esp_log_timestamp();
    if (s_frame_fifo_mutex == NULL ||
        xSemaphoreTake(s_frame_fifo_mutex, RADAR_FRAME_FIFO_MUTEX_WAIT_TICKS) != pdTRUE) {
        ++s_frame_lock_drop_count;
        return;
    }
    ++s_frame_sequence;
    const bool queued = radar_frame_fifo_push(&s_frame_fifo,
                                              data,
                                              length,
                                              s_frame_sequence,
                                              timestamp_ms);
    if (queued) {
        s_last_frame_length = length;
        s_last_frame_sequence = s_frame_sequence;
        s_last_frame_timestamp_ms = timestamp_ms;
    }
    (void)xSemaphoreGive(s_frame_fifo_mutex);
    if (queued && s_uplink_notification_task != NULL) {
        xTaskNotifyGive(s_uplink_notification_task);
    }
}

static void radar_uart_log_parser_stats(void)
{
    const uint32_t now_ms = (uint32_t)esp_log_timestamp();
    if (s_stats_log_timestamp_valid &&
        (uint32_t)(now_ms - s_last_stats_log_ms) < RADAR_PARSER_STATS_LOG_PERIOD_MS) {
        return;
    }
    s_last_stats_log_ms = now_ms;
    s_stats_log_timestamp_valid = true;

    radar_parser_stats_t stats;
    radar_parser_get_stats(&s_parser, &stats);

    radar_frame_fifo_stats_t fifo_stats = {0};
    size_t latest_length = 0U;
    uint32_t latest_sequence = 0U;
    uint32_t latest_timestamp_ms = 0U;
    if (s_frame_fifo_mutex != NULL &&
        xSemaphoreTake(s_frame_fifo_mutex, 0U) == pdTRUE) {
        radar_frame_fifo_get_stats(&s_frame_fifo, &fifo_stats);
        latest_length = s_last_frame_length;
        latest_sequence = s_last_frame_sequence;
        latest_timestamp_ms = s_last_frame_timestamp_ms;
        (void)xSemaphoreGive(s_frame_fifo_mutex);
    }

    const uint32_t age_now_ms = (uint32_t)esp_log_timestamp();
    const uint32_t age_ms = latest_length == 0U ? 0U :
                            (uint32_t)(age_now_ms - latest_timestamp_ms);
    static uint32_t last_valid_frame_count;
    const uint32_t valid_delta = stats.valid_frame_count - last_valid_frame_count;
    last_valid_frame_count = stats.valid_frame_count;
    size_t uart_buffered_bytes = 0U;
    (void)uart_get_buffered_data_len(RADAR_UART_PORT, &uart_buffered_bytes);
    const UBaseType_t stack_hwm_words =
        s_uart_task == NULL ? 0U : uxTaskGetStackHighWaterMark(s_uart_task);
    const size_t internal_free = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    const size_t psram_free = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);

    ESP_LOGI(TAG,
             "RADAR_PARSER_STATS valid=%lu checksum_errors=%lu invalid=%lu "
             "distance=%lu intensity=%lu last_sample_bytes=%u resync=%lu "
             "parser_overflow=%lu uart_overflow=%lu fifo_depth=%u fifo_capacity=%u "
             "fifo_hwm=%u fifo_drop_oldest=%lu lock_drop=%lu "
             "packets_s=%lu latest_seq=%lu latest_len=%u age_ms=%lu "
             "uart_buffered=%u stack_hwm_words=%u internal_free=%u psram_free=%u",
             (unsigned long)stats.valid_frame_count,
             (unsigned long)stats.checksum_error_count,
             (unsigned long)stats.invalid_frame_count,
             (unsigned long)stats.valid_distance_frame_count,
             (unsigned long)stats.valid_intensity_frame_count,
             (unsigned int)stats.last_sample_bytes,
             (unsigned long)stats.header_resync_count,
             (unsigned long)stats.overflow_count,
             (unsigned long)s_uart_overflow_count,
             (unsigned int)fifo_stats.count,
             (unsigned int)fifo_stats.capacity,
             (unsigned int)fifo_stats.high_watermark,
             (unsigned long)fifo_stats.dropped_oldest_count,
             (unsigned long)s_frame_lock_drop_count,
             (unsigned long)valid_delta,
             (unsigned long)latest_sequence,
             (unsigned int)latest_length,
             (unsigned long)age_ms,
             (unsigned int)uart_buffered_bytes,
             (unsigned int)stack_hwm_words,
             (unsigned int)internal_free,
             (unsigned int)psram_free);

    if (s3_ble_is_log_ready()) {
        int written = snprintf(s_ble_log_buffer,
                               sizeof(s_ble_log_buffer),
                               "RADAR_STATS valid_delta=%lu cs=%lu invalid=%lu mode=%u seq=%lu q=%u drop=%lu",
                               (unsigned long)valid_delta,
                               (unsigned long)stats.checksum_error_count,
                               (unsigned long)stats.invalid_frame_count,
                               (unsigned int)stats.last_sample_bytes,
                               (unsigned long)latest_sequence,
                               (unsigned int)fifo_stats.count,
                               (unsigned long)fifo_stats.dropped_oldest_count);
        if (written > 0 && (size_t)written < sizeof(s_ble_log_buffer)) {
            (void)s3_log_info(s_ble_log_buffer);
        }
    }
}

#if RADAR_UART_RAW_LOG_ENABLED
static void radar_uart_log_hex(const uint8_t *data, size_t length)
{
    const uint32_t now_ms = (uint32_t)esp_log_timestamp();
    if (s_hex_log_timestamp_valid &&
        (uint32_t)(now_ms - s_last_hex_log_ms) < RADAR_UART_HEX_LOG_PERIOD_MS) {
        return;
    }
    s_last_hex_log_ms = now_ms;
    s_hex_log_timestamp_valid = true;

    static const char hex_digits[] = "0123456789ABCDEF";
    size_t bytes_to_log = length;
    if (bytes_to_log > RADAR_UART_HEX_LOG_BYTES) {
        bytes_to_log = RADAR_UART_HEX_LOG_BYTES;
    }

    for (size_t index = 0U; index < bytes_to_log; ++index) {
        const uint8_t value = data[index];
        s_hex_buffer[index * 3U] = hex_digits[value >> 4U];
        s_hex_buffer[index * 3U + 1U] = hex_digits[value & 0x0FU];
        s_hex_buffer[index * 3U + 2U] = ' ';
    }
    if (bytes_to_log > 0U) {
        s_hex_buffer[bytes_to_log * 3U - 1U] = '\0';
    } else {
        s_hex_buffer[0] = '\0';
    }

    ESP_LOGI(TAG, "RADAR_HEX:");
    ESP_LOGI(TAG, "%s", s_hex_buffer);

#if RADAR_BLE_RAW_UART_LOG_ENABLED
    const uint32_t ble_now_ms = (uint32_t)esp_log_timestamp();
    if (s3_ble_is_log_ready()) {
        if (s_ble_log_timestamp_valid &&
            (uint32_t)(ble_now_ms - s_last_ble_log_ms) < RADAR_BLE_LOG_PERIOD_MS) {
            return;
        }
        s_last_ble_log_ms = ble_now_ms;
        s_ble_log_timestamp_valid = true;

        bytes_to_log = length;
        if (bytes_to_log > RADAR_BLE_HEX_LOG_BYTES) {
            bytes_to_log = RADAR_BLE_HEX_LOG_BYTES;
        }

        int written = snprintf(s_ble_log_buffer,
                               sizeof(s_ble_log_buffer),
                               "RADAR_UART_RX len=%u show=%u HEX=",
                               (unsigned int)length,
                               (unsigned int)bytes_to_log);
        if (written < 0 || (size_t)written >= sizeof(s_ble_log_buffer)) {
            return;
        }

        size_t offset = (size_t)written;
        for (size_t index = 0U; index < bytes_to_log; ++index) {
            if (offset + 2U >= sizeof(s_ble_log_buffer)) {
                break;
            }
            const uint8_t value = data[index];
            s_ble_log_buffer[offset++] = hex_digits[value >> 4U];
            s_ble_log_buffer[offset++] = hex_digits[value & 0x0FU];
            if (index + 1U < bytes_to_log && offset + 1U < sizeof(s_ble_log_buffer)) {
                s_ble_log_buffer[offset++] = ' ';
            }
        }
        s_ble_log_buffer[offset] = '\0';
        (void)s3_log_info(s_ble_log_buffer);
    }
#endif
}
#endif

static void radar_uart_drain_rx_buffer(void)
{
    for (;;) {
        size_t buffered = 0U;
        if (uart_get_buffered_data_len(RADAR_UART_PORT, &buffered) != ESP_OK ||
            buffered == 0U) {
            return;
        }

        size_t requested = buffered;
        if (requested > sizeof(s_read_buffer)) {
            requested = sizeof(s_read_buffer);
        }
        const int received = uart_read_bytes(RADAR_UART_PORT,
                                             s_read_buffer,
                                             requested,
                                             0U);
        if (received <= 0) {
            return;
        }

#if RADAR_UART_RAW_LOG_ENABLED
        radar_uart_log_hex(s_read_buffer, (size_t)received);
#endif
        radar_parser_feed(&s_parser,
                          s_read_buffer,
                          (size_t)received,
                          radar_uart_frame_callback,
                          NULL);
    }
}

static void radar_uart_task(void *context)
{
    (void)context;

    for (;;) {
        uart_event_t event;
        const BaseType_t event_received =
            xQueueReceive(s_uart_event_queue,
                          &event,
                          pdMS_TO_TICKS(RADAR_UART_EVENT_WAIT_MS));
        const bool stream_reset = event_received == pdTRUE &&
                                  radar_uart_handle_event(&event);
        if (!stream_reset) {
            radar_uart_drain_rx_buffer();
        }
        radar_uart_log_parser_stats();
    }
}

esp_err_t radar_uart_init(void)
{
    if (s_uart_ready) {
        return ESP_ERR_INVALID_STATE;
    }

    radar_parser_init(&s_parser);
    s_uplink_notification_task = NULL;
    const size_t fifo_storage_bytes = RADAR_FRAME_FIFO_DEPTH *
                                      sizeof(*s_frame_fifo_entries);
    s_frame_fifo_entries = heap_caps_calloc(RADAR_FRAME_FIFO_DEPTH,
                                             sizeof(*s_frame_fifo_entries),
                                             MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (s_frame_fifo_entries == NULL ||
        !radar_frame_fifo_init(&s_frame_fifo,
                               s_frame_fifo_entries,
                               RADAR_FRAME_FIFO_DEPTH)) {
        ESP_LOGE(TAG,
                 "RADAR FIFO PSRAM ALLOC FAILED entries=%u bytes=%u free=%u",
                 (unsigned int)RADAR_FRAME_FIFO_DEPTH,
                 (unsigned int)fifo_storage_bytes,
                 (unsigned int)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
        if (s_frame_fifo_entries != NULL) {
            heap_caps_free(s_frame_fifo_entries);
            s_frame_fifo_entries = NULL;
        }
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG,
             "RADAR FIFO READY storage=PSRAM entries=%u bytes=%u free=%u",
             (unsigned int)RADAR_FRAME_FIFO_DEPTH,
             (unsigned int)fifo_storage_bytes,
             (unsigned int)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    s_frame_sequence = 0U;
    s_last_frame_length = 0U;
    s_last_frame_sequence = 0U;
    s_last_frame_timestamp_ms = 0U;
    s_frame_lock_drop_count = 0U;
    s_uart_overflow_count = 0U;
    s_frame_fifo_mutex = xSemaphoreCreateMutex();
    if (s_frame_fifo_mutex == NULL) {
        heap_caps_free(s_frame_fifo_entries);
        s_frame_fifo_entries = NULL;
        return ESP_ERR_NO_MEM;
    }

    const uart_config_t config = {
        .baud_rate = RADAR_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t ret = uart_param_config(RADAR_UART_PORT, &config);
    if (ret == ESP_OK) {
        ret = uart_set_pin(RADAR_UART_PORT,
                           UART_PIN_NO_CHANGE,
                           RADAR_UART_RX_GPIO,
                           UART_PIN_NO_CHANGE,
                           UART_PIN_NO_CHANGE);
    }
    bool uart_driver_installed = false;
    if (ret == ESP_OK) {
        ret = uart_driver_install(RADAR_UART_PORT,
                                  RADAR_UART_DRIVER_BUFFER_SIZE,
                                  0,
                                  RADAR_UART_EVENT_QUEUE_SIZE,
                                  &s_uart_event_queue,
                                  0);
        uart_driver_installed = ret == ESP_OK;
    }
    if (ret == ESP_OK) {
        ret = uart_set_rx_full_threshold(RADAR_UART_PORT,
                                         RADAR_UART_RX_FULL_THRESHOLD);
    }
    if (ret == ESP_OK) {
        ret = uart_set_rx_timeout(RADAR_UART_PORT,
                                  RADAR_UART_RX_TIMEOUT_SYMBOLS);
    }
    if (ret != ESP_OK) {
        if (uart_driver_installed) {
            (void)uart_driver_delete(RADAR_UART_PORT);
        }
        vSemaphoreDelete(s_frame_fifo_mutex);
        s_frame_fifo_mutex = NULL;
        heap_caps_free(s_frame_fifo_entries);
        s_frame_fifo_entries = NULL;
        s_uart_event_queue = NULL;
        return ret;
    }

    BaseType_t created = xTaskCreate(radar_uart_task,
                                     "radar_uart",
                                     RADAR_UART_TASK_STACK_SIZE,
                                     NULL,
                                     RADAR_UART_TASK_PRIORITY,
                                     &s_uart_task);
    if (created != pdPASS) {
        (void)uart_driver_delete(RADAR_UART_PORT);
        vSemaphoreDelete(s_frame_fifo_mutex);
        s_frame_fifo_mutex = NULL;
        heap_caps_free(s_frame_fifo_entries);
        s_frame_fifo_entries = NULL;
        s_uart_event_queue = NULL;
        s_uart_task = NULL;
        return ESP_ERR_NO_MEM;
    }

    s_uart_ready = true;
    ESP_LOGI(TAG,
             "UART%u ready TX=DISABLED RX=GPIO%u baud=%u rx_buffer=%u",
             (unsigned int)RADAR_UART_PORT,
             (unsigned int)RADAR_UART_RX_GPIO,
             (unsigned int)RADAR_UART_BAUD_RATE,
             (unsigned int)RADAR_UART_DRIVER_BUFFER_SIZE);
    return ESP_OK;
}

bool radar_uart_is_running(void)
{
    return s_uart_ready && s_uart_task != NULL;
}

void radar_uart_set_frame_notification_task(TaskHandle_t task)
{
    s_uplink_notification_task = task;
    if (task != NULL) {
        xTaskNotifyGive(task);
    }
}

bool radar_uart_pop_frame(uint8_t *buffer,
                          size_t capacity,
                          size_t *length,
                          uint32_t *sequence,
                          uint32_t *timestamp_ms,
                          uint32_t *age_ms)
{
    if (buffer == NULL || length == NULL || capacity == 0U) {
        return false;
    }

    size_t frame_length = 0U;
    uint32_t frame_sequence = 0U;
    uint32_t frame_timestamp_ms = 0U;

    if (s_frame_fifo_mutex == NULL ||
        xSemaphoreTake(s_frame_fifo_mutex, RADAR_FRAME_FIFO_MUTEX_WAIT_TICKS) != pdTRUE) {
        return false;
    }
    const bool popped = radar_frame_fifo_pop(&s_frame_fifo,
                                             buffer,
                                             capacity,
                                             &frame_length,
                                             &frame_sequence,
                                             &frame_timestamp_ms);
    (void)xSemaphoreGive(s_frame_fifo_mutex);
    if (!popped) {
        *length = frame_length;
        return false;
    }

    *length = frame_length;
    const uint32_t now_ms = (uint32_t)esp_log_timestamp();
    if (sequence != NULL) {
        *sequence = frame_sequence;
    }
    if (timestamp_ms != NULL) {
        *timestamp_ms = frame_timestamp_ms;
    }
    if (age_ms != NULL) {
        *age_ms = frame_length == 0U ? 0U :
                  (uint32_t)(now_ms - frame_timestamp_ms);
    }
    return true;
}

esp_err_t radar_pwm_init(void)
{
    if (s_pwm_ready) {
        return ESP_ERR_INVALID_STATE;
    }

    const ledc_timer_config_t timer_config = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = RADAR_PWM_FREQUENCY_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };

    esp_err_t ret = ledc_timer_config(&timer_config);
    if (ret != ESP_OK) {
        return ret;
    }

    const ledc_channel_config_t channel_config = {
        .gpio_num = RADAR_PWM_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0,
        .flags = {0},
    };
    ret = ledc_channel_config(&channel_config);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, RADAR_PWM_DUTY);
    if (ret == ESP_OK) {
        ret = ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    }
    if (ret != ESP_OK) {
        return ret;
    }

    s_pwm_ready = true;
    ESP_LOGI(TAG,
             "M_CTR PWM ready GPIO=%u frequency=%uHz duty=%u%%",
             (unsigned int)RADAR_PWM_GPIO,
             (unsigned int)RADAR_PWM_FREQUENCY_HZ,
             (unsigned int)RADAR_PWM_DUTY_PERCENT);
    return ESP_OK;
}

bool radar_pwm_is_running(void)
{
    return s_pwm_ready;
}
