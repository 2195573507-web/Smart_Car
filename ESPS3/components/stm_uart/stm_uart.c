#include "stm_uart.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "srp_def.h"
#include "srp_registry.h"

#define STM_UART_STORAGE_SIZE 8192U
#define STM_UART_TASK_STACK_SIZE 3072U
#define STM_UART_RX_TASK_PRIORITY (configMAX_PRIORITIES - 3U)
#define STM_UART_RX_TASK_CORE 1
#define STM_UART_TASK_READ_SIZE 256U
#define STM_UART_TASK_READ_TIMEOUT_MS 10U
#define STM_UART_RECEIVE_TIMEOUT_MS 10U
#define STM_UART_EVENT_QUEUE_DEPTH 32U
#define STM_UART_RX_FULL_THRESHOLD 64
#define STM_UART_RX_TIMEOUT_SYMBOLS 10
#define STM_UART_EVENT_BUDGET 8U
#define STM_UART_BREAK_CONSECUTIVE_WINDOW_US UINT64_C(1000000)
#define STM_UART_TX_TIMEOUT_MS 100U
#define STM_UART_PRIORITY_COUNT 4U
#define STM_UART_ERROR_LOG_PERIOD_US UINT64_C(500000)
#define STM_UART_BOOT_CAPTURE_BYTES 32U

/* ESP32-S3 UART_INT_CLR bit positions; keep this component independent of
 * the private esp_driver_uart HAL enum names. */
#define STM_UART_INTR_PARITY_ERR (UINT32_C(1) << 2U)
#define STM_UART_INTR_FRAME_ERR (UINT32_C(1) << 3U)
#define STM_UART_INTR_RXFIFO_OVF (UINT32_C(1) << 4U)
#define STM_UART_INTR_BREAK (UINT32_C(1) << 7U)

static TaskHandle_t s_rx_task;
static QueueHandle_t s_event_queue;
static SemaphoreHandle_t s_storage_mutex;
static SemaphoreHandle_t s_tx_mutex;
static bool s_initialized;
static uint8_t s_storage[STM_UART_STORAGE_SIZE];
static size_t s_storage_head;
static size_t s_storage_tail;
static size_t s_storage_count;
static stm_uart_stats_t s_stats;
static volatile bool s_sync_state;
static volatile bool s_rx_discontinuity;
static volatile bool s_break_recovery_pending;
static uint32_t s_break_consecutive_count;
static uint64_t s_last_break_event_us;
static uint64_t s_last_sync_guard_log_us;
static uint64_t s_last_error_log_us;
static uint8_t s_boot_capture[STM_UART_BOOT_CAPTURE_BYTES];
static size_t s_boot_capture_len;
static bool s_boot_capture_logged;

static const char *stm_uart_event_name(uart_event_type_t type)
{
    switch (type) {
    case UART_BREAK: return "BREAK";
    case UART_DATA_BREAK: return "DATA_BREAK";
    case UART_BUFFER_FULL: return "BUFFER_FULL";
    case UART_FIFO_OVF: return "FIFO_OVF";
    case UART_FRAME_ERR: return "FRAME_ERR";
    case UART_PARITY_ERR: return "PARITY_ERR";
    default: return "UNKNOWN";
    }
}

static void stm_uart_handle_event(const uart_event_t *event)
{
    uint32_t clear_mask = 0U;
    bool flush_input = false;
    bool discontinuity = false;
    uint64_t now_us;

    if (event == NULL) {
        return;
    }
    switch (event->type) {
    case UART_FIFO_OVF:
        clear_mask = STM_UART_INTR_RXFIFO_OVF;
        flush_input = true;
        discontinuity = true;
        break;
    case UART_BUFFER_FULL:
        flush_input = true;
        discontinuity = true;
        break;
    case UART_FRAME_ERR:
        clear_mask = STM_UART_INTR_FRAME_ERR;
        flush_input = true;
        discontinuity = true;
        break;
    case UART_PARITY_ERR:
        clear_mask = STM_UART_INTR_PARITY_ERR;
        flush_input = true;
        discontinuity = true;
        break;
    case UART_BREAK:
        clear_mask = STM_UART_INTR_BREAK;
        flush_input = true;
        discontinuity = true;
        break;
    case UART_DATA_BREAK:
        clear_mask = STM_UART_INTR_BREAK;
        flush_input = true;
        discontinuity = true;
        break;
    default:
        return;
    }
    if (clear_mask != 0U) {
        (void)uart_clear_intr_status(STM_UART_PORT, clear_mask);
    }
    if (flush_input) {
        (void)uart_flush_input(STM_UART_PORT);
    }
    if (discontinuity) {
        /* The parser, not the transport, owns the incomplete SRP frame. Keep
         * already staged bytes so a complete frame before the line error is
         * still deliverable and the next parser feed can seek AA 55. */
        __atomic_store_n(&s_rx_discontinuity, true, __ATOMIC_RELEASE);
    }
    if (xSemaphoreTake(s_storage_mutex, 0U) == pdTRUE) {
        ++s_stats.rx_error_events;
        ++s_stats.hal_error;
        if (event->type == UART_BREAK || event->type == UART_DATA_BREAK) {
            ++s_stats.break_events;
        }
        if (event->type == UART_FIFO_OVF || event->type == UART_BUFFER_FULL) {
            ++s_stats.overflow;
        }
        (void)xSemaphoreGive(s_storage_mutex);
    }
    if (event->type == UART_BREAK || event->type == UART_DATA_BREAK) {
        const uint64_t break_now_us = (uint64_t)esp_timer_get_time();
        if (s_last_break_event_us == 0U ||
            break_now_us - s_last_break_event_us >
                STM_UART_BREAK_CONSECUTIVE_WINDOW_US) {
            __atomic_store_n(&s_break_consecutive_count, 0U,
                             __ATOMIC_RELEASE);
        }
        s_last_break_event_us = break_now_us;
        const uint32_t break_count = __atomic_add_fetch(
            &s_break_consecutive_count, 1U, __ATOMIC_ACQ_REL);
        if (break_count >= STM_UART_BREAK_RECOVERY_THRESHOLD &&
            !__atomic_exchange_n(&s_break_recovery_pending, true,
                                 __ATOMIC_ACQ_REL)) {
            if (xSemaphoreTake(s_storage_mutex, 0U) == pdTRUE) {
                ++s_stats.break_recoveries;
                (void)xSemaphoreGive(s_storage_mutex);
            }
        }
    }
    now_us = (uint64_t)esp_timer_get_time();
    if (now_us - s_last_error_log_us >= STM_UART_ERROR_LOG_PERIOD_US) {
        s_last_error_log_us = now_us;
        ESP_LOGW("STM_UART", "UART2 RX error event=%s; hardware input flushed; "
                 "software ring preserved; parser reset pending",
                 stm_uart_event_name(event->type));
    }
}

static bool stm_uart_is_motion_type(uint8_t type)
{
    return type == SRP_MSG_ID_MOTOR_CMD ||
           type == SRP_MSG_ID_WHEEL_SPEED_SINGLE_CMD ||
           type == SRP_MSG_ID_MASTER_SPEED_CMD ||
           type == SRP_MSG_ID_CHASSIS_SPEED_CMD ||
           type == SRP_MSG_ID_CHASSIS_HEADING_CMD;
}

static uint8_t stm_uart_priority(const uint8_t *data, size_t len)
{
    if (data == NULL || len < SRP_HEADER_SIZE || data[0] != SRP_MAGIC_BYTE0 ||
        data[1] != SRP_MAGIC_BYTE1 || data[7] >= STM_UART_PRIORITY_COUNT) {
        return UINT8_MAX;
    }
    return data[7];
}

static void stm_uart_capture_boot_bytes(const uint8_t *data, size_t len)
{
    uint8_t snapshot[STM_UART_BOOT_CAPTURE_BYTES];
    bool dump = false;

    if (data == NULL || len == 0U || s_boot_capture_logged ||
        xSemaphoreTake(s_storage_mutex, pdMS_TO_TICKS(10U)) != pdTRUE) {
        return;
    }
    if (s_boot_capture_len < STM_UART_BOOT_CAPTURE_BYTES) {
        const size_t remaining = STM_UART_BOOT_CAPTURE_BYTES - s_boot_capture_len;
        const size_t copy_len = len < remaining ? len : remaining;

        (void)memcpy(&s_boot_capture[s_boot_capture_len], data, copy_len);
        s_boot_capture_len += copy_len;
        if (s_boot_capture_len == STM_UART_BOOT_CAPTURE_BYTES) {
            (void)memcpy(snapshot, s_boot_capture, sizeof(snapshot));
            s_boot_capture_logged = true;
            dump = true;
        }
    }
    (void)xSemaphoreGive(s_storage_mutex);

    if (dump) {
        char hex[(STM_UART_BOOT_CAPTURE_BYTES * 3U) + 1U];
        size_t offset = 0U;

        hex[0] = '\0';
        for (size_t index = 0U; index < sizeof(snapshot); ++index) {
            const int written = snprintf(&hex[offset], sizeof(hex) - offset,
                                         index == 0U ? "%02X" : " %02X",
                                         (unsigned)snapshot[index]);
            if (written < 0 || (size_t)written >= sizeof(hex) - offset) {
                break;
            }
            offset += (size_t)written;
        }
        ESP_LOGI("STM_UART", "[STM_UART_BOOT32] %s", hex);
    }
}

static void stm_uart_store(const uint8_t *data, size_t len)
{
    if (xSemaphoreTake(s_storage_mutex, pdMS_TO_TICKS(10U)) != pdTRUE) {
        s_stats.drop += (uint32_t)len;
        return;
    }
    if (len >= STM_UART_STORAGE_SIZE) {
        s_stats.overflow++;
        s_stats.drop += (uint32_t)(s_storage_count + len - STM_UART_STORAGE_SIZE);
        data += len - STM_UART_STORAGE_SIZE;
        len = STM_UART_STORAGE_SIZE;
        s_storage_head = 0U;
        s_storage_tail = 0U;
        s_storage_count = 0U;
    }
    while (s_storage_count + len > STM_UART_STORAGE_SIZE) {
        ++s_stats.overflow;
        ++s_stats.drop;
        s_storage_tail = (s_storage_tail + 1U) % STM_UART_STORAGE_SIZE;
        --s_storage_count;
    }
    for (size_t index = 0U; index < len; ++index) {
        s_storage[s_storage_head] = data[index];
        s_storage_head = (s_storage_head + 1U) % STM_UART_STORAGE_SIZE;
    }
    s_storage_count += len;
    (void)xSemaphoreGive(s_storage_mutex);
}

static void stm_uart_rx_task(void *context)
{
    uint8_t read_buffer[STM_UART_TASK_READ_SIZE];
    uart_event_t event;
    uint8_t event_budget;

    (void)context;
    for (;;) {
        event_budget = 0U;
        while (event_budget < STM_UART_EVENT_BUDGET && s_event_queue != NULL &&
               xQueueReceive(s_event_queue, &event, 0U) == pdPASS) {
            stm_uart_handle_event(&event);
            ++event_budget;
        }
        const int received = uart_read_bytes(STM_UART_PORT, read_buffer,
                                             sizeof(read_buffer),
                                             pdMS_TO_TICKS(STM_UART_TASK_READ_TIMEOUT_MS));
        if (received > 0) {
            stm_uart_capture_boot_bytes(read_buffer, (size_t)received);
            if (xSemaphoreTake(s_storage_mutex, pdMS_TO_TICKS(10U)) == pdTRUE) {
                s_stats.rx_bytes += (uint32_t)received;
                ++s_stats.rx_task_reads;
                (void)xSemaphoreGive(s_storage_mutex);
            }
            stm_uart_store(read_buffer, (size_t)received);
        } else if (received < 0) {
            if (xSemaphoreTake(s_storage_mutex, pdMS_TO_TICKS(10U)) == pdTRUE) {
                ++s_stats.hal_error;
                (void)xSemaphoreGive(s_storage_mutex);
            }
        }
    }
}

esp_err_t stm_uart_init(void)
{
    const uart_config_t config = {
        .baud_rate = STM_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_DEFAULT,
    };
    esp_err_t ret;

    if (s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    ESP_LOGI("STM_UART",
             "UART2 init TX=GPIO%u RX=GPIO%u baud=%u rx_buffer=%u tx_buffer=%u",
             (unsigned)STM_UART_TX_GPIO, (unsigned)STM_UART_RX_GPIO,
             (unsigned)STM_UART_BAUD_RATE,
             (unsigned)STM_UART_RX_DRIVER_BUFFER_SIZE,
             (unsigned)STM_UART_TX_DRIVER_BUFFER_SIZE);
    ret = uart_param_config(STM_UART_PORT, &config);
    if (ret == ESP_OK) {
        ret = uart_set_pin(STM_UART_PORT, STM_UART_TX_GPIO, STM_UART_RX_GPIO,
                           UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    }
    if (ret == ESP_OK) {
        ret = uart_driver_install(STM_UART_PORT, STM_UART_RX_DRIVER_BUFFER_SIZE,
                                  STM_UART_TX_DRIVER_BUFFER_SIZE,
                                  STM_UART_EVENT_QUEUE_DEPTH, &s_event_queue, 0);
    }
    if (ret == ESP_OK) {
        ret = uart_set_rx_full_threshold(STM_UART_PORT,
                                          STM_UART_RX_FULL_THRESHOLD);
    }
    if (ret == ESP_OK) {
        ret = uart_set_rx_timeout(STM_UART_PORT, STM_UART_RX_TIMEOUT_SYMBOLS);
    }
    if (ret != ESP_OK) {
        ESP_LOGE("STM_UART", "UART2 init failed err=%d", (int)ret);
        return ret;
    }
    s_storage_mutex = xSemaphoreCreateMutex();
    s_tx_mutex = xSemaphoreCreateMutex();
    if (s_storage_mutex == NULL || s_tx_mutex == NULL) {
        if (s_storage_mutex != NULL) {
            vSemaphoreDelete(s_storage_mutex);
            s_storage_mutex = NULL;
        }
        if (s_tx_mutex != NULL) {
            vSemaphoreDelete(s_tx_mutex);
            s_tx_mutex = NULL;
        }
        (void)uart_driver_delete(STM_UART_PORT);
        ESP_LOGE("STM_UART", "UART2 mutex allocation failed");
        return ESP_ERR_NO_MEM;
    }
    s_storage_head = 0U;
    s_storage_tail = 0U;
    s_storage_count = 0U;
    (void)memset(&s_stats, 0, sizeof(s_stats));
    s_sync_state = false;
    s_rx_discontinuity = false;
    s_break_recovery_pending = false;
    __atomic_store_n(&s_break_consecutive_count, 0U, __ATOMIC_RELEASE);
    s_last_break_event_us = 0U;
    s_last_sync_guard_log_us = 0U;
    s_last_error_log_us = 0U;
    (void)memset(s_boot_capture, 0, sizeof(s_boot_capture));
    s_boot_capture_len = 0U;
    s_boot_capture_logged = false;
    s_initialized = true;
    if (xTaskCreatePinnedToCore(stm_uart_rx_task, "srp_uart_rx",
                                STM_UART_TASK_STACK_SIZE, NULL,
                                STM_UART_RX_TASK_PRIORITY, &s_rx_task,
                                STM_UART_RX_TASK_CORE) != pdPASS) {
        s_initialized = false;
        ESP_LOGE("STM_UART", "UART2 RX task creation failed rx=%p",
                 (void *)s_rx_task);
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI("STM_UART", "UART2 ready rx_task=%p blocking_tx=1",
             (void *)s_rx_task);
    return ESP_OK;
}

int stm_uart_send(const uint8_t *data, size_t len)
{
    const uint8_t priority = stm_uart_priority(data, len);
    int written;
    esp_err_t wait_status = ESP_FAIL;

    if (!s_initialized || priority >= STM_UART_PRIORITY_COUNT ||
        len == 0U || len > SRP_MAX_FRAME_SIZE) {
        return -1;
    }
    if (!s_sync_state && len >= SRP_HEADER_SIZE &&
        stm_uart_is_motion_type(data[6])) {
        if (xSemaphoreTake(s_storage_mutex, 0U) == pdTRUE) {
            ++s_stats.sync_guard_drop;
            (void)xSemaphoreGive(s_storage_mutex);
        }
        {
            const uint64_t now_us = (uint64_t)esp_timer_get_time();
            if (now_us - s_last_sync_guard_log_us >= UINT64_C(500000)) {
                s_last_sync_guard_log_us = now_us;
                ESP_LOGW("STM_UART", "motion TX dropped before SRP sync type=0x%02X",
                         (unsigned)data[6]);
            }
        }
        return -1;
    }
    if (xSemaphoreTake(s_tx_mutex, pdMS_TO_TICKS(STM_UART_TX_TIMEOUT_MS)) !=
        pdTRUE) {
        if (xSemaphoreTake(s_storage_mutex, 0U) == pdTRUE) {
            ++s_stats.hal_error;
            (void)xSemaphoreGive(s_storage_mutex);
        }
        return -1;
    }
    written = uart_write_bytes(STM_UART_PORT, data, len);
    if (written == (int)len) {
        wait_status = uart_wait_tx_done(STM_UART_PORT,
                                        pdMS_TO_TICKS(STM_UART_TX_TIMEOUT_MS));
    }
    (void)xSemaphoreGive(s_tx_mutex);

    if (xSemaphoreTake(s_storage_mutex, pdMS_TO_TICKS(10U)) == pdTRUE) {
        if (written > 0) {
            s_stats.tx_bytes += (uint32_t)written;
        }
        if (written != (int)len) {
            ++s_stats.short_write;
            ++s_stats.tx_write_errors;
            if (written < (int)len) {
                s_stats.drop += (uint32_t)(len - (written > 0 ? written : 0));
            }
        } else if (wait_status != ESP_OK) {
            ++s_stats.tx_write_errors;
            ++s_stats.hal_error;
        }
        (void)xSemaphoreGive(s_storage_mutex);
    }
    return written == (int)len && wait_status == ESP_OK ? written : -1;
}

void stm_uart_get_stats(stm_uart_stats_t *stats)
{
    if (stats == NULL || s_storage_mutex == NULL ||
        xSemaphoreTake(s_storage_mutex, pdMS_TO_TICKS(10U)) != pdTRUE) {
        if (stats != NULL) {
            (void)memset(stats, 0, sizeof(*stats));
        }
        return;
    }
    *stats = s_stats;
    stats->rx_buffered = (uint16_t)(s_storage_count > UINT16_MAX
                                        ? UINT16_MAX : s_storage_count);
    stats->tx_queue_pending = 0U;
    (void)xSemaphoreGive(s_storage_mutex);
}

void stm_uart_recover(void)
{
    if (!s_initialized) {
        return;
    }
    /* Drop both hardware FIFOs before a recovery or baud-rate switch. */
    (void)uart_flush(STM_UART_PORT);
    (void)uart_flush_input(STM_UART_PORT);
    (void)uart_clear_intr_status(STM_UART_PORT,
                                 STM_UART_INTR_RXFIFO_OVF |
                                     STM_UART_INTR_FRAME_ERR |
                                     STM_UART_INTR_PARITY_ERR |
                                     STM_UART_INTR_BREAK);
    if (s_event_queue != NULL) {
        (void)xQueueReset(s_event_queue);
    }
    if (xSemaphoreTake(s_storage_mutex, pdMS_TO_TICKS(10U)) == pdTRUE) {
        s_storage_head = 0U;
        s_storage_tail = 0U;
        s_storage_count = 0U;
        s_rx_discontinuity = true;
        (void)xSemaphoreGive(s_storage_mutex);
    }
    s_break_recovery_pending = false;
    __atomic_store_n(&s_break_consecutive_count, 0U, __ATOMIC_RELEASE);
    s_last_break_event_us = 0U;
}

esp_err_t stm_uart_set_baud_rate(uint32_t baud_rate)
{
    if (!s_initialized || baud_rate == 0U) {
        return ESP_ERR_INVALID_STATE;
    }
    stm_uart_recover();
    return uart_set_baudrate(STM_UART_PORT, baud_rate);
}

static int stm_uart_receive_with_timeout(uint8_t *buffer, size_t max_len,
                                         TickType_t timeout_ticks)
{
    if (!s_initialized || buffer == NULL || max_len == 0U ||
        xSemaphoreTake(s_storage_mutex, timeout_ticks) != pdTRUE) {
        return 0;
    }
    const size_t length = s_storage_count < max_len ? s_storage_count : max_len;
    for (size_t index = 0U; index < length; ++index) {
        buffer[index] = s_storage[s_storage_tail];
        s_storage_tail = (s_storage_tail + 1U) % STM_UART_STORAGE_SIZE;
    }
    s_storage_count -= length;
    (void)xSemaphoreGive(s_storage_mutex);
    return (int)length;
}

int stm_uart_receive_nonblock(uint8_t *buffer, size_t max_len)
{
    return stm_uart_receive_with_timeout(buffer, max_len, 0U);
}

int stm_uart_receive(uint8_t *buffer, size_t max_len)
{
    return stm_uart_receive_with_timeout(buffer, max_len,
                                         pdMS_TO_TICKS(STM_UART_RECEIVE_TIMEOUT_MS));
}

bool stm_uart_take_rx_discontinuity(void)
{
    /* Atomic exchange avoids losing an event that arrives while the service
     * task is consuming the previous notification. */
    return __atomic_exchange_n(&s_rx_discontinuity, false, __ATOMIC_ACQ_REL);
}

bool stm_uart_take_break_recovery(void)
{
    return __atomic_exchange_n(&s_break_recovery_pending, false,
                               __ATOMIC_ACQ_REL);
}

void stm_uart_set_sync_state(bool synced)
{
    s_sync_state = synced;
    if (synced) {
        __atomic_store_n(&s_break_consecutive_count, 0U, __ATOMIC_RELEASE);
        __atomic_store_n(&s_break_recovery_pending, false, __ATOMIC_RELEASE);
        s_last_break_event_us = 0U;
    }
}
