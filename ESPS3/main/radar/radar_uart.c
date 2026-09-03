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
#include "smartcar_debug_config.h"

/* 雷达 UART1 接收、解析和 FIFO 投递实现；创建人：待确认（当前维护人：Zhiqin）。 */

static const char *TAG = "RADAR";

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

/**
 * @brief  识别 UART FIFO/驱动环形缓冲溢出事件并重置接收流边界。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（函数契约补充）。
 * @param  event UART 驱动事件队列弹出的借用对象；可为 NULL，仅本次调用期间读取。
 * @return 已处理 UART_FIFO_OVF 或 UART_BUFFER_FULL 时为 true；其他事件或 NULL 返回 false。
 * 调用方式：仅由 radar_uart_task() 收到事件后调用；处理时清驱动输入、复位事件队列并丢弃 parser 半帧。
 * 线程约束：运行在雷达 UART 任务而非 ISR；会调用驱动/FreeRTOS API 和日志，禁止并发或 ISR 调用。
 * 失败语义：溢出计数增加后无法恢复已丢字节，只能重新等待完整帧头。
 */
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

/**
 * @brief  将解析器给出的完整帧复制到有界 FIFO，并通知可选上行任务。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（函数契约补充）。
 * @param  data 解析器栈上的临时完整帧，仅在回调返回前有效；函数不保留指针。
 * @param  length 帧字节数，必须为 1..RADAR_PARSER_MAX_FRAME_SIZE。
 * @param  context parser 回调上下文，当前实现忽略，可为 NULL。
 * @return 无；参数无效、mutex 未创建或 1 tick 内取锁失败时静默丢弃该帧并按条件计数。
 * 调用方式：由 radar_parser_feed() 在雷达 UART 任务栈同步回调；取得 mutex 后先递增序号，
 *           只有 FIFO push 成功才更新最近帧元数据并通知消费者。
 * 线程约束：非 ISR；最多等待 FIFO mutex 1 个 RTOS tick并复制整帧。FIFO 满会丢最旧帧，锁失败丢当前帧。
 */
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

/**
 * @brief  按统一周期汇总解析、FIFO、UART、任务栈和堆资源诊断。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return 无；节流周期未到时立即返回，FIFO mutex 忙时仍输出其余统计且 FIFO 快照保持零值。
 * 调用方式：radar_uart_task() 每轮末尾调用；可选 BLE 日志只有 FFE3 就绪且格式化成功时尝试发送。
 * 线程约束：仅雷达 UART 任务调用；FIFO mutex 使用零等待，但日志、heap/stack 查询和 snprintf 不适合 ISR。
 * 数据边界：统计只证明本地 UART/解析/FIFO 状态，不证明雷达物理量或 TCP/ROS2 端到端可用。
 */
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
/**
 * @brief  按调试宏和节流周期输出有限字节的 UART 十六进制预览。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（函数契约补充）。
 * @param  data 接收块起始地址；调用点保证 length 大于 0 时非 NULL，函数不保留指针。
 * @param  length 本次 UART 接收字节数；输出最多显示统一宏配置的字节上限。
 * @return 无；未到日志周期、BLE 未就绪或格式化空间不足时跳过相应输出。
 * 调用方式：仅在 RADAR_UART_RAW_LOG_ENABLED 编译开启时由 drain 路径调用；BLE 原始日志还受依赖宏控制。
 * 线程约束：使用模块级静态字符缓冲、不可重入；运行在 UART 任务，日志和 snprintf 会增加延迟，
 *           禁止 ISR 调用。
 */
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

/**
 * @brief  非阻塞排空当前 UART 驱动缓冲，并把字节块交给增量解析器。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return 无；查询失败、无缓存字节或零超时读取未取得数据时结束本轮排空。
 * 调用方式：radar_uart_task() 在非溢出事件或周期唤醒后调用；每个接收块可同步产生多个帧回调。
 * 线程约束：只允许雷达 UART 任务调用并独占 s_read_buffer/s_parser；read 超时为 0，但解析/复制/调试会耗时，
 *           禁止 ISR 或其他任务并发调用。
 */
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

/**
 * @brief  持续消费 UART1 事件、排空雷达字节流、解析完整帧并输出健康统计。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（函数契约补充）。
 * @param  context xTaskCreate() 传入的任务上下文，当前固定为 NULL 并被忽略。
 * @return 无；FreeRTOS 任务进入永久循环，不应返回。
 * 调用方式：仅由 radar_uart_init() 创建一次；事件队列最多阻塞 RADAR_UART_EVENT_WAIT_MS 后仍执行排空/统计。
 * 线程约束：该任务独占 UART1 接收解析器和读缓冲；不是 ISR，禁止手工直接调用或访问 STM UART2。
 * 故障恢复：FIFO/驱动缓冲溢出会清输入并重置半帧，已丢数据不可恢复，后续从新帧头继续。
 */
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

/**
 * @brief  配置雷达 UART1 接收驱动、PSRAM 帧 FIFO、mutex 和持续解析任务。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return ESP_OK 表示所有资源和任务创建成功；重复调用为 ESP_ERR_INVALID_STATE，
 *         PSRAM/mutex/任务不足为 ESP_ERR_NO_MEM，UART 配置失败返回对应 ESP-IDF 错误。
 * 调用方式：app_main() 在 radar_uplink_init() 前调用一次；失败时不得启动依赖原始帧的上行链路。
 * 线程约束：仅启动任务调用；会分配大量 PSRAM、创建 FreeRTOS 对象和任务，禁止 ISR 或并发初始化。
 * 硬件边界：只配置 UART1 RX=GPIO44、115200 8N1，不配置 TX，也不访问 STM UART2 GPIO17/18。
 */
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

/**
 * @brief  查询雷达 UART 初始化标志与接收任务句柄是否同时有效。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return 两项均有效时为 true；不表示 UART 收到字节、解析出帧或雷达物理在线。
 * 调用方式：启动状态和诊断读取；链路健康仍需结合 parser/FIFO 计数和帧时间戳。
 * 线程约束：无锁快照、不阻塞；初始化完成后读取，禁止把返回值当作同步屏障。
 */
bool radar_uart_is_running(void)
{
    return s_uart_ready && s_uart_task != NULL;
}

/**
 * @brief  设置完整帧入队后通过 task notification 唤醒的消费者任务。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（函数契约补充）。
 * @param  task FreeRTOS 任务句柄；非 NULL 时保存并立即发一次通知，NULL 表示取消通知目标。
 * @return 无。
 * 调用方式：UART 初始化成功且上行任务创建后注册；消费者删除前应先传 NULL，避免悬空句柄。
 * 线程约束：启动/上行生命周期串行调用，无内部锁；禁止 ISR 调用或与帧回调、任务删除并发修改。
 */
void radar_uart_set_frame_notification_task(TaskHandle_t task)
{
    s_uplink_notification_task = task;
    if (task != NULL) {
        xTaskNotifyGive(task);
    }
}

/**
 * @brief  在有限 mutex 等待内复制并弹出最旧的校验通过雷达帧。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（函数契约补充）。
 * @param[out] buffer 非 NULL、至少可写 capacity 字节的输出缓冲。
 * @param  capacity buffer 字节容量，必须大于 0。
 * @param[out] length 必须非 NULL；成功写帧长，空队列写 0，短缓冲写所需长度；
 *                    参数错误或 mutex 获取失败时保持调用前值。
 * @param[out] sequence 可为 NULL；成功时写 UART 接收侧帧序号。
 * @param[out] timestamp_ms 可为 NULL；成功时写采集时间，单位 ms。
 * @param[out] age_ms 可为 NULL；成功时写当前时间减采集时间，单位 ms，允许无符号回绕。
 * @return 已复制并消费时为 true；参数、初始化、mutex、空队列或短缓冲问题时为 false。
 * 调用方式：单一上行任务调用；false 且 length 写为更大值时可扩容重试，其他 false 不应忙等。
 * 线程约束：最多等待 FIFO mutex 1 个 RTOS tick并复制整帧；禁止 ISR 调用。
 */
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

/**
 * @brief  配置 GPIO4 上的 X3PRO M_CTR LEDC PWM 并提交默认占空比。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return ESP_OK 表示 timer、channel 和默认 duty 更新成功；重复调用为 ESP_ERR_INVALID_STATE，
 *         其他值为对应 LEDC 配置或更新错误。
 * 调用方式：app_main() 启动阶段调用；成功后再初始化 radar_control，运行期 duty 由其状态机管理。
 * 线程约束：仅启动任务调用一次；会配置 LEDC/GPIO4，禁止 ISR 或并发调用。
 * 硬件边界：返回成功只表示 S3 外设寄存器配置完成，不证明 PWM 波形、电机供电或雷达转动。
 */
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

/**
 * @brief  查询雷达 LEDC PWM 是否完成本地初始化。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return timer/channel/duty 更新成功后为 true；不表示当前 duty 非零、GPIO 有波形或电机已旋转。
 * 调用方式：仅用于初始化状态诊断；运行期门控和占空比状态读取使用 radar_control 接口。
 * 线程约束：无锁快照、不阻塞；初始化完成后读取，禁止作为跨任务同步屏障。
 */
bool radar_pwm_is_running(void)
{
    return s_pwm_ready;
}
