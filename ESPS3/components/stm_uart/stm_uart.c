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
#include "smartcar_debug_config.h"

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

/**
 * @brief 将 ESP-IDF UART 错误事件类型映射为稳定的只读诊断名称。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param type UART driver 事件类型。
 * @return 指向静态字符串常量的借用指针；未列出的事件统一返回 "UNKNOWN"。
 * 调用方式：仅由 stm_uart_handle_event() 的限频错误日志路径调用。
 * 线程约束：纯查询、可重入、不阻塞，不转移字符串所有权；当前只在 UART RX 任务使用。
 */
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

/**
 * @brief 处理 UART 驱动溢出、帧、校验和两类 BREAK 事件。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param event ESP-IDF UART 事件借用指针；NULL 时返回，仅在调用期间有效。
 * @return 无；会清硬件输入、保留软件 ring、置 RX 不连续标志并累计有界诊断。
 * 调用方式：仅由 stm_uart_rx_task() 消费 driver event queue 时调用，不解析 SRP。
 * 线程约束：RX 任务单 owner；统计 mutex 仅零等待，禁止 ISR 或其他任务并发调用。
 */
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

/**
 * @brief 判断一个 SRP 消息 ID 是否属于同步完成前必须拦截的运动命令。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param type SRP 消息 ID 的低 8 位。
 * @return 命中电机、单轮、主速度、底盘速度或底盘航向命令时为 true，否则为 false。
 * 调用方式：仅由 stm_uart_send() 在本地 sync gate 为 false 时检查已编码帧头。
 * 线程约束：纯判断、可重入、不阻塞；不解析 payload，也不能替代服务层完整安全准入。
 */
static bool stm_uart_is_motion_type(uint8_t type)
{
    return type == SRP_MSG_ID_MOTOR_CMD ||
           type == SRP_MSG_ID_WHEEL_SPEED_SINGLE_CMD ||
           type == SRP_MSG_ID_MASTER_SPEED_CMD ||
           type == SRP_MSG_ID_CHASSIS_SPEED_CMD ||
           type == SRP_MSG_ID_CHASSIS_HEADING_CMD;
}

/**
 * @brief 从最小 SRP 头中读取优先级，并拒绝 magic、长度或优先级字段非法的输入。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param data 只读已编码字节流；允许 NULL，函数不保留指针。
 * @param len data 可读字节数，至少为 SRP_HEADER_SIZE 才会访问头字段。
 * @return 0..STM_UART_PRIORITY_COUNT-1 的优先级；输入非法时返回 UINT8_MAX。
 * 调用方式：由 stm_uart_send() 在获取 TX mutex 前做最小传输层检查；不校验总长、CRC 或注册表。
 * 线程约束：纯读取、可重入、不阻塞；data 必须在调用期间有效。
 */
static uint8_t stm_uart_priority(const uint8_t *data, size_t len)
{
    if (data == NULL || len < SRP_HEADER_SIZE || data[0] != SRP_MAGIC_BYTE0 ||
        data[1] != SRP_MAGIC_BYTE1 || data[7] >= STM_UART_PRIORITY_COUNT) {
        return UINT8_MAX;
    }
    return data[7];
}

/**
 * @brief 累积启动阶段前 STM_UART_BOOT_CAPTURE_BYTES 个 RX 字节并只打印一次十六进制快照。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param data 本次 UART driver 读取的只读缓冲；返回前完成复制，允许在 len 为 0 时为 NULL。
 * @param len 本次可读字节数；超过剩余抓取容量的尾部不进入诊断快照。
 * @return 返回值：无（void）；参数无效、已打印或 10 ms 内取锁失败时跳过本次抓取。
 * 调用方式：仅由 UART RX 任务在把同一批字节存入软件 ring 前调用；不改变业务接收内容。
 * 线程约束：最多等待 storage mutex 10 ms，格式化/ESP_LOG 在解锁后执行；单 RX 任务 owner，禁止 ISR 或其他任务调用。
 */
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

/**
 * @brief 把新 UART 字节复制进软件 ring，容量不足时丢最旧数据并累计 overflow/drop。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param data 只读输入；调用方必须保证 len>0 时非 NULL，返回前完成复制。
 * @param len 输入字节数；大于整个 ring 时只保留最后一段。
 * @return 无；10 ms 内拿不到 storage mutex 时整段丢弃并增加 drop。
 * 调用方式：只由 UART RX 任务在 driver read 成功后调用。
 * 线程约束：最多等待 mutex 10 ms，禁止 ISR 调用；单生产者、服务任务单消费者。
 */
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

/**
 * @brief 串行消费 UART driver 事件并把字节搬运到软件 ring，不执行业务解析。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param context FreeRTOS 任务参数，当前忽略，允许 NULL。
 * @return 不返回；任务永久运行。
 * 调用方式：仅由 stm_uart_init() 创建一个实例，smartcar_service 从独立 API 消费副本。
 * 线程约束：UART2 RX 唯一 owner；每轮有界消费事件并阻塞读 driver，禁止手工/ISR 调用。
 */
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

/**
 * @brief 初始化 UART2 驱动事件队列、收发 mutex、8 KiB 软件 ring 和固定核 RX 任务。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return 首次完整初始化返回 ESP_OK；已初始化返回 ESP_ERR_INVALID_STATE；驱动、mutex 或任务创建失败返回对应错误。
 * 调用方式：app_main 在任何收发 API 前调用一次；任务创建失败路径不会完整删除已安装 driver/mutex，不能假定可无副作用重试。
 * 线程约束：非幂等启动接口，仅启动任务串行调用；会分配驱动缓冲、队列、mutex 和 3072 字节任务栈，禁止 ISR/GATT 回调或并发调用。
 */
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

/**
 * @brief 经最小 SRP 头/优先级与运动同步门校验后，阻塞式发送一条完整帧。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param data 只读完整 SRP 帧；函数不接管所有权，返回前必须保持有效。
 * @param len 完整帧字节数，范围为 1..SRP_MAX_FRAME_SIZE，且须含可识别优先级头。
 * @return 全长写入且 uart_wait_tx_done() 成功时返回 len；参数、同步门、mutex、短写或等待失败返回 -1，失败前可能已有部分字节写入/发出。
 * 调用方式：仅由 smartcar_service/SRP link 的串行 transport 回调调用；同步前运动消息会被本层再次拒绝。
 * 线程约束：最多等待 TX mutex 100 ms，uart_write_bytes() 还可能等待驱动缓冲，随后再等待 TX 完成最多 100 ms；禁止 ISR/GATT 回调和递归发送。
 */
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

/**
 * @brief 在 storage mutex 下复制 UART 统计和当前软件 ring 深度，不清零累计计数。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param stats 可写输出结构；允许 NULL，NULL 时直接返回；锁失败时非 NULL 输出被清零。
 * @return 返回值：无（void）。
 * 调用方式：smartcar_service 低频诊断调用；rx_buffered 为饱和到 UINT16_MAX 的快照，阻塞 TX 实现下 tx_queue_pending 固定为 0。
 * 线程约束：最多等待 storage mutex 10 ms；禁止 ISR 或已持有同一 mutex 的路径调用，快照不证明物理 UART 收发成功。
 */
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

/**
 * @brief 丢弃 UART driver 输入/事件并尝试清空软件 ring，为上层重新建立 SRP 同步做准备。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return 返回值：无（void）；未初始化时不动作，10 ms 内拿不到 storage mutex 时软件 ring 不会清空且本函数不报告失败。
 * 调用方式：由服务任务在失步、连续 BREAK、BUS_OFF 或波特率切换前调用；调用后必须重新同步解析/会话状态。
 * 线程约束：会调用 driver flush/中断清理、重置事件队列并与 RX 任务并发；调用方须确保没有并发 TX，禁止 ISR/GATT 回调。
 */
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

/**
 * @brief 清理当前收发状态后，把 UART2 切换到调用方指定的任意非零波特率。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param baud_rate 目标 bit/s；传输层只拒绝 0，不校验 SRP 允许的默认/调试白名单。
 * @return 驱动接受切换返回 ESP_OK；未初始化或 0 返回 ESP_ERR_INVALID_STATE；其他值为 uart_set_baudrate() 错误。
 * 调用方式：仅由服务层完成 TLV 白名单校验并停止当前帧事务后调用；即使最终切换失败，先前 recover() 的丢弃/复位已发生。
 * 线程约束：任务上下文，可能 flush driver、重置队列并等待 storage mutex；禁止 ISR/GATT 回调或并发收发状态切换。
 */
esp_err_t stm_uart_set_baud_rate(uint32_t baud_rate)
{
    if (!s_initialized || baud_rate == 0U) {
        return ESP_ERR_INVALID_STATE;
    }
    stm_uart_recover();
    return uart_set_baudrate(STM_UART_PORT, baud_rate);
}

/**
 * @brief 在给定 mutex 等待时间内，从软件 ring 破坏性复制最多 max_len 字节。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param buffer 调用方可写输出；成功复制后字节归调用方，NULL 时返回 0。
 * @param max_len buffer 容量；0 时返回 0，实际复制量不超过 8 KiB ring 当前深度。
 * @param timeout_ticks 获取 storage mutex 的 FreeRTOS tick 上限，0 表示不等待。
 * @return 实际复制并从 ring 消费的字节数；未初始化、参数非法、锁超时或无数据均返回 0，调用方无法仅由 0 区分原因。
 * 调用方式：仅由 nonblock/短超时两个公开包装调用，保持 SRP 服务为单消费者。
 * 线程约束：持 mutex 完成逐字节复制，可能阻塞 RX 生产者；禁止 ISR、并发消费者或持有同一 mutex 调用。
 */
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

/**
 * @brief 以零等待方式从软件 ring 破坏性读取当前可用字节。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param buffer 调用方可写输出；NULL 时返回 0，返回后已复制字节归调用方。
 * @param max_len buffer 容量；0 时返回 0。
 * @return 实际复制字节数；无数据、参数/初始化错误或 mutex 正忙均返回 0。
 * 调用方式：由唯一 smartcar_service SRP 解析任务轮询；输出随后喂给 parser。
 * 线程约束：零等待获取 storage mutex、不阻塞；禁止 ISR 和并发消费者。
 */
int stm_uart_receive_nonblock(uint8_t *buffer, size_t max_len)
{
    return stm_uart_receive_with_timeout(buffer, max_len, 0U);
}

/**
 * @brief 最多等待 10 ms 获取 mutex，并从软件 ring 破坏性读取可用字节。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param buffer 调用方可写输出；NULL 时返回 0，返回后已复制字节归调用方。
 * @param max_len buffer 容量；0 时返回 0。
 * @return 实际复制字节数；无数据、参数/初始化错误或锁超时均返回 0。
 * 调用方式：需要短等待的单一服务任务可调用；当前主循环使用 nonblock 版本。
 * 线程约束：最多等待 storage mutex 10 ms，持锁期间复制数据；禁止 ISR/GATT 回调、并发消费者或锁内递归。
 */
int stm_uart_receive(uint8_t *buffer, size_t max_len)
{
    return stm_uart_receive_with_timeout(buffer, max_len,
                                         pdMS_TO_TICKS(STM_UART_RECEIVE_TIMEOUT_MS));
}

/**
 * @brief 原子读取并清除一次 UART RX 不连续通知。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return 自上次 take 后发生过 FIFO/driver 满、帧/校验或 BREAK 错误时为 true，否则为 false；多次事件合并为一次通知。
 * 调用方式：smartcar_service 在 parser feed 前轮询；true 时重置半帧并执行上层恢复策略。
 * 线程约束：使用 acquire-release 原子 exchange，可跨任务安全置位/消费；保持单一服务任务消费者，禁止 ISR 执行业务恢复。
 */
bool stm_uart_take_rx_discontinuity(void)
{
    /* Atomic exchange avoids losing an event that arrives while the service
     * task is consuming the previous notification. */
    return __atomic_exchange_n(&s_rx_discontinuity, false, __ATOMIC_ACQ_REL);
}

/**
 * @brief 原子读取并清除一次“1 秒窗口内连续 BREAK 达阈值”的恢复请求。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return 达到 STM_UART_BREAK_RECOVERY_THRESHOLD 后首次消费返回 true；否则 false，多次置位会合并。
 * 调用方式：由 smartcar_service 主循环轮询并在任务上下文执行 recover/重同步，不在 UART 事件路径直接重置协议。
 * 线程约束：使用 acquire-release 原子 exchange；保持单一服务任务消费者，禁止 ISR 执行后续恢复。
 */
bool stm_uart_take_break_recovery(void)
{
    return __atomic_exchange_n(&s_break_recovery_pending, false,
                               __ATOMIC_ACQ_REL);
}

/**
 * @brief 更新传输层运动帧同步门；进入同步时同时清除连续 BREAK 计数/待恢复标志。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param synced true 允许运动类 SRP 帧通过本层最小门控；false 拦截这些消息，但不停止其他协议流量。
 * @return 返回值：无（void）。
 * 调用方式：仅由 S3 服务状态机在握手完成、断链、超时或 BUS_OFF 边沿调用；本函数不能替代上层会话/安全门。
 * 线程约束：sync 为 volatile 写，BREAK 标志/计数用原子操作；服务任务负责状态机串行化，RX 任务仍可并发产生新 BREAK，禁止 ISR 调用。
 */
void stm_uart_set_sync_state(bool synced)
{
    s_sync_state = synced;
    if (synced) {
        __atomic_store_n(&s_break_consecutive_count, 0U, __ATOMIC_RELEASE);
        __atomic_store_n(&s_break_recovery_pending, false, __ATOMIC_RELEASE);
        s_last_break_event_us = 0U;
    }
}
