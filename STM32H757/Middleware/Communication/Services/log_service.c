#include "log_service.h"

#include <string.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include "bsp_uart.h"
#include "rtos_health.h"
#include "smartcar_debug_config.h"

/* CM7 日志服务实现；创建人：待确认（当前维护人：Zhiqin）。 */

#define LOG_SERVICE_QUEUE_DEPTH UINT32_C(24)
#define LOG_SERVICE_TASK_STACK_WORDS UINT32_C(384)
#define LOG_SERVICE_TASK_PRIORITY (tskIDLE_PRIORITY + 1U)
typedef struct {
    uint8_t level;
    char text[SMARTCAR_LOG_MAX_PAYLOAD + 1U];
} log_item_t;

static QueueHandle_t s_log_queue;
static TaskHandle_t s_log_task;
static volatile uint32_t s_log_drop_count;

/**
 * @brief 将 NUL 结尾日志文本有界复制到队列项并保证目标尾部终止。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[out] destination 调用方拥有的可写目标缓冲区；允许为 NULL。
 * @param[in] capacity 目标容量，单位字节；为 0 时不写入。
 * @param[in] source 调用期间只读的 NUL 结尾源字符串；NULL 时向有效目标写空字符串。
 * @return 无返回值；目标无效/零容量时静默返回，源超长时截断至 `capacity-1`；源若没有
 *         可达 NUL 结尾，`strlen()` 可能越界读取且无错误码。
 * 调用方式：仅 `log_service_write()` 在零等待入队前构造栈上 `log_item_t` 时调用。
 * 线程约束：不获取 mutex、不主动阻塞且可重入，但执行时间随源长度增长，禁止 ISR；
 *           不保存或接管源/目标所有权，同一目标不得被并发写入。
 */
static void log_copy_text(char *destination, size_t capacity, const char *source)
{
    size_t length;

    if (destination == NULL || capacity == 0U) {
        return;
    }
    if (source == NULL) {
        destination[0] = '\0';
        return;
    }
    length = strlen(source);
    if (length >= capacity) {
        length = capacity - 1U;
    }
    memcpy(destination, source, length);
    destination[length] = '\0';
}

/**
 * @brief 串行输出日志队列，并按配置周期采样 RTOS `.noinit` 健康快照。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] argument FreeRTOS 任务参数，当前忽略且不保存，允许 NULL。
 * @return 不返回；任务永久运行，每次队列等待最多 250 ms。
 * 调用方式：仅由 `log_service_start()` 创建一个实例；实际 UART 写失败由 BSP 统计观察。
 * 线程约束：日志队列唯一消费者；`xQueueReceive()` 最多阻塞 250 ms，UART 写还可能按超时
 *           阻塞并获取 BSP mutex，RTOS 健康读取无全局事务锁；严禁 ISR/并发实例，队列项
 *           按值接收且任务不接管 argument 所有权。
 */
static void log_service_task(void *argument)
{
    log_item_t item;
    TickType_t last_health_tick = xTaskGetTickCount();

    (void)argument;
    for (;;) {
        if (xQueueReceive(s_log_queue, &item, pdMS_TO_TICKS(250U)) == pdTRUE) {
            (void)bsp_uart_log_write_link_level(item.level, item.text);
        }

        if ((xTaskGetTickCount() - last_health_tick) >=
            pdMS_TO_TICKS(LOG_SERVICE_HEALTH_PERIOD_MS)) {
#if !SMARTCAR_BMI323_DEBUG_ONLY
            rtos_health_snapshot_t snapshot;
            char line[SMARTCAR_LOG_MAX_PAYLOAD + 1U];

            last_health_tick = xTaskGetTickCount();
            rtos_health_sample();
            if (rtos_health_get_snapshot(&snapshot)) {
                (void)snprintf(line, sizeof(line),
                               "RTOS_HEALTH h=%lu m=%lu i=%lu u=%lu s=%lu",
                               (unsigned long)snapshot.current_free_heap_bytes,
                               (unsigned long)snapshot.minimum_free_heap_bytes,
                               (unsigned long)snapshot.stacks[0].current_free_stack_words,
                               (unsigned long)snapshot.stacks[1].current_free_stack_words,
                               (unsigned long)snapshot.stacks[2].current_free_stack_words);
                (void)bsp_uart_log_write_link_level(
                    SMARTCAR_LOG_LEVEL_INFO, line);
                (void)snprintf(line, sizeof(line),
                               "RTOS_HEALTH l=%lu p=%lu d=%lu o=%lu f=%lu",
                               (unsigned long)snapshot.stacks[3].current_free_stack_words,
                               (unsigned long)snapshot.stacks[4].current_free_stack_words,
                               (unsigned long)snapshot.stacks[5].current_free_stack_words,
                               (unsigned long)snapshot.stack_overflow_count,
                               (unsigned long)snapshot.malloc_failed_count);
                (void)bsp_uart_log_write_link_level(
                    SMARTCAR_LOG_LEVEL_INFO, line);
            }
            (void)snprintf(line, sizeof(line), "LOG_STATS drop=%lu",
                           (unsigned long)s_log_drop_count);
            (void)bsp_uart_log_write_link_level(SMARTCAR_LOG_LEVEL_INFO, line);
#else
            last_health_tick = xTaskGetTickCount();
#endif
        }
    }
}

/** 复制有限长度文本入日志队列；队列满时丢弃并计数。 */
void log_service_write(smartcar_log_level_t level, const char *text)
{
    log_item_t item;

    if (text == NULL || level > SMARTCAR_LOG_LEVEL_ERROR ||
        s_log_queue == NULL) {
        ++s_log_drop_count;
        return;
    }
    item.level = (uint8_t)level;
    log_copy_text(item.text, sizeof(item.text), text);
    if (xQueueSend(s_log_queue, &item, 0U) != pdTRUE) {
        ++s_log_drop_count;
    }
}

/** 初始化日志队列和统计。 */
void log_service_init(void)
{
    s_log_drop_count = 0U;
    s_log_task = NULL;
    s_log_queue = xQueueCreate(LOG_SERVICE_QUEUE_DEPTH, sizeof(log_item_t));
}

/** 创建唯一日志任务。 */
void log_service_start(void)
{
    if (s_log_queue == NULL || s_log_task != NULL) {
        return;
    }
    (void)xTaskCreate(log_service_task, "logger", LOG_SERVICE_TASK_STACK_WORDS,
                      NULL, LOG_SERVICE_TASK_PRIORITY, &s_log_task);
    if (s_log_task == NULL) {
        ++s_log_drop_count;
    }
}

/** 读取日志队列丢弃累计值。 */
uint32_t log_service_get_drop_count(void)
{
    return s_log_drop_count;
}
