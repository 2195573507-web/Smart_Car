#include "boot_log.h"

/* 启动日志实现；创建人：待确认（当前维护人：Zhiqin）。 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "main.h"
#include "log_service.h"

#define BOOT_LOG_QUEUE_DEPTH    16U
#define BOOT_LOG_MODULE_LENGTH  16U
#define BOOT_LOG_STATUS_LENGTH  64U
#define BOOT_LOG_LINE_LENGTH    128U

typedef struct
{
    char module[BOOT_LOG_MODULE_LENGTH];
    char status[BOOT_LOG_STATUS_LENGTH];
    uint32_t elapsed_ms;
} boot_log_pending_t;

static boot_log_pending_t pending_logs[BOOT_LOG_QUEUE_DEPTH];
static uint32_t pending_count;
static uint32_t boot_start_ms;
static uint8_t boot_started;
static uint8_t uart_ready;

/**
 * @brief 将字符串截断复制到启动缓存，并保证尾部为 NUL。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[out] destination 调用方拥有的目标缓冲区；允许为 NULL，成功时始终以 NUL 结尾。
 * @param[in] capacity 目标缓冲区容量，单位字节；为 0 时不写入。
 * @param[in] source 调用期间只读的 NUL 结尾源字符串；NULL 时向有效目标写空字符串。
 * @return 无返回值；目标无效/零容量时静默返回，超长源由 `snprintf()` 截断。
 * 调用方式：仅 `boot_log()` 的 UART 未就绪缓存路径分别复制 module 和 status。
 * 线程约束：执行有界格式化，不获取 mutex、无主动等待但非 ISR 安全；启动期调用方必须
 *           串行访问 pending 队列，函数不保存或接管任一缓冲区所有权。
 */
static void boot_log_copy(char *destination, size_t capacity, const char *source)
{
    if (destination == NULL || capacity == 0U) {
        return;
    }
    if (source == NULL) {
        destination[0] = '\0';
        return;
    }
    (void)snprintf(destination, capacity, "%s", source);
}

/**
 * @brief 将启动事件格式化为有界文本，并提交给日志服务。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] module 调用期间只读的模块名；SYSTEM/LSM303 指定组合映射为稳定标记。
 * @param[in] status 调用期间只读的启动状态文本。
 * @param[in] elapsed_ms 相对 `boot_log_start()` 的耗时，单位 ms。
 * @return 无返回值；module/status 为 NULL 时不输出；格式化截断、日志队列未就绪或满导致
 *         的丢弃不会回传。
 * 调用方式：由 `boot_log_uart_ready()` 刷新缓存或 UART ready 后的 `boot_log()` 调用。
 * 线程约束：执行 `strcmp()`/`snprintf()` 并通过日志服务零等待入队，不直接持有 mutex；
 *           仅启动/任务上下文串行调用，禁止 ISR，不保存两个文本指针或接管所有权。
 */
static void boot_log_emit(const char *module, const char *status,
                          uint32_t elapsed_ms)
{
    char line[BOOT_LOG_LINE_LENGTH];

    if (module == NULL || status == NULL) {
        return;
    }
    (void)snprintf(line, sizeof(line), "BOOT[%s] %s t=%lums",
                   module, status, (unsigned long)elapsed_ms);
    if (strcmp(module, "SYSTEM") == 0 && strcmp(status, "START") == 0) {
        (void)snprintf(line, sizeof(line), "BOOT_START");
    } else if (strcmp(module, "SYSTEM") == 0 && strcmp(status, "READY") == 0) {
        (void)snprintf(line, sizeof(line), "BOOT_READY");
    } else if (strcmp(module, "LSM303") == 0 && strcmp(status, "INIT OK") == 0) {
        (void)snprintf(line, sizeof(line), "LSM303_INIT_OK");
    } else if (strcmp(module, "LSM303") == 0 && strcmp(status, "INIT FAIL") == 0) {
        (void)snprintf(line, sizeof(line), "LSM303_INIT_FAIL");
    }
    LOG_INFO(line);
}

/** 建立启动时间原点并清空早期事件。 */
void boot_log_start(void)
{
    boot_start_ms = HAL_GetTick();
    pending_count = 0U;
    boot_started = 1U;
    uart_ready = 0U;
}

/** 标记 USART1 日志可用并刷新早期事件。 */
void boot_log_uart_ready(void)
{
    uint32_t index;

    uart_ready = 1U;
    for (index = 0U; index < pending_count; ++index) {
        boot_log_emit(pending_logs[index].module,
                      pending_logs[index].status,
                      pending_logs[index].elapsed_ms);
    }
    pending_count = 0U;
}

/** 记录一条标准化启动事件。 */
void boot_log(const char *module, const char *status)
{
    uint32_t elapsed_ms;

    if (module == NULL || status == NULL) {
        return;
    }
    if (boot_started == 0U) {
        boot_log_start();
    }
    elapsed_ms = HAL_GetTick() - boot_start_ms;
    if (uart_ready != 0U) {
        boot_log_emit(module, status, elapsed_ms);
        return;
    }
    if (pending_count >= BOOT_LOG_QUEUE_DEPTH) {
        return;
    }
    boot_log_copy(pending_logs[pending_count].module,
                  sizeof(pending_logs[pending_count].module), module);
    boot_log_copy(pending_logs[pending_count].status,
                  sizeof(pending_logs[pending_count].status), status);
    pending_logs[pending_count].elapsed_ms = elapsed_ms;
    ++pending_count;
}
