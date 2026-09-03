#include "bsp_uart.h"

/* UART BSP 实现；创建人：待确认（当前维护人：Zhiqin）。 */

#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "semphr.h"
#include "main.h"
#include "s3_service.h"
#include "uart_link.h"

extern UART_HandleTypeDef huart1;

static SemaphoreHandle_t uart_tx_mutex;
static bsp_uart_log_stats_t uart_log_stats;

/**
 * @brief 将 BSP UART 端口枚举映射到本模块允许使用的 HAL handle。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param port 待解析的 BSP UART 端口枚举。
 * @return BSP_UART_USART1 返回 huart1 地址；USART2、USART6 或非法值返回 NULL。
 * 调用方式：由本文件初始化、阻塞收发和 USART1 日志路径在访问 HAL handle 前调用。
 * 线程约束：纯指针映射，不阻塞、不使用 mutex；函数本身可在 ISR 调用栈执行。
 *           返回 handle 的发送所有权仍由 USART1 BSP mutex 管理；USART2 和 USART6 不归本模块。
 */
static UART_HandleTypeDef *uart_handle(bsp_uart_port_t port)
{
    return port == BSP_UART_USART1 ? &huart1 : NULL;
}

/**
 * @brief 将 HAL UART 调用状态收敛为 BSP 通用状态。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param status 待转换的 HAL_StatusTypeDef 返回值。
 * @return HAL_OK 映射为 BSP_STATUS_OK，HAL_TIMEOUT 映射为 BSP_STATUS_TIMEOUT，其余状态映射为 BSP_STATUS_ERROR。
 * 调用方式：仅由本文件在 USART1 阻塞 HAL 收发结束后转换结果；HAL_BUSY 的单独统计由发送助手完成。
 * 线程约束：纯值转换，不阻塞、不使用 mutex；函数本身可在 ISR 调用栈执行。
 *           USART1 所有权和 HAL 阻塞约束由外层接口负责。
 */
static bsp_status_t uart_map_hal_status(HAL_StatusTypeDef status)
{
    if (status == HAL_OK) {
        return BSP_STATUS_OK;
    }
    if (status == HAL_TIMEOUT) {
        return BSP_STATUS_TIMEOUT;
    }
    return BSP_STATUS_ERROR;
}

/**
 * @brief 按需创建并保存 USART1 阻塞发送使用的单例 FreeRTOS mutex。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * 传入参数：无。
 * @return mutex 已存在或创建成功返回 BSP_STATUS_OK；FreeRTOS heap 分配失败返回 BSP_STATUS_ERROR。
 * 调用方式：仅由 bsp_uart_init() 在 huart1 实例、波特率和 HAL READY 状态校验通过后调用；成功后的重复调用幂等。
 * 线程约束：可能执行动态内存分配，但不会等待 mutex。
 *           全局句柄创建过程本身无锁，只允许启动任务串行调用；禁止 ISR 或并发初始化。
 */
static bsp_status_t uart_create_tx_mutex(void)
{
    if (uart_tx_mutex == NULL) {
        uart_tx_mutex = xSemaphoreCreateMutex();
        if (uart_tx_mutex == NULL) {
            return BSP_STATUS_ERROR;
        }
    }
    return BSP_STATUS_OK;
}

/**
 * @brief 在临界区内累计 USART1 文本日志的成功、失败和忙统计。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param status 本次 USART1 文本发送的 BSP 状态。
 * @param busy 非零表示失败归因于 mutex/handle/HAL busy 路径；成功时忽略该标志。
 * @return 无；32 位累计计数自然回绕，不报告溢出或持久化失败。
 * 调用方式：仅由 uart_log_write_usart1() 在空文本拒绝或一次阻塞发送结束后调用。
 * 线程约束：使用 taskENTER_CRITICAL() 而非 mutex，不发生等待式阻塞。
 *           统计对象归本模块所有；禁止 ISR 调用，任务间更新由临界区串行化。
 */
static void uart_record_log_result(bsp_status_t status, uint8_t busy)
{
    taskENTER_CRITICAL();
    if (status == BSP_STATUS_OK) {
        ++uart_log_stats.tx_count;
    } else {
        ++uart_log_stats.tx_fail;
        if (busy != 0U) {
            ++uart_log_stats.tx_busy;
        }
    }
    taskEXIT_CRITICAL();
}

/**
 * @brief 在共享发送 mutex 保护下，以总超时预算执行一次 USART1 HAL 阻塞发送。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param handle 调用方已校验的 USART1 HAL handle，必须非 NULL 且调用期间保持有效。
 * @param data 非 NULL、至少包含 size 字节的只读发送缓冲；函数返回后不保留所有权。
 * @param size 由调用方保证位于 1..UINT16_MAX，函数内部会转换为 uint16_t。
 * @param timeout_ms 等待 mutex 与 HAL_UART_Transmit() 共用的总预算，单位 ms；0 表示两阶段均不等待。
 * @param[out] busy 可为 NULL；非 NULL 时先清零，锁超时、预算耗尽、handle 非 READY 或 HAL_BUSY 时置 1。
 * @return OK、NOT_READY、TIMEOUT，或其他 HAL 失败映射的 ERROR；失败不证明硬件未发送任何字节。
 * 调用方式：由 bsp_uart_transmit() 和 USART1 日志助手在完成端口、指针和长度校验后调用。
 * 线程约束：仅限任务上下文；先等待非递归 uart_tx_mutex，再阻塞执行 HAL。
 *           USART1 TX 所有权在返回前释放；禁止 ISR，也不得在已持同一 mutex 时递归调用。
 */
static bsp_status_t uart_transmit_locked(UART_HandleTypeDef *handle,
                                         const uint8_t *data, size_t size,
                                         uint32_t timeout_ms, uint8_t *busy)
{
    const uint32_t start_ms = HAL_GetTick();
    uint32_t elapsed_ms;
    uint32_t remaining_ms;
    HAL_StatusTypeDef hal_status;

    if (busy != NULL) {
        *busy = 0U;
    }
    if (uart_tx_mutex == NULL) {
        return BSP_STATUS_NOT_READY;
    }
    if (xSemaphoreTake(uart_tx_mutex, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        if (busy != NULL) {
            *busy = 1U;
        }
        return BSP_STATUS_TIMEOUT;
    }

    elapsed_ms = HAL_GetTick() - start_ms;
    if (timeout_ms != 0U && elapsed_ms >= timeout_ms) {
        (void)xSemaphoreGive(uart_tx_mutex);
        if (busy != NULL) {
            *busy = 1U;
        }
        return BSP_STATUS_TIMEOUT;
    }
    if (HAL_UART_GetState(handle) != HAL_UART_STATE_READY) {
        (void)xSemaphoreGive(uart_tx_mutex);
        if (busy != NULL) {
            *busy = 1U;
        }
        return BSP_STATUS_NOT_READY;
    }

    remaining_ms = timeout_ms == 0U ? 0U : timeout_ms - elapsed_ms;
    hal_status = HAL_UART_Transmit(handle, (uint8_t *)data, (uint16_t)size,
                                   remaining_ms);
    (void)xSemaphoreGive(uart_tx_mutex);
    if (hal_status == HAL_BUSY && busy != NULL) {
        *busy = 1U;
    }
    return uart_map_hal_status(hal_status);
}

/** 初始化指定 UART 端口和共享日志锁。 */
bsp_status_t bsp_uart_init(bsp_uart_port_t port, uint32_t baud_rate)
{
    UART_HandleTypeDef *handle = uart_handle(port);
    if (handle == NULL) {
        return BSP_STATUS_UNSUPPORTED;
    }
    if (handle->Instance != USART1 || handle->Init.BaudRate != baud_rate ||
        HAL_UART_GetState(handle) != HAL_UART_STATE_READY) {
        return BSP_STATUS_NOT_READY;
    }
    return uart_create_tx_mutex();
}

/** 任务上下文阻塞发送。 */
bsp_status_t bsp_uart_transmit(bsp_uart_port_t port, const uint8_t *data,
                               size_t size, uint32_t timeout_ms)
{
    UART_HandleTypeDef *handle = uart_handle(port);
    if (handle == NULL) {
        return BSP_STATUS_UNSUPPORTED;
    }
    if (data == NULL || size == 0U || size > UINT16_MAX) {
        return BSP_STATUS_INVALID_ARG;
    }
    return uart_transmit_locked(handle, data, size, timeout_ms, NULL);
}

/** 任务上下文阻塞接收。 */
bsp_status_t bsp_uart_receive(bsp_uart_port_t port, uint8_t *data,
                              size_t size, uint32_t timeout_ms)
{
    UART_HandleTypeDef *handle = uart_handle(port);
    if (handle == NULL) {
        return BSP_STATUS_UNSUPPORTED;
    }
    if (data == NULL || size == 0U || size > UINT16_MAX) {
        return BSP_STATUS_INVALID_ARG;
    }
    if (HAL_UART_GetState(handle) != HAL_UART_STATE_READY) {
        return BSP_STATUS_NOT_READY;
    }
    return uart_map_hal_status(HAL_UART_Receive(handle, data, (uint16_t)size, timeout_ms));
}

/** DMA 发送预留入口；当前实现不访问参数并固定返回不支持。 */
bsp_status_t bsp_uart_transmit_dma(bsp_uart_port_t port, const uint8_t *data, size_t size)
{
    (void)port;
    (void)data;
    (void)size;
    /* DMA streams are not assigned to USART1/USART6 in the current IOC. */
    return BSP_STATUS_UNSUPPORTED;
}

/** DMA 接收预留入口；当前实现不访问参数并固定返回不支持。 */
bsp_status_t bsp_uart_receive_dma(bsp_uart_port_t port, uint8_t *data, size_t size)
{
    (void)port;
    (void)data;
    (void)size;
    return BSP_STATUS_UNSUPPORTED;
}

/**
 * @brief 校验文本后通过 USART1 mutex 路径阻塞发送，并更新文本日志统计。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param text 非 NULL、NUL 结尾且长度为 1..UINT16_MAX 的字符串；函数不保留指针，也不限制 strlen() 扫描范围。
 * @param timeout_ms 等待 uart_tx_mutex 与 HAL 发送共用的总预算，单位 ms。
 * @return 透传 uart_transmit_locked() 状态；NULL 或空文本返回 BSP_STATUS_INVALID_ARG。
 *         NULL 不计入统计；空文本累计一次失败。
 * 调用方式：仅由 bsp_uart_log_write_level() 调用，USART1 BSP mutex 必须已通过 bsp_uart_init() 创建。
 * 线程约束：仅限任务上下文，可能阻塞等待 mutex 和 HAL。
 *           USART1 文本发送与统计归本模块所有；禁止 ISR、实时控制环或同 mutex 递归调用。
 */
static bsp_status_t uart_log_write_usart1(const char *text, uint32_t timeout_ms)
{
    size_t length;
    bsp_status_t status;
    uint8_t busy = 0U;

    if (text == NULL) {
        return BSP_STATUS_INVALID_ARG;
    }
    length = strlen(text);
    if (length == 0U) {
        uart_record_log_result(BSP_STATUS_INVALID_ARG, 0U);
        return BSP_STATUS_INVALID_ARG;
    }
    status = uart_transmit_locked(uart_handle(BSP_UART_USART1),
                                  (const uint8_t *)text, length,
                                  timeout_ms, &busy);
    uart_record_log_result(status, busy);
    return status;
}

/**
 * @brief 将文本封装为带来源、级别和 HAL tick 的 SRPv4 LOG payload 并提交给 S3 服务。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param level BSP_UART_LOG_LEVEL_DEBUG..ERROR；超出范围时静默返回。
 * @param text NUL 结尾字符串；NULL 时静默返回，超过 96 字节时截断，允许空字符串。
 * @return 无；s3_service_send_log() 结果被忽略，调用完成不表示 UART 字节已发送或 S3 已收到。
 * 调用方式：由公共日志入口及 SPI 首次诊断路径构造低频日志；栈上 payload 仅在同步提交调用期间借用。
 * 线程约束：使用约 104 字节局部缓冲，并可能阻塞等待 s3_service 的 link/UART mutex。
 *           日志路径仅供任务上下文使用；禁止 ISR、实时控制环或在持有非递归服务锁时调用。
 */
static void uart_log_write_usart2(uint8_t level, const char *text)
{
    uint8_t payload[8U + BSP_UART_LOG_TEXT_MAX];
    size_t text_length;
    const uint32_t timestamp = HAL_GetTick();

    if (text == NULL || level > BSP_UART_LOG_LEVEL_ERROR) {
        return;
    }

    text_length = strlen(text);
    if (text_length > BSP_UART_LOG_TEXT_MAX) {
        text_length = BSP_UART_LOG_TEXT_MAX;
    }

    payload[0] = 0U;
    payload[1] = level;
    payload[2] = (uint8_t)(timestamp & 0xFFU);
    payload[3] = (uint8_t)((timestamp >> 8U) & 0xFFU);
    payload[4] = (uint8_t)((timestamp >> 16U) & 0xFFU);
    payload[5] = (uint8_t)((timestamp >> 24U) & 0xFFU);
    payload[6] = (uint8_t)(text_length & 0xFFU);
    payload[7] = (uint8_t)(text_length >> 8U);
    if (text_length != 0U) {
        memcpy(&payload[8], text, text_length);
    }

    (void)s3_service_send_log(payload, (uint8_t)(8U + text_length));
}

/** 发送带级别日志文本。 */
bsp_status_t bsp_uart_log_write_level(uint8_t level, const char *text,
                                      uint32_t timeout_ms)
{
    bsp_status_t status;

    if (level > BSP_UART_LOG_LEVEL_ERROR) {
        return BSP_STATUS_INVALID_ARG;
    }

    status = uart_log_write_usart1(text, timeout_ms);
    if (status != BSP_STATUS_INVALID_ARG) {
        uart_log_write_usart2(level, text);
    }
    return status;
}

/** 编码并通过 USART2 发送 SRP LOG 帧。 */
bsp_status_t bsp_uart_log_write_link_level(uint8_t level, const char *text)
{
    if (text == NULL || level > BSP_UART_LOG_LEVEL_ERROR) {
        return BSP_STATUS_INVALID_ARG;
    }
    uart_log_write_usart2(level, text);
    return BSP_STATUS_OK;
}

/** 通过 USART1 发送普通文本日志。 */
bsp_status_t bsp_uart_log_write(const char *text, uint32_t timeout_ms)
{
    return bsp_uart_log_write_level(BSP_UART_LOG_LEVEL_INFO, text, timeout_ms);
}

/** 复制日志发送统计。 */
bsp_status_t bsp_uart_get_log_stats(bsp_uart_log_stats_t *stats)
{
    if (stats == NULL) {
        return BSP_STATUS_INVALID_ARG;
    }

    taskENTER_CRITICAL();
    *stats = uart_log_stats;
    taskEXIT_CRITICAL();
    return BSP_STATUS_OK;
}

/**
 * @brief 实现 C 库标准输出的单字符底层钩子，将低 8 位通过 USART1 BSP 发送。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（外部实现入口契约补充）。
 * @param ch 待输出字符的 int 表示；实际发送值为转换后的 uint8_t 低 8 位。
 * @return USART1 成功发送一个字节时返回原 ch；初始化、锁、超时或 HAL 失败时返回 EOF。
 * 调用方式：由 CM7 syscalls.c 的弱 _write() 和 Picolibc putc 重定向隐式调用。
 *           使用 printf/putchar 前必须先成功执行 bsp_uart_init()。
 * 线程约束：每个字符最多阻塞 100 ms，并获取非递归 uart_tx_mutex；USART1 标准输出归本 BSP 所有。
 *           禁止 ISR、故障停机路径、调度器或 mutex 就绪前调用，也不得在已持同一锁时调用。
 */
int __io_putchar(int ch)
{
    const uint8_t byte = (uint8_t)ch;

    return bsp_uart_transmit(BSP_UART_USART1, &byte, 1U, 100U) == BSP_STATUS_OK
               ? ch
               : EOF;
}
