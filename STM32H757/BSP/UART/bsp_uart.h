#ifndef BSP_UART_H
#define BSP_UART_H

#include <stddef.h>
#include <stdint.h>
#include "bsp_status.h"

/*
 * STM32 UART BSP 公共接口。
 * 创建人：待确认（当前维护人：Zhiqin）。
 * USART2 归 S3 SRP 链路、USART6 归 MotorBoard transport；二者只为历史枚举兼容
 * 保留，当前 bsp_uart_* 数据接口仅支持 CubeMX 已配置的 USART1 日志端口。
 * 调用方不得借此 BSP 绕过 UART_Link、s3_service 或 MotorBoard transport。
 */

#ifdef __cplusplus
extern "C" {
#endif

/** UART BSP 逻辑端口；只有 USART1 数据接口由本模块实际支持。 */
typedef enum {
    BSP_UART_USART1 = 0, /**< 调试/文本日志端口。 */
    BSP_UART_USART6, /**< MotorBoard 专用端口，仅作枚举兼容。 */
    BSP_UART_USART2 /**< S3 SRP 专用端口，仅作枚举兼容。 */
} bsp_uart_port_t;

/** USART1 文本日志发送累计统计。 */
typedef struct {
    uint32_t tx_count; /**< HAL 完整发送成功次数。 */
    uint32_t tx_fail; /**< HAL 超时以外的发送失败次数。 */
    uint32_t tx_busy; /**< mutex 等待超时或 HAL_TIMEOUT 次数。 */
} bsp_uart_log_stats_t;

/* LOG payload 依次包含 source、level、timestamp、文本长度和 UTF-8 文本。 */
#define BSP_UART_LOG_LEVEL_DEBUG UINT8_C(0)
#define BSP_UART_LOG_LEVEL_INFO  UINT8_C(1)
#define BSP_UART_LOG_LEVEL_WARN  UINT8_C(2)
#define BSP_UART_LOG_LEVEL_ERROR UINT8_C(3)
#define BSP_UART_LOG_TEXT_MAX    UINT16_C(96)

/**
 * @brief  校验 CubeMX 已初始化的 USART1 配置，并创建共享发送 mutex。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（接口契约补充）。
 * @param  port 当前仅支持 BSP_UART_USART1；USART2/USART6 返回 UNSUPPORTED。
 * @param  baud_rate 期望的既有波特率，单位 bit/s；函数不会重配硬件。
 * @return OK、UNSUPPORTED、NOT_READY，或 mutex 分配失败时的 ERROR；重复成功调用幂等。
 * 调用方式：MX_USART1_UART_Init() 成功后、首次文本日志前调用。
 * 线程约束：会按需分配 FreeRTOS mutex，只允许调度器可用后的任务上下文调用，禁止 ISR 调用。
 */
bsp_status_t bsp_uart_init(bsp_uart_port_t port, uint32_t baud_rate);
/**
 * @brief  通过 USART1 执行带 mutex 的阻塞发送。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（接口契约补充）。
 * @param  port 当前仅支持 BSP_UART_USART1。
 * @param  data 非 NULL 的只读缓冲；函数返回后不保留指针。
 * @param  size 发送字节数，范围 1..UINT16_MAX。
 * @param  timeout_ms 等待 mutex 加 HAL_UART_Transmit 的总预算，单位 ms；0 表示不等待。
 * @return OK、UNSUPPORTED、INVALID_ARG、NOT_READY、TIMEOUT 或 ERROR。
 * 调用方式：先成功调用 bsp_uart_init()；只用于 USART1 文本/诊断，不发送 STM/S3 或 MotorBoard 协议。
 * 线程约束：任务上下文阻塞调用；内部串行化发送，禁止 ISR 调用和在持有相关服务锁时递归调用。
 */
bsp_status_t bsp_uart_transmit(bsp_uart_port_t port, const uint8_t *data,
                               size_t size, uint32_t timeout_ms);
/**
 * @brief  通过 USART1 执行阻塞接收。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（接口契约补充）。
 * @param  port 当前仅支持 BSP_UART_USART1。
 * @param[out] data 非 NULL、至少可写 size 字节的缓冲。
 * @param  size 接收字节数，范围 1..UINT16_MAX。
 * @param  timeout_ms HAL_UART_Receive() 超时，单位 ms。
 * @return OK、UNSUPPORTED、INVALID_ARG、NOT_READY、TIMEOUT 或 ERROR；非 OK 时输出无效。
 * 调用方式：先成功初始化；当前 USART1 主要用于日志，协议 RX 不应通过此接口接入。
 * 线程约束：任务上下文阻塞调用；接收没有独立 mutex，调用方负责单消费者和与发送状态协调，禁止 ISR 调用。
 */
bsp_status_t bsp_uart_receive(bsp_uart_port_t port, uint8_t *data,
                              size_t size, uint32_t timeout_ms);
/**
 * @brief  USART BSP 的 DMA 发送预留入口。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（接口契约补充）。
 * @param  port 预留端口参数。
 * @param  data 预留输入缓冲参数；当前实现不读取。
 * @param  size 预留长度参数；当前实现不读取。
 * @return 当前实现固定返回 BSP_STATUS_UNSUPPORTED，不会启动 DMA，也不会持有 data。
 * 调用方式：不得以返回前缓冲生命周期推断 DMA 已开始；USART2 DMA 使用 UART_Link 专用接口。
 * 线程约束：不阻塞；当前无硬件副作用，但仍不作为 ISR API 使用。
 */
bsp_status_t bsp_uart_transmit_dma(bsp_uart_port_t port, const uint8_t *data, size_t size);
/**
 * @brief  USART BSP 的 DMA 接收预留入口。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（接口契约补充）。
 * @param  port 预留端口参数。
 * @param[out] data 预留输出缓冲参数；当前实现不会写入。
 * @param  size 预留长度参数；当前实现不读取。
 * @return 当前实现固定返回 BSP_STATUS_UNSUPPORTED，不会启动 DMA。
 * 调用方式：USART2 ReceiveToIdle DMA 必须通过 UART_Link 所有权路径配置，不能调用本接口替代。
 * 线程约束：不阻塞；当前无硬件副作用，但仍不作为 ISR API 使用。
 */
bsp_status_t bsp_uart_receive_dma(bsp_uart_port_t port, uint8_t *data, size_t size);
/**
 * @brief  发送 INFO 级文本日志，并尽力镜像为一条 S3 SRPv4 LOG 帧。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（接口契约补充）。
 * @param  text 非 NULL、非空、NUL 结尾字符串；S3 镜像最多复制 BSP_UART_LOG_TEXT_MAX 字节。
 * @param  timeout_ms USART1 mutex 加阻塞发送的总预算，单位 ms。
 * @return 仅反映 USART1 文本发送状态；S3 镜像结果被忽略，不能由 OK 推断链路收到日志。
 * 调用方式：初始化 USART1 BSP 后用于低频诊断；日志失败不得改变控制路径或无限重试。
 * 线程约束：任务上下文，可能等待 mutex/HAL 并进入 s3_service；禁止 ISR、实时控制环和服务锁内递归调用。
 */
bsp_status_t bsp_uart_log_write(const char *text, uint32_t timeout_ms);
/**
 * @brief  发送指定级别的 USART1 文本，并尽力镜像为 S3 SRPv4 LOG 帧。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（接口契约补充）。
 * @param  level DEBUG..ERROR 四个宏之一。
 * @param  text 非 NULL、非空、NUL 结尾字符串；S3 文本超长时截断到 96 字节。
 * @param  timeout_ms USART1 mutex 加阻塞发送的总预算，单位 ms。
 * @return USART1 路径状态；level/text 无效返回 INVALID_ARG，S3 镜像失败不反映在返回值中。
 * 调用方式：与 bsp_uart_log_write() 相同，level 只进入 SRP payload，USART1 仍发送原始文本。
 * 线程约束：任务上下文且可能阻塞；禁止 ISR、控制环和服务锁内递归调用。
 */
bsp_status_t bsp_uart_log_write_level(uint8_t level, const char *text,
                                      uint32_t timeout_ms);
/**
 * @brief  将文本编码为 SRPv4 LOG payload，并通过 s3_service 提交到 USART2 链路。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（接口契约补充）。
 * @param  level DEBUG..ERROR 四个宏之一。
 * @param  text 非 NULL、NUL 结尾字符串；最多复制 96 字节，允许空字符串。
 * @return 参数有效即返回 BSP_STATUS_OK；当前实现忽略 s3_service_send_log() 结果，OK 不表示帧已发送或收到。
 * 调用方式：仅用于服务层可用后的有界诊断；不发送 USART1 文本，也不得绕过业务安全门承载控制数据。
 * 线程约束：任务上下文；内部可能获取服务/UART 锁并阻塞，禁止 ISR 和服务锁内递归调用。
 */
bsp_status_t bsp_uart_log_write_link_level(uint8_t level, const char *text);
/**
 * @brief  原子复制 USART1 文本日志发送统计。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（接口契约补充）。
 * @param[out] stats 必须非 NULL；成功时写入累计成功、失败和忙/锁超时次数。
 * @return BSP_STATUS_OK；stats 为 NULL 时返回 BSP_STATUS_INVALID_ARG。
 * 调用方式：健康监测任务低频读取；计数不包含纯 link_level 的 S3 LOG 结果。
 * 线程约束：使用 FreeRTOS critical section 复制；不阻塞，但不应从高优先级 ISR 调用。
 */
bsp_status_t bsp_uart_get_log_stats(bsp_uart_log_stats_t *stats);

/* 以下短名称仅作兼容包装，完整契约继承对应 bsp_* API。 */
/** @copydoc bsp_uart_init
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（兼容包装契约补充）。 */
static inline bsp_status_t uart_init(bsp_uart_port_t port, uint32_t baud_rate)
{
    return bsp_uart_init(port, baud_rate);
}
/** @copydoc bsp_uart_transmit
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（兼容包装契约补充）。 */
static inline bsp_status_t uart_transmit(bsp_uart_port_t port, const uint8_t *data,
                                         size_t size, uint32_t timeout_ms)
{
    return bsp_uart_transmit(port, data, size, timeout_ms);
}
/** @copydoc bsp_uart_receive
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（兼容包装契约补充）。 */
static inline bsp_status_t uart_receive(bsp_uart_port_t port, uint8_t *data,
                                        size_t size, uint32_t timeout_ms)
{
    return bsp_uart_receive(port, data, size, timeout_ms);
}
/** @copydoc bsp_uart_transmit_dma
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（兼容包装契约补充）。 */
static inline bsp_status_t uart_transmit_dma(bsp_uart_port_t port, const uint8_t *data,
                                             size_t size)
{
    return bsp_uart_transmit_dma(port, data, size);
}
/** @copydoc bsp_uart_receive_dma
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（兼容包装契约补充）。 */
static inline bsp_status_t uart_receive_dma(bsp_uart_port_t port, uint8_t *data, size_t size)
{
    return bsp_uart_receive_dma(port, data, size);
}
/** @copydoc bsp_uart_log_write
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（兼容包装契约补充）。 */
static inline bsp_status_t uart_log_write(const char *text, uint32_t timeout_ms)
{
    return bsp_uart_log_write(text, timeout_ms);
}

#ifdef __cplusplus
}
#endif

#endif /* BSP_UART_H */
