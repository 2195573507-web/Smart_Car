#ifndef UART_LINK_H
#define UART_LINK_H

#include <stddef.h>
#include <stdint.h>

#include "srp_def.h"
#include "stm32h7xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * STM32H757 CM7 USART2 链路适配层。
 * 创建人：待确认（当前维护人：Zhiqin）。
 * 说明：本层负责 HAL/DMA、有限环形缓冲和中断入口；SRP 编解码、ACK、同步和
 *       运动安全由上层服务处理。DMA 缓冲的 cache 一致性由实现内部维护。
 */

#define UART_LINK_USART USART2
#define UART_LINK_BAUD_RATE UINT32_C(921600)
#define UART_LINK_RX_RING_SIZE UINT16_C(2048)
#define UART_LINK_RX_DMA_SIZE UINT16_C(512)

#define UART_TX_OK HAL_OK
#define UART_TX_FAIL HAL_ERROR

/** STM USART2/DMA 链路累计计数与最近寄存器快照；仅用于诊断。 */
typedef struct {
    uint32_t uart_tx_count; /**< 阻塞 TX 完整成功次数。 */
    uint32_t uart_tx_bytes; /**< 阻塞 TX 完整成功字节数。 */
    uint32_t uart_tx_timeout; /**< HAL 或 TX mutex 超时次数。 */
    uint32_t uart_rx_bytes; /**< HAL RX event 搬运到软件 ring 的字节数。 */
    uint32_t uart_rx_overflow; /**< 软件 ring 发生覆盖的事件次数。 */
    uint32_t uart_rx_drop; /**< ring 满时被覆盖的旧字节总数。 */
    uint32_t uart_hal_error; /**< UART/DMA/HAL 配置和运行错误累计值。 */
    uint32_t uart_tx_queue_drop; /**< TX mutex 在 deadline 内不可得次数。 */
    uint32_t uart_tx_preemptions; /**< 历史 TX 抢占统计；阻塞兼容路径通常为 0。 */
    uint32_t uart_rx_events; /**< ReceiveToIdle 回调次数。 */
    uint32_t uart_rx_event_bytes; /**< 所有 RX event 有效字节合计。 */
    uint32_t uart_rx_rearms; /**< 尝试武装 ReceiveToIdle DMA 的次数。 */
    uint32_t uart_rx_rearm_failures; /**< DMA 重装失败次数。 */
    uint32_t uart_tx_dma_starts; /**< 历史 DMA TX 启动次数；当前阻塞路径通常为 0。 */
    uint32_t uart_tx_dma_errors; /**< 历史 DMA TX 错误次数。 */
    uint32_t uart_tx_gstate; /**< 最近 TX 前保存的 HAL gState。 */
    uint32_t uart_tx_error_code; /**< 最近 TX 后保存的 HAL ErrorCode。 */
    uint32_t uart_tx_busy_recoveries; /**< 发送前发现 stale busy 并 abort 的次数。 */
    uint32_t uart_usart_errors; /**< HAL USART error callback 次数。 */
    uint32_t uart_last_error_code; /**< 最近一次 USART HAL 错误码。 */
    uint32_t uart_dma_rx_irqs; /**< DMA1 Stream0 IRQ 入口累计次数。 */
    uint32_t uart_dma_tx_irqs; /**< TX DMA 兼容 IRQ 入口累计次数。 */
    uint32_t uart_usart_irqs; /**< USART2 IRQ 入口累计次数。 */
    uint32_t uart_rx_callback_rejects; /**< handle、active 或长度不符的 RX 回调次数。 */
    uint32_t uart_rx_dma_error_code; /**< 最近采样的 RX DMA ErrorCode。 */
    uint32_t uart_rx_dmamux_request; /**< 最近采样的 RX DMAMUX request ID。 */
    uint32_t uart_tx_dmamux_request; /**< TX DMA request 快照；阻塞路径固定为 0。 */
    uint32_t uart_rx_dma_cr; /**< 最近采样的 DMA1 Stream0 CR。 */
    uint32_t uart_rx_gpio_state; /**< PA3 mode/pull/AF/电平压缩诊断位。 */
    uint16_t uart_rx_dma_ndtr; /**< 最近采样的 RX DMA NDTR。 */
    uint8_t uart_rx_active; /**< 软件记录的 ReceiveToIdle active 标志。 */
    uint16_t rx_buffered; /**< 软件 RX ring 当前字节数。 */
    uint16_t rx_buffer_capacity; /**< 软件 RX ring 固定容量。 */
} uart_link_stats_t;

/**
 * @brief 初始化 USART2、DMA 接收和链路内部队列。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * 传入参数：无。
 * 返回值：无；底层失败通过 uart_link_is_ready() 和统计快照暴露。
 * 调用方式：CM7 启动阶段、任务创建前调用一次；不得从 ISR 调用。
 * 线程约束：非幂等；初始化期间不得与收发/恢复接口并发。
 */
void uart_link_init(void);

/**
 * @brief 查询 USART2/DMA 接收路径是否已准备好。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * 传入参数：无。
 * 返回值：1 表示底层链路 ready，0 表示未初始化或配置/重装失败；不代表 SRP 已同步。
 * 调用方式：任务上下文可读取；仅用于底层健康判断。
 * 线程约束：读取 volatile 状态，不阻塞。
 */
uint8_t uart_link_is_ready(void);

/**
 * @brief 读取最近一次成功搬运 RX 字节的 HAL 毫秒 tick。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * 传入参数：无。
 * 返回值：最近 RX tick；0 表示尚未收到或状态已重置。
 * 调用方式：服务任务用于 freshness 判断；使用无符号差值处理 tick 回绕。
 * 线程约束：短临界区读取，禁止从高优先级 ISR 调用。
 */
uint32_t uart_link_get_last_rx_time(void);

/**
 * @brief 发送一段 SRP 线缆字节。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * 传入参数：data 为只读完整帧，length 为实际字节数。
 * 返回值：HAL_OK 表示阻塞式物理发送已完成；HAL_TIMEOUT 可能来自 TX 互斥量或
 *          HAL 发送超时，其他值表示参数、UART 状态或硬件错误。
 * 调用方式：仅任务上下文；函数先等待 TX mutex，再调用 HAL_UART_Transmit()，
 *           两段等待各自最多 UART_LINK_TX_TIMEOUT_MS。data 只需保持到函数返回。
 * 线程约束：同一 USART2 TX 由内部 mutex 串行化；禁止从 DMA/USART ISR 调用。
 */
HAL_StatusTypeDef uart_link_send(const uint8_t *data, uint16_t length);

/**
 * @brief 尝试中止当前物理 TX，使后续同步响应获得干净的发送状态。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * 传入参数：无。
 * 返回值：无；TX mutex 当前不可用时保持现状并立即返回。
 * 调用方式：仅任务上下文；函数以零等待获取 TX mutex，不维护或清空软件发送队列。
 * 线程约束：禁止从 ISR 调用。
 */
void uart_link_flush_tx(void);

/**
 * @brief 无阻塞读取接收环形缓冲。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * 传入参数：data/capacity 为输出空间。
 * 返回值：实际复制字节数；0 表示暂无数据。
 * 调用方式：由 SRP 服务任务调用；输出副本不再依赖 DMA 缓冲。
 * 线程约束：使用短临界区保护 ring；禁止从 ISR 调用。
 */
size_t uart_link_read(uint8_t *data, size_t capacity);

/**
 * @brief 复制 UART/DMA 诊断统计快照。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * 传入参数：stats 为可写输出；NULL 时直接返回。
 * 返回值：无；快照是 RAM 计数，不证明物理链路正确。
 * 调用方式：低频诊断任务调用。
 * 线程约束：使用短临界区复制，不从 ISR 调用。
 */
void uart_link_get_stats(uart_link_stats_t *stats);

/**
 * @brief 清理 UART/DMA 接收状态并尝试重新启动 ReceiveToIdle DMA。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * 传入参数：无。返回值：无；结果通过 ready/rearm 统计观察。
 * 调用方式：仅 UART worker 或受控服务恢复路径调用；恢复后必须重新建立 SRP 同步。
 * 线程约束：可能调用 HAL abort/init，禁止从 ISR 和控制闭环直接调用。
 */
void uart_link_recover(void);

/**
 * @brief 切换 USART2 波特率并重启接收路径。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * 传入参数：baud_rate 仅接受 SRP_BAUDRATE_DEFAULT 或 SRP_BAUDRATE_DEBUG。
 * 返回值：HAL_OK 表示切换并恢复成功；否则保持不可用/原状态并由服务层降级。
 * 调用方式：双方完成 TLV 协商、停止当前收发后由 S3 服务调用。
 * 线程约束：阻塞式 HAL 重配置，禁止从 ISR 调用。
 */
HAL_StatusTypeDef uart_link_set_baud_rate(uint32_t baud_rate);

/**
 * @brief 转发 USART2 RX DMA IRQ 到 HAL。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * 传入参数：无。
 * @return 无。
 * 调用方式：只由 DMA1_Stream0_IRQHandler 调用。
 * 线程约束：ISR 上下文，不得由任务直接调用。
 */
void uart_link_handle_dma_rx_irq(void);

/**
 * @brief USART2 TX DMA IRQ 兼容入口；当前阻塞 TX 路径不使用 TX DMA。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * 传入参数：无。
 * @return 无；当前实现为空操作。
 * 调用方式：仅保留给对应 ISR/兼容代码，任务不得调用。
 * 线程约束：仅对应 DMA ISR 上下文；不能用于判断 TX 已完成。
 */
void uart_link_handle_dma_tx_irq(void);

/**
 * @brief 转发 USART2 IRQ 到 HAL 并更新中断诊断快照。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * 传入参数：无。
 * @return 无。
 * 调用方式：只由 USART2_IRQHandler 调用。
 * 线程约束：ISR 上下文。
 */
void uart_link_handle_usart_irq(void);

/**
 * @brief UART 链路 FreeRTOS 任务入口。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * 传入参数：argument 预留，当前实现不解引用。
 * 返回值：不返回；任务退出前必须释放其资源。
 * 调用方式：由 uart_link_task_start() 创建，禁止手动在 ISR 中调用。
 * 线程约束：唯一 worker；每次最多等待通知 1 ms，恢复路径可能调用阻塞 HAL。
 */
void uart_link_task(void *argument);

/**
 * @brief 创建并启动唯一 UART 链路 worker。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * 传入参数：无。返回值：无；任务创建失败通过日志和 handle 状态暴露。
 * 调用方式：uart_link_init() 后由 CM7 启动路径调用；重复调用不会创建第二个任务。
 * 线程约束：仅启动/恢复管理任务调用，禁止从 ISR 调用。
 */
void uart_link_task_start(void);

#ifdef __cplusplus
}
#endif

#endif /* UART_LINK_H */
