#include "motor_board_transport_uart.h"

/* MotorBoard USART6 传输实现；创建人：待确认（当前维护人：Zhiqin）。 */

#include <stddef.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"

#include "log_service.h"

extern UART_HandleTypeDef huart6;

static volatile uint8_t s_ready;
static volatile uint8_t s_rx_ring[MB_TRANSPORT_RX_RING_SIZE];
static volatile uint8_t s_tx_ring[MB_TRANSPORT_TX_RING_SIZE];
static volatile uint16_t s_rx_head;
static volatile uint16_t s_rx_tail;
static volatile uint16_t s_rx_count;
static volatile uint16_t s_tx_head;
static volatile uint16_t s_tx_tail;
static volatile uint16_t s_tx_count;
static volatile uint32_t s_rx_bytes;
static volatile uint32_t s_rx_overflow;
static volatile uint32_t s_tx_bytes;
static volatile uint32_t s_tx_overflow;
static volatile uint32_t s_uart_errors;

/**
 * @brief 根据 USART ISR 快照清除 ORE/FE/NE/PE，并累计一次线路错误。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param instance USART 寄存器基址；允许 NULL，NULL 时不动作。
 * @param isr 调用点读取的 ISR 快照，用于判断是否存在任一接收错误。
 * @return 无；多个错误位同时出现只增加一次 s_uart_errors。
 * 调用方式：初始化、任务健康恢复和 USART6 ISR 在读取硬件状态后调用。
 * 线程约束：直接写 ICR；任务与 ISR 都可能调用，错误计数只作诊断快照，不保证跨上下文原子精确。
 */
static void transport_clear_error_flags(USART_TypeDef *instance,
                                        uint32_t isr)
{
    if (instance != NULL &&
        (isr & (USART_ISR_ORE | USART_ISR_FE | USART_ISR_NE |
                USART_ISR_PE)) != 0U) {
        instance->ICR = USART_ICR_ORECF | USART_ICR_FECF |
                        USART_ICR_NECF | USART_ICR_PECF;
        ++s_uart_errors;
    }
}

/**
 * @brief 直接设置 USART6 UE/RE/TE/RXNE/错误中断位并验证寄存器接收路径已武装。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param reason 只在失败日志中借用的零结尾原因；允许 NULL。
 * @return 必需 CR1 位回读全部置位时 true；handle 不是 USART6 或回读失败时 false。
 * 调用方式：初始化和任务上下文健康检查调用；本路径不启动 HAL receive transaction。
 * 线程约束：直接修改 USART6 CR1/CR3，失败时入日志队列；禁止 ISR 调用或与外设重配置并发。
 */
static bool transport_enable_register_rx(const char *reason)
{
    char line[SMARTCAR_LOG_MAX_PAYLOAD + 1U];
    USART_TypeDef *instance = huart6.Instance;
    const uint32_t required = USART_CR1_UE | USART_CR1_RE |
                              USART_CR1_TE | USART_CR1_RXNEIE_RXFNEIE;

    if (instance != USART6) {
        return false;
    }

    /* USART6 uses the register-level byte ring below; HAL receive transactions
     * never own the RXNE interrupt. */
    SET_BIT(instance->CR1, required);
    CLEAR_BIT(instance->CR3, USART_CR3_OVRDIS);
    SET_BIT(instance->CR1, USART_CR1_PEIE);
    SET_BIT(instance->CR3, USART_CR3_EIE);

    if ((instance->CR1 & required) != required) {
        (void)snprintf(line, sizeof(line),
                       "[MOTOR_BOARD] USART6 RX register arm failed reason=%s isr=%08lX cr1=%08lX",
                       reason == NULL ? "?" : reason,
                       (unsigned long)instance->ISR,
                       (unsigned long)instance->CR1);
        LOG_WARN(line);
        return false;
    }
    return true;
}

/**
 * @brief 把 ring 索引前进一步并在恰好到达容量时回绕为 0。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param index 当前索引，调用方必须保证小于 capacity。
 * @param capacity 非零 ring 容量。
 * @return 下一索引；本函数不处理 index 已越界或 capacity 为 0 的非法输入。
 * 调用方式：USART6 RX/TX ring 的 producer/consumer 在持有相应上下文保护时调用。
 * 线程约束：纯数值计算、可重入、不阻塞。
 */
static uint16_t ring_next(uint16_t index, uint16_t capacity)
{
    ++index;
    return index == capacity ? 0U : index;
}

/**
 * @brief 清除 USART TXE/TXFNF 中断使能位，停止空 ring 的发送中断。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param instance 有效 USART 寄存器基址；当前调用方固定传 USART6，不能为 NULL。
 * @return 无。
 * 调用方式：仅 transport_service_tx() 发现 TX ring 已空时调用。
 * 线程约束：USART6 ISR 上下文直接改 CR1，禁止普通任务手工调用。
 */
static void transport_disable_tx_irq(USART_TypeDef *instance)
{
    CLEAR_BIT(instance->CR1, USART_CR1_TXEIE_TXFNFIE);
}

/**
 * @brief 设置 USART TXE/TXFNF 中断使能位，启动已排队字节的中断发送。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param instance 有效 USART 寄存器基址；当前调用方固定传 USART6，不能为 NULL。
 * @return 无。
 * 调用方式：MB_Transport_Send() 完整复制命令并退出临界区后调用。
 * 线程约束：任务上下文直接改 CR1，和 ISR 关闭位形成 producer/consumer 交接；禁止其他 owner 调用。
 */
static void transport_enable_tx_irq(USART_TypeDef *instance)
{
    SET_BIT(instance->CR1, USART_CR1_TXEIE_TXFNFIE);
}

/**
 * @brief 把一个 USART6 接收字节压入 RX ring，满时丢弃新字节并累计 overflow。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param byte 从 RDR 读取的低 8 位数据。
 * @return 无；s_rx_bytes 统计所有硬件读取字节，s_rx_overflow 统计因满 ring 丢弃的新字节。
 * 调用方式：仅 MB_Transport_IRQHandler() 排空 RXNE 时调用。
 * 线程约束：USART6 ISR 是唯一 producer；任务消费者通过临界区读取，禁止任务直接调用。
 */
static void transport_push_rx_byte(uint8_t byte)
{
    ++s_rx_bytes;
    if (s_rx_count < MB_TRANSPORT_RX_RING_SIZE) {
        s_rx_ring[s_rx_head] = byte;
        s_rx_head = ring_next(s_rx_head, MB_TRANSPORT_RX_RING_SIZE);
        ++s_rx_count;
    } else {
        ++s_rx_overflow;
    }
}

/**
 * @brief 在临界区清零 RX/TX ring 的 head、tail 和 buffered count。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * 传入参数：无。
 * @return 无；不清累计字节、overflow 或 UART 错误统计。
 * 调用方式：仅 MB_Transport_Init() 在启用 USART6 IRQ 前调用。
 * 线程约束：任务上下文短临界区；运行期调用会丢弃待收/待发数据，因此不得并发重复初始化。
 */
static void transport_reset_buffers(void)
{
    taskENTER_CRITICAL();
    s_rx_head = 0U;
    s_rx_tail = 0U;
    s_rx_count = 0U;
    s_tx_head = 0U;
    s_tx_tail = 0U;
    s_tx_count = 0U;
    taskEXIT_CRITICAL();
}

/** 初始化 USART6 传输状态和 RX/TX 缓冲。 */
void MB_Transport_Init(void)
{
    s_ready = 0U;
    s_rx_bytes = 0U;
    s_rx_overflow = 0U;
    s_tx_bytes = 0U;
    s_tx_overflow = 0U;
    s_uart_errors = 0U;
    transport_reset_buffers();

    if (huart6.Instance != USART6) {
        LOG_WARN("[MOTOR_BOARD] USART6 HAL init not ready");
        return;
    }

    /* HAL_UART_Init() has configured the pins and baud rate. USART6 data
     * movement below does not read or write HAL receive-state bookkeeping. */
    transport_clear_error_flags(USART6, USART6->ISR);
    CLEAR_BIT(USART6->CR1, USART_CR1_TXEIE_TXFNFIE);
    __HAL_UART_ENABLE_IT(&huart6, UART_IT_RXNE);
    __HAL_UART_ENABLE_IT(&huart6, UART_IT_ERR);
    HAL_NVIC_SetPriority(USART6_IRQn, 5U, 0U);
    HAL_NVIC_EnableIRQ(USART6_IRQn);
    if (transport_enable_register_rx("init")) {
        s_ready = 1U;
    } else {
        LOG_WARN("[MOTOR_BOARD] USART6 RX not active after init");
    }
}

/** 返回 USART6 是否已准备好。 */
bool MB_Transport_IsReady(void)
{
    return s_ready != 0U;
}

/** 检查并恢复 RX 中断活动状态。 */
bool MB_Transport_Ensure_Rx_Active(void)
{
    if (huart6.Instance != USART6) {
        return false;
    }

    /* The register-level receiver remains authoritative even if HAL bookkeeping
     * is stale; it cannot disable RXNE or block motor-board TX. */
    transport_clear_error_flags(USART6, USART6->ISR);
    if (transport_enable_register_rx("health")) {
        s_ready = 1U;
        return true;
    }
    return false;
}

/** 任务上下文发送 MotorBoard 文本字节。 */
bool MB_Transport_Send(const uint8_t *data, uint16_t length)
{
    uint16_t index;

    if (!MB_Transport_Ensure_Rx_Active() || data == NULL || length == 0U ||
        length > MB_TRANSPORT_TX_RING_SIZE) {
        return false;
    }

    taskENTER_CRITICAL();
    if ((uint16_t)(MB_TRANSPORT_TX_RING_SIZE - s_tx_count) < length) {
        ++s_tx_overflow;
        taskEXIT_CRITICAL();
        return false;
    }
    index = s_tx_head;
    for (uint16_t offset = 0U; offset < length; ++offset) {
        s_tx_ring[index] = data[offset];
        index = ring_next(index, MB_TRANSPORT_TX_RING_SIZE);
    }
    s_tx_head = index;
    s_tx_count = (uint16_t)(s_tx_count + length);
    taskEXIT_CRITICAL();

    transport_enable_tx_irq(USART6);
    return true;
}

/** 无阻塞取出一个 RX 字节。 */
bool MB_Transport_ReadByte(uint8_t *byte)
{
    if (byte == NULL) {
        return false;
    }
    taskENTER_CRITICAL();
    if (s_rx_count == 0U) {
        taskEXIT_CRITICAL();
        return false;
    }
    *byte = s_rx_ring[s_rx_tail];
    s_rx_tail = ring_next(s_rx_tail, MB_TRANSPORT_RX_RING_SIZE);
    --s_rx_count;
    taskEXIT_CRITICAL();
    return true;
}

/** 清空 RX 环形缓冲并恢复帧边界。 */
void MB_Transport_ClearRx(void)
{
    taskENTER_CRITICAL();
    s_rx_head = 0U;
    s_rx_tail = 0U;
    s_rx_count = 0U;
    taskEXIT_CRITICAL();
}

/** 复制传输统计快照。 */
void MB_Transport_GetStats(mb_transport_stats_t *stats)
{
    if (stats == NULL) {
        return;
    }
    taskENTER_CRITICAL();
    stats->rx_bytes = s_rx_bytes;
    stats->rx_overflow = s_rx_overflow;
    stats->tx_bytes = s_tx_bytes;
    stats->tx_overflow = s_tx_overflow;
    stats->uart_errors = s_uart_errors;
    stats->rx_buffered = s_rx_count;
    stats->tx_buffered = s_tx_count;
    taskEXIT_CRITICAL();
}

/**
 * @brief 在 TXE/TXFNF 中断有效时从 TX ring 发送至多一个字节，空 ring 时关闭 TX IRQ。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param instance USART6 寄存器基址；调用方必须保证非 NULL。
 * @param isr 当前 ISR 快照；只有 TXE/TXFNF 置位且 CR1 仍启用中断才处理。
 * @return 无；写 TDR 后推进 tail/count 并累计 tx_bytes。
 * 调用方式：MB_Transport_IRQHandler() 排空 RX 后调用一次；后续字节由下一次 TXE IRQ 推进。
 * 线程约束：USART6 ISR 唯一 consumer；任务 producer 在临界区更新 ring 后才启用 IRQ。
 */
static void transport_service_tx(USART_TypeDef *instance, uint32_t isr)
{
    if ((instance->CR1 & USART_CR1_TXEIE_TXFNFIE) != 0U &&
        (isr & USART_ISR_TXE_TXFNF) != 0U) {
        if (s_tx_count != 0U) {
            instance->TDR = s_tx_ring[s_tx_tail];
            s_tx_tail = ring_next(s_tx_tail, MB_TRANSPORT_TX_RING_SIZE);
            --s_tx_count;
            ++s_tx_bytes;
        } else {
            transport_disable_tx_irq(instance);
        }
    }
}

/** USART6 ISR 入口：只搬运字节和清理硬件标志。 */
void MB_Transport_IRQHandler(void)
{
    USART_TypeDef *instance = huart6.Instance;
    uint32_t isr;

    if (instance != USART6) {
        return;
    }

    isr = instance->ISR;
    transport_clear_error_flags(instance, isr);

    /* Drain all pending bytes. FIFO mode is disabled by MX_USART6_UART_Init(),
     * but the loop also handles bytes that arrived back-to-back before entry. */
    while ((instance->ISR & USART_ISR_RXNE_RXFNE) != 0U) {
        transport_push_rx_byte((uint8_t)(instance->RDR & 0xFFU));
    }

    transport_service_tx(instance, instance->ISR);
}
