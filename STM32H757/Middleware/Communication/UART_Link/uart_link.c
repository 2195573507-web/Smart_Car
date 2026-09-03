#include "uart_link.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include "log_service.h"
#include "srp_codec.h"
#include "srp_registry.h"
#include "cm7_raw_diag.h"
#include "smartcar_debug_config.h"

/* CM7 USART2 DMA 链路实现；创建人：待确认（当前维护人：Zhiqin）。 */

#define UART_LINK_TASK_STACK_WORDS UINT16_C(1024)
#define UART_LINK_TASK_PRIORITY (tskIDLE_PRIORITY + 2U)
#define UART_LINK_IRQ_PRIORITY UINT32_C(5)
#define UART_LINK_DCACHE_LINE_SIZE UINT32_C(32)
#define UART_LINK_TX_TIMEOUT_MS UINT32_C(50)

_Static_assert(SRP_HEADER_SIZE + SRP_PAYLOAD_CMD_SYNC_REQ_SIZE + SRP_TRAILER_SIZE ==
                   UINT16_C(16),
               "SRP startup sync frame must be 16 bytes");

static UART_HandleTypeDef s_handle;
static DMA_HandleTypeDef s_rx_dma;
static SemaphoreHandle_t s_tx_mutex;
static volatile uint8_t s_ready;
static volatile uint8_t s_rx_active;
static volatile uint8_t s_restart_requested;
static volatile uint8_t s_recovering;
static uint32_t s_baud_rate = UART_LINK_BAUD_RATE;
static uint8_t s_ring[UART_LINK_RX_RING_SIZE];
static uint8_t s_dma_rx[UART_LINK_RX_DMA_SIZE]
    __attribute__((section(".dma_buffer"), aligned(32)));
static volatile uint16_t s_head;
static volatile uint16_t s_tail;
static volatile uint16_t s_count;
static volatile uint8_t s_tx_active;
static volatile uint16_t s_tx_active_length;
static volatile uint32_t s_rx_bytes;
static volatile uint32_t s_rx_frames;
static volatile uint32_t s_last_rx_time;
static volatile uint32_t s_rx_overflow_count;
static volatile uint32_t s_rx_drop_bytes;
static volatile uint32_t s_tx_count_total;
static volatile uint32_t s_tx_bytes_total;
static volatile uint32_t s_tx_timeout_count;
static volatile uint32_t s_tx_queue_drop;
static volatile uint32_t s_tx_preemptions;
static volatile uint32_t s_hal_error_count;
static volatile uint32_t s_rx_event_count;
static volatile uint32_t s_rx_event_bytes;
static volatile uint32_t s_rx_rearm_count;
static volatile uint32_t s_rx_rearm_failures;
static volatile uint32_t s_tx_dma_start_count;
static volatile uint32_t s_tx_dma_error_count;
static volatile uint32_t s_tx_last_gstate;
static volatile uint32_t s_tx_last_error_code;
static volatile uint32_t s_tx_busy_recovery_count;
static volatile uint32_t s_usart_error_count;
static volatile uint32_t s_usart_last_error_code;
static volatile uint32_t s_dma_rx_irq_count;
static volatile uint32_t s_dma_tx_irq_count;
static volatile uint32_t s_usart_irq_count;
static volatile uint32_t s_rx_callback_reject_count;
static volatile uint32_t s_rx_dma_error_snapshot;
static volatile uint32_t s_rx_dmamux_request_snapshot;
static volatile uint32_t s_tx_dmamux_request_snapshot;
static volatile uint32_t s_rx_dma_cr_snapshot;
static volatile uint32_t s_rx_gpio_state_snapshot;
static volatile uint16_t s_rx_dma_ndtr_snapshot;
static volatile uint8_t s_error_log_pending;
static volatile uint32_t s_error_log_code;
static TaskHandle_t s_uart_link_task_handle;

static void uart_link_clear_error_flags(void);

/**
 * @brief 在调用方已经进入临界区时把字节写入 RX ring，满时逐字丢弃最旧数据。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param data 至少包含 length 字节的只读输入；length>0 时必须非 NULL。
 * @param length 待写字节数。
 * @return 本次丢弃的旧字节数；只要发生过丢弃，overflow_count 本次仅增加一次。
 * 调用方式：当前仅 ring_push_from_isr() 在 FROM_ISR 临界区内调用。
 * 线程约束：函数本身不加锁；调用方必须屏蔽与 ring 读写竞争的上下文，禁止无保护调用。
 */
static uint16_t ring_push_locked(const uint8_t *data, uint16_t length)
{
    uint16_t dropped = 0U;

    for (uint16_t index = 0U; index < length; ++index) {
        if (s_count == UART_LINK_RX_RING_SIZE) {
            if (dropped == 0U) {
                ++s_rx_overflow_count;
            }
            s_tail = (uint16_t)((s_tail + 1U) % UART_LINK_RX_RING_SIZE);
            --s_count;
            ++s_rx_drop_bytes;
            ++dropped;
        }
        s_ring[s_head] = data[index];
        s_head = (uint16_t)((s_head + 1U) % UART_LINK_RX_RING_SIZE);
        ++s_count;
    }
    return dropped;
}

/**
 * @brief 按 32 字节 cache line 扩展地址范围并使 D-cache 内容失效。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param data DMA 缓冲或其片段起始地址；NULL 时返回。
 * @param length 字节数；0 时返回。
 * @return 无；对齐后范围可能包含 data 前后同一 cache line 的邻接字节。
 * 调用方式：启动 DMA 前使整个 RX buffer 失效，RX event 读取 DMA 数据前再次失效有效片段。
 * 线程约束：直接执行 SCB cache 维护，可在任务或 UART HAL 回调上下文调用；缓冲必须按 cache 规则隔离。
 */
static void dcache_invalidate(const uint8_t *data, uint16_t length)
{
    uintptr_t start;
    uintptr_t end;
    uintptr_t aligned_start;
    uintptr_t aligned_end;

    if (data == NULL || length == 0U) {
        return;
    }
    start = (uintptr_t)data;
    end = start + length;
    aligned_start = start & ~(uintptr_t)(UART_LINK_DCACHE_LINE_SIZE - 1U);
    aligned_end = (end + UART_LINK_DCACHE_LINE_SIZE - 1U) &
                  ~(uintptr_t)(UART_LINK_DCACHE_LINE_SIZE - 1U);
    SCB_InvalidateDCache_by_Addr((uint32_t *)aligned_start,
                                 (int32_t)(aligned_end - aligned_start));
}

/**
 * @brief 采集 RX DMA/DMAMUX、PA3 GPIO 和 NDTR 的无锁诊断快照。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * 传入参数：无。
 * @return 无；TX DMA 在阻塞兼容路径固定记录为 0，快照不证明物理线路正确。
 * 调用方式：DMA 配置、重装、RX callback 和 IRQ 转发边界调用。
 * 线程约束：任务和 ISR 都可写这些 volatile 字段，不提供多字段原子一致性，仅用于诊断。
 */
static void uart_link_snapshot_dma_state(void)
{
    const DMA_Stream_TypeDef *rx_stream =
        (const DMA_Stream_TypeDef *)s_rx_dma.Instance;

    s_rx_dma_error_snapshot = s_rx_dma.ErrorCode;
    s_rx_dmamux_request_snapshot =
        s_rx_dma.DMAmuxChannel == NULL
            ? 0U
            : (s_rx_dma.DMAmuxChannel->CCR & DMAMUX_CxCR_DMAREQ_ID);
    /* TX DMA is intentionally disabled in the compatibility path. */
    s_tx_dmamux_request_snapshot = 0U;
    s_rx_gpio_state_snapshot =
        ((GPIOA->MODER >> 6U) & 0x3U) |
        (((GPIOA->PUPDR >> 6U) & 0x3U) << 2U) |
        (((GPIOA->AFR[0] >> 12U) & 0xFU) << 4U) |
        (((GPIOA->IDR & GPIO_PIN_3) != 0U) ? (1U << 8U) : 0U);
    if (rx_stream != NULL) {
        s_rx_dma_cr_snapshot = rx_stream->CR;
        s_rx_dma_ndtr_snapshot = (uint16_t)rx_stream->NDTR;
    } else {
        s_rx_dma_cr_snapshot = 0U;
        s_rx_dma_ndtr_snapshot = 0U;
    }
}

/**
 * @brief 校验完整候选缓冲的 SRP magic、长度上限、精确总长、header、CRC 和 EOF。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param data 只读完整帧候选；允许 NULL。
 * @param length 候选字节数，必须不超过 SRP_MAX_FRAME_SIZE。
 * @return srp_decode() 完整通过时返回 1，否则返回 0。
 * 调用方式：阻塞 TX 前防御性验证编码结果，以及本地启动 self-test 使用。
 * 线程约束：纯内存解码、可重入、不阻塞；decoded payload 仅在函数内借用。
 */
static uint8_t uart_link_is_valid_frame(const uint8_t *data, uint16_t length)
{
    uint16_t payload_length;
    uint16_t expected_length;
    srp_frame_t decoded;

    if (data == NULL || length < SRP_HEADER_SIZE ||
        length > SRP_MAX_FRAME_SIZE || data[0] != SRP_MAGIC_BYTE0 ||
        data[1] != SRP_MAGIC_BYTE1) {
        return 0U;
    }
    payload_length = (uint16_t)data[2] | (uint16_t)((uint16_t)data[3] << 8U);
    if (payload_length > SRP_MAX_PAYLOAD) {
        return 0U;
    }
    expected_length = (uint16_t)(SRP_HEADER_SIZE + payload_length +
                                 SRP_TRAILER_SIZE);
    if (expected_length != length) {
        return 0U;
    }
    return srp_decode(data, length, &decoded) == SRP_CODEC_OK ? 1U : 0U;
}

/**
 * @brief 在内存中编码并解码固定 16 字节 CMD_SYNC_REQ，验证本地 SRP codec。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * 传入参数：无。
 * @return 无；失败增加 HAL error 诊断计数，成功/失败均只写日志。
 * 调用方式：USART2 DMA RX 首次武装后由 uart_link_init() 调用一次；不调用 uart_link_send()，
 *           不在物理 UART2 上发送任何字节，实际同步仍由 S3 发起。
 * 线程约束：使用静态编码缓冲，仅单线程启动上下文调用；会使用日志队列，禁止 ISR。
 */
static void uart_link_startup_self_test(void)
{
    static const uint8_t payload[SRP_PAYLOAD_CMD_SYNC_REQ_SIZE] = {
        SRP_PROTOCOL_VERSION_MAJOR, SRP_PROTOCOL_VERSION_MINOR, 0U, 0U
    };
    const srp_frame_t frame = {
        .priority = SRP_PRIORITY_COMMAND,
        .type = SRP_MSG_ID_CMD_SYNC_REQ,
        .sequence = 0U,
        .flags = SRP_FLAG_STREAM_DATA,
        .length = sizeof(payload),
        .payload = payload,
    };
    static uint8_t encoded[SRP_MAX_FRAME_SIZE];
    uint16_t encoded_length = 0U;
    char line[SMARTCAR_LOG_MAX_PAYLOAD + 1U];
    int result;

    result = srp_encode_frame(&frame, encoded, sizeof(encoded), &encoded_length);
    if (result != SRP_CODEC_OK || encoded_length != UINT16_C(16) ||
        encoded[0] != SRP_MAGIC_BYTE0 || encoded[1] != SRP_MAGIC_BYTE1 ||
        uart_link_is_valid_frame(encoded, encoded_length) == 0U) {
        ++s_hal_error_count;
        LOG_ERROR("[SRP_STARTUP_SELFTEST] encode/validation failed\r\n");
        return;
    }

    (void)snprintf(line, sizeof(line),
                   "[SRP_STARTUP_SELFTEST] local encode/decode len=%u bytes=%02X %02X %02X %02X "
                   "%02X %02X %02X %02X ...\r\n",
                   (unsigned)encoded_length, encoded[0], encoded[1], encoded[2],
                   encoded[3], encoded[4], encoded[5], encoded[6], encoded[7]);
    LOG_INFO(line);
}

/**
 * @brief 根据 PCLK1、prescaler、oversampling 和 BRR 估算 USART2 实际波特率并记录配置。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * 传入参数：无；要求 s_handle 已由 HAL_UART_Init() 成功配置且 Instance 有效。
 * @return 无；PCLK/BRR 为 0 时 actual/error 保持 0，只提供诊断，不改变时钟或 UART。
 * 调用方式：每次 uart_link_configure_uart() 成功后调用并输出两条 INFO 日志。
 * 线程约束：读取 RCC/UART 寄存器并格式化日志，仅启动/恢复任务调用，禁止 ISR。
 */
static void uart_link_log_clock_diagnostics(void)
{
    const uint32_t pclk = HAL_RCC_GetPCLK1Freq();
    const uint32_t brr = s_handle.Instance->BRR;
    const uint32_t configured_baud = s_handle.Init.BaudRate;
    uint32_t clock_prescaler = 1U;
    uint32_t actual_baud = 0U;
    uint32_t error_ppm = 0U;
    char line[192];

    switch (s_handle.Init.ClockPrescaler) {
    case UART_PRESCALER_DIV2:   clock_prescaler = 2U;   break;
    case UART_PRESCALER_DIV4:   clock_prescaler = 4U;   break;
    case UART_PRESCALER_DIV6:   clock_prescaler = 6U;   break;
    case UART_PRESCALER_DIV8:   clock_prescaler = 8U;   break;
    case UART_PRESCALER_DIV10:  clock_prescaler = 10U;  break;
    case UART_PRESCALER_DIV12:  clock_prescaler = 12U;  break;
    case UART_PRESCALER_DIV16:  clock_prescaler = 16U;  break;
    case UART_PRESCALER_DIV32:  clock_prescaler = 32U;  break;
    case UART_PRESCALER_DIV64:  clock_prescaler = 64U;  break;
    case UART_PRESCALER_DIV128: clock_prescaler = 128U; break;
    case UART_PRESCALER_DIV256: clock_prescaler = 256U; break;
    case UART_PRESCALER_DIV1:
    default:
        clock_prescaler = 1U;
        break;
    }
    if (pclk != 0U && brr != 0U) {
        const uint64_t uart_clock = pclk / clock_prescaler;
        const uint64_t brr_numerator =
            s_handle.Init.OverSampling == UART_OVERSAMPLING_8
                ? uart_clock * UINT64_C(2)
                : uart_clock;

        /* STM32H7 stores fclk/baud in BRR for OVER16 and 2*fclk/baud
         * for OVER8; BRR is not a fixed-point oversampling divider. */
        actual_baud = (uint32_t)(brr_numerator / brr);
        if (configured_baud != 0U) {
            const uint64_t delta = actual_baud >= configured_baud
                                       ? (uint64_t)(actual_baud - configured_baud)
                                       : (uint64_t)(configured_baud - actual_baud);
            error_ppm = (uint32_t)((delta * UINT64_C(1000000)) /
                                   configured_baud);
        }
    }
    (void)snprintf(line, sizeof(line),
                   "[SRP_UART2_CLOCK] SystemCoreClock=%lu D2PCLK1=%lu "
                   "BRR=0x%08lX baud=%lu actual=%lu error_ppm=%lu\r\n",
                   (unsigned long)SystemCoreClock, (unsigned long)pclk,
                   (unsigned long)brr, (unsigned long)configured_baud,
                   (unsigned long)actual_baud, (unsigned long)error_ppm);
    LOG_INFO(line);
    (void)snprintf(line, sizeof(line),
                   "[SRP_UART2_CONFIG] word=%lu stop=%lu parity=%lu mode=%lu "
                   "flow=%lu oversampling=%lu\r\n",
                   (unsigned long)s_handle.Init.WordLength,
                   (unsigned long)s_handle.Init.StopBits,
                   (unsigned long)s_handle.Init.Parity,
                   (unsigned long)s_handle.Init.Mode,
                   (unsigned long)s_handle.Init.HwFlowCtl,
                   (unsigned long)s_handle.Init.OverSampling);
    LOG_INFO(line);
}

/**
 * @brief 在 HAL RX event 上下文把 DMA 字节复制到软件 ring 并更新时间/计数。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param data DMA 接收片段；NULL 时返回。
 * @param length 有效字节数；0 时返回。
 * @return 无；ring 满时覆盖最旧字节并累计 drop，不解析 SRP、不写日志。
 * 调用方式：只由 HAL_UARTEx_RxEventCallback() 在重装下一次 DMA 前调用。
 * 线程约束：HAL callback/ISR 上下文；先做 cache invalidate，再进入 FROM_ISR 临界区，禁止任务直接调用。
 */
static void ring_push_from_isr(const uint8_t *data, uint16_t length)
{
    UBaseType_t mask;

    if (data == NULL || length == 0U) {
        return;
    }
    dcache_invalidate(data, length);
    mask = taskENTER_CRITICAL_FROM_ISR();
    (void)ring_push_locked(data, length);
    s_rx_bytes += length;
    s_rx_event_bytes += length;
    ++s_rx_frames;
    s_last_rx_time = HAL_GetTick();
    taskEXIT_CRITICAL_FROM_ISR(mask);
}

/**
 * @brief 初始化 DMA1 Stream0 为 USART2 RX 的高优先级 normal-mode 字节传输并校验 DMAMUX。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * 传入参数：无；要求 s_handle 已指向 USART2。
 * @return HAL DMA 初始化且 DMAMUX request 回读为 DMA_REQUEST_USART2_RX 时返回 1，否则 0。
 * 调用方式：首次初始化和任务上下文 recovery 在 UART 配置成功后调用；失败会记录有界日志。
 * 线程约束：重置/链接全局 DMA handle 并访问 RCC/HAL，仅启动或恢复 owner 调用，禁止 ISR/并发调用。
 */
static uint8_t uart_link_configure_dma(void)
{
    __HAL_RCC_DMA1_CLK_ENABLE();
    (void)memset(&s_rx_dma, 0, sizeof(s_rx_dma));

    s_rx_dma.Instance = DMA1_Stream0;
    s_rx_dma.Init.Request = DMA_REQUEST_USART2_RX;
    s_rx_dma.Init.Direction = DMA_PERIPH_TO_MEMORY;
    s_rx_dma.Init.PeriphInc = DMA_PINC_DISABLE;
    s_rx_dma.Init.MemInc = DMA_MINC_ENABLE;
    s_rx_dma.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    s_rx_dma.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    /* ReceiveToIdle is used as a bounded transaction.  The HAL marks a
     * normal-mode transaction READY before invoking the Rx event callback,
     * which lets the callback unconditionally arm the next transaction. */
    s_rx_dma.Init.Mode = DMA_NORMAL;
    s_rx_dma.Init.Priority = DMA_PRIORITY_HIGH;
    s_rx_dma.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&s_rx_dma) != HAL_OK) {
        return 0U;
    }
    __HAL_LINKDMA(&s_handle, hdmarx, s_rx_dma);

    uart_link_snapshot_dma_state();
    if (s_rx_dmamux_request_snapshot != DMA_REQUEST_USART2_RX) {
        char line[SMARTCAR_LOG_MAX_PAYLOAD + 1U];

        ++s_hal_error_count;
        (void)snprintf(line, sizeof(line),
                       "[SRP_DMA] bad_req rx=%lu\r\n",
                       (unsigned long)s_rx_dmamux_request_snapshot);
        LOG_ERROR(line);
        return 0U;
    }

    return 1U;
}

/**
 * @brief 把 DMA1 Stream0 与 USART2 NVIC 优先级设为 5 并启用两个中断。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * 传入参数：无。
 * @return 无。
 * 调用方式：DMA/UART handle 均配置完成后、启动 ReceiveToIdle 前调用；recovery 重新配置后也调用。
 * 线程约束：直接修改 NVIC，仅启动/恢复任务 owner 调用，禁止 ISR 或并发外设重配置。
 */
static void uart_link_enable_irqs(void)
{
    HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, UART_LINK_IRQ_PRIORITY, 0U);
    HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
    HAL_NVIC_SetPriority(USART2_IRQn, UART_LINK_IRQ_PRIORITY, 0U);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
}

/**
 * @brief 以当前 s_baud_rate 配置 USART2 为 8N1、TX/RX、无流控、16 倍采样并启用 FIFO。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * 传入参数：无。
 * @return HAL_UART_Init 成功后返回 1，失败返回 0；FIFO threshold API 的返回值当前不传播。
 * 调用方式：首次初始化和 recovery 在 DMA 配置前调用；成功后清线路错误并输出时钟诊断。
 * 线程约束：访问 HAL/UART/GPIO MSP 与日志队列，仅启动/恢复任务调用，禁止 ISR/并发调用。
 */
static uint8_t uart_link_configure_uart(void)
{
    s_handle.Instance = UART_LINK_USART;
    s_handle.Init.BaudRate = s_baud_rate;
    s_handle.Init.WordLength = UART_WORDLENGTH_8B;
    s_handle.Init.StopBits = UART_STOPBITS_1;
    s_handle.Init.Parity = UART_PARITY_NONE;
    s_handle.Init.Mode = UART_MODE_TX_RX;
    s_handle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    s_handle.Init.OverSampling = UART_OVERSAMPLING_16;
    s_handle.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    s_handle.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    s_handle.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&s_handle) != HAL_OK) {
        return 0U;
    }
    /* A reset-time line glitch may have latched an error before the first SRP
     * transmission. Start from a known-clear UART state. */
    uart_link_clear_error_flags();
    (void)HAL_UARTEx_SetTxFifoThreshold(&s_handle, UART_TXFIFO_THRESHOLD_1_8);
    (void)HAL_UARTEx_SetRxFifoThreshold(&s_handle, UART_RXFIFO_THRESHOLD_1_8);
    (void)HAL_UARTEx_EnableFifoMode(&s_handle);
    uart_link_log_clock_diagnostics();
    return 1U;
}

/**
 * @brief 武装一次 UART2 normal-mode ReceiveToIdle DMA 事务并关闭 half-transfer 中断。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * 传入参数：无；使用固定 512 字节、32 字节对齐的 s_dma_rx。
 * @return HAL 接受事务时返回 1；失败清 active、累计 HAL/rearm failure 并返回 0。
 * 调用方式：启动/恢复任务以及 HAL RX/error callback 都会调用；active 在 HAL 调用前先置 1，
 *           防止已挂起 IDLE/error 回调把首帧误判为未武装。
 * 线程约束：任务与 HAL callback 共享无锁状态，调用序列由 HAL RxState/recovering 约束；禁止任意并发调用。
 */
static uint8_t uart_link_start_dma_receive(void)
{
    HAL_StatusTypeDef status;

    ++s_rx_rearm_count;
    /* Mark the stream active before enabling UART IDLE/DMA interrupts.  The
     * HAL call enables those sources internally; an already-pending IDLE or
     * error interrupt can otherwise enter the callback before the call
     * returns and discard the first bytes of a sync request. */
    s_rx_active = 1U;
    dcache_invalidate(s_dma_rx, UART_LINK_RX_DMA_SIZE);
    uart_link_clear_error_flags();
    status = HAL_UARTEx_ReceiveToIdle_DMA(&s_handle, s_dma_rx,
                                           UART_LINK_RX_DMA_SIZE);
    if (status != HAL_OK) {
        s_rx_active = 0U;
        ++s_hal_error_count;
        ++s_rx_rearm_failures;
        return 0U;
    }
    __HAL_DMA_DISABLE_IT(&s_rx_dma, DMA_IT_HT);
    uart_link_snapshot_dma_state();
    return 1U;
}

/**
 * @brief 保留已经 BUSY_RX 的有效事务，否则尝试重新武装 ReceiveToIdle DMA。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * 传入参数：无。
 * @return active 且 HAL RxState 为 BUSY_RX，或重新武装成功时返回 1；否则 0。
 * 调用方式：本地 codec self-test 后确认测试过程没有意外丢失 RX 事务。
 * 线程约束：启动任务调用；不会替换 callback 已经重装的事务，禁止 ISR 手工调用。
 */
static uint8_t uart_link_ensure_dma_receive(void)
{
    /* The raw startup marker precedes DMA setup by design.  This helper keeps
     * RX armed after the later local codec self-test and avoids replacing a
     * transaction that an RX callback has already rearmed. */
    if (s_rx_active != 0U && s_handle.RxState == HAL_UART_STATE_BUSY_RX) {
        return 1U;
    }
    return uart_link_start_dma_receive();
}

/**
 * @brief 清除 USART2 ORE/NE/PE/FE 硬件标志，不修改 HAL ErrorCode 或软件统计。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * 传入参数：无；要求 s_handle 已指向有效 USART2 实例。
 * @return 无。
 * 调用方式：UART 配置、DMA 重装、每次阻塞 TX 和 HAL error callback 的恢复边界调用。
 * 线程约束：任务和 HAL callback 都可调用寄存器清标志；调用方负责与 deinit/reconfigure 串行化。
 */
static void uart_link_clear_error_flags(void)
{
    __HAL_UART_CLEAR_FLAG(&s_handle,
                          UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_PEF |
                              UART_CLEAR_FEF);
}

/**
 * @brief 在 FreeRTOS 临界区丢弃软件 RX ring 并清除 TX active 标志。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * 传入参数：无。
 * @return 无；不清累计统计、last_rx_time 或 s_tx_active_length，也不操作 DMA/UART 硬件。
 * 调用方式：recovery 已禁 IRQ并 abort/deinit UART/DMA 后、重新配置前调用。
 * 线程约束：任务上下文临界区；运行期调用会丢 RX 数据，只允许 recovery owner。
 */
static void uart_link_reset_state(void)
{
    taskENTER_CRITICAL();
    s_head = 0U;
    s_tail = 0U;
    s_count = 0U;
    s_tx_active = 0U;
    taskEXIT_CRITICAL();
}

/** 清理排队/在途 TX，使同步响应优先。 */
void uart_link_flush_tx(void)
{
    if (s_tx_mutex != NULL &&
        xSemaphoreTake(s_tx_mutex, 0U) == pdTRUE) {
        if (s_tx_active != 0U) {
            (void)HAL_UART_AbortTransmit(&s_handle);
            s_tx_active = 0U;
            s_tx_active_length = 0U;
        }
        (void)xSemaphoreGive(s_tx_mutex);
    }
}

/** 初始化 USART2 DMA、互斥量和错误快照。 */
void uart_link_init(void)
{
    (void)memset(&s_handle, 0, sizeof(s_handle));
    s_tx_mutex = xSemaphoreCreateMutex();
    s_ready = 0U;
    s_restart_requested = 0U;
    s_recovering = 0U;
    s_baud_rate = UART_LINK_BAUD_RATE;
    s_rx_bytes = 0U;
    s_rx_frames = 0U;
    s_last_rx_time = 0U;
    s_rx_overflow_count = 0U;
    s_rx_drop_bytes = 0U;
    s_tx_count_total = 0U;
    s_tx_bytes_total = 0U;
    s_tx_timeout_count = 0U;
    s_tx_queue_drop = 0U;
    s_tx_preemptions = 0U;
    s_hal_error_count = 0U;
    s_rx_event_count = 0U;
    s_rx_event_bytes = 0U;
    s_rx_rearm_count = 0U;
    s_rx_rearm_failures = 0U;
    s_tx_dma_start_count = 0U;
    s_tx_dma_error_count = 0U;
    s_tx_last_gstate = HAL_UART_STATE_RESET;
    s_tx_last_error_code = HAL_UART_ERROR_NONE;
    s_tx_busy_recovery_count = 0U;
    s_usart_error_count = 0U;
    s_usart_last_error_code = 0U;
    s_dma_rx_irq_count = 0U;
    s_dma_tx_irq_count = 0U;
    s_usart_irq_count = 0U;
    s_rx_callback_reject_count = 0U;
    s_rx_dma_error_snapshot = 0U;
    s_rx_dmamux_request_snapshot = 0U;
    s_tx_dmamux_request_snapshot = 0U;
    s_rx_dma_cr_snapshot = 0U;
    s_rx_gpio_state_snapshot = 0U;
    s_rx_dma_ndtr_snapshot = 0U;
    s_error_log_pending = 0U;
    s_error_log_code = 0U;
    s_uart_link_task_handle = NULL;
    if (s_tx_mutex == NULL || uart_link_configure_uart() == 0U ||
        uart_link_configure_dma() == 0U) {
        ++s_hal_error_count;
        return;
    }
    /* Enable the vectors before arming the first transaction so a DMA TC or
     * USART IDLE that occurs during startup is latched and serviced without a
     * post-arm interrupt window. */
    uart_link_enable_irqs();
    if (uart_link_start_dma_receive() == 0U) {
        /* Keep the worker alive so a transient first-arm failure can recover
         * after the scheduler starts instead of leaving USART2 permanently
         * unarmed with s_ready cleared. */
        s_restart_requested = 1U;
        ++s_hal_error_count;
        return;
    }
    s_ready = 1U;

    /* DMA RX is armed before the local codec self-test.  S3 exclusively
     * initiates CMD_SYNC_REQ; this test must not emit anything on USART2. */
    uart_link_startup_self_test();
    if (uart_link_ensure_dma_receive() == 0U) {
        s_restart_requested = 1U;
        ++s_hal_error_count;
    }
}

/** 返回 UART/DMA 就绪标志，不代表 SRP 会话已同步。 */
uint8_t uart_link_is_ready(void)
{
    return s_ready;
}

/** 返回最近一次 RX 字节的 HAL tick。 */
uint32_t uart_link_get_last_rx_time(void)
{
    uint32_t last_rx_time;

    taskENTER_CRITICAL();
    last_rx_time = s_last_rx_time;
    taskEXIT_CRITICAL();
    return last_rx_time;
}

/** 任务上下文阻塞发送 SRP 帧；输入缓冲只需保持到函数返回。 */
HAL_StatusTypeDef uart_link_send(const uint8_t *data, uint16_t length)
{
    HAL_StatusTypeDef status;

#if SMARTCAR_RAW_DIAGNOSTICS
    cm7_raw_diag_tx_phase("ENTER", length);
#endif
    if (s_ready == 0U ||
        data == NULL || length == 0U ||
        length > SRP_MAX_FRAME_SIZE || !uart_link_is_valid_frame(data, length) ||
        s_tx_mutex == NULL) {
        ++s_hal_error_count;
#if SMARTCAR_RAW_DIAGNOSTICS
        cm7_raw_diag_tx_phase("REJECT", s_ready);
#endif
        return UART_TX_FAIL;
    }
    if (xSemaphoreTake(s_tx_mutex, pdMS_TO_TICKS(UART_LINK_TX_TIMEOUT_MS)) !=
        pdTRUE) {
        ++s_tx_queue_drop;
#if SMARTCAR_RAW_DIAGNOSTICS
        cm7_raw_diag_tx_phase("LOCK_FAIL", length);
#endif
        return HAL_TIMEOUT;
    }
#if SMARTCAR_RAW_DIAGNOSTICS
    cm7_raw_diag_tx_phase("LOCKED", length);
#endif
    s_tx_active = 1U;
    s_tx_active_length = length;
    /* A receive error can leave status flags latched, and a prior interrupted
     * TX can leave gState busy.  Clear line errors before every physical TX;
     * abort only a stale TX state while holding the sole TX-owner mutex. */
    uart_link_clear_error_flags();
    s_tx_last_gstate = s_handle.gState;
    s_tx_last_error_code = s_handle.ErrorCode;
    if (s_handle.gState != HAL_UART_STATE_READY) {
        ++s_tx_busy_recovery_count;
        if (HAL_UART_AbortTransmit(&s_handle) != HAL_OK) {
            status = HAL_ERROR;
            goto tx_done;
        }
        uart_link_clear_error_flags();
    }
#if SMARTCAR_RAW_DIAGNOSTICS
    cm7_raw_diag_tx_phase("HAL_BEGIN", length);
#endif
    status = HAL_UART_Transmit(&s_handle, (uint8_t *)data, length,
                               UART_LINK_TX_TIMEOUT_MS);
tx_done:
    s_tx_last_error_code = s_handle.ErrorCode;
#if SMARTCAR_RAW_DIAGNOSTICS
    cm7_raw_diag_tx_phase("HAL_DONE", (uint32_t)status);
#endif
    s_tx_active = 0U;
    s_tx_active_length = 0U;
    (void)xSemaphoreGive(s_tx_mutex);
    if (status == HAL_OK) {
        s_tx_bytes_total += length;
        ++s_tx_count_total;
    } else if (status == HAL_TIMEOUT) {
        ++s_tx_timeout_count;
    } else {
        ++s_hal_error_count;
    }
    return status;
}

/** 从软件 RX ring 复制字节；不在此处解析 SRP。 */
size_t uart_link_read(uint8_t *data, size_t capacity)
{
    size_t length;

    if (data == NULL || capacity == 0U) {
        return 0U;
    }
    taskENTER_CRITICAL();
    length = s_count < capacity ? s_count : capacity;
    for (size_t index = 0U; index < length; ++index) {
        data[index] = s_ring[s_tail];
        s_tail = (uint16_t)((s_tail + 1U) % UART_LINK_RX_RING_SIZE);
    }
    s_count = (uint16_t)(s_count - length);
    taskEXIT_CRITICAL();
    return length;
}

/** 复制 DMA/UART 统计快照。 */
void uart_link_get_stats(uart_link_stats_t *stats)
{
    if (stats == NULL) {
        return;
    }
    taskENTER_CRITICAL();
    stats->uart_tx_count = s_tx_count_total;
    stats->uart_tx_bytes = s_tx_bytes_total;
    stats->uart_tx_timeout = s_tx_timeout_count;
    stats->uart_rx_bytes = s_rx_bytes;
    stats->uart_rx_overflow = s_rx_overflow_count;
    stats->uart_rx_drop = s_rx_drop_bytes;
    stats->uart_hal_error = s_hal_error_count;
    stats->uart_tx_queue_drop = s_tx_queue_drop;
    stats->uart_tx_preemptions = s_tx_preemptions;
    stats->uart_rx_events = s_rx_event_count;
    stats->uart_rx_event_bytes = s_rx_event_bytes;
    stats->uart_rx_rearms = s_rx_rearm_count;
    stats->uart_rx_rearm_failures = s_rx_rearm_failures;
    stats->uart_tx_dma_starts = s_tx_dma_start_count;
    stats->uart_tx_dma_errors = s_tx_dma_error_count;
    stats->uart_tx_gstate = s_tx_last_gstate;
    stats->uart_tx_error_code = s_tx_last_error_code;
    stats->uart_tx_busy_recoveries = s_tx_busy_recovery_count;
    stats->uart_usart_errors = s_usart_error_count;
    stats->uart_last_error_code = s_usart_last_error_code;
    stats->uart_dma_rx_irqs = s_dma_rx_irq_count;
    stats->uart_dma_tx_irqs = s_dma_tx_irq_count;
    stats->uart_usart_irqs = s_usart_irq_count;
    stats->uart_rx_callback_rejects = s_rx_callback_reject_count;
    stats->uart_rx_dma_error_code = s_rx_dma_error_snapshot;
    stats->uart_rx_dmamux_request = s_rx_dmamux_request_snapshot;
    stats->uart_tx_dmamux_request = s_tx_dmamux_request_snapshot;
    stats->uart_rx_dma_cr = s_rx_dma_cr_snapshot;
    stats->uart_rx_gpio_state = s_rx_gpio_state_snapshot;
    stats->uart_rx_dma_ndtr = s_rx_dma_ndtr_snapshot;
    stats->uart_rx_active = s_rx_active;
    stats->rx_buffered = s_count;
    stats->rx_buffer_capacity = UART_LINK_RX_RING_SIZE;
    taskEXIT_CRITICAL();
}

/** 清理 UART/DMA 状态并重新启动接收；上层需重新同步。 */
void uart_link_recover(void)
{
    uint8_t restart_ok = 0U;

    if (s_recovering != 0U) {
        s_restart_requested = 1U;
        return;
    }
    s_recovering = 1U;
    s_ready = 0U;
    taskENTER_CRITICAL();
    HAL_NVIC_DisableIRQ(DMA1_Stream0_IRQn);
    HAL_NVIC_DisableIRQ(USART2_IRQn);
    HAL_NVIC_ClearPendingIRQ(DMA1_Stream0_IRQn);
    HAL_NVIC_ClearPendingIRQ(USART2_IRQn);
    s_rx_active = 0U;
    s_tx_active = 0U;
    s_tx_active_length = 0U;
    taskEXIT_CRITICAL();
    (void)HAL_UART_Abort(&s_handle);
    (void)HAL_DMA_Abort(&s_rx_dma);
    (void)HAL_DMA_DeInit(&s_rx_dma);
    (void)HAL_UART_DeInit(&s_handle);
    uart_link_reset_state();
    if (uart_link_configure_uart() != 0U && uart_link_configure_dma() != 0U) {
        uart_link_enable_irqs();
        restart_ok = uart_link_start_dma_receive();
    }
    if (restart_ok != 0U) {
        s_ready = 1U;
        s_restart_requested = 0U;
    } else {
        s_ready = 0U;
        ++s_hal_error_count;
        s_restart_requested = 1U;
    }
    s_recovering = 0U;
}

/** 在无在途发送且双方已协商时切换波特率。 */
HAL_StatusTypeDef uart_link_set_baud_rate(uint32_t baud_rate)
{
    if (baud_rate == 0U || s_ready == 0U) {
        return HAL_ERROR;
    }
    s_baud_rate = baud_rate;
    uart_link_recover();
    return s_ready != 0U ? HAL_OK : HAL_ERROR;
}

/**
 * @brief 处理 USART2 ReceiveToIdle normal-mode 完成/IDLE 事件，复制字节并立即重装下一事务。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（HAL 回调契约补充）。
 * @param huart HAL 回调 handle；仅地址等于内部 s_handle 时处理。
 * @param size s_dma_rx 中本次有效字节数，允许 0，不得超过 512。
 * @return 无；handle/active/size 不符时累计 reject 并返回，重装失败置 restart_requested。
 * 调用方式：由 HAL_UARTEx/USART2-DMA IRQ 调用栈同步触发；数据复制后通知 UART worker。
 * 线程约束：ISR/HAL callback 上下文；只执行 cache/ring/计数/rearm/通知，不解析 SRP、不写普通日志。
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    BaseType_t higher_priority_task_woken = pdFALSE;

    if (huart != &s_handle || s_rx_active == 0U || size > UART_LINK_RX_DMA_SIZE) {
        ++s_rx_callback_reject_count;
        return;
    }
    ++s_rx_event_count;
    if (size != 0U) {
        ring_push_from_isr(s_dma_rx, size);
    }

    /* Normal-mode ReceiveToIdle completes the HAL transaction before this
     * callback.  Always rearm here, including a full-buffer/TC event, so a
     * single received frame can never silently disable USART2 RX. */
    s_rx_active = 0U;
    if (uart_link_start_dma_receive() == 0U) {
        s_restart_requested = 1U;
    }
    if (s_uart_link_task_handle != NULL) {
        vTaskNotifyGiveFromISR(s_uart_link_task_handle,
                               &higher_priority_task_woken);
    }
    uart_link_snapshot_dma_state();
    portYIELD_FROM_ISR(higher_priority_task_woken);
}

/**
 * @brief 保留的 USART2 DMA TX 完成兼容回调，清 active 并唤醒 worker。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（HAL 回调契约补充）。
 * @param huart HAL 回调 handle；仅地址等于内部 s_handle 时处理。
 * @return 无。
 * 调用方式：只有 HAL DMA TX 路径会触发；当前 uart_link_send() 使用阻塞 HAL_UART_Transmit，
 *           正常发送统计在任务返回路径更新，不依赖本回调。
 * 线程约束：ISR/HAL callback 上下文；只更新 volatile 计数/状态并 FromISR 通知任务。
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &s_handle) {
        s_tx_bytes_total += s_tx_active_length;
        s_tx_active = 0U;
        s_tx_active_length = 0U;
        ++s_tx_count_total;
        if (s_uart_link_task_handle != NULL) {
            BaseType_t higher_priority_task_woken = pdFALSE;
            vTaskNotifyGiveFromISR(s_uart_link_task_handle,
                                   &higher_priority_task_woken);
            portYIELD_FROM_ISR(higher_priority_task_woken);
        }
    }
}

/**
 * @brief 处理 USART2 HAL 错误，清线路标志、保存诊断并优先现场重装 RX DMA。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（HAL 回调契约补充）。
 * @param huart HAL 回调 handle；仅地址等于内部 s_handle 时处理。
 * @return 无；RxState 已 READY 且未 recovery 时尝试立即 rearm，失败则置 restart_requested。
 * 调用方式：由 HAL_UART_IRQHandler/error abort 完成路径同步触发，并唤醒 UART worker 输出日志或恢复。
 * 线程约束：ISR/HAL callback 上下文；不执行完整 deinit/reconfigure 或普通日志，错误文本交给任务处理。
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == &s_handle) {
        BaseType_t higher_priority_task_woken = pdFALSE;

        /* Clear all receive error sources before the worker aborts/restarts
         * DMA. Leaving ORE/FE/NE/PE latched can suppress the next RX event. */
        uart_link_clear_error_flags();
        s_usart_last_error_code = huart->ErrorCode;
        s_error_log_code = huart->ErrorCode;
        s_error_log_pending = 1U;
        ++s_hal_error_count;
        ++s_usart_error_count;
        s_rx_active = 0U;
        /* HAL invokes this callback after ending the DMA RX transaction and
         * completing its abort callback, so RxState is READY for an immediate
         * ReceiveToIdle re-arm. This closes the error-to-worker gap at 921600
         * baud. A failed re-arm still falls back to task-context recovery,
         * where abort/deinit remains safe. */
        if (s_recovering == 0U &&
            huart->RxState == HAL_UART_STATE_READY &&
            uart_link_start_dma_receive() != 0U) {
            s_restart_requested = 0U;
        } else {
            s_restart_requested = 1U;
        }
        if (s_uart_link_task_handle != NULL) {
            /* Wake the worker for diagnostics or fallback recovery without
             * waiting for its one-millisecond polling timeout. */
            vTaskNotifyGiveFromISR(s_uart_link_task_handle,
                                   &higher_priority_task_woken);
            portYIELD_FROM_ISR(higher_priority_task_woken);
        }
    }
}

/** DMA RX IRQ 转发入口，仅由中断处理函数调用。 */
void uart_link_handle_dma_rx_irq(void)
{
    ++s_dma_rx_irq_count;
    uart_link_snapshot_dma_state();
    HAL_DMA_IRQHandler(&s_rx_dma);
}

/** DMA TX IRQ 转发入口，仅由中断处理函数调用。 */
void uart_link_handle_dma_tx_irq(void)
{
    /* DMA1 Stream1 TX is intentionally unused in the compatibility path. */
}

/** USART2 IRQ 转发入口，仅由中断处理函数调用。 */
void uart_link_handle_usart_irq(void)
{
    ++s_usart_irq_count;
    uart_link_snapshot_dma_state();
    HAL_UART_IRQHandler(&s_handle);
}

/**
 * @brief 输出 UART worker 栈水位、RX/TX/DMA/IRQ/PA3 的两条低频诊断摘要。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * 传入参数：无。
 * @return 无；快照只反映 RAM/寄存器观察值，不证明 UART 物理链路或 SRP 同步成功。
 * 调用方式：uart_link_task 按 UART_LINK_STACK_MONITOR_PERIOD_MS 周期调用。
 * 线程约束：任务上下文，进入短临界区复制统计并使用较大栈缓冲/日志队列，禁止 ISR。
 */
static void uart_link_log_stack(void)
{
    uart_link_stats_t stats;
    const UBaseType_t free_stack = s_uart_link_task_handle == NULL
                                       ? 0U
                                       : uxTaskGetStackHighWaterMark(
                                             s_uart_link_task_handle);
    char line[256U];

    uart_link_get_stats(&stats);
    (void)snprintf(line, sizeof(line),
                   "[SRP_UART2_DIAG] ready=%u active=%u free_words=%lu rx=%lu "
                   "events=%lu/%lu rearm=%lu/%lu buffered=%u tx=%lu starts=%lu dma_err=%lu "
                   "qdrop=%lu hal=%lu tx_state=0x%08lX tx_err=0x%08lX "
                   "busy_fix=%lu usart_err=%lu errcode=0x%08lX\r\n",
                   (unsigned)s_ready, (unsigned)stats.uart_rx_active,
                   (unsigned long)free_stack, (unsigned long)stats.uart_rx_bytes,
                   (unsigned long)stats.uart_rx_events,
                   (unsigned long)stats.uart_rx_event_bytes,
                   (unsigned long)stats.uart_rx_rearms,
                   (unsigned long)stats.uart_rx_rearm_failures,
                   (unsigned)stats.rx_buffered,
                   (unsigned long)stats.uart_tx_count,
                   (unsigned long)stats.uart_tx_dma_starts,
                   (unsigned long)stats.uart_tx_dma_errors,
                   (unsigned long)stats.uart_tx_queue_drop,
                   (unsigned long)stats.uart_hal_error,
                   (unsigned long)stats.uart_tx_gstate,
                   (unsigned long)stats.uart_tx_error_code,
                   (unsigned long)stats.uart_tx_busy_recoveries,
                   (unsigned long)stats.uart_usart_errors,
                   (unsigned long)stats.uart_last_error_code);
    LOG_INFO(line);
    (void)snprintf(line, sizeof(line),
                   "[SRP_UART2_HW] irq=%lu/%lu/%lu cbx=%lu ndtr=%u "
                   "req=%lu/%lu de=%lu gpio=0x%03lX\r\n",
                   (unsigned long)stats.uart_dma_rx_irqs,
                   (unsigned long)stats.uart_dma_tx_irqs,
                   (unsigned long)stats.uart_usart_irqs,
                   (unsigned long)stats.uart_rx_callback_rejects,
                   (unsigned)stats.uart_rx_dma_ndtr,
                   (unsigned long)stats.uart_rx_dmamux_request,
                   (unsigned long)stats.uart_tx_dmamux_request,
                   (unsigned long)stats.uart_rx_dma_error_code,
                   (unsigned long)stats.uart_rx_gpio_state);
    LOG_INFO(line);
}

/**
 * @brief 原子取走 HAL error callback 保存的最近错误码并在任务上下文输出 WARN。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * 传入参数：无。
 * @return 无；没有 pending 错误时不输出，读取后清 s_error_log_pending。
 * 调用方式：UART worker 每轮 recovery 之后调用，把 ISR 日志工作移出回调。
 * 线程约束：使用短 FreeRTOS 临界区与日志队列，仅 UART worker 调用，禁止 ISR。
 */
static void uart_link_log_isr_diagnostics(void)
{
    uint8_t error_pending;
    uint32_t error_code;
    char line[SMARTCAR_LOG_MAX_PAYLOAD + 1U];

    /* Copy and clear the ISR error sample atomically; normal RX events are
     * accounted for in counters but remain silent in the production log. */
    taskENTER_CRITICAL();
    error_pending = s_error_log_pending;
    error_code = s_error_log_code;
    s_error_log_pending = 0U;
    taskEXIT_CRITICAL();

    if (error_pending != 0U) {
        (void)snprintf(line, sizeof(line),
                       "[UART2_ERR] err=0x%lx\r\n",
                       (unsigned long)error_code);
        LOG_WARN(line);
    }
}

/** UART 链路任务：重装 DMA、搬运错误状态和低频诊断。 */
void uart_link_task(void *argument)
{
    uint32_t last_stack_monitor_ms;

    (void)argument;
    last_stack_monitor_ms = HAL_GetTick();
    for (;;) {
        (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1U));
        if (s_restart_requested != 0U) {
            uart_link_recover();
        }
        uart_link_log_isr_diagnostics();
        if ((uint32_t)(HAL_GetTick() - last_stack_monitor_ms) >=
            UART_LINK_STACK_MONITOR_PERIOD_MS) {
            last_stack_monitor_ms = HAL_GetTick();
            uart_link_log_stack();
        }
    }
}

/** 创建唯一 UART 链路任务；重复调用保持幂等。 */
void uart_link_task_start(void)
{
    if (s_uart_link_task_handle != NULL ||
        (s_ready == 0U && s_restart_requested == 0U)) {
        return;
    }
    if (xTaskCreate(uart_link_task, "srp_uart", UART_LINK_TASK_STACK_WORDS,
                    NULL, UART_LINK_TASK_PRIORITY,
                    &s_uart_link_task_handle) != pdPASS) {
        s_uart_link_task_handle = NULL;
        LOG_ERROR("SRP UART2 worker creation failed\r\n");
    }
}
