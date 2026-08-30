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

#ifndef SMARTCAR_RAW_DIAGNOSTICS
#define SMARTCAR_RAW_DIAGNOSTICS 0
#endif

#define UART_LINK_TASK_STACK_WORDS UINT16_C(1024)
#define UART_LINK_TASK_PRIORITY (tskIDLE_PRIORITY + 2U)
#define UART_LINK_STACK_MONITOR_PERIOD_MS UINT32_C(5000)
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

static void uart_link_enable_irqs(void)
{
    HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, UART_LINK_IRQ_PRIORITY, 0U);
    HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
    HAL_NVIC_SetPriority(USART2_IRQn, UART_LINK_IRQ_PRIORITY, 0U);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
}

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

static void uart_link_clear_error_flags(void)
{
    __HAL_UART_CLEAR_FLAG(&s_handle,
                          UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_PEF |
                              UART_CLEAR_FEF);
}

static void uart_link_reset_state(void)
{
    taskENTER_CRITICAL();
    s_head = 0U;
    s_tail = 0U;
    s_count = 0U;
    s_tx_active = 0U;
    taskEXIT_CRITICAL();
}

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

uint8_t uart_link_is_ready(void)
{
    return s_ready;
}

uint32_t uart_link_get_last_rx_time(void)
{
    uint32_t last_rx_time;

    taskENTER_CRITICAL();
    last_rx_time = s_last_rx_time;
    taskEXIT_CRITICAL();
    return last_rx_time;
}

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

HAL_StatusTypeDef uart_link_set_baud_rate(uint32_t baud_rate)
{
    if (baud_rate == 0U || s_ready == 0U) {
        return HAL_ERROR;
    }
    s_baud_rate = baud_rate;
    uart_link_recover();
    return s_ready != 0U ? HAL_OK : HAL_ERROR;
}

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

void uart_link_handle_dma_rx_irq(void)
{
    ++s_dma_rx_irq_count;
    uart_link_snapshot_dma_state();
    HAL_DMA_IRQHandler(&s_rx_dma);
}

void uart_link_handle_dma_tx_irq(void)
{
    /* DMA1 Stream1 TX is intentionally unused in the compatibility path. */
}

void uart_link_handle_usart_irq(void)
{
    ++s_usart_irq_count;
    uart_link_snapshot_dma_state();
    HAL_UART_IRQHandler(&s_handle);
}

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
