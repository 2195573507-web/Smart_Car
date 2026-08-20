#include "uart_link.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include "log_service.h"

#define UART_LINK_TASK_STACK_WORDS UINT16_C(384)
#define UART_LINK_TASK_PRIORITY (tskIDLE_PRIORITY + 1U)
#define UART_LINK_STACK_MONITOR_PERIOD_MS UINT32_C(5000)
#define UART_LINK_IRQ_PRIORITY UINT32_C(5)
#define UART_LINK_DCACHE_LINE_SIZE UINT32_C(32)

static UART_HandleTypeDef s_handle;
static DMA_HandleTypeDef s_rx_dma;
static SemaphoreHandle_t s_tx_mutex;
static volatile uint8_t s_ready;
static volatile uint8_t s_dma_active;
static volatile uint8_t s_restart_requested;
static volatile uint8_t s_recovering;
static uint8_t s_ring[UART_LINK_RX_RING_SIZE];
static uint8_t s_dma_rx[UART_LINK_RX_DMA_SIZE]
    __attribute__((section(".dma_buffer"), aligned(32)));
static volatile uint16_t s_head;
static volatile uint16_t s_tail;
static volatile uint16_t s_count;
static volatile uint32_t s_rx_bytes;
static volatile uint32_t s_rx_frames;
static volatile uint32_t s_last_rx_time;
static volatile uint32_t s_rx_overflow_count;
static volatile uint32_t s_rx_drop_bytes;
static volatile uint32_t s_tx_count;
static volatile uint32_t s_tx_timeout_count;
static volatile uint32_t s_hal_error_count;
static TaskHandle_t s_uart_link_task_handle;

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

static uint16_t ring_push_from_isr(const uint8_t *data, uint16_t length)
{
    UBaseType_t mask;
    uint16_t dropped;

    if (data == NULL || length == 0U) {
        return 0U;
    }
    mask = taskENTER_CRITICAL_FROM_ISR();
    dropped = ring_push_locked(data, length);
    s_rx_bytes += length;
    ++s_rx_frames;
    s_last_rx_time = HAL_GetTick();
    taskEXIT_CRITICAL_FROM_ISR(mask);
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
    s_rx_dma.Init.Mode = DMA_NORMAL;
    s_rx_dma.Init.Priority = DMA_PRIORITY_HIGH;
    s_rx_dma.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&s_rx_dma) != HAL_OK) {
        return 0U;
    }
    __HAL_LINKDMA(&s_handle, hdmarx, s_rx_dma);
    HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, UART_LINK_IRQ_PRIORITY, 0U);
    HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
    HAL_NVIC_SetPriority(USART2_IRQn, UART_LINK_IRQ_PRIORITY, 0U);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
    return 1U;
}

static uint8_t uart_link_configure_uart(void)
{
    s_handle.Instance = UART_LINK_USART;
    s_handle.Init.BaudRate = UART_LINK_BAUD_RATE;
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
    (void)HAL_UARTEx_SetTxFifoThreshold(&s_handle, UART_TXFIFO_THRESHOLD_1_8);
    (void)HAL_UARTEx_SetRxFifoThreshold(&s_handle, UART_RXFIFO_THRESHOLD_1_8);
    (void)HAL_UARTEx_EnableFifoMode(&s_handle);
    return 1U;
}

static uint8_t uart_link_start_dma_receive(void)
{
    HAL_StatusTypeDef status;

    s_dma_active = 0U;
    dcache_invalidate(s_dma_rx, UART_LINK_RX_DMA_SIZE);
    __HAL_UART_CLEAR_FLAG(&s_handle,
                          UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_PEF |
                              UART_CLEAR_FEF);
    status = HAL_UARTEx_ReceiveToIdle_DMA(&s_handle, s_dma_rx,
                                           UART_LINK_RX_DMA_SIZE);
    if (status != HAL_OK) {
        ++s_hal_error_count;
        __HAL_UART_CLEAR_FLAG(&s_handle,
                              UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_PEF |
                                  UART_CLEAR_FEF);
        return 0U;
    }
    /* ReceiveToIdle reports half-transfer events by default; only IDLE/full events
       terminate and rearm this normal-mode transaction. */
    __HAL_DMA_DISABLE_IT(&s_rx_dma, DMA_IT_HT);
    s_dma_active = 1U;
    return 1U;
}

void uart_link_init(void)
{
    (void)memset(&s_handle, 0, sizeof(s_handle));
    s_head = 0U;
    s_tail = 0U;
    s_count = 0U;
    s_rx_bytes = 0U;
    s_rx_frames = 0U;
    s_last_rx_time = 0U;
    s_rx_overflow_count = 0U;
    s_rx_drop_bytes = 0U;
    s_tx_count = 0U;
    s_tx_timeout_count = 0U;
    s_hal_error_count = 0U;
    s_dma_active = 0U;
    s_restart_requested = 0U;
    s_recovering = 0U;
    s_tx_mutex = xSemaphoreCreateMutex();
    if (s_tx_mutex == NULL || uart_link_configure_uart() == 0U ||
        uart_link_configure_dma() == 0U || uart_link_start_dma_receive() == 0U) {
        s_ready = 0U;
        ++s_hal_error_count;
        return;
    }
    s_ready = 1U;
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
    const uint32_t start_ms = HAL_GetTick();
    uint32_t elapsed_ms;

    if (s_ready == 0U || data == NULL || length == 0U || s_tx_mutex == NULL) {
        ++s_hal_error_count;
        return UART_TX_FAIL;
    }
    if (xSemaphoreTake(s_tx_mutex, pdMS_TO_TICKS(UART_LINK_TX_TIMEOUT_MS)) != pdTRUE) {
        ++s_tx_timeout_count;
        return HAL_TIMEOUT;
    }
    elapsed_ms = HAL_GetTick() - start_ms;
    if (elapsed_ms >= UART_LINK_TX_TIMEOUT_MS) {
        (void)xSemaphoreGive(s_tx_mutex);
        ++s_tx_timeout_count;
        return HAL_TIMEOUT;
    }
    status = HAL_UART_Transmit(&s_handle, (uint8_t *)data, length,
                               UART_LINK_TX_TIMEOUT_MS - elapsed_ms);
    (void)xSemaphoreGive(s_tx_mutex);
    if (status == HAL_OK) {
        ++s_tx_count;
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
    stats->uart_tx_count = s_tx_count;
    stats->uart_tx_timeout = s_tx_timeout_count;
    stats->uart_rx_bytes = s_rx_bytes;
    stats->uart_rx_overflow = s_rx_overflow_count;
    stats->uart_rx_drop = s_rx_drop_bytes;
    stats->uart_hal_error = s_hal_error_count;
    stats->rx_buffered = s_count;
    stats->rx_buffer_capacity = UART_LINK_RX_RING_SIZE;
    taskEXIT_CRITICAL();
}

void uart_link_recover(void)
{
    if (s_tx_mutex == NULL || s_recovering != 0U) {
        return;
    }
    if (xSemaphoreTake(s_tx_mutex, pdMS_TO_TICKS(UART_LINK_TX_TIMEOUT_MS)) != pdTRUE) {
        s_restart_requested = 1U;
        return;
    }
    s_recovering = 1U;
    s_ready = 0U;
    s_dma_active = 0U;
    (void)HAL_UART_Abort(&s_handle);
    (void)HAL_DMA_Abort(&s_rx_dma);
    (void)HAL_DMA_DeInit(&s_rx_dma);
    (void)HAL_UART_DeInit(&s_handle);
    if (uart_link_configure_uart() != 0U && uart_link_configure_dma() != 0U &&
        uart_link_start_dma_receive() != 0U) {
        s_ready = 1U;
        s_restart_requested = 0U;
    } else {
        ++s_hal_error_count;
    }
    s_recovering = 0U;
    (void)xSemaphoreGive(s_tx_mutex);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    if (huart != &s_handle || s_dma_active == 0U || size == 0U ||
        size > UART_LINK_RX_DMA_SIZE) {
        return;
    }
    s_dma_active = 0U;
    dcache_invalidate(s_dma_rx, size);
    (void)ring_push_from_isr(s_dma_rx, size);
    if (uart_link_start_dma_receive() == 0U) {
        s_restart_requested = 1U;
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == &s_handle) {
        ++s_hal_error_count;
        __HAL_UART_CLEAR_FLAG(&s_handle,
                              UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_PEF |
                                  UART_CLEAR_FEF);
        s_dma_active = 0U;
        s_restart_requested = 1U;
    }
}

void uart_link_handle_dma_rx_irq(void)
{
    HAL_DMA_IRQHandler(&s_rx_dma);
}

void uart_link_handle_usart_irq(void)
{
    HAL_UART_IRQHandler(&s_handle);
}

static void uart_link_log_stack(void)
{
    const UBaseType_t free_stack = s_uart_link_task_handle == NULL
                                       ? 0U
                                       : uxTaskGetStackHighWaterMark(
                                             s_uart_link_task_handle);
    char line[96U];

    (void)snprintf(line, sizeof(line),
                   "[UART2_DMA_STACK] free_words=%lu rx=%lu frame_events=%lu hal_errors=%lu\r\n",
                   (unsigned long)free_stack, (unsigned long)s_rx_bytes,
                   (unsigned long)s_rx_frames, (unsigned long)s_hal_error_count);
    LOG_INFO(line);
}

void uart_link_task(void *argument)
{
    uint32_t last_stack_monitor_ms;

    (void)argument;
    last_stack_monitor_ms = HAL_GetTick();
    for (;;) {
        if (s_restart_requested != 0U) {
            uart_link_recover();
        }
        if ((uint32_t)(HAL_GetTick() - last_stack_monitor_ms) >=
            UART_LINK_STACK_MONITOR_PERIOD_MS) {
            last_stack_monitor_ms = HAL_GetTick();
            uart_link_log_stack();
        }
        vTaskDelay(pdMS_TO_TICKS(1U));
    }
}

void uart_link_task_start(void)
{
    if (s_ready == 0U || s_uart_link_task_handle != NULL) {
        return;
    }
    if (xTaskCreate(uart_link_task, "uart_link", UART_LINK_TASK_STACK_WORDS,
                    NULL, UART_LINK_TASK_PRIORITY, &s_uart_link_task_handle) != pdPASS) {
        s_uart_link_task_handle = NULL;
        LOG_ERROR("UART2 DMA worker creation failed\r\n");
    }
}
