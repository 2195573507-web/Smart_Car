#include "uart_link.h"

#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include "log_service.h"

#define UART_LINK_RX_CHUNK_SIZE UINT16_C(128)
#define UART_LINK_RX_TIMEOUT_MS UINT32_C(5)
#define UART_LINK_TASK_STACK_WORDS UINT16_C(384)
#define UART_LINK_TASK_PRIORITY (tskIDLE_PRIORITY + 1U)
#define UART_LINK_STACK_MONITOR_PERIOD_MS UINT32_C(5000)

static UART_HandleTypeDef s_handle;
static SemaphoreHandle_t s_tx_mutex;
static uint8_t s_ready;
static uint8_t s_ring[UART_LINK_RX_RING_SIZE];
static volatile uint16_t s_head;
static volatile uint16_t s_tail;
static volatile uint16_t s_count;
static uint32_t s_rx_bytes;
static uint32_t s_rx_frames;
static uint32_t s_last_rx_time;
static TaskHandle_t s_uart_link_task_handle;

static void uart_link_log_stack(void)
{
    char line[80];
    const UBaseType_t free_stack = s_uart_link_task_handle == NULL
                                       ? 0U
                                       : uxTaskGetStackHighWaterMark(
                                             s_uart_link_task_handle);

    (void)snprintf(line, sizeof(line),
                   "[TASK_STACK]\r\ntask=uart_link_task\r\nfree_stack=%lu\r\n",
                   (unsigned long)free_stack);
    LOG_INFO(line);
}
static uint32_t s_rx_overflow_count;
static uint32_t s_rx_drop_bytes;
static uint32_t s_tx_count;
static uint32_t s_tx_timeout_count;
static uint32_t s_hal_error_count;

static uint16_t ring_push(const uint8_t *data, uint16_t length)
{
    uint16_t dropped = 0U;

    taskENTER_CRITICAL();
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
    taskEXIT_CRITICAL();
    return dropped;
}

void uart_link_init(void)
{
    memset(&s_handle, 0, sizeof(s_handle));
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
    s_ready = HAL_UART_Init(&s_handle) == HAL_OK ? 1U : 0U;
    if (s_ready != 0U) {
        (void)HAL_UARTEx_SetTxFifoThreshold(&s_handle, UART_TXFIFO_THRESHOLD_1_8);
        (void)HAL_UARTEx_SetRxFifoThreshold(&s_handle, UART_RXFIFO_THRESHOLD_1_8);
        (void)HAL_UARTEx_EnableFifoMode(&s_handle);
        s_tx_mutex = xSemaphoreCreateMutex();
        if (s_tx_mutex == NULL) {
            s_ready = 0U;
        }
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
    const uint32_t start_ms = HAL_GetTick();

    if (s_ready == 0U || data == NULL || length == 0U || s_tx_mutex == NULL) {
        taskENTER_CRITICAL();
        ++s_hal_error_count;
        taskEXIT_CRITICAL();
        return UART_TX_FAIL;
    }
    if (xSemaphoreTake(s_tx_mutex, pdMS_TO_TICKS(UART_LINK_TX_TIMEOUT_MS)) != pdTRUE) {
        taskENTER_CRITICAL();
        ++s_tx_timeout_count;
        taskEXIT_CRITICAL();
        return HAL_TIMEOUT;
    }
    const uint32_t elapsed_ms = (uint32_t)(HAL_GetTick() - start_ms);
    if (elapsed_ms >= UART_LINK_TX_TIMEOUT_MS) {
        (void)xSemaphoreGive(s_tx_mutex);
        taskENTER_CRITICAL();
        ++s_tx_timeout_count;
        taskEXIT_CRITICAL();
        return HAL_TIMEOUT;
    }
    status = HAL_UART_Transmit(&s_handle, (uint8_t *)data, length,
                               UART_LINK_TX_TIMEOUT_MS - elapsed_ms);
    (void)xSemaphoreGive(s_tx_mutex);
    taskENTER_CRITICAL();
    if (status == HAL_OK) {
        ++s_tx_count;
    } else if (status == HAL_TIMEOUT) {
        ++s_tx_timeout_count;
    } else {
        ++s_hal_error_count;
    }
    taskEXIT_CRITICAL();
    return status;
}

size_t uart_link_read(uint8_t *data, size_t capacity)
{
    if (data == NULL || capacity == 0U) {
        return 0U;
    }
    taskENTER_CRITICAL();
    size_t length = s_count < capacity ? s_count : capacity;
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

void uart_link_task(void *argument)
{
    uint32_t last_stack_monitor_ms;

    (void)argument;
    last_stack_monitor_ms = HAL_GetTick();
    uint8_t chunk[UART_LINK_RX_CHUNK_SIZE];
    for (;;) {
        uint16_t received = 0U;
        const HAL_StatusTypeDef status = HAL_UARTEx_ReceiveToIdle(
            &s_handle, chunk, sizeof(chunk), &received, UART_LINK_RX_TIMEOUT_MS);
        if (received != 0U) {
            const uint16_t dropped = ring_push(chunk, received);
            taskENTER_CRITICAL();
            s_rx_bytes += received;
            taskEXIT_CRITICAL();
            ++s_rx_frames;
            taskENTER_CRITICAL();
            s_last_rx_time = HAL_GetTick();
            taskEXIT_CRITICAL();
            if (dropped != 0U) {
                uart_link_stats_t stats;
                uart_link_get_stats(&stats);
                char line[96];
                (void)snprintf(line, sizeof(line),
                               "UART_RX_OVERFLOW dropped=%u uart_rx_overflow=%lu uart_rx_drop=%lu\r\n",
                               (unsigned)dropped,
                               (unsigned long)stats.uart_rx_overflow,
                               (unsigned long)stats.uart_rx_drop);
                LOG_WARN(line);
            }
        }
        if (status != HAL_OK && status != HAL_TIMEOUT) {
            taskENTER_CRITICAL();
            ++s_hal_error_count;
            taskEXIT_CRITICAL();
            (void)HAL_UART_AbortReceive(&s_handle);
        }
        if ((uint32_t)(HAL_GetTick() - last_stack_monitor_ms) >=
            UART_LINK_STACK_MONITOR_PERIOD_MS) {
            last_stack_monitor_ms = HAL_GetTick();
            uart_link_log_stack();
        }
        taskYIELD();
    }
}

void uart_link_task_start(void)
{
    if (s_ready == 0U) {
        LOG_ERROR("uart_link_task start result=NOT_READY");
        return;
    }
    const BaseType_t result =
        xTaskCreate(uart_link_task, "uart_link", UART_LINK_TASK_STACK_WORDS,
                    NULL, UART_LINK_TASK_PRIORITY, &s_uart_link_task_handle);
    char line[64];
    (void)snprintf(line, sizeof(line), "uart_link_task start result=%ld\r\n",
                   (long)result);
    if (result == pdPASS) {
        LOG_INFO(line);
    } else {
        LOG_ERROR(line);
    }
}
