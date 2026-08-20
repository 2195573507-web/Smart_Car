#include "bsp_uart.h"

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

static UART_HandleTypeDef *uart_handle(bsp_uart_port_t port)
{
    return port == BSP_UART_USART1 ? &huart1 : NULL;
}

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

bsp_status_t bsp_uart_transmit_dma(bsp_uart_port_t port, const uint8_t *data, size_t size)
{
    (void)port;
    (void)data;
    (void)size;
    /* DMA streams are not assigned to USART1/USART6 in the current IOC. */
    return BSP_STATUS_UNSUPPORTED;
}

bsp_status_t bsp_uart_receive_dma(bsp_uart_port_t port, uint8_t *data, size_t size)
{
    (void)port;
    (void)data;
    (void)size;
    return BSP_STATUS_UNSUPPORTED;
}

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

bsp_status_t bsp_uart_log_write_link_level(uint8_t level, const char *text)
{
    if (text == NULL || level > BSP_UART_LOG_LEVEL_ERROR) {
        return BSP_STATUS_INVALID_ARG;
    }
    uart_log_write_usart2(level, text);
    return BSP_STATUS_OK;
}

bsp_status_t bsp_uart_log_write(const char *text, uint32_t timeout_ms)
{
    return bsp_uart_log_write_level(BSP_UART_LOG_LEVEL_INFO, text, timeout_ms);
}

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

int __io_putchar(int ch)
{
    const uint8_t byte = (uint8_t)ch;

    return bsp_uart_transmit(BSP_UART_USART1, &byte, 1U, 100U) == BSP_STATUS_OK
               ? ch
               : EOF;
}
