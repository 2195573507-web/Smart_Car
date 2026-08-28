#ifndef UART_LINK_H
#define UART_LINK_H

#include <stddef.h>
#include <stdint.h>

#include "srp_def.h"
#include "stm32h7xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

#define UART_LINK_USART USART2
#define UART_LINK_BAUD_RATE UINT32_C(921600)
#define UART_LINK_RX_RING_SIZE UINT16_C(2048)
#define UART_LINK_RX_DMA_SIZE UINT16_C(512)

#define UART_TX_OK HAL_OK
#define UART_TX_FAIL HAL_ERROR

typedef struct {
    uint32_t uart_tx_count;
    uint32_t uart_tx_bytes;
    uint32_t uart_tx_timeout;
    uint32_t uart_rx_bytes;
    uint32_t uart_rx_overflow;
    uint32_t uart_rx_drop;
    uint32_t uart_hal_error;
    uint32_t uart_tx_queue_drop;
    uint32_t uart_tx_preemptions;
    uint32_t uart_rx_events;
    uint32_t uart_rx_event_bytes;
    uint32_t uart_rx_rearms;
    uint32_t uart_rx_rearm_failures;
    uint32_t uart_tx_dma_starts;
    uint32_t uart_tx_dma_errors;
    uint32_t uart_tx_gstate;
    uint32_t uart_tx_error_code;
    uint32_t uart_tx_busy_recoveries;
    uint32_t uart_usart_errors;
    uint32_t uart_last_error_code;
    uint32_t uart_dma_rx_irqs;
    uint32_t uart_dma_tx_irqs;
    uint32_t uart_usart_irqs;
    uint32_t uart_rx_callback_rejects;
    uint32_t uart_rx_dma_error_code;
    uint32_t uart_rx_dmamux_request;
    uint32_t uart_tx_dmamux_request;
    uint32_t uart_rx_dma_cr;
    uint32_t uart_rx_gpio_state;
    uint16_t uart_rx_dma_ndtr;
    uint8_t uart_rx_active;
    uint16_t rx_buffered;
    uint16_t rx_buffer_capacity;
} uart_link_stats_t;

void uart_link_init(void);
uint8_t uart_link_is_ready(void);
uint32_t uart_link_get_last_rx_time(void);
HAL_StatusTypeDef uart_link_send(const uint8_t *data, uint16_t length);
/* Drop queued/in-flight TX data so a fresh sync response is sent first. */
void uart_link_flush_tx(void);
size_t uart_link_read(uint8_t *data, size_t capacity);
void uart_link_get_stats(uart_link_stats_t *stats);
void uart_link_recover(void);
HAL_StatusTypeDef uart_link_set_baud_rate(uint32_t baud_rate);
void uart_link_handle_dma_rx_irq(void);
void uart_link_handle_dma_tx_irq(void);
void uart_link_handle_usart_irq(void);
void uart_link_task(void *argument);
void uart_link_task_start(void);

#ifdef __cplusplus
}
#endif

#endif /* UART_LINK_H */
