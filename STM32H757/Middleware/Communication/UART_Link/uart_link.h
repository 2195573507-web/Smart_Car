#ifndef UART_LINK_H
#define UART_LINK_H

#include <stddef.h>
#include <stdint.h>

#include "stm32h7xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

#define UART_LINK_USART USART2
#define UART_LINK_BAUD_RATE UINT32_C(115200)
#define UART_LINK_RX_RING_SIZE UINT16_C(512)
#define UART_LINK_TX_TIMEOUT_MS UINT32_C(20)

/* uart_link_send() preserves the HAL status; these aliases name its outcome. */
#define UART_TX_OK HAL_OK
#define UART_TX_FAIL HAL_ERROR

typedef struct {
    uint32_t uart_tx_count;
    uint32_t uart_tx_timeout;
    uint32_t uart_rx_bytes;
    uint32_t uart_rx_overflow;
    uint32_t uart_rx_drop;
    uint32_t uart_hal_error;
    uint16_t rx_buffered;
    uint16_t rx_buffer_capacity;
} uart_link_stats_t;

void uart_link_init(void);
uint8_t uart_link_is_ready(void);
uint32_t uart_link_get_last_rx_time(void);
HAL_StatusTypeDef uart_link_send(const uint8_t *data, uint16_t length);
size_t uart_link_read(uint8_t *data, size_t capacity);
void uart_link_get_stats(uart_link_stats_t *stats);
void uart_link_task(void *argument);
void uart_link_task_start(void);

#ifdef __cplusplus
}
#endif

#endif /* UART_LINK_H */
