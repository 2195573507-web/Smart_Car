#ifndef BSP_UART_H
#define BSP_UART_H

#include <stddef.h>
#include <stdint.h>
#include "bsp_status.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    BSP_UART_USART1 = 0,
    BSP_UART_USART6,
    BSP_UART_USART2
} bsp_uart_port_t;

typedef struct {
    uint32_t tx_count;
    uint32_t tx_fail;
    uint32_t tx_busy;
} bsp_uart_log_stats_t;

/* Log payload: source, level, timestamp, text length, and UTF-8 text. */
#define BSP_UART_LOG_LEVEL_DEBUG UINT8_C(0)
#define BSP_UART_LOG_LEVEL_INFO  UINT8_C(1)
#define BSP_UART_LOG_LEVEL_WARN  UINT8_C(2)
#define BSP_UART_LOG_LEVEL_ERROR UINT8_C(3)
#define BSP_UART_LOG_TEXT_MAX    UINT16_C(96)

bsp_status_t bsp_uart_init(bsp_uart_port_t port, uint32_t baud_rate);
bsp_status_t bsp_uart_transmit(bsp_uart_port_t port, const uint8_t *data,
                               size_t size, uint32_t timeout_ms);
bsp_status_t bsp_uart_receive(bsp_uart_port_t port, uint8_t *data,
                              size_t size, uint32_t timeout_ms);
bsp_status_t bsp_uart_transmit_dma(bsp_uart_port_t port, const uint8_t *data, size_t size);
bsp_status_t bsp_uart_receive_dma(bsp_uart_port_t port, uint8_t *data, size_t size);
bsp_status_t bsp_uart_log_write(const char *text, uint32_t timeout_ms);
bsp_status_t bsp_uart_log_write_level(uint8_t level, const char *text,
                                      uint32_t timeout_ms);
/* Sends an SRPv4 LOG frame through USART2 only. */
bsp_status_t bsp_uart_log_write_link_level(uint8_t level, const char *text);
bsp_status_t bsp_uart_get_log_stats(bsp_uart_log_stats_t *stats);

static inline bsp_status_t uart_init(bsp_uart_port_t port, uint32_t baud_rate)
{
    return bsp_uart_init(port, baud_rate);
}
static inline bsp_status_t uart_transmit(bsp_uart_port_t port, const uint8_t *data,
                                         size_t size, uint32_t timeout_ms)
{
    return bsp_uart_transmit(port, data, size, timeout_ms);
}
static inline bsp_status_t uart_receive(bsp_uart_port_t port, uint8_t *data,
                                        size_t size, uint32_t timeout_ms)
{
    return bsp_uart_receive(port, data, size, timeout_ms);
}
static inline bsp_status_t uart_transmit_dma(bsp_uart_port_t port, const uint8_t *data,
                                             size_t size)
{
    return bsp_uart_transmit_dma(port, data, size);
}
static inline bsp_status_t uart_receive_dma(bsp_uart_port_t port, uint8_t *data, size_t size)
{
    return bsp_uart_receive_dma(port, data, size);
}
static inline bsp_status_t uart_log_write(const char *text, uint32_t timeout_ms)
{
    return bsp_uart_log_write(text, timeout_ms);
}

#ifdef __cplusplus
}
#endif

#endif /* BSP_UART_H */
