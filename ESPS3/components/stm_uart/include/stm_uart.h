#ifndef STM_UART_H
#define STM_UART_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_err.h"

/* UART2 connection to STM32H757: S3 TX GPIO17 -> STM RX, S3 RX GPIO18 <- STM TX. */
#define STM_UART_PORT UART_NUM_2
#define STM_UART_TX_GPIO GPIO_NUM_17
#define STM_UART_RX_GPIO GPIO_NUM_18
#define STM_UART_BAUD_RATE 921600
#define STM_UART_RX_DRIVER_BUFFER_SIZE 8192U
#define STM_UART_TX_DRIVER_BUFFER_SIZE 2048U
#define STM_UART_BREAK_RECOVERY_THRESHOLD 20U

typedef struct {
    uint32_t rx_bytes;
    uint32_t tx_bytes;
    uint32_t overflow;
    uint32_t drop;
    uint32_t short_write;
    uint32_t hal_error;
    uint32_t tx_queue_drop;
    uint32_t sync_guard_drop;
    uint32_t rx_task_reads;
    uint32_t tx_write_errors;
    uint32_t rx_error_events;
    uint32_t break_events;
    uint32_t break_recoveries;
    uint16_t rx_buffered;
    uint16_t tx_queue_pending;
} stm_uart_stats_t;

esp_err_t stm_uart_init(void);
int stm_uart_send(const uint8_t *data, size_t len);
void stm_uart_get_stats(stm_uart_stats_t *stats);
void stm_uart_recover(void);
esp_err_t stm_uart_set_baud_rate(uint32_t baud_rate);
int stm_uart_receive_nonblock(uint8_t *buffer, size_t max_len);
int stm_uart_receive(uint8_t *buffer, size_t max_len);
/* Reports that bytes were dropped or a UART line error interrupted a frame. */
bool stm_uart_take_rx_discontinuity(void);
/* Reports one threshold crossing; the service task owns SRP recovery. */
bool stm_uart_take_break_recovery(void);
/* Motion frames are rejected in the transport until SRP sync is complete. */
void stm_uart_set_sync_state(bool synced);

#endif /* STM_UART_H */
