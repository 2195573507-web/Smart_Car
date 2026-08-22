#ifndef MOTOR_BOARD_TRANSPORT_UART_H
#define MOTOR_BOARD_TRANSPORT_UART_H

#include <stdbool.h>
#include <stdint.h>

#include "stm32h7xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MB_TRANSPORT_BAUD_RATE UINT32_C(115200)
#define MB_TRANSPORT_RX_RING_SIZE UINT16_C(512)
#define MB_TRANSPORT_TX_RING_SIZE UINT16_C(512)

typedef struct {
    uint32_t rx_bytes;
    uint32_t rx_overflow;
    uint32_t tx_bytes;
    uint32_t tx_overflow;
    uint32_t uart_errors;
    uint16_t rx_buffered;
    uint16_t tx_buffered;
} mb_transport_stats_t;

/* The USART6 handle is initialized by the CM7 startup/MSP layer. */
void MB_Transport_Init(void);
bool MB_Transport_IsReady(void);
bool MB_Transport_Ensure_Rx_Active(void);
bool MB_Transport_Send(const uint8_t *data, uint16_t length);
bool MB_Transport_ReadByte(uint8_t *byte);
void MB_Transport_ClearRx(void);
void MB_Transport_GetStats(mb_transport_stats_t *stats);

/* Called only by USART6_IRQHandler. It never enters a critical section. */
void MB_Transport_IRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_BOARD_TRANSPORT_UART_H */
