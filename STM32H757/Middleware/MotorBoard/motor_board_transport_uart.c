#include "motor_board_transport_uart.h"

#include <stddef.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"

#include "log_service.h"

extern UART_HandleTypeDef huart6;

static volatile uint8_t s_ready;
static volatile uint8_t s_rx_ring[MB_TRANSPORT_RX_RING_SIZE];
static volatile uint8_t s_tx_ring[MB_TRANSPORT_TX_RING_SIZE];
static volatile uint16_t s_rx_head;
static volatile uint16_t s_rx_tail;
static volatile uint16_t s_rx_count;
static volatile uint16_t s_tx_head;
static volatile uint16_t s_tx_tail;
static volatile uint16_t s_tx_count;
static volatile uint32_t s_rx_bytes;
static volatile uint32_t s_rx_overflow;
static volatile uint32_t s_tx_bytes;
static volatile uint32_t s_tx_overflow;
static volatile uint32_t s_uart_errors;

static void transport_clear_error_flags(USART_TypeDef *instance,
                                        uint32_t isr)
{
    if (instance != NULL &&
        (isr & (USART_ISR_ORE | USART_ISR_FE | USART_ISR_NE |
                USART_ISR_PE)) != 0U) {
        instance->ICR = USART_ICR_ORECF | USART_ICR_FECF |
                        USART_ICR_NECF | USART_ICR_PECF;
        ++s_uart_errors;
    }
}

static bool transport_enable_register_rx(const char *reason)
{
    char line[SMARTCAR_LOG_MAX_PAYLOAD + 1U];
    USART_TypeDef *instance = huart6.Instance;
    const uint32_t required = USART_CR1_UE | USART_CR1_RE |
                              USART_CR1_TE | USART_CR1_RXNEIE_RXFNEIE;

    if (instance != USART6) {
        return false;
    }

    /* USART6 uses the register-level byte ring below; HAL receive transactions
     * never own the RXNE interrupt. */
    SET_BIT(instance->CR1, required);
    CLEAR_BIT(instance->CR3, USART_CR3_OVRDIS);
    SET_BIT(instance->CR1, USART_CR1_PEIE);
    SET_BIT(instance->CR3, USART_CR3_EIE);

    if ((instance->CR1 & required) != required) {
        (void)snprintf(line, sizeof(line),
                       "[MOTOR_BOARD] USART6 RX register arm failed reason=%s isr=%08lX cr1=%08lX",
                       reason == NULL ? "?" : reason,
                       (unsigned long)instance->ISR,
                       (unsigned long)instance->CR1);
        LOG_WARN(line);
        return false;
    }
    return true;
}

static uint16_t ring_next(uint16_t index, uint16_t capacity)
{
    ++index;
    return index == capacity ? 0U : index;
}

static void transport_disable_tx_irq(USART_TypeDef *instance)
{
    CLEAR_BIT(instance->CR1, USART_CR1_TXEIE_TXFNFIE);
}

static void transport_enable_tx_irq(USART_TypeDef *instance)
{
    SET_BIT(instance->CR1, USART_CR1_TXEIE_TXFNFIE);
}

static void transport_push_rx_byte(uint8_t byte)
{
    ++s_rx_bytes;
    if (s_rx_count < MB_TRANSPORT_RX_RING_SIZE) {
        s_rx_ring[s_rx_head] = byte;
        s_rx_head = ring_next(s_rx_head, MB_TRANSPORT_RX_RING_SIZE);
        ++s_rx_count;
    } else {
        ++s_rx_overflow;
    }
}

static void transport_reset_buffers(void)
{
    taskENTER_CRITICAL();
    s_rx_head = 0U;
    s_rx_tail = 0U;
    s_rx_count = 0U;
    s_tx_head = 0U;
    s_tx_tail = 0U;
    s_tx_count = 0U;
    taskEXIT_CRITICAL();
}

void MB_Transport_Init(void)
{
    s_ready = 0U;
    s_rx_bytes = 0U;
    s_rx_overflow = 0U;
    s_tx_bytes = 0U;
    s_tx_overflow = 0U;
    s_uart_errors = 0U;
    transport_reset_buffers();

    if (huart6.Instance != USART6) {
        LOG_WARN("[MOTOR_BOARD] USART6 HAL init not ready");
        return;
    }

    /* HAL_UART_Init() has configured the pins and baud rate. USART6 data
     * movement below does not read or write HAL receive-state bookkeeping. */
    transport_clear_error_flags(USART6, USART6->ISR);
    CLEAR_BIT(USART6->CR1, USART_CR1_TXEIE_TXFNFIE);
    __HAL_UART_ENABLE_IT(&huart6, UART_IT_RXNE);
    __HAL_UART_ENABLE_IT(&huart6, UART_IT_ERR);
    HAL_NVIC_SetPriority(USART6_IRQn, 5U, 0U);
    HAL_NVIC_EnableIRQ(USART6_IRQn);
    if (transport_enable_register_rx("init")) {
        s_ready = 1U;
    } else {
        LOG_WARN("[MOTOR_BOARD] USART6 RX not active after init");
    }
}

bool MB_Transport_IsReady(void)
{
    return s_ready != 0U;
}

bool MB_Transport_Ensure_Rx_Active(void)
{
    if (huart6.Instance != USART6) {
        return false;
    }

    /* The register-level receiver remains authoritative even if HAL bookkeeping
     * is stale; it cannot disable RXNE or block motor-board TX. */
    transport_clear_error_flags(USART6, USART6->ISR);
    if (transport_enable_register_rx("health")) {
        s_ready = 1U;
        return true;
    }
    return false;
}

bool MB_Transport_Send(const uint8_t *data, uint16_t length)
{
    uint16_t index;

    if (!MB_Transport_Ensure_Rx_Active() || data == NULL || length == 0U ||
        length > MB_TRANSPORT_TX_RING_SIZE) {
        return false;
    }

    taskENTER_CRITICAL();
    if ((uint16_t)(MB_TRANSPORT_TX_RING_SIZE - s_tx_count) < length) {
        ++s_tx_overflow;
        taskEXIT_CRITICAL();
        return false;
    }
    index = s_tx_head;
    for (uint16_t offset = 0U; offset < length; ++offset) {
        s_tx_ring[index] = data[offset];
        index = ring_next(index, MB_TRANSPORT_TX_RING_SIZE);
    }
    s_tx_head = index;
    s_tx_count = (uint16_t)(s_tx_count + length);
    taskEXIT_CRITICAL();

    transport_enable_tx_irq(USART6);
    return true;
}

bool MB_Transport_ReadByte(uint8_t *byte)
{
    if (byte == NULL) {
        return false;
    }
    taskENTER_CRITICAL();
    if (s_rx_count == 0U) {
        taskEXIT_CRITICAL();
        return false;
    }
    *byte = s_rx_ring[s_rx_tail];
    s_rx_tail = ring_next(s_rx_tail, MB_TRANSPORT_RX_RING_SIZE);
    --s_rx_count;
    taskEXIT_CRITICAL();
    return true;
}

void MB_Transport_ClearRx(void)
{
    taskENTER_CRITICAL();
    s_rx_head = 0U;
    s_rx_tail = 0U;
    s_rx_count = 0U;
    taskEXIT_CRITICAL();
}

void MB_Transport_GetStats(mb_transport_stats_t *stats)
{
    if (stats == NULL) {
        return;
    }
    taskENTER_CRITICAL();
    stats->rx_bytes = s_rx_bytes;
    stats->rx_overflow = s_rx_overflow;
    stats->tx_bytes = s_tx_bytes;
    stats->tx_overflow = s_tx_overflow;
    stats->uart_errors = s_uart_errors;
    stats->rx_buffered = s_rx_count;
    stats->tx_buffered = s_tx_count;
    taskEXIT_CRITICAL();
}

static void transport_service_tx(USART_TypeDef *instance, uint32_t isr)
{
    if ((instance->CR1 & USART_CR1_TXEIE_TXFNFIE) != 0U &&
        (isr & USART_ISR_TXE_TXFNF) != 0U) {
        if (s_tx_count != 0U) {
            instance->TDR = s_tx_ring[s_tx_tail];
            s_tx_tail = ring_next(s_tx_tail, MB_TRANSPORT_TX_RING_SIZE);
            --s_tx_count;
            ++s_tx_bytes;
        } else {
            transport_disable_tx_irq(instance);
        }
    }
}

void MB_Transport_IRQHandler(void)
{
    USART_TypeDef *instance = huart6.Instance;
    uint32_t isr;

    if (instance != USART6) {
        return;
    }

    isr = instance->ISR;
    transport_clear_error_flags(instance, isr);

    /* Drain all pending bytes. FIFO mode is disabled by MX_USART6_UART_Init(),
     * but the loop also handles bytes that arrived back-to-back before entry. */
    while ((instance->ISR & USART_ISR_RXNE_RXFNE) != 0U) {
        transport_push_rx_byte((uint8_t)(instance->RDR & 0xFFU));
    }

    transport_service_tx(instance, instance->ISR);
}
