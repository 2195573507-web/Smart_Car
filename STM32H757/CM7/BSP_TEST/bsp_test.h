#ifndef BSP_TEST_H
#define BSP_TEST_H

#include <stdint.h>
#include <stddef.h>

#include "bsp_status.h"
#include "bsp_pwm.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Compile-only API smoke test. It does not touch hardware or start peripherals. */
uint32_t bsp_test_compile(void);
bsp_status_t bsp_test_spi_loopback(const uint8_t *tx_data, uint8_t *rx_data,
                                   size_t size, uint32_t timeout_ms);
size_t bsp_test_i2c_scan(uint8_t *addresses, size_t capacity, uint32_t timeout_ms);
bsp_status_t bsp_test_uart_output(const char *text, uint32_t timeout_ms);
bsp_status_t bsp_test_pwm_output(bsp_pwm_channel_t channel, uint8_t duty_percent);

#ifdef __cplusplus
}
#endif

#endif /* BSP_TEST_H */
