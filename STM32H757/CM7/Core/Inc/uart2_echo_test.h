#ifndef UART2_ECHO_TEST_H
#define UART2_ECHO_TEST_H

#include <stdint.h>

#include "stm32h7xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

#define UART2_ECHO_TEST_BAUD_RATE UINT32_C(115200)

HAL_StatusTypeDef uart2_echo_test_init(void);
void uart2_echo_test_start(void);

#ifdef __cplusplus
}
#endif

#endif /* UART2_ECHO_TEST_H */
