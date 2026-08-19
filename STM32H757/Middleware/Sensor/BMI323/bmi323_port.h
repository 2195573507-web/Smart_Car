#ifndef SMARTCAR_SENSOR_BMI323_PORT_H
#define SMARTCAR_SENSOR_BMI323_PORT_H

#include <stdint.h>

#include "bsp_gpio.h"
#include "bsp_status.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Hardware-only BMI323 adapter. It does not interpret register addresses. */
typedef struct
{
    bsp_gpio_level_t cs_before;
    bsp_gpio_level_t cs_after;
    bsp_status_t cs_before_status;
    bsp_status_t cs_after_status;
} bmi323_port_trace_t;

bsp_status_t bmi323_port_init(void);
bsp_status_t bmi323_port_cs_low(void);
bsp_status_t bmi323_port_cs_high(void);
bsp_status_t bmi323_port_spi_read(const uint8_t *tx_data, uint8_t *rx_data,
                                  uint16_t length, uint32_t timeout_ms,
                                  bmi323_port_trace_t *trace);
bsp_status_t bmi323_port_spi_write(const uint8_t *tx_data, uint16_t length,
                                   uint32_t timeout_ms);
void bmi323_port_delay_us(uint32_t delay_us);
void bmi323_port_delay_ms(uint32_t delay_ms);
int32_t bmi323_port_get_last_hal_status(void);

#ifdef __cplusplus
}
#endif

#endif /* SMARTCAR_SENSOR_BMI323_PORT_H */
