#ifndef BSP_I2C_H
#define BSP_I2C_H

#include <stddef.h>
#include <stdint.h>
#include "bsp_status.h"

#ifdef __cplusplus
extern "C" {
#endif

bsp_status_t bsp_i2c_init(void);
bsp_status_t bsp_i2c_write(uint16_t address_7bit, const uint8_t *data,
                           size_t size, uint32_t timeout_ms);
bsp_status_t bsp_i2c_read(uint16_t address_7bit, uint8_t *data,
                          size_t size, uint32_t timeout_ms);
bsp_status_t bsp_i2c_write_read(uint16_t address_7bit, const uint8_t *tx_data,
                                size_t tx_size, uint8_t *rx_data,
                                size_t rx_size, uint32_t timeout_ms);
bsp_status_t bsp_i2c_probe(uint16_t address_7bit, uint32_t trials, uint32_t timeout_ms);

static inline bsp_status_t i2c_init(void) { return bsp_i2c_init(); }
static inline bsp_status_t i2c_write(uint16_t address_7bit, const uint8_t *data,
                                     size_t size, uint32_t timeout_ms)
{
    return bsp_i2c_write(address_7bit, data, size, timeout_ms);
}
static inline bsp_status_t i2c_read(uint16_t address_7bit, uint8_t *data,
                                    size_t size, uint32_t timeout_ms)
{
    return bsp_i2c_read(address_7bit, data, size, timeout_ms);
}
static inline bsp_status_t i2c_write_read(uint16_t address_7bit, const uint8_t *tx_data,
                                          size_t tx_size, uint8_t *rx_data,
                                          size_t rx_size, uint32_t timeout_ms)
{
    return bsp_i2c_write_read(address_7bit, tx_data, tx_size, rx_data, rx_size, timeout_ms);
}
static inline bsp_status_t i2c_probe(uint16_t address_7bit, uint32_t trials, uint32_t timeout_ms)
{
    return bsp_i2c_probe(address_7bit, trials, timeout_ms);
}

#ifdef __cplusplus
}
#endif

#endif /* BSP_I2C_H */
