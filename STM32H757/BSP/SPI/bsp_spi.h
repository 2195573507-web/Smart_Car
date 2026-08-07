#ifndef BSP_SPI_H
#define BSP_SPI_H

#include <stddef.h>
#include <stdint.h>
#include "bsp_status.h"
#include "bsp_gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

bsp_status_t bsp_spi_init(void);
bsp_status_t bsp_spi_transmit(const uint8_t *data, size_t size, uint32_t timeout_ms);
bsp_status_t bsp_spi_receive(uint8_t *data, size_t size, uint32_t timeout_ms);
bsp_status_t bsp_spi_write_read(const uint8_t *tx_data, uint8_t *rx_data,
                                size_t size, uint32_t timeout_ms);

/* Raw HAL status from the most recent SPI HAL operation. */
int32_t bsp_spi_get_last_hal_status(void);

/* Read the physical MISO input while SPI1 remains configured for AF5. */
bsp_status_t bsp_spi_read_miso_level(bsp_gpio_level_t *level);

static inline bsp_status_t spi_init(void) { return bsp_spi_init(); }
static inline bsp_status_t spi_transmit(const uint8_t *data, size_t size, uint32_t timeout_ms)
{
    return bsp_spi_transmit(data, size, timeout_ms);
}
static inline bsp_status_t spi_receive(uint8_t *data, size_t size, uint32_t timeout_ms)
{
    return bsp_spi_receive(data, size, timeout_ms);
}
static inline bsp_status_t spi_write_read(const uint8_t *tx_data, uint8_t *rx_data,
                                          size_t size, uint32_t timeout_ms)
{
    return bsp_spi_write_read(tx_data, rx_data, size, timeout_ms);
}

static inline int32_t spi_get_last_hal_status(void)
{
    return bsp_spi_get_last_hal_status();
}

static inline bsp_status_t spi_read_miso_level(bsp_gpio_level_t *level)
{
    return bsp_spi_read_miso_level(level);
}

#ifdef __cplusplus
}
#endif

#endif /* BSP_SPI_H */
