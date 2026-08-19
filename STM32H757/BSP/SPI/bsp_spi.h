#ifndef BSP_SPI_H
#define BSP_SPI_H

#include <stddef.h>
#include <stdint.h>
#include "bsp_status.h"
#include "bsp_gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t cs_active;
    uint8_t rx0;
    uint8_t rx1;
    uint32_t spi_state_before;
    uint32_t spi_error_before;
    uint32_t spi_sr_before;
    uint32_t spi_state_after;
    uint32_t spi_error_after;
    uint32_t spi_sr_after;
    int32_t hal_result;
} bsp_spi_first_access_diagnostics_t;

/* One bounded low-speed transaction used solely by the BMI323 wiring probe.
 * A nonzero first_segment_length keeps CS asserted across two HAL calls. */
typedef struct {
    uint8_t cs_before;
    uint8_t cs_active;
    uint8_t cs_after;
    uint32_t spi_state_before;
    uint32_t spi_error_before;
    uint32_t spi_sr_before;
    uint32_t spi_cfg1_before;
    uint32_t spi_cfg2_before;
    uint32_t spi_cr1_before;
    uint32_t spi_cr2_before;
    uint32_t spi_state_after;
    uint32_t spi_error_after;
    uint32_t spi_sr_after;
    uint32_t spi_cfg1_after;
    uint32_t spi_cfg2_after;
    uint32_t spi_cr1_after;
    uint32_t spi_cr2_after;
    int32_t hal_status;
    uint32_t hal_error;
    uint32_t spi_hz;
} bsp_spi_bmi323_raw_diagnostics_t;

bsp_status_t bsp_spi_init(void);
bsp_status_t bsp_spi_transmit(const uint8_t *data, size_t size, uint32_t timeout_ms);
bsp_status_t bsp_spi_receive(uint8_t *data, size_t size, uint32_t timeout_ms);
bsp_status_t bsp_spi_write_read(const uint8_t *tx_data, uint8_t *rx_data,
                                size_t size, uint32_t timeout_ms);

/* Raw HAL status from the most recent SPI HAL operation. */
int32_t bsp_spi_get_last_hal_status(void);

/* One-shot state captured around the first synchronous SPI write/read call. */
uint8_t bsp_spi_get_first_access_diagnostics(
    bsp_spi_first_access_diagnostics_t *diagnostics);

bsp_status_t bsp_spi_bmi323_raw_transaction(
    const uint8_t *tx_data, uint8_t *rx_data, size_t size,
    size_t first_segment_length, uint32_t timeout_ms,
    bsp_spi_bmi323_raw_diagnostics_t *diagnostics);

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
