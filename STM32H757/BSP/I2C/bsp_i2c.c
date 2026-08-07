#include "bsp_i2c.h"

#include "main.h"

extern I2C_HandleTypeDef hi2c4;

static uint8_t i2c_ready;

static bsp_status_t i2c_map_hal_status(HAL_StatusTypeDef status)
{
    if (status == HAL_OK) {
        return BSP_STATUS_OK;
    }
    if (status == HAL_TIMEOUT) {
        return BSP_STATUS_TIMEOUT;
    }
    return BSP_STATUS_ERROR;
}

static bsp_status_t i2c_validate_address(uint16_t address_7bit)
{
    return address_7bit <= 0x7FU ? BSP_STATUS_OK : BSP_STATUS_INVALID_ARG;
}

bsp_status_t bsp_i2c_init(void)
{
    if (hi2c4.Instance != I2C4) {
        return BSP_STATUS_NOT_READY;
    }
    if (HAL_I2C_GetState(&hi2c4) != HAL_I2C_STATE_READY) {
        return BSP_STATUS_NOT_READY;
    }
    i2c_ready = 1U;
    return BSP_STATUS_OK;
}

bsp_status_t bsp_i2c_write(uint16_t address_7bit, const uint8_t *data,
                           size_t size, uint32_t timeout_ms)
{
    if (i2c_validate_address(address_7bit) != BSP_STATUS_OK || data == NULL ||
        size == 0U || size > UINT16_MAX) {
        return BSP_STATUS_INVALID_ARG;
    }
    if (i2c_ready == 0U) {
        return BSP_STATUS_NOT_READY;
    }
    return i2c_map_hal_status(HAL_I2C_Master_Transmit(&hi2c4, address_7bit << 1,
                                                      (uint8_t *)data, (uint16_t)size,
                                                      timeout_ms));
}

bsp_status_t bsp_i2c_read(uint16_t address_7bit, uint8_t *data,
                          size_t size, uint32_t timeout_ms)
{
    if (i2c_validate_address(address_7bit) != BSP_STATUS_OK || data == NULL ||
        size == 0U || size > UINT16_MAX) {
        return BSP_STATUS_INVALID_ARG;
    }
    if (i2c_ready == 0U) {
        return BSP_STATUS_NOT_READY;
    }
    return i2c_map_hal_status(HAL_I2C_Master_Receive(&hi2c4, address_7bit << 1,
                                                     data, (uint16_t)size, timeout_ms));
}

bsp_status_t bsp_i2c_write_read(uint16_t address_7bit, const uint8_t *tx_data,
                                size_t tx_size, uint8_t *rx_data,
                                size_t rx_size, uint32_t timeout_ms)
{
    HAL_StatusTypeDef hal_status;
    if (i2c_validate_address(address_7bit) != BSP_STATUS_OK || tx_data == NULL ||
        rx_data == NULL || tx_size == 0U || rx_size == 0U ||
        tx_size > UINT16_MAX || rx_size > UINT16_MAX) {
        return BSP_STATUS_INVALID_ARG;
    }
    if (i2c_ready == 0U) {
        return BSP_STATUS_NOT_READY;
    }
    hal_status = HAL_I2C_Master_Transmit(&hi2c4, address_7bit << 1,
                                         (uint8_t *)tx_data, (uint16_t)tx_size,
                                         timeout_ms);
    if (hal_status != HAL_OK) {
        return i2c_map_hal_status(hal_status);
    }
    return i2c_map_hal_status(HAL_I2C_Master_Receive(&hi2c4, address_7bit << 1,
                                                     rx_data, (uint16_t)rx_size,
                                                     timeout_ms));
}

bsp_status_t bsp_i2c_probe(uint16_t address_7bit, uint32_t trials, uint32_t timeout_ms)
{
    if (i2c_validate_address(address_7bit) != BSP_STATUS_OK || trials == 0U) {
        return BSP_STATUS_INVALID_ARG;
    }
    if (i2c_ready == 0U) {
        return BSP_STATUS_NOT_READY;
    }
    return i2c_map_hal_status(HAL_I2C_IsDeviceReady(&hi2c4, address_7bit << 1,
                                                    trials, timeout_ms));
}
