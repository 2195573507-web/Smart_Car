#include "bsp_spi.h"

#include "main.h"
#include "stm32h7xx_hal_spi.h"
#include "bsp_gpio.h"

static SPI_HandleTypeDef hspi1_bsp;
static uint8_t spi_ready;
static int32_t spi_last_hal_status = -1;

static bsp_status_t spi_map_hal_status(HAL_StatusTypeDef status)
{
    if (status == HAL_OK) {
        return BSP_STATUS_OK;
    }
    if (status == HAL_TIMEOUT) {
        return BSP_STATUS_TIMEOUT;
    }
    return BSP_STATUS_ERROR;
}

static void spi_hw_init(void)
{
    GPIO_InitTypeDef config = {0};
    __HAL_RCC_SPI1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    config.Pin = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
    config.Mode = GPIO_MODE_AF_PP;
    config.Pull = GPIO_NOPULL;
    config.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    config.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &config);
}

bsp_status_t bsp_spi_init(void)
{
    if (spi_ready != 0U) {
        return BSP_STATUS_OK;
    }
    if (bsp_gpio_init() != BSP_STATUS_OK) {
        return BSP_STATUS_ERROR;
    }
    spi_hw_init();
    hspi1_bsp.Instance = SPI1;
    hspi1_bsp.Init.Mode = SPI_MODE_MASTER;
    hspi1_bsp.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1_bsp.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1_bsp.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi1_bsp.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi1_bsp.Init.NSS = SPI_NSS_SOFT;
    /* SPI123 kernel clock is 240 MHz; /128 gives about 1.875 MHz for diagnosis. */
    hspi1_bsp.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
    hspi1_bsp.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1_bsp.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1_bsp.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1_bsp.Init.CRCPolynomial = 7;
    hspi1_bsp.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
    hspi1_bsp.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
    hspi1_bsp.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
    hspi1_bsp.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
    hspi1_bsp.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
    hspi1_bsp.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
    hspi1_bsp.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
    hspi1_bsp.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
    hspi1_bsp.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
    hspi1_bsp.Init.IOSwap = SPI_IO_SWAP_DISABLE;
    spi_last_hal_status = (int32_t)HAL_SPI_Init(&hspi1_bsp);
    if (spi_last_hal_status != (int32_t)HAL_OK) {
        return BSP_STATUS_ERROR;
    }
    (void)bsp_gpio_write(BSP_GPIO_BMI323_CS, BSP_GPIO_HIGH);
    spi_ready = 1U;
    return BSP_STATUS_OK;
}

bsp_status_t bsp_spi_transmit(const uint8_t *data, size_t size, uint32_t timeout_ms)
{
    if (data == NULL || size == 0U || size > UINT16_MAX) {
        return BSP_STATUS_INVALID_ARG;
    }
    if (spi_ready == 0U) {
        return BSP_STATUS_NOT_READY;
    }
    spi_last_hal_status = (int32_t)HAL_SPI_Transmit(&hspi1_bsp, (uint8_t *)data,
                                                    (uint16_t)size, timeout_ms);
    return spi_map_hal_status((HAL_StatusTypeDef)spi_last_hal_status);
}

bsp_status_t bsp_spi_receive(uint8_t *data, size_t size, uint32_t timeout_ms)
{
    if (data == NULL || size == 0U || size > UINT16_MAX) {
        return BSP_STATUS_INVALID_ARG;
    }
    if (spi_ready == 0U) {
        return BSP_STATUS_NOT_READY;
    }
    spi_last_hal_status = (int32_t)HAL_SPI_Receive(&hspi1_bsp, data, (uint16_t)size, timeout_ms);
    return spi_map_hal_status((HAL_StatusTypeDef)spi_last_hal_status);
}

bsp_status_t bsp_spi_write_read(const uint8_t *tx_data, uint8_t *rx_data,
                                size_t size, uint32_t timeout_ms)
{
    if (tx_data == NULL || rx_data == NULL || size == 0U || size > UINT16_MAX) {
        return BSP_STATUS_INVALID_ARG;
    }
    if (spi_ready == 0U) {
        return BSP_STATUS_NOT_READY;
    }
    spi_last_hal_status = (int32_t)HAL_SPI_TransmitReceive(&hspi1_bsp,
                                                            (uint8_t *)tx_data, rx_data,
                                                            (uint16_t)size, timeout_ms);
    return spi_map_hal_status((HAL_StatusTypeDef)spi_last_hal_status);
}

int32_t bsp_spi_get_last_hal_status(void)
{
    return spi_last_hal_status;
}

bsp_status_t bsp_spi_read_miso_level(bsp_gpio_level_t *level)
{
    if (level == NULL) {
        return BSP_STATUS_INVALID_ARG;
    }
    if (spi_ready == 0U) {
        return BSP_STATUS_NOT_READY;
    }
    *level = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) == GPIO_PIN_SET
                 ? BSP_GPIO_HIGH : BSP_GPIO_LOW;
    return BSP_STATUS_OK;
}
