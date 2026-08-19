#include "bsp_spi.h"

#include <stdio.h>

#include "main.h"
#include "stm32h7xx_hal_spi.h"
#include "bsp_gpio.h"
#include "bsp_timer.h"
#include "bsp_uart.h"

#define BSP_SPI1_KERNEL_HZ                 UINT32_C(240000000)
#define BSP_SPI_BMI323_RAW_PRESCALER       SPI_BAUDRATEPRESCALER_256
#define BSP_SPI_BMI323_RAW_HZ              \
    (BSP_SPI1_KERNEL_HZ / UINT32_C(256))
#define BSP_SPI_BMI323_CS_DELAY_US         UINT32_C(2)

static SPI_HandleTypeDef hspi1_bsp;
static uint8_t spi_ready;
static int32_t spi_last_hal_status = -1;
static uint32_t diag_count;
static uint8_t spi_first_access_diagnostics_valid;
static bsp_spi_first_access_diagnostics_t spi_first_access_diagnostics;

static void spi_log_first_access_before(void)
{
    char line[96];

    (void)snprintf(line, sizeof(line),
                   "[BMI323_SPI_STATE]\r\n"
                   "cs_active=%u\r\n"
                   "spi_state_before=%lu\r\n",
                   (unsigned)spi_first_access_diagnostics.cs_active,
                   (unsigned long)spi_first_access_diagnostics.spi_state_before);
    (void)bsp_uart_log_write_link_level(BSP_UART_LOG_LEVEL_ERROR, line);
    (void)snprintf(line, sizeof(line),
                   "[BMI323_SPI_STATE]\r\n"
                   "spi_error_before=0x%08lX\r\n"
                   "spi1_sr_before=0x%08lX\r\n",
                   (unsigned long)spi_first_access_diagnostics.spi_error_before,
                   (unsigned long)spi_first_access_diagnostics.spi_sr_before);
    (void)bsp_uart_log_write_link_level(BSP_UART_LOG_LEVEL_ERROR, line);
}

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

static void spi_delay_us(uint32_t delay_us)
{
    const uint64_t start = timer_get_us();

    while ((uint64_t)(timer_get_us() - start) < (uint64_t)delay_us) {
        /* The diagnostic CS setup/hold time is independent of the SPI clock. */
    }
}

static uint8_t spi_bmi323_cs_output_level(void)
{
    return (BMI323_CS_GPIO_Port->ODR & BMI323_CS_Pin) != 0U ? 1U : 0U;
}

static void spi_capture_bmi323_raw_before(bsp_spi_bmi323_raw_diagnostics_t *diagnostics)
{
    diagnostics->spi_state_before = (uint32_t)HAL_SPI_GetState(&hspi1_bsp);
    diagnostics->spi_error_before = HAL_SPI_GetError(&hspi1_bsp);
    diagnostics->spi_sr_before = SPI1->SR;
    diagnostics->spi_cfg1_before = SPI1->CFG1;
    diagnostics->spi_cfg2_before = SPI1->CFG2;
    diagnostics->spi_cr1_before = SPI1->CR1;
    diagnostics->spi_cr2_before = SPI1->CR2;
}

static void spi_capture_bmi323_raw_after(bsp_spi_bmi323_raw_diagnostics_t *diagnostics)
{
    diagnostics->spi_state_after = (uint32_t)HAL_SPI_GetState(&hspi1_bsp);
    diagnostics->spi_error_after = HAL_SPI_GetError(&hspi1_bsp);
    diagnostics->spi_sr_after = SPI1->SR;
    diagnostics->spi_cfg1_after = SPI1->CFG1;
    diagnostics->spi_cfg2_after = SPI1->CFG2;
    diagnostics->spi_cr1_after = SPI1->CR1;
    diagnostics->spi_cr2_after = SPI1->CR2;
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
    const uint8_t capture_first_access = diag_count == 0U ? 1U : 0U;

    if (tx_data == NULL || rx_data == NULL || size == 0U || size > UINT16_MAX) {
        return BSP_STATUS_INVALID_ARG;
    }
    if (spi_ready == 0U) {
        return BSP_STATUS_NOT_READY;
    }

    if (capture_first_access != 0U) {
        bsp_gpio_level_t cs_active = BSP_GPIO_LOW;

        spi_first_access_diagnostics = (bsp_spi_first_access_diagnostics_t){0};
        if (bsp_gpio_read(BSP_GPIO_BMI323_CS, &cs_active) == BSP_STATUS_OK) {
            spi_first_access_diagnostics.cs_active = (uint8_t)cs_active;
        }
        spi_first_access_diagnostics.spi_state_before =
            (uint32_t)HAL_SPI_GetState(&hspi1_bsp);
        spi_first_access_diagnostics.spi_error_before = HAL_SPI_GetError(&hspi1_bsp);
        spi_first_access_diagnostics.spi_sr_before = SPI1->SR;
        spi_log_first_access_before();
    }
    spi_last_hal_status = (int32_t)HAL_SPI_TransmitReceive(&hspi1_bsp,
                                                            (uint8_t *)tx_data, rx_data,
                                                            (uint16_t)size, timeout_ms);
    if (capture_first_access != 0U) {
        uint8_t tx_trace[4] = {0U};
        uint8_t rx_trace[4] = {0U};
        const size_t trace_length = size < sizeof(tx_trace) ? size : sizeof(tx_trace);
        char line[128];

        spi_first_access_diagnostics.spi_state_after =
            (uint32_t)HAL_SPI_GetState(&hspi1_bsp);
        spi_first_access_diagnostics.spi_error_after = HAL_SPI_GetError(&hspi1_bsp);
        spi_first_access_diagnostics.spi_sr_after = SPI1->SR;
        spi_first_access_diagnostics.hal_result = spi_last_hal_status;
        spi_first_access_diagnostics.rx0 = rx_data[0];
        spi_first_access_diagnostics.rx1 = size > 1U ? rx_data[1] : 0U;
        spi_first_access_diagnostics_valid = 1U;

        for (size_t index = 0U; index < trace_length; ++index) {
            tx_trace[index] = tx_data[index];
            rx_trace[index] = rx_data[index];
        }
        (void)snprintf(line, sizeof(line),
                       "[BMI323][SPI_TRACE]\r\n"
                       "len=%lu\r\n"
                       "tx:\r\n"
                       "%02X %02X %02X %02X\r\n"
                       "\r\n"
                       "rx:\r\n"
                       "%02X %02X %02X %02X\r\n"
                       "\r\n"
                       "hal=%ld\r\n",
                       (unsigned long)size,
                       (unsigned)tx_trace[0], (unsigned)tx_trace[1],
                       (unsigned)tx_trace[2], (unsigned)tx_trace[3],
                       (unsigned)rx_trace[0], (unsigned)rx_trace[1],
                       (unsigned)rx_trace[2], (unsigned)rx_trace[3],
                       (long)spi_last_hal_status);
        (void)bsp_uart_log_write_link_level(BSP_UART_LOG_LEVEL_ERROR, line);
        ++diag_count;
    }
    return spi_map_hal_status((HAL_StatusTypeDef)spi_last_hal_status);
}

int32_t bsp_spi_get_last_hal_status(void)
{
    return spi_last_hal_status;
}

uint8_t bsp_spi_get_first_access_diagnostics(
    bsp_spi_first_access_diagnostics_t *diagnostics)
{
    if (diagnostics == NULL || spi_first_access_diagnostics_valid == 0U) {
        return 0U;
    }
    *diagnostics = spi_first_access_diagnostics;
    return 1U;
}

bsp_status_t bsp_spi_bmi323_raw_transaction(
    const uint8_t *tx_data, uint8_t *rx_data, size_t size,
    size_t first_segment_length, uint32_t timeout_ms,
    bsp_spi_bmi323_raw_diagnostics_t *diagnostics)
{
    HAL_StatusTypeDef hal_status = HAL_ERROR;
    bsp_status_t result = BSP_STATUS_ERROR;
    const uint32_t cfg1_before_probe = SPI1->CFG1;

    if (tx_data == NULL || rx_data == NULL || diagnostics == NULL ||
        size == 0U || size > UINT16_MAX || first_segment_length >= size) {
        return BSP_STATUS_INVALID_ARG;
    }
    if (spi_ready == 0U || HAL_SPI_GetState(&hspi1_bsp) != HAL_SPI_STATE_READY) {
        return BSP_STATUS_NOT_READY;
    }

    *diagnostics = (bsp_spi_bmi323_raw_diagnostics_t){0};
    diagnostics->hal_status = -1;
    diagnostics->spi_hz = BSP_SPI_BMI323_RAW_HZ;
    spi_last_hal_status = -1;

    /* PC4 is explicitly released before every raw probe transaction. */
    result = bsp_gpio_write(BSP_GPIO_BMI323_CS, BSP_GPIO_HIGH);
    if (result != BSP_STATUS_OK) {
        return result;
    }
    spi_delay_us(BSP_SPI_BMI323_CS_DELAY_US);
    diagnostics->cs_before = spi_bmi323_cs_output_level();

    /* The probe temporarily uses 240 MHz / 256 = 937.5 kHz. */
    CLEAR_BIT(SPI1->CR1, SPI_CR1_SPE);
    MODIFY_REG(SPI1->CFG1, SPI_CFG1_MBR, BSP_SPI_BMI323_RAW_PRESCALER);
    spi_capture_bmi323_raw_before(diagnostics);

    result = bsp_gpio_write(BSP_GPIO_BMI323_CS, BSP_GPIO_LOW);
    if (result == BSP_STATUS_OK) {
        spi_delay_us(BSP_SPI_BMI323_CS_DELAY_US);
        diagnostics->cs_active = spi_bmi323_cs_output_level();
        if (first_segment_length == 0U) {
            hal_status = HAL_SPI_TransmitReceive(&hspi1_bsp, (uint8_t *)tx_data,
                                                 rx_data, (uint16_t)size,
                                                 timeout_ms);
        } else {
            hal_status = HAL_SPI_TransmitReceive(&hspi1_bsp, (uint8_t *)tx_data,
                                                 rx_data,
                                                 (uint16_t)first_segment_length,
                                                 timeout_ms);
            if (hal_status == HAL_OK) {
                hal_status = HAL_SPI_TransmitReceive(
                    &hspi1_bsp, (uint8_t *)&tx_data[first_segment_length],
                    &rx_data[first_segment_length],
                    (uint16_t)(size - first_segment_length), timeout_ms);
            }
        }
        spi_last_hal_status = (int32_t)hal_status;
        result = spi_map_hal_status(hal_status);
    }

    diagnostics->hal_status = spi_last_hal_status;
    diagnostics->hal_error = HAL_SPI_GetError(&hspi1_bsp);
    spi_capture_bmi323_raw_after(diagnostics);

    (void)bsp_gpio_write(BSP_GPIO_BMI323_CS, BSP_GPIO_HIGH);
    spi_delay_us(BSP_SPI_BMI323_CS_DELAY_US);
    diagnostics->cs_after = spi_bmi323_cs_output_level();

    CLEAR_BIT(SPI1->CR1, SPI_CR1_SPE);
    MODIFY_REG(SPI1->CFG1, SPI_CFG1_MBR, cfg1_before_probe & SPI_CFG1_MBR);
    return result;
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
