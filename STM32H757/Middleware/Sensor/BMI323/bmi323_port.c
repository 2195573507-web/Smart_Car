#include "bmi323_port.h"

#include <stdio.h>

#include "FreeRTOS.h"
#include "main.h"
#include "stm32h7xx_hal_spi.h"
#include "task.h"
#include "bsp_gpio.h"
#include "bsp_spi.h"
#include "bsp_uart.h"
#include "imu_time.h"

#define BMI323_PORT_CS_DELAY_US UINT32_C(2)
#define BMI323_PORT_LOG_TIMEOUT_MS UINT32_C(100)
#define BMI323_PORT_DELAY_TRACE_LIMIT UINT8_C(8)

static uint8_t bmi323_port_trace_active;
static uint8_t bmi323_port_spi_config_logged;
static uint8_t bmi323_port_delay_trace_count;

static void bmi323_port_allow_hal_tick(void)
{
    /* BMI323 init runs before vTaskStartScheduler(). FreeRTOS's pre-scheduler
     * critical-section sentinel can leave BASEPRI raised after queue/mutex use;
     * release only that mask so HAL SPI timeout processing can observe SysTick. */
    if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED) {
        __set_BASEPRI(0U);
    }
}

static uint8_t bmi323_port_delay_trace_begin(void)
{
    if (bmi323_port_delay_trace_count >= BMI323_PORT_DELAY_TRACE_LIMIT) {
        return 0U;
    }
    ++bmi323_port_delay_trace_count;
    return 1U;
}

static void bmi323_port_log_delay_enter(uint32_t delay_us, uint32_t delay_ms)
{
    char line[80];

    (void)snprintf(line, sizeof(line),
                   "[BMI][D+] us=%lu ms=%lu time_ms=%lu\r\n",
                   (unsigned long)delay_us,
                   (unsigned long)delay_ms,
                   (unsigned long)imu_time_now_ms());
    (void)uart_log_write(line, BMI323_PORT_LOG_TIMEOUT_MS);
}

static void bmi323_port_log_delay_exit(void)
{
    char line[48];

    (void)snprintf(line, sizeof(line), "[BMI][D-] time_ms=%lu\r\n",
                   (unsigned long)imu_time_now_ms());
    (void)uart_log_write(line, BMI323_PORT_LOG_TIMEOUT_MS);
}

static void bmi323_port_log_spi_config(void)
{
    char line[96];

    if (bmi323_port_spi_config_logged != 0U) {
        return;
    }
    bmi323_port_spi_config_logged = 1U;
    (void)snprintf(line, sizeof(line),
                   "[BMI][SPI] inst=1 mode=0 cpol=0 cpha=0 bits=8 first=%u\r\n",
                   (unsigned)SPI_FIRSTBIT_MSB);
    (void)uart_log_write(line, BMI323_PORT_LOG_TIMEOUT_MS);
}

static void bmi323_port_log_cs(const char *state)
{
    char line[48];

    if (bmi323_port_trace_active == 0U || state == NULL) {
        return;
    }
    (void)snprintf(line, sizeof(line), "[BMI323][CS] %s\r\n", state);
    (void)uart_log_write(line, BMI323_PORT_LOG_TIMEOUT_MS);
}

static bsp_status_t bmi323_port_finish_transaction(bsp_status_t status,
                                                    bmi323_port_trace_t *trace)
{
    const bsp_status_t cs_status = bmi323_port_cs_high();

    bmi323_port_delay_us(BMI323_PORT_CS_DELAY_US);
    if (trace != NULL) {
        trace->cs_after_status = bsp_gpio_read(BSP_GPIO_BMI323_CS, &trace->cs_after);
        bmi323_port_trace_active = 0U;
    }
    if (status == BSP_STATUS_OK && cs_status != BSP_STATUS_OK) {
        return cs_status;
    }
    return status;
}

bsp_status_t bmi323_port_init(void)
{
    bsp_status_t status;

    bmi323_port_allow_hal_tick();
    status = bsp_gpio_init();
    if (status != BSP_STATUS_OK) {
        return status;
    }
    status = bmi323_port_cs_high();
    if (status != BSP_STATUS_OK) {
        return status;
    }
    status = bsp_spi_init();

    if (status != BSP_STATUS_OK) {
        return status;
    }
    bmi323_port_log_spi_config();
    return bmi323_port_cs_high();
}

bsp_status_t bmi323_port_cs_low(void)
{
    bmi323_port_allow_hal_tick();
    const bsp_status_t status = bsp_gpio_write(BSP_GPIO_BMI323_CS, BSP_GPIO_LOW);

    if (status == BSP_STATUS_OK) {
        bmi323_port_log_cs("BMI323_CS_LOW");
    }
    if (status == BSP_STATUS_OK) {
        bmi323_port_delay_us(BMI323_PORT_CS_DELAY_US);
    }
    return status;
}

bsp_status_t bmi323_port_cs_high(void)
{
    bmi323_port_allow_hal_tick();
    const bsp_status_t status = bsp_gpio_write(BSP_GPIO_BMI323_CS, BSP_GPIO_HIGH);

    if (status == BSP_STATUS_OK) {
        bmi323_port_log_cs("BMI323_CS_HIGH");
    }
    return status;
}

bsp_status_t bmi323_port_spi_read(const uint8_t *tx_data, uint8_t *rx_data,
                                  uint16_t length, uint32_t timeout_ms,
                                  bmi323_port_trace_t *trace)
{
    bsp_status_t status;

    bmi323_port_allow_hal_tick();
    if (tx_data == NULL || rx_data == NULL || length == 0U) {
        return BSP_STATUS_INVALID_ARG;
    }
    if (trace != NULL) {
        *trace = (bmi323_port_trace_t){
            .cs_before = BSP_GPIO_LOW,
            .cs_after = BSP_GPIO_LOW,
            .cs_before_status = BSP_STATUS_ERROR,
            .cs_after_status = BSP_STATUS_ERROR
        };
        trace->cs_before_status = bsp_gpio_read(BSP_GPIO_BMI323_CS, &trace->cs_before);
        bmi323_port_trace_active = 1U;
    }
    status = bmi323_port_cs_low();
    if (status == BSP_STATUS_OK) {
        status = bsp_spi_write_read(tx_data, rx_data, length, timeout_ms);
        bmi323_port_delay_us(BMI323_PORT_CS_DELAY_US);
    }
    return bmi323_port_finish_transaction(status, trace);
}

bsp_status_t bmi323_port_spi_write(const uint8_t *tx_data, uint16_t length,
                                   uint32_t timeout_ms)
{
    bsp_status_t status;

    bmi323_port_allow_hal_tick();
    if (tx_data == NULL || length == 0U) {
        return BSP_STATUS_INVALID_ARG;
    }
    status = bmi323_port_cs_low();
    if (status == BSP_STATUS_OK) {
        status = bsp_spi_transmit(tx_data, length, timeout_ms);
        bmi323_port_delay_us(BMI323_PORT_CS_DELAY_US);
    }
    return bmi323_port_finish_transaction(status, NULL);
}

void bmi323_port_delay_us(uint32_t delay_us)
{
    const uint8_t trace = bmi323_port_delay_trace_begin();

    bmi323_port_allow_hal_tick();
    if (trace != 0U) {
        bmi323_port_log_delay_enter(delay_us, delay_us / UINT32_C(1000));
    }
    bmi323_port_allow_hal_tick();
    const uint64_t start = imu_time_now_us();

    while ((uint64_t)(imu_time_now_us() - start) < (uint64_t)delay_us) {
        /* Keep CS setup and hold time independent of the SPI clock. */
    }
    if (trace != 0U) {
        bmi323_port_log_delay_exit();
    }
}

void bmi323_port_delay_ms(uint32_t delay_ms)
{
    const uint8_t trace = bmi323_port_delay_trace_begin();
    const uint64_t delay_us = (uint64_t)delay_ms * UINT64_C(1000);
    const uint32_t trace_delay_us = delay_us > UINT32_MAX
                                        ? UINT32_MAX
                                        : (uint32_t)delay_us;

    bmi323_port_allow_hal_tick();
    if (trace != 0U) {
        bmi323_port_log_delay_enter(trace_delay_us, delay_ms);
    }
    const uint64_t start = imu_time_now_us();

    while ((uint64_t)(imu_time_now_us() - start) < delay_us) {
        /* BMI323 reset settling time shares the IMU monotonic time base. */
    }
    if (trace != 0U) {
        bmi323_port_log_delay_exit();
    }
}

int32_t bmi323_port_get_last_hal_status(void)
{
    return bsp_spi_get_last_hal_status();
}
