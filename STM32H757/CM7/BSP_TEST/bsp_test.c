#include "bsp_test.h"

#include "bsp_adc.h"
#include "bsp_gpio.h"
#include "bsp_i2c.h"
#include "bsp_pwm.h"
#include "bsp_spi.h"
#include "bsp_timer.h"
#include "bsp_uart.h"

uint32_t bsp_test_compile(void)
{
    bsp_gpio_level_t level = BSP_GPIO_LOW;
    uint8_t tx[1] = {0U};
    uint8_t rx[1] = {0U};
    uint16_t adc_value = 0U;
    uint64_t timestamp = timer_get_us();

    (void)gpio_read(BSP_GPIO_BMI323_INT1, &level);
    (void)spi_write_read(tx, rx, sizeof(tx), 1U);
    (void)i2c_write_read(0x19U, tx, sizeof(tx), rx, sizeof(rx), 1U);
    (void)pwm_set_duty(BSP_PWM_CHANNEL_1, 0U);
    (void)uart_transmit_dma(BSP_UART_USART6, tx, sizeof(tx));
    (void)adc_read(0U, &adc_value);
    return (uint32_t)timestamp;
}

bsp_status_t bsp_test_spi_loopback(const uint8_t *tx_data, uint8_t *rx_data,
                                   size_t size, uint32_t timeout_ms)
{
    if (tx_data == NULL || rx_data == NULL || size == 0U) {
        return BSP_STATUS_INVALID_ARG;
    }
    return spi_write_read(tx_data, rx_data, size, timeout_ms);
}

size_t bsp_test_i2c_scan(uint8_t *addresses, size_t capacity, uint32_t timeout_ms)
{
    size_t count = 0U;
    uint16_t address;
    for (address = 0x08U; address <= 0x77U; ++address) {
        if (i2c_probe(address, 2U, timeout_ms) == BSP_STATUS_OK) {
            if (addresses != NULL && count < capacity) {
                addresses[count] = (uint8_t)address;
            }
            ++count;
        }
    }
    return count;
}

bsp_status_t bsp_test_uart_output(const char *text, uint32_t timeout_ms)
{
    return uart_log_write(text, timeout_ms);
}

bsp_status_t bsp_test_pwm_output(bsp_pwm_channel_t channel, uint8_t duty_percent)
{
    bsp_status_t status = pwm_init();
    if (status != BSP_STATUS_OK) {
        return status;
    }
    status = pwm_set_duty(channel, duty_percent);
    if (status != BSP_STATUS_OK) {
        return status;
    }
    return pwm_start(channel);
}
