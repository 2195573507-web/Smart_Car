#include "bmi323.h"

#include <stddef.h>
#include <stdio.h>

#include "main.h"
#include "bsp_gpio.h"
#include "bsp_spi.h"
#include "bsp_timer.h"
#include "bsp_uart.h"

#define BMI323_REG_CHIP_ID       UINT8_C(0x00)
#define BMI323_REG_ACC_DATA_X    UINT8_C(0x03)
#define BMI323_REG_GYR_DATA_X    UINT8_C(0x06)
#define BMI323_REG_TEMP_DATA     UINT8_C(0x09)
#define BMI323_REG_ACC_CONF      UINT8_C(0x20)
#define BMI323_REG_GYR_CONF      UINT8_C(0x21)
#define BMI323_REG_IO_INT_CTRL   UINT8_C(0x38)
#define BMI323_REG_INT_MAP2      UINT8_C(0x3B)
#define BMI323_REG_CMD            UINT8_C(0x7E)

#define BMI323_SPI_READ           UINT8_C(0x80)
#define BMI323_CMD_SOFT_RESET_LSB UINT8_C(0xAF)
#define BMI323_CMD_SOFT_RESET_MSB UINT8_C(0xDE)
#define BMI323_SPI_TIMEOUT_MS     UINT32_C(20)
#define BMI323_LOG_TIMEOUT_MS     UINT32_C(100)
#define BMI323_CS_SETUP_US        UINT32_C(10)
#define BMI323_CS_HOLD_US         UINT32_C(2)
#define BMI323_POWER_ON_DELAY_MS  UINT32_C(10)
#define BMI323_SPI_MODE_WAIT_MS   UINT32_C(10)
#define BMI323_RESET_DELAY_MS     UINT32_C(2)

#define BMI323_ACC_RANGE_G        4.0f
#define BMI323_GYRO_RANGE_DPS     2000.0f
#define BMI323_GRAVITY_MPS2       9.80665f
#define BMI323_DEG_TO_RAD         0.01745329251994329577f

static uint8_t bmi323_ready;
static uint8_t bmi323_chip_id;

static void bmi323_log_status(const char *label, bsp_status_t status)
{
    char line[96];

    if (label == NULL) {
        return;
    }
    (void)snprintf(line, sizeof(line), "%s status=%d\r\n", label, (int)status);
    (void)uart_log_write(line, BMI323_LOG_TIMEOUT_MS);
}

static void bmi323_log_hal_status(int32_t status)
{
    char line[96];

    (void)snprintf(line, sizeof(line), "HAL_SPI status: %ld\r\n", (long)status);
    (void)uart_log_write(line, BMI323_LOG_TIMEOUT_MS);
}

static void bmi323_log_gpio_level(const char *label, bsp_gpio_level_t level)
{
    char line[96];

    if (label == NULL) {
        return;
    }
    (void)snprintf(line, sizeof(line), "%s=%s\r\n", label,
                   level == BSP_GPIO_HIGH ? "HIGH" : "LOW");
    (void)uart_log_write(line, BMI323_LOG_TIMEOUT_MS);
}

static uint32_t bmi323_gpio_mode(GPIO_TypeDef *port, uint32_t pin)
{
    return (port->MODER >> (pin * 2U)) & 0x3U;
}

static uint32_t bmi323_gpio_af(GPIO_TypeDef *port, uint32_t pin)
{
    return (port->AFR[pin / 8U] >> ((pin % 8U) * 4U)) & 0xFU;
}

static const char *bmi323_gpio_mode_name(GPIO_TypeDef *port, uint32_t pin)
{
    const uint32_t mode = bmi323_gpio_mode(port, pin);

    if (mode == 0x1U) {
        return (port->OTYPER & (UINT32_C(1) << pin)) != 0U
                   ? "OUTPUT_OD" : "OUTPUT_PP";
    }
    if (mode == 0x2U) {
        return (port->OTYPER & (UINT32_C(1) << pin)) != 0U
                   ? "AF_OD" : "AF_PP";
    }
    return mode == 0x3U ? "ANALOG" : "INPUT";
}

static void bmi323_log_gpio_config(void)
{
    char line[256];
    bsp_gpio_level_t cs_level = BSP_GPIO_LOW;
    const bsp_status_t cs_status = bsp_gpio_read(BSP_GPIO_BMI323_CS, &cs_level);
    const uint32_t pa5_mode = bmi323_gpio_mode(GPIOA, 5U);
    const uint32_t pa6_mode = bmi323_gpio_mode(GPIOA, 6U);
    const uint32_t pa7_mode = bmi323_gpio_mode(GPIOA, 7U);
    const uint32_t pc4_mode = bmi323_gpio_mode(GPIOC, 4U);

    (void)snprintf(line, sizeof(line),
                   "BMI_CS GPIO STATUS: %s (read_status=%d)\r\n"
                   "BMI323 GPIO CONFIG:\r\n"
                   "PA5: Mode=%s(0x%lu) AF=%lu\r\n"
                   "PA6: Mode=%s(0x%lu) AF=%lu\r\n"
                   "PA7: Mode=%s(0x%lu) AF=%lu\r\n"
                   "PC4: Mode=%s(0x%lu) AF=N/A\r\n"
                   "BMI323 RESET PIN: NONE (software command only)\r\n",
                   cs_level == BSP_GPIO_HIGH ? "HIGH" : "LOW", (int)cs_status,
                   bmi323_gpio_mode_name(GPIOA, 5U), (unsigned long)pa5_mode,
                   (unsigned long)bmi323_gpio_af(GPIOA, 5U),
                   bmi323_gpio_mode_name(GPIOA, 6U), (unsigned long)pa6_mode,
                   (unsigned long)bmi323_gpio_af(GPIOA, 6U),
                   bmi323_gpio_mode_name(GPIOA, 7U), (unsigned long)pa7_mode,
                   (unsigned long)bmi323_gpio_af(GPIOA, 7U),
                   bmi323_gpio_mode_name(GPIOC, 4U), (unsigned long)pc4_mode);
    (void)uart_log_write(line, BMI323_LOG_TIMEOUT_MS);
}

static void bmi323_log_bytes(const char *label, const uint8_t *data, size_t length)
{
    char line[128];
    size_t offset = 0U;

    if (label == NULL || data == NULL || length == 0U) {
        return;
    }

    offset = (size_t)snprintf(line, sizeof(line), "%s", label);
    if (offset >= sizeof(line)) {
        offset = sizeof(line) - 1U;
    }
    for (size_t index = 0U; index < length && offset < (sizeof(line) - 1U); ++index) {
        const int written = snprintf(line + offset, sizeof(line) - offset,
                                     index == 0U ? " 0x%02X" : " %02X",
                                     data[index]);
        if (written <= 0) {
            break;
        }
        if ((size_t)written >= sizeof(line) - offset) {
            offset = sizeof(line) - 1U;
            break;
        }
        offset += (size_t)written;
    }
    (void)snprintf(line + offset, sizeof(line) - offset, "\r\n");
    (void)uart_log_write(line, BMI323_LOG_TIMEOUT_MS);
}

static void bmi323_log_chip_id_read(const uint8_t *rx, uint16_t chip_id)
{
    char line[128];

    if (rx == NULL) {
        return;
    }
    (void)snprintf(line, sizeof(line),
                   "BMI323 SPI READ RAW RX:\r\n"
                   "0x%02X 0x%02X 0x%02X 0x%02X\r\n\r\n"
                   "BMI323 CHIP_ID:\r\n"
                   "0x%04X\r\n",
                   rx[0], rx[1], rx[2], rx[3], chip_id);
    (void)uart_log_write(line, BMI323_LOG_TIMEOUT_MS);
}

static void bmi323_delay_ms(uint32_t delay_ms)
{
    const uint32_t start = timer_get_ms();
    while ((uint32_t)(timer_get_ms() - start) < delay_ms) {
        /* Reset settling delay; the normal path is only two milliseconds. */
    }
}

static void bmi323_delay_us(uint32_t delay_us)
{
    const uint64_t start = timer_get_us();
    while ((uint64_t)(timer_get_us() - start) < (uint64_t)delay_us) {
        /* Keep CS setup/hold timing independent of the SPI clock. */
    }
}

static bsp_status_t bmi323_enter_spi_mode(uint8_t diagnostic)
{
    const uint8_t tx[2] = {UINT8_C(0x7F), UINT8_C(0x00)};
    uint8_t rx[sizeof(tx)] = {0U};
    bsp_status_t status;
    bsp_status_t cs_status;

    if (diagnostic != 0U) {
        (void)uart_log_write("BMI323 SPI MODE SWITCH START\r\n", BMI323_LOG_TIMEOUT_MS);
    }

    status = bsp_gpio_write(BSP_GPIO_BMI323_CS, BSP_GPIO_HIGH);
    if (status != BSP_STATUS_OK) {
        return status;
    }
    bmi323_delay_ms(BMI323_SPI_MODE_WAIT_MS);
    if (diagnostic != 0U) {
        (void)uart_log_write("BMI323 CS HIGH WAIT DONE\r\n", BMI323_LOG_TIMEOUT_MS);
        (void)uart_log_write("BMI323 SPI MODE TEST\r\n", BMI323_LOG_TIMEOUT_MS);
    }

    bmi323_delay_us(BMI323_CS_SETUP_US);
    status = bsp_gpio_write(BSP_GPIO_BMI323_CS, BSP_GPIO_LOW);
    if (status == BSP_STATUS_OK) {
        bmi323_delay_us(BMI323_CS_SETUP_US);
        status = bsp_spi_write_read(tx, rx, sizeof(tx), BMI323_SPI_TIMEOUT_MS);
        bmi323_delay_us(BMI323_CS_HOLD_US);
    }
    cs_status = bsp_gpio_write(BSP_GPIO_BMI323_CS, BSP_GPIO_HIGH);
    bmi323_delay_us(BMI323_CS_HOLD_US);

    if (diagnostic != 0U) {
        bmi323_log_bytes("BMI323 SPI MODE TEST TX:", tx, sizeof(tx));
        bmi323_log_bytes("BMI323 SPI MODE TEST RX:", rx, sizeof(rx));
        bmi323_log_status("BMI323 SPI MODE TEST", status);
        bmi323_log_status("BMI323 SPI MODE TEST CS HIGH", cs_status);
        bmi323_log_hal_status(bsp_spi_get_last_hal_status());
    }
    if (status == BSP_STATUS_OK && cs_status != BSP_STATUS_OK) {
        status = cs_status;
    }
    return status;
}

static bsp_status_t bmi323_read_regs(uint8_t reg, uint8_t *data, size_t length,
                                      uint8_t diagnostic)
{
    uint8_t tx[1U + 1U + 26U] = {0};
    uint8_t rx[1U + 1U + 26U] = {0};
    bsp_status_t status;
    bsp_status_t cs_high_status;
    bsp_status_t cs_low_status;
    bsp_status_t cs_status;

    if (data == NULL || length == 0U || length > 26U) {
        return BSP_STATUS_INVALID_ARG;
    }

    tx[0] = (uint8_t)(reg | BMI323_SPI_READ);
    /* Explicitly frame every transaction: idle high, setup delay, then low. */
    cs_high_status = bsp_gpio_write(BSP_GPIO_BMI323_CS, BSP_GPIO_HIGH);
    bmi323_delay_us(BMI323_CS_SETUP_US);
    cs_low_status = bsp_gpio_write(BSP_GPIO_BMI323_CS, BSP_GPIO_LOW);
    status = cs_high_status != BSP_STATUS_OK ? cs_high_status : cs_low_status;
    if (status == BSP_STATUS_OK) {
        bmi323_delay_us(BMI323_CS_SETUP_US);
        status = bsp_spi_write_read(tx, rx, length + 2U, BMI323_SPI_TIMEOUT_MS);
        bmi323_delay_us(BMI323_CS_HOLD_US);
    }
    cs_status = bsp_gpio_write(BSP_GPIO_BMI323_CS, BSP_GPIO_HIGH);
    bmi323_delay_us(BMI323_CS_HOLD_US);
    if (diagnostic != 0U) {
        /* Keep UART output outside the CS-low window so it cannot stretch a transaction. */
        bmi323_log_bytes("SPI TX:", tx, length + 2U);
        bmi323_log_bytes("SPI RX:", rx, length + 2U);
        bmi323_log_hal_status(bsp_spi_get_last_hal_status());
        bmi323_log_status("BMI323 CS HIGH PRE", cs_high_status);
        bmi323_log_status("BMI323 CS LOW", cs_low_status);
        bmi323_log_status("BMI323 CS HIGH", cs_status);
    }
    if (status == BSP_STATUS_OK && cs_status != BSP_STATUS_OK) {
        status = cs_status;
    }
    if (diagnostic != 0U && status != BSP_STATUS_OK) {
        bmi323_log_status("BMI323 SPI READ FAIL", status);
    }

    if (status == BSP_STATUS_OK) {
        /* BMI323 SPI reads return one command/dummy byte before payload. */
        for (size_t index = 0U; index < length; ++index) {
            data[index] = rx[index + 2U];
        }
    }
    return status;
}

static bsp_status_t bmi323_read_chip_id_test(uint16_t *chip_id)
{
    const uint8_t tx[4] = {BMI323_SPI_READ | BMI323_REG_CHIP_ID, 0U, 0U, 0U};
    uint8_t rx[sizeof(tx)] = {0U};
    const uint8_t *payload = &rx[2];
    bsp_status_t status;
    bsp_status_t cs_high_status;
    bsp_status_t cs_low_status;
    bsp_status_t cs_status;

    if (chip_id == NULL) {
        return BSP_STATUS_INVALID_ARG;
    }

    cs_high_status = bsp_gpio_write(BSP_GPIO_BMI323_CS, BSP_GPIO_HIGH);
    bmi323_delay_us(BMI323_CS_SETUP_US);
    cs_low_status = bsp_gpio_write(BSP_GPIO_BMI323_CS, BSP_GPIO_LOW);
    status = cs_high_status != BSP_STATUS_OK ? cs_high_status : cs_low_status;
    if (status == BSP_STATUS_OK) {
        bmi323_delay_us(BMI323_CS_SETUP_US);
        status = bsp_spi_write_read(tx, rx, sizeof(tx), BMI323_SPI_TIMEOUT_MS);
        bmi323_delay_us(BMI323_CS_HOLD_US);
    }
    cs_status = bsp_gpio_write(BSP_GPIO_BMI323_CS, BSP_GPIO_HIGH);
    bmi323_delay_us(BMI323_CS_HOLD_US);

    if (status == BSP_STATUS_OK && cs_status != BSP_STATUS_OK) {
        status = cs_status;
    }
    if (status == BSP_STATUS_OK) {
        *chip_id = (uint16_t)payload[0] | ((uint16_t)payload[1] << 8);
    }
    bmi323_log_chip_id_read(rx, status == BSP_STATUS_OK ? *chip_id : 0U);
    bmi323_log_hal_status(bsp_spi_get_last_hal_status());
    bmi323_log_status("BMI323 CS HIGH PRE", cs_high_status);
    bmi323_log_status("BMI323 CS LOW", cs_low_status);
    bmi323_log_status("BMI323 CS HIGH", cs_status);
    return status;
}

static bsp_status_t bmi323_write_regs(uint8_t reg, const uint8_t *data, size_t length,
                                      uint8_t diagnostic)
{
    uint8_t tx[1U + 2U] = {0};
    bsp_status_t status;
    bsp_status_t cs_high_status;
    bsp_status_t cs_status;

    if (data == NULL || length == 0U || length > 2U) {
        return BSP_STATUS_INVALID_ARG;
    }

    tx[0] = (uint8_t)(reg & UINT8_C(0x7F));
    for (size_t index = 0U; index < length; ++index) {
        tx[index + 1U] = data[index];
    }

    cs_high_status = bsp_gpio_write(BSP_GPIO_BMI323_CS, BSP_GPIO_HIGH);
    bmi323_delay_us(BMI323_CS_SETUP_US);
    status = bsp_gpio_write(BSP_GPIO_BMI323_CS, BSP_GPIO_LOW);
    if (cs_high_status == BSP_STATUS_OK && status == BSP_STATUS_OK) {
        bmi323_delay_us(BMI323_CS_SETUP_US);
        status = bsp_spi_transmit(tx, length + 1U, BMI323_SPI_TIMEOUT_MS);
        bmi323_delay_us(BMI323_CS_HOLD_US);
    }
    cs_status = bsp_gpio_write(BSP_GPIO_BMI323_CS, BSP_GPIO_HIGH);
    bmi323_delay_us(BMI323_CS_HOLD_US);
    if (status == BSP_STATUS_OK && cs_status != BSP_STATUS_OK) {
        status = cs_status;
    }
    if (diagnostic != 0U) {
        bmi323_log_bytes("SPI WRITE TX:", tx, length + 1U);
        bmi323_log_status("BMI323 SPI WRITE", status);
        bmi323_log_hal_status(bsp_spi_get_last_hal_status());
        bmi323_log_status("BMI323 CS HIGH PRE", cs_high_status);
        bmi323_log_status("BMI323 CS HIGH POST", cs_status);
    }
    return status;
}

static int16_t bmi323_s16(const uint8_t *data)
{
    return (int16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
}

static void bmi323_scale_acc(Vector3f *acc, const uint8_t *raw)
{
    const float scale = (BMI323_ACC_RANGE_G * BMI323_GRAVITY_MPS2) / 32768.0f;
    acc->x = (float)bmi323_s16(&raw[0]) * scale;
    acc->y = (float)bmi323_s16(&raw[2]) * scale;
    acc->z = (float)bmi323_s16(&raw[4]) * scale;
}

static void bmi323_scale_gyro(Vector3f *gyro, const uint8_t *raw)
{
    const float scale = (BMI323_GYRO_RANGE_DPS / 32768.0f) * BMI323_DEG_TO_RAD;
    gyro->x = (float)bmi323_s16(&raw[0]) * scale;
    gyro->y = (float)bmi323_s16(&raw[2]) * scale;
    gyro->z = (float)bmi323_s16(&raw[4]) * scale;
}

static bsp_status_t bmi323_init_internal(uint8_t diagnostic)
{
    uint16_t chip_id = 0U;
    uint8_t id = 0U;
    bsp_gpio_level_t cs_level;
    uint8_t reset_dummy_id[2] = {0U};
    uint8_t reset_cmd[2] = {BMI323_CMD_SOFT_RESET_LSB, BMI323_CMD_SOFT_RESET_MSB};
    /*
     * ACC_CONF: 100 Hz, +/-4 g, ODR/2 bandwidth, normal mode.
     * GYR_CONF: 100 Hz, +/-2000 dps, ODR/2 bandwidth, normal mode.
     * The configuration registers are 16-bit and transmitted LSB first.
     */
    uint8_t acc_conf[2] = {UINT8_C(0x18), UINT8_C(0x40)};
    uint8_t gyr_conf[2] = {UINT8_C(0x48), UINT8_C(0x40)};
    /* INT1 active-high push-pull output enabled. */
    uint8_t int_ctrl[2] = {UINT8_C(0x05), UINT8_C(0x00)};
    /* Map accelerometer and gyro data-ready to INT1 (INT_MAP2[15:8] = 0x05). */
    uint8_t int_map2[2] = {UINT8_C(0x00), UINT8_C(0x05)};
    bsp_status_t status;

    bmi323_ready = 0U;
    if (diagnostic != 0U) {
        (void)uart_log_write("BMI323 SPI TEST START\r\n", BMI323_LOG_TIMEOUT_MS);
    }
    status = bsp_spi_init();
    if (diagnostic != 0U) {
        bmi323_log_status("BMI323 SPI INIT", status);
        (void)uart_log_write("BMI323 SPI CLOCK=1.875MHz\r\n"
                             "BMI323 SPI PINS: PA5=SCK(AF5) PA6=MISO(AF5) PA7=MOSI(AF5) "
                             "PC4=CS PB2=INT1\r\n",
                             BMI323_LOG_TIMEOUT_MS);
    }
    if (status != BSP_STATUS_OK) {
        return status;
    }
    if (diagnostic != 0U) {
        bmi323_log_gpio_config();
    }
    if (diagnostic != 0U) {
        (void)uart_log_write("BMI323 POWER-ON DELAY BEGIN 10ms\r\n",
                             BMI323_LOG_TIMEOUT_MS);
    }
    bmi323_delay_ms(BMI323_POWER_ON_DELAY_MS);
    if (diagnostic != 0U) {
        (void)uart_log_write("BMI323 POWER-ON DELAY END\r\n",
                             BMI323_LOG_TIMEOUT_MS);
    }
    status = bmi323_enter_spi_mode(diagnostic);
    if (status != BSP_STATUS_OK) {
        return status;
    }
    if (diagnostic != 0U &&
        bsp_gpio_read(BSP_GPIO_BMI323_CS, &cs_level) == BSP_STATUS_OK) {
        bmi323_log_gpio_level("BMI323 CS IDLE", cs_level);
    }
    if (diagnostic != 0U &&
        bsp_spi_read_miso_level(&cs_level) == BSP_STATUS_OK) {
        bmi323_log_gpio_level("BMI323 MISO IDLE BEFORE WHO_AM_I", cs_level);
    }
    if (diagnostic != 0U) {
        status = bmi323_read_chip_id_test(&chip_id);
        if (status != BSP_STATUS_OK) {
            return status;
        }
        bmi323_chip_id = (uint8_t)chip_id;
        return chip_id == BMI323_CHIP_ID_VALUE ? BSP_STATUS_OK : BSP_STATUS_ERROR;
    }

    status = bmi323_read_regs(BMI323_REG_CHIP_ID, &id, 1U, 0U);
    if (status != BSP_STATUS_OK || id != (uint8_t)BMI323_CHIP_ID_VALUE) {
        return status != BSP_STATUS_OK ? status : BSP_STATUS_ERROR;
    }
    bmi323_chip_id = id;

    status = bmi323_write_regs(BMI323_REG_CMD, reset_cmd, sizeof(reset_cmd), 0U);
    if (status != BSP_STATUS_OK) {
        return status;
    }
    bmi323_delay_ms(BMI323_RESET_DELAY_MS);

    /* Bosch requires a post-reset SPI read to complete the bus state transition. */
    status = bmi323_read_regs(BMI323_REG_CHIP_ID, reset_dummy_id,
                               sizeof(reset_dummy_id), 0U);
    if (status != BSP_STATUS_OK) {
        return status;
    }

    status = bmi323_write_regs(BMI323_REG_ACC_CONF, acc_conf, sizeof(acc_conf), 0U);
    if (status != BSP_STATUS_OK) {
        return status;
    }
    status = bmi323_write_regs(BMI323_REG_GYR_CONF, gyr_conf, sizeof(gyr_conf), 0U);
    if (status != BSP_STATUS_OK) {
        return status;
    }

    /* Enable INT1 and map accelerometer/gyro data-ready events to it. */
    status = bmi323_write_regs(BMI323_REG_IO_INT_CTRL, int_ctrl, sizeof(int_ctrl), 0U);
    if (status != BSP_STATUS_OK) {
        return status;
    }
    status = bmi323_write_regs(BMI323_REG_INT_MAP2, int_map2, sizeof(int_map2), 0U);
    if (status != BSP_STATUS_OK) {
        return status;
    }

    bmi323_ready = 1U;
    return BSP_STATUS_OK;
}

bsp_status_t bmi323_init(void)
{
    return bmi323_init_internal(0U);
}

bsp_status_t bmi323_init_diag(void)
{
    return bmi323_init_internal(1U);
}

bsp_status_t bmi323_get_chip_id(uint8_t *chip_id)
{
    if (chip_id == NULL) {
        return BSP_STATUS_INVALID_ARG;
    }
    if (bmi323_ready == 0U) {
        return BSP_STATUS_NOT_READY;
    }
    *chip_id = bmi323_chip_id;
    return BSP_STATUS_OK;
}

bsp_status_t bmi323_read_acc(Vector3f *acc)
{
    uint8_t raw[6];
    bsp_status_t status;
    if (acc == NULL) {
        return BSP_STATUS_INVALID_ARG;
    }
    if (bmi323_ready == 0U) {
        return BSP_STATUS_NOT_READY;
    }
    status = bmi323_read_regs(BMI323_REG_ACC_DATA_X, raw, sizeof(raw), 0U);
    if (status == BSP_STATUS_OK) {
        bmi323_scale_acc(acc, raw);
    }
    return status;
}

bsp_status_t bmi323_read_gyro(Vector3f *gyro)
{
    uint8_t raw[6];
    bsp_status_t status;
    if (gyro == NULL) {
        return BSP_STATUS_INVALID_ARG;
    }
    if (bmi323_ready == 0U) {
        return BSP_STATUS_NOT_READY;
    }
    status = bmi323_read_regs(BMI323_REG_GYR_DATA_X, raw, sizeof(raw), 0U);
    if (status == BSP_STATUS_OK) {
        bmi323_scale_gyro(gyro, raw);
    }
    return status;
}

bsp_status_t bmi323_read_temperature(float *temperature)
{
    uint8_t raw[2];
    bsp_status_t status;
    if (temperature == NULL) {
        return BSP_STATUS_INVALID_ARG;
    }
    if (bmi323_ready == 0U) {
        return BSP_STATUS_NOT_READY;
    }
    status = bmi323_read_regs(BMI323_REG_TEMP_DATA, raw, sizeof(raw), 0U);
    if (status == BSP_STATUS_OK) {
        const int16_t temp_raw = bmi323_s16(raw);
        *temperature = ((float)temp_raw / 512.0f) + 23.0f;
    }
    return status;
}

uint8_t bmi323_is_ready(void)
{
    return bmi323_ready;
}
