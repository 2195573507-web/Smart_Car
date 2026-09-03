#include "bmi323.h"

/* 旧/兼容 BMI323 驱动实现；创建人：待确认（当前维护人：Zhiqin）。 */

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

/**
 * @brief 将 BSP 状态与调用方标签格式化后写入 UART 日志。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] label 以 NUL 结尾的日志标签；NULL 时静默跳过。
 * @param[in] status 要记录的 BSP 状态码。
 * @return 无返回值；UART 写入结果被忽略，失败时输入和驱动状态保持不变。
 * 调用方式：由兼容驱动的 SPI 模式切换、寄存器读写和初始化诊断路径调用。
 * 线程约束：UART 写入最多阻塞 BMI323_LOG_TIMEOUT_MS，不使用 mutex，禁止 ISR 调用；标签仅借用且不保留所有权。
 */
static void bmi323_log_status(const char *label, bsp_status_t status)
{
    char line[96];

    if (label == NULL) {
        return;
    }
    (void)snprintf(line, sizeof(line), "%s status=%d\r\n", label, (int)status);
    (void)uart_log_write(line, BMI323_LOG_TIMEOUT_MS);
}

/**
 * @brief 输出最近一次 HAL SPI 状态码。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] status 由 BSP 提供的 HAL 状态值。
 * @return 无返回值；格式化或 UART 写入失败不向上层报告。
 * 调用方式：由 SPI 模式测试、寄存器诊断和芯片 ID 测试在事务完成后调用。
 * 线程约束：UART 写入最多阻塞 BMI323_LOG_TIMEOUT_MS，不使用 mutex，禁止 ISR 调用；仅按值读取参数，无所有权转移。
 */
static void bmi323_log_hal_status(int32_t status)
{
    char line[96];

    (void)snprintf(line, sizeof(line), "HAL_SPI status: %ld\r\n", (long)status);
    (void)uart_log_write(line, BMI323_LOG_TIMEOUT_MS);
}

/**
 * @brief 以 HIGH/LOW 文本输出指定 BMI323 GPIO 电平。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] label 以 NUL 结尾的电平标签；NULL 时静默跳过。
 * @param[in] level 待记录的 BSP GPIO 电平；非 HIGH 值按当前实现记录为 LOW。
 * @return 无返回值；UART 失败被忽略，不修改 GPIO 或调用方数据。
 * 调用方式：仅由 bmi323_init_internal() 的诊断分支记录 CS/MISO 空闲电平。
 * 线程约束：UART 写入最多阻塞 BMI323_LOG_TIMEOUT_MS，不使用 mutex，禁止 ISR 调用；标签仅借用且不保留所有权。
 */
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

/**
 * @brief 从 STM32 GPIO MODER 寄存器提取指定引脚的两位模式值。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] port 有效 GPIO 寄存器块指针，必须非 NULL。
 * @param[in] pin GPIO 引脚序号，当前调用固定为 0..15 范围内值。
 * @return 0..3 的原始模式值；当前实现不校验参数，非法端口/引脚无失败保护。
 * 调用方式：由 bmi323_gpio_mode_name() 和 bmi323_log_gpio_config() 读取固定 PA5/6/7、PC4 配置时调用。
 * 线程约束：仅执行易失寄存器读取，不阻塞、不使用 mutex；当前仅由启动诊断任务调用。
 *           不在 ISR 使用；port 仅在调用期间借用。
 */
static uint32_t bmi323_gpio_mode(GPIO_TypeDef *port, uint32_t pin)
{
    return (port->MODER >> (pin * 2U)) & 0x3U;
}

/**
 * @brief 从 STM32 GPIO AFR 寄存器提取指定引脚的四位复用功能编号。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] port 有效 GPIO 寄存器块指针，必须非 NULL。
 * @param[in] pin GPIO 引脚序号，当前调用固定为 0..15 范围内值。
 * @return 0..15 的原始 AF 编号；当前实现不校验参数，非法端口/引脚无失败保护。
 * 调用方式：仅由 bmi323_log_gpio_config() 读取 PA5/6/7 的复用配置。
 * 线程约束：仅执行易失寄存器读取，不阻塞、不使用 mutex；当前仅由启动诊断任务调用。
 *           不在 ISR 使用；port 仅在调用期间借用。
 */
static uint32_t bmi323_gpio_af(GPIO_TypeDef *port, uint32_t pin)
{
    return (port->AFR[pin / 8U] >> ((pin % 8U) * 4U)) & 0xFU;
}

/**
 * @brief 将 GPIO 模式和输出类型寄存器值转换为诊断名称。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] port 有效 GPIO 寄存器块指针，必须非 NULL。
 * @param[in] pin GPIO 引脚序号，当前调用固定为 0..15 范围内值。
 * @return 指向只读静态字符串的借用指针；当前实现不返回 NULL，非法参数无失败保护。
 * 调用方式：由 bmi323_log_gpio_config() 格式化 PA5/6/7 和 PC4 模式时调用。
 * 线程约束：仅读取 GPIO 寄存器，不阻塞、不使用 mutex；当前仅由启动诊断任务调用。
 *           不在 ISR 使用；返回字符串为只读静态存储，调用方不得修改或释放。
 */
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

/**
 * @brief 读取并输出 BMI323 SPI1 引脚模式、复用功能及 CS 电平摘要。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * 传入参数：无。
 * @return 无返回值；CS 读取失败时仍输出默认 LOW 和失败状态，UART 失败不向上层报告。
 * 调用方式：仅由 bmi323_init_internal() 在 diagnostic 非零且 SPI 初始化成功后调用。
 * 线程约束：包含 GPIO/MMIO 读取，并可能因 UART 日志阻塞最多 BMI323_LOG_TIMEOUT_MS。
 *           函数不使用 mutex，禁止从 ISR 调用，也不得与引脚重配置并发；无所有权转移。
 */
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

/**
 * @brief 将字节缓冲按十六进制拼接到固定长度日志行并发送。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] label 以 NUL 结尾的前缀；NULL 时静默跳过。
 * @param[in] data 至少包含 length 字节的输入缓冲；NULL 时静默跳过。
 * @param[in] length 要格式化的字节数；0 时静默跳过，超出行容量的尾部按当前实现截断。
 * @return 无返回值；格式化截断或 UART 失败不向上层报告，输入缓冲保持不变。
 * 调用方式：由 SPI 模式切换及寄存器读写诊断分支在 CS 已恢复高电平后调用。
 * 线程约束：UART 写入最多阻塞 BMI323_LOG_TIMEOUT_MS，不使用 mutex，禁止 ISR 调用；label/data 仅借用且不保留所有权。
 */
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

/**
 * @brief 输出四字节 CHIP_ID 原始响应和解析后的 16 位诊断值。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] rx 至少包含 4 字节的接收缓冲；NULL 时静默跳过。
 * @param[in] chip_id 已解析的诊断 ID；事务失败时调用方传入 0。
 * @return 无返回值；UART 写入失败被忽略，输入缓冲和驱动状态保持不变。
 * 调用方式：仅由 bmi323_read_chip_id_test() 在 CS 收尾后调用。
 * 线程约束：UART 写入最多阻塞 BMI323_LOG_TIMEOUT_MS，不使用 mutex，禁止 ISR 调用；rx 仅借用且不保留所有权。
 */
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

/**
 * @brief 基于 BSP 单调毫秒计时执行指定时长的忙等待。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] delay_ms 忙等待时长，单位毫秒。
 * @return 无返回值；无超时或计时器失效保护，计时源不前进时函数不会返回。
 * 调用方式：由 SPI 模式进入和 BMI323 初始化的上电/复位稳定阶段调用。
 * 线程约束：全程占用 CPU、不让出调度，不使用 mutex，禁止 ISR 调用；仅按值读取参数，无所有权转移。
 */
static void bmi323_delay_ms(uint32_t delay_ms)
{
    const uint32_t start = timer_get_ms();
    while ((uint32_t)(timer_get_ms() - start) < delay_ms) {
        /* Reset settling delay; the normal path is only two milliseconds. */
    }
}

/**
 * @brief 基于 BSP 单调微秒计时执行 CS 建立/保持所需忙等待。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] delay_us 忙等待时长，单位微秒。
 * @return 无返回值；无超时或计时器失效保护，计时源不前进时函数不会返回。
 * 调用方式：由 SPI 模式进入、寄存器读写和芯片 ID 测试围绕 CS/SPI 操作调用。
 * 线程约束：全程占用 CPU、不让出调度，不使用 mutex，禁止 ISR 调用；仅按值读取参数，无所有权转移。
 */
static void bmi323_delay_us(uint32_t delay_us)
{
    const uint64_t start = timer_get_us();
    while ((uint64_t)(timer_get_us() - start) < (uint64_t)delay_us) {
        /* Keep CS setup/hold timing independent of the SPI clock. */
    }
}

/**
 * @brief 通过 CS 时序和 `0x7F 0x00` 事务将兼容 BMI323 路径切换到 SPI 模式。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] diagnostic 非零时输出模式切换帧、CS 和 HAL 诊断，零时仅执行事务。
 * @return 返回首个 GPIO/SPI 失败；若前序成功但最终 CS 拉高失败，则返回该失败。
 *         全部步骤成功返回 BSP_STATUS_OK；失败时不回滚已经发生的硬件时序副作用。
 * 调用方式：仅由 bmi323_init_internal() 在 SPI 初始化和上电等待后调用。
 * 线程约束：包含忙等待、最长 BMI323_SPI_TIMEOUT_MS 的 SPI 阻塞，以及可选的多次 UART 阻塞。
 *           函数不使用 mutex；禁止从 ISR 调用或并发使用 SPI；无外部缓冲所有权转移。
 */
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

/**
 * @brief 按兼容驱动时序读取最多 26 字节的 BMI323 连续寄存器。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] reg 起始寄存器地址，函数自动设置 SPI 读位。
 * @param[out] data 接收 length 字节有效载荷的缓冲；必须非 NULL 且容量足够。
 * @param[in] length 读取长度，合法范围为 1..26。
 * @param[in] diagnostic 非零时在 CS 拉高后输出 TX/RX、HAL 和 GPIO 状态。
 * @return 参数非法返回 BSP_STATUS_INVALID_ARG；GPIO、SPI 或最终 CS 任一步失败时返回对应状态。
 *         全部成功返回 BSP_STATUS_OK 并复制 data；失败时调用方输出保持原值。
 * 调用方式：由初始化身份读取、复位后读以及公开加速度/陀螺/温度接口同步调用。
 * 线程约束：包含 CS 忙等待、最长 BMI323_SPI_TIMEOUT_MS 的阻塞 SPI，以及可选 UART 阻塞。
 *           函数没有 mutex，要求 SPI1/CS 由单一任务持有；禁止 ISR 或并发事务。
 *           data 仅在调用期间借用。
 */
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

/**
 * @brief 执行固定四字节 CHIP_ID 诊断事务并解析 RX[2..3] 为 16 位值。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[out] chip_id 接收小端 16 位诊断 ID；NULL 时返回参数错误。
 * @return 参数非法返回 BSP_STATUS_INVALID_ARG；GPIO、SPI 或最终 CS 失败时返回对应状态。
 *         成功时写入 chip_id 并返回 BSP_STATUS_OK；失败时 chip_id 保持调用前内容。
 * 调用方式：仅由 bmi323_init_internal() 的 diagnostic 分支调用，随后调用方按完整 16 位值判断身份。
 * 线程约束：包含 CS 忙等待、阻塞 SPI 和多次 UART 日志；函数不使用 mutex。
 *           禁止从 ISR 调用或并发使用 SPI；chip_id 仅在调用期间借用。
 */
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

/**
 * @brief 按兼容驱动时序写入最多两字节 BMI323 寄存器值。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] reg 目标寄存器地址，函数会清除最高读标志位。
 * @param[in] data 包含 length 字节的发送缓冲；必须非 NULL。
 * @param[in] length 写入长度，合法范围为 1..2。
 * @param[in] diagnostic 非零时在事务后输出发送帧和状态诊断。
 * @return 参数非法返回 BSP_STATUS_INVALID_ARG；其余路径返回 CS 拉低、SPI 或最终 CS 拉高状态。
 *         前置 CS 拉高失败只决定是否发送并参与日志，不会合并到最终返回值；因此未发送时也可能返回 OK。
 *         输入缓冲始终保持不变。
 * 调用方式：由 bmi323_init_internal() 写软复位、ACC/GYR 配置和 INT1 映射寄存器。
 * 线程约束：包含忙等待、最长 BMI323_SPI_TIMEOUT_MS 的阻塞 SPI，以及可选 UART 阻塞。
 *           函数没有 mutex，要求 SPI1/CS 由单一任务持有；禁止 ISR 或并发事务。
 *           data 仅在调用期间借用。
 */
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

/**
 * @brief 将两个小端字节解码为有符号 16 位原始量。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] data 至少包含 2 字节的输入缓冲，必须非 NULL。
 * @return 解码后的 int16_t；当前实现不校验指针，前置条件不满足时无失败保护。
 * 调用方式：由三轴缩放函数和公开温度读取在 SPI 成功后调用。
 * 线程约束：纯计算，不阻塞、不使用 mutex；当前仅在任务路径调用，未设计为 ISR 接口；缓冲仅借用且不保留所有权。
 */
static int16_t bmi323_s16(const uint8_t *data)
{
    return (int16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
}

/**
 * @brief 将六字节原始加速度计数换算为三轴 m/s^2 向量。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[out] acc 接收完整 x/y/z 结果的向量，必须非 NULL。
 * @param[in] raw 至少包含 6 字节的小端三轴原始缓冲，必须非 NULL。
 * @return 无返回值；无失败通道，成功进入函数即依次覆盖三个输出轴；非法指针无保护。
 * 调用方式：仅由 bmi323_read_acc() 在寄存器读取成功后调用。
 * 线程约束：纯计算，不阻塞、不使用 mutex；当前仅在采样任务调用而非 ISR；acc/raw 均仅借用且不保留所有权。
 */
static void bmi323_scale_acc(Vector3f *acc, const uint8_t *raw)
{
    const float scale = (BMI323_ACC_RANGE_G * BMI323_GRAVITY_MPS2) / 32768.0f;
    acc->x = (float)bmi323_s16(&raw[0]) * scale;
    acc->y = (float)bmi323_s16(&raw[2]) * scale;
    acc->z = (float)bmi323_s16(&raw[4]) * scale;
}

/**
 * @brief 将六字节原始陀螺计数换算为三轴 rad/s 向量。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[out] gyro 接收完整 x/y/z 结果的向量，必须非 NULL。
 * @param[in] raw 至少包含 6 字节的小端三轴原始缓冲，必须非 NULL。
 * @return 无返回值；无失败通道，成功进入函数即依次覆盖三个输出轴；非法指针无保护。
 * 调用方式：仅由 bmi323_read_gyro() 在寄存器读取成功后调用。
 * 线程约束：纯计算，不阻塞、不使用 mutex；当前仅在采样任务调用而非 ISR；gyro/raw 均仅借用且不保留所有权。
 */
static void bmi323_scale_gyro(Vector3f *gyro, const uint8_t *raw)
{
    const float scale = (BMI323_GYRO_RANGE_DPS / 32768.0f) * BMI323_DEG_TO_RAD;
    gyro->x = (float)bmi323_s16(&raw[0]) * scale;
    gyro->y = (float)bmi323_s16(&raw[2]) * scale;
    gyro->z = (float)bmi323_s16(&raw[4]) * scale;
}

/**
 * @brief 执行兼容 BMI323 路径的 SPI 进入、身份检查及可选完整配置。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] diagnostic 非零时运行详细诊断并在 CHIP_ID 测试后提前返回；零时继续软复位、ACC/GYR 和 INT1 配置。
 * @return BSP_STATUS_OK 表示当前分支完成；否则返回 BSP、GPIO、SPI 或身份错误。
 *         入口总会清除 ready；普通分支仅在全部配置成功后重新置位。
 *         诊断分支即使返回 OK 也保持 ready=0；失败时不回滚已经执行的硬件写入。
 * 调用方式：由 bmi323_init() 以 0 调用，或由 bmi323_init_diag() 以 1 调用；该兼容源当前不在 CM7 target_sources 中。
 * 线程约束：包含 SPI/GPIO 阻塞、毫秒/微秒忙等待和可选 UART 阻塞，且不使用 mutex。
 *           必须由单一启动/恢复任务独占 SPI1/CS；禁止 ISR、重入或与采样并发。
 *           不涉及外部缓冲所有权。
 */
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

/** 初始化兼容 BMI323 设备路径。 */
bsp_status_t bmi323_init(void)
{
    return bmi323_init_internal(0U);
}

/** 初始化并输出一次性诊断信息。 */
bsp_status_t bmi323_init_diag(void)
{
    return bmi323_init_internal(1U);
}

/** 读取兼容驱动的芯片 ID。 */
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

/** 读取兼容路径加速度，单位 m/s^2。 */
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

/** 读取兼容路径角速度，单位 rad/s。 */
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

/** 读取兼容路径温度，单位 degC。 */
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

/** 查询兼容驱动就绪状态。 */
uint8_t bmi323_is_ready(void)
{
    return bmi323_ready;
}
