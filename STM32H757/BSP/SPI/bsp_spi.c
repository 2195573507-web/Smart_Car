#include "bsp_spi.h"

/* SPI1 BSP 实现；创建人：待确认（当前维护人：Zhiqin）。 */

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

/**
 * @brief 将首次 SPI 访问前的 CS、HAL 状态和寄存器快照分两条日志提交给 S3。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * 传入参数：无。
 * @return 无；格式化截断和两次日志提交结果均被忽略，不向 SPI 调用方报告失败。
 * 调用方式：仅由 bsp_spi_write_read() 在第一次实际 HAL 传输前、填充全局首次访问快照后调用。
 * 线程约束：使用约 96 字节局部缓冲，并可能阻塞等待 S3 link/UART mutex。
 *           首次诊断快照无锁，要求 SPI1 由单一任务持有；禁止 ISR、并发调用，
 *           也不得在持有 s3_service 锁时递归调用。
 */
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

/**
 * @brief 将 HAL SPI 调用状态收敛为 BSP 通用状态。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param status 待转换的 HAL_StatusTypeDef 返回值。
 * @return HAL_OK 映射为 BSP_STATUS_OK，HAL_TIMEOUT 映射为 BSP_STATUS_TIMEOUT，其余状态映射为 BSP_STATUS_ERROR。
 * 调用方式：仅由本文件在 SPI1 阻塞 HAL 事务结束后转换结果；不修改 SPI 或诊断状态。
 * 线程约束：纯值转换，不阻塞、不使用 mutex；函数本身可在 ISR 调用栈执行。
 *           SPI1 所有权和 HAL 阻塞约束由外层接口负责。
 */
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

/**
 * @brief 以 DWT 微秒时间为基准忙等指定时长，满足 BMI323 原始诊断的 CS 建立/保持时间。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param delay_us 忙等时长，单位 us；0 时立即返回。
 * @return 无；没有失败状态，若 DWT 时基不推进且 delay_us 非零则循环无法自行结束。
 * 调用方式：仅由 bsp_spi_bmi323_raw_transaction() 在切换 PC4 CS 前后以 2 us 调用。
 * 线程约束：忙等期间持续占用 CPU，每次读取时基还会短暂屏蔽 IRQ；函数不使用 mutex。
 *           必须由独占 SPI1/PC4 的任务调用；禁止 ISR 或并发调用。
 */
static void spi_delay_us(uint32_t delay_us)
{
    const uint64_t start = timer_get_us();

    while ((uint64_t)(timer_get_us() - start) < (uint64_t)delay_us) {
        /* The diagnostic CS setup/hold time is independent of the SPI clock. */
    }
}

/**
 * @brief 读取 BMI323 CS 对应 GPIO 输出数据寄存器中的锁存电平。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * 传入参数：无。
 * @return PC4 输出锁存位为高时返回 1，否则返回 0；不表示引脚外部物理电平一定一致。
 * 调用方式：仅由 BMI323 原始诊断事务在 CS 释放、有效和再次释放后采样诊断字段。
 * 线程约束：直接读取寄存器，不阻塞、不使用 mutex；函数本身可在 ISR 调用栈执行。
 *           PC4 属于 SPI1/BMI323 路径，调用方必须避免与 CS 切换并发。
 */
static uint8_t spi_bmi323_cs_output_level(void)
{
    return (BMI323_CS_GPIO_Port->ODR & BMI323_CS_Pin) != 0U ? 1U : 0U;
}

/**
 * @brief 在 BMI323 原始 HAL 事务前采集 SPI1 状态、错误和配置寄存器。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[out] diagnostics 调用方拥有的诊断对象，必须非 NULL；成功调用后写入全部 before 字段。
 * @return 无；不校验 diagnostics，传入 NULL 会产生非法访问，也不提供部分写入的失败状态。
 * 调用方式：仅由 bsp_spi_bmi323_raw_transaction() 在临时修改分频并关闭 SPI1 后调用。
 * 线程约束：不阻塞、不使用 mutex，但会直接读取共享 HAL handle 和 SPI1 寄存器。
 *           必须由独占 SPI1 的任务调用；禁止 ISR 或并发事务。
 */
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

/**
 * @brief 在 BMI323 原始 HAL 事务后采集 SPI1 状态、错误和配置寄存器。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[out] diagnostics 调用方拥有的诊断对象，必须非 NULL；成功调用后写入全部 after 字段。
 * @return 无；不校验 diagnostics，传入 NULL 会产生非法访问，也不提供部分写入的失败状态。
 * 调用方式：仅由 bsp_spi_bmi323_raw_transaction() 在 HAL 尝试结束后、释放 CS 和恢复分频前调用。
 * 线程约束：不阻塞、不使用 mutex，但会直接读取共享 HAL handle 和 SPI1 寄存器。
 *           必须由独占 SPI1 的任务调用；禁止 ISR 或并发事务。
 */
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

/**
 * @brief 开启 SPI1/GPIOA 时钟并将 PA5、PA6、PA7 配置为 SPI1 AF5。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * 传入参数：无。
 * @return 无；HAL GPIO 配置接口没有返回值，本函数不校验配置是否在硬件上生效。
 * 调用方式：仅由 bsp_spi_init() 在构造私有 hspi1_bsp 并调用 HAL_SPI_Init() 前执行一次。
 * 线程约束：同步改写时钟和 GPIO 寄存器，不使用 mutex，也没有等待式阻塞。
 *           SPI1 和 PA5..PA7 仅由启动路径配置；禁止 ISR、运行期并发初始化或外部抢占复用。
 */
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

/** 初始化 SPI1、CS 和一次性诊断状态。 */
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

/** 任务上下文阻塞发送事务。 */
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

/** 任务上下文阻塞接收事务。 */
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

/** 执行等长全双工事务；CS 时序仍由调用方负责。 */
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

/** 获取最近一次 HAL 状态（诊断只读）。 */
int32_t bsp_spi_get_last_hal_status(void)
{
    return spi_last_hal_status;
}

/** 获取首次访问的一次性诊断快照。 */
uint8_t bsp_spi_get_first_access_diagnostics(
    bsp_spi_first_access_diagnostics_t *diagnostics)
{
    if (diagnostics == NULL || spi_first_access_diagnostics_valid == 0U) {
        return 0U;
    }
    *diagnostics = spi_first_access_diagnostics;
    return 1U;
}

/** 执行受限 BMI323 原始事务并采集前后寄存器状态。 */
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

/** 读取 SPI 配置下的物理 MISO 电平。 */
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
