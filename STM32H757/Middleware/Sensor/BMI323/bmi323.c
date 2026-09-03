#include "bmi323.h"

/* BMI323 驱动实现；创建人：待确认（当前维护人：Zhiqin）。 */

#include <stddef.h>
#include <stdio.h>

#include "bmi323_port.h"
#include "bmi323_reg.h"
#include "bsp_gpio.h"
#include "bsp_spi.h"
#include "bsp_uart.h"
#include "imu_time.h"

#define BMI323_SPI_RAW_BYTES         UINT16_C(4)
#define BMI323_SPI_TIMEOUT_MS        UINT32_C(20)
#define BMI323_LOG_TIMEOUT_MS        UINT32_C(100)
#define BMI323_RESET_DELAY_MS        UINT32_C(2)
#define BMI323_MAX_READ_BYTES        UINT16_C(26)
#define BMI323_MAX_WRITE_BYTES       UINT16_C(2)

#define BMI323_ACC_RANGE_G           4.0f
#define BMI323_GYRO_RANGE_DPS        500.0f
#define BMI323_GRAVITY_MPS2          9.80665f
#define BMI323_DEG_TO_RAD            0.01745329251994329577f
#define BMI323_TEMP_OFFSET_C         23.0f
#define BMI323_TEMP_SCALE            512.0f

static volatile bool bmi323_ready;
static volatile bmi323_sample_rate_t bmi323_sample_rate =
    BMI323_SAMPLE_RATE_200HZ;
static bmi323_error_t bmi323_last_error;
static bmi323_diagnostics_t bmi323_diagnostics;
static uint8_t whoami_trace_done = 0U;
static uint8_t whoami_trace_tx[BMI323_SPI_RAW_BYTES];
static uint8_t whoami_trace_rx[BMI323_SPI_RAW_BYTES];
static int32_t whoami_trace_hal_status = -1;
static uint8_t bmi323_probe_done;
/* Probe result is latched only within one bmi323_init() attempt. */
static uint8_t bmi323_probe_result;
static bmi323_diag_t bmi323_diag = {
    .valid = 0U,
    .whoami = 0U,
    .rx0 = 0U,
    .rx1 = 0U,
    .spi_status = UINT8_MAX
};

/**
 * @brief 将采样率枚举映射为 BMI323 配置寄存器的 ODR 位编码。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] sample_rate 待转换的采样率枚举；无效值按当前实现回退为 100 Hz 编码。
 * @return 对应的 ODR 位编码；本函数不报告失败，也不修改调用方对象。
 * 调用方式：由 bmi323_build_config() 和 bmi323_apply_sample_rate() 在组装配置值时同步调用。
 * 线程约束：纯计算，不阻塞、不使用 mutex；当前仅在任务路径调用，未设计为 ISR 接口；不取得参数所有权。
 */
static uint16_t bmi323_odr_code(bmi323_sample_rate_t sample_rate)
{
    switch (sample_rate) {
    case BMI323_SAMPLE_RATE_100HZ: return BMI323_CONF_ODR_100HZ;
    case BMI323_SAMPLE_RATE_200HZ: return BMI323_CONF_ODR_200HZ;
    case BMI323_SAMPLE_RATE_400HZ: return BMI323_CONF_ODR_400HZ;
    case BMI323_SAMPLE_RATE_800HZ: return BMI323_CONF_ODR_800HZ;
    default: return BMI323_CONF_ODR_100HZ;
    }
}

/**
 * @brief 检查采样率是否属于驱动支持的 100/200/400/800 Hz 集合。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] sample_rate 待校验的采样率枚举。
 * @return 支持时返回 1，否则返回 0；无副作用，不涉及输出保持问题。
 * 调用方式：由 bmi323_set_sample_rate() 在修改软件状态或硬件寄存器前调用。
 * 线程约束：纯计算，不阻塞、不使用 mutex；当前仅在任务路径调用，未设计为 ISR 接口；不取得参数所有权。
 */
static uint8_t bmi323_sample_rate_valid(bmi323_sample_rate_t sample_rate)
{
    return sample_rate == BMI323_SAMPLE_RATE_100HZ ||
                   sample_rate == BMI323_SAMPLE_RATE_200HZ ||
                   sample_rate == BMI323_SAMPLE_RATE_400HZ ||
                   sample_rate == BMI323_SAMPLE_RATE_800HZ
               ? 1U
               : 0U;
}

/**
 * @brief 以既有量程/带宽基础值和指定 ODR 组装两字节小端配置。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] sample_rate 目标采样率；无效值经 bmi323_odr_code() 回退为 100 Hz。
 * @param[in] base_config 保留除 ODR 字段外的配置基础值。
 * @param[out] config 接收低字节和高字节的两字节缓冲；必须非 NULL 且至少可写 2 字节。
 * @return 无返回值；当前实现不校验 config，前置条件不满足时输出语义未定义。
 * 调用方式：由 bmi323_init() 和 bmi323_apply_sample_rate() 使用栈上两字节数组调用。
 * 线程约束：纯计算，不阻塞、不使用 mutex；当前仅在任务路径调用，未设计为 ISR 接口；缓冲仅借用且不保留所有权。
 */
static void bmi323_build_config(bmi323_sample_rate_t sample_rate,
                                uint16_t base_config, uint8_t config[2])
{
    const uint16_t value = (uint16_t)((base_config & ~BMI323_CONF_ODR_MASK) |
                                      bmi323_odr_code(sample_rate));

    config[0] = (uint8_t)(value & 0xFFU);
    config[1] = (uint8_t)(value >> 8U);
}

/**
 * @brief 通过现有 UART 日志通道发送一条短诊断文本。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] line 以 NUL 结尾的待发送文本；NULL 时静默跳过。
 * @return 无返回值；UART 写入结果被丢弃，发送失败不会修改调用方缓冲。
 * 调用方式：由本文件初始化、探针和错误记录路径同步调用。
 * 线程约束：uart_log_write() 最多阻塞 BMI323_LOG_TIMEOUT_MS，且不使用本文件 mutex。
 *           禁止 ISR；字符串仅在调用期间借用，不转移所有权。
 */
static void bmi323_log_short(const char *line)
{
    if (line != NULL) {
        (void)uart_log_write(line, BMI323_LOG_TIMEOUT_MS);
    }
}

/**
 * @brief 将初始化错误格式化为紧凑的 `[BMI][E1]` 诊断记录。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] error 初始化错误；NONE 按当前实现替换为 WHO_AM_I_READ。
 * @return 无返回值；格式化或 UART 发送失败不向上层报告，也不改变错误状态。
 * 调用方式：仅由 bmi323_init_fail() 在初始化失败收口时同步调用。
 * 线程约束：间接 UART 写入最多阻塞 BMI323_LOG_TIMEOUT_MS，无 mutex，禁止 ISR 调用；无指针所有权转移。
 */
static void bmi323_log_init_fail(bmi323_error_t error)
{
    char line[64];
    const bmi323_error_t reported_error =
        error == BMI323_ERROR_NONE ? BMI323_ERROR_WHO_AM_I_READ : error;

    (void)snprintf(line, sizeof(line), "[BMI][E1] init_fail rc=%ld\r\n",
                   -(long)reported_error);
    bmi323_log_short(line);
}

/**
 * @brief 对诊断计数器执行饱和加一。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in,out] counter 待更新计数器；NULL 或已为 UINT32_MAX 时保持原值。
 * @return 无返回值；无失败上报，不能更新时输出保持不变。
 * 调用方式：由 SPI、读写、WHO_AM_I 和采样统计路径直接调用。
 * 线程约束：不阻塞、不使用 mutex，读改写不是原子操作。
 *           禁止任务或 ISR 并发更新同一计数器；指针仅在调用期间借用。
 */
static void bmi323_increment_counter(uint32_t *counter)
{
    if (counter != NULL && *counter != UINT32_MAX) {
        ++(*counter);
    }
}

/**
 * @brief 同步增加通用 SPI 错误的两个兼容诊断计数器。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * 传入参数：无。
 * @return 无返回值；计数器达到 UINT32_MAX 后保持饱和。
 * 调用方式：由寄存器传输、初始化和原始探针的失败分支调用。
 * 线程约束：不阻塞、不使用 mutex，内部计数更新不是原子操作。
 *           禁止任务与 ISR 并发，多个任务之间也不得并发调用；不涉及外部对象所有权。
 */
static void bmi323_record_spi_error(void)
{
    bmi323_increment_counter(&bmi323_diagnostics.spi_error_count);
    bmi323_increment_counter(&bmi323_diagnostics.spi_error);
}

/**
 * @brief 同时更新驱动最近错误和诊断快照中的最近错误字段。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] error 要记录的错误枚举。
 * @return 无返回值；赋值本身无失败路径，其他诊断字段保持不变。
 * 调用方式：由参数校验、SPI 失败、配置失败和初始化失败路径调用。
 * 线程约束：不阻塞、不使用 mutex；共享状态写入未同步，禁止 ISR 或多个任务并发调用；不涉及指针所有权。
 */
static void bmi323_set_error(bmi323_error_t error)
{
    bmi323_last_error = error;
    bmi323_diagnostics.last_error = error;
}

/**
 * @brief 捕获首次 WHO_AM_I 原始事务，并刷新最近一次接收字节诊断。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] reg 本次读取的寄存器；非 WHO_AM_I 时不更新任何状态。
 * @param[in] tx 事务发送缓冲；必须至少覆盖 min(length, BMI323_SPI_RAW_BYTES) 字节，NULL 时不更新。
 * @param[in] rx 事务接收缓冲；必须至少覆盖 min(length, BMI323_SPI_RAW_BYTES) 字节，NULL 时不更新。
 * @param[in] length 事务字节数；实际最多复制和解析前 BMI323_SPI_RAW_BYTES 字节。
 * @return 无返回值；寄存器或指针无效时所有诊断输出保持原值。
 * 调用方式：由 bmi323_read_reg() 和 bmi323_spi_probe() 在事务结束后同步调用。
 * 线程约束：不阻塞、不使用 mutex；会写多个共享诊断字段，禁止 ISR/多任务并发调用；tx/rx 仅借用且不保留所有权。
 */
static void bmi323_capture_whoami_raw(uint8_t reg, const uint8_t *tx,
                                      const uint8_t *rx, uint16_t length)
{
    const uint16_t trace_length = length < BMI323_SPI_RAW_BYTES
                                      ? length : BMI323_SPI_RAW_BYTES;

    if (reg != BMI323_WHO_AM_I_REG || tx == NULL || rx == NULL) {
        return;
    }
    if (whoami_trace_done == 0U) {
        for (uint16_t index = 0U; index < trace_length; ++index) {
            whoami_trace_tx[index] = tx[index];
            whoami_trace_rx[index] = rx[index];
        }
        whoami_trace_hal_status = bmi323_port_get_last_hal_status();
        bmi323_diag.whoami = trace_length > 2U ? rx[2] : 0U;
        bmi323_diag.rx0 = trace_length > 0U ? rx[0] : 0U;
        bmi323_diag.rx1 = trace_length > 1U ? rx[1] : 0U;
        bmi323_diag.rx2 = trace_length > 2U ? rx[2] : 0U;
        bmi323_diag.rx3 = trace_length > 3U ? rx[3] : 0U;
        bmi323_diag.spi_status = whoami_trace_hal_status < 0
                                     ? UINT8_MAX
                                     : (uint8_t)whoami_trace_hal_status;
        whoami_trace_done = 1U;
    }
    bmi323_diagnostics.last_rx0 = trace_length > 0U ? rx[0] : 0U;
    bmi323_diagnostics.last_rx1 = trace_length > 1U ? rx[1] : 0U;
    bmi323_diagnostics.last_rx2 = trace_length > 2U ? rx[2] : 0U;
    bmi323_diagnostics.last_rx3 = trace_length > 3U ? rx[3] : 0U;
    bmi323_diagnostics.last_whoami = bmi323_diagnostics.last_rx2;
}

/**
 * @brief 将 BMI323 错误枚举转换为固定诊断名称。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] error 待转换的错误枚举。
 * @return 指向只读静态字符串的借用指针；NONE 和未知值均返回 "NONE"，不会返回 NULL。
 * 调用方式：由 bmi323_log_debug() 和 bmi323_log_init() 格式化日志时调用。
 * 线程约束：纯查询、可重入，不阻塞也不使用 mutex；当前仅由任务日志路径调用。
 *           返回字符串为只读静态存储，调用方不得修改或释放。
 */
static const char *bmi323_error_name(bmi323_error_t error)
{
    switch (error) {
    case BMI323_ERROR_SPI_INIT: return "SPI_INIT_FAIL";
    case BMI323_ERROR_WHO_AM_I_READ: return "WHO_AM_I_FAIL";
    case BMI323_ERROR_WHO_AM_I_MISMATCH: return "WHO_AM_I_FAIL";
    case BMI323_ERROR_SOFT_RESET: return "SOFT_RESET_FAIL";
    case BMI323_ERROR_POST_RESET_READ: return "POST_RESET_READ_FAIL";
    case BMI323_ERROR_ACCEL_CONFIG: return "ACCEL_CONFIG_FAIL";
    case BMI323_ERROR_GYRO_CONFIG: return "GYRO_CONFIG_FAIL";
    case BMI323_ERROR_DATA_READ: return "DATA_READ_FAIL";
    case BMI323_ERROR_DATA_WRITE: return "DATA_WRITE_FAIL";
    case BMI323_ERROR_WHO_AM_I_TIMEOUT: return "WHO_AM_I_TIMEOUT";
    case BMI323_ERROR_SPI_TX_FAIL: return "SPI_TX_FAIL";
    case BMI323_ERROR_SPI_RX_FAIL: return "SPI_RX_FAIL";
    case BMI323_ERROR_WHO_AM_I_VALUE: return "WHO_AM_I_VALUE_ERROR";
    case BMI323_ERROR_NONE:
    default: return "NONE";
    }
}

/**
 * @brief 将当前 BMI323 在线状态和累计诊断字段输出为多行调试记录。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * 传入参数：无。
 * @return 无返回值；snprintf 截断或 UART 失败均不向上层报告，诊断状态保持不变。
 * 调用方式：由 bmi323_init() 成功路径及 bmi323_init_fail() 失败收口调用。
 * 线程约束：UART 写入最多阻塞 BMI323_LOG_TIMEOUT_MS，且不使用 mutex。
 *           并发写入可能使诊断快照不一致；禁止 ISR；无所有权转移。
 */
static void bmi323_log_debug(void)
{
    char line[256];

    (void)snprintf(line, sizeof(line),
                   "[BMI323][DEBUG]\r\n"
                   "online=%u\r\n"
                   "last_whoami=0x%02X\r\n"
                   "rx0=0x%02X\r\n"
                   "rx1=0x%02X\r\n"
                   "rx2=0x%02X\r\n"
                   "rx3=0x%02X\r\n"
                   "read_ok=%lu\r\n"
                   "read_fail=%lu\r\n"
                   "write_fail=%lu\r\n"
                   "spi_error=%lu\r\n"
                   "spi_tx_fail=%lu\r\n"
                   "spi_rx_fail=%lu\r\n"
                   "whoami_fail=%lu\r\n"
                   "last_error=%s\r\n"
                   "last_status=%s\r\n",
                   bmi323_ready ? 1U : 0U,
                   (unsigned)bmi323_diagnostics.last_whoami,
                   (unsigned)bmi323_diagnostics.last_rx0,
                   (unsigned)bmi323_diagnostics.last_rx1,
                   (unsigned)bmi323_diagnostics.last_rx2,
                   (unsigned)bmi323_diagnostics.last_rx3,
                   (unsigned long)bmi323_diagnostics.read_ok,
                   (unsigned long)bmi323_diagnostics.read_fail,
                   (unsigned long)bmi323_diagnostics.write_fail,
                   (unsigned long)bmi323_diagnostics.spi_error,
                   (unsigned long)bmi323_diagnostics.spi_tx_fail,
                   (unsigned long)bmi323_diagnostics.spi_rx_fail,
                   (unsigned long)bmi323_diagnostics.whoami_fail,
                   bmi323_error_name(bmi323_diagnostics.last_error),
                   bmi323_diag_status_name(bmi323_diagnostics.last_status));
    (void)uart_log_write(line, BMI323_LOG_TIMEOUT_MS);
}

/**
 * @brief 输出 WHO_AM_I、初始化状态和 ACC/GYR ODR 配置摘要。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * 传入参数：无。
 * @return 无返回值；两次 UART 写入结果均被忽略，不改变驱动或诊断状态。
 * 调用方式：由 bmi323_init() 成功路径及 bmi323_init_fail() 失败收口调用。
 * 线程约束：最多执行两次 BMI323_LOG_TIMEOUT_MS 阻塞写入，无 mutex；禁止 ISR 调用并要求初始化路径串行；无所有权转移。
 */
static void bmi323_log_init(void)
{
    char line[96];

    (void)snprintf(line, sizeof(line),
                   "[BMI323][INIT]\r\nWHO_AM_I=0x%02X\r\nSTATE=%s\r\n",
                   (unsigned)bmi323_diagnostics.who_am_i,
                   bmi323_ready ? "ONLINE" : bmi323_error_name(bmi323_last_error));
    (void)uart_log_write(line, BMI323_LOG_TIMEOUT_MS);
    (void)snprintf(line, sizeof(line),
                   "[BMI323][ODR]\r\nrate_hz=%u\r\nACC_CONF=0x%04X\r\n"
                   "GYR_CONF=0x%04X\r\n",
                   (unsigned)bmi323_diagnostics.sample_rate_hz,
                   (unsigned)bmi323_diagnostics.acc_conf,
                   (unsigned)bmi323_diagnostics.gyr_conf);
    (void)uart_log_write(line, BMI323_LOG_TIMEOUT_MS);
}

/**
 * @brief 分段输出一次底层 BMI323 原始 SPI 探针的引脚、帧和寄存器诊断。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] test 探针序号，仅用于日志标识。
 * @param[in] sequence 以 NUL 结尾的事务方案名称；NULL 时不输出。
 * @param[in] tx 至少 BMI323_SPI_RAW_BYTES 字节的发送缓冲；NULL 时不输出。
 * @param[in] rx 至少 BMI323_SPI_RAW_BYTES 字节的接收缓冲；NULL 时不输出。
 * @param[in] diagnostics 已由 BSP 填充的原始事务诊断；NULL 时不输出。
 * @return 无返回值；任一指针无效时静默返回，UART 失败不改变输入或探针结果。
 * 调用方式：仅由 bmi323_spi_probe() 对三种事务方案逐次调用。
 * 线程约束：包含多次 UART 阻塞写入，每次最长 BMI323_LOG_TIMEOUT_MS；函数没有 mutex。
 *           禁止 ISR；所有输入均仅在调用期间借用。
 */
static void bmi323_log_raw_probe(uint8_t test, const char *sequence,
                                  const uint8_t tx[BMI323_SPI_RAW_BYTES],
                                  const uint8_t rx[BMI323_SPI_RAW_BYTES],
                                  const bsp_spi_bmi323_raw_diagnostics_t *diagnostics)
{
    char line[96];

    if (sequence == NULL || tx == NULL || rx == NULL || diagnostics == NULL) {
        return;
    }

    (void)snprintf(line, sizeof(line), "[BMI][RAW] test=%u sequence=%s\r\n",
                   (unsigned)test, sequence);
    bmi323_log_short(line);
    bmi323_log_short("[BMI][RAW] spi_instance=1 sck_pin=PA5 af=AF5\r\n");
    bmi323_log_short("[BMI][RAW] miso_pin=PA6 af=AF5 mosi_pin=PA7 af=AF5\r\n");
    bmi323_log_short("[BMI][RAW] cs_pin=PC4\r\n");
    (void)snprintf(line, sizeof(line), "[BMI][RAW] spi_hz=%lu\r\n",
                   (unsigned long)diagnostics->spi_hz);
    bmi323_log_short(line);
    (void)snprintf(line, sizeof(line),
                   "[BMI][RAW] tx_len=%u tx=%02X %02X %02X %02X\r\n",
                   (unsigned)BMI323_SPI_RAW_BYTES, (unsigned)tx[0],
                   (unsigned)tx[1], (unsigned)tx[2], (unsigned)tx[3]);
    bmi323_log_short(line);
    (void)snprintf(line, sizeof(line),
                   "[BMI][RAW] rx_len=%u rx=%02X %02X %02X %02X\r\n",
                   (unsigned)BMI323_SPI_RAW_BYTES, (unsigned)rx[0],
                   (unsigned)rx[1], (unsigned)rx[2], (unsigned)rx[3]);
    bmi323_log_short(line);
    (void)snprintf(line, sizeof(line),
                   "[BMI][RAW] hal_status=%ld hal_error=0x%08lX\r\n",
                   (long)diagnostics->hal_status,
                   (unsigned long)diagnostics->hal_error);
    bmi323_log_short(line);
    (void)snprintf(line, sizeof(line),
                   "[BMI][RAW] cs_before=%u cs_low=%u cs_after=%u\r\n",
                   (unsigned)diagnostics->cs_before,
                   (unsigned)diagnostics->cs_active,
                   (unsigned)diagnostics->cs_after);
    bmi323_log_short(line);
    (void)snprintf(line, sizeof(line),
                   "[BMI][RAW] state_before=%lu error_before=0x%08lX\r\n",
                   (unsigned long)diagnostics->spi_state_before,
                   (unsigned long)diagnostics->spi_error_before);
    bmi323_log_short(line);
    (void)snprintf(line, sizeof(line),
                   "[BMI][RAW] sr_before=0x%08lX cfg1_before=0x%08lX\r\n",
                   (unsigned long)diagnostics->spi_sr_before,
                   (unsigned long)diagnostics->spi_cfg1_before);
    bmi323_log_short(line);
    (void)snprintf(line, sizeof(line),
                   "[BMI][RAW] cfg2_before=0x%08lX cr1_before=0x%08lX\r\n",
                   (unsigned long)diagnostics->spi_cfg2_before,
                   (unsigned long)diagnostics->spi_cr1_before);
    bmi323_log_short(line);
    (void)snprintf(line, sizeof(line), "[BMI][RAW] cr2_before=0x%08lX\r\n",
                   (unsigned long)diagnostics->spi_cr2_before);
    bmi323_log_short(line);
    (void)snprintf(line, sizeof(line),
                   "[BMI][RAW] state_after=%lu error_after=0x%08lX\r\n",
                   (unsigned long)diagnostics->spi_state_after,
                   (unsigned long)diagnostics->spi_error_after);
    bmi323_log_short(line);
    (void)snprintf(line, sizeof(line),
                   "[BMI][RAW] sr_after=0x%08lX cfg1_after=0x%08lX\r\n",
                   (unsigned long)diagnostics->spi_sr_after,
                   (unsigned long)diagnostics->spi_cfg1_after);
    bmi323_log_short(line);
    (void)snprintf(line, sizeof(line),
                   "[BMI][RAW] cfg2_after=0x%08lX cr1_after=0x%08lX\r\n",
                   (unsigned long)diagnostics->spi_cfg2_after,
                   (unsigned long)diagnostics->spi_cr1_after);
    bmi323_log_short(line);
    (void)snprintf(line, sizeof(line), "[BMI][RAW] cr2_after=0x%08lX\r\n",
                   (unsigned long)diagnostics->spi_cr2_after);
    bmi323_log_short(line);
}

/**
 * @brief 统一收口 BMI323 初始化失败状态并输出失败诊断。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] error 初始化错误；NONE 会规范化为 WHO_AM_I_READ。
 * @param[in] status 对外诊断状态。
 * @return 固定返回 false；失败时清除 ready、更新错误/结果字段并保留已累计的其他诊断字段。
 * 调用方式：仅由 bmi323_init() 的端口、身份、复位和配置失败分支 return 调用。
 * 线程约束：包含多次阻塞 UART 日志，且不使用 mutex。
 *           必须由单一初始化任务调用；禁止 ISR 或并发初始化；不涉及指针所有权。
 */
static bool bmi323_init_fail(bmi323_error_t error, bmi323_diag_status_t status)
{
    const bmi323_error_t reported_error =
        error == BMI323_ERROR_NONE ? BMI323_ERROR_WHO_AM_I_READ : error;

    bmi323_ready = false;
    bmi323_set_error(reported_error);
    bmi323_diagnostics.init_result = -(int32_t)reported_error;
    bmi323_diagnostics.last_status = status;
    bmi323_log_init_fail(reported_error);
    bmi323_log_init();
    bmi323_log_debug();
    return false;
}

/**
 * @brief 将两个小端字节解码为有符号 16 位原始量。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] data 至少包含 2 字节的输入缓冲，必须非 NULL。
 * @return 解码后的 int16_t；当前实现不校验指针，前置条件不满足时无失败保护。
 * 调用方式：由采样和温度读取路径在 SPI 成功后同步调用。
 * 线程约束：纯计算，不阻塞、不使用 mutex；当前仅在任务路径调用，未设计为 ISR 接口；缓冲仅借用且不保留所有权。
 */
static int16_t bmi323_decode_s16(const uint8_t *data)
{
    return (int16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8U));
}

/**
 * @brief 将两个小端字节解码为无符号 16 位寄存器值。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] data 至少包含 2 字节的输入缓冲，必须非 NULL。
 * @return 解码后的 uint16_t；当前实现不校验指针，前置条件不满足时无失败保护。
 * 调用方式：由初始化和 ODR 配置读回校验路径同步调用。
 * 线程约束：纯计算，不阻塞、不使用 mutex；当前仅在任务路径调用，未设计为 ISR 接口；缓冲仅借用且不保留所有权。
 */
static uint16_t bmi323_decode_u16(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8U);
}

/** 读取连续寄存器；参数校验和 SPI 失败均通过 false 返回。 */
bool bmi323_read_reg(uint8_t reg, uint8_t *data, uint16_t len)
{
    uint8_t tx[BMI323_MAX_READ_BYTES + 2U] = {0U};
    uint8_t rx[BMI323_MAX_READ_BYTES + 2U] = {0U};
    const uint16_t frame_length = (uint16_t)(len + 2U);
    bmi323_port_trace_t port_trace = {0};
    bsp_status_t status;

    if (data == NULL || len == 0U || len > BMI323_MAX_READ_BYTES) {
        bmi323_set_error(BMI323_ERROR_DATA_READ);
        return false;
    }

    tx[0] = (uint8_t)(reg | BMI323_SPI_READ_MASK);
    tx[1] = 0U;
    /* BMI323 SPI needs one dummy byte after the command; payload starts at RX[2]. */
    bmi323_increment_counter(&bmi323_diagnostics.spi_read_count);
    status = bmi323_port_spi_read(tx, rx, frame_length, BMI323_SPI_TIMEOUT_MS,
                                  reg == BMI323_WHO_AM_I_REG &&
                                      whoami_trace_done == 0U
                                      ? &port_trace : NULL);
    bmi323_capture_whoami_raw(reg, tx, rx, frame_length);
    if (status != BSP_STATUS_OK) {
        bmi323_increment_counter(&bmi323_diagnostics.spi_read_fail);
        bmi323_increment_counter(&bmi323_diagnostics.spi_rx_fail);
        bmi323_record_spi_error();
        if (reg == BMI323_WHO_AM_I_REG && status == BSP_STATUS_TIMEOUT) {
            bmi323_diagnostics.last_status = BMI323_DIAG_STATUS_WHO_AM_I_TIMEOUT;
            bmi323_set_error(BMI323_ERROR_WHO_AM_I_TIMEOUT);
        } else {
            bmi323_diagnostics.last_status = BMI323_DIAG_STATUS_SPI_RX_FAIL;
            bmi323_set_error(reg == BMI323_WHO_AM_I_REG
                                 ? BMI323_ERROR_SPI_RX_FAIL : BMI323_ERROR_DATA_READ);
        }
        return false;
    }

    bmi323_increment_counter(&bmi323_diagnostics.spi_read_success);
    for (uint16_t index = 0U; index < len; ++index) {
        data[index] = rx[index + 2U];
    }
    return true;
}

/** 写入连续寄存器；调用方仍拥有输入缓冲。 */
bool bmi323_write_reg(uint8_t reg, const uint8_t *data, uint16_t len)
{
    uint8_t tx[BMI323_MAX_WRITE_BYTES + 1U] = {0U};
    bsp_status_t status;

    if (data == NULL || len == 0U || len > BMI323_MAX_WRITE_BYTES) {
        bmi323_set_error(BMI323_ERROR_DATA_WRITE);
        return false;
    }

    tx[0] = (uint8_t)(reg & UINT8_C(0x7F));
    for (uint16_t index = 0U; index < len; ++index) {
        tx[index + 1U] = data[index];
    }
    bmi323_increment_counter(&bmi323_diagnostics.write_count);
    status = bmi323_port_spi_write(tx, (uint16_t)(len + 1U), BMI323_SPI_TIMEOUT_MS);
    if (status != BSP_STATUS_OK) {
        bmi323_increment_counter(&bmi323_diagnostics.write_fail);
        bmi323_increment_counter(&bmi323_diagnostics.spi_tx_fail);
        bmi323_record_spi_error();
        bmi323_diagnostics.last_status = BMI323_DIAG_STATUS_SPI_TX_FAIL;
        bmi323_set_error(BMI323_ERROR_SPI_TX_FAIL);
        return false;
    }
    bmi323_increment_counter(&bmi323_diagnostics.write_ok);
    return true;
}

/**
 * @brief 写入并读回校验加速度计和陀螺仪的目标 ODR 配置。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] sample_rate 已由调用方校验的目标采样率。
 * @return 两组寄存器均写入且读回匹配时返回 true；任一步失败返回 false 并记录错误。
 *         失败不会回滚已经写入的前序硬件寄存器；后续诊断字段也可能只更新一部分。
 * 调用方式：仅由 bmi323_set_sample_rate() 在设备 ready 后同步调用。
 * 线程约束：包含带超时的阻塞 SPI 读写，且不使用 mutex。
 *           必须由 BMI323 单一所有者任务串行调用；禁止 ISR 或并发采样。
 *           不涉及外部缓冲所有权。
 */
static bool bmi323_apply_sample_rate(bmi323_sample_rate_t sample_rate)
{
    uint8_t accel_config[2U] = {0U};
    uint8_t gyro_config[2U] = {0U};
    uint8_t config[2U] = {0U};
    const uint16_t expected_accel =
        (uint16_t)((UINT16_C(0x4018) & ~BMI323_CONF_ODR_MASK) |
                   bmi323_odr_code(sample_rate));
    const uint16_t expected_gyro =
        (uint16_t)((UINT16_C(0x4028) & ~BMI323_CONF_ODR_MASK) |
                   bmi323_odr_code(sample_rate));

    bmi323_build_config(sample_rate, UINT16_C(0x4018), accel_config);
    bmi323_build_config(sample_rate, UINT16_C(0x4028), gyro_config);
    if (!bmi323_write_reg(BMI323_REG_ACC_CONF, accel_config,
                          sizeof(accel_config)) ||
        !bmi323_read_reg(BMI323_REG_ACC_CONF, config, sizeof(config)) ||
        bmi323_decode_u16(config) != expected_accel) {
        bmi323_set_error(BMI323_ERROR_ACCEL_CONFIG);
        bmi323_diagnostics.last_status = BMI323_DIAG_STATUS_CONFIG_FAIL;
        return false;
    }
    bmi323_diagnostics.acc_conf = bmi323_decode_u16(config);
    if (!bmi323_write_reg(BMI323_REG_GYR_CONF, gyro_config,
                          sizeof(gyro_config)) ||
        !bmi323_read_reg(BMI323_REG_GYR_CONF, config, sizeof(config)) ||
        bmi323_decode_u16(config) != expected_gyro) {
        bmi323_set_error(BMI323_ERROR_GYRO_CONFIG);
        bmi323_diagnostics.last_status = BMI323_DIAG_STATUS_CONFIG_FAIL;
        return false;
    }
    bmi323_diagnostics.gyr_conf = bmi323_decode_u16(config);
    bmi323_diagnostics.ctrl_acc = expected_accel;
    bmi323_diagnostics.ctrl_gyr = expected_gyro;
    bmi323_diagnostics.sample_rate_hz = (uint16_t)sample_rate;
    return true;
}

/** 更新 BMI323 ODR 配置并记录诊断状态。 */
bool bmi323_set_sample_rate(bmi323_sample_rate_t sample_rate)
{
    const bmi323_sample_rate_t previous_rate = bmi323_sample_rate;

    if (bmi323_sample_rate_valid(sample_rate) == 0U) {
        return false;
    }
    bmi323_sample_rate = sample_rate;
    if (!bmi323_ready) {
        return true;
    }
    if (!bmi323_apply_sample_rate(sample_rate)) {
        bmi323_sample_rate = previous_rate;
        return false;
    }
    return true;
}

/** 返回当前 ODR 配置。 */
bmi323_sample_rate_t bmi323_get_sample_rate(void)
{
    return bmi323_sample_rate;
}

/** 执行软复位、WHO_AM_I、配置和就绪检查。 */
bool bmi323_init(void)
{
    uint8_t who_am_i = 0U;
    uint8_t reset_dummy[2U] = {0U};
    uint8_t config[2U] = {0U};
    bsp_status_t driver_status;
    bsp_status_t cs_status;
    int32_t chip_id_rc;
    char line[80];
    const uint8_t soft_reset[2U] = {BMI323_SOFT_RESET_LSB, BMI323_SOFT_RESET_MSB};
    uint8_t accel_config[2U] = {0U};
    uint8_t gyro_config[2U] = {0U};

    /* ACC_CONF/GYR_CONF keep the existing range, bandwidth, and normal mode;
     * only the ODR field is selected by the independent acquisition task. */
    bmi323_build_config(bmi323_sample_rate, UINT16_C(0x4018), accel_config);
    bmi323_build_config(bmi323_sample_rate, UINT16_C(0x4028), gyro_config);

    bmi323_log_short("[BMI][01] init_enter\r\n");
    bmi323_ready = false;
    bmi323_last_error = BMI323_ERROR_NONE;
    bmi323_diagnostics.last_error = BMI323_ERROR_NONE;
    bmi323_diagnostics.last_status = BMI323_DIAG_STATUS_SPI_READ_FAIL;
    bmi323_diagnostics.init_result = -(int32_t)BMI323_ERROR_WHO_AM_I_READ;
    bmi323_diagnostics.who_am_i = 0U;
    bmi323_diagnostics.post_reset_who_am_i = 0U;
    bmi323_diagnostics.last_whoami = 0U;
    bmi323_diagnostics.last_rx0 = 0U;
    bmi323_diagnostics.last_rx1 = 0U;
    bmi323_diagnostics.last_rx2 = 0U;
    bmi323_diagnostics.last_rx3 = 0U;
    bmi323_diagnostics.accel_raw_x = 0;
    bmi323_diagnostics.accel_raw_y = 0;
    bmi323_diagnostics.accel_raw_z = 0;
    bmi323_diagnostics.gyro_raw_x = 0;
    bmi323_diagnostics.gyro_raw_y = 0;
    bmi323_diagnostics.gyro_raw_z = 0;
    bmi323_diagnostics.ctrl_acc = bmi323_decode_u16(accel_config);
    bmi323_diagnostics.ctrl_gyr = bmi323_decode_u16(gyro_config);
    bmi323_diagnostics.sample_rate_hz = (uint16_t)bmi323_sample_rate;
    bmi323_diagnostics.acc_conf = 0U;
    bmi323_diagnostics.gyr_conf = 0U;
    bmi323_probe_done = 0U;
    bmi323_probe_result = 0U;
    whoami_trace_done = 0U;
    whoami_trace_hal_status = -1;
    bmi323_diag = (bmi323_diag_t){
        .valid = 0U,
        .spi_status = UINT8_MAX
    };

    bmi323_log_short("[BMI][06] driver_init_enter\r\n");
    driver_status = bmi323_port_init();
    (void)snprintf(line, sizeof(line), "[BMI][07] driver_init_exit rc=%ld\r\n",
                   (long)driver_status);
    bmi323_log_short(line);
    if (driver_status != BSP_STATUS_OK) {
        bmi323_record_spi_error();
        return bmi323_init_fail(BMI323_ERROR_SPI_INIT, BMI323_DIAG_STATUS_SPI_READ_FAIL);
    }
    cs_status = bmi323_port_cs_high();
    if (cs_status != BSP_STATUS_OK) {
        bmi323_record_spi_error();
        return bmi323_init_fail(BMI323_ERROR_SPI_INIT, BMI323_DIAG_STATUS_SPI_READ_FAIL);
    }
    bmi323_log_short("[BMI][05] cs_high\r\n");
    (void)uart_log_write("[BMI323_INIT] before_delay\r\n", BMI323_LOG_TIMEOUT_MS);
    bmi323_log_short("[BMI][02] pre_delay\r\n");
    bmi323_log_short("[BMI][03] delay_enter\r\n");
    (void)snprintf(line, sizeof(line), "[IMU_TIME] t0_ms=%lu\r\n",
                   (unsigned long)imu_time_now_ms());
    bmi323_log_short(line);
    bmi323_port_delay_ms(UINT32_C(10));
    bmi323_log_short("[BMI][04] delay_exit\r\n");
    (void)uart_log_write("[BMI323_INIT] after_delay\r\n", BMI323_LOG_TIMEOUT_MS);
    bmi323_log_short("[BMI][08] chip_id_read_enter\r\n");
    const bool chip_id_ok = bmi323_spi_probe();
    who_am_i = bmi323_diagnostics.last_whoami;
    chip_id_rc = chip_id_ok ? 0 : -(int32_t)bmi323_last_error;
    if (!chip_id_ok && chip_id_rc == 0) {
        chip_id_rc = -1;
    }
    (void)snprintf(line, sizeof(line), "[BMI][09] chip_id=0x%02X rc=%ld\r\n",
                   (unsigned)who_am_i, (long)chip_id_rc);
    bmi323_log_short(line);
    if (!chip_id_ok) {
        bmi323_increment_counter(&bmi323_diagnostics.whoami_fail);
        (void)uart_log_write("WHOAMI_FAIL\r\n", BMI323_LOG_TIMEOUT_MS);
        return bmi323_init_fail(bmi323_diagnostics.last_error,
                                 bmi323_diagnostics.last_status);
    }
    bmi323_diagnostics.who_am_i = who_am_i;
    who_am_i = bmi323_diagnostics.last_whoami;
    if (who_am_i != BMI323_WHO_AM_I_VALUE) {
        bmi323_increment_counter(&bmi323_diagnostics.whoami_fail);
        (void)uart_log_write("WHOAMI_FAIL\r\n", BMI323_LOG_TIMEOUT_MS);
        return bmi323_init_fail(BMI323_ERROR_WHO_AM_I_VALUE,
                                 BMI323_DIAG_STATUS_WHO_AM_I_VALUE_ERROR);
    }

    if (!bmi323_write_reg(BMI323_REG_CMD, soft_reset, sizeof(soft_reset))) {
        return bmi323_init_fail(BMI323_ERROR_SOFT_RESET, BMI323_DIAG_STATUS_CONFIG_FAIL);
    }
    bmi323_port_delay_ms(BMI323_RESET_DELAY_MS);

    /* BMI323 requires one post-reset SPI read to restore the SPI state. */
    if (!bmi323_read_reg(BMI323_REG_CHIP_ID, reset_dummy,
                         sizeof(reset_dummy))) {
        return bmi323_init_fail(BMI323_ERROR_POST_RESET_READ,
                                 BMI323_DIAG_STATUS_CONFIG_FAIL);
    }
    bmi323_log_short("[BMI][RESET_DUMMY] ok\r\n");

    if (!bmi323_read_reg(BMI323_REG_CHIP_ID, &who_am_i, 1U)) {
        return bmi323_init_fail(BMI323_ERROR_POST_RESET_READ,
                                 BMI323_DIAG_STATUS_CONFIG_FAIL);
    }
    bmi323_diagnostics.post_reset_who_am_i = who_am_i;
    if (who_am_i != BMI323_WHO_AM_I_VALUE) {
        bmi323_increment_counter(&bmi323_diagnostics.whoami_fail);
        return bmi323_init_fail(BMI323_ERROR_WHO_AM_I_VALUE,
                                 BMI323_DIAG_STATUS_WHO_AM_I_VALUE_ERROR);
    }
    if (!bmi323_write_reg(BMI323_REG_ACC_CONF, accel_config, sizeof(accel_config)) ||
        !bmi323_read_reg(BMI323_REG_ACC_CONF, config, sizeof(config)) ||
        bmi323_decode_u16(config) != bmi323_diagnostics.ctrl_acc) {
        return bmi323_init_fail(BMI323_ERROR_ACCEL_CONFIG, BMI323_DIAG_STATUS_CONFIG_FAIL);
    }
    bmi323_diagnostics.acc_conf = bmi323_decode_u16(config);
    if (!bmi323_write_reg(BMI323_REG_GYR_CONF, gyro_config, sizeof(gyro_config)) ||
        !bmi323_read_reg(BMI323_REG_GYR_CONF, config, sizeof(config)) ||
        bmi323_decode_u16(config) != bmi323_diagnostics.ctrl_gyr) {
        return bmi323_init_fail(BMI323_ERROR_GYRO_CONFIG, BMI323_DIAG_STATUS_CONFIG_FAIL);
    }
    bmi323_diagnostics.gyr_conf = bmi323_decode_u16(config);

    bmi323_ready = true;
    bmi323_diagnostics.init_result = 0;
    bmi323_diagnostics.last_status = BMI323_DIAG_STATUS_OK;
    bmi323_log_short("[BMI][10] init_ok\r\n");
    bmi323_log_init();
    bmi323_log_debug();
    return true;
}

/** 执行一次低速 SPI/WHO_AM_I 诊断探针。 */
bool bmi323_spi_probe(void)
{
    const uint8_t tx[BMI323_SPI_RAW_BYTES] = {
        (uint8_t)(BMI323_WHO_AM_I_REG | BMI323_SPI_READ_MASK), 0U, 0U, 0U
    };
    uint8_t rx[3U][BMI323_SPI_RAW_BYTES] = {{0U}};
    bsp_spi_bmi323_raw_diagnostics_t raw[3U];
    bsp_status_t status[3U];

    if (bmi323_probe_done != 0U) {
        return bmi323_probe_result != 0U;
    }
    bmi323_probe_done = 1U;
    bmi323_probe_result = 0U;
    status[0] = bmi323_port_init();
    if (status[0] != BSP_STATUS_OK) {
        bmi323_record_spi_error();
        bmi323_set_error(BMI323_ERROR_SPI_INIT);
        bmi323_diagnostics.last_status = BMI323_DIAG_STATUS_SPI_READ_FAIL;
        return false;
    }

    /* Test 1: command and dummy/data phases are separate HAL calls, while
     * CS remains asserted. Tests 2 and 3 clock the same four bytes in one HAL
     * call; test 3 is the Bosch callback contract: dummy + two payload bytes. */
    status[0] = bsp_spi_bmi323_raw_transaction(tx, rx[0], sizeof(tx), 1U,
                                                BMI323_SPI_TIMEOUT_MS, &raw[0]);
    bmi323_log_raw_probe(1U, "datasheet_split_cs_low", tx, rx[0], &raw[0]);
    status[1] = bsp_spi_bmi323_raw_transaction(tx, rx[1], sizeof(tx), 0U,
                                                BMI323_SPI_TIMEOUT_MS, &raw[1]);
    bmi323_log_raw_probe(2U, "single_hal_cs_low", tx, rx[1], &raw[1]);
    status[2] = bsp_spi_bmi323_raw_transaction(tx, rx[2], sizeof(tx), 0U,
                                                BMI323_SPI_TIMEOUT_MS, &raw[2]);
    bmi323_log_raw_probe(3U, "bosch_dummy1_callback_len3", tx, rx[2], &raw[2]);

    for (uint8_t index = 0U; index < 3U; ++index) {
        bmi323_increment_counter(&bmi323_diagnostics.spi_read_count);
        if (status[index] == BSP_STATUS_OK) {
            bmi323_increment_counter(&bmi323_diagnostics.spi_read_success);
        } else {
            bmi323_increment_counter(&bmi323_diagnostics.spi_read_fail);
            bmi323_increment_counter(&bmi323_diagnostics.spi_rx_fail);
        }
    }

    bmi323_capture_whoami_raw(BMI323_WHO_AM_I_REG, tx, rx[2], sizeof(tx));
    if (status[2] != BSP_STATUS_OK) {
        bmi323_record_spi_error();
        bmi323_diagnostics.last_status = status[2] == BSP_STATUS_TIMEOUT
                                              ? BMI323_DIAG_STATUS_WHO_AM_I_TIMEOUT
                                              : BMI323_DIAG_STATUS_SPI_RX_FAIL;
        bmi323_set_error(status[2] == BSP_STATUS_TIMEOUT
                             ? BMI323_ERROR_WHO_AM_I_TIMEOUT
                             : BMI323_ERROR_SPI_RX_FAIL);
        return false;
    }
    if (rx[2][2] != BMI323_WHO_AM_I_VALUE) {
        bmi323_set_error(BMI323_ERROR_WHO_AM_I_VALUE);
        bmi323_diagnostics.last_status = BMI323_DIAG_STATUS_WHO_AM_I_LOW_BYTE_ERROR;
        return false;
    }
    /*
     * The BMI323 identity contract used by the active driver is the 8-bit
     * CHIP_ID value in RX[2]. RX[3] is retained in the raw trace, but must not
     * turn a valid 0x43 response into an offline result: the observed device
     * returns a non-zero value for the extra diagnostic byte.
     */
    bmi323_probe_result = 1U;
    bmi323_diag.valid = 1U;
    bmi323_diagnostics.last_status = BMI323_DIAG_STATUS_OK;
    return true;
}

/** 读取换算后的三轴加速度，单位 m/s^2。 */
bool bmi323_read_accel(float *x, float *y, float *z)
{
    uint8_t raw[6U] = {0U};
    int16_t raw_x;
    int16_t raw_y;
    int16_t raw_z;

    if (x == NULL || y == NULL || z == NULL) {
        bmi323_set_error(BMI323_ERROR_DATA_READ);
        return false;
    }
    if (!bmi323_ready) {
        bmi323_increment_counter(&bmi323_diagnostics.read_fail);
        if (bmi323_diagnostics.last_status == BMI323_DIAG_STATUS_OK) {
            bmi323_diagnostics.last_status = BMI323_DIAG_STATUS_DATA_NOT_READY;
        }
        return false;
    }
    if (!bmi323_read_reg(BMI323_REG_ACC_DATA_X, raw, sizeof(raw))) {
        bmi323_increment_counter(&bmi323_diagnostics.read_fail);
        return false;
    }
    raw_x = bmi323_decode_s16(&raw[0]);
    raw_y = bmi323_decode_s16(&raw[2]);
    raw_z = bmi323_decode_s16(&raw[4]);
    *x = bmi323_accel_raw_to_mps2(raw_x);
    *y = bmi323_accel_raw_to_mps2(raw_y);
    *z = bmi323_accel_raw_to_mps2(raw_z);
    bmi323_diagnostics.accel_raw_x = raw_x;
    bmi323_diagnostics.accel_raw_y = raw_y;
    bmi323_diagnostics.accel_raw_z = raw_z;
    bmi323_increment_counter(&bmi323_diagnostics.read_ok);
    bmi323_diagnostics.last_status = BMI323_DIAG_STATUS_OK;
    return true;
}

/** 读取换算后的三轴角速度，单位 rad/s。 */
bool bmi323_read_gyro(float *x, float *y, float *z)
{
    uint8_t raw[6U] = {0U};
    int16_t raw_x;
    int16_t raw_y;
    int16_t raw_z;

    if (x == NULL || y == NULL || z == NULL) {
        bmi323_set_error(BMI323_ERROR_DATA_READ);
        return false;
    }
    if (!bmi323_ready) {
        bmi323_increment_counter(&bmi323_diagnostics.read_fail);
        if (bmi323_diagnostics.last_status == BMI323_DIAG_STATUS_OK) {
            bmi323_diagnostics.last_status = BMI323_DIAG_STATUS_DATA_NOT_READY;
        }
        return false;
    }
    if (!bmi323_read_reg(BMI323_REG_GYR_DATA_X, raw, sizeof(raw))) {
        bmi323_increment_counter(&bmi323_diagnostics.read_fail);
        return false;
    }
    raw_x = bmi323_decode_s16(&raw[0]);
    raw_y = bmi323_decode_s16(&raw[2]);
    raw_z = bmi323_decode_s16(&raw[4]);
    *x = bmi323_gyro_raw_to_rads(raw_x);
    *y = bmi323_gyro_raw_to_rads(raw_y);
    *z = bmi323_gyro_raw_to_rads(raw_z);
    bmi323_diagnostics.gyro_raw_x = raw_x;
    bmi323_diagnostics.gyro_raw_y = raw_y;
    bmi323_diagnostics.gyro_raw_z = raw_z;
    bmi323_increment_counter(&bmi323_diagnostics.read_ok);
    bmi323_diagnostics.last_status = BMI323_DIAG_STATUS_OK;
    return true;
}

/** 读取原始加速度/陀螺计数，供标定和回放使用。 */
bool bmi323_read_raw_sample(int16_t accel[3], int16_t gyro[3])
{
    uint8_t status = 0U;
    uint8_t raw[12U] = {0U};

    if (accel == NULL || gyro == NULL) {
        bmi323_set_error(BMI323_ERROR_DATA_READ);
        return false;
    }
    if (!bmi323_ready) {
        bmi323_increment_counter(&bmi323_diagnostics.read_fail);
        if (bmi323_diagnostics.last_status == BMI323_DIAG_STATUS_OK) {
            bmi323_diagnostics.last_status = BMI323_DIAG_STATUS_DATA_NOT_READY;
        }
        return false;
    }

    /* Do not timestamp/integrate a register image that has not advanced since
     * the previous poll. The sensor status read is deliberately separate from
     * the data burst to preserve the existing register-layout contract. */
    if (!bmi323_read_reg(BMI323_REG_STATUS, &status, sizeof(status))) {
        bmi323_increment_counter(&bmi323_diagnostics.read_fail);
        return false;
    }
    if ((status & (BMI323_STATUS_ACC_DATA_READY |
                   BMI323_STATUS_GYR_DATA_READY)) !=
        (BMI323_STATUS_ACC_DATA_READY | BMI323_STATUS_GYR_DATA_READY)) {
        bmi323_diagnostics.last_status = BMI323_DIAG_STATUS_DATA_NOT_READY;
        return false;
    }

    /* ACC_DATA_X through GYR_DATA_Z are contiguous 16-bit registers. One
     * transaction keeps accel and gyro from being sampled in separate reads. */
    if (!bmi323_read_reg(BMI323_REG_ACC_DATA_X, raw, sizeof(raw))) {
        bmi323_increment_counter(&bmi323_diagnostics.read_fail);
        return false;
    }
    for (uint8_t axis = 0U; axis < 3U; ++axis) {
        accel[axis] = bmi323_decode_s16(&raw[(uint16_t)axis * 2U]);
        gyro[axis] = bmi323_decode_s16(&raw[6U + ((uint16_t)axis * 2U)]);
    }
    bmi323_diagnostics.accel_raw_x = accel[0];
    bmi323_diagnostics.accel_raw_y = accel[1];
    bmi323_diagnostics.accel_raw_z = accel[2];
    bmi323_diagnostics.gyro_raw_x = gyro[0];
    bmi323_diagnostics.gyro_raw_y = gyro[1];
    bmi323_diagnostics.gyro_raw_z = gyro[2];
    bmi323_increment_counter(&bmi323_diagnostics.read_ok);
    bmi323_diagnostics.last_status = BMI323_DIAG_STATUS_OK;
    return true;
}

float bmi323_accel_raw_to_mps2(int16_t raw)
{
    return (float)raw * ((BMI323_ACC_RANGE_G * BMI323_GRAVITY_MPS2) /
                         32768.0f);
}

float bmi323_gyro_raw_to_rads(int16_t raw)
{
    return (float)raw * ((BMI323_GYRO_RANGE_DPS / 32768.0f) *
                         BMI323_DEG_TO_RAD);
}

/** 读取芯片温度诊断值。 */
bool bmi323_read_temperature(float *temperature)
{
    uint8_t raw[2U] = {0U};

    if (temperature == NULL) {
        bmi323_set_error(BMI323_ERROR_DATA_READ);
        return false;
    }
    if (!bmi323_ready) {
        bmi323_increment_counter(&bmi323_diagnostics.read_fail);
        if (bmi323_diagnostics.last_status == BMI323_DIAG_STATUS_OK) {
            bmi323_diagnostics.last_status = BMI323_DIAG_STATUS_DATA_NOT_READY;
        }
        return false;
    }
    if (!bmi323_read_reg(BMI323_REG_TEMP_DATA, raw, sizeof(raw))) {
        bmi323_increment_counter(&bmi323_diagnostics.read_fail);
        return false;
    }
    *temperature = ((float)bmi323_decode_s16(raw) / BMI323_TEMP_SCALE) +
                  BMI323_TEMP_OFFSET_C;
    bmi323_increment_counter(&bmi323_diagnostics.read_ok);
    bmi323_diagnostics.last_status = BMI323_DIAG_STATUS_OK;
    return true;
}

bmi323_error_t bmi323_get_last_error(void)
{
    return bmi323_last_error;
}

bmi323_diag_status_t bmi323_get_status(void)
{
    return bmi323_diagnostics.last_status;
}

uint8_t bmi323_is_online(void)
{
    return bmi323_ready ? 1U : 0U;
}

uint8_t bmi323_is_ready(void)
{
    return bmi323_is_online();
}

void bmi323_refresh_data_ready_status(void)
{
    uint8_t status = 0U;
    const uint8_t required = BMI323_STATUS_ACC_DATA_READY | BMI323_STATUS_GYR_DATA_READY;

    if (!bmi323_ready) {
        return;
    }
    if (!bmi323_read_reg(BMI323_REG_STATUS, &status, sizeof(status))) {
        bmi323_diagnostics.last_status = BMI323_DIAG_STATUS_SPI_READ_FAIL;
        return;
    }
    bmi323_diagnostics.last_status = (status & required) == required
                                        ? BMI323_DIAG_STATUS_OK
                                        : BMI323_DIAG_STATUS_DATA_NOT_READY;
}

/** 复制完整诊断快照。 */
void bmi323_get_diagnostics(bmi323_diagnostics_t *diagnostics)
{
    if (diagnostics != NULL) {
        *diagnostics = bmi323_diagnostics;
    }
}

/** 复制一次性原始 WHO_AM_I 探针结果。 */
void bmi323_get_diag(bmi323_diag_t *diag)
{
    if (diag != NULL) {
        *diag = bmi323_diag;
    }
}

const char *bmi323_diag_status_name(bmi323_diag_status_t status)
{
    switch (status) {
    case BMI323_DIAG_STATUS_WHO_AM_I_FAIL: return "WHO_AM_I_FAIL";
    case BMI323_DIAG_STATUS_SPI_READ_FAIL: return "SPI_READ_FAIL";
    case BMI323_DIAG_STATUS_CONFIG_FAIL: return "CONFIG_FAIL";
    case BMI323_DIAG_STATUS_DATA_NOT_READY: return "DATA_NOT_READY";
    case BMI323_DIAG_STATUS_WHO_AM_I_TIMEOUT: return "WHO_AM_I_TIMEOUT";
    case BMI323_DIAG_STATUS_SPI_TX_FAIL: return "SPI_TX_FAIL";
    case BMI323_DIAG_STATUS_SPI_RX_FAIL: return "SPI_RX_FAIL";
    case BMI323_DIAG_STATUS_WHO_AM_I_VALUE_ERROR: return "WHO_AM_I_VALUE_ERROR";
    case BMI323_DIAG_STATUS_WHO_AM_I_LOW_BYTE_ERROR: return "WHO_AM_I_LOW_BYTE_ERROR";
    case BMI323_DIAG_STATUS_WHO_AM_I_HIGH_BYTE_ERROR: return "WHO_AM_I_HIGH_BYTE_ERROR";
    case BMI323_DIAG_STATUS_OK:
    default: return "OK";
    }
}
