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
#include "smartcar_debug_config.h"

/* BMI323 SPI/CS 端口适配实现；创建人：待确认（当前维护人：Zhiqin）。 */

#define BMI323_PORT_CS_DELAY_US UINT32_C(2)

static uint8_t bmi323_port_trace_active;
static uint8_t bmi323_port_spi_config_logged;
static uint8_t bmi323_port_delay_trace_count;

/**
 * @brief 在调度器尚未启动时清除 BASEPRI，使 HAL 超时能够观察 SysTick。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * 传入参数：无。
 * @return 无返回值；调度器已启动时保持中断屏蔽状态不变，未启动时直接写 BASEPRI=0。
 * 调用方式：由端口初始化、CS 控制、SPI 事务和延时入口在调用 HAL/计时服务前同步调用。
 * 线程约束：不阻塞、不使用 mutex，但会修改 CPU 中断屏蔽寄存器。
 *           禁止 ISR，也不得在需要保留 BASEPRI 的临界区中调用；无对象所有权转移。
 */
static void bmi323_port_allow_hal_tick(void)
{
    /* BMI323 init runs before vTaskStartScheduler(). FreeRTOS's pre-scheduler
     * critical-section sentinel can leave BASEPRI raised after queue/mutex use;
     * release only that mask so HAL SPI timeout processing can observe SysTick. */
    if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED) {
        __set_BASEPRI(0U);
    }
}

/**
 * @brief 为有限次数的延时诊断申请一个日志名额并递增全局计数。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * 传入参数：无。
 * @return 尚未达到 BMI323_PORT_DELAY_TRACE_LIMIT 时返回 1；达到上限后返回 0 且计数保持不变。
 * 调用方式：由 bmi323_port_delay_us() 和 bmi323_port_delay_ms() 在延时开始前调用。
 * 线程约束：不阻塞、不使用 mutex，计数读改写也不是原子操作。
 *           BMI323 端口必须由单一任务串行使用；禁止 ISR 或多任务并发；无所有权转移。
 */
static uint8_t bmi323_port_delay_trace_begin(void)
{
    if (bmi323_port_delay_trace_count >= BMI323_PORT_DELAY_TRACE_LIMIT) {
        return 0U;
    }
    ++bmi323_port_delay_trace_count;
    return 1U;
}

/**
 * @brief 输出一次 BMI323 忙等待开始时的微秒、毫秒和当前时间戳。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] delay_us 日志中记录的微秒延时；超大毫秒延时由调用方饱和到 UINT32_MAX。
 * @param[in] delay_ms 日志中记录的毫秒延时。
 * @return 无返回值；格式化或 UART 写入失败不向调用方报告，也不改变延时参数。
 * 调用方式：由两个公共延时函数在有限 trace 名额有效时同步调用。
 * 线程约束：UART 写入最多阻塞 BMI323_PORT_LOG_TIMEOUT_MS，且不使用 mutex。
 *           禁止 ISR；参数按值读取，无所有权转移。
 */
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

/**
 * @brief 输出一次 BMI323 忙等待结束时的当前时间戳。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * 传入参数：无。
 * @return 无返回值；格式化或 UART 写入失败被忽略，不修改端口状态。
 * 调用方式：由两个公共延时函数在对应的 trace 入口日志之后同步调用。
 * 线程约束：UART 写入最多阻塞 BMI323_PORT_LOG_TIMEOUT_MS，不使用 mutex，禁止 ISR 调用；无所有权转移。
 */
static void bmi323_port_log_delay_exit(void)
{
    char line[48];

    (void)snprintf(line, sizeof(line), "[BMI][D-] time_ms=%lu\r\n",
                   (unsigned long)imu_time_now_ms());
    (void)uart_log_write(line, BMI323_PORT_LOG_TIMEOUT_MS);
}

/**
 * @brief 在当前进程生命周期内最多一次输出 SPI1 固定配置摘要。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * 传入参数：无。
 * @return 无返回值；已记录标志非零时不输出；标志在 UART 调用前置位，因此发送失败也不会重试。
 * 调用方式：仅由 bmi323_port_init() 在 BSP SPI 初始化成功后调用。
 * 线程约束：UART 写入最多阻塞 BMI323_PORT_LOG_TIMEOUT_MS，且不使用 mutex；标志访问不是原子操作。
 *           禁止 ISR 或并发初始化；无所有权转移。
 */
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

/**
 * @brief 在原始 WHO_AM_I trace 激活期间输出一次 CS 状态标签。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] state 以 NUL 结尾的状态文本；NULL 或 trace 未激活时静默跳过。
 * @return 无返回值；UART 发送结果被忽略，输入文本和 trace 标志保持不变。
 * 调用方式：由 bmi323_port_cs_low() / bmi323_port_cs_high() 在 GPIO 写成功后调用。
 * 线程约束：UART 写入最多阻塞 BMI323_PORT_LOG_TIMEOUT_MS；低电平分支会在 CS 已拉低时执行日志。
 *           函数没有 mutex；禁止 ISR 或并发事务；字符串仅在调用期间借用。
 */
static void bmi323_port_log_cs(const char *state)
{
    char line[48];

    if (bmi323_port_trace_active == 0U || state == NULL) {
        return;
    }
    (void)snprintf(line, sizeof(line), "[BMI323][CS] %s\r\n", state);
    (void)uart_log_write(line, BMI323_PORT_LOG_TIMEOUT_MS);
}

/**
 * @brief 拉高 CS、满足保持时间并完成可选 trace 的事务收尾。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] status 前序 SPI/CS 操作状态。
 * @param[in,out] trace 可选事务 trace；非 NULL 时写入结束 CS 电平/读取状态并关闭全局 trace，NULL 时不写输出。
 * @return 前序状态非 OK 时原样返回；前序成功但 CS 拉高失败时返回该失败。
 *         全部成功返回 BSP_STATUS_OK；失败时不回滚已经写入的 trace 字段。
 * 调用方式：由 bmi323_port_spi_read() 和 bmi323_port_spi_write() 在每次事务末尾调用。
 * 线程约束：包含 GPIO 操作和微秒忙等待，且不使用 mutex。
 *           SPI/CS 必须由单一任务持有；禁止 ISR 或并发事务；trace 仅在调用期间借用。
 */
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

/** 初始化端口、CS 安全高电平和一次性诊断状态。 */
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

/** 拉低 BMI323 CS，并保留必要建立时间。 */
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

/** 拉高 BMI323 CS，结束当前 SPI 事务。 */
bsp_status_t bmi323_port_cs_high(void)
{
    bmi323_port_allow_hal_tick();
    const bsp_status_t status = bsp_gpio_write(BSP_GPIO_BMI323_CS, BSP_GPIO_HIGH);

    if (status == BSP_STATUS_OK) {
        bmi323_port_log_cs("BMI323_CS_HIGH");
    }
    return status;
}

/** 执行一次带 trace 的 SPI 读事务。 */
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

/** 执行一次 SPI 写事务。 */
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

/** 提供 BMI323 所需微秒级忙等待和有限诊断日志。 */
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

/** 提供 BMI323 所需毫秒级忙等待和有限诊断日志。 */
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

/** 返回最近一次 SPI HAL 状态。 */
int32_t bmi323_port_get_last_hal_status(void)
{
    return bsp_spi_get_last_hal_status();
}
