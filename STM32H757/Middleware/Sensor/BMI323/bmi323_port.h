#ifndef SMARTCAR_SENSOR_BMI323_PORT_H
#define SMARTCAR_SENSOR_BMI323_PORT_H

#include <stdint.h>

#include "bsp_gpio.h"
#include "bsp_status.h"

/* BMI323 硬件端口适配层；创建人：待确认（当前维护人：Zhiqin）。
 * 只处理 CS/SPI/延时，不解释寄存器地址或数据含义。 */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * 单次 BMI323 SPI 读事务的片选诊断快照；端口层按值写入调用方提供的对象，不保留其地址。
 */
typedef struct
{
    bsp_gpio_level_t cs_before; /**< 事务拉低 CS 前读取到的引脚电平。 */
    bsp_gpio_level_t cs_after; /**< 事务结束并释放 CS 后读取到的引脚电平。 */
    bsp_status_t cs_before_status; /**< 读取事务前 CS 电平时的 BSP 返回状态。 */
    bsp_status_t cs_after_status; /**< 读取事务后 CS 电平时的 BSP 返回状态。 */
} bmi323_port_trace_t;

/**
 * @brief 初始化 BMI323 GPIO/SPI 端口并把 CS 恢复为安全高电平。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * 传入参数：无。
 * @return `BSP_STATUS_OK` 表示 GPIO、SPI 和最终 CS 高电平均成功；否则返回
 *         首个底层 BSP 错误。
 * @note 调用方式与线程约束：仅 BMI323 初始化/探针任务调用；可能调整调度器启动前的
 *       BASEPRI 并输出有限日志，禁止与 SPI 事务并发或从 ISR 调用。
 */
bsp_status_t bmi323_port_init(void);
/**
 * @brief 拉低 BMI323 CS 并执行固定建立时间忙等待。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * 传入参数：无。
 * @return 返回 GPIO 写入的 `bsp_status_t`；失败时不得开始 SPI 事务。
 * @note 调用方式与线程约束：仅由端口事务封装调用；会忙等待并可能记录日志，非线程安全，
 *       禁止从 ISR 或绕过设备事务 owner 调用。
 */
bsp_status_t bmi323_port_cs_low(void);
/**
 * @brief 拉高 BMI323 CS 以结束或复位当前 SPI 事务边界。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * 传入参数：无。
 * @return 返回 GPIO 写入的 `bsp_status_t`。
 * @note 调用方式与线程约束：仅由初始化或端口事务封装调用；可能输出有限日志，非线程安全，
 *       禁止从 ISR 调用。
 */
bsp_status_t bmi323_port_cs_high(void);
/**
 * @brief 在单个 CS 低电平窗口内执行阻塞式全双工 SPI 读事务。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * @param[in] tx_data 调用方拥有的发送字节，调用期间只读，不允许为 NULL。
 * @param[out] rx_data 调用方拥有的接收缓冲区，容量至少为 `length` 字节。
 * @param[in] length 同时发送和接收的字节数，必须大于 0。
 * @param[in] timeout_ms 底层 SPI 事务超时，单位 ms。
 * @param[out] trace 可选 CS 前后电平诊断输出；NULL 表示不采集 trace。
 * @return `BSP_STATUS_OK` 表示 SPI 和最终 CS 释放均成功；参数、SPI 或 GPIO
 *         失败返回对应状态，失败时接收内容不得作为有效数据。
 * @note 调用方式与线程约束：仅 BMI323 驱动 owner 在任务上下文调用；阻塞时间受
 *       `timeout_ms` 和忙等待影响，缓冲区所有权不转移，禁止并发或 ISR 调用。
 */
bsp_status_t bmi323_port_spi_read(const uint8_t *tx_data, uint8_t *rx_data,
                                  uint16_t length, uint32_t timeout_ms,
                                  bmi323_port_trace_t *trace);
/**
 * @brief 在单个 CS 低电平窗口内执行阻塞式 SPI 写事务。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * @param[in] tx_data 调用方拥有的发送缓冲区，不允许为 NULL。
 * @param[in] length 发送字节数，必须大于 0。
 * @param[in] timeout_ms 底层 SPI 事务超时，单位 ms。
 * @return `BSP_STATUS_OK` 表示发送和最终 CS 释放成功；参数、SPI 或 GPIO
 *         失败返回对应状态。
 * @note 调用方式与线程约束：仅 BMI323 驱动 owner 在任务上下文调用；函数在返回前完成
 *       事务且不保留输入指针，可能阻塞，禁止并发或从 ISR 调用。
 */
bsp_status_t bmi323_port_spi_write(const uint8_t *tx_data, uint16_t length,
                                   uint32_t timeout_ms);
/**
 * @brief 使用 IMU 单调时基执行微秒级忙等待。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * @param[in] delay_us 目标等待时间，单位 us；0 表示立即返回。
 * @return 无返回值。
 * @note 调用方式与线程约束：仅用于 BMI323 CS 建立/保持和初始化时序；占用 CPU 且前若干次
 *       可能输出日志，禁止在 ISR、长实时控制段或任意大延时路径调用。
 */
void bmi323_port_delay_us(uint32_t delay_us);
/**
 * @brief 使用 IMU 单调时基执行毫秒级忙等待。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * @param[in] delay_ms 目标等待时间，单位 ms；内部扩展为 64 位微秒。
 * @return 无返回值。
 * @note 调用方式与线程约束：仅用于 BMI323 上电/软复位稳定时间；全程占用 CPU，禁止在
 *       调度敏感任务、实时控制循环或 ISR 中调用。
 */
void bmi323_port_delay_ms(uint32_t delay_ms);
/**
 * @brief 读取 SPI BSP 缓存的最近一次原始 HAL 状态码。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * 传入参数：无。
 * @return 最近一次 SPI HAL 状态的 `int32_t` 表示；读取不清零。
 * @note 调用方式与线程约束：仅用于故障诊断；不阻塞、不访问总线，值会被任何后续 SPI
 *       操作覆盖，不能单独证明当前事务状态。
 */
int32_t bmi323_port_get_last_hal_status(void);

#ifdef __cplusplus
}
#endif

#endif /* SMARTCAR_SENSOR_BMI323_PORT_H */
