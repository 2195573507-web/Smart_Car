#ifndef BSP_TEST_H
#define BSP_TEST_H

#include <stdint.h>
#include <stddef.h>

#include "bsp_status.h"
#include "bsp_pwm.h"

#ifdef __cplusplus
extern "C" {
#endif

/* BSP 编译/台架探针；创建人：待确认（当前维护人：Zhiqin）。
 * 当前生产启动路径没有调用这些接口，但一旦显式执行就会访问真实外设，不能当作纯主机测试。 */

/**
 * @brief 依次调用 GPIO/SPI/I2C/PWM/UART/ADC 兼容 API，维持编译和链接覆盖。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * 传入参数：无。
 * @return 调用开始时 DWT 微秒时间戳的低 32 位；不会汇总各 BSP 返回状态。
 * 调用方式：正常固件只编译而不执行；若台架显式调用，会实际访问 SPI1 和 I2C4，
 *           不能在已连接执行器/传感器且未评估副作用时运行。
 * 线程约束：包含多个同步 HAL/BSP 调用、无统一锁，禁止 ISR 或与外设 owner 并发调用。
 */
uint32_t bsp_test_compile(void);
/**
 * @brief 通过 bsp_spi_write_read() 执行一段 SPI1 全双工台架事务。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * @param tx_data 非 NULL、至少 size 字节的只读发送缓冲。
 * @param rx_data 非 NULL、至少 size 字节的可写接收缓冲；失败时不得使用内容。
 * @param size 字节数，必须大于 0 并满足 BSP uint16_t 长度限制。
 * @param timeout_ms 传给阻塞 SPI HAL 的超时，单位 ms。
 * @return BSP 状态；参数非法直接返回 INVALID_ARG，其余语义继承 bsp_spi_write_read()。
 * 调用方式：仅接线正确、CS 所有权已由调用方处理的静止台架；名称不保证存在物理 loopback。
 * 线程约束：无内部互斥，必须独占 SPI1/CS，禁止 ISR 调用。
 */
bsp_status_t bsp_test_spi_loopback(const uint8_t *tx_data, uint8_t *rx_data,
                                   size_t size, uint32_t timeout_ms);
/**
 * @brief 阻塞探测 I2C 7 位地址 0x08..0x77，每个地址最多尝试两次。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * @param addresses 可选输出数组；非 NULL 时最多写 capacity 个发现地址。
 * @param capacity addresses 可写元素数；不会限制总扫描或返回计数。
 * @param timeout_ms 每次 HAL probe 的超时，单位 ms；总最坏阻塞随 112 个地址累积。
 * @return 发现设备总数，可能大于 capacity；不返回单地址错误原因。
 * 调用方式：只在确认总线设备允许全地址探测的台架运行，车辆实时任务不得调用。
 * 线程约束：无 I2C mutex，必须独占 I2C4，禁止 ISR/并发调用。
 */
size_t bsp_test_i2c_scan(uint8_t *addresses, size_t capacity, uint32_t timeout_ms);
/**
 * @brief 通过 UART1 日志 BSP 阻塞发送一条零结尾文本。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * @param text 只读零结尾文本；NULL 的失败语义继承 uart_log_write()。
 * @param timeout_ms UART 日志写入超时，单位 ms。
 * @return bsp_uart_log_write() 的 BSP 状态。
 * 调用方式：UART1 BSP 就绪后做台架输出验证；不证明 BLE/SRP 日志链路。
 * 线程约束：可能阻塞并使用 UART 日志锁，禁止 ISR 调用。
 */
bsp_status_t bsp_test_uart_output(const char *text, uint32_t timeout_ms);
/**
 * @brief 初始化 PWM BSP、设置占空比并启动指定 TIM3 通道。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * @param channel 仅允许 BSP 已开放的 CH3/CH4；CH1/CH2 因 USART6 归属会被拒绝。
 * @param duty_percent 占空比 0..100。
 * @return 首个失败步骤的 BSP 状态，全部成功返回 OK。
 * 调用方式：仅在执行器隔离的静止台架运行；本函数不会自动停止通道或验证外部安全电平。
 * 线程约束：直接操作 TIM3、无内部锁，必须独占通道，禁止 ISR/控制任务并发调用。
 */
bsp_status_t bsp_test_pwm_output(bsp_pwm_channel_t channel, uint8_t duty_percent);

#ifdef __cplusplus
}
#endif

#endif /* BSP_TEST_H */
