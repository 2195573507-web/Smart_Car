#ifndef BSP_I2C_H
#define BSP_I2C_H

#include <stddef.h>
#include <stdint.h>
#include "bsp_status.h"

/* I2C4 BSP；创建人：待确认（当前维护人：Zhiqin）。
 * LSM303 与其他 I2C 设备共享总线；当前实现没有 mutex，所有事务必须由上层
 * 在任务上下文串行化，不能把“HAL 阻塞调用”误解为并发保护。 */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  校验 CubeMX 已初始化的 I2C4 handle，并开放 BSP 事务接口。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（接口契约补充）。
 * 传入参数：无。
 * @return BSP_STATUS_OK 表示 I2C4 当前 READY；实例/状态不符返回 BSP_STATUS_NOT_READY。
 * 调用方式：在 CubeMX MX_I2C4_Init() 成功后、任何传感器事务前调用。
 * 线程约束：只允许启动任务调用；不创建 mutex，不得与正在进行的 I2C 事务并发。
 */
bsp_status_t bsp_i2c_init(void);
/**
 * @brief  以主机模式阻塞发送一段 I2C 数据。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（接口契约补充）。
 * @param  address_7bit 0x00..0x7F 的 7 位地址，不包含 R/W 位；函数内部左移一位。
 * @param  data 非 NULL 的只读发送缓冲；函数返回前不会保留指针。
 * @param  size 发送字节数，范围 1..UINT16_MAX。
 * @param  timeout_ms 传给 HAL 单次发送操作的超时，单位 ms。
 * @return OK、INVALID_ARG、NOT_READY、TIMEOUT 或 HAL 失败映射的 ERROR。
 * 调用方式：先成功调用 bsp_i2c_init()；失败后由设备驱动决定重试/降级，不得使用旧 ACK 推断在线。
 * 线程约束：任务上下文阻塞调用，无内部锁；同一 I2C4 的所有调用方必须在外层串行化，禁止从 ISR 调用。
 */
bsp_status_t bsp_i2c_write(uint16_t address_7bit, const uint8_t *data,
                           size_t size, uint32_t timeout_ms);
/**
 * @brief  以主机模式阻塞读取一段 I2C 数据。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（接口契约补充）。
 * @param  address_7bit 0x00..0x7F 的 7 位地址，不包含 R/W 位。
 * @param[out] data 非 NULL、至少可写 size 字节的输出缓冲。
 * @param  size 接收字节数，范围 1..UINT16_MAX。
 * @param  timeout_ms 传给 HAL 单次接收操作的超时，单位 ms。
 * @return OK、INVALID_ARG、NOT_READY、TIMEOUT 或 ERROR；非 OK 时 data 不得作为有效样本。
 * 调用方式：先成功调用 bsp_i2c_init()，并在上层持有 I2C4 总线所有权。
 * 线程约束：任务上下文阻塞调用，无内部锁；禁止并发和 ISR 调用。
 */
bsp_status_t bsp_i2c_read(uint16_t address_7bit, uint8_t *data,
                          size_t size, uint32_t timeout_ms);
/**
 * @brief  先阻塞写入、成功后再阻塞读取，供寄存器地址加数据读取使用。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（接口契约补充）。
 * @param  address_7bit 0x00..0x7F 的 7 位地址。
 * @param  tx_data 非 NULL、包含寄存器地址/命令的只读缓冲。
 * @param  tx_size 写入长度，范围 1..UINT16_MAX。
 * @param[out] rx_data 非 NULL、至少可写 rx_size 字节的输出缓冲。
 * @param  rx_size 读取长度，范围 1..UINT16_MAX。
 * @param  timeout_ms 分别传给写和读两次 HAL 调用；最坏阻塞可能接近两倍该值。
 * @return 首个失败阶段的 BSP 状态；读取失败时 rx_data 不得作为有效样本。
 * 调用方式：当前实现是两个独立 Master_Transmit/Master_Receive 调用，不能保证 repeated-start；
 *           依赖重复起始的器件须改用匹配 HAL API，并以总线波形验证。
 * 线程约束：任务上下文阻塞调用，无内部锁；整个写后读序列必须由外层锁保护，禁止从 ISR 调用。
 */
bsp_status_t bsp_i2c_write_read(uint16_t address_7bit, const uint8_t *tx_data,
                                size_t tx_size, uint8_t *rx_data,
                                size_t rx_size, uint32_t timeout_ms);
/**
 * @brief  使用 HAL_I2C_IsDeviceReady 探测一个 7 位设备地址。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（接口契约补充）。
 * @param  address_7bit 0x00..0x7F 的 7 位地址。
 * @param  trials HAL 最大探测次数，必须大于 0。
 * @param  timeout_ms HAL 探测超时，单位 ms。
 * @return OK 表示器件应答；其余状态区分参数、未初始化、超时或总线错误。
 * 调用方式：只用于启动探测/有界故障恢复，不应放入 IMU 高频采样循环。
 * 线程约束：任务上下文阻塞调用，无内部锁；与其他 I2C4 事务串行化，禁止从 ISR 调用。
 */
bsp_status_t bsp_i2c_probe(uint16_t address_7bit, uint32_t trials, uint32_t timeout_ms);

/* 以下短名称仅作兼容包装，完整契约继承对应 bsp_* API。 */
/** @copydoc bsp_i2c_init
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（兼容包装契约补充）。 */
static inline bsp_status_t i2c_init(void) { return bsp_i2c_init(); }
/** @copydoc bsp_i2c_write
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（兼容包装契约补充）。 */
static inline bsp_status_t i2c_write(uint16_t address_7bit, const uint8_t *data,
                                     size_t size, uint32_t timeout_ms)
{
    return bsp_i2c_write(address_7bit, data, size, timeout_ms);
}
/** @copydoc bsp_i2c_read
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（兼容包装契约补充）。 */
static inline bsp_status_t i2c_read(uint16_t address_7bit, uint8_t *data,
                                    size_t size, uint32_t timeout_ms)
{
    return bsp_i2c_read(address_7bit, data, size, timeout_ms);
}
/** @copydoc bsp_i2c_write_read
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（兼容包装契约补充）。 */
static inline bsp_status_t i2c_write_read(uint16_t address_7bit, const uint8_t *tx_data,
                                          size_t tx_size, uint8_t *rx_data,
                                          size_t rx_size, uint32_t timeout_ms)
{
    return bsp_i2c_write_read(address_7bit, tx_data, tx_size, rx_data, rx_size, timeout_ms);
}
/** @copydoc bsp_i2c_probe
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（兼容包装契约补充）。 */
static inline bsp_status_t i2c_probe(uint16_t address_7bit, uint32_t trials, uint32_t timeout_ms)
{
    return bsp_i2c_probe(address_7bit, trials, timeout_ms);
}

#ifdef __cplusplus
}
#endif

#endif /* BSP_I2C_H */
