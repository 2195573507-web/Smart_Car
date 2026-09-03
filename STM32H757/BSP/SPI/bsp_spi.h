#ifndef BSP_SPI_H
#define BSP_SPI_H

#include <stddef.h>
#include <stdint.h>
#include "bsp_status.h"
#include "bsp_gpio.h"

/* SPI BSP；创建人：待确认（当前维护人：Zhiqin）。
 * BMI323 原始诊断接口只提供有界事务，不改变正常驱动的 CS/时序所有权。 */

#ifdef __cplusplus
extern "C" {
#endif

/** 首次普通 SPI 全双工访问的前后状态诊断快照。 */
typedef struct {
    uint8_t cs_active; /**< 事务期间 BMI323 CS 是否观察为低电平。 */
    uint8_t rx0; /**< 首个接收字节。 */
    uint8_t rx1; /**< 第二个接收字节；短事务保持默认值。 */
    uint32_t spi_state_before; /**< 调用 HAL 前的 HAL SPI state。 */
    uint32_t spi_error_before; /**< 调用 HAL 前的 HAL ErrorCode。 */
    uint32_t spi_sr_before; /**< 调用 HAL 前的 SPI1 SR 寄存器。 */
    uint32_t spi_state_after; /**< HAL 返回后的 SPI state。 */
    uint32_t spi_error_after; /**< HAL 返回后的 ErrorCode。 */
    uint32_t spi_sr_after; /**< HAL 返回后的 SPI1 SR 寄存器。 */
    int32_t hal_result; /**< 本次 HAL_StatusTypeDef 的整数表示。 */
} bsp_spi_first_access_diagnostics_t;

/* 仅供 BMI323 接线探针使用的一次有界低速事务。first_segment_length 非零时，
 * 在两次 HAL 调用之间持续保持 CS 有效。 */
typedef struct {
    uint8_t cs_before; /**< 拉低前读到的 CS 逻辑电平。 */
    uint8_t cs_active; /**< 事务进行时读到的 CS 电平。 */
    uint8_t cs_after; /**< 释放后读到的 CS 电平。 */
    uint32_t spi_state_before; /**< 事务前 HAL SPI state。 */
    uint32_t spi_error_before; /**< 事务前 HAL ErrorCode。 */
    uint32_t spi_sr_before; /**< 事务前 SPI1 SR。 */
    uint32_t spi_cfg1_before; /**< 事务前 SPI1 CFG1。 */
    uint32_t spi_cfg2_before; /**< 事务前 SPI1 CFG2。 */
    uint32_t spi_cr1_before; /**< 事务前 SPI1 CR1。 */
    uint32_t spi_cr2_before; /**< 事务前 SPI1 CR2。 */
    uint32_t spi_state_after; /**< 恢复后 HAL SPI state。 */
    uint32_t spi_error_after; /**< 恢复后 HAL ErrorCode。 */
    uint32_t spi_sr_after; /**< 恢复后 SPI1 SR。 */
    uint32_t spi_cfg1_after; /**< 恢复后 SPI1 CFG1。 */
    uint32_t spi_cfg2_after; /**< 恢复后 SPI1 CFG2。 */
    uint32_t spi_cr1_after; /**< 恢复后 SPI1 CR1。 */
    uint32_t spi_cr2_after; /**< 恢复后 SPI1 CR2。 */
    int32_t hal_status; /**< 原始事务的 HAL_StatusTypeDef 整数值。 */
    uint32_t hal_error; /**< 原始事务结束时的 HAL ErrorCode。 */
    uint32_t spi_hz; /**< 诊断事务临时 SPI 时钟估算值，单位 Hz。 */
} bsp_spi_bmi323_raw_diagnostics_t;

/**
 * @brief  配置项目私有 SPI1 handle，并将 BMI323 CS 释放为高电平。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（接口契约补充）。
 * 传入参数：无。
 * @return OK；GPIO 初始化或 HAL_SPI_Init 失败返回 ERROR。重复调用在已就绪时返回 OK。
 * 调用方式：系统时钟稳定后、BMI323 驱动访问前调用；本函数会调用 bsp_gpio_init()。
 * 线程约束：只允许启动任务调用；会配置 GPIO/SPI 寄存器，无内部锁，禁止与事务或 ISR 并发。
 */
bsp_status_t bsp_spi_init(void);
/**
 * @brief  使用 SPI1 执行一段阻塞发送。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（接口契约补充）。
 * @param  data 非 NULL 的只读发送缓冲；函数返回后不保留指针。
 * @param  size 发送字节数，范围 1..UINT16_MAX。
 * @param  timeout_ms 传给 HAL_SPI_Transmit() 的单次超时，单位 ms。
 * @return OK、INVALID_ARG、NOT_READY、TIMEOUT 或 ERROR。
 * 调用方式：先成功初始化；本函数不控制 BMI323 CS，调用方必须在事务边界正确拉低/释放 CS。
 * 线程约束：任务上下文阻塞调用，无 mutex；SPI1 所有调用和 CS 操作必须由上层串行化，禁止 ISR 调用。
 */
bsp_status_t bsp_spi_transmit(const uint8_t *data, size_t size, uint32_t timeout_ms);
/**
 * @brief  使用 SPI1 执行一段阻塞接收。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（接口契约补充）。
 * @param[out] data 非 NULL、至少可写 size 字节的输出缓冲。
 * @param  size 接收字节数，范围 1..UINT16_MAX。
 * @param  timeout_ms 传给 HAL_SPI_Receive() 的单次超时，单位 ms。
 * @return OK、INVALID_ARG、NOT_READY、TIMEOUT 或 ERROR；非 OK 时 data 不得作为有效样本。
 * 调用方式：本函数不控制 CS；调用方负责完整片选时序及失败后的 CS 释放。
 * 线程约束：任务上下文阻塞调用，无 mutex；SPI1 必须由上层串行化，禁止 ISR 调用。
 */
bsp_status_t bsp_spi_receive(uint8_t *data, size_t size, uint32_t timeout_ms);
/**
 * @brief  使用 HAL_SPI_TransmitReceive() 执行等长全双工阻塞传输。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（接口契约补充）。
 * @param  tx_data 非 NULL、包含 size 字节的只读发送缓冲。
 * @param[out] rx_data 非 NULL、至少可写 size 字节的接收缓冲；不得与 tx_data 重叠。
 * @param  size 同时发送和接收的字节数，范围 1..UINT16_MAX。
 * @param  timeout_ms 传给单次 HAL_SPI_TransmitReceive() 的超时，单位 ms。
 * @return OK、INVALID_ARG、NOT_READY、TIMEOUT 或 ERROR。
 * 调用方式：本函数不改变 CS；驱动必须在调用前拉低、返回后释放。首次调用还会采集并
 *           通过日志链路输出诊断快照，不能据普通调用时延估算首次调用 WCET。
 * 线程约束：任务上下文阻塞调用，无 mutex；上层必须串行化 SPI1/CS，禁止 ISR 调用。
 */
bsp_status_t bsp_spi_write_read(const uint8_t *tx_data, uint8_t *rx_data,
                                size_t size, uint32_t timeout_ms);

/**
 * @brief  读取最近一次由本 BSP 记录的 HAL SPI 返回值。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（接口契约补充）。
 * 传入参数：无。
 * @return HAL_StatusTypeDef 的整数值；尚未执行事务时可能为 -1。
 * 调用方式：只用于故障日志，业务判断应使用各事务函数返回的 bsp_status_t。
 * 线程约束：无锁快照；与正在执行的 SPI 事务并发读取可能得到前一笔或当前笔状态。
 */
int32_t bsp_spi_get_last_hal_status(void);

/**
 * @brief  复制首次 bsp_spi_write_read() 产生的诊断快照。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（接口契约补充）。
 * @param[out] diagnostics 非 NULL 的输出对象；成功时完整复制静态快照。
 * @return 1 表示快照有效并已复制；NULL 或首次事务尚未完成时返回 0。
 * 调用方式：初始化诊断阶段读取；读取不会清除快照，后续调用仍返回同一首次记录。
 * 线程约束：无锁复制；应在首次事务完成后由单一诊断任务读取，禁止从 ISR 调用。
 */
uint8_t bsp_spi_get_first_access_diagnostics(
    bsp_spi_first_access_diagnostics_t *diagnostics);

/**
 * @brief 执行一次受限 BMI323 原始 SPI 事务并采集寄存器状态。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（接口契约补充）。
 * @param  tx_data 非 NULL、包含 size 字节的诊断发送缓冲。
 * @param[out] rx_data 非 NULL、至少可写 size 字节的诊断接收缓冲。
 * @param  size 总事务长度，范围 1..UINT16_MAX。
 * @param  first_segment_length 0 表示单次 HAL 事务；1..size-1 表示保持 CS 的两段事务。
 * @param  timeout_ms 分别用于每个 HAL 段；两段模式最坏阻塞可能接近两倍该值。
 * @param[out] diagnostics 必须非 NULL；返回前填充 CS、SPI 寄存器、HAL 状态和临时时钟。
 * @return OK、INVALID_ARG、NOT_READY、TIMEOUT 或 ERROR；失败路径仍尽力释放 CS、恢复分频。
 * 调用方式：仅限 BMI323 接线/启动诊断镜像；函数临时改 SPI1 分频并忙等 CS 建立/保持时间，
 *           不得在正常采样、控制周期或生产固件常态路径调用。
 * 线程约束：无 mutex 且直接操作 SPI1 寄存器；必须独占 SPI1 和 PC4，禁止 ISR/并发调用。
 */
bsp_status_t bsp_spi_bmi323_raw_transaction(
    const uint8_t *tx_data, uint8_t *rx_data, size_t size,
    size_t first_segment_length, uint32_t timeout_ms,
    bsp_spi_bmi323_raw_diagnostics_t *diagnostics);

/**
 * @brief  在 SPI1 AF5 配置下直接读取 PA6/MISO 的物理电平。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（接口契约补充）。
 * @param[out] level 非 NULL；成功时写入 HIGH/LOW。
 * @return OK、INVALID_ARG 或 NOT_READY。
 * 调用方式：只用于初始化接线诊断；结果是瞬时电平，不代表 SPI 帧或器件通信有效。
 * 线程约束：同步寄存器读、不阻塞；应避免与正在切换 SPI/CS 的事务并发。
 */
bsp_status_t bsp_spi_read_miso_level(bsp_gpio_level_t *level);

/* 以下短名称仅作兼容包装，完整契约继承对应 bsp_* API。 */
/** @copydoc bsp_spi_init
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（兼容包装契约补充）。 */
static inline bsp_status_t spi_init(void) { return bsp_spi_init(); }
/** @copydoc bsp_spi_transmit
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（兼容包装契约补充）。 */
static inline bsp_status_t spi_transmit(const uint8_t *data, size_t size, uint32_t timeout_ms)
{
    return bsp_spi_transmit(data, size, timeout_ms);
}
/** @copydoc bsp_spi_receive
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（兼容包装契约补充）。 */
static inline bsp_status_t spi_receive(uint8_t *data, size_t size, uint32_t timeout_ms)
{
    return bsp_spi_receive(data, size, timeout_ms);
}
/** @copydoc bsp_spi_write_read
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（兼容包装契约补充）。 */
static inline bsp_status_t spi_write_read(const uint8_t *tx_data, uint8_t *rx_data,
                                          size_t size, uint32_t timeout_ms)
{
    return bsp_spi_write_read(tx_data, rx_data, size, timeout_ms);
}

/** @copydoc bsp_spi_get_last_hal_status
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（兼容包装契约补充）。 */
static inline int32_t spi_get_last_hal_status(void)
{
    return bsp_spi_get_last_hal_status();
}

/** @copydoc bsp_spi_read_miso_level
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（兼容包装契约补充）。 */
static inline bsp_status_t spi_read_miso_level(bsp_gpio_level_t *level)
{
    return bsp_spi_read_miso_level(level);
}

#ifdef __cplusplus
}
#endif

#endif /* BSP_SPI_H */
