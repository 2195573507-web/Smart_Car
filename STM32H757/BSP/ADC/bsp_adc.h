#ifndef BSP_ADC_H
#define BSP_ADC_H

#include <stdint.h>
#include "bsp_status.h"

/* ADC BSP；创建人：待确认（当前维护人：Zhiqin）。 */

#ifdef __cplusplus
extern "C" {
#endif

typedef uint32_t bsp_adc_channel_t;

/**
 * @brief  查询并初始化当前板级 ADC 能力。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（接口契约补充）。
 * 传入参数：无。
 * @return 当前 IOC 未配置 ADC，固定返回 BSP_STATUS_UNSUPPORTED。
 * 调用方式：板级启动阶段可调用以探测能力；返回不支持后不得继续读取 ADC。
 * 线程约束：不分配资源、不阻塞；当前实现不访问外设，也不应从 ISR 依赖该接口。
 */
bsp_status_t bsp_adc_init(void);
/**
 * @brief  读取指定 ADC 通道的原始转换值。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（接口契约补充）。
 * @param  channel 板级通道标识；当前实现不支持任何通道。
 * @param  value 原始值输出地址；当前实现不会写入该地址。
 * @return 当前 IOC 未配置 ADC，固定返回 BSP_STATUS_UNSUPPORTED，包括 value 为 NULL 时。
 * 调用方式：仅在 bsp_adc_init() 返回 BSP_STATUS_OK 后调用；当前工程不会满足此前置条件。
 * 线程约束：不阻塞；返回不支持时必须保留原输出值，不得把 value 当作新样本。
 */
bsp_status_t bsp_adc_read(bsp_adc_channel_t channel, uint16_t *value);

/* 短名称仅作源码兼容包装，不增加状态；参数、返回值和上下文约束继承对应 bsp_* API。 */
/** @copydoc bsp_adc_init
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（兼容包装契约补充）。 */
static inline bsp_status_t adc_init(void) { return bsp_adc_init(); }
/** @copydoc bsp_adc_read
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（兼容包装契约补充）。 */
static inline bsp_status_t adc_read(bsp_adc_channel_t channel, uint16_t *value)
{
    return bsp_adc_read(channel, value);
}

#ifdef __cplusplus
}
#endif

#endif /* BSP_ADC_H */
