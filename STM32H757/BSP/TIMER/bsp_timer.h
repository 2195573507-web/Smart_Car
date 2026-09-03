#ifndef BSP_TIMER_H
#define BSP_TIMER_H

#include <stdint.h>
#include "bsp_status.h"

/* 时间基准 BSP；创建人：待确认（当前维护人：Zhiqin）。
 * 微秒接口基于 DWT CYCCNT 软件扩展；毫秒接口直接返回 HAL_GetTick()，两者并非
 * 同一计数源，跨接口比较或换算前必须明确所需时基。 */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  幂等启用 DWT CYCCNT，并清零本 BSP 的微秒扩展状态。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（接口契约补充）。
 * 传入参数：无。
 * @return 固定返回 BSP_STATUS_OK；当前实现不校验目标内核是否实际支持 DWT。
 * 调用方式：时钟配置稳定后、启动需要微秒时间戳的任务前调用；重复调用不会再次清零。
 * 线程约束：初始化段短暂屏蔽并恢复 IRQ；只允许启动/任务上下文调用，不在高优先级 ISR 调用。
 */
bsp_status_t bsp_timer_init(void);
/**
 * @brief  返回由 DWT 周期数换算的 64 位微秒计数。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（接口契约补充）。
 * 传入参数：无。
 * @return 自本 BSP 初始化以来的微秒数；SystemCoreClock 异常导致 cycles/us 为 0 时返回 0。
 * 调用方式：未显式初始化时会自动调用 bsp_timer_init()；时间差使用无符号减法。
 * 线程约束：为扩展 32 位 CYCCNT 会短暂屏蔽 IRQ；调用必须足够频繁以观测每次 CYCCNT
 *           回绕，否则无法恢复跨过多个回绕周期的真实时间。不要用于长时间无人采样的绝对时钟。
 */
uint64_t bsp_timer_get_us(void);
/**
 * @brief  返回 STM32 HAL 毫秒 tick。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（接口契约补充）。
 * 传入参数：无。
 * @return HAL_GetTick() 的 32 位值，单位 ms，约 2^32 ms 后自然回绕。
 * 调用方式：用于 HAL/RTOS 周期和超时；不要假定它与 bsp_timer_get_us() 同源或严格同步。
 * 线程约束：无阻塞、无内部锁；中断暂停 tick 时不会推进，比较时使用无符号差值。
 */
uint32_t bsp_timer_get_ms(void);

/* 以下短名称仅作兼容包装，完整契约继承对应 bsp_* API。 */
/** @copydoc bsp_timer_init
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（兼容包装契约补充）。 */
static inline bsp_status_t timer_init(void) { return bsp_timer_init(); }
/** @copydoc bsp_timer_get_us
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（兼容包装契约补充）。 */
static inline uint64_t timer_get_us(void) { return bsp_timer_get_us(); }
/** @copydoc bsp_timer_get_ms
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（兼容包装契约补充）。 */
static inline uint32_t timer_get_ms(void) { return bsp_timer_get_ms(); }

#ifdef __cplusplus
}
#endif

#endif /* BSP_TIMER_H */
