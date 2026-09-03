#ifndef IMU_TIME_H
#define IMU_TIME_H

#include <stdint.h>

#include "bsp_status.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * IMU 时间基准接口；创建人：待确认（当前维护人：Zhiqin）。
 * 采样时间始终来自单调 DWT 微秒计数；毫秒接口只是同一计数的兼容视图。
 */
/**
 * @brief  初始化 IMU 使用的 DWT 微秒时间基准。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（接口契约补充）。
 * 传入参数：无。
 * @return 直接返回 bsp_timer_init() 状态；当前实现通常为 BSP_STATUS_OK。
 * 调用方式：IMU 管理器启动采样前调用一次；具体 DWT/IRQ 限制见 bsp_timer_init()。
 * 线程约束：仅启动/任务上下文调用；禁止在高优先级 ISR 中初始化时基。
 */
bsp_status_t imu_time_init(void);
/**
 * @brief  读取 IMU 的 64 位微秒时间戳。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（接口契约补充）。
 * 传入参数：无。
 * @return bsp_timer_get_us() 的结果，单位 us。
 * 调用方式：记录采样完成时间和新鲜度；时间差使用无符号减法，不与 HAL tick 混算。
 * 线程约束：内部会短暂屏蔽 IRQ并承担 DWT 回绕观测限制；高频 ISR 不应调用。
 */
uint64_t imu_time_now_us(void);
/**
 * @brief  将同一 DWT 微秒源除以 1000 后返回 32 位毫秒视图。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（接口契约补充）。
 * 传入参数：无。
 * @return DWT 时间的低 32 位毫秒值，约 2^32 ms 后自然回绕。
 * 调用方式：仅与 imu_time_now_ms() 产生的时间戳比较；不要与 HAL_GetTick() 假定同源。
 * 线程约束：继承 imu_time_now_us() 的临界区和回绕观测限制。
 */
uint32_t imu_time_now_ms(void);

#ifdef __cplusplus
}
#endif

#endif /* IMU_TIME_H */
