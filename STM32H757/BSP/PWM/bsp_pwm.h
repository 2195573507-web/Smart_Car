#ifndef BSP_PWM_H
#define BSP_PWM_H

#include <stdint.h>
#include "bsp_status.h"

/* STM32 TIM3 PWM BSP；创建人：待确认（当前维护人：Zhiqin）。
 * CH1/CH2 对应 PC6/PC7，永久归 USART6 MotorBoard 链路；枚举为源码兼容保留，
 * 所有 start/stop/set_duty 调用都会拒绝这两个通道。当前仅 CH3/CH4 可用。 */

#ifdef __cplusplus
extern "C" {
#endif

/** TIM3 PWM 逻辑通道；当前仅 CH3/CH4 可由本 BSP 使用。 */
typedef enum {
    BSP_PWM_CHANNEL_1 = 0, /**< TIM3 CH1，PC6 已归 USART6，调用会拒绝。 */
    BSP_PWM_CHANNEL_2, /**< TIM3 CH2，PC7 已归 USART6，调用会拒绝。 */
    BSP_PWM_CHANNEL_3, /**< TIM3 CH3，可用 PWM 通道。 */
    BSP_PWM_CHANNEL_4, /**< TIM3 CH4，可用 PWM 通道。 */
    BSP_PWM_CHANNEL_COUNT /**< 通道数量哨兵，不是有效通道。 */
} bsp_pwm_channel_t;

/**
 * @brief  校验 CubeMX 已初始化的 TIM3 PWM handle，并开放 CH3/CH4 接口。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（接口契约补充）。
 * 传入参数：无。
 * @return BSP_STATUS_OK；TIM3 实例错误或 HAL 状态为 RESET 时返回 BSP_STATUS_NOT_READY。
 * 调用方式：在 MX_TIM3_Init() 成功后调用；本函数不启动通道，也不改占空比。
 * 线程约束：仅启动任务调用；无内部锁，禁止与 TIM3 HAL 操作或 ISR 并发。
 */
bsp_status_t bsp_pwm_init(void);
/**
 * @brief  启动 TIM3 的一个允许通道。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（接口契约补充）。
 * @param  channel 仅允许 BSP_PWM_CHANNEL_3 或 BSP_PWM_CHANNEL_4。
 * @return OK、INVALID_ARG、NOT_READY 或 HAL 启动失败映射的 ERROR。
 * 调用方式：先成功调用 bsp_pwm_init() 并设置所需占空比；业务层仍负责输出安全门。
 * 线程约束：任务上下文同步 HAL 操作，无内部锁；禁止从 ISR 调用或与同通道并发操作。
 */
bsp_status_t bsp_pwm_start(bsp_pwm_channel_t channel);
/**
 * @brief  停止 TIM3 的一个允许通道。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（接口契约补充）。
 * @param  channel 仅允许 BSP_PWM_CHANNEL_3 或 BSP_PWM_CHANNEL_4。
 * @return OK、INVALID_ARG、NOT_READY 或 HAL 停止失败映射的 ERROR。
 * 调用方式：故障/停机路径由上层调用；当前函数只执行 HAL_TIM_PWM_Stop()，不会清零
 *           CCR，也不能单独证明外部驱动已进入安全电平。
 * 线程约束：任务上下文同步 HAL 操作，无内部锁；禁止从 ISR 调用或与同通道并发操作。
 */
bsp_status_t bsp_pwm_stop(bsp_pwm_channel_t channel);
/**
 * @brief  设置允许通道的 PWM 比较值。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（接口契约补充）。
 * @param  channel 仅允许 BSP_PWM_CHANNEL_3 或 BSP_PWM_CHANNEL_4。
 * @param  duty_percent 占空比百分数，范围 0..100。
 * @return BSP_STATUS_OK；参数越界返回 INVALID_ARG，未初始化返回 NOT_READY。
 * 调用方式：可在 start 前预装比较值或运行期更新；函数不检查 chassis/MotorBoard 安全门。
 * 线程约束：直接更新 CCR、无内部锁；同一通道只能由单一控制所有者更新，禁止 ISR/任务并发。
 */
bsp_status_t bsp_pwm_set_duty(bsp_pwm_channel_t channel, uint8_t duty_percent);

/* 以下短名称仅作兼容包装，完整契约继承对应 bsp_* API。 */
/** @copydoc bsp_pwm_init
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（兼容包装契约补充）。 */
static inline bsp_status_t pwm_init(void) { return bsp_pwm_init(); }
/** @copydoc bsp_pwm_start
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（兼容包装契约补充）。 */
static inline bsp_status_t pwm_start(bsp_pwm_channel_t channel) { return bsp_pwm_start(channel); }
/** @copydoc bsp_pwm_stop
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（兼容包装契约补充）。 */
static inline bsp_status_t pwm_stop(bsp_pwm_channel_t channel) { return bsp_pwm_stop(channel); }
/** @copydoc bsp_pwm_set_duty
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（兼容包装契约补充）。 */
static inline bsp_status_t pwm_set_duty(bsp_pwm_channel_t channel, uint8_t duty_percent)
{
    return bsp_pwm_set_duty(channel, duty_percent);
}

#ifdef __cplusplus
}
#endif

#endif /* BSP_PWM_H */
