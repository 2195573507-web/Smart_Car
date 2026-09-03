#include "bsp_pwm.h"

/* PWM BSP 实现；创建人：待确认（当前维护人：Zhiqin）。 */

#include "main.h"

extern TIM_HandleTypeDef htim3;

static uint8_t pwm_ready;

/**
 * @brief 校验 PWM 通道是否属于当前允许使用的 TIM3 CH3/CH4。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param channel 待校验的 BSP PWM 通道；CH1/CH2 因 PC6/PC7 归 USART6 所有而被拒绝。
 * @return CH3/CH4 返回 BSP_STATUS_OK，其余枚举或越界值返回 BSP_STATUS_INVALID_ARG。
 * 调用方式：本文件 start、stop、set_duty 入口必须先调用，成功后才可映射 HAL 通道。
 * 线程约束：纯值判断，不阻塞、不使用 mutex；函数自身可在 ISR 调用栈执行，但不会转移 TIM3/USART6 引脚所有权。
 */
static bsp_status_t pwm_validate(bsp_pwm_channel_t channel)
{
    /* PC6/PC7 are permanently owned by USART6 in the motor-board image.
     * Keep the legacy enum values source-compatible, but never allow TIM3
     * CH1/CH2 to be started or updated through this BSP. */
    return (channel == BSP_PWM_CHANNEL_3 || channel == BSP_PWM_CHANNEL_4)
               ? BSP_STATUS_OK : BSP_STATUS_INVALID_ARG;
}

/**
 * @brief 将已经校验通过的 BSP PWM 通道转换为 HAL TIM 通道常量。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param channel 必须是已由 pwm_validate() 接受的 CH3 或 CH4。
 * @return CH3 返回 TIM_CHANNEL_3；其他值均返回 TIM_CHANNEL_4，因此调用方必须先完成校验。
 * 调用方式：仅由本文件在 pwm_validate() 成功后、访问 TIM3 HAL/CCR 前调用。
 * 线程约束：纯值转换，不阻塞、不使用 mutex；函数自身可在 ISR 调用栈执行，TIM3 通道所有权仍由外层 BSP 调用者串行管理。
 */
static uint32_t pwm_hal_channel(bsp_pwm_channel_t channel)
{
    return channel == BSP_PWM_CHANNEL_3 ? TIM_CHANNEL_3 : TIM_CHANNEL_4;
}

/** 初始化 PWM 定时器但不启动输出。 */
bsp_status_t bsp_pwm_init(void)
{
    if (htim3.Instance != TIM3 || HAL_TIM_PWM_GetState(&htim3) == HAL_TIM_STATE_RESET) {
        return BSP_STATUS_NOT_READY;
    }
    pwm_ready = 1U;
    return BSP_STATUS_OK;
}

/** 启动指定 PWM 通道。 */
bsp_status_t bsp_pwm_start(bsp_pwm_channel_t channel)
{
    if (pwm_validate(channel) != BSP_STATUS_OK) {
        return BSP_STATUS_INVALID_ARG;
    }
    if (pwm_ready == 0U) {
        return BSP_STATUS_NOT_READY;
    }
    return HAL_TIM_PWM_Start(&htim3, pwm_hal_channel(channel)) == HAL_OK
               ? BSP_STATUS_OK : BSP_STATUS_ERROR;
}

/** 停止指定 PWM 通道；比较寄存器保持原值，由上层另行完成安全门控。 */
bsp_status_t bsp_pwm_stop(bsp_pwm_channel_t channel)
{
    if (pwm_validate(channel) != BSP_STATUS_OK) {
        return BSP_STATUS_INVALID_ARG;
    }
    if (pwm_ready == 0U) {
        return BSP_STATUS_NOT_READY;
    }
    return HAL_TIM_PWM_Stop(&htim3, pwm_hal_channel(channel)) == HAL_OK
               ? BSP_STATUS_OK : BSP_STATUS_ERROR;
}

/** 设置指定通道百分比占空比。 */
bsp_status_t bsp_pwm_set_duty(bsp_pwm_channel_t channel, uint8_t duty_percent)
{
    uint32_t period;
    if (pwm_validate(channel) != BSP_STATUS_OK || duty_percent > 100U) {
        return BSP_STATUS_INVALID_ARG;
    }
    if (pwm_ready == 0U) {
        return BSP_STATUS_NOT_READY;
    }
    period = __HAL_TIM_GET_AUTORELOAD(&htim3);
    __HAL_TIM_SET_COMPARE(&htim3, pwm_hal_channel(channel),
                          ((period + 1U) * duty_percent) / 100U);
    return BSP_STATUS_OK;
}
