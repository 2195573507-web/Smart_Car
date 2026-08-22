#include "bsp_pwm.h"

#include "main.h"

extern TIM_HandleTypeDef htim3;

static uint8_t pwm_ready;

static bsp_status_t pwm_validate(bsp_pwm_channel_t channel)
{
    /* PC6/PC7 are permanently owned by USART6 in the motor-board image.
     * Keep the legacy enum values source-compatible, but never allow TIM3
     * CH1/CH2 to be started or updated through this BSP. */
    return (channel == BSP_PWM_CHANNEL_3 || channel == BSP_PWM_CHANNEL_4)
               ? BSP_STATUS_OK : BSP_STATUS_INVALID_ARG;
}

static uint32_t pwm_hal_channel(bsp_pwm_channel_t channel)
{
    return channel == BSP_PWM_CHANNEL_3 ? TIM_CHANNEL_3 : TIM_CHANNEL_4;
}

bsp_status_t bsp_pwm_init(void)
{
    if (htim3.Instance != TIM3 || HAL_TIM_PWM_GetState(&htim3) == HAL_TIM_STATE_RESET) {
        return BSP_STATUS_NOT_READY;
    }
    pwm_ready = 1U;
    return BSP_STATUS_OK;
}

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
