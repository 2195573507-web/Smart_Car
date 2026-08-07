#ifndef BSP_PWM_H
#define BSP_PWM_H

#include <stdint.h>
#include "bsp_status.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    BSP_PWM_CHANNEL_1 = 0,
    BSP_PWM_CHANNEL_2,
    BSP_PWM_CHANNEL_3,
    BSP_PWM_CHANNEL_4,
    BSP_PWM_CHANNEL_COUNT
} bsp_pwm_channel_t;

bsp_status_t bsp_pwm_init(void);
bsp_status_t bsp_pwm_start(bsp_pwm_channel_t channel);
bsp_status_t bsp_pwm_stop(bsp_pwm_channel_t channel);
bsp_status_t bsp_pwm_set_duty(bsp_pwm_channel_t channel, uint8_t duty_percent);

static inline bsp_status_t pwm_init(void) { return bsp_pwm_init(); }
static inline bsp_status_t pwm_start(bsp_pwm_channel_t channel) { return bsp_pwm_start(channel); }
static inline bsp_status_t pwm_stop(bsp_pwm_channel_t channel) { return bsp_pwm_stop(channel); }
static inline bsp_status_t pwm_set_duty(bsp_pwm_channel_t channel, uint8_t duty_percent)
{
    return bsp_pwm_set_duty(channel, duty_percent);
}

#ifdef __cplusplus
}
#endif

#endif /* BSP_PWM_H */
