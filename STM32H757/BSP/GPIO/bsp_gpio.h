#ifndef BSP_GPIO_H
#define BSP_GPIO_H

#include <stdint.h>
#include "bsp_status.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    BSP_GPIO_BMI323_CS = 0,
    BSP_GPIO_BMI323_INT1,
    BSP_GPIO_LF_INT1,
    BSP_GPIO_LF_INT2,
    BSP_GPIO_LB_INT1,
    BSP_GPIO_LB_INT2,
    BSP_GPIO_STM_RX,
    BSP_GPIO_STM_TX,
    BSP_GPIO_AT1_AIN2,
    BSP_GPIO_AT1_BIN2,
    BSP_GPIO_AT2_AIN2,
    BSP_GPIO_AT2_BIN2,
    BSP_GPIO_COUNT
} bsp_gpio_pin_t;

typedef enum {
    BSP_GPIO_LOW = 0,
    BSP_GPIO_HIGH = 1
} bsp_gpio_level_t;

bsp_status_t bsp_gpio_init(void);
bsp_status_t bsp_gpio_write(bsp_gpio_pin_t pin, bsp_gpio_level_t level);
bsp_status_t bsp_gpio_read(bsp_gpio_pin_t pin, bsp_gpio_level_t *level);
bsp_status_t bsp_gpio_toggle(bsp_gpio_pin_t pin);

/* Short names keep the interface usable from small device drivers. */
static inline bsp_status_t gpio_init(void) { return bsp_gpio_init(); }
static inline bsp_status_t gpio_write(bsp_gpio_pin_t pin, bsp_gpio_level_t level)
{
    return bsp_gpio_write(pin, level);
}
static inline bsp_status_t gpio_read(bsp_gpio_pin_t pin, bsp_gpio_level_t *level)
{
    return bsp_gpio_read(pin, level);
}
static inline bsp_status_t gpio_toggle(bsp_gpio_pin_t pin) { return bsp_gpio_toggle(pin); }

#ifdef __cplusplus
}
#endif

#endif /* BSP_GPIO_H */
