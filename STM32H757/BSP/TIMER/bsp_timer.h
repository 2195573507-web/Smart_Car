#ifndef BSP_TIMER_H
#define BSP_TIMER_H

#include <stdint.h>
#include "bsp_status.h"

#ifdef __cplusplus
extern "C" {
#endif

bsp_status_t bsp_timer_init(void);
uint64_t bsp_timer_get_us(void);
uint32_t bsp_timer_get_ms(void);

static inline bsp_status_t timer_init(void) { return bsp_timer_init(); }
static inline uint64_t timer_get_us(void) { return bsp_timer_get_us(); }
static inline uint32_t timer_get_ms(void) { return bsp_timer_get_ms(); }

#ifdef __cplusplus
}
#endif

#endif /* BSP_TIMER_H */
