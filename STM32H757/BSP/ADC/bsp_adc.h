#ifndef BSP_ADC_H
#define BSP_ADC_H

#include <stdint.h>
#include "bsp_status.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef uint32_t bsp_adc_channel_t;

bsp_status_t bsp_adc_init(void);
bsp_status_t bsp_adc_read(bsp_adc_channel_t channel, uint16_t *value);

static inline bsp_status_t adc_init(void) { return bsp_adc_init(); }
static inline bsp_status_t adc_read(bsp_adc_channel_t channel, uint16_t *value)
{
    return bsp_adc_read(channel, value);
}

#ifdef __cplusplus
}
#endif

#endif /* BSP_ADC_H */
