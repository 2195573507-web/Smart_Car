#include "bsp_adc.h"

bsp_status_t bsp_adc_init(void)
{
    /* No ADC instance or channel is present in the current CubeMX IOC. */
    return BSP_STATUS_UNSUPPORTED;
}

bsp_status_t bsp_adc_read(bsp_adc_channel_t channel, uint16_t *value)
{
    (void)channel;
    (void)value;
    return BSP_STATUS_UNSUPPORTED;
}
