#include "bsp_adc.h"

/* ADC BSP 实现；创建人：待确认（当前维护人：Zhiqin）。 */

/** 当前 IOC 未分配 ADC 外设，返回能力不支持。 */
bsp_status_t bsp_adc_init(void)
{
    /* No ADC instance or channel is present in the current CubeMX IOC. */
    return BSP_STATUS_UNSUPPORTED;
}

/** 当前 IOC 无 ADC 通道，保持输出地址原值并返回能力不支持。 */
bsp_status_t bsp_adc_read(bsp_adc_channel_t channel, uint16_t *value)
{
    (void)channel;
    (void)value;
    return BSP_STATUS_UNSUPPORTED;
}
