#include "imu_time.h"

#include "bsp_timer.h"

bsp_status_t imu_time_init(void)
{
    return bsp_timer_init();
}

uint64_t imu_time_now_us(void)
{
    return bsp_timer_get_us();
}

uint32_t imu_time_now_ms(void)
{
    return (uint32_t)(imu_time_now_us() / UINT64_C(1000));
}
