#include "imu_time.h"

/* IMU 单调时间基准实现；创建人：待确认（当前维护人：Zhiqin）。 */

#include "bsp_timer.h"

/** 初始化 DWT 周期和时间换算参数。 */
bsp_status_t imu_time_init(void)
{
    return bsp_timer_init();
}

/** 读取单调微秒计数。 */
uint64_t imu_time_now_us(void)
{
    return bsp_timer_get_us();
}

/** 读取由同一微秒源换算的毫秒计数。 */
uint32_t imu_time_now_ms(void)
{
    return (uint32_t)(imu_time_now_us() / UINT64_C(1000));
}
