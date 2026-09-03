#include "imu_filter.h"

/* IMU 中值/低通滤波实现；创建人：待确认（当前维护人：Zhiqin）。 */

#include <stddef.h>
#include <string.h>

#if defined(IMU_MANAGER_USE_FREERTOS)
#include "FreeRTOS.h"
#include "semphr.h"
#endif

static imu_filter_output_t imu_filter_output;
static uint8_t imu_filter_initialized;
static uint8_t imu_filter_window_count;
static uint8_t imu_filter_window_index;
static float imu_filter_accel_window[3][IMU_FILTER_MEDIAN_WINDOW];
static float imu_filter_mag_window[3][IMU_FILTER_MEDIAN_WINDOW];

#if defined(IMU_MANAGER_USE_FREERTOS)
static SemaphoreHandle_t imu_filter_mutex;
#endif

/**
 * @brief 获取滤波状态互斥量；无 RTOS 构建下为空操作。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * 传入参数：无。
 * @return 无返回值；RTOS mutex 尚未创建或非 RTOS 构建时不提供互斥，`xSemaphoreTake()`
 *         结果被忽略且失败不向调用方报告。
 * 调用方式：本文件 init/update/getter 在访问全部滤波共享状态前调用，并与 unlock 成对。
 * 线程约束：RTOS 下可能以 `portMAX_DELAY` 无限阻塞获取 mutex，严禁 ISR 调用；函数不创建、
 *           删除或转移 mutex 所有权，成功返回后当前任务拥有 mutex 直至 unlock。
 */
static void imu_filter_lock(void)
{
#if defined(IMU_MANAGER_USE_FREERTOS)
    if (imu_filter_mutex != NULL) {
        (void)xSemaphoreTake(imu_filter_mutex, portMAX_DELAY);
    }
#endif
}

/**
 * @brief 释放滤波状态互斥量；无 RTOS 构建下为空操作。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * 传入参数：无。
 * @return 无返回值；mutex 不存在时为空操作，`xSemaphoreGive()` 失败不会上报。
 * 调用方式：仅在持有滤波 mutex 的 init/update/getter 尾部与 `imu_filter_lock()` 成对调用。
 * 线程约束：不主动阻塞但 FreeRTOS mutex 只能由持有任务释放，严禁 ISR/未持锁调用；函数
 *           不删除或转移 mutex 对象所有权，返回后当前任务不再拥有该锁。
 */
static void imu_filter_unlock(void)
{
#if defined(IMU_MANAGER_USE_FREERTOS)
    if (imu_filter_mutex != NULL) {
        (void)xSemaphoreGive(imu_filter_mutex);
    }
#endif
}

/**
 * @brief 对有界小数组执行插入排序并返回中值。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] values 调用期间只读且至少包含 `count` 个 float 的数组；不可为 NULL。
 * @param[in] count 参与计算的元素数；内部调用保证范围为 1..`IMU_FILTER_MEDIAN_WINDOW`。
 * @return 返回排序副本下标 `count/2` 的值；NULL、0 或超窗 count 没有防御性输出，NaN
 *         会按浮点比较规则影响排序且无错误标志。
 * 调用方式：仅 `imu_filter_update()` 在持有滤波 mutex 时对六个轴窗口依次调用。
 * 线程约束：使用固定栈数组、纯计算且不主动阻塞；依赖调用方已持锁保护源窗口，禁止 ISR，
 *           不保存输入指针或接管数组所有权。
 */
static float imu_filter_median(const float *values, uint8_t count)
{
    float sorted[IMU_FILTER_MEDIAN_WINDOW];
    uint8_t i;
    uint8_t j;

    for (i = 0U; i < count; ++i) {
        sorted[i] = values[i];
    }
    for (i = 1U; i < count; ++i) {
        const float value = sorted[i];
        j = i;
        while (j > 0U && sorted[j - 1U] > value) {
            sorted[j] = sorted[j - 1U];
            --j;
        }
        sorted[j] = value;
    }
    return sorted[count / 2U];
}

/** 清空滤波历史并置未就绪。 */
void imu_filter_init(void)
{
#if defined(IMU_MANAGER_USE_FREERTOS)
    if (imu_filter_mutex == NULL) {
        imu_filter_mutex = xSemaphoreCreateMutex();
    }
#endif
    imu_filter_lock();
    imu_filter_output = (imu_filter_output_t){0};
    imu_filter_initialized = 0U;
    imu_filter_window_count = 0U;
    imu_filter_window_index = 0U;
    (void)memset(imu_filter_accel_window, 0, sizeof(imu_filter_accel_window));
    (void)memset(imu_filter_mag_window, 0, sizeof(imu_filter_mag_window));
    imu_filter_unlock();
}

/** 输入已由调用方通过有效性门的标定快照，并更新滤波输出。 */
void imu_filter_update(const imu_calibrated_data_t *calibrated_data)
{
    if (calibrated_data == NULL) {
        return;
    }

    imu_filter_lock();
    if (imu_filter_initialized == 0U) {
        uint8_t index;

        /* Prime all five slots so the first median is deterministic. */
        for (index = 0U; index < IMU_FILTER_MEDIAN_WINDOW; ++index) {
            imu_filter_accel_window[0][index] = calibrated_data->ax;
            imu_filter_accel_window[1][index] = calibrated_data->ay;
            imu_filter_accel_window[2][index] = calibrated_data->az;
            imu_filter_mag_window[0][index] = calibrated_data->mx;
            imu_filter_mag_window[1][index] = calibrated_data->my;
            imu_filter_mag_window[2][index] = calibrated_data->mz;
        }
        imu_filter_window_count = IMU_FILTER_MEDIAN_WINDOW;
        imu_filter_window_index = 0U;
        imu_filter_output.ax = calibrated_data->ax;
        imu_filter_output.ay = calibrated_data->ay;
        imu_filter_output.az = calibrated_data->az;
        imu_filter_output.mx = calibrated_data->mx;
        imu_filter_output.my = calibrated_data->my;
        imu_filter_output.mz = calibrated_data->mz;
        imu_filter_initialized = 1U;
    } else {
        const uint8_t index = imu_filter_window_index;
        float median_ax;
        float median_ay;
        float median_az;
        float median_mx;
        float median_my;
        float median_mz;

        imu_filter_accel_window[0][index] = calibrated_data->ax;
        imu_filter_accel_window[1][index] = calibrated_data->ay;
        imu_filter_accel_window[2][index] = calibrated_data->az;
        imu_filter_mag_window[0][index] = calibrated_data->mx;
        imu_filter_mag_window[1][index] = calibrated_data->my;
        imu_filter_mag_window[2][index] = calibrated_data->mz;
        imu_filter_window_index =
            (uint8_t)((index + 1U) % IMU_FILTER_MEDIAN_WINDOW);
        if (imu_filter_window_count < IMU_FILTER_MEDIAN_WINDOW) {
            ++imu_filter_window_count;
        }
        median_ax = imu_filter_median(imu_filter_accel_window[0],
                                      imu_filter_window_count);
        median_ay = imu_filter_median(imu_filter_accel_window[1],
                                      imu_filter_window_count);
        median_az = imu_filter_median(imu_filter_accel_window[2],
                                      imu_filter_window_count);
        median_mx = imu_filter_median(imu_filter_mag_window[0],
                                      imu_filter_window_count);
        median_my = imu_filter_median(imu_filter_mag_window[1],
                                      imu_filter_window_count);
        median_mz = imu_filter_median(imu_filter_mag_window[2],
                                      imu_filter_window_count);
        imu_filter_output.ax = (IMU_FILTER_ALPHA * imu_filter_output.ax) +
                               ((1.0f - IMU_FILTER_ALPHA) * median_ax);
        imu_filter_output.ay = (IMU_FILTER_ALPHA * imu_filter_output.ay) +
                               ((1.0f - IMU_FILTER_ALPHA) * median_ay);
        imu_filter_output.az = (IMU_FILTER_ALPHA * imu_filter_output.az) +
                               ((1.0f - IMU_FILTER_ALPHA) * median_az);
        imu_filter_output.mx = (IMU_FILTER_ALPHA * imu_filter_output.mx) +
                               ((1.0f - IMU_FILTER_ALPHA) * median_mx);
        imu_filter_output.my = (IMU_FILTER_ALPHA * imu_filter_output.my) +
                               ((1.0f - IMU_FILTER_ALPHA) * median_my);
        imu_filter_output.mz = (IMU_FILTER_ALPHA * imu_filter_output.mz) +
                               ((1.0f - IMU_FILTER_ALPHA) * median_mz);
    }
    imu_filter_output.timestamp = calibrated_data->timestamp;
    imu_filter_output.online = calibrated_data->online;
    imu_filter_unlock();
}

/** 按值返回最新滤波快照。 */
imu_filtered_data_t imu_filter_get_output(void)
{
    imu_filtered_data_t output;

    imu_filter_lock();
    output = imu_filter_output;
    imu_filter_unlock();
    return output;
}

/** 查询是否已接受至少一个非 NULL 样本，不代表当前 online。 */
uint8_t imu_filter_is_ready(void)
{
    uint8_t ready;

    imu_filter_lock();
    ready = imu_filter_initialized;
    imu_filter_unlock();
    return ready;
}
