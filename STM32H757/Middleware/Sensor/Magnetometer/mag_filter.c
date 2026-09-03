#include "mag_filter.h"

/* 磁力计滤波实现；创建人：待确认（当前维护人：Zhiqin）。 */

#if defined(IMU_MANAGER_USE_FREERTOS)
#include "FreeRTOS.h"
#include "task.h"
#endif

/* 新磁场样本的一阶低通权重；历史输出权重为 1-MAG_FILTER_ALPHA。 */
#define MAG_FILTER_ALPHA 0.1f

static mag_filter_data_t mag_filter_output;
static bool mag_filter_initialized;

/**
 * @brief 进入磁场滤波共享状态的任务级临界区；无 RTOS 构建下为空操作。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * 传入参数：无。
 * @return 无返回值；当前实现无失败上报，返回前保持临界区处于已进入状态。
 * 调用方式：由 mag_filter_init()、mag_filter_update() 和 mag_filter_get() 在访问输出及初始化标志前调用。
 * 线程约束：RTOS 构建使用 `taskENTER_CRITICAL()` 提高中断屏蔽级而非获取 mutex，不等待、不阻塞，
 * 但在配对退出前增加受影响中断的延迟；禁止 ISR 调用，无 RTOS 构建不提供并发保护，不涉及指针所有权。
 */
static void mag_filter_lock(void)
{
#if defined(IMU_MANAGER_USE_FREERTOS)
    taskENTER_CRITICAL();
#endif
}

/**
 * @brief 退出磁场滤波共享状态的任务级临界区；无 RTOS 构建下为空操作。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * 传入参数：无。
 * @return 无返回值；当前实现无失败上报，调用后共享状态不再受本临界区保护。
 * 调用方式：由三个公共接口在完成共享状态访问后调用，必须与同一路径的 mag_filter_lock() 严格成对。
 * 线程约束：RTOS 构建使用 `taskEXIT_CRITICAL()` 而非 mutex，退出操作不等待、不阻塞；
 * 禁止 ISR 调用或未配对调用，无 RTOS 构建为空操作，不涉及指针所有权。
 */
static void mag_filter_unlock(void)
{
#if defined(IMU_MANAGER_USE_FREERTOS)
    taskEXIT_CRITICAL();
#endif
}

/** 清空磁力计滤波历史。 */
void mag_filter_init(void)
{
    mag_filter_lock();
    mag_filter_output = (mag_filter_data_t){0};
    mag_filter_initialized = false;
    mag_filter_unlock();
}

/** 输入一条已由调用方验证的 LSM303 磁场快照并更新滤波结果。 */
void mag_filter_update(const lsm_mag_data_t *raw)
{
    if (raw == NULL) {
        return;
    }

    mag_filter_lock();
    if (!mag_filter_initialized) {
        /* Seed with the first sample so the output starts at the sensor value. */
        mag_filter_output.mx = raw->mx;
        mag_filter_output.my = raw->my;
        mag_filter_output.mz = raw->mz;
        mag_filter_initialized = true;
    } else {
        mag_filter_output.mx = (MAG_FILTER_ALPHA * raw->mx) +
                               ((1.0f - MAG_FILTER_ALPHA) * mag_filter_output.mx);
        mag_filter_output.my = (MAG_FILTER_ALPHA * raw->my) +
                               ((1.0f - MAG_FILTER_ALPHA) * mag_filter_output.my);
        mag_filter_output.mz = (MAG_FILTER_ALPHA * raw->mz) +
                               ((1.0f - MAG_FILTER_ALPHA) * mag_filter_output.mz);
    }
    mag_filter_unlock();
}

/** 复制最近一次有效磁场结果。 */
bool mag_filter_get(mag_filter_data_t *out)
{
    bool ready;

    if (out == NULL) {
        return false;
    }

    mag_filter_lock();
    ready = mag_filter_initialized;
    if (ready) {
        *out = mag_filter_output;
    }
    mag_filter_unlock();
    return ready;
}
