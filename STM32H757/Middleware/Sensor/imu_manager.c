#include "imu_manager.h"

#include "imu_time.h"
#include "attitude.h"
#include "dual_ahrs.h"
#include "boot_log.h"
#include "lsm303.h"
#include "BMI323/bmi323.h"
#include "imu_calibration.h"
#include "imu_boot_manager.h"
#include "imu_filter.h"
#include "mag_filter.h"
#include "log_service.h"
#include "smartcar_debug_config.h"

/* 双 IMU 采集管理器实现；创建人：待确认（当前维护人：Zhiqin）。 */

#include <stdio.h>
#include <string.h>

#if defined(IMU_MANAGER_USE_FREERTOS)
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#endif

#define IMU_TASK_PERIOD_MS       UINT32_C(10)
#define IMU_RECOVERY_PERIOD_MS   UINT32_C(1000)
#define IMU_LSM303_ONLINE_TIMEOUT_MS UINT32_C(1000)
#define IMU_LSM303_FAIL_LIMIT         UINT8_C(10)
#define IMU_LSM303_HEALTH_PERIOD_MS   UINT32_C(1000)
#if defined(IMU_MANAGER_USE_FREERTOS)
#define BMI323_TASK_STACK_WORDS       UINT16_C(384)
#define BMI323_TASK_PRIORITY          (tskIDLE_PRIORITY + 1U)
#define DUAL_IMU_INIT_TASK_STACK_WORDS UINT16_C(384)
#define DUAL_IMU_INIT_TASK_PRIORITY    (tskIDLE_PRIORITY + 1U)
#endif

/* BMI323 samples remain manager-local and do not enter the LSM303 AHRS path. */
static bmi323_data_t bmi_data;
static bmi323_ring_buffer_t bmi_ring_buffer;
static bmi323_capture_stat_t bmi_capture_stats;
static volatile uint32_t bmi_capture_contention_drop_count;
static lsm_accel_data_t lsm_accel_data;
static lsm_mag_data_t lsm_mag_data;
static imu_raw_data_t imu_snapshot;
static imu_sensor_stats_t bmi_stats;
static imu_sensor_stats_t lsm_stats;
static volatile uint8_t imu_initialized;
static volatile uint8_t imu_prepared;
static volatile uint8_t bmi323_acquisition_enabled;
static uint8_t lsm303_init_success;
static uint8_t bmi323_init_success;
static imu_dual_init_status_t dual_init_status;
static uint32_t last_accel_success_tick;
static uint32_t last_mag_success_tick;
static uint64_t last_filter_accel_timestamp_us;
static uint8_t filter_lifecycle_active;
static uint8_t accel_fail_count;
static uint8_t mag_fail_count;
static uint8_t lsm_accel_valid;
static uint8_t lsm_mag_valid;
static uint8_t dual_ahrs_bias_injected;
static bmi323_diag_status_t bmi323_last_logged_error;
static imu_leveling_state_t g_leveling_bmi;
static imu_leveling_state_t g_leveling_lsm;
#if defined(IMU_MANAGER_USE_FREERTOS)
static TaskHandle_t bmi323_task_handle;
static uint8_t bmi323_task_started;
static TaskHandle_t dual_lsm_init_task_handle;
static TaskHandle_t dual_bmi_init_task_handle;
#endif

typedef struct
{
    uint8_t init;
    uint32_t accel_age;
    uint32_t mag_age;
    uint8_t accel_fail;
    uint8_t mag_fail;
    uint8_t online;
} imu_lsm_health_t;

/**
 * @brief 判断 LSM303 样本的单调微秒时间戳是否仍在在线时限内。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] timestamp_us 样本捕获时间，单位 us；0 表示尚无有效样本。
 * @param[in] now_us 与样本同一 64 位单调时钟域的当前时间。
 * @return 时间戳非 0、不晚于当前时间且年龄小于 1000 ms 时返回 1，否则返回 0。
 * 调用方式：统一快照发布与双 AHRS 输入组装在标记 LSM 分路有效性时调用。
 * 线程约束：纯计算、可重入、不阻塞，可在 ISR 调用；调用方负责在锁内或快照上获得时间戳。
 */
static uint8_t imu_lsm_timestamp_is_fresh(uint64_t timestamp_us,
                                          uint64_t now_us)
{
    const uint64_t timeout_us =
        (uint64_t)IMU_LSM303_ONLINE_TIMEOUT_MS * UINT64_C(1000);

    if (timestamp_us == 0U || now_us < timestamp_us ||
        (now_us - timestamp_us) >= timeout_us) {
        return 0U;
    }
    return 1U;
}

/**
 * @brief 将非空 IMU 初始化/运行诊断文本以 INFO 级别交给日志服务。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] text 仅在本次调用期间借用的 NUL 结尾文本；`NULL` 时静默忽略。
 * @return 无返回值；日志裁剪、队列满或输出失败不向上传递。
 * 调用方式：由初始化 worker、健康日志与运行任务同步调用。
 * 线程约束：仅限任务上下文，不可在 ISR 调用；实时性与阻塞性取决于日志后端。
 */
static void imu_init_log(const char *text)
{
    if (text != NULL) {
        LOG_INFO(text);
    }
}

/**
 * @brief 将初始化阶段名与 BSP 返回码格式化为一条 INFO 日志。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] stage 仅在调用期间借用的 NUL 结尾阶段名；`NULL` 时静默返回。
 * @param[in] status 待输出的 `bsp_status_t` 状态码。
 * @return 无返回值；超过 47 个可用字符的内容被截断，日志失败不上报。
 * 调用方式：LSM303 初始化 worker 完成驱动调用后记录结果。
 * 线程约束：仅限任务上下文；使用栈上 48 字节缓冲和 `snprintf()`，并可在日志后端阻塞，不可在 ISR 调用。
 */
static void imu_init_log_status(const char *stage, bsp_status_t status)
{
    char line[48];

    if (stage == NULL) {
        return;
    }
    (void)snprintf(line, sizeof(line), "%s ret=%d\r\n", stage, (int)status);
    imu_init_log(line);
}

#if defined(IMU_MANAGER_USE_FREERTOS)
static SemaphoreHandle_t bmi_data_mutex;
static SemaphoreHandle_t bmi_driver_mutex;
static SemaphoreHandle_t lsm_data_mutex;
static SemaphoreHandle_t snapshot_mutex;
static SemaphoreHandle_t dual_init_mutex;
#endif

#if !defined(IMU_MANAGER_USE_FREERTOS)
/**
 * @brief 在非 FreeRTOS 构建中使用 IMU 单调时钟忙等指定毫秒。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] delay_ms 需要忙等的毫秒数，0 时立即返回。
 * @return 无返回值；时钟不前进时函数不会返回。
 * 调用方式：非 RTOS `imu_task()` 永久循环每次迭代结束后调用。
 * 线程约束：同步忙等并阻塞调用方、持续占用 CPU，不可从 ISR 或高实时性路径调用；支持 32 位毫秒回绕。
 */
static void imu_delay_ms(uint32_t delay_ms)
{
    const uint32_t start = imu_time_now_ms();
    while ((uint32_t)(imu_time_now_ms() - start) < delay_ms) {
        /* Non-RTOS fallback for the standalone task entrypoint. */
    }
}
#endif

/**
 * @brief 在 FreeRTOS 下按需创建 BMI 数据/驱动、LSM 数据、统一快照和双初始化五个互斥量。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * 传入参数：无。
 * @return 所需锁均可用返回 `BSP_STATUS_OK`；任一创建失败返回 `BSP_STATUS_ERROR`，
 * 之前已创建的锁保留供后续重试，不回滚或释放。
 * 调用方式：生命周期准备、调试初始化和设置 BMI323 ODR 前调用。
 * 线程约束：必须在任务上下文串行执行，不可在 ISR 调用；函数不等待现有锁，但会分配 FreeRTOS 堆，
 * 且无自身锁，并发首次调用可重复分配内核对象。
 */
static bsp_status_t imu_create_data_locks(void)
{
#if defined(IMU_MANAGER_USE_FREERTOS)
    if (bmi_data_mutex == NULL) {
        bmi_data_mutex = xSemaphoreCreateMutex();
        if (bmi_data_mutex == NULL) {
            return BSP_STATUS_ERROR;
        }
    }
    if (bmi_driver_mutex == NULL) {
        bmi_driver_mutex = xSemaphoreCreateMutex();
        if (bmi_driver_mutex == NULL) {
            return BSP_STATUS_ERROR;
        }
    }
    if (lsm_data_mutex == NULL) {
        lsm_data_mutex = xSemaphoreCreateMutex();
        if (lsm_data_mutex == NULL) {
            return BSP_STATUS_ERROR;
        }
    }
    if (snapshot_mutex == NULL) {
        snapshot_mutex = xSemaphoreCreateMutex();
        if (snapshot_mutex == NULL) {
            return BSP_STATUS_ERROR;
        }
    }
    if (dual_init_mutex == NULL) {
        dual_init_mutex = xSemaphoreCreateMutex();
        if (dual_init_mutex == NULL) {
            return BSP_STATUS_ERROR;
        }
    }
#endif
    return BSP_STATUS_OK;
}

/**
 * @brief 永久等待并获取 BMI323 数据、环形缓冲和捕获统计互斥量。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * 传入参数：无。
 * @return 无返回值；非 FreeRTOS 构建时为空操作。
 * 调用方式：读写 `bmi_data`、`bmi_ring_buffer`、`bmi_capture_stats` 或 `bmi_stats` 前调用，并与 `imu_unlock_bmi()` 成对。
 * 线程约束：调用前必须已成功创建 `bmi_data_mutex`；FreeRTOS 下可按 `portMAX_DELAY` 永久阻塞，
 * 不可在 ISR 调用或在同一任务重入获取。
 */
static void imu_lock_bmi(void)
{
#if defined(IMU_MANAGER_USE_FREERTOS)
    (void)xSemaphoreTake(bmi_data_mutex, portMAX_DELAY);
#endif
}

/**
 * @brief 释放由当前任务持有的 BMI323 数据互斥量。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * 传入参数：无。
 * @return 无返回值；非 FreeRTOS 构建时为空操作。
 * 调用方式：仅在成功获取 BMI 数据锁的路径末尾成对调用。
 * 线程约束：仅限当前持锁任务，不可在 ISR 调用；释放路径不等待、正常情况下不阻塞，函数不检查锁句柄或所有权。
 */
static void imu_unlock_bmi(void)
{
#if defined(IMU_MANAGER_USE_FREERTOS)
    (void)xSemaphoreGive(bmi_data_mutex);
#endif
}

/**
 * @brief 以零等待方式尝试获取 BMI323 数据互斥量。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * 传入参数：无。
 * @return FreeRTOS 下锁存在且立即获取成功返回 1，否则返回 0；非 FreeRTOS 构建固定返回 1。
 * 调用方式：高频 BMI323 生产者入队或记录读失败时调用，锁竞争时丢弃当次观测。
 * 线程约束：任务上下文零等待、不阻塞；不使用 `FromISR` API，因此不可在 ISR 调用。
 */
static uint8_t imu_try_lock_bmi(void)
{
#if defined(IMU_MANAGER_USE_FREERTOS)
    if (bmi_data_mutex == NULL) {
        return 0U;
    }
    return xSemaphoreTake(bmi_data_mutex, (TickType_t)0) == pdTRUE ? 1U : 0U;
#else
    return 1U;
#endif
}

/*
 * LSM303 sensor frame -> vehicle Body Frame.
 *
 * The selected board mapping is a 180-degree rotation about Body Z.  Keep the
 * map as one proper rotation instead of distributing axis sign changes across
 * the acquisition and attitude paths.  PCB orientation remains a hardware
 * acceptance item; this is the source-level mapping recorded by the project:
 *
 *                  [-1  0  0]
 *     R_lsm_body = [ 0 -1  0], det(R) = +1
 *                  [ 0  0  1]
 */
static const float lsm303_sensor_to_body[3][3] = {
    {-1.0f, 0.0f, 0.0f},
    {0.0f, -1.0f, 0.0f},
    {0.0f, 0.0f, 1.0f},
};

/**
 * @brief 将 LSM303 传感器坐标中的三轴向量按冻结旋转矩阵转到车体坐标。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] sensor LSM303 传感器坐标中的 X/Y/Z 向量，单位原样保留。
 * @return 绕 Body Z 轴旋转 180 度后的向量，即 X/Y 取反、Z 保持。
 * 调用方式：LSM303 加速度和磁场读取成功后，在进入标定/姿态链之前调用。
 * 线程约束：纯定长浮点计算、可重入、不阻塞，不访问可变共享状态；仅在已保存 FPU 上下文时可从 ISR 调用。
 */
static Vector3f lsm303_to_body(Vector3f sensor)
{
    return (Vector3f){
        (lsm303_sensor_to_body[0][0] * sensor.x) +
            (lsm303_sensor_to_body[0][1] * sensor.y) +
            (lsm303_sensor_to_body[0][2] * sensor.z),
        (lsm303_sensor_to_body[1][0] * sensor.x) +
            (lsm303_sensor_to_body[1][1] * sensor.y) +
            (lsm303_sensor_to_body[1][2] * sensor.z),
        (lsm303_sensor_to_body[2][0] * sensor.x) +
            (lsm303_sensor_to_body[2][1] * sensor.y) +
            (lsm303_sensor_to_body[2][2] * sensor.z),
    };
}

/**
 * @brief 永久等待并获取 BMI323 驱动访问互斥量。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * 传入参数：无。
 * @return 无返回值；非 FreeRTOS 构建时为空操作。
 * 调用方式：BMI323 初始化、ODR 设置、诊断读取或其他可阻塞 SPI 驱动操作前调用。
 * 线程约束：调用前必须已成功创建 `bmi_driver_mutex`；FreeRTOS 下可永久阻塞，
 * 不可在 ISR 调用或在同一任务重入获取。
 */
static void imu_lock_bmi_driver(void)
{
#if defined(IMU_MANAGER_USE_FREERTOS)
    (void)xSemaphoreTake(bmi_driver_mutex, portMAX_DELAY);
#endif
}

/**
 * @brief 释放由当前任务持有的 BMI323 驱动互斥量。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * 传入参数：无。
 * @return 无返回值；非 FreeRTOS 构建时为空操作。
 * 调用方式：仅在成功获取 BMI 驱动锁的路径末尾成对调用。
 * 线程约束：仅限当前持锁任务，不可在 ISR 调用；释放路径不等待、正常情况下不阻塞，函数不检查句柄或所有权。
 */
static void imu_unlock_bmi_driver(void)
{
#if defined(IMU_MANAGER_USE_FREERTOS)
    (void)xSemaphoreGive(bmi_driver_mutex);
#endif
}

/**
 * @brief 以零等待方式尝试获取 BMI323 驱动互斥量。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * 传入参数：无。
 * @return FreeRTOS 下锁存在且立即获取成功返回 1，否则返回 0；非 FreeRTOS 构建固定返回 1。
 * 调用方式：高频 BMI323 采样任务每周期在 SPI 读取前调用，竞争时记录丢样而不等待。
 * 线程约束：任务上下文零等待、不阻塞；不使用 `FromISR` API，因此不可在 ISR 调用。
 */
static uint8_t imu_try_lock_bmi_driver(void)
{
#if defined(IMU_MANAGER_USE_FREERTOS)
    if (bmi_driver_mutex == NULL) {
        return 0U;
    }
    return xSemaphoreTake(bmi_driver_mutex, (TickType_t)0) == pdTRUE ? 1U : 0U;
#else
    return 1U;
#endif
}

/**
 * @brief 永久等待并获取 LSM303 数据、有效性、健康与统计互斥量。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * 传入参数：无。
 * @return 无返回值；非 FreeRTOS 构建时为空操作。
 * 调用方式：读写 LSM303 缓存样本、初始化标志、失败计数和时间戳前调用。
 * 线程约束：调用前必须已成功创建 `lsm_data_mutex`；FreeRTOS 下可永久阻塞，
 * 不可在 ISR 调用或在同一任务重入获取。
 */
static void imu_lock_lsm(void)
{
#if defined(IMU_MANAGER_USE_FREERTOS)
    (void)xSemaphoreTake(lsm_data_mutex, portMAX_DELAY);
#endif
}

/**
 * @brief 释放由当前任务持有的 LSM303 数据互斥量。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * 传入参数：无。
 * @return 无返回值；非 FreeRTOS 构建时为空操作。
 * 调用方式：仅在成功获取 LSM 数据锁的路径末尾成对调用。
 * 线程约束：仅限当前持锁任务，不可在 ISR 调用；释放路径不等待、正常情况下不阻塞，函数不检查句柄或所有权。
 */
static void imu_unlock_lsm(void)
{
#if defined(IMU_MANAGER_USE_FREERTOS)
    (void)xSemaphoreGive(lsm_data_mutex);
#endif
}

/**
 * @brief 永久等待并获取统一 `imu_snapshot` 互斥量。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * 传入参数：无。
 * @return 无返回值；非 FreeRTOS 构建时为空操作。
 * 调用方式：发布或复制聚合的双 IMU 原始快照前调用。
 * 线程约束：调用前必须已创建 `snapshot_mutex`；FreeRTOS 下可永久阻塞，不可在 ISR 或重入调用。
 */
static void imu_lock_snapshot(void)
{
#if defined(IMU_MANAGER_USE_FREERTOS)
    (void)xSemaphoreTake(snapshot_mutex, portMAX_DELAY);
#endif
}

/**
 * @brief 释放由当前任务持有的统一快照互斥量。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * 传入参数：无。
 * @return 无返回值；非 FreeRTOS 构建时为空操作。
 * 调用方式：仅在成功获取快照锁的路径末尾成对调用。
 * 线程约束：仅限当前持锁任务，不可在 ISR 调用；释放路径不等待、正常情况下不阻塞，函数不检查句柄或所有权。
 */
static void imu_unlock_snapshot(void)
{
#if defined(IMU_MANAGER_USE_FREERTOS)
    (void)xSemaphoreGive(snapshot_mutex);
#endif
}

/**
 * @brief 获取双 IMU 初始化 worker 进度互斥量。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * 传入参数：无。
 * @return 无返回值；非 FreeRTOS 构建或锁句柄为 `NULL` 时直接返回。
 * 调用方式：读写 `dual_init_status` 和初始化 worker 句柄前调用，并与 `imu_unlock_dual_init()` 成对。
 * 线程约束：仅限任务上下文，锁存在时可永久阻塞，不可在 ISR 或重入调用。
 * 锁不存在时不提供并发保护，调用方应先完成锁创建。
 */
static void imu_lock_dual_init(void)
{
#if defined(IMU_MANAGER_USE_FREERTOS)
    if (dual_init_mutex != NULL) {
        (void)xSemaphoreTake(dual_init_mutex, portMAX_DELAY);
    }
#endif
}

/**
 * @brief 释放双 IMU 初始化 worker 进度互斥量。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * 传入参数：无。
 * @return 无返回值；非 FreeRTOS 构建或锁句柄为 `NULL` 时直接返回。
 * 调用方式：仅在成功获取双初始化锁的路径末尾成对调用。
 * 线程约束：仅限当前持锁任务，不可在 ISR 调用；释放路径不等待、正常情况下不阻塞，
 * 锁句柄为空时不检查调用与获取是否成对。
 */
static void imu_unlock_dual_init(void)
{
#if defined(IMU_MANAGER_USE_FREERTOS)
    if (dual_init_mutex != NULL) {
        (void)xSemaphoreGive(dual_init_mutex);
    }
#endif
}

/**
 * @brief 根据 BMI323 首末捕获时间和样本间隔数计算四舍五入的实测采样率。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] stats 仅在调用期间借用的捕获统计快照。
 * @return 至少两个样本且末时间晚于首时间时返回 Hz，超过 `UINT16_MAX` 饱和；否则返回 0。
 * 调用方式：高频入队更新统计及外部捕获统计查询时调用。
 * 线程约束：纯计算、不阻塞；`stats` 指向稳定局部快照时可在 ISR 调用，
 * 若指向共享状态则调用者必须持有 BMI 数据锁且不得从 ISR 访问。
 */
static uint16_t imu_bmi_measured_rate_hz(const bmi323_capture_stat_t *stats)
{
    uint64_t elapsed_us;
    uint64_t rate_hz;

    if (stats == NULL || stats->sample_count < 2U ||
        stats->last_timestamp_us <= stats->first_timestamp_us) {
        return 0U;
    }
    elapsed_us = stats->last_timestamp_us - stats->first_timestamp_us;
    rate_hz = (((uint64_t)(stats->sample_count - 1U) * UINT64_C(1000000)) +
               (elapsed_us / UINT64_C(2))) /
              elapsed_us;
    return rate_hz > UINT16_MAX ? UINT16_MAX : (uint16_t)rate_hz;
}

/**
 * @brief 在持有 BMI 数据锁时清空环形缓冲、竞争丢样计数和捕获统计。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * 传入参数：无。
 * @return 无返回值；新统计仅保留当前 BMI323 配置采样率。
 * 调用方式：数据复位、BMI 初始化状态更新和 ODR 更改成功后调用。
 * 线程约束：调用者必须已持有 BMI 数据锁；函数不自行加锁且不阻塞，
 * 但会读取驱动的当前采样率，不可在 ISR 调用。
 */
static void imu_bmi_ring_reset_locked(void)
{
    (void)memset(&bmi_ring_buffer, 0, sizeof(bmi_ring_buffer));
    bmi_capture_contention_drop_count = 0U;
    bmi_capture_stats = (bmi323_capture_stat_t){
        .configured_rate_hz = (uint16_t)bmi323_get_sample_rate()
    };
}

/**
 * @brief 以饱和方式累加一次 BMI323 驱动或数据锁竞争丢样。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * 传入参数：无。
 * @return 无返回值；计数已为 `UINT32_MAX` 时保持饱和。
 * 调用方式：高频 BMI323 生产者无法立即获取驱动锁或数据锁时调用。
 * 线程约束：不阻塞，但对 `volatile` 计数的读-改-写不是原子操作；
 * 当前契约仅允许单一 BMI323 生产任务更新，不可从多任务或 ISR 并发调用。
 */
static void imu_bmi_capture_note_contention_drop(void)
{
    const uint32_t drops = bmi_capture_contention_drop_count;

    if (drops != UINT32_MAX) {
        bmi_capture_contention_drop_count = drops + 1U;
    }
}

/**
 * @brief 将有效 BMI323 原始样本以零等待方式写入环形缓冲并更新捕获统计。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] sample 仅在调用期间借用的原始样本；函数将结构按值复制，`NULL` 或 `valid=0` 时忽略。
 * @return 无返回值；数据锁竞争时记录丢样并返回，环形缓冲满时覆盖最旧样本并饱和累加溢出计数。
 * 调用方式：独立 BMI323 ODR 任务在 SPI 原始读取成功后调用。
 * 线程约束：仅限单一高频生产任务，不可在 ISR 调用；数据锁采用零等待，
 * 锁内执行定长复制、取模和 64 位采样率计算，不保留输入指针。
 */
static void imu_bmi_ring_push(const bmi323_raw_sample_t *sample)
{
    if (sample == NULL || sample->valid == 0U) {
        return;
    }

    /* The producer never waits behind the 10 ms manager. A lock contention
     * drops this observation and is separate from full-ring overflow. */
    if (imu_try_lock_bmi() == 0U) {
        imu_bmi_capture_note_contention_drop();
        return;
    }
    if (bmi_ring_buffer.count >= BMI_RING_BUFFER_SIZE) {
        bmi_ring_buffer.tail =
            (uint16_t)((bmi_ring_buffer.tail + 1U) % BMI_RING_BUFFER_SIZE);
        if (bmi_ring_buffer.overflow_count != UINT32_MAX) {
            ++bmi_ring_buffer.overflow_count;
        }
    } else {
        ++bmi_ring_buffer.count;
    }
    bmi_ring_buffer.buffer[bmi_ring_buffer.head] = *sample;
    bmi_ring_buffer.head =
        (uint16_t)((bmi_ring_buffer.head + 1U) % BMI_RING_BUFFER_SIZE);
    bmi_ring_buffer.last_capture_us = sample->timestamp_us;
    if (bmi_capture_stats.sample_count == 0U) {
        bmi_capture_stats.first_timestamp_us = sample->timestamp_us;
    }
    if (bmi_capture_stats.sample_count != UINT32_MAX) {
        ++bmi_capture_stats.sample_count;
    }
    bmi_capture_stats.overflow_count = bmi_ring_buffer.overflow_count;
    bmi_capture_stats.last_timestamp_us = sample->timestamp_us;
    bmi_capture_stats.last_timestamp =
        (uint32_t)(sample->timestamp_us / UINT64_C(1000));
    bmi_capture_stats.measured_rate_hz =
        imu_bmi_measured_rate_hz(&bmi_capture_stats);
    bmi_capture_stats.pending_count = bmi_ring_buffer.count;
    ++bmi_stats.read_calls;
    ++bmi_stats.update_count;
    bmi_stats.last_update_ms =
        (uint32_t)(sample->timestamp_us / UINT64_C(1000));
    bmi_stats.last_status = BSP_STATUS_OK;
    imu_unlock_bmi();
}

/**
 * @brief 以零等待方式记录一次 BMI323 真实读取失败。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * 传入参数：无。
 * @return 无返回值；数据锁竞争时只记竞争丢样，不更新读失败统计。
 * 成功持锁时 `read_fail_count` 饱和，但 `read_calls` 与 `invalid_count` 依照无符号回绕语义累加。
 * 调用方式：BMI323 ODR 任务在在线传感器读取失败且非 DATA_NOT_READY 时调用。
 * 线程约束：仅限单一 BMI323 生产任务，不可在 ISR 调用；数据锁零等待，函数不阻塞。
 */
static void imu_bmi_capture_failed(void)
{
    if (imu_try_lock_bmi() == 0U) {
        imu_bmi_capture_note_contention_drop();
        return;
    }
    ++bmi_stats.read_calls;
    bmi_stats.last_status = BSP_STATUS_ERROR;
    ++bmi_data.invalid_count;
    if (bmi_capture_stats.read_fail_count != UINT32_MAX) {
        ++bmi_capture_stats.read_fail_count;
    }
    bmi_capture_stats.pending_count = bmi_ring_buffer.count;
    imu_unlock_bmi();
}

/**
 * @brief 从 BMI323 环形缓冲复制最新样本，并在 O(1) 时间内丢弃所有较旧待处理样本。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[out] sample 成功时接收最新原始样本的可写结构；`NULL` 时失败。
 * @param[out] capture_us 可选接收环形缓冲最后捕获时间的指针；`NULL` 时忽略，失败时不修改。
 * @return 成功取出最新样本返回 1；输出指针为空或环形缓冲无样本返回 0。
 * 调用方式：10 ms IMU 管理路径每次更新 BMI 发布值时调用。
 * 线程约束：仅限任务上下文，内部可永久等待 BMI 数据锁，不可在 ISR 调用；
 * 输出结构由调用者所有，函数不保留指针。
 */
static uint8_t imu_bmi_ring_take_latest(bmi323_raw_sample_t *sample,
                                        uint64_t *capture_us)
{
    uint16_t latest_index;

    if (sample == NULL) {
        return 0U;
    }
    imu_lock_bmi();
    if (bmi_ring_buffer.count == 0U) {
        bmi_capture_stats.pending_count = 0U;
        imu_unlock_bmi();
        return 0U;
    }
    latest_index = (uint16_t)((bmi_ring_buffer.head + BMI_RING_BUFFER_SIZE -
                               1U) % BMI_RING_BUFFER_SIZE);
    *sample = bmi_ring_buffer.buffer[latest_index];
    if (capture_us != NULL) {
        *capture_us = bmi_ring_buffer.last_capture_us;
    }
    /* The manager intentionally publishes only the newest sample. Older
     * high-rate samples are discarded in O(1) time for the next manager tick. */
    bmi_ring_buffer.tail = bmi_ring_buffer.head;
    bmi_ring_buffer.count = 0U;
    bmi_capture_stats.pending_count = 0U;
    imu_unlock_bmi();
    return 1U;
}

/**
 * @brief 清除双 IMU 样本、有效性、环形缓冲、滤波/零参考和聚合快照运行状态。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] reset_count 非 0 时同时清零 BMI/LSM 历史统计；0 时保留计数但将最后状态置为 NOT_READY。
 * @return 无返回值；函数不重新初始化硬件，也不返回锁或子模块错误。
 * 调用方式：双 IMU 初始化开始、生命周期准备和 BMI 独立调试初始化时调用。
 * 线程约束：仅限任务上下文，不可在 ISR 调用；按 BMI→LSM→快照顺序分别可永久等待三把锁。
 * 调用方必须先禁用高频采集并串行化生命周期，避免无锁的滤波/零参考标志与运行任务竞争。
 */
static void imu_reset_data(uint8_t reset_count)
{
    attitude_zero_reset();
    last_filter_accel_timestamp_us = 0U;
    filter_lifecycle_active = 0U;
    imu_lock_bmi();
    bmi_data = (bmi323_data_t){0};
    imu_bmi_ring_reset_locked();
    bmi323_init_success = 0U;
    if (reset_count != 0U) {
        bmi_stats = (imu_sensor_stats_t){0};
    }
    bmi_stats.last_status = BSP_STATUS_NOT_READY;
    imu_unlock_bmi();

    imu_lock_lsm();
    lsm_accel_data = (lsm_accel_data_t){0};
    lsm_mag_data = (lsm_mag_data_t){0};
    lsm_accel_valid = 0U;
    lsm_mag_valid = 0U;
    lsm303_init_success = 0U;
    last_accel_success_tick = 0U;
    last_mag_success_tick = 0U;
    accel_fail_count = IMU_LSM303_FAIL_LIMIT;
    mag_fail_count = IMU_LSM303_FAIL_LIMIT;
    if (reset_count != 0U) {
        lsm_stats = (imu_sensor_stats_t){0};
    }
    lsm_stats.last_status = BSP_STATUS_NOT_READY;
    imu_unlock_lsm();

    imu_lock_snapshot();
    imu_snapshot = (imu_raw_data_t){0};
    imu_unlock_snapshot();
}

/**
 * @brief 提交 LSM303 驱动初始化结果并将两个数据分路重置为未验证状态。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] status `lsm303_init()` 返回的 BSP 状态。
 * @return 无返回值；仅 `BSP_STATUS_OK` 将初始化标志置 1，但在新加速度和磁场样本成功前仍保持离线。
 * 调用方式：LSM303 独立初始化 worker 完成驱动调用后调用。
 * 线程约束：仅限任务上下文，内部可永久等待 LSM 数据锁，不可在 ISR 调用。
 */
static void imu_mark_lsm_initialized(bsp_status_t status)
{
    imu_lock_lsm();
    lsm303_init_success = status == BSP_STATUS_OK ? 1U : 0U;
    lsm_accel_valid = 0U;
    lsm_mag_valid = 0U;
    last_accel_success_tick = 0U;
    last_mag_success_tick = 0U;
    accel_fail_count = IMU_LSM303_FAIL_LIMIT;
    mag_fail_count = IMU_LSM303_FAIL_LIMIT;
    /* Keep the aggregate status for diagnostics; online uses independent
     * initialization, freshness, and failure health. */
    lsm_stats.last_status = status;
    imu_unlock_lsm();
}

/**
 * @brief 提交 BMI323 初始化结果，同时清空数据、环形缓冲和捕获统计。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] initialized BMI323 驱动初始化结果，0 表示失败，非 0 表示成功。
 * @return 无返回值；失败时 `bmi_stats.last_status` 置 ERROR，成功时置 OK。
 * 调用方式：BMI323 初始化 worker 或独立调试初始化在释放驱动锁后调用。
 * 线程约束：仅限任务上下文，内部可永久等待 BMI 数据锁，不可在 ISR 调用。
 */
static void imu_mark_bmi323_initialized(uint8_t initialized)
{
    imu_lock_bmi();
    bmi_data = (bmi323_data_t){0};
    imu_bmi_ring_reset_locked();
    bmi_stats = (imu_sensor_stats_t){0};
    bmi_stats.last_status = initialized != 0U ? BSP_STATUS_OK : BSP_STATUS_ERROR;
    bmi323_init_success = initialized;
    imu_unlock_bmi();
}

/**
 * @brief 在 LSM 数据锁下组装初始化、新鲜度、失败计数和联合在线状态快照。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] now_ms 与成功时间戳同域的当前 32 位单调毫秒值。
 * @param[out] health 成功时接收全字段快照的可写结构；`NULL` 时静默返回。
 * @return 无返回值；从未成功且失败数达限的分路年龄置 `UINT32_MAX`。
 * 在线需同时满足初始化成功、两路失败数均小于 10，且两路年龄均小于 1000 ms。
 * 调用方式：在线查询与每秒健康日志调用。
 * 线程约束：仅限任务上下文，内部可永久等待 LSM 数据锁，不可在 ISR 调用；输出指针不被保留。
 */
static void imu_lsm_health_snapshot(uint32_t now_ms, imu_lsm_health_t *health)
{
    if (health == NULL) {
        return;
    }

    imu_lock_lsm();
    health->init = lsm303_init_success;
    health->accel_fail = accel_fail_count;
    health->mag_fail = mag_fail_count;
    health->accel_age = (accel_fail_count >= IMU_LSM303_FAIL_LIMIT &&
                         last_accel_success_tick == 0U)
                            ? UINT32_MAX
                            : (uint32_t)(now_ms - last_accel_success_tick);
    health->mag_age = (mag_fail_count >= IMU_LSM303_FAIL_LIMIT &&
                       last_mag_success_tick == 0U)
                          ? UINT32_MAX
                          : (uint32_t)(now_ms - last_mag_success_tick);
    health->online = (health->init != 0U &&
                      health->accel_fail < IMU_LSM303_FAIL_LIMIT &&
                      health->mag_fail < IMU_LSM303_FAIL_LIMIT &&
                      health->accel_age < IMU_LSM303_ONLINE_TIMEOUT_MS &&
                      health->mag_age < IMU_LSM303_ONLINE_TIMEOUT_MS)
                         ? 1U
                         : 0U;
    imu_unlock_lsm();
}

/**
 * @brief 使用当前 IMU 毫秒时钟查询 LSM303 加速度与磁力计联合在线状态。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * 传入参数：无。
 * @return 初始化、两路失败计数和 1000 ms 新鲜度门限全部通过返回 1，否则返回 0。
 * 调用方式：IMU 公共 READY/在线查询调用。
 * 线程约束：仅限任务上下文；间接可永久等待 LSM 数据锁，不可在 ISR 调用。
 */
static uint8_t imu_lsm_is_online(void)
{
    imu_lsm_health_t health = {0};

    imu_lsm_health_snapshot(imu_time_now_ms(), &health);
    return health.online;
}

/**
 * @brief 格式化并输出 LSM303 初始化、两路年龄/失败计数和在线标志。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] now_ms 用于计算样本年龄的当前单调毫秒时间。
 * @return 无返回值；格式化裁剪或日志失败不上报。
 * 调用方式：IMU 任务按 1000 ms 周期调用。
 * 线程约束：仅限任务上下文，不可在 ISR 调用；内部获取 LSM 锁并可在日志后端阻塞，使用栈上 160 字节缓冲。
 */
static void imu_lsm_health_log(uint32_t now_ms)
{
    char line[160];
    imu_lsm_health_t health = {0};

    imu_lsm_health_snapshot(now_ms, &health);
    (void)snprintf(line, sizeof(line),
                   "[LSM303_HEALTH] init=%u accel_age=%lu mag_age=%lu "
                   "accel_fail=%u mag_fail=%u online=%u\r\n",
                   (unsigned)health.init, (unsigned long)health.accel_age,
                   (unsigned long)health.mag_age, (unsigned)health.accel_fail,
                   (unsigned)health.mag_fail, (unsigned)health.online);
    imu_init_log(line);
}

/**
 * @brief 对 BMI323 诊断错误做变化检测，仅在非 OK 错误首次出现时输出 ERROR 日志。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] status 当前 BMI323 诊断状态。
 * @return 无返回值；OK 只清除去重状态不打印，与上次相同的错误被抑制，日志失败不上报。
 * 调用方式：BMI323 初始化摘要和周期调试日志在决定最终状态后调用。
 * 线程约束：仅限已串行的任务上下文，可在日志后端阻塞，不可在 ISR 调用；
 * `bmi323_last_logged_error` 无锁，并发初始化 worker 与运行日志调用可丢失去重一致性。
 */
static void imu_bmi323_error_log(bmi323_diag_status_t status)
{
    char line[64];

    if (status == BMI323_DIAG_STATUS_OK) {
        bmi323_last_logged_error = BMI323_DIAG_STATUS_OK;
        return;
    }
    if (status == bmi323_last_logged_error) {
        return;
    }

    (void)snprintf(line, sizeof(line), "[BMI323][ERROR]\r\nreason=%s\r\n",
                   bmi323_diag_status_name(status));
    LOG_ERROR(line);
    bmi323_last_logged_error = status;
}

/**
 * @brief 读取并输出 BMI323 探测 ID、配置寄存器、SPI 错误数和最后状态。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * 传入参数：无。
 * @return 无返回值；格式化裁剪、诊断快照内部错误或日志发送失败都不上报。
 * 调用方式：BMI323 初始化结果提交后在 worker 或独立调试初始化路径调用。
 * 线程约束：仅限任务上下文，不可在 ISR 调用；内部可永久等待 BMI 驱动锁，
 * 使用一个 `LOG_SERVICE_TEXT_MAX+1` 栈缓冲并连续发送多条日志。
 */
static void imu_bmi323_init_log(void)
{
    char line[LOG_SERVICE_TEXT_MAX + 1U];
    bmi323_diagnostics_t diagnostics;

    imu_lock_bmi_driver();
    bmi323_get_diagnostics(&diagnostics);
    imu_unlock_bmi_driver();
    (void)snprintf(line, sizeof(line),
                   "[BMI323][INIT]\r\nPROBE_ID=0x%02X\r\n"
                   "POST_RESET_ID=0x%02X\r\ninit_result=%ld\r\n",
                   (unsigned)diagnostics.who_am_i,
                   (unsigned)diagnostics.post_reset_who_am_i,
                   (long)diagnostics.init_result);
    imu_init_log(line);
    (void)snprintf(line, sizeof(line),
                   "[BMI323][INIT]\r\nCTRL_ACC=0x%04X\r\nCTRL_GYR=0x%04X\r\n"
                   "ACC_CONF=0x%04X\r\nGYR_CONF=0x%04X\r\n",
                   (unsigned)diagnostics.ctrl_acc, (unsigned)diagnostics.ctrl_gyr,
                   (unsigned)diagnostics.acc_conf, (unsigned)diagnostics.gyr_conf);
    imu_init_log(line);
    (void)snprintf(line, sizeof(line), "[BMI323][INIT]\r\nspi_error_count=%lu\r\n",
                   (unsigned long)diagnostics.spi_error_count);
    imu_init_log(line);
    imu_bmi323_error_log(diagnostics.last_status);
}

/**
 * @brief 聚合 BMI323 驱动诊断、最新 SI 样本与捕获统计，输出周期调试日志。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * 传入参数：无。
 * @return 无返回值；无有效 raw diag 且驱动状态为 OK 时对外报 DATA_NOT_READY，
 * 捕获统计查询失败被忽略并保留局部零值，日志失败不上报。
 * 调用方式：IMU 任务按 1000 ms 健康周期调用。
 * 线程约束：仅限低频任务上下文，可在锁或日志后端阻塞，不可在 ISR 调用；顺序获取并释放驱动锁、BMI 数据锁，
 * 再通过公共 API 重新获取 BMI 数据锁；栈上同时占用 4 个 `LOG_SERVICE_TEXT_MAX+1` 文本缓冲，CPU/日志开销较高。
 */
static void imu_bmi323_debug_log(void)
{
    char line[LOG_SERVICE_TEXT_MAX + 1U];
    char raw_line[LOG_SERVICE_TEXT_MAX + 1U];
    char state_line[LOG_SERVICE_TEXT_MAX + 1U];
    char sample_line[LOG_SERVICE_TEXT_MAX + 1U];
    bmi323_diagnostics_t diagnostics;
    bmi323_diag_t diag;
    bmi323_data_t sample;
    bmi323_capture_stat_t capture = {0};
    bmi323_diag_status_t reported_status;
    uint8_t online;

    imu_lock_bmi_driver();
    bmi323_get_diagnostics(&diagnostics);
    bmi323_get_diag(&diag);
    online = bmi323_is_online();
    imu_unlock_bmi_driver();
    imu_lock_bmi();
    sample = bmi_data;
    imu_unlock_bmi();
    (void)imu_manager_get_bmi323_capture_stats(&capture);
    reported_status = diagnostics.last_status;
    if (diag.valid == 0U && reported_status == BMI323_DIAG_STATUS_OK) {
        reported_status = BMI323_DIAG_STATUS_DATA_NOT_READY;
    }
    imu_bmi323_error_log(reported_status);
    (void)snprintf(state_line, sizeof(state_line),
                   "[BMI323][STATE]\r\nonline=%u\r\nraw_valid=%u\r\n"
                   "init_result=%ld\r\n",
                   (unsigned)online, (unsigned)diag.valid,
                   (long)diagnostics.init_result);
    imu_init_log(state_line);
    (void)snprintf(line, sizeof(line),
                   "[BMI323][DEBUG]\r\nread_ok=%lu\r\nread_fail=%lu\r\n"
                   "last_status=%s\r\n",
                   (unsigned long)diagnostics.read_ok,
                   (unsigned long)diagnostics.read_fail,
                   bmi323_diag_status_name(reported_status));
    imu_init_log(line);
    (void)snprintf(raw_line, sizeof(raw_line),
                   "[BMI323][DEBUG]\r\nwhoami=0x%02X\r\n"
                   "rx0=0x%02X\r\nrx1=0x%02X\r\n"
                   "rx2=0x%02X\r\nrx3=0x%02X\r\n"
                   "spi_status=0x%02X\r\n",
                   (unsigned)diag.whoami,
                   (unsigned)diag.rx0,
                   (unsigned)diag.rx1,
                   (unsigned)diag.rx2,
                   (unsigned)diag.rx3,
                   (unsigned)diag.spi_status);
    imu_init_log(raw_line);
    (void)snprintf(sample_line, sizeof(sample_line),
                   "[BMI323][SAMPLE]\r\nvalid=%u sample=%lu timestamp_ms=%lu\r\n",
                   (unsigned)sample.valid, (unsigned long)sample.sample_count,
                   (unsigned long)sample.timestamp);
    imu_init_log(sample_line);
    (void)snprintf(sample_line, sizeof(sample_line),
                   "[BMI323][CAPTURE]\r\nconfigured_rate=%u\r\n"
                   "measured_rate=%u\r\nsample=%lu\r\noverflow=%lu\r\n",
                   (unsigned)capture.configured_rate_hz,
                   (unsigned)capture.measured_rate_hz,
                   (unsigned long)capture.sample_count,
                   (unsigned long)capture.overflow_count);
    imu_init_log(sample_line);
    (void)snprintf(sample_line, sizeof(sample_line),
                   "[BMI323][CAPTURE]\r\npending=%u\r\nlatency_us=%lu\r\n"
                   "read_fail=%lu\r\n",
                   (unsigned)capture.pending_count,
                   (unsigned long)capture.max_latency_us,
                   (unsigned long)capture.read_fail_count);
    imu_init_log(sample_line);
    (void)snprintf(sample_line, sizeof(sample_line),
                   "[BMI323][RAW]\r\naccel=%d,%d,%d\r\n",
                   (int)diagnostics.accel_raw_x, (int)diagnostics.accel_raw_y,
                   (int)diagnostics.accel_raw_z);
    imu_init_log(sample_line);
    (void)snprintf(sample_line, sizeof(sample_line),
                   "[BMI323][RAW]\r\ngyro=%d,%d,%d\r\n",
                   (int)diagnostics.gyro_raw_x, (int)diagnostics.gyro_raw_y,
                   (int)diagnostics.gyro_raw_z);
    imu_init_log(sample_line);
}

/**
 * @brief 在完整启动门禁后对最新 LSM303 快照执行标定、水平旋转、滤波和旧姿态更新。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] data 仅在调用期间借用的聚合原始快照；必须 `online!=0`，函数不保留指针。
 * @return 无返回值；启动未 READY 时重置滤波历史/姿态零点并返回，空或离线数据也静默返回。
 * 调用方式：聚合快照发布仅在 LSM 加速度时间戳前进且加速度/磁场都新鲜时调用。
 * 线程约束：仅限 10 ms IMU 管理任务，不可在 ISR 调用；内部访问标定/滤波/姿态共享状态时可因互斥量阻塞，并执行浮点旋转。
 * `g_leveling_lsm` 与生命周期标志未在本函数加锁，调用方必须将水平状态提交/复位与运行更新串行化。
 */
static void imu_publish_filter_snapshot(const imu_raw_data_t *data)
{
    imu_calibrated_data_t calibrated_data;

    if (imu_boot_manager_is_ready() == 0U) {
        if (filter_lifecycle_active != 0U) {
            /* A failed/restarted static window must not reuse pre-reset
             * filter history when the next zero-reference window begins. */
            imu_filter_init();
            filter_lifecycle_active = 0U;
            last_filter_accel_timestamp_us = 0U;
        }
        attitude_zero_reset();
        /* Never seed or advance the filter before the full boot sequence. */
        return;
    }
    if (data == NULL || data->online == 0U) {
        return;
    }
    if (filter_lifecycle_active == 0U) {
        imu_filter_init();
        filter_lifecycle_active = 1U;
    }
    calibrated_data = imu_calibration_apply(data);
    {
        float accel_input[3] = {calibrated_data.ax, calibrated_data.ay,
                                calibrated_data.az};
        float accel_output[3];
        float mag_input[3] = {calibrated_data.mx, calibrated_data.my,
                              calibrated_data.mz};
        float mag_output[3];

        /* LSM303 values were mapped to the Body Frame at acquisition. Apply
         * only the frozen leveling matrix here; do not mirror an axis twice. */
        imu_leveling_rotate_vector(&g_leveling_lsm, accel_input, accel_output);
        imu_leveling_rotate_vector(&g_leveling_lsm, mag_input, mag_output);
        calibrated_data.ax = accel_output[0];
        calibrated_data.ay = accel_output[1];
        calibrated_data.az = accel_output[2];
        calibrated_data.lsm_ax = accel_output[0];
        calibrated_data.lsm_ay = accel_output[1];
        calibrated_data.lsm_az = accel_output[2];
        calibrated_data.mx = mag_output[0];
        calibrated_data.my = mag_output[1];
        calibrated_data.mz = mag_output[2];
        calibrated_data.lsm_mx = calibrated_data.mx;
        calibrated_data.lsm_my = calibrated_data.my;
        calibrated_data.lsm_mz = calibrated_data.mz;
    }
    imu_filter_update(&calibrated_data);
    if (imu_filter_is_ready() != 0U) {
        attitude_update();
    }
}

/**
 * @brief 分别复制 LSM303 与 BMI323 最新数据，生成带新鲜度的统一原始快照并驱动生命周期。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * 传入参数：无。
 * @return 无返回值；两路数据分别加锁复制，快照是最新可用值的聚合而非硬件同时刻原子捕获。
 * 调用方式：`imu_update()` 在完成当次 LSM 读取与 BMI 环形缓冲消费后调用。
 * 有任一有效分路时调用 `imu_boot_manager_update()`，否则仅步进超时；完整且新的 LSM 快照才进入旧滤波/姿态链。
 * 线程约束：仅限 10 ms IMU 管理任务，不可在 ISR 调用；依次获取 LSM、BMI、快照锁（不同时持有）并可阻塞，
 * 随后可进入标定、启动状态机、滤波与姿态路径，具有有界浮点/RTOS 锁开销。
 */
static void imu_publish_unified_snapshot(void)
{
    imu_raw_data_t snapshot = {0};
    bmi323_data_t bmi = {0};
    lsm_accel_data_t accel = {0};
    lsm_mag_data_t mag = {0};
    uint8_t accel_valid;
    uint8_t mag_valid;
    const uint64_t now_us = imu_time_now_us();
    uint8_t accel_fresh;
    uint8_t mag_fresh;

    imu_lock_lsm();
    accel = lsm_accel_data;
    mag = lsm_mag_data;
    accel_valid = lsm_accel_valid;
    mag_valid = lsm_mag_valid;
    imu_unlock_lsm();
    accel_fresh = (accel_valid != 0U &&
                   imu_lsm_timestamp_is_fresh(accel.timestamp_us, now_us))
                      ? 1U
                      : 0U;
    mag_fresh = (mag_valid != 0U &&
                 imu_lsm_timestamp_is_fresh(mag.timestamp_us, now_us))
                    ? 1U
                    : 0U;
    imu_lock_bmi();
    bmi = bmi_data;
    imu_unlock_bmi();

    snapshot.ax = accel.ax;
    snapshot.ay = accel.ay;
    snapshot.az = accel.az;
    snapshot.mx = mag.mx;
    snapshot.my = mag.my;
    snapshot.mz = mag.mz;
    snapshot.timestamp_us = accel.timestamp_us >= mag.timestamp_us
                                ? accel.timestamp_us
                                : mag.timestamp_us;
    snapshot.timestamp = (uint32_t)(snapshot.timestamp_us /
                                    UINT64_C(1000));
    snapshot.online = (accel_fresh != 0U && mag_fresh != 0U) ? 1U : 0U;
    snapshot.lsm_ax = snapshot.ax;
    snapshot.lsm_ay = snapshot.ay;
    snapshot.lsm_az = snapshot.az;
    snapshot.lsm_mx = snapshot.mx;
    snapshot.lsm_my = snapshot.my;
    snapshot.lsm_mz = snapshot.mz;
    snapshot.bmi_ax = bmi.accel_x;
    snapshot.bmi_ay = bmi.accel_y;
    snapshot.bmi_az = bmi.accel_z;
    snapshot.bmi_gx = bmi.gyro_x;
    snapshot.bmi_gy = bmi.gyro_y;
    snapshot.bmi_gz = bmi.gyro_z;
    snapshot.lsm_timestamp = snapshot.timestamp;
    snapshot.lsm_timestamp_us = snapshot.timestamp_us;
    snapshot.lsm_accel_timestamp_us = accel.timestamp_us;
    snapshot.lsm_mag_timestamp_us = mag.timestamp_us;
    snapshot.bmi_timestamp = bmi.timestamp;
    snapshot.bmi_timestamp_us = bmi.timestamp_us;
    snapshot.lsm_accel_valid = accel_fresh;
    snapshot.lsm_mag_valid = mag_fresh;
    snapshot.bmi_accel_valid = bmi.accel_valid;
    snapshot.bmi_gyro_valid = bmi.gyro_valid;

    imu_lock_snapshot();
    imu_snapshot = snapshot;
    imu_unlock_snapshot();

    if (snapshot.lsm_accel_valid != 0U || snapshot.lsm_mag_valid != 0U ||
        snapshot.bmi_accel_valid != 0U || snapshot.bmi_gyro_valid != 0U) {
        imu_boot_manager_update(&snapshot);
    } else {
        imu_boot_manager_step();
    }
    /* Only the legacy complete LSM view is allowed to reach the existing
     * calibration/filter/attitude chain. BMI323 remains telemetry-only. */
    if (snapshot.online != 0U &&
        snapshot.lsm_accel_timestamp_us > last_filter_accel_timestamp_us) {
        last_filter_accel_timestamp_us = snapshot.lsm_accel_timestamp_us;
        imu_publish_filter_snapshot(&snapshot);
    }
}

/**
 * @brief 在双 AHRS 更新前检查启动门禁，并一次性注入冻结的双 IMU 偏置。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * 传入参数：无。
 * @return IMU 启动状态 READY 后返回 1；未 READY 时清除曾注入的偏置并返回 0。
 * 偏置查询与注入 API 无失败返回通道，因此 1 仅表示门禁通过。
 * 调用方式：每个 BMI323 高频样本进入双 AHRS 前调用，首个 READY 样本完成偏置注入。
 * 线程约束：仅限单一 BMI323 采样任务，不可在 ISR 调用；内部读取启动/标定共享状态时可因互斥量阻塞。
 * `dual_ahrs_bias_injected` 无锁，生命周期复位必须先关闭 BMI 采集并与本路径串行化。
 */
static uint8_t imu_dual_ahrs_prepare(void)
{
    if (imu_boot_manager_is_ready() == 0U) {
        if (dual_ahrs_bias_injected != 0U) {
            dual_ahrs_set_bias(NULL);
            dual_ahrs_bias_injected = 0U;
        }
        return 0U;
    }

    if (dual_ahrs_bias_injected == 0U) {
        const imu_calibration_result_t result = imu_calibration_get_result();
        const dual_ahrs_bias_t bias = {
            .bmi_accel = {result.bmi_accel_bias.x,
                          result.bmi_accel_bias.y,
                          result.bmi_accel_bias.z},
            .bmi_gyro = {result.bmi_gyro_bias.x,
                         result.bmi_gyro_bias.y,
                         result.bmi_gyro_bias.z},
            .lsm_accel = {result.lsm_accel_bias.x,
                          result.lsm_accel_bias.y,
                          result.lsm_accel_bias.z},
        };
        dual_ahrs_set_bias(&bias);
        dual_ahrs_bias_injected = 1U;
    }
    return 1U;
}

/** 清除两路已冻结水平校准状态。 */
void imu_manager_reset_leveling(void)
{
    imu_leveling_init(&g_leveling_bmi);
    imu_leveling_init(&g_leveling_lsm);
#if !SMARTCAR_BMI323_DEBUG_ONLY
    dual_ahrs_set_leveling(&g_leveling_bmi, &g_leveling_lsm);
    dual_ahrs_set_local_gravity(g_leveling_bmi.g_local_mps2);
#endif
}

/** 在静态窗口关闭时一次性提交水平校准状态。 */
void imu_manager_commit_leveling(void)
{
    const imu_calibration_static_statistics_t statistics =
        imu_calibration_get_static_statistics();

    (void)imu_leveling_compute_with_accel_std_limit(
        &g_leveling_lsm, statistics.lsm.accel_mean,
        statistics.lsm.gyro_rms_radps, statistics.lsm.accel_std_mps2,
        statistics.lsm.valid_ratio, LSM_ACCEL_STD_MAX);
    (void)imu_leveling_compute_with_accel_std_limit(
        &g_leveling_bmi, statistics.bmi.accel_mean,
        statistics.bmi.gyro_rms_radps, statistics.bmi.accel_std_mps2,
        statistics.bmi.valid_ratio, BMI_ACCEL_STD_MAX);
#if !SMARTCAR_BMI323_DEBUG_ONLY
    dual_ahrs_set_leveling(&g_leveling_bmi, &g_leveling_lsm);
    dual_ahrs_set_local_gravity(g_leveling_bmi.g_local_mps2);
#endif
}

/** 复制两路水平校准快照。 */
void imu_manager_get_leveling_states(imu_leveling_state_t *bmi,
                                     imu_leveling_state_t *lsm)
{
    if (bmi != NULL) {
        *bmi = g_leveling_bmi;
    }
    if (lsm != NULL) {
        *lsm = g_leveling_lsm;
    }
}

/**
 * @brief 从高频 BMI323 生产者构造独立双 AHRS 输入，并附带最新且新鲜的 LSM303 加速度/磁场快照。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] bmi_sample 仅在调用期间借用的 SI 单位 BMI323 样本；`NULL` 时静默返回。
 * @return 无返回值；启动门禁未 READY 时不调用 `dual_ahrs_update()`，底层姿态更新也无失败返回通道。
 * 调用方式：BMI323 ODR 任务在每次成功样本、真实读失败或驱动锁竞争后调用，
 * 失败/竞争路径传入无效 BMI 样本以驱动姿态新鲜度逻辑。
 * 线程约束：仅限单一 BMI323 高频任务，不可在 ISR 调用；内部可读取标定/启动锁、永久等待 LSM 数据锁并执行姿态浮点计算。
 * 有效标志在 LSM 锁内冻结，但当前实现在解锁后再读模块级 `lsm_*_valid` 决定时间戳是否置零，
 * 若另一任务并发更改标志，同一 `input` 的 valid 与 timestamp 可不一致。
 */
static void imu_dual_ahrs_feed_bmi(const bmi323_data_t *bmi_sample)
{
    lsm_accel_data_t lsm_accel = {0};
    lsm_mag_data_t lsm_mag = {0};
    dual_ahrs_input_t input = {0};
    const uint64_t now_us = imu_time_now_us();

    if (bmi_sample == NULL) {
        return;
    }
    if (imu_dual_ahrs_prepare() == 0U) {
        return;
    }
    imu_lock_lsm();
    lsm_accel = lsm_accel_data;
    lsm_mag = lsm_mag_data;
    input.lsm_accel_valid =
        (lsm_accel_valid != 0U &&
         imu_lsm_timestamp_is_fresh(lsm_accel.timestamp_us, now_us))
            ? 1U
            : 0U;
    input.lsm_mag_valid =
        (lsm_mag_valid != 0U &&
         imu_lsm_timestamp_is_fresh(lsm_mag.timestamp_us, now_us))
            ? 1U
            : 0U;
    imu_unlock_lsm();
    input.bmi_accel = (dual_ahrs_vector3_t){bmi_sample->accel_x,
                                             bmi_sample->accel_y,
                                             bmi_sample->accel_z};
    /* BMI acceleration and gyro share the same frozen leveling rotation in
     * DualAHRS; no axis-specific sign change is permitted between them. */
    input.gyro = (dual_ahrs_vector3_t){bmi_sample->gyro_x, bmi_sample->gyro_y,
                                      bmi_sample->gyro_z};
    input.lsm_accel = (dual_ahrs_vector3_t){lsm_accel.ax, lsm_accel.ay,
                                            lsm_accel.az};
    input.mag = (dual_ahrs_vector3_t){lsm_mag.mx, lsm_mag.my, lsm_mag.mz};
    input.bmi_timestamp_us = bmi_sample->timestamp_us;
    input.lsm_accel_timestamp_us =
        lsm_accel_valid != 0U ? lsm_accel.timestamp_us : 0U;
    input.lsm_mag_timestamp_us =
        lsm_mag_valid != 0U ? lsm_mag.timestamp_us : 0U;
    input.lsm_timestamp_us = input.lsm_accel_timestamp_us >=
                                     input.lsm_mag_timestamp_us
                                 ? input.lsm_accel_timestamp_us
                                 : input.lsm_mag_timestamp_us;
    input.bmi_accel_valid = bmi_sample->accel_valid;
    input.bmi_gyro_valid = bmi_sample->gyro_valid;
    dual_ahrs_update(&input);
}

/**
 * @brief 分别读取 LSM303 加速度和磁场，转到车体坐标并更新新鲜度/失败统计。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * 传入参数：无。
 * @return 两路均成功或仅返回 `BSP_STATUS_NOT_READY` 时返回 `BSP_STATUS_OK`；
 * 真实错误优先返回加速度错误，否则返回磁力计错误。NOT_READY 保留上次值、旧时间戳和原有 valid，
 * 后续由 1000 ms 新鲜度门限使其离线；真实错误则清除对应 valid 并饱和累加失败数。
 * 调用方式：标准 `imu_update()` 在初始化屏障通过后每 10 ms 调用。
 * 线程约束：仅限单一 IMU 管理任务，不可在 ISR 调用；两次 I2C 驱动读取可阻塞，
 * 开始/结束时间中点作为捕获时间。函数在 I2C 读取期间不持有 LSM 数据锁，更新后可调用磁力计滤波。
 */
static bsp_status_t imu_update_lsm303(void)
{
    lsm_accel_data_t next_accel;
    lsm_mag_data_t next_mag;
    Vector3f acc;
    Vector3f mag;
    bsp_status_t acc_status;
    bsp_status_t mag_status;
    bsp_status_t status;
    uint8_t previous_accel_valid;
    uint8_t previous_mag_valid;
    uint32_t accel_success_tick = 0U;
    uint32_t mag_success_tick = 0U;
    uint64_t accel_read_start_us = 0U;
    uint64_t accel_read_end_us = 0U;
    uint64_t mag_read_start_us = 0U;
    uint64_t mag_read_end_us = 0U;
    uint64_t accel_timestamp_us = 0U;
    uint64_t mag_timestamp_us = 0U;
    uint32_t timestamp;

    imu_lock_lsm();
    next_accel = lsm_accel_data;
    next_mag = lsm_mag_data;
    previous_accel_valid = lsm_accel_valid;
    previous_mag_valid = lsm_mag_valid;
    imu_unlock_lsm();

    accel_read_start_us = imu_time_now_us();
    acc_status = lsm303_read_acc(&acc);
    accel_read_end_us = imu_time_now_us();
    if (acc_status == BSP_STATUS_OK) {
        accel_timestamp_us = accel_read_start_us +
                             ((accel_read_end_us - accel_read_start_us) / 2U);
        accel_success_tick = (uint32_t)(accel_timestamp_us / UINT64_C(1000));
    }
    mag_read_start_us = imu_time_now_us();
    mag_status = lsm303_read_mag(&mag);
    mag_read_end_us = imu_time_now_us();
    if (mag_status == BSP_STATUS_OK) {
        mag_timestamp_us = mag_read_start_us +
                           ((mag_read_end_us - mag_read_start_us) / 2U);
        mag_success_tick = (uint32_t)(mag_timestamp_us / UINT64_C(1000));
    }
    /* A data-ready miss is a normal no-new-sample result, not a bus fault.
     * Keep the last valid value cached while preserving its old timestamp. */
    status = BSP_STATUS_OK;
    if (acc_status != BSP_STATUS_OK && acc_status != BSP_STATUS_NOT_READY) {
        status = acc_status;
    }
    if (mag_status != BSP_STATUS_OK && mag_status != BSP_STATUS_NOT_READY &&
        status == BSP_STATUS_OK) {
        status = mag_status;
    }
    timestamp = imu_time_now_ms();
    if (acc_status == BSP_STATUS_OK) {
        const Vector3f body_acc = lsm303_to_body(acc);

        /* Convert the installed LSM303 sensor frame to the vehicle Body
         * Frame before calibration and leveling. BMI323 remains unchanged. */
        next_accel.ax = body_acc.x;
        next_accel.ay = body_acc.y;
        next_accel.az = body_acc.z;
        next_accel.timestamp = accel_success_tick;
        next_accel.timestamp_us = accel_timestamp_us;
    }
    if (mag_status == BSP_STATUS_OK) {
        const Vector3f body_mag = lsm303_to_body(mag);

        next_mag.mx = body_mag.x;
        next_mag.my = body_mag.y;
        next_mag.mz = body_mag.z;
        next_mag.timestamp = mag_success_tick;
        next_mag.timestamp_us = mag_timestamp_us;
    }

    imu_lock_lsm();
    lsm_accel_data = next_accel;
    lsm_mag_data = next_mag;
    lsm_accel_valid = acc_status == BSP_STATUS_OK
                          ? 1U
                          : (acc_status == BSP_STATUS_NOT_READY
                                 ? previous_accel_valid
                                 : 0U);
    lsm_mag_valid = mag_status == BSP_STATUS_OK
                        ? 1U
                        : (mag_status == BSP_STATUS_NOT_READY
                               ? previous_mag_valid
                               : 0U);
    if (acc_status == BSP_STATUS_OK) {
        last_accel_success_tick = accel_success_tick;
        accel_fail_count = 0U;
    } else if (acc_status != BSP_STATUS_NOT_READY &&
               accel_fail_count < UINT8_MAX) {
        ++accel_fail_count;
    }
    if (mag_status == BSP_STATUS_OK) {
        last_mag_success_tick = mag_success_tick;
        mag_fail_count = 0U;
    } else if (mag_status != BSP_STATUS_NOT_READY &&
               mag_fail_count < UINT8_MAX) {
        ++mag_fail_count;
    }
    ++lsm_stats.read_calls;
    if (acc_status == BSP_STATUS_OK || mag_status == BSP_STATUS_OK) {
        ++lsm_stats.update_count;
        lsm_stats.last_update_ms = timestamp;
    }
    lsm_stats.last_status = (status == BSP_STATUS_OK &&
                             (acc_status == BSP_STATUS_NOT_READY ||
                              mag_status == BSP_STATUS_NOT_READY))
                                ? BSP_STATUS_NOT_READY
                                : status;
    imu_unlock_lsm();

    if (mag_status == BSP_STATUS_OK) {
        mag_filter_update(&next_mag);
    }

    return status;
}

/**
 * @brief 消费 BMI323 环形缓冲中的最新原始样本，转换为 SI 单位并刷新发布缓存/延迟统计。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * 传入参数：无。
 * @return 无返回值；无待处理样本时将当前 BMI 发布有效标志清零并累加 `invalid_count`，
 * 但保留上次数值/时间戳；有样本时只发布最新一条，更旧待处理样本已被丢弃。
 * 调用方式：`imu_update()` 每 10 ms 调用，与独立 BMI323 ODR 生产任务通过环形缓冲解耦。
 * 线程约束：仅限单一 IMU 管理任务，不可在 ISR 调用；内部可两次永久等待 BMI 数据锁。
 * 无样本路径在 BMI 数据锁内读取驱动在线标志但未获取驱动锁；当前需依赖驱动查询为无阻塞快照。
 */
static void imu_update_bmi323(void)
{
    bmi323_raw_sample_t raw_sample = {0};
    uint64_t capture_timestamp_us = 0U;

    if (imu_bmi_ring_take_latest(&raw_sample, &capture_timestamp_us) == 0U) {
        imu_lock_bmi();
        bmi_data.valid = 0U;
        bmi_data.accel_valid = 0U;
        bmi_data.gyro_valid = 0U;
        if (bmi_data.invalid_count != UINT32_MAX) {
            ++bmi_data.invalid_count;
        }
        if (bmi323_is_online() == 0U) {
            bmi_stats.last_status = BSP_STATUS_NOT_READY;
        }
        imu_unlock_bmi();
        return;
    }

    imu_lock_bmi();
    bmi_data.valid = raw_sample.valid;
    bmi_data.accel_valid = raw_sample.valid;
    bmi_data.gyro_valid = raw_sample.valid;
    bmi_data.accel_x = bmi323_accel_raw_to_mps2(raw_sample.accel[0]);
    bmi_data.accel_y = bmi323_accel_raw_to_mps2(raw_sample.accel[1]);
    bmi_data.accel_z = bmi323_accel_raw_to_mps2(raw_sample.accel[2]);
    bmi_data.gyro_x = bmi323_gyro_raw_to_rads(raw_sample.gyro[0]);
    bmi_data.gyro_y = bmi323_gyro_raw_to_rads(raw_sample.gyro[1]);
    bmi_data.gyro_z = bmi323_gyro_raw_to_rads(raw_sample.gyro[2]);
    bmi_data.timestamp_us = raw_sample.timestamp_us;
    bmi_data.timestamp =
        (uint32_t)(raw_sample.timestamp_us / UINT64_C(1000));
    bmi_data.sample_count = bmi_capture_stats.sample_count;
    bmi_stats.last_status = BSP_STATUS_OK;
    if (capture_timestamp_us != 0U) {
        const uint64_t latency_us =
            imu_time_now_us() - capture_timestamp_us;
        const uint32_t bounded_latency_us = latency_us > UINT32_MAX
                                                ? UINT32_MAX
                                                : (uint32_t)latency_us;
        if (bounded_latency_us > bmi_capture_stats.max_latency_us) {
            bmi_capture_stats.max_latency_us = bounded_latency_us;
        }
    }
    imu_unlock_bmi();
}

#if defined(IMU_MANAGER_USE_FREERTOS)
/**
 * @brief 使用累积相位误差将 BMI323 Hz 采样率转为当次 FreeRTOS 延迟 tick 数。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] sample_rate 数值表示 Hz 的 BMI323 采样率枚举。
 * @param[in,out] phase 调用者所有并保存的累积相位余数；采样率改变后由调用者重置，指针不被保留。
 * @return 当次延迟 tick 数，最小为 1；`phase==NULL` 或采样率为 0 时也返回 1。
 * 调用方式：BMI323 采样任务每次循环末尾把结果传给 `vTaskDelayUntil()`；
 * 例如 1 kHz tick/800 Hz 用 2/1/1/1 tick 节拍近似，而不是每次都截为 1 tick。
 * 线程约束：纯整数计算、不阻塞；`phase` 必须由单一任务独占，函数不适用于 ISR 调度。
 */
static TickType_t imu_bmi323_period_ticks(bmi323_sample_rate_t sample_rate,
                                           uint32_t *phase)
{
    const uint32_t rate_hz = (uint32_t)sample_rate;
    TickType_t delay_ticks;

    if (phase == NULL || rate_hz == 0U) {
        return 1U;
    }
    /* The caller seeds phase with rate_hz - 1, so this computes the
     * ceil-rounded cumulative tick boundary. For 800 Hz on a 1 kHz tick,
     * that produces 2/1/1/1 ticks instead of clamping every cycle to 1 tick. */
    *phase += (uint32_t)configTICK_RATE_HZ;
    delay_ticks = (TickType_t)(*phase / rate_hz);
    *phase %= rate_hz;
    return delay_ticks == 0U ? 1U : delay_ticks;
}

/**
 * @brief BMI323 独立 ODR 采样任务，零等待争用 SPI 驱动锁并分发标定、环形缓冲和双 AHRS 输入。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] argument FreeRTOS 任务参数，当前忽略，允许为 `NULL`；函数不解引用、不保留也不承接所有权。
 * @return 无正常返回；任务在永久循环中运行，采集禁用时按 10 ms 周期休眠。
 * 读成功时以 SPI 读取起止时间中点作为捕获时间；DATA_NOT_READY 保留旧姿态供新鲜度超时，
 * 真实读错误或驱动锁竞争则向双 AHRS 提交无效 BMI 样本。
 * 调用方式：由 `imu_manager_start_bmi323_task()` 创建且只能创建一次。
 * 线程约束：FreeRTOS 永久任务，不可直接从 ISR 调用；驱动锁采用零等待，
 * 成功获锁后 SPI 原始读取可阻塞。每个样本执行多次浮点单位转换和双 AHRS 更新，应监测 384 word 任务栈与 WCET。
 */
static void imu_bmi323_task(void *argument)
{
    bmi323_sample_rate_t previous_rate = bmi323_get_sample_rate();
    uint32_t phase = (uint32_t)previous_rate - 1U;
    TickType_t last_wake = xTaskGetTickCount();

    (void)argument;
    for (;;) {
        bmi323_raw_sample_t sample = {0};
        bool read_ok = false;
        bmi323_diag_status_t read_status = BMI323_DIAG_STATUS_OK;
        uint64_t read_start_us = 0U;
        uint64_t read_end_us = 0U;
        uint64_t capture_timestamp_us;

        if (bmi323_acquisition_enabled == 0U) {
            vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(IMU_TASK_PERIOD_MS));
            continue;
        }

        if (imu_try_lock_bmi_driver() != 0U) {
            read_start_us = imu_time_now_us();
            read_ok = bmi323_is_online() != 0U &&
                      bmi323_read_raw_sample(sample.accel, sample.gyro);
            read_status = bmi323_is_online() == 0U
                              ? BMI323_DIAG_STATUS_DATA_NOT_READY
                              : bmi323_get_status();
            read_end_us = imu_time_now_us();
            imu_unlock_bmi_driver();
            capture_timestamp_us = read_start_us +
                                   ((read_end_us - read_start_us) / 2U);
            if (read_ok) {
                sample.timestamp_us = capture_timestamp_us;
                sample.valid = 1U;
                if (imu_calibration_bmi_capture_active() != 0U) {
                    imu_calibration_update_bmi323(
                        bmi323_accel_raw_to_mps2(sample.accel[0]),
                        bmi323_accel_raw_to_mps2(sample.accel[1]),
                        bmi323_accel_raw_to_mps2(sample.accel[2]),
                        bmi323_gyro_raw_to_rads(sample.gyro[0]),
                        bmi323_gyro_raw_to_rads(sample.gyro[1]),
                        bmi323_gyro_raw_to_rads(sample.gyro[2]),
                        sample.timestamp_us);
                }
                imu_bmi_ring_push(&sample);
                {
                    const bmi323_data_t dual_sample = {
                        .accel_x = bmi323_accel_raw_to_mps2(sample.accel[0]),
                        .accel_y = bmi323_accel_raw_to_mps2(sample.accel[1]),
                        .accel_z = bmi323_accel_raw_to_mps2(sample.accel[2]),
                        .gyro_x = bmi323_gyro_raw_to_rads(sample.gyro[0]),
                        .gyro_y = bmi323_gyro_raw_to_rads(sample.gyro[1]),
                        .gyro_z = bmi323_gyro_raw_to_rads(sample.gyro[2]),
                        .timestamp = (uint32_t)(capture_timestamp_us /
                                                UINT64_C(1000)),
                        .timestamp_us = capture_timestamp_us,
                        .valid = 1U,
                        .accel_valid = 1U,
                        .gyro_valid = 1U,
                    };
                    imu_dual_ahrs_feed_bmi(&dual_sample);
                }
            } else if (read_status == BMI323_DIAG_STATUS_DATA_NOT_READY) {
                /* No data-ready edge means the sensor still owns the last
                 * sample. Leave the previous estimate intact and let its
                 * freshness deadline decide whether motion must stop. */
            } else {
                const bmi323_data_t dual_sample = {
                    .timestamp = (uint32_t)(capture_timestamp_us /
                                            UINT64_C(1000)),
                    .timestamp_us = capture_timestamp_us,
                    .valid = 0U,
                    .accel_valid = 0U,
                    .gyro_valid = 0U,
                };
                imu_bmi_capture_failed();
                imu_dual_ahrs_feed_bmi(&dual_sample);
            }
        } else {
            /* Recovery or ODR reconfiguration owns the driver briefly. Do not
             * delay the high-rate producer; record the skipped interval. */
            imu_bmi_capture_note_contention_drop();
            capture_timestamp_us = imu_time_now_us();
            {
                const bmi323_data_t dual_sample = {
                    .timestamp = (uint32_t)(capture_timestamp_us /
                                            UINT64_C(1000)),
                    .timestamp_us = capture_timestamp_us,
                    .valid = 0U,
                    .accel_valid = 0U,
                    .gyro_valid = 0U,
                };
                imu_dual_ahrs_feed_bmi(&dual_sample);
            }
        }

        const bmi323_sample_rate_t current_rate = bmi323_get_sample_rate();
        if (current_rate != previous_rate) {
            previous_rate = current_rate;
            phase = (uint32_t)current_rate - 1U;
        }
        vTaskDelayUntil(&last_wake,
                        imu_bmi323_period_ticks(current_rate, &phase));
    }
}
#endif

#if defined(IMU_MANAGER_USE_FREERTOS)
/**
 * @brief 等待公共启动通知后初始化 LSM303，提交 worker 结果并自删除。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] argument FreeRTOS 任务参数，当前忽略，允许为 `NULL`；函数不解引用、不保留也不承接所有权。
 * @return 无正常返回；收到通知后执行一次初始化，将自身句柄清空后调用 `vTaskDelete(NULL)`。
 * 调用方式：由双 IMU 初始化入口创建，待 LSM/BMI 两个 worker 都创建成功后通知释放。
 * 线程约束：FreeRTOS 一次性任务，起始可按 `portMAX_DELAY` 永久等待通知；
 * `lsm303_init()` 可执行阻塞 I2C/延时，结果提交又可永久等待 LSM 数据锁和双初始化锁，不可在 ISR 调用。
 */
static void imu_dual_lsm_init_task(void *argument)
{
    bsp_status_t status;

    (void)argument;
    /* The caller creates both workers before releasing either notification.
     * A task notification remains pending if this task has not run yet, so
     * INIT does not depend on the caller's priority relative to these workers. */
    (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
#if SMARTCAR_BMI323_DEBUG_ONLY
    status = BSP_STATUS_UNSUPPORTED;
#else
    imu_init_log("LSM303_INIT_BEGIN\r\n");
    status = lsm303_init();
    imu_init_log_status("LSM303_INIT_END", status);
#endif
    imu_mark_lsm_initialized(status);

    imu_lock_dual_init();
    dual_init_status.lsm_complete = 1U;
    dual_init_status.lsm_success = status == BSP_STATUS_OK ? 1U : 0U;
    dual_init_status.lsm_end_time = imu_time_now_ms();
    dual_lsm_init_task_handle = NULL;
    imu_unlock_dual_init();
    vTaskDelete(NULL);
}

/**
 * @brief 等待公共启动通知后在驱动锁内初始化 BMI323，记录结果并自删除。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] argument FreeRTOS 任务参数，当前忽略，允许为 `NULL`；函数不解引用、不保留也不承接所有权。
 * @return 无正常返回；提交初始化结果、输出诊断、清空自身句柄后调用 `vTaskDelete(NULL)`。
 * 调用方式：由双 IMU 初始化入口创建，待两个 worker 都创建成功后与 LSM worker 同时通知释放。
 * 线程约束：FreeRTOS 一次性任务，起始可永久等待通知；初始化期间永久等待并持有 BMI 驱动锁，
 * 驱动内部可阻塞 SPI/延时，后续还获取 BMI 数据锁和双初始化锁并输出多条日志，不可在 ISR 调用。
 */
static void imu_dual_bmi_init_task(void *argument)
{
    bool initialized;

    (void)argument;
    /* See imu_dual_lsm_init_task(): both buses leave the start gate together. */
    (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    imu_lock_bmi_driver();
    initialized = bmi323_init();
    imu_unlock_bmi_driver();
    imu_mark_bmi323_initialized(initialized ? 1U : 0U);
    imu_bmi323_init_log();

    imu_lock_dual_init();
    dual_init_status.bmi_complete = 1U;
    dual_init_status.bmi_success = initialized ? 1U : 0U;
    dual_init_status.bmi_end_time = imu_time_now_ms();
    dual_bmi_init_task_handle = NULL;
    imu_unlock_dual_init();
    vTaskDelete(NULL);
}
#endif

/** 启动 LSM303/BMI323 两条独立初始化 worker。 */
bsp_status_t imu_manager_start_dual_initialization(void)
{
#if defined(IMU_MANAGER_USE_FREERTOS)
    BaseType_t lsm_task_status;
    BaseType_t bmi_task_status;
    const uint32_t now_ms = imu_time_now_ms();

    if (imu_prepared == 0U || imu_create_data_locks() != BSP_STATUS_OK) {
        return BSP_STATUS_NOT_READY;
    }
#if SMARTCAR_BMI323_DEBUG_ONLY
    return BSP_STATUS_UNSUPPORTED;
#else
    imu_lock_dual_init();
    if (dual_init_status.lsm_started != 0U || dual_init_status.bmi_started != 0U) {
        const uint8_t complete = dual_init_status.lsm_complete != 0U &&
                                 dual_init_status.bmi_complete != 0U;
        const uint8_t success = dual_init_status.lsm_success != 0U &&
                                dual_init_status.bmi_success != 0U;
        imu_unlock_dual_init();
        return complete != 0U && success != 0U ? BSP_STATUS_OK : BSP_STATUS_NOT_READY;
    }
    dual_init_status = (imu_dual_init_status_t){
        .lsm_started = 1U,
        .bmi_started = 1U,
        .lsm_start_time = now_ms,
        .bmi_start_time = now_ms,
    };
    imu_unlock_dual_init();

    /* Stop the independent producer before either driver is reset. */
    bmi323_acquisition_enabled = 0U;
    imu_initialized = 0U;
    imu_reset_data(1U);

    lsm_task_status = xTaskCreate(imu_dual_lsm_init_task, "imu_lsm_init",
                                  DUAL_IMU_INIT_TASK_STACK_WORDS, NULL,
                                  DUAL_IMU_INIT_TASK_PRIORITY,
                                  &dual_lsm_init_task_handle);
    if (lsm_task_status != pdPASS) {
        imu_lock_dual_init();
        dual_init_status.lsm_complete = 1U;
        dual_init_status.bmi_complete = 1U;
        dual_init_status.lsm_success = 0U;
        dual_init_status.bmi_success = 0U;
        dual_init_status.lsm_end_time = now_ms;
        dual_init_status.bmi_end_time = now_ms;
        imu_unlock_dual_init();
        return BSP_STATUS_ERROR;
    }
    bmi_task_status = xTaskCreate(imu_dual_bmi_init_task, "imu_bmi_init",
                                  DUAL_IMU_INIT_TASK_STACK_WORDS, NULL,
                                  DUAL_IMU_INIT_TASK_PRIORITY,
                                  &dual_bmi_init_task_handle);
    if (bmi_task_status != pdPASS) {
        vTaskDelete(dual_lsm_init_task_handle);
        dual_lsm_init_task_handle = NULL;
        imu_lock_dual_init();
        dual_init_status.lsm_complete = 1U;
        dual_init_status.bmi_complete = 1U;
        dual_init_status.lsm_success = 0U;
        dual_init_status.bmi_success = 0U;
        dual_init_status.lsm_end_time = now_ms;
        dual_init_status.bmi_end_time = now_ms;
        imu_unlock_dual_init();
        return BSP_STATUS_ERROR;
    }
    /* Release both workers while scheduling is suspended. This is a real
     * common start gate, independent of task priority or creation order. */
    vTaskSuspendAll();
    (void)xTaskNotifyGive(dual_lsm_init_task_handle);
    (void)xTaskNotifyGive(dual_bmi_init_task_handle);
    (void)xTaskResumeAll();
    return BSP_STATUS_OK;
#endif
#else
    return BSP_STATUS_UNSUPPORTED;
#endif
}

/** 复制双初始化 worker 的进度和结果。 */
void imu_manager_get_dual_initialization_status(imu_dual_init_status_t *status)
{
    if (status == NULL) {
        return;
    }
    imu_lock_dual_init();
    *status = dual_init_status;
    imu_unlock_dual_init();
}

/** 汇总两个 worker 的结果并提交初始化屏障。 */
uint8_t imu_manager_finalize_dual_initialization(void)
{
    imu_dual_init_status_t status = {0};

    imu_manager_get_dual_initialization_status(&status);
    if (status.lsm_complete == 0U || status.bmi_complete == 0U ||
        status.lsm_success == 0U || status.bmi_success == 0U) {
        return 0U;
    }
    /* Primary AHRS is designed for the fixed 200 Hz BMI323 input stream.
     * Apply the ODR after hardware init so a previous runtime-rate request
     * cannot silently desynchronize the estimator and its anti-alias filter. */
    if (imu_manager_set_bmi323_sample_rate(BMI323_SAMPLE_RATE_200HZ) !=
        BSP_STATUS_OK) {
        return 0U;
    }
    imu_initialized = 1U;
    bmi323_acquisition_enabled = 1U;
    if (imu_manager_start_bmi323_task() != BSP_STATUS_OK) {
        bmi323_acquisition_enabled = 0U;
        imu_initialized = 0U;
        return 0U;
    }
    return 1U;
}

/** 设置 BMI323 独立采样任务的数据率。 */
bsp_status_t imu_manager_set_bmi323_sample_rate(bmi323_sample_rate_t sample_rate)
{
    uint8_t success;

    /* The active Primary AHRS filter is designed for BMI323 ODR=200 Hz.
     * Reject other manager-level rates instead of allowing a silent LPF and
     * decimation mismatch. The low-level driver enum remains available for
     * isolated diagnostics that do not feed DualAHRS. */
    if (sample_rate != BMI323_SAMPLE_RATE_200HZ) {
        return BSP_STATUS_INVALID_ARG;
    }
    if (imu_create_data_locks() != BSP_STATUS_OK) {
        return BSP_STATUS_ERROR;
    }
    imu_lock_bmi_driver();
    success = bmi323_set_sample_rate(sample_rate) ? 1U : 0U;
    imu_unlock_bmi_driver();
    if (success == 0U) {
        return BSP_STATUS_INVALID_ARG;
    }
    imu_lock_bmi();
    imu_bmi_ring_reset_locked();
    imu_unlock_bmi();
    return BSP_STATUS_OK;
}

/** 读取 BMI323 当前数据率配置。 */
bmi323_sample_rate_t imu_manager_get_bmi323_sample_rate(void)
{
    return bmi323_get_sample_rate();
}

/** 复制 BMI323 原始捕获统计。 */
bsp_status_t imu_manager_get_bmi323_capture_stats(bmi323_capture_stat_t *stats)
{
    if (stats == NULL) {
        return BSP_STATUS_INVALID_ARG;
    }
    if (imu_initialized == 0U) {
        return BSP_STATUS_NOT_READY;
    }
    imu_lock_bmi();
    *stats = bmi_capture_stats;
    stats->overflow_count = bmi_ring_buffer.overflow_count;
    stats->contention_drop_count = bmi_capture_contention_drop_count;
    stats->pending_count = bmi_ring_buffer.count;
    stats->configured_rate_hz = (uint16_t)bmi323_get_sample_rate();
    stats->measured_rate_hz = imu_bmi_measured_rate_hz(stats);
    imu_unlock_bmi();
    return BSP_STATUS_OK;
}

/** 创建独立 BMI323 采样任务。 */
bsp_status_t imu_manager_start_bmi323_task(void)
{
#if defined(IMU_MANAGER_USE_FREERTOS)
    BaseType_t task_status;

    if (imu_initialized == 0U) {
        return BSP_STATUS_NOT_READY;
    }
    if (bmi323_task_started != 0U) {
        return BSP_STATUS_OK;
    }
    task_status = xTaskCreate(imu_bmi323_task, "bmi323_task",
                              BMI323_TASK_STACK_WORDS, NULL,
                              BMI323_TASK_PRIORITY, &bmi323_task_handle);
    if (task_status != pdPASS) {
        return BSP_STATUS_ERROR;
    }
    bmi323_task_started = 1U;
    return BSP_STATUS_OK;
#else
    return BSP_STATUS_UNSUPPORTED;
#endif
}

/**
 * @brief 准备双 IMU 生命周期：禁用 BMI 采集、清理快照/水平/初始化状态并初始化标定、滤波与启动管理器。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] reset_count 透传给 `imu_reset_data()`；非 0 时同时清零 BMI/LSM 历史计数。
 * @return 任一互斥量创建失败返回 `BSP_STATUS_ERROR`；否则返回 `BSP_STATUS_OK`。
 * 标定/滤波/磁力计/启动管理器初始化 API 均无失败返回通道，因此 OK 不代表传感器硬件已初始化。
 * 调用方式：正常 `imu_init()` 与 `imu_recover()` 在时钟初始化成功后调用；硬件初始化由后续双 worker 独占。
 * 线程约束：仅限串行生命周期任务，不可在 ISR 调用；需先禁用高频 BMI 生产者，
 * 内部可永久等待 BMI/LSM/快照/双初始化锁，并会重置姿态、标定、滤波与磁力计全局状态。
 */
static bsp_status_t imu_prepare_lifecycle(uint8_t reset_count)
{
    if (imu_create_data_locks() != BSP_STATUS_OK) {
        return BSP_STATUS_ERROR;
    }

    /* Hardware drivers are deliberately not called here. DUAL_IMU_BOOT/INIT
     * owns both worker launches and prevents either sensor from starting a
     * later lifecycle phase by itself. */
    bmi323_acquisition_enabled = 0U;
    imu_initialized = 0U;
    dual_ahrs_bias_injected = 0U;
    imu_manager_reset_leveling();
    imu_reset_data(reset_count);
    imu_lock_dual_init();
    dual_init_status = (imu_dual_init_status_t){0};
    imu_unlock_dual_init();
    imu_prepared = 1U;

#if !SMARTCAR_BMI323_DEBUG_ONLY
    imu_calibration_init();
    imu_filter_init();
    mag_filter_init();
    imu_boot_manager_init();
#endif
    return BSP_STATUS_OK;
}

#if SMARTCAR_BMI323_DEBUG_ONLY
/**
 * @brief 在 BMI323 独立调试构建中直接初始化 BMI323，提交诊断状态并打开采集门。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * 传入参数：无。
 * @return 互斥量创建失败或 BMI323 初始化失败返回 `BSP_STATUS_ERROR`；驱动初始化成功返回 `BSP_STATUS_OK`。
 * 调用方式：仅在 `SMARTCAR_BMI323_DEBUG_ONLY=1` 时由 `imu_init()` 和 `imu_recover()` 调用。
 * 线程约束：仅限串行初始化/恢复任务，不可在 ISR 调用；内部永久等待 BMI 驱动锁，
 * `bmi323_init()` 可阻塞 SPI/延时，后续可获取 BMI 数据锁并输出多条日志。
 */
static bsp_status_t imu_init_bmi_debug(void)
{
    bool initialized;

    if (imu_create_data_locks() != BSP_STATUS_OK) {
        return BSP_STATUS_ERROR;
    }
    imu_reset_data(1U);
    imu_manager_reset_leveling();
    imu_lock_bmi_driver();
    initialized = bmi323_init();
    imu_unlock_bmi_driver();
    imu_mark_bmi323_initialized(initialized ? 1U : 0U);
    imu_bmi323_init_log();
    imu_prepared = 1U;
    imu_initialized = initialized ? 1U : 0U;
    bmi323_acquisition_enabled = initialized ? 1U : 0U;
    return initialized ? BSP_STATUS_OK : BSP_STATUS_ERROR;
}
#endif

/** 初始化两路 IMU 驱动和生命周期状态。 */
bsp_status_t imu_init(void)
{
    if (imu_time_init() != BSP_STATUS_OK) {
        return BSP_STATUS_ERROR;
    }
#if SMARTCAR_BMI323_DEBUG_ONLY
    return imu_init_bmi_debug();
#else
    return imu_prepare_lifecycle(1U);
#endif
}

/** 复位 IMU 运行状态并重试初始化。 */
bsp_status_t imu_recover(void)
{
    if (imu_time_init() != BSP_STATUS_OK) {
        return BSP_STATUS_ERROR;
    }
#if SMARTCAR_BMI323_DEBUG_ONLY
    return imu_init_bmi_debug();
#else
    if (imu_prepare_lifecycle(1U) != BSP_STATUS_OK) {
        return BSP_STATUS_ERROR;
    }
    imu_init_log("[IMU_RECOVER] dual lifecycle reset\r\n");
    return BSP_STATUS_OK;
#endif
}

/** 读取一次传感器并刷新统一原始快照。 */
bsp_status_t imu_update(void)
{
#if SMARTCAR_BMI323_DEBUG_ONLY
    if (imu_initialized == 0U) {
        return BSP_STATUS_NOT_READY;
    }
    /* Keep BMI323 acquisition isolated from LSM303 and the AHRS path. */
    imu_update_bmi323();
    imu_publish_unified_snapshot();
    return BSP_STATUS_OK;
#else
    bsp_status_t lsm_status;

    if (imu_prepared == 0U) {
        return BSP_STATUS_NOT_READY;
    }
    if (imu_initialized == 0U) {
        /* INIT workers are running or the lifecycle is terminal. Returning
         * success avoids a legacy recovery loop from starting a sequential
         * initialization path before DUAL_IMU_BOOT decides the outcome. */
        return BSP_STATUS_OK;
    }

    lsm_status = imu_update_lsm303();
    imu_update_bmi323();
    imu_publish_unified_snapshot();
    return lsm_status;
#endif
}

/** 复制最新 BMI323 数据。 */
bsp_status_t imu_get_bmi323_data(bmi323_data_t *data)
{
    if (data == NULL) {
        return BSP_STATUS_INVALID_ARG;
    }
    if (imu_initialized == 0U) {
        return BSP_STATUS_NOT_READY;
    }

    imu_lock_bmi();
    *data = bmi_data;
    imu_unlock_bmi();
    return BSP_STATUS_OK;
}

/** 复制最新 LSM303 加速度数据。 */
bsp_status_t imu_manager_get_lsm_accel(lsm_accel_data_t *data)
{
    if (data == NULL) {
        return BSP_STATUS_INVALID_ARG;
    }
    if (imu_initialized == 0U) {
        return BSP_STATUS_NOT_READY;
    }

    imu_lock_lsm();
    *data = lsm_accel_data;
    imu_unlock_lsm();
    return BSP_STATUS_OK;
}

/** 复制最新 LSM303 磁场数据。 */
bsp_status_t imu_manager_get_lsm_mag(lsm_mag_data_t *data)
{
    if (data == NULL) {
        return BSP_STATUS_INVALID_ARG;
    }
    if (imu_initialized == 0U) {
        return BSP_STATUS_NOT_READY;
    }

    imu_lock_lsm();
    *data = lsm_mag_data;
    imu_unlock_lsm();
    return BSP_STATUS_OK;
}

bsp_status_t imu_manager_get_snapshot(imu_raw_data_t *snapshot)
{
    if (snapshot == NULL) {
        return BSP_STATUS_INVALID_ARG;
    }
    if (imu_initialized == 0U) {
        return BSP_STATUS_NOT_READY;
    }

    imu_lock_snapshot();
    *snapshot = imu_snapshot;
    imu_unlock_snapshot();
    return BSP_STATUS_OK;
}

bsp_status_t imu_get_bmi323_stats(imu_sensor_stats_t *stats)
{
    if (stats == NULL) {
        return BSP_STATUS_INVALID_ARG;
    }
    if (imu_initialized == 0U) {
        return BSP_STATUS_NOT_READY;
    }

    imu_lock_bmi();
    *stats = bmi_stats;
    imu_unlock_bmi();
    return BSP_STATUS_OK;
}

bsp_status_t imu_get_lsm303_stats(imu_sensor_stats_t *stats)
{
    if (stats == NULL) {
        return BSP_STATUS_INVALID_ARG;
    }
    if (imu_initialized == 0U) {
        return BSP_STATUS_NOT_READY;
    }

    imu_lock_lsm();
    *stats = lsm_stats;
    imu_unlock_lsm();
    return BSP_STATUS_OK;
}

/** 执行一次 IMU 任务迭代。 */
bsp_status_t imu_task_step(void)
{
    return imu_update();
}

/** FreeRTOS IMU 任务入口。 */
void imu_task(void *argument)
{
    (void)argument;
    imu_init_log("[INFO] IMU_TASK_RUN\r\n");
#if defined(IMU_MANAGER_USE_FREERTOS)
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(IMU_TASK_PERIOD_MS);
    uint32_t last_recovery_ms = imu_time_now_ms();
    uint32_t last_health_ms = last_recovery_ms;
    for (;;) {
#if !SMARTCAR_BMI323_DEBUG_ONLY
        imu_boot_manager_step();
#endif
        const bsp_status_t status = imu_task_step();
        const uint32_t now_ms = imu_time_now_ms();
        if (status != BSP_STATUS_OK &&
#if !SMARTCAR_BMI323_DEBUG_ONLY
            imu_boot_manager_is_ready() != 0U &&
#endif
            (uint32_t)(now_ms - last_recovery_ms) >= IMU_RECOVERY_PERIOD_MS) {
            last_recovery_ms = now_ms;
            (void)imu_recover();
        }
        if ((uint32_t)(now_ms - last_health_ms) >= IMU_LSM303_HEALTH_PERIOD_MS) {
            last_health_ms = now_ms;
#if !SMARTCAR_BMI323_DEBUG_ONLY
            imu_lsm_health_log(now_ms);
#endif
            imu_bmi323_debug_log();
        }
        vTaskDelayUntil(&last_wake, period);
    }
#else
    uint32_t last_health_ms = imu_time_now_ms();
    for (;;) {
        const uint32_t now_ms = imu_time_now_ms();
        (void)imu_task_step();
        if ((uint32_t)(now_ms - last_health_ms) >= IMU_LSM303_HEALTH_PERIOD_MS) {
            last_health_ms = now_ms;
#if !SMARTCAR_BMI323_DEBUG_ONLY
            imu_lsm_health_log(now_ms);
#endif
            imu_bmi323_debug_log();
        }
        imu_delay_ms(IMU_TASK_PERIOD_MS);
    }
#endif
}

/** 查询双 IMU 初始化屏障是否完成。 */
uint8_t imu_is_ready(void)
{
    if (imu_initialized == 0U) {
        return 0U;
    }
#if SMARTCAR_BMI323_DEBUG_ONLY
    return bmi323_init_success != 0U && bmi323_is_online() != 0U ? 1U : 0U;
#else
    return imu_lsm_is_online();
#endif
}

/** 查询 LSM303 初始化结果。 */
uint8_t imu_manager_get_lsm303_init_success(void)
{
    uint8_t initialized;

    imu_lock_lsm();
    initialized = lsm303_init_success;
    imu_unlock_lsm();
    return initialized;
}

/** 查询 LSM303 最近数据在线状态。 */
uint8_t imu_manager_get_lsm303_online(void)
{
    return imu_lsm_is_online();
}
