#include "imu_calibration.h"

/* 双 IMU 标定实现；创建人：待确认（当前维护人：Zhiqin）。 */

#include <float.h>
#include <math.h>
#include <string.h>

#include "imu_time.h"

/* All calibration accumulators store SI units. BMI323 is configured for the
 * +/-4 g range, where one raw LSB is 9.80665/8192 m/s^2. The active manager
 * already performs this conversion; the bounded check below only protects
 * this API when a raw BMI323 sample is supplied by an alternate caller. */
#define IMU_CALIBRATION_GRAVITY_MPS2 (9.80665f)
#define IMU_CALIBRATION_BMI323_LSB_PER_G (8192.0f)
#define IMU_CALIBRATION_RAW_ACCEL_THRESHOLD_MPS2 (100.0f)
#define IMU_CALIBRATION_GYRO_MOTION_THRESHOLD_RADPS (0.15f)
#define IMU_CALIBRATION_ACCEL_MOTION_DELTA_MPS2 (1.0f)

#if defined(IMU_MANAGER_USE_FREERTOS)
#include "FreeRTOS.h"
#include "semphr.h"
#endif

typedef struct
{
    double sum_x;
    double sum_y;
    double sum_z;
    double sum_square_x;
    double sum_square_y;
    double sum_square_z;
    uint32_t sample_count;
} imu_axis_accumulator_t;

typedef struct
{
    imu_axis_accumulator_t lsm_accel;
    imu_axis_accumulator_t bmi_accel;
    imu_axis_accumulator_t bmi_gyro;
    imu_calibration_bias_t bias;
    imu_calibration_result_t result;
    imu_calibrated_data_t last_calibrated;
    uint64_t window_start_timestamp_us;
    uint64_t last_lsm_accel_timestamp_us;
    uint16_t bmi_configured_rate_hz;
    uint8_t window_active;
    uint8_t complete;
    uint8_t lsm_observed;
    uint8_t bmi_accel_observed;
    uint8_t bmi_gyro_observed;
    uint8_t static_motion_detected;
    imu_calibration_quality_t quality;
    imu_calibration_static_statistics_t static_statistics;
} imu_calibration_state_t;

static imu_calibration_state_t s_calibration;
static volatile uint8_t s_bmi_capture_active;

#if defined(IMU_MANAGER_USE_FREERTOS)
static SemaphoreHandle_t s_mutex;
#endif

/**
 * @brief 获取标定互斥量，保护累计器、质量数据和冻结结果。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * 传入参数：无。
 * @return 无返回值；非 FreeRTOS 构建或互斥量未创建时直接返回。
 * 调用方式：低频管理路径读写 `s_calibration` 前调用，并与 `unlock_calibration()` 成对。
 * 线程约束：仅限任务上下文；FreeRTOS 下可按 `portMAX_DELAY` 永久阻塞，
 * 不可在 ISR 调用或在同一任务重入获取非递归互斥量。
 */
static void lock_calibration(void)
{
#if defined(IMU_MANAGER_USE_FREERTOS)
    if (s_mutex != NULL) {
        (void)xSemaphoreTake(s_mutex, portMAX_DELAY);
    }
#endif
}

/**
 * @brief 释放由当前任务持有的标定互斥量。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * 传入参数：无。
 * @return 无返回值；非 FreeRTOS 构建或互斥量未创建时直接返回。
 * 调用方式：仅在成功获取标定锁的路径末尾成对调用。
 * 线程约束：仅限当前持锁任务，不可在 ISR 中调用；释放路径不等待、正常情况下不阻塞，且不检查所有权。
 */
static void unlock_calibration(void)
{
#if defined(IMU_MANAGER_USE_FREERTOS)
    if (s_mutex != NULL) {
        (void)xSemaphoreGive(s_mutex);
    }
#endif
}

/**
 * @brief 以零等待方式尝试获取标定互斥量。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * 传入参数：无。
 * @return FreeRTOS 下获锁成功返回 1，锁未创建或已被占用返回 0；非 FreeRTOS 构建固定返回 1。
 * 调用方式：BMI323 ODR 捕获路径在写入累计器前调用，失败时丢弃当前样本。
 * 线程约束：任务上下文零等待、不阻塞；不使用 `FromISR` API，因此不可在 ISR 调用。
 */
static uint8_t try_lock_calibration(void)
{
#if defined(IMU_MANAGER_USE_FREERTOS)
    if (s_mutex == NULL) {
        return 0U;
    }
    return xSemaphoreTake(s_mutex, (TickType_t)0) == pdTRUE ? 1U : 0U;
#else
    return 1U;
#endif
}

/**
 * @brief 判断三轴浮点分量是否全部为有限数。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] x X 轴分量。
 * @param[in] y Y 轴分量。
 * @param[in] z Z 轴分量。
 * @return 三个分量均非 NaN/无穷时返回 1，否则返回 0。
 * 调用方式：单位转换和累计前执行输入有效性检查。
 * 线程约束：纯计算、可重入、不阻塞，不访问共享状态；仅在工程允许 ISR 使用浮点上下文时可从 ISR 调用。
 */
static uint8_t finite_xyz(float x, float y, float z)
{
    return isfinite(x) && isfinite(y) && isfinite(z) ? 1U : 0U;
}

/**
 * @brief 必要时将 BMI323 三轴加速度从 +/-4 g 原始 LSB 原地转为 m/s^2。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in,out] accel 至少 3 个 `float` 的可写数组；分量非有限或指针为 `NULL` 时保持不变。
 * @return 无返回值；函数不显式报告是否发生了单位转换。
 * 调用方式：BMI323 标定捕获将输入复制到局部数组后调用；
 * 只有最大绝对分量大于 100 时按 LSB 转换，当前管理器通常已提供 SI 单位。
 * 线程约束：只修改调用者所有数组、不阻塞；并发所有权由调用者保证，仅在已保存 FPU 上下文时可从 ISR 调用。
 */
static void bmi323_accel_input_to_mps2(float accel[3])
{
    const float scale = IMU_CALIBRATION_GRAVITY_MPS2 /
                        IMU_CALIBRATION_BMI323_LSB_PER_G;

    if (accel == NULL || finite_xyz(accel[0], accel[1], accel[2]) == 0U) {
        return;
    }
    /* Detect units from the complete vector so a near-zero raw axis is never
     * left in LSB while the gravity-bearing axes are converted to m/s^2. */
    if (fmaxf(fabsf(accel[0]), fmaxf(fabsf(accel[1]), fabsf(accel[2]))) >
        IMU_CALIBRATION_RAW_ACCEL_THRESHOLD_MPS2) {
        accel[0] *= scale;
        accel[1] *= scale;
        accel[2] *= scale;
    }
}

/**
 * @brief 检查将一个值加入现有双精度和值后是否仍为有限数。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] sum 当前累计和。
 * @param[in] value 待累加值。
 * @return `sum`、`value` 及两者之和均为有限数时返回 1，否则返回 0。
 * 调用方式：`accumulate_xyz()` 更新三轴和值前逐轴调用。
 * 线程约束：纯计算、可重入、不阻塞；仅在已保存 FPU 上下文时可从 ISR 调用。
 */
static uint8_t sum_add_is_safe(double sum, double value)
{
    return isfinite(sum) && isfinite(value) && isfinite(sum + value) ? 1U : 0U;
}

/**
 * @brief 检查将样本平方加入平方和时是否有效且不溢出。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] sum_square 当前非负平方和。
 * @param[in] value 待平方后累加的样本值。
 * @return 输入有限、平方和非负且 `value^2` 不会超过 `DBL_MAX - sum_square` 时返回 1，否则返回 0。
 * 调用方式：`accumulate_xyz()` 更新三轴平方和前逐轴调用。
 * 线程约束：纯计算、可重入、不阻塞；仅在已保存 FPU 上下文时可从 ISR 调用。
 */
static uint8_t sum_square_add_is_safe(double sum_square, double value)
{
    const double square = value * value;

    if (!isfinite(sum_square) || sum_square < 0.0 || !isfinite(square)) {
        return 0U;
    }
    return square <= (DBL_MAX - sum_square) ? 1U : 0U;
}

/**
 * @brief 将一组三轴样本累加到和、平方和与样本计数，并拒绝无效或溢出输入。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in,out] accumulator 由调用者所有并持锁保护的三轴累计器，不得为 `NULL`；所有权不转移。
 * @param[in] x X 轴 SI 单位样本。
 * @param[in] y Y 轴 SI 单位样本。
 * @param[in] z Z 轴 SI 单位样本。
 * @return 全部字段成功累加返回 1；空指针、计数饱和、非有限数或任一累加风险返回 0 且不修改累计器。
 * 调用方式：LSM303 管理路径和 BMI323 ODR 捕获路径在样本落入静态窗口时调用。
 * 线程约束：函数不自行加锁、不阻塞；调用者必须已持有标定锁或处于非 RTOS 串行环境，不可独立从 ISR 调用。
 */
static uint8_t accumulate_xyz(imu_axis_accumulator_t *accumulator,
                              float x, float y, float z)
{
    const double value_x = (double)x;
    const double value_y = (double)y;
    const double value_z = (double)z;

    if (accumulator == NULL || accumulator->sample_count == UINT32_MAX ||
        finite_xyz(x, y, z) == 0U ||
        sum_add_is_safe(accumulator->sum_x, value_x) == 0U ||
        sum_add_is_safe(accumulator->sum_y, value_y) == 0U ||
        sum_add_is_safe(accumulator->sum_z, value_z) == 0U ||
        sum_square_add_is_safe(accumulator->sum_square_x, value_x) == 0U ||
        sum_square_add_is_safe(accumulator->sum_square_y, value_y) == 0U ||
        sum_square_add_is_safe(accumulator->sum_square_z, value_z) == 0U) {
        return 0U;
    }
    accumulator->sum_x += value_x;
    accumulator->sum_y += value_y;
    accumulator->sum_z += value_z;
    accumulator->sum_square_x += value_x * value_x;
    accumulator->sum_square_y += value_y * value_y;
    accumulator->sum_square_z += value_z * value_z;
    ++accumulator->sample_count;
    return 1U;
}

/**
 * @brief 根据三轴累计和计算算术平均值。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] accumulator 仅在调用期间借用的累计器快照。
 * @return 有样本时返回 X/Y/Z 平均值；指针为 `NULL` 或样本数为 0 时返回全零结构。
 * 调用方式：静态窗口质量通过后计算 BMI323 陀螺仪偏置。
 * 线程约束：纯计算、不阻塞；当前仅在持有标定锁的任务中读取共享累计器，不可从 ISR 调用。
 */
static imu_bias_xyz_t mean_xyz(const imu_axis_accumulator_t *accumulator)
{
    imu_bias_xyz_t result = {0};
    if (accumulator == NULL || accumulator->sample_count == 0U) {
        return result;
    }
    const double count = (double)accumulator->sample_count;
    result.x = (float)(accumulator->sum_x / count);
    result.y = (float)(accumulator->sum_y / count);
    result.z = (float)(accumulator->sum_z / count);
    return result;
}

/**
 * @brief 按配置频率与窗口时长计算期望样本数。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] configured_rate_hz 传感器配置采样率，单位 Hz。
 * @param[in] duration_us 统计窗口时长，单位 us。
 * @return 整数除法向下取整的样本数，超过 `UINT32_MAX` 时饱和为 `UINT32_MAX`。
 * 调用方式：质量统计根据标定窗口宏和配置 ODR 调用。
 * 线程约束：纯计算、可重入、不阻塞，可在 ISR 调用；当前调用方使用有界时长，不依赖乘法回绕语义。
 */
static uint32_t expected_sample_count(uint16_t configured_rate_hz,
                                      uint64_t duration_us)
{
    const uint64_t expected =
        ((uint64_t)configured_rate_hz * duration_us) / UINT64_C(1000000);

    return expected > UINT32_MAX ? UINT32_MAX : (uint32_t)expected;
}

/**
 * @brief 根据允许丢样百分比计算向上取整的最小合格样本数。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] expected 窗口期望样本总数。
 * @return `ceil(expected * (100 - tolerance) / 100)` 的 32 位结果。
 * 调用方式：`set_quality()` 建立每条数据流的质量门限时调用。
 * 线程约束：纯计算、可重入、不阻塞，可在 ISR 调用。
 */
static uint32_t minimum_sample_count(uint32_t expected)
{
    const uint64_t numerator =
        (uint64_t)expected *
        (UINT64_C(100) - IMU_CALIBRATION_SAMPLE_TOLERANCE_PERCENT);

    return (uint32_t)((numerator + UINT64_C(99)) / UINT64_C(100));
}

static uint16_t actual_rate_hz(uint32_t sample_count, uint64_t duration_us);

/**
 * @brief 根据配置 ODR 和固定偏置样本目标冻结 BMI323 陀螺仪质量结果。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * 传入参数：无。
 * @return 无返回值；只有实际样本数恰好等于 `IMU_CAL_GYRO_BIAS_SAMPLE_COUNT` 时才置 `quality_ok`。
 * 调用方式：`store_quality()` 在静态窗口结束时调用。
 * 线程约束：调用者必须已持有标定锁；函数不自行加锁、不阻塞，不可独立从 ISR 调用。
 */
static void set_gyro_bias_quality(void)
{
    const uint16_t rate_hz = s_calibration.bmi_configured_rate_hz;
    const uint64_t duration_us = rate_hz == 0U
                                     ? 0U
                                     : ((uint64_t)IMU_CAL_GYRO_BIAS_SAMPLE_COUNT *
                                        UINT64_C(1000000)) /
                                           rate_hz;
    const uint32_t actual = s_calibration.bmi_gyro.sample_count;

    s_calibration.quality.bmi_gyro = (imu_sample_quality_t){
        .configured_rate_hz = rate_hz,
        .actual_rate_hz = actual_rate_hz(actual, duration_us),
        .expected_sample_count = IMU_CAL_GYRO_BIAS_SAMPLE_COUNT,
        .minimum_sample_count = IMU_CAL_GYRO_BIAS_SAMPLE_COUNT,
        .actual_sample_count = actual,
        .quality_ok = actual == IMU_CAL_GYRO_BIAS_SAMPLE_COUNT ? 1U : 0U,
    };
}

/**
 * @brief 由样本数和实际窗口时长计算四舍五入的实测采样率。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] sample_count 窗口内实际接收的样本数。
 * @param[in] duration_us 统计时长，单位 us；0 表示无法计算。
 * @return 四舍五入后的 Hz；时长为 0 返回 0，超过 `UINT16_MAX` 时饱和。
 * 调用方式：构造加速度与陀螺仪质量快照时调用。
 * 线程约束：纯计算、可重入、不阻塞，可在 ISR 调用。
 */
static uint16_t actual_rate_hz(uint32_t sample_count, uint64_t duration_us)
{
    const uint64_t rate = duration_us == 0U
                              ? 0U
                              : (((uint64_t)sample_count * UINT64_C(1000000)) +
                                 (duration_us / UINT64_C(2))) /
                                    duration_us;

    return rate > UINT16_MAX ? UINT16_MAX : (uint16_t)rate;
}

/**
 * @brief 根据配置频率、实际样本数和容差填充一条采样质量记录。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[out] quality 调用者所有、待整体覆盖的质量结构；`NULL` 时静默返回，指针不被保留。
 * @param[in] configured_rate_hz 配置采样率，单位 Hz。
 * @param[in] actual_sample_count 静态窗口内实际样本数。
 * @return 无返回值；期望样本数为 0 时 `quality_ok` 固定为 0。
 * 调用方式：`store_quality()` 分别为 LSM303 加速度与 BMI323 加速度构造快照。
 * 线程约束：函数不自行加锁、不阻塞；目标结构的独占访问由调用者保证，当前持锁调用路径不可从 ISR 调用。
 */
static void set_quality(imu_sample_quality_t *quality,
                        uint16_t configured_rate_hz,
                        uint32_t actual_sample_count)
{
    const uint32_t expected = expected_sample_count(
        configured_rate_hz, IMU_CALIBRATION_WINDOW_US);
    const uint32_t minimum = minimum_sample_count(expected);

    if (quality == NULL) {
        return;
    }
    *quality = (imu_sample_quality_t){
        .configured_rate_hz = configured_rate_hz,
        .actual_rate_hz = actual_rate_hz(actual_sample_count,
                                         IMU_CALIBRATION_WINDOW_US),
        .expected_sample_count = expected,
        .minimum_sample_count = minimum,
        .actual_sample_count = actual_sample_count,
        .quality_ok = (expected != 0U && actual_sample_count >= minimum) ? 1U : 0U,
    };
}

/**
 * @brief 将当前三条累计流的采样率与样本数冻结为质量快照。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * 传入参数：无。
 * @return 无返回值；质量不合格通过各 `quality_ok=0` 表示，不单独报错。
 * 调用方式：静态窗口到期后，在决定是否接受标定结果前调用。
 * 线程约束：调用者必须已持有标定锁；函数不再加锁、不阻塞，不可独立从 ISR 调用。
 */
static void store_quality(void)
{
    set_quality(&s_calibration.quality.lsm_accel,
                (uint16_t)IMU_CALIBRATION_SAMPLE_RATE_HZ,
                s_calibration.lsm_accel.sample_count);
    set_quality(&s_calibration.quality.bmi_accel,
                s_calibration.bmi_configured_rate_hz,
                s_calibration.bmi_accel.sample_count);
    set_gyro_bias_quality();
}

/**
 * @brief 判断样本时间戳是否落在当前左闭右开的静态窗口内。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] timestamp_us 待检查样本的 64 位单调微秒时间戳。
 * @return 窗口已开启且 `timestamp_us - start < window` 时返回 1，否则返回 0。
 * 调用方式：LSM303/BMI323 样本累计前调用；调用方应传入与窗口起点同域且不早于起点的时间戳。
 * 线程约束：调用者必须已持有标定锁；函数不加锁、不阻塞，不可独立从 ISR 调用。
 */
static uint8_t window_accepts_sample(uint64_t timestamp_us)
{
    return s_calibration.window_active != 0U &&
                   (timestamp_us - s_calibration.window_start_timestamp_us) <
                       IMU_CALIBRATION_WINDOW_US
               ? 1U
               : 0U;
}

/**
 * @brief 检查 LSM 加速度、BMI 加速度和 BMI 陀螺仪三条流是否全部达标。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * 传入参数：无。
 * @return 三个 `quality_ok` 均非 0 时返回 1，任一不合格返回 0。
 * 调用方式：静态窗口结束并完成质量冻结后，决定是否接收偏置结果。
 * 线程约束：调用者必须已持有标定锁；只读共享快照，不阻塞，不可独立从 ISR 调用。
 */
static uint8_t all_streams_meet_quality(void)
{
    return s_calibration.quality.lsm_accel.quality_ok != 0U &&
                   s_calibration.quality.bmi_accel.quality_ok != 0U &&
                   s_calibration.quality.bmi_gyro.quality_ok != 0U
               ? 1U
               : 0U;
}

/**
 * @brief 将当前三条累计器的样本数复制到冻结标定结果。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * 传入参数：无。
 * @return 无返回值；仅复制实际计数，不执行质量判定。
 * 调用方式：静态窗口到期时在 `store_quality()` 之前调用。
 * 线程约束：调用者必须已持有标定锁；函数不加锁、不阻塞，不可独立从 ISR 调用。
 */
static void store_sample_counts(void)
{
    s_calibration.result.sample_counts.lsm_accel =
        s_calibration.lsm_accel.sample_count;
    s_calibration.result.sample_counts.bmi_accel =
        s_calibration.bmi_accel.sample_count;
    s_calibration.result.sample_counts.bmi_gyro =
        s_calibration.bmi_gyro.sample_count;
}

/**
 * @brief 计算实际样本数与期望样本数的比值。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] quality 仅在调用期间借用的质量记录。
 * @return `actual/expected` 浮点比值；指针为 `NULL` 或期望样本数为 0 时返回 0.0。
 * 调用方式：静态统计将采样完整率填入 LSM/BMI 结果时调用。
 * 线程约束：纯计算、不阻塞；指向稳定局部快照时可在 ISR 调用，
 * 若指向共享状态则调用者必须持有标定锁且不得从 ISR 访问。
 */
static float quality_ratio(const imu_sample_quality_t *quality)
{
    if (quality == NULL || quality->expected_sample_count == 0U) {
        return 0.0f;
    }
    return (float)quality->actual_sample_count /
           (float)quality->expected_sample_count;
}

/**
 * @brief 由加速度累计器生成均值、三轴合成标准差和有效比例。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[out] statistics 待整体覆盖的静态传感器统计；`NULL` 时静默返回。
 * @param[in] accumulator 仅在调用期间借用的加速度累计器；空或无样本时仅保留 `valid_ratio`。
 * @param[in] valid_ratio 实际样本数与期望样本数比值，本函数不限幅。
 * @return 无返回值；因浮点误差得到的负方差在开方前截为 0。
 * 调用方式：`store_static_statistics()` 分别为 LSM303 与 BMI323 生成静态快照。
 * 线程约束：函数不自行加锁、不阻塞；输入和输出的独占访问由调用者保证，
 * 当前在持有标定锁的任务中调用，不可从 ISR 调用。
 */
static void store_accel_statistics(imu_calibration_static_sensor_t *statistics,
                                   const imu_axis_accumulator_t *accumulator,
                                   float valid_ratio)
{
    double variance_x;
    double variance_y;
    double variance_z;
    const double count = accumulator == NULL ? 0.0 :
                         (double)accumulator->sample_count;

    if (statistics == NULL) {
        return;
    }
    *statistics = (imu_calibration_static_sensor_t){
        .valid_ratio = valid_ratio,
    };
    if (accumulator == NULL || accumulator->sample_count == 0U) {
        return;
    }

    statistics->accel_mean[0] = (float)(accumulator->sum_x / count);
    statistics->accel_mean[1] = (float)(accumulator->sum_y / count);
    statistics->accel_mean[2] = (float)(accumulator->sum_z / count);
    variance_x = (accumulator->sum_square_x / count) -
                 ((double)statistics->accel_mean[0] *
                  (double)statistics->accel_mean[0]);
    variance_y = (accumulator->sum_square_y / count) -
                 ((double)statistics->accel_mean[1] *
                  (double)statistics->accel_mean[1]);
    variance_z = (accumulator->sum_square_z / count) -
                 ((double)statistics->accel_mean[2] *
                  (double)statistics->accel_mean[2]);
    variance_x = variance_x > 0.0 ? variance_x : 0.0;
    variance_y = variance_y > 0.0 ? variance_y : 0.0;
    variance_z = variance_z > 0.0 ? variance_z : 0.0;
    statistics->accel_std_mps2 =
        sqrtf((float)((variance_x + variance_y + variance_z) / 3.0));
}

/**
 * @brief 由陀螺仪三轴累计值计算去均值后的合成 RMS。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] accumulator 仅在调用期间借用的陀螺仪累计器。
 * @return 三轴方差平均值的平方根，单位 rad/s；指针为 `NULL` 或无样本时返回 0.0。
 * 调用方式：静态统计冻结时生成 BMI323 运动判定量。
 * 线程约束：纯计算、不阻塞；当前在持有标定锁的任务中读取共享累计器，不可从 ISR 调用。
 */
static float gyro_rms(const imu_axis_accumulator_t *accumulator)
{
    const double count = accumulator == NULL ? 0.0 :
                         (double)accumulator->sample_count;
    double variance_x;
    double variance_y;
    double variance_z;

    if (accumulator == NULL || accumulator->sample_count == 0U) {
        return 0.0f;
    }
    variance_x = (accumulator->sum_square_x / count) -
                 ((accumulator->sum_x / count) *
                  (accumulator->sum_x / count));
    variance_y = (accumulator->sum_square_y / count) -
                 ((accumulator->sum_y / count) *
                  (accumulator->sum_y / count));
    variance_z = (accumulator->sum_square_z / count) -
                 ((accumulator->sum_z / count) *
                  (accumulator->sum_z / count));
    variance_x = variance_x > 0.0 ? variance_x : 0.0;
    variance_y = variance_y > 0.0 ? variance_y : 0.0;
    variance_z = variance_z > 0.0 ? variance_z : 0.0;
    return sqrtf((float)((variance_x + variance_y + variance_z) / 3.0));
}

/**
 * @brief 在窗口结束时冻结两路加速度统计、BMI 陀螺仪 RMS 和运动标志。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * 传入参数：无。
 * @return 无返回值；任一加速度标准差或陀螺仪 RMS 超限时置 `static_motion_detected`，不单独返回失败。
 * 调用方式：静态窗口到期且样本数与质量已冻结后调用。
 * 线程约束：调用者必须已持有标定锁；内部执行浮点方差/开方计算，不阻塞但有有界 CPU 开销，不可从 ISR 调用。
 */
static void store_static_statistics(void)
{
    const float lsm_ratio = quality_ratio(&s_calibration.quality.lsm_accel);
    const float bmi_accel_ratio =
        quality_ratio(&s_calibration.quality.bmi_accel);
    const float bmi_gyro_ratio = quality_ratio(&s_calibration.quality.bmi_gyro);
    const float bmi_ratio = bmi_accel_ratio < bmi_gyro_ratio
                                ? bmi_accel_ratio
                                : bmi_gyro_ratio;

    store_accel_statistics(&s_calibration.static_statistics.lsm,
                           &s_calibration.lsm_accel, lsm_ratio);
    store_accel_statistics(&s_calibration.static_statistics.bmi,
                           &s_calibration.bmi_accel, bmi_ratio);
    s_calibration.static_statistics.bmi.gyro_rms_radps =
        gyro_rms(&s_calibration.bmi_gyro);

    if (s_calibration.static_statistics.lsm.accel_std_mps2 >
            LSM_ACCEL_STD_MAX ||
        s_calibration.static_statistics.bmi.accel_std_mps2 >
            BMI_ACCEL_STD_MAX ||
        s_calibration.static_statistics.bmi.gyro_rms_radps >
            IMU_CALIBRATION_GYRO_MOTION_THRESHOLD_RADPS) {
        s_calibration.static_motion_detected = 1U;
    }
}

/** 清空所有累计器、质量结果和生命周期标志。 */
void imu_calibration_init(void)
{
#if defined(IMU_MANAGER_USE_FREERTOS)
    if (s_mutex == NULL) {
        s_mutex = xSemaphoreCreateMutex();
    }
#endif
    s_bmi_capture_active = 0U;
    lock_calibration();
    (void)memset(&s_calibration, 0, sizeof(s_calibration));
    unlock_calibration();
}

/** 开启新一轮标定并清理动态样本标志。 */
void imu_calibration_start(void)
{
    s_bmi_capture_active = 0U;
    lock_calibration();
    (void)memset(&s_calibration, 0, sizeof(s_calibration));
    unlock_calibration();
}

/** 记录静态窗口起点及 BMI323 配置采样率。 */
void imu_calibration_begin_window(uint64_t start_timestamp_us,
                                  uint16_t bmi_configured_rate_hz)
{
    lock_calibration();
    s_calibration.window_start_timestamp_us = start_timestamp_us;
    s_calibration.bmi_configured_rate_hz = bmi_configured_rate_hz;
    s_calibration.window_active = 1U;
    s_calibration.complete = 0U;
    s_calibration.last_lsm_accel_timestamp_us = 0U;
    s_bmi_capture_active = 1U;
    unlock_calibration();
}

/** 查询 BMI323 原始捕获窗口是否打开。 */
uint8_t imu_calibration_bmi_capture_active(void)
{
    return s_bmi_capture_active;
}

/** 判断给定单调时间戳是否已超过静态窗口。 */
uint8_t imu_calibration_window_expired(uint64_t now_timestamp_us)
{
    uint8_t expired;

    lock_calibration();
    expired = s_calibration.window_active != 0U &&
                      (now_timestamp_us -
                       s_calibration.window_start_timestamp_us) >=
                          IMU_CALIBRATION_WINDOW_US
                  ? 1U
                  : 0U;
    unlock_calibration();
    return expired;
}

/** 查询最近静态窗口是否观察到动态样本。 */
uint8_t imu_calibration_static_motion_detected(void)
{
    uint8_t detected;

    lock_calibration();
    detected = s_calibration.static_motion_detected;
    unlock_calibration();
    return detected;
}

/** 关闭窗口、计算质量并冻结偏置结果。 */
uint8_t imu_calibration_finish_window(uint64_t now_timestamp_us)
{
    uint8_t complete;

    lock_calibration();
    if (s_calibration.window_active != 0U &&
        (now_timestamp_us - s_calibration.window_start_timestamp_us) >=
            IMU_CALIBRATION_WINDOW_US) {
        s_calibration.window_active = 0U;
        s_bmi_capture_active = 0U;
        store_sample_counts();
        store_quality();
        store_static_statistics();
        if (all_streams_meet_quality() != 0U) {
            /* A one-pose static window cannot identify sensor bias separately
             * from mechanical incline. R_level owns the gravity direction;
             * leaving these at zero prevents gravity from being subtracted
             * once as a pseudo-bias and again by the rotation. */
            s_calibration.result.lsm_accel_bias = (imu_bias_xyz_t){0};
            s_calibration.result.bmi_accel_bias = (imu_bias_xyz_t){0};
            s_calibration.result.bmi_gyro_bias =
                mean_xyz(&s_calibration.bmi_gyro);
            s_calibration.bias = (imu_calibration_bias_t){0};
            s_calibration.complete = 1U;
        }
    }
    complete = s_calibration.complete;
    unlock_calibration();
    return complete;
}

/** 累计一组双 IMU 原始快照及静态统计。 */
void imu_calibration_update(const imu_raw_data_t *raw_data)
{
    if (raw_data == NULL) {
        return;
    }

    lock_calibration();
    const uint8_t lsm_accel_valid = raw_data->lsm_accel_valid != 0U ||
                                     raw_data->online != 0U;
    const float lsm_ax = raw_data->lsm_accel_valid != 0U ? raw_data->lsm_ax : raw_data->ax;
    const float lsm_ay = raw_data->lsm_accel_valid != 0U ? raw_data->lsm_ay : raw_data->ay;
    const float lsm_az = raw_data->lsm_accel_valid != 0U ? raw_data->lsm_az : raw_data->az;
    const uint64_t lsm_timestamp_us = raw_data->lsm_accel_timestamp_us != 0U
                                          ? raw_data->lsm_accel_timestamp_us
                                          : (raw_data->lsm_accel_valid != 0U
                                                 ? raw_data->lsm_timestamp_us
                                                 : raw_data->timestamp_us);
    const uint8_t lsm_sample_is_new =
        (lsm_accel_valid != 0U && lsm_timestamp_us != 0U &&
         lsm_timestamp_us > s_calibration.last_lsm_accel_timestamp_us)
            ? 1U
            : 0U;

    if (s_calibration.window_active != 0U && lsm_sample_is_new != 0U) {
        const float accel_norm = sqrtf((lsm_ax * lsm_ax) +
                                       (lsm_ay * lsm_ay) +
                                       (lsm_az * lsm_az));
        if (isfinite(accel_norm) &&
            fabsf(accel_norm - IMU_CALIBRATION_GRAVITY_MPS2) >
                IMU_CALIBRATION_ACCEL_MOTION_DELTA_MPS2) {
            s_calibration.static_motion_detected = 1U;
        }
    }

    if (lsm_sample_is_new != 0U &&
        window_accepts_sample(lsm_timestamp_us) != 0U) {
        if (accumulate_xyz(&s_calibration.lsm_accel, lsm_ax, lsm_ay, lsm_az) !=
            0U) {
            s_calibration.lsm_observed = 1U;
        }
        s_calibration.last_lsm_accel_timestamp_us = lsm_timestamp_us;
    }
    unlock_calibration();
}

/** 累计一条 BMI323 加速度/陀螺仪样本。 */
void imu_calibration_update_bmi323(float accel_x, float accel_y, float accel_z,
                                   float gyro_x, float gyro_y, float gyro_z,
                                   uint64_t timestamp_us)
{
    float accel_mps2[3] = {accel_x, accel_y, accel_z};

    /* The ODR task must never wait behind the 100 Hz manager. If another
     * reader owns the accumulator briefly, the reported actual count reflects
     * the skipped observation. */
    if (s_bmi_capture_active == 0U || try_lock_calibration() == 0U) {
        return;
    }
    bmi323_accel_input_to_mps2(accel_mps2);
    if (s_calibration.window_active != 0U) {
        const float accel_norm = sqrtf((accel_mps2[0] * accel_mps2[0]) +
                                       (accel_mps2[1] * accel_mps2[1]) +
                                       (accel_mps2[2] * accel_mps2[2]));
        const float gyro_norm = sqrtf((gyro_x * gyro_x) + (gyro_y * gyro_y) +
                                      (gyro_z * gyro_z));
        if ((isfinite(accel_norm) &&
             fabsf(accel_norm - IMU_CALIBRATION_GRAVITY_MPS2) >
                 IMU_CALIBRATION_ACCEL_MOTION_DELTA_MPS2) ||
            (isfinite(gyro_norm) &&
             gyro_norm > IMU_CALIBRATION_GYRO_MOTION_THRESHOLD_RADPS)) {
            s_calibration.static_motion_detected = 1U;
        }
    }
    if (window_accepts_sample(timestamp_us) != 0U) {
        if (accumulate_xyz(&s_calibration.bmi_accel,
                           accel_mps2[0], accel_mps2[1],
                           accel_mps2[2]) != 0U) {
            s_calibration.bmi_accel_observed = 1U;
        }
        if (s_calibration.bmi_gyro.sample_count <
                IMU_CAL_GYRO_BIAS_SAMPLE_COUNT &&
            accumulate_xyz(&s_calibration.bmi_gyro,
                           gyro_x, gyro_y, gyro_z) != 0U) {
            s_calibration.bmi_gyro_observed = 1U;
        }
    }
    unlock_calibration();
}

/** 查询双 IMU 标定是否完成。 */
uint8_t imu_calibration_is_complete(void)
{
    uint8_t complete;
    lock_calibration();
    complete = s_calibration.complete;
    unlock_calibration();
    return complete;
}

uint32_t imu_calibration_get_sample_count(void)
{
    uint32_t count;
    lock_calibration();
    count = s_calibration.lsm_accel.sample_count;
    if (s_calibration.bmi_accel.sample_count < count) {
        count = s_calibration.bmi_accel.sample_count;
    }
    if (s_calibration.bmi_gyro.sample_count < count) {
        count = s_calibration.bmi_gyro.sample_count;
    }
    unlock_calibration();
    return count;
}

uint32_t imu_calibration_get_sample_total(void)
{
    return IMU_CALIBRATION_LSM303_NOMINAL_SAMPLES;
}

uint8_t imu_calibration_get_progress(void)
{
    uint8_t progress;

    lock_calibration();
    if (s_calibration.complete != 0U) {
        progress = 100U;
    } else if (s_calibration.window_active != 0U) {
        const uint64_t elapsed_us = imu_time_now_us() -
                                    s_calibration.window_start_timestamp_us;
        const uint64_t percent =
            (elapsed_us * UINT64_C(100)) / IMU_CALIBRATION_WINDOW_US;
        progress = (uint8_t)(percent > 100U ? 100U : percent);
    } else {
        progress = 0U;
    }
    unlock_calibration();
    return progress;
}

imu_calibration_bias_t imu_calibration_get_bias(void)
{
    imu_calibration_bias_t bias;
    lock_calibration();
    bias = s_calibration.bias;
    unlock_calibration();
    return bias;
}

imu_calibration_result_t imu_calibration_get_result(void)
{
    imu_calibration_result_t result;
    lock_calibration();
    result = s_calibration.result;
    unlock_calibration();
    return result;
}

imu_calibration_sample_counts_t imu_calibration_get_sample_counts(void)
{
    imu_calibration_sample_counts_t counts;

    lock_calibration();
    counts.lsm_accel = s_calibration.lsm_accel.sample_count;
    counts.bmi_accel = s_calibration.bmi_accel.sample_count;
    counts.bmi_gyro = s_calibration.bmi_gyro.sample_count;
    unlock_calibration();
    return counts;
}

imu_calibration_quality_t imu_calibration_get_quality(void)
{
    imu_calibration_quality_t quality;

    lock_calibration();
    quality = s_calibration.quality;
    unlock_calibration();
    return quality;
}

imu_calibration_static_statistics_t imu_calibration_get_static_statistics(void)
{
    imu_calibration_static_statistics_t statistics;

    lock_calibration();
    statistics = s_calibration.static_statistics;
    unlock_calibration();
    return statistics;
}

uint8_t imu_calibration_is_lsm_complete(void)
{
    uint8_t complete;
    lock_calibration();
    complete = (s_calibration.complete != 0U &&
                s_calibration.quality.lsm_accel.quality_ok != 0U) ? 1U : 0U;
    unlock_calibration();
    return complete;
}

uint8_t imu_calibration_is_bmi_complete(void)
{
    uint8_t complete;
    lock_calibration();
    complete = (s_calibration.complete != 0U &&
                s_calibration.quality.bmi_accel.quality_ok != 0U &&
                s_calibration.quality.bmi_gyro.quality_ok != 0U) ? 1U : 0U;
    unlock_calibration();
    return complete;
}

imu_calibrated_data_t imu_calibration_apply(const imu_raw_data_t *raw_data)
{
    imu_calibrated_data_t calibrated = {0};
    imu_calibration_bias_t bias;

    if (raw_data == NULL) {
        return calibrated;
    }
    bias = imu_calibration_get_bias();
    calibrated = *raw_data;
    calibrated.ax -= bias.ax;
    calibrated.ay -= bias.ay;
    calibrated.az -= bias.az;
    lock_calibration();
    calibrated.lsm_ax = calibrated.ax - s_calibration.result.lsm_accel_bias.x + bias.ax;
    calibrated.lsm_ay = calibrated.ay - s_calibration.result.lsm_accel_bias.y + bias.ay;
    calibrated.lsm_az = calibrated.az - s_calibration.result.lsm_accel_bias.z + bias.az;
    calibrated.bmi_ax -= s_calibration.result.bmi_accel_bias.x;
    calibrated.bmi_ay -= s_calibration.result.bmi_accel_bias.y;
    calibrated.bmi_az -= s_calibration.result.bmi_accel_bias.z;
    calibrated.bmi_gx -= s_calibration.result.bmi_gyro_bias.x;
    calibrated.bmi_gy -= s_calibration.result.bmi_gyro_bias.y;
    calibrated.bmi_gz -= s_calibration.result.bmi_gyro_bias.z;
    s_calibration.last_calibrated = calibrated;
    unlock_calibration();
    return calibrated;
}

imu_calibrated_data_t imu_calibration_get_data(void)
{
    imu_calibrated_data_t calibrated;
    lock_calibration();
    calibrated = s_calibration.last_calibrated;
    unlock_calibration();
    return calibrated;
}
