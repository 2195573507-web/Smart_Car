#ifndef IMU_CALIBRATION_H
#define IMU_CALIBRATION_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 双 IMU 标定接口。
 * 创建人：待确认（当前维护人：Zhiqin）。
 * 静态窗口、样本质量和偏置结果由本模块维护；窗口未完成时不得把结果用于运动控制。
 */

#define IMU_CAL_STATIC_WINDOW_MS UINT32_C(6000)
/* Retained for callers that use the older calibration-window name. */
#define IMU_CALIBRATION_WINDOW_MS IMU_CAL_STATIC_WINDOW_MS
#define IMU_CALIBRATION_WINDOW_US \
    ((uint64_t)IMU_CAL_STATIC_WINDOW_MS * UINT64_C(1000))
#define IMU_CALIBRATION_SAMPLE_RATE_HZ UINT32_C(100)
#define IMU_CALIBRATION_SAMPLE_TOLERANCE_PERCENT UINT32_C(10)
/* BMI323 runs at 200 Hz in the normal image. A six-second static window
 * leaves rate/scheduling headroom while the gyro-bias accumulator itself is
 * capped at exactly this many accepted samples. */
#define IMU_CAL_GYRO_BIAS_SAMPLE_COUNT UINT32_C(1000)
#define BMI_ACCEL_STD_MAX (0.10f)
#define LSM_ACCEL_STD_MAX (0.50f)
#define IMU_CALIBRATION_LSM303_NOMINAL_SAMPLES \
    ((IMU_CALIBRATION_SAMPLE_RATE_HZ * IMU_CAL_STATIC_WINDOW_MS) / \
     UINT32_C(1000))

/**
 * @brief LSM303 与 BMI323 最新数据的统一值快照，并保留旧 LSM303 字段视图。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * @note 当前生产者分别复制两路传感器状态后再聚合，因此各字段不是同一硬件时刻的原子采样。
 *       结构体无内部指针，调用方按值拥有副本；使用每条流前必须同时检查对应 valid 和时间戳。
 */
typedef struct
{
    /* 旧滤波/姿态路径保留的 LSM303 兼容视图。 */
    float ax; /**< Body Frame X 轴 LSM303 加速度，单位 m/s^2。 */
    float ay; /**< Body Frame Y 轴 LSM303 加速度，单位 m/s^2。 */
    float az; /**< Body Frame Z 轴 LSM303 加速度，单位 m/s^2。 */
    float mx; /**< Body Frame X 轴 LSM303 磁场，单位 uT。 */
    float my; /**< Body Frame Y 轴 LSM303 磁场，单位 uT。 */
    float mz; /**< Body Frame Z 轴 LSM303 磁场，单位 uT。 */
    uint32_t timestamp; /**< LSM 加速度/磁场较新时间戳的毫秒视图，单位 ms，会自然回绕。 */
    uint64_t timestamp_us; /**< LSM 加速度/磁场较新时间戳，单位 us。 */
    uint8_t online; /**< 两路 LSM 数据在快照生成时均新鲜为 1；不表示 BMI 有效。 */

    /* 统一快照字段；消费者不得依赖传感器声明顺序推断来源。 */
    float lsm_ax; /**< Body Frame X 轴 LSM303 加速度，单位 m/s^2。 */
    float lsm_ay; /**< Body Frame Y 轴 LSM303 加速度，单位 m/s^2。 */
    float lsm_az; /**< Body Frame Z 轴 LSM303 加速度，单位 m/s^2。 */
    float lsm_mx; /**< Body Frame X 轴 LSM303 磁场，单位 uT。 */
    float lsm_my; /**< Body Frame Y 轴 LSM303 磁场，单位 uT。 */
    float lsm_mz; /**< Body Frame Z 轴 LSM303 磁场，单位 uT。 */
    float bmi_ax; /**< Body Frame X 轴 BMI323 加速度，单位 m/s^2。 */
    float bmi_ay; /**< Body Frame Y 轴 BMI323 加速度，单位 m/s^2。 */
    float bmi_az; /**< Body Frame Z 轴 BMI323 加速度，单位 m/s^2。 */
    float bmi_gx; /**< Body Frame X 轴 BMI323 角速度，单位 rad/s。 */
    float bmi_gy; /**< Body Frame Y 轴 BMI323 角速度，单位 rad/s。 */
    float bmi_gz; /**< Body Frame Z 轴 BMI323 角速度，单位 rad/s。 */
    uint32_t lsm_timestamp; /**< `lsm_timestamp_us` 的 32 位毫秒兼容视图。 */
    uint32_t bmi_timestamp; /**< BMI323 样本的 32 位毫秒时间戳，单位 ms。 */
    uint64_t lsm_timestamp_us; /**< LSM 加速度/磁场较新时间戳，单位 us。 */
    uint64_t lsm_accel_timestamp_us; /**< 当前 LSM303 加速度样本时间戳，单位 us。 */
    uint64_t lsm_mag_timestamp_us; /**< 当前 LSM303 磁场样本时间戳，单位 us。 */
    uint64_t bmi_timestamp_us; /**< 当前 BMI323 聚合样本时间戳，单位 us。 */
    uint8_t lsm_accel_valid; /**< LSM 加速度在快照时刻通过有效性和 freshness 检查为 1。 */
    uint8_t lsm_mag_valid; /**< LSM 磁场在快照时刻通过有效性和 freshness 检查为 1。 */
    uint8_t bmi_accel_valid; /**< 最新 BMI323 样本包含有效加速度为 1。 */
    uint8_t bmi_gyro_valid; /**< 最新 BMI323 样本包含有效陀螺仪数据为 1。 */
} imu_raw_data_t;

/** 标定应用后的统一快照别名；布局、时间戳和 valid 语义与 `imu_raw_data_t` 完全相同。 */
typedef imu_raw_data_t imu_calibrated_data_t;

/**
 * @brief 旧 LSM303 标定/滤波路径使用的六轴加性偏置值。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * @note 由 calibration 模块拥有并按值返回；当前单姿态静态方案将该结构保持为零，
 *       由水平矩阵承担重力方向，不应把其非零与标定完成等同。
 */
typedef struct
{
    float ax; /**< 旧视图 X 轴加速度偏置，单位 m/s^2。 */
    float ay; /**< 旧视图 Y 轴加速度偏置，单位 m/s^2。 */
    float az; /**< 旧视图 Z 轴加速度偏置，单位 m/s^2。 */
    float mx; /**< 旧视图 X 轴磁场偏置，单位 uT。 */
    float my; /**< 旧视图 Y 轴磁场偏置，单位 uT。 */
    float mz; /**< 旧视图 Z 轴磁场偏置，单位 uT。 */
} imu_calibration_bias_t;

/**
 * @brief 通用三轴偏置值对象，具体物理单位由所在结果字段决定。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * @note 结构体无所有权资源；加速度字段使用 m/s^2，陀螺仪字段使用 rad/s。
 */
typedef struct
{
    float x; /**< X 轴偏置；单位由承载该对象的传感器字段决定。 */
    float y; /**< Y 轴偏置；单位由承载该对象的传感器字段决定。 */
    float z; /**< Z 轴偏置；单位由承载该对象的传感器字段决定。 */
} imu_bias_xyz_t;

/**
 * @brief 当前静态窗口内三条数据流实际通过检查并成功累计的样本数。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * @note BMI 高频路径锁竞争时会主动丢样，因此该计数是实际接受值，不是硬件 ODR 推算值。
 */
typedef struct
{
    uint32_t lsm_accel; /**< 已接受的 LSM303 加速度样本数。 */
    uint32_t bmi_accel; /**< 已接受的 BMI323 加速度样本数。 */
    uint32_t bmi_gyro; /**< 已接受的 BMI323 陀螺样本数；当前最多累计 1000 点。 */
} imu_calibration_sample_counts_t;

/**
 * @brief 单条传感器数据流在静态窗口关闭时冻结的采样完整性记录。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * @note 该记录只评价速率/数量门，不单独证明传感器静止、数值有限或物理标定正确。
 */
typedef struct
{
    uint16_t configured_rate_hz; /**< 质量计算采用的配置采样率，单位 Hz。 */
    uint16_t actual_rate_hz; /**< 由样本数和统计时长四舍五入得到的实测速率，单位 Hz。 */
    uint32_t expected_sample_count; /**< 按配置速率和对应窗口推算的期望样本数。 */
    uint32_t minimum_sample_count; /**< 考虑容差后的最小合格样本数。 */
    uint32_t actual_sample_count; /**< 实际通过输入/窗口检查并成功累计的样本数。 */
    uint8_t quality_ok; /**< 实际计数达到该流门限为 1，否则为 0。 */
} imu_sample_quality_t;

/**
 * @brief 静态窗口三条数据流的采样质量快照。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * @note 由 calibration 模块在窗口结束时整体冻结，getter 按值复制给诊断与启动管理器。
 */
typedef struct
{
    imu_sample_quality_t lsm_accel; /**< LSM303 加速度流的 100 Hz 名义窗口质量。 */
    imu_sample_quality_t bmi_accel; /**< BMI323 加速度流按当前配置 ODR 计算的窗口质量。 */
    imu_sample_quality_t bmi_gyro; /**< BMI323 陀螺固定 1000 点偏置样本质量。 */
} imu_calibration_quality_t;

/**
 * @brief 单路 IMU 在静态窗口关闭时冻结的原始静止观测统计。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * @note 供水平校准和运动判定使用，不把重力向量误作加速度偏置；对象由模块内部所有，
 *       对外只通过包含它的统计结构按值复制。
 */
typedef struct
{
    float accel_mean[3]; /**< Body Frame 三轴静态加速度均值 `[X,Y,Z]`，单位 m/s^2。 */
    float accel_std_mps2; /**< 三轴方差平均后开方的合成标准差，单位 m/s^2。 */
    float gyro_rms_radps; /**< 去均值后三轴方差平均的 RMS，单位 rad/s；LSM 视图为 0。 */
    float valid_ratio; /**< 实际样本数/期望样本数；未限幅，数据过采样时可大于 1。 */
} imu_calibration_static_sensor_t;

/**
 * @brief LSM303 与 BMI323 两路静态观测统计的冻结快照。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * @note getter 返回完整副本；只有静态窗口结束后字段才稳定，可作为水平校准输入。
 */
typedef struct
{
    imu_calibration_static_sensor_t lsm; /**< LSM303 加速度统计；无陀螺来源。 */
    imu_calibration_static_sensor_t bmi; /**< BMI323 加速度统计及陀螺 RMS。 */
} imu_calibration_static_statistics_t;

/**
 * @brief 质量门通过后冻结的双 IMU 偏置与实际样本计数结果。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * @note 当前单姿态方案把两路加速度偏置保持为零，仅估计 BMI323 陀螺均值偏置；
 *       调用方必须先确认 calibration complete，再把按值副本用于姿态链。
 */
typedef struct
{
    imu_bias_xyz_t lsm_accel_bias; /**< LSM303 加速度偏置，单位 m/s^2；当前设计冻结为零。 */
    imu_bias_xyz_t bmi_accel_bias; /**< BMI323 加速度偏置，单位 m/s^2；当前设计冻结为零。 */
    imu_bias_xyz_t bmi_gyro_bias; /**< BMI323 静止角速度均值偏置，单位 rad/s。 */
    imu_calibration_sample_counts_t sample_counts; /**< 窗口关闭时冻结的三条流实际接受计数。 */
} imu_calibration_result_t;

/**
 * @brief 创建标定互斥量并清空累计器、质量、偏置和最近输出。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * 传入参数：无。
 * @return 无返回值；互斥量创建失败时实现仍清空静态状态，但不提供并发保护。
 * @note 调用方式与线程约束：IMU manager 初始化阶段、任何标定 API 前调用一次；可能分配
 *       FreeRTOS mutex 并阻塞，禁止从 ISR 或与采样入口并发调用。
 */
void imu_calibration_init(void);
/**
 * @brief 清空当前标定窗口、样本累计、质量结果和动态样本标志。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * 传入参数：无。
 * @return 无返回值；BMI 高频捕获标志同步关闭。
 * @note 调用方式与线程约束：启动管理器开始/重启静态窗口前调用；获取可无限等待的 mutex，
 *       会使旧结果失效，禁止从 ISR 调用。
 */
void imu_calibration_start(void);
/**
 * @brief 打开新的静态标定时间窗并记录 BMI323 配置采样率。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * @param[in] start_timestamp_us 窗口起点的单调时间戳，单位 us，必须非零且与后续
 *                               样本/结束时间使用同一时基。
 * @param[in] bmi_configured_rate_hz BMI323 当前配置 ODR，单位 Hz，用于质量统计。
 * @return 无返回值；调用后 BMI 高频样本入口变为 active。
 * @note 调用方式与线程约束：仅启动管理器在 `imu_calibration_start()` 后调用；获取可阻塞
 *       mutex，不校验时间戳/采样率，调用方负责正确性，禁止从 ISR 调用。
 */
void imu_calibration_begin_window(uint64_t start_timestamp_us,
                                  uint16_t bmi_configured_rate_hz);
/**
 * @brief 查询 BMI323 高频样本捕获窗口是否处于 active。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * 传入参数：无。
 * @return 1 表示允许尝试累计 BMI323 样本，0 表示窗口关闭。
 * @note 调用方式与线程约束：BMI323 采样任务在提交样本前快速读取；无锁、不阻塞，结果是
 *       瞬时门控，后续 try-lock 或窗口时间检查仍可能拒绝样本。
 */
uint8_t imu_calibration_bmi_capture_active(void);
/**
 * @brief 根据同一单调时基判断当前静态窗口是否达到固定时长。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * @param[in] now_timestamp_us 当前单调时间戳，单位 us；必须不早于窗口起点。
 * @return active 且经过时间不少于 `IMU_CALIBRATION_WINDOW_US` 时返回 1，否则 0。
 * @note 调用方式与线程约束：启动管理器周期调用；获取可阻塞 mutex，禁止从 ISR 调用；
 *       调用方必须保证时间单调，避免无符号时间差误判。
 */
uint8_t imu_calibration_window_expired(uint64_t now_timestamp_us);
/**
 * @brief 在窗口到期后关闭采样，冻结计数/质量/静态统计和偏置结果。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * @param[in] now_timestamp_us 当前单调时间戳，单位 us，必须不早于窗口起点。
 * @return 1 表示所有 LSM/BMI 数据流通过质量门并冻结结果；0 表示窗口未到期
 *         或质量失败，失败结果不得用于运动控制。
 * @note 调用方式与线程约束：启动管理器确认窗口到期后调用一次；持有可阻塞 mutex 并执行
 *       浮点统计，关闭 BMI capture，禁止从 ISR 或与 reset 并发调用。
 */
uint8_t imu_calibration_finish_window(uint64_t now_timestamp_us);
/* True when a dynamic sample was observed during the most recent static
 * window. The flag is cleared by imu_calibration_start(). */
/**
 * @brief 查询最近静态窗口是否检测到超限加速度/角速度或静态统计异常。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * 传入参数：无。
 * @return 检测到运动返回 1，否则返回 0；`imu_calibration_start()` 会清除此标志。
 * @note 调用方式与线程约束：启动管理器周期/窗口结束时读取；获取可阻塞 mutex，禁止从 ISR 调用。
 */
uint8_t imu_calibration_static_motion_detected(void);
/**
 * @brief 从统一快照累计一条新的 LSM303 加速度静态样本。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * @param[in] raw_data 调用方拥有的只读双 IMU 快照；加速度单位 m/s^2，时间戳
 *                     单位 us；NULL、无效、重复或窗口外样本不累计。
 * @return 无返回值。
 * @note 调用方式与线程约束：IMU manager 在发布统一快照后调用；输入仅在调用期间读取，
 *       函数获取可无限等待的 mutex，禁止从 ISR 或高频 BMI ODR 路径调用。
 */
void imu_calibration_update(const imu_raw_data_t *raw_data);
/**
 * @brief 以零等待锁策略累计一条 BMI323 加速度/陀螺仪静态样本。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * @param[in] accel_x X 轴加速度，当前生产调用单位 m/s^2。
 * @param[in] accel_y Y 轴加速度，当前生产调用单位 m/s^2。
 * @param[in] accel_z Z 轴加速度，当前生产调用单位 m/s^2。
 * @param[in] gyro_x X 轴角速度，单位 rad/s。
 * @param[in] gyro_y Y 轴角速度，单位 rad/s。
 * @param[in] gyro_z Z 轴角速度，单位 rad/s。
 * @param[in] timestamp_us 样本单调时间戳，单位 us。
 * @return 无返回值；窗口关闭、锁忙、时间戳越界或非有限值时样本被忽略。
 * @note 调用方式与线程约束：BMI323 200 Hz owner 每个新样本调用；使用零等待 try-lock，
 *       不阻塞高频任务且锁忙会丢样，禁止从 ISR 调用。
 */
void imu_calibration_update_bmi323(float accel_x, float accel_y, float accel_z,
                                   float gyro_x, float gyro_y, float gyro_z,
                                   uint64_t timestamp_us);
/**
 * @brief 查询双 IMU 静态标定结果是否已经通过全部质量门并冻结。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * 传入参数：无。
 * @return 完成返回 1，否则返回 0。
 * @note 调用方式与线程约束：姿态/启动管理器在任务上下文读取；获取可阻塞 mutex，
 *       reset/start 后结果立即失效，禁止从 ISR 调用。
 */
uint8_t imu_calibration_is_complete(void);
/**
 * @brief 读取三条数据流已接受样本数的最小值。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * 传入参数：无。
 * @return LSM 加速度、BMI 加速度、BMI 陀螺计数中的最小值。
 * @note 调用方式与线程约束：诊断/状态任务读取；获取可阻塞 mutex，禁止从 ISR 调用。
 */
uint32_t imu_calibration_get_sample_count(void);
/**
 * @brief 获取兼容状态使用的 LSM303 名义目标样本数。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * 传入参数：无。
 * @return 编译期名义样本总数；不等于 BMI323 各流实际期望计数。
 * @note 调用方式与线程约束：任务或测试可直接调用；常量查询、不阻塞且可重入。
 */
uint32_t imu_calibration_get_sample_total(void);
/**
 * @brief 按静态窗口经过时间读取 0..100 的标定进度。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * 传入参数：无。
 * @return 完成时 100，活动窗口按时间比例返回 0..100，否则返回 0。
 * @note 调用方式与线程约束：状态/遥测任务低频读取；获取可阻塞 mutex 并读取单调时基，
 *       禁止从 ISR 调用；进度不代表样本质量通过。
 */
uint8_t imu_calibration_get_progress(void);
/**
 * @brief 按值复制兼容旧路径的 LSM303 偏置视图。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * 传入参数：无。
 * @return 当前 `imu_calibration_bias_t` 副本；调用方应先检查 complete。
 * @note 调用方式与线程约束：滤波/诊断任务读取；获取可阻塞 mutex，不暴露内部存储，
 *       start/reset 后旧副本不再代表当前生命周期。
 */
imu_calibration_bias_t imu_calibration_get_bias(void);
/**
 * @brief 按值复制冻结的双 IMU 偏置和样本计数结果。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * 传入参数：无。
 * @return 当前 `imu_calibration_result_t` 副本；complete 为 0 时不得用于控制。
 * @note 调用方式与线程约束：IMU manager 提交偏置前读取；获取可阻塞 mutex，不暴露内部存储。
 */
imu_calibration_result_t imu_calibration_get_result(void);
/**
 * @brief 按值复制 LSM 加速度、BMI 加速度和 BMI 陀螺已接受样本数。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * 传入参数：无。
 * @return 三条数据流的计数副本。
 * @note 调用方式与线程约束：质量日志/诊断读取；获取可阻塞 mutex，禁止从 ISR 调用。
 */
imu_calibration_sample_counts_t imu_calibration_get_sample_counts(void);
/**
 * @brief 按值复制各流配置/实际采样率、期望计数和质量判定。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * 传入参数：无。
 * @return 当前 `imu_calibration_quality_t` 副本；窗口结束前字段可能尚未冻结。
 * @note 调用方式与线程约束：窗口结束后的启动管理器/日志读取；获取可阻塞 mutex。
 */
imu_calibration_quality_t imu_calibration_get_quality(void);
/**
 * @brief 按值复制静态窗口均值、加速度标准差、陀螺 RMS 和有效率。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * 传入参数：无。
 * @return 当前 `imu_calibration_static_statistics_t` 副本；物理单位分别为
 *         m/s^2、m/s^2、rad/s 和比例值。
 * @note 调用方式与线程约束：水平校准提交或诊断日志读取；获取可阻塞 mutex，窗口完成前
 *       结果未冻结，禁止从 ISR 调用。
 */
imu_calibration_static_statistics_t imu_calibration_get_static_statistics(void);
/**
 * @brief 查询整体完成且 LSM303 加速度质量门通过。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * 传入参数：无。
 * @return 条件同时满足返回 1，否则返回 0。
 * @note 调用方式与线程约束：启动管理器结束窗口后读取；获取可阻塞 mutex，禁止从 ISR 调用。
 */
uint8_t imu_calibration_is_lsm_complete(void);
/**
 * @brief 查询整体完成且 BMI323 加速度/陀螺质量门均通过。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * 传入参数：无。
 * @return 三项条件同时满足返回 1，否则返回 0。
 * @note 调用方式与线程约束：启动管理器结束窗口后读取；获取可阻塞 mutex，禁止从 ISR 调用。
 */
uint8_t imu_calibration_is_bmi_complete(void);
/**
 * @brief 复制原始快照、应用当前冻结偏置并保存最近一次标定输出。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * @param[in] raw_data 调用方拥有的只读原始快照；NULL 时返回全零快照。
 * @return 应用偏置后的按值副本，物理量和时间戳单位保持输入契约。
 * @note 调用方式与线程约束：IMU manager 发布滤波快照前调用；输入仅在调用期间读取，函数
 *       获取可阻塞 mutex并更新内部 `last_calibrated`，非纯函数，禁止从 ISR 调用。
 */
imu_calibrated_data_t imu_calibration_apply(const imu_raw_data_t *raw_data);
/**
 * @brief 按值复制最近一次 `imu_calibration_apply()` 保存的标定快照。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * 传入参数：无。
 * @return 最近一次标定输出；尚未 apply 或 reset 后返回清零状态。
 * @note 调用方式与线程约束：低频诊断任务读取；获取可阻塞 mutex，不暴露内部存储，
 *       禁止从 ISR 调用。
 */
imu_calibrated_data_t imu_calibration_get_data(void);

#ifdef __cplusplus
}
#endif

#endif /* IMU_CALIBRATION_H */
