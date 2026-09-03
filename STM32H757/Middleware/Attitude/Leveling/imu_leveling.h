#ifndef IMU_LEVELING_H
#define IMU_LEVELING_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 静态窗口水平校准接口；创建人：待确认（当前维护人：Zhiqin）。
 * 本层只计算旋转矩阵和质量原因，不决定是否允许电机运动。
 */

#define IMU_LEVELING_TILT_MAX_DEG (30.0f)
#define IMU_LEVELING_G_MIN (9.0f)
#define IMU_LEVELING_G_MAX (10.5f)
#define IMU_LEVELING_G_DEFAULT_MPS2 \
    ((IMU_LEVELING_G_MIN + IMU_LEVELING_G_MAX) * 0.5f)

/* Static-window admission limits. The gyro gate applies to BMI323; callers
 * without a gyro provide 0.0f and retain the accelerometer-quality gates. */
#define IMU_LEVELING_VALID_RATIO_MIN (0.90f)
#define IMU_LEVELING_GYRO_RMS_MAX_RADPS (0.15f)
#define IMU_LEVELING_ACCEL_STD_MAX_MPS2 (0.15f)

/**
 * @brief 水平矩阵计算的结果或拒绝原因。
 *
 * 枚举值随 `imu_leveling_state_t` 快照保存；失败时实现把 `r_level` 恢复为单位矩阵，
 * 诊断统计字段仍可保留本次被拒绝的输入，调用方必须同时检查 `valid`。
 */
typedef enum
{
    IMU_LEVELING_FALLBACK_NONE = 0, /**< 计算成功，矩阵已通过方向和残差校验。 */
    IMU_LEVELING_FALLBACK_NOT_COMPUTED, /**< 仅完成初始化，尚未尝试计算。 */
    IMU_LEVELING_FALLBACK_SAMPLE_INSUFFICIENT, /**< 有效样本比例低于 0.90。 */
    IMU_LEVELING_FALLBACK_GYRO_NOISE, /**< 陀螺 RMS 超过 0.15 rad/s 静止门限。 */
    IMU_LEVELING_FALLBACK_ACCEL_VARIANCE, /**< 加速度标准差超过调用方指定门限。 */
    IMU_LEVELING_FALLBACK_GRAVITY_OUT_OF_RANGE, /**< 加速度均值模长不在 9.0..10.5 m/s^2。 */
    IMU_LEVELING_FALLBACK_TILT_EXCEEDED, /**< 起始倾角超过 30 度。 */
    IMU_LEVELING_FALLBACK_INVERTED_START, /**< 加速度方向与目标 +Z 夹角不小于 90 度。 */
    IMU_LEVELING_FALLBACK_NONFINITE, /**< 向量/统计非有限，或 RMS、标准差、门限为负。 */
    IMU_LEVELING_FALLBACK_MATRIX_INVALID /**< 旋转矩阵行列式非正/非有限或对齐残差超限。 */
} imu_leveling_fallback_reason_t;

/**
 * @brief 一次静态窗口水平校准的结果与诊断快照。
 *
 * 对象由调用方拥有，计算函数原地覆盖；类型内不含动态资源。只有 `valid=true` 时
 * `r_level` 才是已接受的水平变换，否则矩阵为安全单位阵，其他统计仅供诊断。
 */
typedef struct
{
    float r_level[3][3]; /**< 将传感器坐标三维列向量旋转到重力对齐 +Z 坐标系的 3x3 矩阵。 */
    float g_local_mps2; /**< 本窗口加速度均值模长，单位 m/s^2；计算前为默认值，失败时也可能保留拒绝值。 */
    float tilt_deg; /**< 由加速度均值估计的起始倾角，单位度；早期质量门失败时保持初始化值。 */
    float accel_mean[3]; /**< 调用方提交的三轴加速度均值副本，单位 m/s^2，失败时仍供诊断。 */
    float gyro_rms_radps; /**< 调用方提交的陀螺 RMS，单位 rad/s；无陀螺策略传 0。 */
    float accel_std_mps2; /**< 调用方提交的加速度标准差，单位 m/s^2。 */
    float valid_ratio; /**< 静态窗口有效样本比例；当前准入下限为 0.90。 */
    bool valid; /**< true 表示矩阵已通过全部质量/几何校验；false 时不得视为已完成水平校准。 */
    imu_leveling_fallback_reason_t fallback_reason; /**< 成功为 NONE，否则记录首个拒绝/未计算原因。 */
} imu_leveling_state_t;

/**
 * @brief 初始化水平校准状态、单位矩阵和默认本地重力参考。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * @param[out] state 调用方拥有的状态对象；NULL 时函数直接返回。
 * @return 无返回值。
 * @note 调用方式与线程约束：新静态窗口或状态复位时调用；纯内存操作、不阻塞且可重入，
 *       同一 `state` 不得被多个上下文并发写入。
 */
void imu_leveling_init(imu_leveling_state_t *state);
/**
 * @brief 使用默认加速度标准差门限计算静态水平旋转矩阵。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * @param[out] state 调用方拥有的计算结果；失败时写入单位矩阵和具体 fallback 原因。
 * @param[in] accel_mean 静态窗口三轴加速度均值，单位 m/s^2，至少 3 个元素。
 * @param[in] gyro_rms 静态窗口陀螺 RMS，单位 rad/s；无陀螺的传感器传 0。
 * @param[in] accel_std 加速度模长/统计标准差，单位 m/s^2，必须非负且有限。
 * @param[in] valid_ratio 有效样本比例；实现要求不低于最小门限。
 * @return true 表示 `state->r_level` 通过质量、倾角和矩阵残差校验；false 表示
 *         输入/质量不合格，原因见 `state->fallback_reason`。
 * @note 调用方式与线程约束：静态标定窗口结束后在任务上下文调用；纯计算、不访问硬件、
 *       不阻塞且可重入，输入只在调用期间读取。
 */
bool imu_leveling_compute(imu_leveling_state_t *state,
                          const float accel_mean[3],
                          float gyro_rms,
                          float accel_std,
                          float valid_ratio);
/* Explicit sensor policy entry. The legacy entry retains the default limit
 * for any caller that does not own a sensor-specific noise budget. */
/**
 * @brief 使用调用方指定的加速度噪声门限计算静态水平旋转矩阵。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * @param[out] state 调用方拥有的计算结果；不可为 NULL。
 * @param[in] accel_mean 静态窗口三轴加速度均值，单位 m/s^2，至少 3 个元素。
 * @param[in] gyro_rms 静态窗口陀螺 RMS，单位 rad/s，必须非负且有限。
 * @param[in] accel_std 加速度标准差，单位 m/s^2，必须非负且有限。
 * @param[in] valid_ratio 有效样本比例；实现要求不低于最小门限。
 * @param[in] accel_std_max 本传感器允许的最大加速度标准差，单位 m/s^2，
 *                          必须非负且有限。
 * @return true 表示矩阵有效；false 时 `state` 仍被初始化并记录 fallback 原因。
 * @note 调用方式与线程约束：由 IMU manager 在冻结静态统计后调用；纯计算、不阻塞且可重入，
 *       输入数组不被保存，同一 `state` 不得并发写入。
 */
bool imu_leveling_compute_with_accel_std_limit(imu_leveling_state_t *state,
                                               const float accel_mean[3],
                                               float gyro_rms,
                                               float accel_std,
                                               float valid_ratio,
                                               float accel_std_max);
/**
 * @brief 使用给定水平矩阵旋转三维向量。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * @param[in] state 水平校准状态；NULL 时按恒等变换复制输入。函数不检查
 *                  `state->valid`，调用方负责只提交已接受状态。
 * @param[in] v_in 三元素输入向量；调用期间只读，不允许为 NULL。
 * @param[out] v_out 三元素输出向量；不允许为 NULL，可与 `v_in` 指向同一数组。
 * @return 无返回值；任一向量指针为 NULL 时不写输出。
 * @note 调用方式与线程约束：采样/融合任务可对加速度、角速度或磁场调用；纯计算、不阻塞、
 *       不保存指针且可重入。
 */
void imu_leveling_rotate_vector(const imu_leveling_state_t *state,
                                const float v_in[3],
                                float v_out[3]);

#ifdef __cplusplus
}
#endif

#endif /* IMU_LEVELING_H */
