#include "dual_ahrs.h"

#include <math.h>
#include <string.h>

#include "imu_time.h"

/* DualAHRS 融合实现；创建人：待确认（当前维护人：Zhiqin）。 */

#define DUAL_AHRS_MIN_DT 0.0005f
#define DUAL_AHRS_MAX_DT 0.0200f
#define DUAL_AHRS_LSM_STALE_US UINT64_C(250000)
#define DUAL_AHRS_PRIMARY_STALE_US UINT64_C(50000)
#define DUAL_AHRS_LSM_PAIR_MAX_AGE_US UINT64_C(100000)
#define DUAL_AHRS_MAG_ALPHA 0.20f
#define DUAL_AHRS_PRIMARY_KP 1.8f
#define DUAL_AHRS_PRIMARY_KI 0.035f
#define DUAL_AHRS_ACCEL_LPF_HZ 20.0f
#define DUAL_AHRS_GYRO_LPF_HZ 80.0f
#define DUAL_AHRS_SAMPLE_HZ 200.0f

typedef struct
{
    float b0;
    float b1;
    float b2;
    float a1;
    float a2;
    float x1;
    float x2;
    float y1;
    float y2;
} dual_ahrs_biquad_t;

typedef struct
{
    dual_ahrs_biquad_t accel_lpf[3];
    dual_ahrs_biquad_t gyro_lpf[3];
    uint64_t last_bmi_timestamp_us;
    dual_ahrs_vector3_t primary_gyro_prev;
    uint8_t primary_gyro_prev_valid;
    float primary_gyro_z_rad_s;
    uint64_t last_lsm_timestamp_us;
    uint64_t last_lsm_accel_input_timestamp_us;
    uint64_t last_lsm_mag_input_timestamp_us;
    uint64_t last_mag_reference_timestamp_us;
    dual_ahrs_quaternion_t primary_q;
    dual_ahrs_vector3_t primary_integral;
    dual_ahrs_attitude_t primary;
    dual_ahrs_attitude_t redundant;
    dual_ahrs_vector3_t redundant_accel;
    dual_ahrs_vector3_t redundant_mag;
    dual_ahrs_vector3_t accel_history[5];
    dual_ahrs_vector3_t mag_history[5];
    uint8_t history_count;
    uint8_t history_index;
    float mag_reference_norm;
    dual_ahrs_bias_t bias;
    imu_leveling_state_t leveling_bmi;
    imu_leveling_state_t leveling_lsm;
    float local_gravity_mps2;
    float primary_yaw_offset;
    float redundant_yaw_offset;
    uint8_t primary_yaw_offset_valid;
    uint8_t redundant_yaw_offset_valid;
    uint8_t primary_zero_pending;
    uint8_t redundant_zero_pending;
    uint8_t bias_valid;
    uint32_t sample_sequence;
    dual_ahrs_output_t output;
    dual_ahrs_state_t state;
} dual_ahrs_context_t;

static dual_ahrs_context_t s_dual;

/**
 * @brief 解析 LSM303 加速度样本的有效微秒时间戳，并兼容旧共享时间戳字段。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] input 调用期间只读的 DualAHRS 输入；允许为 NULL，不保存该指针。
 * @return 优先返回非零 `lsm_accel_timestamp_us`；该字段为零但加速度有效时返回
 *         `lsm_timestamp_us`；NULL 或无有效加速度且无独立时间戳时返回 0。
 * 调用方式：由静态姿态初始化、冗余分支更新和 `dual_ahrs_update()` 统一解析时间契约。
 * 线程约束：纯只读计算、不阻塞、不获取 mutex 且可重入；禁止从 ISR 调用，输入对象
 *           所有权始终归调用方。
 */
static uint64_t input_lsm_accel_timestamp(const dual_ahrs_input_t *input)
{
    if (input == NULL) {
        return 0U;
    }
    if (input->lsm_accel_timestamp_us != 0U) {
        return input->lsm_accel_timestamp_us;
    }
    return input->lsm_accel_valid != 0U ? input->lsm_timestamp_us : 0U;
}

/**
 * @brief 解析 LSM303 磁场样本的有效微秒时间戳，并兼容旧共享时间戳字段。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] input 调用期间只读的 DualAHRS 输入；允许为 NULL，不保存该指针。
 * @return 优先返回非零 `lsm_mag_timestamp_us`；该字段为零但磁场有效时返回
 *         `lsm_timestamp_us`；NULL 或无有效磁场且无独立时间戳时返回 0。
 * 调用方式：由静态姿态初始化、冗余分支更新和 `dual_ahrs_update()` 统一解析时间契约。
 * 线程约束：纯只读计算、不阻塞、不获取 mutex 且可重入；禁止从 ISR 调用，输入对象
 *           所有权始终归调用方。
 */
static uint64_t input_lsm_mag_timestamp(const dual_ahrs_input_t *input)
{
    if (input == NULL) {
        return 0U;
    }
    if (input->lsm_mag_timestamp_us != 0U) {
        return input->lsm_mag_timestamp_us;
    }
    return input->lsm_mag_valid != 0U ? input->lsm_timestamp_us : 0U;
}

/**
 * @brief 判断一对 LSM303 加速度/磁场时间戳是否落在允许的配对时间差内。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] accel_timestamp_us 加速度样本时间戳，单位 us；本函数不单独拒绝 0。
 * @param[in] mag_timestamp_us 磁场样本时间戳，单位 us；本函数不单独拒绝 0。
 * @return 两时间戳绝对差小于等于 `DUAL_AHRS_LSM_PAIR_MAX_AGE_US` 时返回 1，
 *         否则返回 0；零值有效性由上层调用点检查。
 * 调用方式：由静态冗余姿态初始化和每次冗余分支更新在消费样本前调用。
 * 线程约束：纯整数计算、不阻塞、不获取 mutex 且可重入；禁止从 ISR 调用，无指针或
 *           所有权转移。
 */
static uint8_t lsm_timestamps_are_coherent(uint64_t accel_timestamp_us,
                                            uint64_t mag_timestamp_us)
{
    const uint64_t difference = accel_timestamp_us >= mag_timestamp_us
                                    ? accel_timestamp_us - mag_timestamp_us
                                    : mag_timestamp_us - accel_timestamp_us;
    return difference <= DUAL_AHRS_LSM_PAIR_MAX_AGE_US ? 1U : 0U;
}

/**
 * @brief 按当前时间判断主 BMI323 姿态输出是否有效且未超过 freshness 门限。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] now_us 调用方取得的当前单调时间，单位 us。
 * @return 主输出有效、最近 BMI 时间戳非零、时间未倒退且年龄严格小于 50000 us
 *         时返回 1；任一条件不满足返回 0。
 * 调用方式：由输出 getter、航向控制 getter 和主分支 freshness 公共查询同步调用。
 * 线程约束：只读访问 `s_dual`，不阻塞且不获取 mutex；当前实现不是一致性快照，必须与
 *           单 writer 约束配合，禁止从 ISR 调用，无对象所有权转移。
 */
static uint8_t primary_is_fresh(uint64_t now_us)
{
    if (s_dual.output.primary.valid == 0U ||
        s_dual.last_bmi_timestamp_us == 0U ||
        now_us < s_dual.last_bmi_timestamp_us ||
        (now_us - s_dual.last_bmi_timestamp_us) >=
            DUAL_AHRS_PRIMARY_STALE_US) {
        return 0U;
    }
    return 1U;
}

/**
 * @brief 按当前时间判断冗余 LSM303 姿态输出是否有效且未超过 freshness 门限。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] now_us 调用方取得的当前单调时间，单位 us。
 * @return 冗余输出有效、最近成对 LSM 时间戳非零、时间未倒退且年龄严格小于
 *         250000 us 时返回 1；任一条件不满足返回 0。
 * 调用方式：仅由 `dual_ahrs_get_output()` 在复制输出后重新施加 freshness 语义。
 * 线程约束：只读访问 `s_dual`，不阻塞且不获取 mutex；当前实现不是一致性快照，必须与
 *           单 writer 约束配合，禁止从 ISR 调用，无对象所有权转移。
 */
static uint8_t redundant_is_fresh(uint64_t now_us)
{
    if (s_dual.output.redundant.valid == 0U ||
        s_dual.last_lsm_timestamp_us == 0U ||
        now_us < s_dual.last_lsm_timestamp_us ||
        (now_us - s_dual.last_lsm_timestamp_us) >=
            DUAL_AHRS_LSM_STALE_US) {
        return 0U;
    }
    return 1U;
}

/**
 * @brief 清空 DualAHRS 运行历史、姿态、零参考和输出，同时保留配置与已提交标定。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * 传入参数：无。
 * @return 无返回值；函数无失败分支，保留滤波系数、bias/leveling/重力配置及
 *         `bias_valid`，状态枚举由调用者随后设置。
 * 调用方式：由 `dual_ahrs_init()` 和 `dual_ahrs_set_bias()` 在初始化或偏置切换时调用。
 * 线程约束：写入整个 `s_dual` 运行区且无内部 mutex，非线程安全、不阻塞；仅生命周期
 *           owner 在任务上下文串行调用，禁止 ISR/并发 reader-writer，无所有权转移。
 */
static void reset_runtime_state(void)
{
    uint8_t index;

    for (index = 0U; index < 3U; ++index) {
        s_dual.accel_lpf[index].x1 = 0.0f;
        s_dual.accel_lpf[index].x2 = 0.0f;
        s_dual.accel_lpf[index].y1 = 0.0f;
        s_dual.accel_lpf[index].y2 = 0.0f;
        s_dual.gyro_lpf[index].x1 = 0.0f;
        s_dual.gyro_lpf[index].x2 = 0.0f;
        s_dual.gyro_lpf[index].y1 = 0.0f;
        s_dual.gyro_lpf[index].y2 = 0.0f;
    }
    s_dual.last_bmi_timestamp_us = 0U;
    s_dual.primary_gyro_prev = (dual_ahrs_vector3_t){0.0f, 0.0f, 0.0f};
    s_dual.primary_gyro_prev_valid = 0U;
    s_dual.primary_gyro_z_rad_s = 0.0f;
    s_dual.last_lsm_timestamp_us = 0U;
    s_dual.last_lsm_accel_input_timestamp_us = 0U;
    s_dual.last_lsm_mag_input_timestamp_us = 0U;
    s_dual.last_mag_reference_timestamp_us = 0U;
    s_dual.primary_q = (dual_ahrs_quaternion_t){1.0f, 0.0f, 0.0f, 0.0f};
    s_dual.primary_integral = (dual_ahrs_vector3_t){0.0f, 0.0f, 0.0f};
    s_dual.primary = (dual_ahrs_attitude_t){0};
    s_dual.primary.quaternion = s_dual.primary_q;
    s_dual.redundant = (dual_ahrs_attitude_t){0};
    s_dual.redundant.quaternion =
        (dual_ahrs_quaternion_t){1.0f, 0.0f, 0.0f, 0.0f};
    s_dual.redundant_accel = (dual_ahrs_vector3_t){0.0f, 0.0f, 0.0f};
    s_dual.redundant_mag = (dual_ahrs_vector3_t){0.0f, 0.0f, 0.0f};
    (void)memset(s_dual.accel_history, 0, sizeof(s_dual.accel_history));
    (void)memset(s_dual.mag_history, 0, sizeof(s_dual.mag_history));
    s_dual.history_count = 0U;
    s_dual.history_index = 0U;
    s_dual.mag_reference_norm = 0.0f;
    s_dual.primary_yaw_offset = 0.0f;
    s_dual.redundant_yaw_offset = 0.0f;
    s_dual.primary_yaw_offset_valid = 0U;
    s_dual.redundant_yaw_offset_valid = 0U;
    s_dual.primary_zero_pending = 1U;
    s_dual.redundant_zero_pending = 1U;
    s_dual.sample_sequence = 0U;
    (void)memset(&s_dual.output, 0, sizeof(s_dual.output));
    s_dual.output.schema = DUAL_AHRS_SCHEMA;
    s_dual.output.primary.quaternion = s_dual.primary_q;
    s_dual.output.redundant.quaternion = s_dual.redundant.quaternion;
}

/**
 * @brief 检查按值传入的三维向量三个分量是否全部为有限数。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] value 待检查的三维向量，单位由调用方字段决定。
 * @return 三个分量均通过 `isfinite()` 时返回 1，否则返回 0。
 * 调用方式：由 `dual_ahrs_set_bias()` 校验 BMI/LSM 偏置后再接纳标定结果。
 * 线程约束：纯计算、不阻塞、不获取 mutex 且可重入；禁止从 ISR 调用，参数按值传递，
 *           不涉及对象所有权。
 */
static uint8_t vector_is_finite(dual_ahrs_vector3_t value)
{
    return (uint8_t)(isfinite(value.x) != 0 && isfinite(value.y) != 0 &&
                     isfinite(value.z) != 0);
}

/**
 * @brief 使用给定冻结水平矩阵旋转一个 DualAHRS 三维向量。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] leveling 调用期间借用的水平校准状态；NULL 时底层执行恒等复制，函数不检查
 *                     `valid` 标志且不保存指针。
 * @param[in] vector 按值传入的加速度、角速度或磁场三维向量。
 * @return 返回 `leveling->r_level` 旋转后的向量；NULL 状态返回原向量，非有限值按浮点
 *         规则传播，无独立失败码。
 * 调用方式：仅由 `dual_ahrs_update()` 在去偏置后、所有滤波/估计前调用四次。
 * 线程约束：只读状态并使用栈上数组，不阻塞、不获取 mutex；任务上下文可重入，但同一
 *           `leveling` 不得并发写入，禁止 ISR 调用，所有权仍归调用方。
 */
static dual_ahrs_vector3_t level_vector(const imu_leveling_state_t *leveling,
                                        dual_ahrs_vector3_t vector)
{
    const float input[3] = {vector.x, vector.y, vector.z};
    float output[3];

    imu_leveling_rotate_vector(leveling, input, output);
    return (dual_ahrs_vector3_t){output[0], output[1], output[2]};
}

/**
 * @brief 将浮点值限制到闭区间 [0, 1]，并把非有限值映射为 0。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] value 待限制的标量。
 * @return 非有限或小于等于 0 返回 0，大于等于 1 返回 1，其余原值返回；无失败码。
 * 调用方式：由重力和磁场 confidence 计算在返回公共结果前调用。
 * 线程约束：纯计算、不阻塞、不获取 mutex 且可重入；禁止从 ISR 调用，按值传参，无
 *           对象所有权或生命周期要求。
 */
static float clamp01(float value)
{
    if (!isfinite(value) || value <= 0.0f) {
        return 0.0f;
    }
    return value >= 1.0f ? 1.0f : value;
}

/**
 * @brief 计算按值传入三维向量的欧氏模长。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] value 待计算向量，单位由调用路径决定。
 * @return 返回 `sqrt(x*x+y*y+z*z)`；非有限分量或溢出会传播为非有限结果，无错误码。
 * 调用方式：供归一化、confidence 与磁场参考更新路径同步复用。
 * 线程约束：纯计算、不阻塞、不获取 mutex 且可重入；禁止从 ISR 调用，参数按值传递，
 *           不涉及对象所有权。
 */
static float vector_norm(dual_ahrs_vector3_t value)
{
    return sqrtf(value.x * value.x + value.y * value.y + value.z * value.z);
}

/**
 * @brief 将三维向量的每个分量乘以同一标量。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] value 按值传入的三维向量。
 * @param[in] scale 缩放系数。
 * @return 返回逐分量缩放结果；非有限输入或溢出按浮点规则传播，无错误码。
 * 调用方式：由向量归一化及主 Mahony 重力误差加权路径同步调用。
 * 线程约束：纯计算、不阻塞、不获取 mutex 且可重入；禁止从 ISR 调用，参数按值传递，
 *           不涉及对象所有权。
 */
static dual_ahrs_vector3_t vector_scale(dual_ahrs_vector3_t value, float scale)
{
    dual_ahrs_vector3_t result = {value.x * scale, value.y * scale,
                                  value.z * scale};
    return result;
}

/**
 * @brief 计算两个三维向量的右手系叉积 `a x b`。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] a 左操作数三维向量。
 * @param[in] b 右操作数三维向量。
 * @return 返回叉积向量；非有限输入或溢出按浮点规则传播，无错误码。
 * 调用方式：由 `update_primary()` 计算实测重力与四元数预测重力的 Mahony 误差。
 * 线程约束：纯计算、不阻塞、不获取 mutex 且可重入；禁止从 ISR 调用，参数按值传递，
 *           不涉及对象所有权。
 */
static dual_ahrs_vector3_t vector_cross(dual_ahrs_vector3_t a,
                                        dual_ahrs_vector3_t b)
{
    dual_ahrs_vector3_t result = {a.y * b.z - a.z * b.y,
                                  a.z * b.x - a.x * b.z,
                                  a.x * b.y - a.y * b.x};
    return result;
}

/**
 * @brief 对三维向量归一化，并可选输出归一化有效标志。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] value 按值传入的三维向量。
 * @param[out] valid 可选的调用方标志地址；非 NULL 时，有限且模长大于 `1e-6` 写 1，
 *                   否则写 0，函数不保存该指针。
 * @return 成功返回单位向量；模长非有限或过小时返回全零向量，未提供 `valid` 时调用方
 *         只能从返回值推断失败。
 * 调用方式：供静态姿态构造以及主、冗余估计器在使用传感器向量前调用。
 * 线程约束：纯计算、不阻塞、不获取 mutex 且可重入；禁止从 ISR 调用，输出地址所有权
 *           始终归调用方且不得指向并发写区域。
 */
static dual_ahrs_vector3_t vector_normalize(dual_ahrs_vector3_t value,
                                            uint8_t *valid)
{
    const float norm = vector_norm(value);
    if (valid != NULL) {
        *valid = (isfinite(norm) && norm > 1.0e-6f) ? 1U : 0U;
    }
    if (!isfinite(norm) || norm <= 1.0e-6f) {
        return (dual_ahrs_vector3_t){0.0f, 0.0f, 0.0f};
    }
    return vector_scale(value, 1.0f / norm);
}

/** 将角度包装到 [-pi, pi)，供航向误差计算复用。 */
float dual_ahrs_wrap_pi(float angle)
{
    if (!isfinite(angle)) {
        return 0.0f;
    }
    while (angle > DUAL_AHRS_PI) {
        angle -= DUAL_AHRS_TWO_PI;
    }
    while (angle < -DUAL_AHRS_PI) {
        angle += DUAL_AHRS_TWO_PI;
    }
    return angle;
}

/** 计算横滚差并处理角度环绕。 */
float delta_roll(float primary, float redundant)
{
    return dual_ahrs_wrap_pi(primary - redundant);
}

/** 计算俯仰差并处理角度环绕。 */
float delta_pitch(float primary, float redundant)
{
    return dual_ahrs_wrap_pi(primary - redundant);
}

/** 计算航向差并处理角度环绕。 */
float delta_yaw(float primary, float redundant)
{
    return dual_ahrs_wrap_pi(primary - redundant);
}

/** 根据加速度模长计算重力可信度。 */
float gravity_confidence(const dual_ahrs_vector3_t *accel)
{
    float local_gravity = s_dual.local_gravity_mps2;
    float norm_error;

    if (accel == NULL) {
        return 0.0f;
    }
    if (!isfinite(local_gravity) || local_gravity < IMU_LEVELING_G_MIN ||
        local_gravity > IMU_LEVELING_G_MAX) {
        local_gravity = IMU_LEVELING_G_DEFAULT_MPS2;
    }
    norm_error = fabsf(vector_norm(*accel) - local_gravity) / local_gravity;
    return clamp01(1.0f - (4.0f * norm_error));
}

/** 根据磁场模长计算磁力可信度。 */
float mag_confidence(const dual_ahrs_vector3_t *mag)
{
    const float norm = mag == NULL ? 0.0f : vector_norm(*mag);
    float deviation;

    if (!isfinite(norm) || norm <= 1.0e-6f) {
        return 0.0f;
    }
    if (s_dual.mag_reference_norm <= 1.0e-6f) {
        return 1.0f;
    }
    deviation = fabsf(norm - s_dual.mag_reference_norm) /
                s_dual.mag_reference_norm;
    return clamp01(1.0f - (3.0f * deviation));
}

/**
 * @brief 为固定 200 Hz 采样率构造二阶 Butterworth 低通滤波器系数和零历史。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] frequency 截止频率，单位 Hz；当前调用点仅传入预定义的 20 Hz 或 80 Hz。
 * @return 返回按 Q=1/sqrt(2) 计算的 biquad 对象，历史状态为 0；函数不校验频率，非法
 *         或非有限输入会产生非有限/不适用系数且无错误码。
 * 调用方式：仅由 `dual_ahrs_init()` 为三轴加速度和陀螺滤波器逐轴构造一次。
 * 线程约束：纯计算、不阻塞、不获取 mutex 且可重入；禁止从 ISR 调用，按值返回对象，
 *           所有权由调用方接收。
 */
static dual_ahrs_biquad_t make_lpf(float frequency)
{
    const float omega = 2.0f * DUAL_AHRS_PI * frequency / DUAL_AHRS_SAMPLE_HZ;
    const float cos_omega = cosf(omega);
    const float sin_omega = sinf(omega);
    const float alpha = sin_omega / (2.0f * 0.70710678f);
    const float a0 = 1.0f + alpha;
    dual_ahrs_biquad_t filter = {
        .b0 = ((1.0f - cos_omega) * 0.5f) / a0,
        .b1 = (1.0f - cos_omega) / a0,
        .b2 = ((1.0f - cos_omega) * 0.5f) / a0,
        .a1 = (-2.0f * cos_omega) / a0,
        .a2 = (1.0f - alpha) / a0,
    };
    return filter;
}

/**
 * @brief 对单轴样本执行一次 Direct Form I biquad，并推进该滤波器的两级历史。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in,out] filter 调用方拥有的滤波器状态；成功时更新 `x1/x2/y1/y2`，允许为 NULL。
 * @param[in] input 当前单轴样本，单位沿用该滤波器的数据流。
 * @return 成功返回当前滤波输出；`filter==NULL` 或输入非有限时返回 0 且不更新历史；
 *         系数或计算结果非有限时仍会写入历史，没有额外错误码。
 * 调用方式：仅由 `filter_vector()` 对同组连续三个轴状态各调用一次。
 * 线程约束：修改 `filter` 指向状态，不阻塞、不获取 mutex；同一滤波器仅允许 IMU 单
 *           writer 在任务上下文调用，禁止 ISR/并发调用，不保存或接管指针所有权。
 */
static float biquad_apply(dual_ahrs_biquad_t *filter, float input)
{
    float output;

    if (filter == NULL || !isfinite(input)) {
        return 0.0f;
    }
    output = filter->b0 * input + filter->b1 * filter->x1 +
             filter->b2 * filter->x2 - filter->a1 * filter->y1 -
             filter->a2 * filter->y2;
    filter->x2 = filter->x1;
    filter->x1 = input;
    filter->y2 = filter->y1;
    filter->y1 = output;
    return output;
}

/**
 * @brief 使用三份独立 biquad 状态逐轴过滤一个三维向量。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] value 当前三轴输入向量，单位由调用方决定。
 * @param[in,out] lpf 至少包含三项的连续滤波器数组；不可为 NULL，逐项历史会被推进。
 * @return 返回三个单轴滤波输出；某一输入非有限时该轴返回 0 且该轴历史不变，非法
 *         `lpf` 指针没有防御性失败输出。
 * 调用方式：由 `dual_ahrs_update()` 对 BMI323 加速度和陀螺各调用一次，再进入主估计器。
 * 线程约束：修改调用方滤波状态且无内部 mutex，非线程安全、不阻塞；仅 IMU 单 writer
 *           任务调用，禁止 ISR/并发调用，数组所有权仍归调用方。
 */
static dual_ahrs_vector3_t filter_vector(dual_ahrs_vector3_t value,
                                          dual_ahrs_biquad_t *lpf)
{
    dual_ahrs_vector3_t result;
    result.x = biquad_apply(&lpf[0], value.x);
    result.y = biquad_apply(&lpf[1], value.y);
    result.z = biquad_apply(&lpf[2], value.z);
    return result;
}

/**
 * @brief 将四元数归一化，并为退化或非有限输入提供单位四元数回退。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] q 按值传入的待归一化四元数。
 * @return 模长有限且大于 `1e-6` 时返回单位化结果；否则返回 `{1,0,0,0}`，不提供
 *         独立错误标志。
 * 调用方式：由欧拉角转换和主 Mahony 积分在提交四元数前调用。
 * 线程约束：纯计算、不阻塞、不获取 mutex 且可重入；禁止从 ISR 调用，按值传参与返回，
 *           不涉及对象所有权。
 */
static dual_ahrs_quaternion_t quaternion_normalize(dual_ahrs_quaternion_t q)
{
    const float norm = sqrtf(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    if (!isfinite(norm) || norm <= 1.0e-6f) {
        return (dual_ahrs_quaternion_t){1.0f, 0.0f, 0.0f, 0.0f};
    }
    q.w /= norm;
    q.x /= norm;
    q.y /= norm;
    q.z /= norm;
    return q;
}

/**
 * @brief 把四元数转换为 ZYX 欧拉角姿态并将结果标记为有效。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] q 调用方应保证已归一化且有限的四元数。
 * @return 返回包含原四元数、rad 制 roll/pitch/yaw 且 `valid=1` 的姿态；函数不验证
 *         `q`，非有限输入可能产生非有限角度但仍标记有效。
 * 调用方式：仅由 `update_primary()` 在成功积分并归一化主四元数后调用。
 * 线程约束：纯计算、不阻塞、不获取 mutex 且可重入；禁止从 ISR 调用，按值传参与返回，
 *           返回对象所有权归调用方。
 */
static dual_ahrs_attitude_t quaternion_to_attitude(dual_ahrs_quaternion_t q)
{
    dual_ahrs_attitude_t result;
    const float sin_pitch = 2.0f * (q.w * q.y - q.z * q.x);
    result.quaternion = q;
    result.roll = atan2f(2.0f * (q.w * q.x + q.y * q.z),
                         1.0f - 2.0f * (q.x * q.x + q.y * q.y));
    result.pitch = asinf(fmaxf(-1.0f, fminf(1.0f, sin_pitch)));
    result.yaw = atan2f(2.0f * (q.w * q.z + q.x * q.y),
                        1.0f - 2.0f * (q.y * q.y + q.z * q.z));
    result.valid = 1U;
    return result;
}

/**
 * @brief 将 ZYX 欧拉角转换为归一化四元数。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] roll 横滚角，单位 rad。
 * @param[in] pitch 俯仰角，单位 rad。
 * @param[in] yaw 航向角，单位 rad。
 * @return 返回归一化四元数；任一角非有限导致计算退化时由归一化 helper 回退为单位
 *         四元数，不提供独立错误标志。
 * 调用方式：仅由 `sync_attitude_quaternion()` 在最终欧拉角被修改后同步序列化表示。
 * 线程约束：纯计算、不阻塞、不获取 mutex 且可重入；禁止从 ISR 调用，参数按值传递，
 *           返回对象所有权归调用方。
 */
static dual_ahrs_quaternion_t euler_to_quaternion(float roll, float pitch,
                                                   float yaw)
{
    const float cr = cosf(roll * 0.5f);
    const float sr = sinf(roll * 0.5f);
    const float cp = cosf(pitch * 0.5f);
    const float sp = sinf(pitch * 0.5f);
    const float cy = cosf(yaw * 0.5f);
    const float sy = sinf(yaw * 0.5f);
    dual_ahrs_quaternion_t q = {
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    };
    return quaternion_normalize(q);
}

/**
 * @brief 依据姿态中的最终欧拉角重建并覆盖其四元数字段。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in,out] attitude 调用方拥有的姿态；非 NULL 且 `valid!=0` 时仅覆盖
 *                         `quaternion`，允许为 NULL。
 * @return 无返回值；NULL 或无效姿态时不写入，非有限欧拉角会通过转换 helper 产生单位
 *         四元数且没有错误上报。
 * 调用方式：由静态姿态构造、冗余更新和零参考输出路径在欧拉角确定后调用。
 * 线程约束：修改调用方对象、不阻塞、不获取 mutex；同一对象不得并发访问，禁止从 ISR
 *           调用，不保存或接管对象所有权。
 */
static void sync_attitude_quaternion(dual_ahrs_attitude_t *attitude)
{
    if (attitude == NULL || attitude->valid == 0U) {
        return;
    }
    attitude->quaternion = euler_to_quaternion(attitude->roll,
                                                attitude->pitch,
                                                attitude->yaw);
}

/**
 * @brief 由加速度和磁场向量构造倾斜补偿的静态绝对姿态。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] accel 已映射并水平变换的加速度向量，单位 m/s^2。
 * @param[in] mag 已映射并水平变换的磁场向量，单位 uT。
 * @param[out] attitude 调用方拥有的输出对象；不可为 NULL，成功时写入欧拉角、四元数和
 *                      `valid=1`。
 * @return 成功返回 1；NULL 输出、任一向量不可归一化或水平磁场退化返回 0。水平磁场
 *         退化分支可能已写 roll/pitch，但不会把结果标记有效，调用方不得使用失败输出。
 * 调用方式：仅由 `initialize_static_attitudes()` 建立 LSM303 冗余分支初始姿态。
 * 线程约束：纯计算并修改调用方输出，不阻塞、不获取 mutex；任务上下文可重入，禁止 ISR
 *           调用，同一输出对象不得并发写，函数不保存指针。
 */
static uint8_t attitude_from_accel_mag(dual_ahrs_vector3_t accel,
                                       dual_ahrs_vector3_t mag,
                                       dual_ahrs_attitude_t *attitude)
{
    dual_ahrs_vector3_t accel_unit;
    uint8_t accel_valid;
    uint8_t mag_valid;
    float horizontal_x;
    float horizontal_y;

    if (attitude == NULL) {
        return 0U;
    }
    accel_unit = vector_normalize(accel, &accel_valid);
    (void)vector_normalize(mag, &mag_valid);
    if (accel_valid == 0U || mag_valid == 0U) {
        return 0U;
    }

    attitude->roll = atan2f(accel_unit.y, accel_unit.z);
    attitude->pitch = atan2f(-accel_unit.x,
                             sqrtf(accel_unit.y * accel_unit.y +
                                   accel_unit.z * accel_unit.z));
    horizontal_x = mag.x * cosf(attitude->pitch) +
                   mag.z * sinf(attitude->pitch);
    horizontal_y = mag.x * sinf(attitude->roll) * sinf(attitude->pitch) +
                   mag.y * cosf(attitude->roll) -
                   mag.z * sinf(attitude->roll) * cosf(attitude->pitch);
    if (!isfinite(horizontal_x) || !isfinite(horizontal_y) ||
        (horizontal_x * horizontal_x + horizontal_y * horizontal_y) <=
            1.0e-12f) {
        return 0U;
    }

    /* The input magnetic vector is already in the vehicle Body Frame, so this
     * is the same tilt-compensated heading used by both estimators. */
    /* Body X points forward and Body Y points right, so heading increases
     * with the positive horizontal Body Y component. */
    attitude->yaw = dual_ahrs_wrap_pi(atan2f(horizontal_y, horizontal_x));
    attitude->valid = 1U;
    sync_attitude_quaternion(attitude);
    return 1U;
}

/**
 * @brief 仅由加速度构造主 6 轴估计器的静态 roll/pitch，并把初始 yaw 设为 0。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] accel 已去偏置并水平变换的 BMI323 加速度，单位 m/s^2。
 * @param[out] attitude 调用方拥有的输出对象；不可为 NULL，成功时写入欧拉角、四元数和
 *                      `valid=1`。
 * @return 成功返回 1；NULL 输出或加速度不可归一化返回 0 且不产生可用姿态。
 * 调用方式：仅由 `initialize_static_attitudes()` 为无磁主分支建立相对航向初值。
 * 线程约束：纯计算并修改调用方输出，不阻塞、不获取 mutex；任务上下文可重入，禁止 ISR
 *           调用，同一输出对象不得并发写，函数不保存指针。
 */
static uint8_t attitude_from_accel(dual_ahrs_vector3_t accel,
                                   dual_ahrs_attitude_t *attitude)
{
    dual_ahrs_vector3_t accel_unit;
    uint8_t accel_valid;

    if (attitude == NULL) {
        return 0U;
    }
    accel_unit = vector_normalize(accel, &accel_valid);
    if (accel_valid == 0U) {
        return 0U;
    }

    attitude->roll = atan2f(accel_unit.y, accel_unit.z);
    attitude->pitch = atan2f(-accel_unit.x,
                             sqrtf(accel_unit.y * accel_unit.y +
                                   accel_unit.z * accel_unit.z));
    /* A 6-axis estimator has no absolute heading. Start yaw at zero and let
     * the calibrated gyro integrate relative heading from this reference. */
    attitude->yaw = 0.0f;
    attitude->valid = 1U;
    sync_attitude_quaternion(attitude);
    return 1U;
}

/**
 * @brief 用常量样本预置单轴 biquad 的输入/输出历史，避免首次滤波从零过渡。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in,out] filter 调用方拥有的滤波器；成功时覆盖 `x1/x2/y1/y2`，保留系数，
 *                       允许为 NULL。
 * @param[in] value 用于预置四个历史槽的常量样本。
 * @return 无返回值；NULL 滤波器或非有限 `value` 时保持原状态且静默返回。
 * 调用方式：由 `seed_primary_filters()` 在主姿态首次建立时对六个单轴滤波器调用。
 * 线程约束：修改滤波器状态、不阻塞、不获取 mutex；仅 IMU 单 writer 任务串行调用，
 *           禁止 ISR/并发调用，不保存或接管对象所有权。
 */
static void seed_biquad_constant(dual_ahrs_biquad_t *filter, float value)
{
    if (filter == NULL || !isfinite(value)) {
        return;
    }
    filter->x1 = value;
    filter->x2 = value;
    filter->y1 = value;
    filter->y2 = value;
}

/**
 * @brief 用首个有效 BMI323 加速度/陀螺样本预置三轴低通滤波历史。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] accel 三轴加速度常量初值，单位 m/s^2。
 * @param[in] gyro 三轴角速度常量初值，单位 rad/s。
 * @return 无返回值；任一非有限分量仅使对应单轴 helper 保持原历史，无整体失败标志。
 * 调用方式：仅由 `initialize_static_attitudes()` 在主零参考尚未捕获时调用。
 * 线程约束：写入 `s_dual` 六个滤波器历史且无内部 mutex，非线程安全、不阻塞；仅 IMU
 *           单 writer 任务调用，禁止 ISR/并发调用，参数按值不涉及所有权。
 */
static void seed_primary_filters(dual_ahrs_vector3_t accel,
                                 dual_ahrs_vector3_t gyro)
{
    const float accel_value[3] = {accel.x, accel.y, accel.z};
    const float gyro_value[3] = {gyro.x, gyro.y, gyro.z};
    uint8_t index;

    for (index = 0U; index < 3U; ++index) {
        seed_biquad_constant(&s_dual.accel_lpf[index], accel_value[index]);
        seed_biquad_constant(&s_dual.gyro_lpf[index], gyro_value[index]);
    }
}

/**
 * @brief 从第一组可用静态输入建立主/冗余姿态并预置各自滤波历史与时间基线。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] input 调用期间只读的已去偏置、极性修正和水平变换输入；允许为 NULL，
 *                  指针不被保存。
 * @return 无返回值；NULL 或分支有效位/时间戳/配对/向量校验失败时跳过相应分支，主分支
 *         成功不保证冗余分支也成功；失败通过后续 `valid`/state 体现。
 * 调用方式：`dual_ahrs_update()` 在每轮正式滤波前调用，直到各自 yaw offset 被捕获。
 * 线程约束：写入 `s_dual` 姿态、滤波器、五点历史和时间戳，无内部 mutex、不阻塞但非线程安全；
 *           仅 IMU 单 writer 任务调用，禁止 ISR/并发调用，输入所有权归调用方。
 */
static void initialize_static_attitudes(const dual_ahrs_input_t *input)
{
    dual_ahrs_attitude_t attitude;
    const uint64_t accel_timestamp_us = input_lsm_accel_timestamp(input);
    const uint64_t mag_timestamp_us = input_lsm_mag_timestamp(input);
    uint8_t index;

    if (input == NULL) {
        return;
    }

    if (s_dual.primary_yaw_offset_valid == 0U &&
        input->bmi_accel_valid != 0U && input->bmi_gyro_valid != 0U &&
        input->bmi_timestamp_us != 0U &&
        attitude_from_accel(input->bmi_accel, &attitude) != 0U) {
        s_dual.primary_q = attitude.quaternion;
        s_dual.primary = attitude;
        s_dual.primary_integral = (dual_ahrs_vector3_t){0.0f, 0.0f, 0.0f};
        seed_primary_filters(input->bmi_accel, input->gyro);
    }

    if (input->lsm_mag_valid == 0U || input->lsm_accel_valid == 0U ||
        accel_timestamp_us == 0U || mag_timestamp_us == 0U ||
        lsm_timestamps_are_coherent(accel_timestamp_us, mag_timestamp_us) == 0U) {
        return;
    }
    if (s_dual.redundant_yaw_offset_valid == 0U &&
        input->lsm_accel_valid != 0U &&
        attitude_from_accel_mag(input->lsm_accel, input->mag, &attitude) != 0U) {
        s_dual.redundant = attitude;
        s_dual.redundant_accel = input->lsm_accel;
        s_dual.redundant_mag = input->mag;
        for (index = 0U; index < 5U; ++index) {
            s_dual.accel_history[index] = input->lsm_accel;
            s_dual.mag_history[index] = input->mag;
        }
        s_dual.history_count = 5U;
        s_dual.history_index = 0U;
        s_dual.last_lsm_accel_input_timestamp_us = accel_timestamp_us;
        s_dual.last_lsm_mag_input_timestamp_us = mag_timestamp_us;
        s_dual.last_lsm_timestamp_us = accel_timestamp_us > mag_timestamp_us
                                            ? accel_timestamp_us
                                            : mag_timestamp_us;
    }
}

/**
 * @brief 对恰好五个浮点样本执行局部插入排序并返回中位数。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] values 调用期间只读且至少含五项的数组；不可为 NULL，函数复制后排序。
 * @return 返回排序后索引 2 的值；输入含 NaN 时比较规则可能使返回值不可用，函数不校验
 *         有限性且无错误码。
 * 调用方式：仅由 `median_vector()` 分别计算五点历史的 x/y/z 中位数。
 * 线程约束：仅使用 20 字节局部浮点数组，纯计算、不阻塞、不获取 mutex 且可重入；
 *           禁止从 ISR 调用，不保存输入指针，数组所有权归调用方。
 */
static float median5(const float *values)
{
    float sorted[5];
    uint8_t i;
    uint8_t j;

    for (i = 0U; i < 5U; ++i) {
        sorted[i] = values[i];
    }
    for (i = 1U; i < 5U; ++i) {
        const float value = sorted[i];
        j = i;
        while (j > 0U && sorted[j - 1U] > value) {
            sorted[j] = sorted[j - 1U];
            --j;
        }
        sorted[j] = value;
    }
    return sorted[2];
}

/**
 * @brief 对五个三维历史样本逐轴取中位数，形成抗离群值向量。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] history 调用期间只读且至少含五个 `dual_ahrs_vector3_t` 的数组；不可为 NULL。
 * @return 返回逐轴五点中位数；任何轴含非有限值时结果可能不可用且无独立错误码。
 * 调用方式：仅由 `update_redundant()` 对 LSM303 加速度和磁场五点环形历史各调用一次。
 * 线程约束：纯计算、使用栈上三个五元素数组，不阻塞且不获取 mutex；任务上下文可重入，
 *           禁止 ISR 调用，历史数组不得在调用期间被并发修改且所有权归调用方。
 */
static dual_ahrs_vector3_t median_vector(const dual_ahrs_vector3_t *history)
{
    float x[5];
    float y[5];
    float z[5];
    uint8_t i;
    for (i = 0U; i < 5U; ++i) {
        x[i] = history[i].x;
        y[i] = history[i].y;
        z[i] = history[i].z;
    }
    return (dual_ahrs_vector3_t){median5(x), median5(y), median5(z)};
}

/**
 * @brief 消费一对新的 LSM303 样本，执行五点中值与一阶平滑并更新冗余姿态。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] input 调用期间只读的已校准/水平变换输入；允许为 NULL，不保存该指针。
 * @return 无返回值；NULL、无效/零/不相干或非递增时间戳时不消费；有效新样本会先推进
 *         时间戳和历史，历史不足或归一化失败时可能已修改内部状态但不会发布新姿态。
 * 调用方式：仅由 `dual_ahrs_update()` 在静态初始化之后每轮调用一次。
 * 线程约束：写入 `s_dual` LSM 时间戳、环形历史、平滑状态和冗余姿态，无内部 mutex、
 *           非线程安全且不阻塞；仅 IMU 单 writer 任务调用，禁止 ISR/并发调用。
 */
static void update_redundant(const dual_ahrs_input_t *input)
{
    dual_ahrs_vector3_t accel;
    dual_ahrs_vector3_t mag;
    dual_ahrs_vector3_t accel_unit;
    const uint64_t accel_timestamp_us = input_lsm_accel_timestamp(input);
    const uint64_t mag_timestamp_us = input_lsm_mag_timestamp(input);
    uint8_t accel_valid;
    uint8_t mag_valid;
    float horizontal_x;
    float horizontal_y;

    if (input == NULL || input->lsm_accel_valid == 0U ||
        input->lsm_mag_valid == 0U || accel_timestamp_us == 0U ||
        mag_timestamp_us == 0U ||
        lsm_timestamps_are_coherent(accel_timestamp_us, mag_timestamp_us) == 0U) {
        return;
    }
    if (accel_timestamp_us <= s_dual.last_lsm_accel_input_timestamp_us ||
        mag_timestamp_us <= s_dual.last_lsm_mag_input_timestamp_us) {
        return;
    }
    s_dual.last_lsm_accel_input_timestamp_us = accel_timestamp_us;
    s_dual.last_lsm_mag_input_timestamp_us = mag_timestamp_us;
    s_dual.accel_history[s_dual.history_index] = input->lsm_accel;
    s_dual.mag_history[s_dual.history_index] = input->mag;
    s_dual.history_index = (uint8_t)((s_dual.history_index + 1U) % 5U);
    if (s_dual.history_count < 5U) {
        ++s_dual.history_count;
    }
    if (s_dual.history_count < 5U) {
        return;
    }
    accel = median_vector(s_dual.accel_history);
    mag = median_vector(s_dual.mag_history);
    s_dual.redundant_accel.x = (1.0f - DUAL_AHRS_MAG_ALPHA) *
                                   s_dual.redundant_accel.x +
                               DUAL_AHRS_MAG_ALPHA * accel.x;
    s_dual.redundant_accel.y = (1.0f - DUAL_AHRS_MAG_ALPHA) *
                                   s_dual.redundant_accel.y +
                               DUAL_AHRS_MAG_ALPHA * accel.y;
    s_dual.redundant_accel.z = (1.0f - DUAL_AHRS_MAG_ALPHA) *
                                   s_dual.redundant_accel.z +
                               DUAL_AHRS_MAG_ALPHA * accel.z;
    s_dual.redundant_mag.x = (1.0f - DUAL_AHRS_MAG_ALPHA) *
                                 s_dual.redundant_mag.x +
                             DUAL_AHRS_MAG_ALPHA * mag.x;
    s_dual.redundant_mag.y = (1.0f - DUAL_AHRS_MAG_ALPHA) *
                                 s_dual.redundant_mag.y +
                             DUAL_AHRS_MAG_ALPHA * mag.y;
    s_dual.redundant_mag.z = (1.0f - DUAL_AHRS_MAG_ALPHA) *
                                 s_dual.redundant_mag.z +
                             DUAL_AHRS_MAG_ALPHA * mag.z;
    accel_unit = vector_normalize(s_dual.redundant_accel, &accel_valid);
    (void)vector_normalize(s_dual.redundant_mag, &mag_valid);
    if (accel_valid == 0U || mag_valid == 0U) {
        return;
    }
    s_dual.redundant.roll = atan2f(accel_unit.y, accel_unit.z);
    s_dual.redundant.pitch = atan2f(-accel_unit.x,
                                    sqrtf(accel_unit.y * accel_unit.y +
                                          accel_unit.z * accel_unit.z));
    horizontal_x = s_dual.redundant_mag.x * cosf(s_dual.redundant.pitch) +
                   s_dual.redundant_mag.z * sinf(s_dual.redundant.pitch);
    horizontal_y = s_dual.redundant_mag.x * sinf(s_dual.redundant.roll) *
                       sinf(s_dual.redundant.pitch) +
                   s_dual.redundant_mag.y * cosf(s_dual.redundant.roll) -
                   s_dual.redundant_mag.z * sinf(s_dual.redundant.roll) *
                       cosf(s_dual.redundant.pitch);
    /* The LSM303 magnetic vector has already been mapped to Body Frame. */
    s_dual.redundant.yaw = dual_ahrs_wrap_pi(atan2f(horizontal_y,
                                                    horizontal_x));
    s_dual.redundant.valid = 1U;
    sync_attitude_quaternion(&s_dual.redundant);
    s_dual.last_lsm_timestamp_us = accel_timestamp_us > mag_timestamp_us
                                       ? accel_timestamp_us
                                       : mag_timestamp_us;
}

/**
 * @brief 用时间戳驱动的梯形角速度与加速度 Mahony 反馈推进 BMI323 主 6 轴姿态。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] input 调用期间只读的输入元数据，使用 BMI 有效位与微秒时间戳；允许为 NULL。
 * @param[in] accel 已低通的 BMI323 加速度，单位 m/s^2。
 * @param[in] gyro 已低通且已完成机体系极性/水平变换的角速度，单位 rad/s。
 * @return 无返回值；首次、重复/倒退、过短/过长间隔仅重锚时间/梯形历史而不积分；加速度
 *         不可归一化也不发布新姿态，失败状态由外层有效位和 freshness 表达。
 * 调用方式：仅由 `dual_ahrs_update()` 在 BMI 输入有效且两路低通完成后调用。
 * 线程约束：写入 `s_dual` 主时间基线、积分项和姿态，无内部 mutex、非线程安全且不阻塞；
 *           仅 200 Hz BMI 单 writer 任务调用，禁止 ISR/并发调用，不保存输入指针。
 */
static void update_primary(const dual_ahrs_input_t *input,
                           dual_ahrs_vector3_t accel,
                           dual_ahrs_vector3_t gyro)
{
    dual_ahrs_vector3_t accel_unit;
    dual_ahrs_vector3_t gravity;
    dual_ahrs_vector3_t error;
    uint8_t accel_valid;
    float dt;
    float half_dt;
    dual_ahrs_quaternion_t q;
    dual_ahrs_quaternion_t previous_q;
    dual_ahrs_vector3_t gyro_integral;

    if (input == NULL || input->bmi_accel_valid == 0U ||
        input->bmi_gyro_valid == 0U || input->bmi_timestamp_us == 0U) {
        return;
    }
    if (s_dual.last_bmi_timestamp_us == 0U ||
        s_dual.primary_gyro_prev_valid == 0U) {
        s_dual.last_bmi_timestamp_us = input->bmi_timestamp_us;
        s_dual.primary_gyro_prev = gyro;
        s_dual.primary_gyro_prev_valid = 1U;
        return;
    }
    if (input->bmi_timestamp_us <= s_dual.last_bmi_timestamp_us) {
        /* A repeated or regressed timestamp cannot define an integration
         * interval. Re-anchor the trapezoid at this sample. */
        s_dual.last_bmi_timestamp_us = input->bmi_timestamp_us;
        s_dual.primary_gyro_prev = gyro;
        s_dual.primary_gyro_prev_valid = 1U;
        return;
    }
    dt = (float)(input->bmi_timestamp_us - s_dual.last_bmi_timestamp_us) /
         1000000.0f;
    s_dual.last_bmi_timestamp_us = input->bmi_timestamp_us;
    gyro_integral.x = 0.5f * (s_dual.primary_gyro_prev.x + gyro.x);
    gyro_integral.y = 0.5f * (s_dual.primary_gyro_prev.y + gyro.y);
    gyro_integral.z = 0.5f * (s_dual.primary_gyro_prev.z + gyro.z);
    s_dual.primary_gyro_prev = gyro;
    if (dt < DUAL_AHRS_MIN_DT || dt > DUAL_AHRS_MAX_DT) {
        /* Never substitute a nominal step or bridge a missing sample. The
         * current sample becomes the next trapezoid's baseline. */
        return;
    }
    accel_unit = vector_normalize(accel, &accel_valid);
    if (accel_valid == 0U) {
        return;
    }

    q = s_dual.primary_q;
    previous_q = q;
    gravity = (dual_ahrs_vector3_t){
        2.0f * (q.x * q.z - q.w * q.y),
        2.0f * (q.w * q.x + q.y * q.z),
        q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z};
    error = vector_scale(vector_cross(accel_unit, gravity),
                         gravity_confidence(&accel));
    s_dual.primary_integral.x += DUAL_AHRS_PRIMARY_KI * error.x * dt;
    s_dual.primary_integral.y += DUAL_AHRS_PRIMARY_KI * error.y * dt;
    s_dual.primary_integral.z += DUAL_AHRS_PRIMARY_KI * error.z * dt;
    s_dual.primary_integral.x = fmaxf(-0.25f, fminf(0.25f,
                                                    s_dual.primary_integral.x));
    s_dual.primary_integral.y = fmaxf(-0.25f, fminf(0.25f,
                                                    s_dual.primary_integral.y));
    s_dual.primary_integral.z = fmaxf(-0.25f, fminf(0.25f,
                                                    s_dual.primary_integral.z));
    {
        const float gx = gyro_integral.x + DUAL_AHRS_PRIMARY_KP * error.x +
                         s_dual.primary_integral.x;
        const float gy = gyro_integral.y + DUAL_AHRS_PRIMARY_KP * error.y +
                         s_dual.primary_integral.y;
        const float gz = gyro_integral.z + s_dual.primary_integral.z;
        half_dt = 0.5f * dt;
        q.w += (-q.x * gx - q.y * gy - q.z * gz) * half_dt;
        q.x += (previous_q.w * gx + previous_q.y * gz - previous_q.z * gy) *
               half_dt;
        q.y += (previous_q.w * gy - previous_q.x * gz + previous_q.z * gx) *
               half_dt;
        q.z += (previous_q.w * gz + previous_q.x * gy - previous_q.y * gx) *
               half_dt;
    }
    s_dual.primary_q = quaternion_normalize(q);
    s_dual.primary = quaternion_to_attitude(s_dual.primary_q);
}

/**
 * @brief 将原始姿态变换到已捕获的航向零参考，并同步最终欧拉角与四元数。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] raw 调用期间只读的原始姿态，不可为 NULL且不被保存。
 * @param[in] yaw_offset 已捕获航向偏移地址，不可为 NULL；本函数只读其值。
 * @param[in] yaw_offset_valid 偏移有效标志地址，不可为 NULL；本函数只读其值。
 * @param[in,out] zero_pending 首帧归零待处理标志，不可为 NULL；非零时会被消费并清零。
 * @return 参数无效返回全零且 `valid=0` 的姿态；原始姿态无效时返回其副本；偏移未就绪时
 *         返回有效但全零/单位四元数占位；正常时返回仅航向减偏移的最终姿态。
 * 调用方式：`dual_ahrs_update()` 每轮分别对主、冗余姿态调用并写入输出快照。
 * 线程约束：除 `zero_pending` 外只读输入，不阻塞、不获取 mutex；指针实际指向 `s_dual`
 *           时仅允许单 writer 任务串行调用，禁止 ISR/并发调用且不转移所有权。
 */
static dual_ahrs_attitude_t zero_reference_attitude(
    const dual_ahrs_attitude_t *raw, float *yaw_offset,
    uint8_t *yaw_offset_valid, uint8_t *zero_pending)
{
    dual_ahrs_attitude_t output = {0};

    if (raw == NULL || yaw_offset == NULL || yaw_offset_valid == NULL ||
        zero_pending == NULL) {
        return output;
    }

    output = *raw;
    if (raw->valid == 0U) {
        return output;
    }
    if (*yaw_offset_valid == 0U) {
        output.roll = 0.0f;
        output.pitch = 0.0f;
        output.yaw = 0.0f;
        output.quaternion =
            (dual_ahrs_quaternion_t){1.0f, 0.0f, 0.0f, 0.0f};
        return output;
    }

    /* Roll and pitch stay in the leveled quaternion frame. Only yaw has a
     * scalar reference, so no Euler roll/pitch subtraction is introduced. */
    output.yaw = dual_ahrs_wrap_pi(raw->yaw - *yaw_offset);
    if (*zero_pending != 0U) {
        output.roll = 0.0f;
        output.pitch = 0.0f;
        output.yaw = 0.0f;
        *zero_pending = 0U;
    }
    /* The output Euler values are the final zero-referenced values. Keep the
     * serialized quaternion in the same frame after every sign/reference
     * adjustment. */
    sync_attitude_quaternion(&output);
    return output;
}

/**
 * @brief 在主或冗余原始姿态首次有效时捕获各自航向零偏并安排首个输出归零。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * 传入参数：无。
 * @return 无返回值；未有效或已捕获的分支保持不变，成功分支保存当前 yaw、置有效标志并
 *         置 `zero_pending=1`，不报告单独失败码。
 * 调用方式：仅由 `dual_ahrs_update()` 在本轮主/冗余估计器更新后、生成零参考输出前调用。
 * 线程约束：读写 `s_dual` 姿态和零参考标志，无内部 mutex、非线程安全且不阻塞；仅 IMU
 *           单 writer 任务调用，禁止 ISR/并发调用，不涉及外部对象所有权。
 */
static void capture_ready_yaw_offsets(void)
{
    if (s_dual.primary.valid != 0U &&
        s_dual.primary_yaw_offset_valid == 0U) {
        s_dual.primary_yaw_offset = s_dual.primary.yaw;
        s_dual.primary_yaw_offset_valid = 1U;
        s_dual.primary_zero_pending = 1U;
    }
    if (s_dual.redundant.valid != 0U &&
        s_dual.redundant_yaw_offset_valid == 0U) {
        s_dual.redundant_yaw_offset = s_dual.redundant.yaw;
        s_dual.redundant_yaw_offset_valid = 1U;
        s_dual.redundant_zero_pending = 1U;
    }
}

/** 初始化 DualAHRS 状态和滤波器。 */
void dual_ahrs_init(void)
{
    uint8_t index;
    (void)memset(&s_dual, 0, sizeof(s_dual));
    for (index = 0U; index < 3U; ++index) {
        s_dual.accel_lpf[index] = make_lpf(DUAL_AHRS_ACCEL_LPF_HZ);
        s_dual.gyro_lpf[index] = make_lpf(DUAL_AHRS_GYRO_LPF_HZ);
    }
    imu_leveling_init(&s_dual.leveling_bmi);
    imu_leveling_init(&s_dual.leveling_lsm);
    s_dual.local_gravity_mps2 = IMU_LEVELING_G_DEFAULT_MPS2;
    reset_runtime_state();
    s_dual.state = DUAL_AHRS_STATE_WAIT_CAL;
    s_dual.output.state = s_dual.state;
}

/** 提交已冻结的两路水平校准状态。 */
void dual_ahrs_set_leveling(const imu_leveling_state_t *bmi,
                            const imu_leveling_state_t *lsm)
{
    if (bmi != NULL) {
        s_dual.leveling_bmi = *bmi;
    } else {
        imu_leveling_init(&s_dual.leveling_bmi);
    }
    if (lsm != NULL) {
        s_dual.leveling_lsm = *lsm;
    } else {
        imu_leveling_init(&s_dual.leveling_lsm);
    }
}

/** 更新本地重力参考值。 */
void dual_ahrs_set_local_gravity(float gravity_mps2)
{
    s_dual.local_gravity_mps2 =
        isfinite(gravity_mps2) && gravity_mps2 >= IMU_LEVELING_G_MIN &&
                gravity_mps2 <= IMU_LEVELING_G_MAX
            ? gravity_mps2
            : IMU_LEVELING_G_DEFAULT_MPS2;
}

/** 设置传感器偏置；NULL 会清除运行历史并等待标定。 */
void dual_ahrs_set_bias(const dual_ahrs_bias_t *bias)
{
    if (bias == NULL || vector_is_finite(bias->bmi_accel) == 0U ||
        vector_is_finite(bias->bmi_gyro) == 0U ||
        vector_is_finite(bias->lsm_accel) == 0U) {
        s_dual.bias_valid = 0U;
        reset_runtime_state();
        s_dual.state = DUAL_AHRS_STATE_WAIT_CAL;
        s_dual.output.state = s_dual.state;
        return;
    }

    s_dual.bias = *bias;
    s_dual.bias_valid = 1U;
    reset_runtime_state();
    s_dual.state = DUAL_AHRS_STATE_READY;
    s_dual.output.state = s_dual.state;
}

/** 消费一组带时间戳/有效位的输入并推进一次融合。 */
void dual_ahrs_update(const dual_ahrs_input_t *input)
{
    float primary_age = 0.0f;
    float lsm_age = 0.0f;
    uint64_t reference_timestamp_us = 0U;
    uint8_t primary_valid;
    uint8_t redundant_valid;
    dual_ahrs_vector3_t delta;
    dual_ahrs_vector3_t filtered_accel;
    dual_ahrs_vector3_t filtered_gyro;
    uint64_t lsm_accel_timestamp_us;
    uint64_t lsm_mag_timestamp_us;

    if (input == NULL) {
        return;
    }
    if (s_dual.state == DUAL_AHRS_STATE_WAIT_CAL ||
        s_dual.bias_valid == 0U) {
        /* The calibration gate is deliberately before every filter, history,
         * timestamp, and gyro integration operation. */
        return;
    }

    dual_ahrs_input_t calibrated_input = *input;
    lsm_accel_timestamp_us = input_lsm_accel_timestamp(input);
    lsm_mag_timestamp_us = input_lsm_mag_timestamp(input);
    calibrated_input.bmi_accel.x -= s_dual.bias.bmi_accel.x;
    calibrated_input.bmi_accel.y -= s_dual.bias.bmi_accel.y;
    calibrated_input.bmi_accel.z -= s_dual.bias.bmi_accel.z;
    calibrated_input.gyro.x -= s_dual.bias.bmi_gyro.x;
    calibrated_input.gyro.y -= s_dual.bias.bmi_gyro.y;
    calibrated_input.gyro.z -= s_dual.bias.bmi_gyro.z;
    /* BMI323 Z rotation is physically opposite to the vehicle Body Frame.
     * Apply the polarity correction after bias removal and before leveling so
     * the calibrated sensor-frame rate is transformed as one vector. */
    calibrated_input.gyro.z = -calibrated_input.gyro.z;
    calibrated_input.lsm_accel.x -= s_dual.bias.lsm_accel.x;
    calibrated_input.lsm_accel.y -= s_dual.bias.lsm_accel.y;
    calibrated_input.lsm_accel.z -= s_dual.bias.lsm_accel.z;
    /* The frozen boot-time matrices are applied after sensor-frame bias
     * removal and before every DualAHRS filter/estimator operation. */
    calibrated_input.bmi_accel =
        level_vector(&s_dual.leveling_bmi, calibrated_input.bmi_accel);
    calibrated_input.gyro =
        level_vector(&s_dual.leveling_bmi, calibrated_input.gyro);
    calibrated_input.lsm_accel =
        level_vector(&s_dual.leveling_lsm, calibrated_input.lsm_accel);
    calibrated_input.mag =
        level_vector(&s_dual.leveling_lsm, calibrated_input.mag);
    input = &calibrated_input;
    if (input->bmi_accel_valid == 0U || input->bmi_gyro_valid == 0U ||
        input->bmi_timestamp_us == 0U) {
        s_dual.primary.valid = 0U;
    }
    if (input->lsm_accel_valid == 0U || input->lsm_mag_valid == 0U ||
        lsm_accel_timestamp_us == 0U || lsm_mag_timestamp_us == 0U) {
        s_dual.redundant.valid = 0U;
    }
    /* R_level has already aligned gravity with +Z. Establish both filters
     * directly from that first static frame so their zero references cannot
     * be captured while the Mahony/IIR state is still converging from zero. */
    initialize_static_attitudes(input);
    update_redundant(input);
    if (input->lsm_mag_valid != 0U && lsm_mag_timestamp_us != 0U &&
        lsm_mag_timestamp_us > s_dual.last_mag_reference_timestamp_us) {
        const float norm = vector_norm(input->mag);
        if (s_dual.mag_reference_norm <= 1.0e-6f && isfinite(norm)) {
            s_dual.mag_reference_norm = norm;
        } else if (isfinite(norm)) {
            s_dual.mag_reference_norm =
                0.995f * s_dual.mag_reference_norm + 0.005f * norm;
        }
        s_dual.last_mag_reference_timestamp_us = lsm_mag_timestamp_us;
    }
    if (input->bmi_accel_valid != 0U && input->bmi_gyro_valid != 0U &&
        input->bmi_timestamp_us != 0U) {
        /* The BMI323 stream is 200 Hz: filter and integrate every sample. */
        filtered_accel = filter_vector(input->bmi_accel, s_dual.accel_lpf);
        filtered_gyro = filter_vector(input->gyro, s_dual.gyro_lpf);
        s_dual.primary_gyro_z_rad_s = filtered_gyro.z;
        update_primary(input, filtered_accel, filtered_gyro);
    }
    capture_ready_yaw_offsets();
    s_dual.output.primary = zero_reference_attitude(
        &s_dual.primary, &s_dual.primary_yaw_offset,
        &s_dual.primary_yaw_offset_valid, &s_dual.primary_zero_pending);
    s_dual.output.redundant = zero_reference_attitude(
        &s_dual.redundant, &s_dual.redundant_yaw_offset,
        &s_dual.redundant_yaw_offset_valid, &s_dual.redundant_zero_pending);
    primary_valid = s_dual.output.primary.valid;
    redundant_valid = s_dual.output.redundant.valid;
    s_dual.output.gravity_confidence = gravity_confidence(&input->bmi_accel);
    s_dual.output.magnetic_confidence = mag_confidence(&input->mag);
    if (primary_valid != 0U && redundant_valid != 0U) {
        /* Compare the final, zero-referenced Euler values so sign corrections
         * and quaternion synchronization cannot be bypassed by the delta. */
        delta.x = delta_roll(s_dual.output.primary.roll,
                             s_dual.output.redundant.roll);
        delta.y = delta_pitch(s_dual.output.primary.pitch,
                              s_dual.output.redundant.pitch);
        delta.z = delta_yaw(s_dual.output.primary.yaw,
                            s_dual.output.redundant.yaw);
        s_dual.output.delta_rad = delta;
    } else {
        s_dual.output.delta_rad = (dual_ahrs_vector3_t){0.0f, 0.0f, 0.0f};
    }
    if (input->bmi_timestamp_us > reference_timestamp_us) {
        reference_timestamp_us = input->bmi_timestamp_us;
    }
    if (lsm_accel_timestamp_us > reference_timestamp_us) {
        reference_timestamp_us = lsm_accel_timestamp_us;
    }
    if (lsm_mag_timestamp_us > reference_timestamp_us) {
        reference_timestamp_us = lsm_mag_timestamp_us;
    }
    if (reference_timestamp_us >= s_dual.last_bmi_timestamp_us &&
        s_dual.last_bmi_timestamp_us != 0U) {
        primary_age = (float)(reference_timestamp_us -
                              s_dual.last_bmi_timestamp_us);
    } else if (s_dual.last_bmi_timestamp_us != 0U) {
        primary_age = (float)DUAL_AHRS_PRIMARY_STALE_US;
    }
    if (reference_timestamp_us >= s_dual.last_lsm_timestamp_us &&
        s_dual.last_lsm_timestamp_us != 0U) {
        lsm_age = (float)(reference_timestamp_us -
                          s_dual.last_lsm_timestamp_us);
    } else if (s_dual.last_lsm_timestamp_us != 0U) {
        lsm_age = (float)DUAL_AHRS_LSM_STALE_US;
    }
    if (primary_valid != 0U && redundant_valid != 0U &&
        primary_age < (float)DUAL_AHRS_PRIMARY_STALE_US &&
        lsm_age < (float)DUAL_AHRS_LSM_STALE_US) {
        s_dual.state = DUAL_AHRS_STATE_TRACKING;
    } else if (primary_valid != 0U || redundant_valid != 0U) {
        s_dual.state = DUAL_AHRS_STATE_DEGRADED;
    } else if (s_dual.state == DUAL_AHRS_STATE_READY) {
        /* Bias hand-off has completed, but no valid sample has arrived yet. */
        s_dual.state = DUAL_AHRS_STATE_READY;
    } else if (input->bmi_accel_valid != 0U || input->lsm_accel_valid != 0U) {
        s_dual.state = DUAL_AHRS_STATE_WARMUP;
    } else {
        s_dual.state = DUAL_AHRS_STATE_FAULT;
    }
    s_dual.output.state = s_dual.state;
    s_dual.output.schema = DUAL_AHRS_SCHEMA;
    s_dual.output.timestamp_ms =
        (uint32_t)(reference_timestamp_us / UINT64_C(1000));
    s_dual.output.sample_sequence = ++s_dual.sample_sequence;
    s_dual.output.flags = (uint8_t)((primary_valid != 0U ? 0x01U : 0U) |
                                    (redundant_valid != 0U ? 0x02U : 0U) |
                                    (input->lsm_mag_valid != 0U &&
                                             lsm_mag_timestamp_us != 0U
                                         ? 0x04U
                                         : 0U) |
                                    (s_dual.output.gravity_confidence < 0.5f
                                         ? 0x10U
                                         : 0U) |
                                    (s_dual.output.magnetic_confidence < 0.25f
                                         ? 0x20U
                                         : 0U) |
                                    ((primary_age >= (float)DUAL_AHRS_PRIMARY_STALE_US ||
                                      lsm_age >= (float)DUAL_AHRS_LSM_STALE_US)
                                         ? 0x40U
                                         : 0U) |
                                    (s_dual.state == DUAL_AHRS_STATE_FAULT
                                         ? 0x80U
                                         : 0U));
}

/** 复制最新融合输出快照。 */
void dual_ahrs_get_output(dual_ahrs_output_t *output)
{
    if (output != NULL) {
        const uint64_t now_us = imu_time_now_us();

        *output = s_dual.output;
        if (primary_is_fresh(now_us) == 0U) {
            output->primary.valid = 0U;
            output->flags = (uint8_t)((output->flags | 0x40U) & ~0x01U);
            if (output->state == DUAL_AHRS_STATE_READY ||
                output->state == DUAL_AHRS_STATE_TRACKING) {
                output->state = DUAL_AHRS_STATE_DEGRADED;
            }
        }
        if (redundant_is_fresh(now_us) == 0U) {
            output->redundant.valid = 0U;
            output->flags = (uint8_t)((output->flags | 0x40U) & ~0x06U);
            if (output->state == DUAL_AHRS_STATE_READY ||
                output->state == DUAL_AHRS_STATE_TRACKING) {
                output->state = DUAL_AHRS_STATE_DEGRADED;
            }
        }
        if (output->primary.valid == 0U || output->redundant.valid == 0U) {
            output->delta_rad = (dual_ahrs_vector3_t){0.0f, 0.0f, 0.0f};
        }
    }
}

/** 获取最新主航向和 Z 轴角速度；过期时拒绝输出。 */
uint8_t dual_ahrs_get_heading_state(float *yaw_rad, float *gyro_z_rad_s)
{
    if (yaw_rad == NULL || gyro_z_rad_s == NULL ||
        primary_is_fresh(imu_time_now_us()) == 0U ||
        !isfinite(s_dual.output.primary.yaw) ||
        !isfinite(s_dual.primary_gyro_z_rad_s)) {
        return 0U;
    }
    *yaw_rad = s_dual.output.primary.yaw;
    *gyro_z_rad_s = s_dual.primary_gyro_z_rad_s;
    return 1U;
}

/** 检查主 AHRS 样本是否在运动 freshness 窗口内。 */
uint8_t dual_ahrs_is_primary_fresh(void)
{
    return primary_is_fresh(imu_time_now_us());
}

/**
 * @brief 将 16 位无符号整数按低字节在前写入调用方缓冲区。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[out] destination 调用方拥有且至少有 2 个可写字节的起始地址；不可为 NULL。
 * @param[in] value 待序列化的 16 位值。
 * @return 无返回值；有效缓冲区总写满 2 字节，非法/过小缓冲区没有防御性失败输出。
 * 调用方式：仅由 `dual_ahrs_pack_payload()` 写 schema=2 保留字段。
 * 线程约束：纯内存写入、不阻塞、不获取 mutex 且可重入；任务上下文调用，禁止 ISR，
 *           不保存或接管缓冲区所有权，同一目标区域不得并发写。
 */
static void put_u16_le(uint8_t *destination, uint16_t value)
{
    destination[0] = (uint8_t)(value & 0xFFU);
    destination[1] = (uint8_t)(value >> 8U);
}

/**
 * @brief 将 32 位无符号整数按低字节在前写入调用方缓冲区。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[out] destination 调用方拥有且至少有 4 个可写字节的起始地址；不可为 NULL。
 * @param[in] value 待序列化的 32 位值。
 * @return 无返回值；有效缓冲区总写满 4 字节，非法/过小缓冲区没有防御性失败输出。
 * 调用方式：由 `put_float_le()` 复用，或由 `dual_ahrs_pack_payload()` 写时间戳和序号。
 * 线程约束：纯内存写入、不阻塞、不获取 mutex 且可重入；任务上下文调用，禁止 ISR，
 *           不保存或接管缓冲区所有权，同一目标区域不得并发写。
 */
static void put_u32_le(uint8_t *destination, uint32_t value)
{
    destination[0] = (uint8_t)(value & 0xFFU);
    destination[1] = (uint8_t)((value >> 8U) & 0xFFU);
    destination[2] = (uint8_t)((value >> 16U) & 0xFFU);
    destination[3] = (uint8_t)(value >> 24U);
}

/**
 * @brief 保留 32 位浮点位模式并以小端字节序写入调用方缓冲区。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[out] destination 调用方拥有且至少有 4 个可写字节的起始地址；不可为 NULL。
 * @param[in] value 待序列化浮点值；NaN/Inf 位模式也会原样写出。
 * @return 无返回值；目标有效时写满 4 字节，非法/过小缓冲区没有防御性失败输出；实现
 *         依赖目标 ABI 的 `float` 为 32 位。
 * 调用方式：由 `put_attitude()` 及 `dual_ahrs_pack_payload()` 写所有浮点字段。
 * 线程约束：仅使用局部位副本并写目标缓冲区，不阻塞、不获取 mutex 且可重入；禁止 ISR
 *           调用，不保存或接管缓冲区所有权，同一目标区域不得并发写。
 */
static void put_float_le(uint8_t *destination, float value)
{
    uint32_t bits;
    (void)memcpy(&bits, &value, sizeof(bits));
    put_u32_le(destination, bits);
}

/**
 * @brief 将单路姿态的三个欧拉角和四元数序列化为固定 28 字节小端布局。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[out] destination 调用方拥有且至少有 28 个可写字节的起始地址；不可为 NULL。
 * @param[in] attitude 调用期间只读的姿态对象；不可为 NULL，`valid` 字段不在此布局写出。
 * @return 无返回值；有效参数时写满 7 个 float 字段，非法参数/容量没有防御性失败输出。
 * 调用方式：仅由 `dual_ahrs_pack_payload()` 分别写 schema=2 的主姿态和冗余姿态段。
 * 线程约束：只读姿态并写调用方缓冲区，不阻塞、不获取 mutex；任务上下文可重入，禁止
 *           ISR 调用，不保存任何指针，同一源/目标对象不得在调用期间并发修改。
 */
static void put_attitude(uint8_t *destination, const dual_ahrs_attitude_t *attitude)
{
    put_float_le(&destination[0], attitude->roll);
    put_float_le(&destination[4], attitude->pitch);
    put_float_le(&destination[8], attitude->yaw);
    put_float_le(&destination[12], attitude->quaternion.w);
    put_float_le(&destination[16], attitude->quaternion.x);
    put_float_le(&destination[20], attitude->quaternion.y);
    put_float_le(&destination[24], attitude->quaternion.z);
}

/** 将姿态输出显式序列化为 SRP schema=2 payload。 */
int dual_ahrs_pack_payload(uint8_t *payload, size_t capacity)
{
    dual_ahrs_output_t output;

    if (payload == NULL || capacity < DUAL_AHRS_PAYLOAD_LENGTH) {
        return -1;
    }
    dual_ahrs_get_output(&output);
    payload[0] = DUAL_AHRS_SCHEMA;
    payload[1] = output.flags;
    put_u16_le(&payload[2], 0U);
    put_u32_le(&payload[4], output.timestamp_ms);
    put_u32_le(&payload[8], output.sample_sequence);
    put_attitude(&payload[12], &output.primary);
    put_attitude(&payload[40], &output.redundant);
    put_float_le(&payload[68], output.delta_rad.x);
    put_float_le(&payload[72], output.delta_rad.y);
    put_float_le(&payload[76], output.delta_rad.z);
    return (int)DUAL_AHRS_PAYLOAD_LENGTH;
}
