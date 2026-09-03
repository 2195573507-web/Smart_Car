#include "imu_leveling.h"

/* IMU 水平校准实现；创建人：待确认（当前维护人：Zhiqin）。 */

#include <math.h>
#include <string.h>

#define IMU_LEVELING_SINGULAR_EPS (1.0e-6f)
#define IMU_LEVELING_RESIDUAL_MAX (1.0e-3f)
#define IMU_LEVELING_RAD_TO_DEG (57.295779513082320876f)

/**
 * @brief 将调用方提供的 3x3 矩阵完整初始化为单位矩阵。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[out] matrix 调用方拥有且至少包含 9 个可写 `float` 的矩阵；不可为 NULL。
 * @return 无返回值；有效缓冲区总被写成单位矩阵，非法指针没有防御性失败输出。
 * 调用方式：由 `imu_leveling_init()`、`set_fallback()` 及水平矩阵构造路径同步调用。
 * 线程约束：纯内存操作、不阻塞、不获取 mutex 且可重入；禁止从 ISR 调用，同一
 *           `matrix` 不得被其他上下文并发访问，函数不保存或接管缓冲区所有权。
 */
static void matrix_identity(float matrix[3][3])
{
    (void)memset(matrix, 0, sizeof(float) * 9U);
    matrix[0][0] = 1.0f;
    matrix[1][1] = 1.0f;
    matrix[2][2] = 1.0f;
}

/**
 * @brief 检查三维向量指针及三个分量是否全部有效且有限。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] vector 调用期间只读的三元素数组；允许为 NULL。
 * @return 非 NULL 且三个分量均通过 `isfinite()` 时返回 true，否则返回 false。
 * 调用方式：由 `imu_leveling_compute_with_accel_std_limit()` 在任何向量解引用前调用。
 * 线程约束：纯只读计算、不阻塞、不获取 mutex 且可重入；可在任务上下文调用，但禁止
 *           从 ISR 调用，指针及其内容所有权始终归调用方。
 */
static bool finite_vector(const float vector[3])
{
    return vector != NULL && isfinite(vector[0]) && isfinite(vector[1]) &&
                   isfinite(vector[2]);
}

/**
 * @brief 计算三维向量的欧氏模长。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] vector 调用期间只读的三元素数组；必须非 NULL 且至少含三个 `float`。
 * @return 返回 `sqrt(x*x+y*y+z*z)`；非有限分量或浮点溢出会传播为非有限结果，
 *         函数本身不提供错误码。
 * 调用方式：由水平校准计算路径用于重力、旋转轴和残差模长，调用前由上层保证指针有效。
 * 线程约束：纯计算、不阻塞、不获取 mutex 且可重入；禁止从 ISR 调用，不保存输入指针，
 *           输入所有权始终归调用方。
 */
static float vector_norm(const float vector[3])
{
    return sqrtf((vector[0] * vector[0]) + (vector[1] * vector[1]) +
                 (vector[2] * vector[2]));
}

/**
 * @brief 按 3x3 展开式计算旋转矩阵行列式。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] matrix 调用期间只读的 3x3 矩阵；必须非 NULL 且包含 9 个有效元素。
 * @return 返回矩阵行列式；输入非有限或运算溢出时非有限值按浮点规则传播，无独立错误码。
 * 调用方式：仅由 `imu_leveling_compute_with_accel_std_limit()` 在 Rodrigues 矩阵生成后校验方向。
 * 线程约束：纯计算、不阻塞、不获取 mutex 且可重入；禁止从 ISR 调用，不保存或接管
 *           `matrix` 所有权。
 */
static float matrix_determinant(const float matrix[3][3])
{
    return matrix[0][0] *
               ((matrix[1][1] * matrix[2][2]) -
                (matrix[1][2] * matrix[2][1])) -
           matrix[0][1] *
               ((matrix[1][0] * matrix[2][2]) -
                (matrix[1][2] * matrix[2][0])) +
           matrix[0][2] *
               ((matrix[1][0] * matrix[2][1]) -
                (matrix[1][1] * matrix[2][0]));
}

/**
 * @brief 计算 3x3 矩阵与三维列向量的乘积。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] matrix 调用期间只读的 3x3 矩阵，不可为 NULL。
 * @param[in] vector 调用期间只读的三元素输入向量，不可为 NULL。
 * @param[out] result 调用方拥有的三元素输出数组，不可为 NULL，且不得与 `vector` 重叠。
 * @return 无返回值；有效输入时写满三个输出分量，非法指针/重叠缓冲区没有防御性失败输出。
 * 调用方式：由水平矩阵残差校验和 `imu_leveling_rotate_vector()` 通过独立临时数组调用。
 * 线程约束：纯计算、不阻塞、不获取 mutex 且可重入；禁止从 ISR 调用，不保存任何指针，
 *           所有缓冲区所有权均归调用方。
 */
static void matrix_multiply_vector(const float matrix[3][3],
                                   const float vector[3], float result[3])
{
    result[0] = matrix[0][0] * vector[0] + matrix[0][1] * vector[1] +
                matrix[0][2] * vector[2];
    result[1] = matrix[1][0] * vector[0] + matrix[1][1] * vector[1] +
                matrix[1][2] * vector[2];
    result[2] = matrix[2][0] * vector[0] + matrix[2][1] * vector[1] +
                matrix[2][2] * vector[2];
}

/**
 * @brief 记录水平校准拒绝原因并把旋转矩阵恢复为安全的单位矩阵。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in,out] state 调用方拥有的水平校准状态，不可为 NULL；函数清除 `valid`、
 *                      写入 `reason` 并覆盖 `r_level`。
 * @param[in] reason 上层质量门或矩阵校验确定的失败枚举值。
 * @return 无返回值；有效 `state` 总产生 `valid=false` 和单位矩阵，NULL 没有防御性处理。
 * 调用方式：仅由 `imu_leveling_compute_with_accel_std_limit()` 的各失败分支同步调用。
 * 线程约束：纯内存写入、不阻塞、不获取 mutex；同一 `state` 仅允许单 writer，禁止从 ISR
 *           或并发上下文调用，函数不保存也不接管状态对象所有权。
 */
static void set_fallback(imu_leveling_state_t *state,
                         imu_leveling_fallback_reason_t reason)
{
    state->valid = false;
    state->fallback_reason = reason;
    matrix_identity(state->r_level);
}

/** 清空水平校准状态并设置默认重力参考。 */
void imu_leveling_init(imu_leveling_state_t *state)
{
    if (state == NULL) {
        return;
    }

    (void)memset(state, 0, sizeof(*state));
    matrix_identity(state->r_level);
    state->g_local_mps2 = IMU_LEVELING_G_DEFAULT_MPS2;
    state->fallback_reason = IMU_LEVELING_FALLBACK_NOT_COMPUTED;
}

/** 依据静态均值、陀螺 RMS 和加速度方差计算水平矩阵。 */
bool imu_leveling_compute_with_accel_std_limit(imu_leveling_state_t *state,
                                               const float accel_mean[3],
                                               float gyro_rms,
                                               float accel_std,
                                               float valid_ratio,
                                               float accel_std_max)
{
    float accel_unit[3];
    float cross[3];
    float skew[3][3];
    float skew_square[3][3];
    float rotated[3];
    float residual[3];
    float g_local;
    float sine;
    float cosine;
    float tilt_deg;
    float determinant;
    uint8_t row;
    uint8_t column;

    if (state == NULL) {
        return false;
    }

    imu_leveling_init(state);
    if (accel_mean != NULL) {
        state->accel_mean[0] = accel_mean[0];
        state->accel_mean[1] = accel_mean[1];
        state->accel_mean[2] = accel_mean[2];
    }
    state->gyro_rms_radps = gyro_rms;
    state->accel_std_mps2 = accel_std;
    state->valid_ratio = valid_ratio;

    if (finite_vector(accel_mean) == false || !isfinite(gyro_rms) ||
        !isfinite(accel_std) || !isfinite(valid_ratio) ||
        !isfinite(accel_std_max) || gyro_rms < 0.0f || accel_std < 0.0f ||
        accel_std_max < 0.0f) {
        set_fallback(state, IMU_LEVELING_FALLBACK_NONFINITE);
        return false;
    }
    if (valid_ratio < IMU_LEVELING_VALID_RATIO_MIN) {
        set_fallback(state, IMU_LEVELING_FALLBACK_SAMPLE_INSUFFICIENT);
        return false;
    }
    if (gyro_rms > IMU_LEVELING_GYRO_RMS_MAX_RADPS) {
        set_fallback(state, IMU_LEVELING_FALLBACK_GYRO_NOISE);
        return false;
    }
    if (accel_std > accel_std_max) {
        set_fallback(state, IMU_LEVELING_FALLBACK_ACCEL_VARIANCE);
        return false;
    }

    g_local = vector_norm(accel_mean);
    if (!isfinite(g_local)) {
        set_fallback(state, IMU_LEVELING_FALLBACK_NONFINITE);
        return false;
    }
    state->g_local_mps2 = g_local;
    if (g_local < IMU_LEVELING_G_MIN || g_local > IMU_LEVELING_G_MAX) {
        set_fallback(state, IMU_LEVELING_FALLBACK_GRAVITY_OUT_OF_RANGE);
        return false;
    }

    accel_unit[0] = accel_mean[0] / g_local;
    accel_unit[1] = accel_mean[1] / g_local;
    accel_unit[2] = accel_mean[2] / g_local;
    cross[0] = accel_unit[1];
    cross[1] = -accel_unit[0];
    cross[2] = 0.0f;
    sine = vector_norm(cross);
    cosine = fmaxf(-1.0f, fminf(1.0f, accel_unit[2]));
    tilt_deg = acosf(cosine) * IMU_LEVELING_RAD_TO_DEG;
    state->tilt_deg = tilt_deg;

    if (cosine <= 0.0f) {
        set_fallback(state, IMU_LEVELING_FALLBACK_INVERTED_START);
        return false;
    }
    if (tilt_deg > IMU_LEVELING_TILT_MAX_DEG) {
        set_fallback(state, IMU_LEVELING_FALLBACK_TILT_EXCEEDED);
        return false;
    }
    if (sine < IMU_LEVELING_SINGULAR_EPS) {
        state->valid = true;
        state->fallback_reason = IMU_LEVELING_FALLBACK_NONE;
        return true;
    }

    cross[0] /= sine;
    cross[1] /= sine;
    cross[2] /= sine;
    skew[0][0] = 0.0f;
    skew[0][1] = -cross[2];
    skew[0][2] = cross[1];
    skew[1][0] = cross[2];
    skew[1][1] = 0.0f;
    skew[1][2] = -cross[0];
    skew[2][0] = -cross[1];
    skew[2][1] = cross[0];
    skew[2][2] = 0.0f;

    for (row = 0U; row < 3U; ++row) {
        for (column = 0U; column < 3U; ++column) {
            skew_square[row][column] =
                skew[row][0] * skew[0][column] +
                skew[row][1] * skew[1][column] +
                skew[row][2] * skew[2][column];
        }
    }
    matrix_identity(state->r_level);
    for (row = 0U; row < 3U; ++row) {
        for (column = 0U; column < 3U; ++column) {
            state->r_level[row][column] += skew[row][column] * sine +
                                            skew_square[row][column] *
                                                (1.0f - cosine);
        }
    }

    matrix_multiply_vector(state->r_level, accel_unit, rotated);
    residual[0] = rotated[0];
    residual[1] = rotated[1];
    residual[2] = rotated[2] - 1.0f;
    determinant = matrix_determinant(state->r_level);
    if (!isfinite(determinant) || determinant <= 0.0f ||
        vector_norm(residual) >= IMU_LEVELING_RESIDUAL_MAX) {
        set_fallback(state, IMU_LEVELING_FALLBACK_MATRIX_INVALID);
        return false;
    }

    state->valid = true;
    state->fallback_reason = IMU_LEVELING_FALLBACK_NONE;
    return true;
}

/** 使用默认方差门限计算水平矩阵。 */
bool imu_leveling_compute(imu_leveling_state_t *state,
                          const float accel_mean[3],
                          float gyro_rms,
                          float accel_std,
                          float valid_ratio)
{
    return imu_leveling_compute_with_accel_std_limit(
        state, accel_mean, gyro_rms, accel_std, valid_ratio,
        IMU_LEVELING_ACCEL_STD_MAX_MPS2);
}

/** 用已验证矩阵旋转三维向量。 */
void imu_leveling_rotate_vector(const imu_leveling_state_t *state,
                                const float v_in[3], float v_out[3])
{
    float result[3];

    if (v_in == NULL || v_out == NULL) {
        return;
    }
    if (state == NULL) {
        v_out[0] = v_in[0];
        v_out[1] = v_in[1];
        v_out[2] = v_in[2];
        return;
    }

    matrix_multiply_vector(state->r_level, v_in, result);
    v_out[0] = result[0];
    v_out[1] = result[1];
    v_out[2] = result[2];
}
