# 雷达连续 PWM 转速自适应 IMU 滤波与姿态解算技术规范

**状态：设计规范（未声明已实现）**<br>
**目标平台：STM32H757 CM7，C99，单精度 FPU**<br>
**建议新增模块：`STM32H757/Middleware/Calibration/imu_adaptive_vibration.h/.c`**<br>
**协议约束：不修改现有 11 字节 `0x0202` 与 16 字节 `0x0208` 负载，也不修改 SCBP-V3 帧格式**

## 0. 范围、现状与证据边界

本文规定连续 `PWM ∈ [0,100]%` 下的振动特征估计、参数平滑、姿态解算动态赋权、数字滤波调度和 ZUPT 判决。它是新增引擎的设计合同，不把本文件中的 API、参数或运行状态写成当前固件已经具备的事实。

当前仓库中已经可以确认的边界如下：

| 事实 | 当前来源 | 对本设计的约束 |
| --- | --- | --- |
| `imu_boot_manager` 统一管理 `STATIC_CALIBRATION -> VIBRATION_CAPTURE -> VERIFY -> READY` | `STM32H757/Middleware/Calibration/README.md` | 标定表只能在 `VERIFY` 成功后发布给运行时引擎 |
| 振动窗口为 20/40/60/80/100%，共享时间窗，质量下限 90% | `STM32H757/Middleware/Calibration/imu_vibration.h/.c` | P1..P5 是实际采样数据；缺失质量位时禁止外推共振参数 |
| 当前 LSM303 滤波/姿态链保持不变，BMI323 不进入现有 `attitude.c` | `STM32H757/Middleware/Calibration/README.md`、`DOCS/imu/*` | 新引擎必须提供兼容旁路，迁移由单独集成任务完成 |
| `s3_service.c` 接收 `RADAR_STATUS` 并调用现有 PWM 滤波选择 | `STM32H757/Middleware/Communication/Services/s3_service.c` | 新 API 接在同一业务边界；解析回调不得执行昂贵重算 |
| `0x0202`、`0x0208` 为已定义传输 ID/负载 | `DOCS/protocol/SCBP_V3_REFERENCE.md` | 自适应状态只能复用已有日志/状态或未来新增 MSG_ID，不能改旧负载 |

以下内容属于设计目标或待验证项：实际 RPM-PWM 曲线、共振频率、噪声参数、CPU/WCET、硬件 PWM 响应、UART/BLE 端到端行为和姿态精度。源码、主机回放、构建、硬件和系统集成证据必须分开记录。

### 0.1 不变量

1. 所有运行时数组、滤波器状态、窗口缓存均静态分配；禁止 `malloc`/`free`。
2. 插值、slew、矩阵对角线和滤波器系数都必须有限、可表示且受上下界保护。
3. PWM 无效或超范围时先钳位，再按状态位记录；不得用 NaN 进入 AHRS/EKF。
4. PWM 只改变测量可信度和滤波参数，不直接修改姿态状态、陀螺积分或协议帧。
5. 高频采样路径不持有 UART、BLE 或互斥锁；通信回调只写入一个原子快照/事件。

## 1. 系统架构与数据流

### 1.1 数据流

```text
 S3 RADAR_STATUS / 本地 PWM
             |
             v
  s3_service.c: 校验 online、0..100、超时
             |
             v
  imu_adaptive_set_pwm()  (只写入输入快照，O(1))
             |
             v
  [PWM 钳位 + 6 节点分段插值]
             |
             v
  [target 参数: RMS、Kp、Racc、fc、f0、ZUPT 阈值]
             |
             v
  [一阶滞后 slew: tau=0.1..0.2 s]
             |
       +-----+---------+-----------------+
       |               |                 |
       v               v                 v
  Adaptive LPF   Tracking notch      Mahony/EKF
       |               |                 |
       +-------+-------+-----------------+
               v
        滤波后的加速度/陀螺仪
               |
               v
       Sliding variance + gyro gate
               |
               v
           Adaptive ZUPT
```

建议的执行频率如下。频率是设计起点，不能替代 CM7 测量：

| 工作 | 建议触发 | 允许做的工作 |
| --- | --- | --- |
| PWM 输入 | `RADAR_STATUS` 到达或本地 PWM 更新 | 校验、存快照、记录时间戳；不调用 `sinf/cosf` |
| 参数更新 | IMU 周期任务，当前仓库典型为 10 ms | 插值、slew、限幅；`float` 乘加 |
| Biquad 系数重算 | PWM/RPM 变化超过阈值或 20 Hz 定时器 | 计算一次 `sinf/cosf`，然后系数 slew |
| 传感器滤波/AHRS | 保持现有 IMU cadence | 只读 runtime snapshot，不能阻塞 |
| ZUPT 窗口 | 每个有效加速度样本 | 固定长度环形数组和方差更新 |

### 1.2 引擎状态机

```text
UNINITIALIZED
      |
      v
WAIT_CALIBRATION -- VERIFY_OK --> PROFILE_READY
      |                                  |
      +-- VERIFY_FAIL --> DEGRADED <-----+
                                         |
                           valid PWM/RPM  |
                                         v
                                      RUNNING
                                         |
                 invalid/stale PWM, bad node, NaN, timeout
                                         v
                                      DEGRADED
                                         |
                          fresh valid table and input window
                                         v
                                      RECOVERING
                                         |
                         tau slew 完成且滤波器状态有效
                                         v
                                      RUNNING
```

状态机只描述自适应引擎；它不取代 `imu_boot_manager` 的双 IMU 生命周期。`PROFILE_READY` 之前输出静态基线，`DEGRADED` 时关闭未验证的 tracking notch、恢复保守 LPF/Kp/R 值，并保留诊断位。状态迁移必须是幂等的，重复 `RADAR_STATUS` 不得重复初始化滤波器。

### 1.3 数值和资源约定

- PWM 统一使用百分比浮点 `0.0f..100.0f`；协议中的 `uint8_t` PWM 先转换为 `float`。
- RMS/加速度使用 `m/s2`，陀螺仪使用 `rad/s`，频率使用 Hz，时间常数使用秒。
- 在 CM7 上使用 `float`，禁止在实时路径引入 `double`。标定采集是否使用 `double` 是已有实现细节，运行时引擎不得依赖它。
- `f0` 必须满足 `0 < f0 < 0.45*fs`；超出 Nyquist 安全区时关闭该 notch 并置 `NOTCH_INVALID`。

## 2. 数学插值与保护算法

### 2.1 节点定义

节点固定为 `P={0,20,40,60,80,100}`。`P0` 使用 30 s 静态窗口的本底标准差；`P1..P5` 使用实际 10 s 振动窗口的 RMS。每个节点应同时保存加速度和陀螺仪三轴/标量 RMS，以及 `quality_ok`、样本数和时间戳。

对 `PWM ∈ [Pi,Pi+1]`：

```text
t       = (PWM-Pi)/(Pi+1-Pi)
V(PWM)  = Vi + t*(Vi+1-Vi)
```

输入先执行 `PWM = clamp(PWM,0,100)`。`NaN`/无穷输入按 0% 处理并置 `INPUT_INVALID`。如果任意插值端点质量无效，默认返回 P0 静态基线并置 `PROFILE_INVALID`；不对未测区间作高阶拟合或无界外推。也可以在产品配置中启用“最近有效节点保持”，但必须作为显式策略位，不能隐式改变结果。

### 2.2 C99 插值、边界和异常保护原型

```c
#include <math.h>
#include <stddef.h>
#include <stdint.h>

#define IMU_ADAPTIVE_NODE_COUNT 6U

typedef struct {
    float pwm_percent;          /* 0,20,40,60,80,100 */
    float accel_rms[3];         /* m/s2: x,y,z */
    float gyro_rms[3];          /* rad/s: x,y,z */
    float accel_total_rms;      /* m/s2 */
    float gyro_total_rms;       /* rad/s */
    uint32_t valid_samples;
    uint8_t quality_ok;
} imu_adaptive_vibration_node_t;

typedef struct {
    imu_adaptive_vibration_node_t node[IMU_ADAPTIVE_NODE_COUNT];
    uint8_t table_valid;
} imu_adaptive_calibration_table_t;

typedef struct {
    float pwm_clamped;
    float accel_rms[3];
    float gyro_rms[3];
    float accel_total_rms;
    float gyro_total_rms;
    uint8_t input_clamped;
    uint8_t input_invalid;
    uint8_t profile_valid;
} imu_adaptive_vibration_estimate_t;

static float imu_adaptive_clampf(float value, float low, float high)
{
    if (!isfinite(value)) {
        return low;
    }
    return value < low ? low : (value > high ? high : value);
}

static uint8_t imu_adaptive_node_finite(
    const imu_adaptive_vibration_node_t *node)
{
    uint8_t axis;

    if (node == NULL || node->quality_ok == 0U ||
        !isfinite(node->pwm_percent) || node->pwm_percent < 0.0f ||
        node->pwm_percent > 100.0f || !isfinite(node->accel_total_rms) ||
        !isfinite(node->gyro_total_rms) || node->accel_total_rms < 0.0f ||
        node->gyro_total_rms < 0.0f) {
        return 0U;
    }
    for (axis = 0U; axis < 3U; ++axis) {
        if (!isfinite(node->accel_rms[axis]) ||
            !isfinite(node->gyro_rms[axis]) || node->accel_rms[axis] < 0.0f ||
            node->gyro_rms[axis] < 0.0f) {
            return 0U;
        }
    }
    return 1U;
}

static void imu_adaptive_copy_node(
    imu_adaptive_vibration_estimate_t *out,
    const imu_adaptive_vibration_node_t *node)
{
    uint8_t axis;

    out->accel_total_rms = node->accel_total_rms;
    out->gyro_total_rms = node->gyro_total_rms;
    for (axis = 0U; axis < 3U; ++axis) {
        out->accel_rms[axis] = node->accel_rms[axis];
        out->gyro_rms[axis] = node->gyro_rms[axis];
    }
}

/* Returns 0 on a valid interpolation or endpoint result, -1 on bad args,
 * and -2 when the table is not usable.  Output is always deterministic. */
int imu_adaptive_interpolate(
    const imu_adaptive_calibration_table_t *table,
    float pwm_percent,
    imu_adaptive_vibration_estimate_t *out)
{
    float pwm;
    uint8_t i;

    if (table == NULL || out == NULL) {
        return -1;
    }
    *out = (imu_adaptive_vibration_estimate_t){0};
    out->input_invalid = isfinite(pwm_percent) ? 0U : 1U;
    pwm = imu_adaptive_clampf(pwm_percent, 0.0f, 100.0f);
    out->input_clamped = (out->input_invalid != 0U || pwm != pwm_percent)
                             ? 1U
                             : 0U;
    out->pwm_clamped = pwm;

    if (table->table_valid == 0U ||
        imu_adaptive_node_finite(&table->node[0]) == 0U) {
        return -2;
    }
    if (pwm <= table->node[0].pwm_percent) {
        imu_adaptive_copy_node(out, &table->node[0]);
        out->profile_valid = 1U;
        return 0;
    }
    for (i = 0U; i + 1U < IMU_ADAPTIVE_NODE_COUNT; ++i) {
        const imu_adaptive_vibration_node_t *left = &table->node[i];
        const imu_adaptive_vibration_node_t *right = &table->node[i + 1U];
        float t;
        uint8_t axis;

        if (pwm > right->pwm_percent) {
            continue;
        }
        if (imu_adaptive_node_finite(left) == 0U ||
            imu_adaptive_node_finite(right) == 0U ||
            right->pwm_percent <= left->pwm_percent) {
            imu_adaptive_copy_node(out, &table->node[0]);
            out->profile_valid = 0U;
            return -2;
        }
        t = (pwm - left->pwm_percent) /
            (right->pwm_percent - left->pwm_percent);
        t = imu_adaptive_clampf(t, 0.0f, 1.0f);
        for (axis = 0U; axis < 3U; ++axis) {
            out->accel_rms[axis] = left->accel_rms[axis] +
                t * (right->accel_rms[axis] - left->accel_rms[axis]);
            out->gyro_rms[axis] = left->gyro_rms[axis] +
                t * (right->gyro_rms[axis] - left->gyro_rms[axis]);
        }
        out->accel_total_rms = left->accel_total_rms +
            t * (right->accel_total_rms - left->accel_total_rms);
        out->gyro_total_rms = left->gyro_total_rms +
            t * (right->gyro_total_rms - left->gyro_total_rms);
        out->profile_valid = 1U;
        return 0;
    }

    /* The loop includes the 100% endpoint; this is defensive for a table
     * whose last PWM value is slightly below 100 due to serialization. */
    if (imu_adaptive_node_finite(&table->node[IMU_ADAPTIVE_NODE_COUNT - 1U])
        != 0U) {
        imu_adaptive_copy_node(out,
            &table->node[IMU_ADAPTIVE_NODE_COUNT - 1U]);
        out->profile_valid = 1U;
        return 0;
    }
    imu_adaptive_copy_node(out, &table->node[0]);
    out->profile_valid = 0U;
    return -2;
}
```

### 2.3 一阶滞后参数平滑

每个运行时参数都必须按同一时间基准执行：

```text
gamma = dt/(tau+dt)
theta[k] = theta[k-1] + gamma*(theta_target[k]-theta[k-1])
```

`dt` 应由单调 `imu_time_now_us()` 计算并限制在 `[0,0.1] s`；`tau` 配置限制在 `[0.1,0.2] s`。首次有效样本直接初始化，避免从零产生长暂态。

```c
static float imu_adaptive_slew_float(float previous, float target,
                                     float dt_s, float tau_s,
                                     uint8_t *initialized)
{
    float gamma;

    if (!isfinite(target)) {
        return previous;
    }
    if (initialized != NULL && *initialized == 0U) {
        *initialized = 1U;
        return target;
    }
    dt_s = imu_adaptive_clampf(dt_s, 0.0f, 0.1f);
    tau_s = imu_adaptive_clampf(tau_s, 0.1f, 0.2f);
    gamma = dt_s / (tau_s + dt_s);
    return previous + gamma * (target - previous);
}
```

## 3. 姿态解算动态注入引擎

### 3.1 Mahony 加速度修正增益

加速度归一化前应先做有效性检查。令 `r=RMS_acc_total(PWM)`，目标比例为：

```text
Kp_target = Kp0/(1+kappa*r*r)
Kp_runtime = slew(Kp_runtime,Kp_target)
```

`Kp` 只衰减重力方向误差反馈，不能衰减陀螺仪传播；积分项 `Ki` 也应在加速度门控失败时停止积分，防止高振动把姿态误差积进偏置。

以下是保持原 `MahonyAHRSupdateIMU()` 调用点、增加上下文的改造原型：

```c
typedef struct {
    float q0, q1, q2, q3;
    float integral_fb[3];
    float kp;
    float ki;
} mahony_imu_t;

void MahonyAHRSupdateIMUAdaptive(mahony_imu_t *ahrs,
                                 float gx, float gy, float gz,
                                 float ax, float ay, float az,
                                 float dt_s,
                                 float kp_runtime,
                                 float ki_runtime,
                                 uint8_t accel_trusted)
{
    float half_err_x = 0.0f;
    float half_err_y = 0.0f;
    float half_err_z = 0.0f;
    const float accel_norm = sqrtf(ax * ax + ay * ay + az * az);

    if (ahrs == NULL || !isfinite(dt_s) || dt_s <= 0.0f ||
        !isfinite(gx) || !isfinite(gy) || !isfinite(gz)) {
        return;
    }
    if (accel_trusted != 0U && isfinite(accel_norm) && accel_norm > 1.0e-6f) {
        const float inv_norm = 1.0f / accel_norm;
        const float ax_n = ax * inv_norm;
        const float ay_n = ay * inv_norm;
        const float az_n = az * inv_norm;
        const float half_vx = ahrs->q1 * ahrs->q3 - ahrs->q0 * ahrs->q2;
        const float half_vy = ahrs->q0 * ahrs->q1 + ahrs->q2 * ahrs->q3;
        const float half_vz = ahrs->q0 * ahrs->q0 - 0.5f +
                             ahrs->q3 * ahrs->q3;

        half_err_x = (ay_n * half_vz - az_n * half_vy);
        half_err_y = (az_n * half_vx - ax_n * half_vz);
        half_err_z = (ax_n * half_vy - ay_n * half_vx);
        if (ki_runtime > 0.0f && isfinite(ki_runtime)) {
            ahrs->integral_fb[0] += ki_runtime * half_err_x * dt_s;
            ahrs->integral_fb[1] += ki_runtime * half_err_y * dt_s;
            ahrs->integral_fb[2] += ki_runtime * half_err_z * dt_s;
            gx += ahrs->integral_fb[0];
            gy += ahrs->integral_fb[1];
            gz += ahrs->integral_fb[2];
        }
    }
    kp_runtime = imu_adaptive_clampf(kp_runtime, 0.0f, 20.0f);
    gx += 2.0f * kp_runtime * half_err_x;
    gy += 2.0f * kp_runtime * half_err_y;
    gz += 2.0f * kp_runtime * half_err_z;

    /* Preserve the existing frame/sign convention, but propagate from one
     * quaternion snapshot so q0 updates cannot alias later equations. */
    {
        const float q0 = ahrs->q0;
        const float q1 = ahrs->q1;
        const float q2 = ahrs->q2;
        const float q3 = ahrs->q3;
        const float half_dt = 0.5f * dt_s;
        float nq0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * half_dt;
        float nq1 = q1 + ( q0 * gx + q2 * gz - q3 * gy) * half_dt;
        float nq2 = q2 + ( q0 * gy - q1 * gz + q3 * gx) * half_dt;
        float nq3 = q3 + ( q0 * gz + q1 * gy - q2 * gx) * half_dt;
        const float norm = sqrtf(nq0 * nq0 + nq1 * nq1 +
                                 nq2 * nq2 + nq3 * nq3);

        if (isfinite(norm) && norm > 1.0e-6f) {
            const float inv_norm = 1.0f / norm;
            ahrs->q0 = nq0 * inv_norm;
            ahrs->q1 = nq1 * inv_norm;
            ahrs->q2 = nq2 * inv_norm;
            ahrs->q3 = nq3 * inv_norm;
        }
    }
}
```

实现时必须修正上述原型中“原地更新 q0 后再使用 q0”的别名问题：先复制 `q0..q3` 到局部变量，再一次性写回并归一化。这一点应由单元测试中的四元数范数断言覆盖。`accel_trusted=0` 时姿态依赖陀螺仪传播，不能把加速度样本强行归一化。

### 3.2 EKF 加速度测量噪声矩阵

对角线按轴独立赋权：

```text
Rxx = R0x + beta_x*RMS_x^2
Ryy = R0y + beta_y*RMS_y^2
Rzz = R0z + beta_z*RMS_z^2
```

`R0` 和 `beta` 必须与 EKF 的测量单位一致，并通过 `R_min/R_max` 限幅。矩阵非对角线保持现有观测模型设定，不因 PWM 自动改成相关噪声。

```c
typedef struct {
    float r_acc[3][3];
    float r_min[3];
    float r_max[3];
} imu_ekf_measurement_t;

void imu_ekf_apply_accel_noise(imu_ekf_measurement_t *measurement,
                               const float accel_rms[3],
                               const float r0[3],
                               const float beta[3])
{
    uint8_t axis;

    if (measurement == NULL || accel_rms == NULL || r0 == NULL ||
        beta == NULL) {
        return;
    }
    for (axis = 0U; axis < 3U; ++axis) {
        float value = r0[axis] + beta[axis] *
                      accel_rms[axis] * accel_rms[axis];
        if (!isfinite(value)) {
            value = measurement->r_max[axis];
        }
        value = imu_adaptive_clampf(value, measurement->r_min[axis],
                                    measurement->r_max[axis]);
        measurement->r_acc[axis][axis] = value;
    }
}
```

R 更新应在 EKF 测量更新之前、与状态协方差同一任务上下文执行。禁止在 DMA/ISR 中写 EKF 矩阵；如果 EKF 不是线程安全的，使用单生产者快照在融合任务内消费。

## 4. 数字滤波链动态调度

### 4.1 自适应 LPF 和转速跟踪 notch

```text
fc_target = max(fc_min, fc_base - lambda*PWM_percent)
f0_target = RPM(PWM)/60
f1_target = 2*f0_target       (仅在第二谐波已由标定确认时启用)
```

`RPM(PWM)` 应是标定表或测量闭环的结果，不得把 PWM 百分比直接当 RPM。`fc_target` 仍需小于 `0.45*fs`。高转速而 `f0` 无效时关闭 notch，LPF 保持最后一个有效目标并置诊断位。

### 4.2 双二阶 Biquad 原型

```c
typedef struct {
    float b0, b1, b2, a1, a2;
} imu_biquad_coeff_t;

typedef struct {
    imu_biquad_coeff_t coeff;
    imu_biquad_coeff_t target;
    float x1, x2, y1, y2;
    uint8_t enabled;
} imu_biquad_t;

static void imu_biquad_design_notch(float fs_hz, float center_hz, float q,
                                    imu_biquad_coeff_t *out)
{
    const float pi = 3.14159265358979323846f;
    float w0;
    float s;
    float c;
    float alpha;
    float a0;

    if (out == NULL || !isfinite(fs_hz) || !isfinite(center_hz) ||
        !isfinite(q) || fs_hz <= 0.0f || center_hz <= 0.0f ||
        center_hz >= 0.45f * fs_hz || q < 0.25f) {
        if (out != NULL) {
            *out = (imu_biquad_coeff_t){1.0f, 0.0f, 0.0f, 0.0f, 0.0f};
        }
        return;
    }
    w0 = 2.0f * pi * center_hz / fs_hz;
    s = sinf(w0);
    c = cosf(w0);
    alpha = s / (2.0f * q);
    a0 = 1.0f + alpha;
    out->b0 = 1.0f / a0;
    out->b1 = (-2.0f * c) / a0;
    out->b2 = 1.0f / a0;
    out->a1 = (-2.0f * c) / a0;
    out->a2 = (1.0f - alpha) / a0;
}

static void imu_biquad_design_lpf(float fs_hz, float cutoff_hz, float q,
                                  imu_biquad_coeff_t *out)
{
    const float pi = 3.14159265358979323846f;
    float w0;
    float s;
    float c;
    float alpha;
    float a0;

    if (out == NULL || !isfinite(fs_hz) || !isfinite(cutoff_hz) ||
        !isfinite(q) || fs_hz <= 0.0f || cutoff_hz <= 0.0f ||
        cutoff_hz >= 0.45f * fs_hz || q < 0.25f) {
        if (out != NULL) {
            *out = (imu_biquad_coeff_t){1.0f, 0.0f, 0.0f, 0.0f, 0.0f};
        }
        return;
    }
    w0 = 2.0f * pi * cutoff_hz / fs_hz;
    s = sinf(w0);
    c = cosf(w0);
    alpha = s / (2.0f * q);
    a0 = 1.0f + alpha;
    out->b0 = ((1.0f - c) * 0.5f) / a0;
    out->b1 = (1.0f - c) / a0;
    out->b2 = ((1.0f - c) * 0.5f) / a0;
    out->a1 = (-2.0f * c) / a0;
    out->a2 = (1.0f - alpha) / a0;
}

static void imu_biquad_slew_coeff(imu_biquad_t *filter, float dt_s,
                                  float tau_s)
{
    float gamma;

    if (filter == NULL) {
        return;
    }
    dt_s = imu_adaptive_clampf(dt_s, 0.0f, 0.1f);
    tau_s = imu_adaptive_clampf(tau_s, 0.1f, 0.2f);
    gamma = dt_s / (tau_s + dt_s);
    filter->coeff.b0 += gamma * (filter->target.b0 - filter->coeff.b0);
    filter->coeff.b1 += gamma * (filter->target.b1 - filter->coeff.b1);
    filter->coeff.b2 += gamma * (filter->target.b2 - filter->coeff.b2);
    filter->coeff.a1 += gamma * (filter->target.a1 - filter->coeff.a1);
    filter->coeff.a2 += gamma * (filter->target.a2 - filter->coeff.a2);
}

static float imu_biquad_process(imu_biquad_t *filter, float input)
{
    float output;

    if (filter == NULL || filter->enabled == 0U) {
        return input;
    }
    if (!isfinite(input)) {
        return filter->y1;
    }
    output = filter->coeff.b0 * input + filter->coeff.b1 * filter->x1 +
             filter->coeff.b2 * filter->x2 - filter->coeff.a1 * filter->y1 -
             filter->coeff.a2 * filter->y2;
    if (!isfinite(output)) {
        return filter->y1;
    }
    filter->x2 = filter->x1;
    filter->x1 = input;
    filter->y2 = filter->y1;
    filter->y1 = output;
    return output;
}

typedef struct {
    imu_biquad_t lpf;
    imu_biquad_t notch[2];
    float fs_hz;
    float fc_runtime_hz;
    float last_f0_hz;
    float last_fc_hz;
} imu_dual_notch_chain_t;

void imu_dual_notch_update(imu_dual_notch_chain_t *chain,
                           float pwm_percent, float rpm,
                           float fc_base_hz, float fc_min_hz,
                           float lambda_hz_per_percent, float q,
                           float dt_s)
{
    float fc;
    float f0;

    if (chain == NULL || !isfinite(chain->fs_hz) || chain->fs_hz <= 0.0f) {
        return;
    }
    pwm_percent = imu_adaptive_clampf(pwm_percent, 0.0f, 100.0f);
    fc = fc_base_hz - lambda_hz_per_percent * pwm_percent;
    fc = imu_adaptive_clampf(fc, fc_min_hz, 0.45f * chain->fs_hz);
    chain->fc_runtime_hz = fc;
    if (fabsf(fc - chain->last_fc_hz) > 0.25f) {
        imu_biquad_design_lpf(chain->fs_hz, fc, 0.70710678f,
                              &chain->lpf.target);
        chain->lpf.enabled = 1U;
        chain->last_fc_hz = fc;
    }

    if (!isfinite(rpm) || rpm <= 0.0f) {
        chain->notch[0].enabled = 0U;
        chain->notch[1].enabled = 0U;
        return;
    }
    f0 = rpm / 60.0f;
    if (f0 >= 0.45f * chain->fs_hz) {
        chain->notch[0].enabled = 0U;
        chain->notch[1].enabled = 0U;
        return;
    }
    if (fabsf(f0 - chain->last_f0_hz) > 0.5f) {
        imu_biquad_design_notch(chain->fs_hz, f0, q,
                                &chain->notch[0].target);
        if (2.0f * f0 < 0.45f * chain->fs_hz) {
            imu_biquad_design_notch(chain->fs_hz, 2.0f * f0, q,
                                    &chain->notch[1].target);
            chain->notch[1].enabled = 1U;
        } else {
            chain->notch[1].enabled = 0U;
        }
        chain->notch[0].enabled = 1U;
        chain->last_f0_hz = f0;
    }
    imu_biquad_slew_coeff(&chain->lpf, dt_s, 0.15f);
    imu_biquad_slew_coeff(&chain->notch[0], dt_s, 0.15f);
    imu_biquad_slew_coeff(&chain->notch[1], dt_s, 0.15f);
}

float imu_dual_notch_process(imu_dual_notch_chain_t *chain, float sample)
{
    sample = imu_biquad_process(&chain->lpf, sample);
    sample = imu_biquad_process(&chain->notch[0], sample);
    return imu_biquad_process(&chain->notch[1], sample);
}
```

系数重算只发生在频率变化事件，不发生在每个采样点；`sinf/cosf` 不是实时热路径。对 Cortex-M7 可替换为 CMSIS-DSP 快速三角函数，但必须用同一回放向量验证幅频和相位。系数 slew 期间保留历史状态，避免阶跃；若系数从无效恢复，先用单位滤波器并用首个有效样本预置 `x/y` 状态。

系数 slew 使用显式字段更新，避免依赖结构体布局或严格别名规则；这也便于 MISRA/静态分析逐字段检查。

## 5. 动态 ZUPT 判决引擎

### 5.1 判决定义

滑动窗口保存加速度模长或去重力后的加速度分量。窗口内：

```text
mean = sum/N
var  = max(0, sum_sq/N - mean*mean)
```

原始需求给出 `Var_threshold = Var_static_base + 3*RMS_acc_total`。严格按量纲检查，方差与 RMS 的单位不同；本规范的可执行实现采用：

```text
Var_threshold = Var_static_base + 3*RMS_acc_total^2
```

如果现有标定表把 `RMS_acc_total` 定义成“方差等效量”，可将平方移除，但必须把这个选择固化为版本化配置，不可在代码中隐式混用。

静止需同时满足 `window_count >= N_min`、`variance <= threshold`、`gyro_norm <= gyro_threshold(PWM)`、加速度样本有效且未过期。只有 `stationary=true` 连续达到 `hold_count` 才允许更新陀螺仪零偏或 ZUPT 速度状态；离开静止时立即清除 ZUPT 锁存。

### 5.2 C99 固定窗口实现

```c
#define IMU_ZUPT_WINDOW_SIZE 32U

typedef struct {
    float value[IMU_ZUPT_WINDOW_SIZE];
    float sum;
    float sum_sq;
    uint16_t count;
    uint16_t index;
    uint8_t stationary;
    uint8_t hold_count;
} imu_zupt_engine_t;

void imu_zupt_init(imu_zupt_engine_t *zupt)
{
    if (zupt != NULL) {
        *zupt = (imu_zupt_engine_t){0};
    }
}

static float imu_zupt_variance(const imu_zupt_engine_t *zupt)
{
    float mean;
    float variance;

    if (zupt == NULL || zupt->count == 0U) {
        return INFINITY;
    }
    mean = zupt->sum / (float)zupt->count;
    variance = zupt->sum_sq / (float)zupt->count - mean * mean;
    return variance > 0.0f ? variance : 0.0f;
}

uint8_t imu_zupt_update(imu_zupt_engine_t *zupt,
                        float accel_norm,
                        float gyro_norm,
                        float accel_total_rms,
                        float var_static_base,
                        float gyro_threshold,
                        uint8_t sample_valid)
{
    float threshold;
    float variance;
    uint8_t now_static;

    if (zupt == NULL || sample_valid == 0U || !isfinite(accel_norm) ||
        !isfinite(gyro_norm) || !isfinite(accel_total_rms) ||
        !isfinite(var_static_base) || !isfinite(gyro_threshold) ||
        accel_total_rms < 0.0f || var_static_base < 0.0f) {
        if (zupt != NULL) {
            zupt->stationary = 0U;
            zupt->hold_count = 0U;
        }
        return 0U;
    }
    if (zupt->count == IMU_ZUPT_WINDOW_SIZE) {
        const float old = zupt->value[zupt->index];
        zupt->sum -= old;
        zupt->sum_sq -= old * old;
    } else {
        ++zupt->count;
    }
    zupt->value[zupt->index] = accel_norm;
    zupt->sum += accel_norm;
    zupt->sum_sq += accel_norm * accel_norm;
    zupt->index = (uint16_t)((zupt->index + 1U) % IMU_ZUPT_WINDOW_SIZE);

    threshold = var_static_base + 3.0f * accel_total_rms * accel_total_rms;
    variance = imu_zupt_variance(zupt);
    now_static = (zupt->count >= 8U && variance <= threshold &&
                  gyro_norm <= gyro_threshold) ? 1U : 0U;
    if (now_static != 0U) {
        if (zupt->hold_count < UINT8_MAX) {
            ++zupt->hold_count;
        }
        if (zupt->hold_count >= 3U) {
            zupt->stationary = 1U;
        }
    } else {
        zupt->hold_count = 0U;
        zupt->stationary = 0U;
    }
    return zupt->stationary;
}
```

窗口大小、`8` 个最小样本、`3` 个连续确认样本和陀螺仪阈值都是初始配置，不是硬件事实。应使用 0/20/100% 合成回放和车体静止实测分别标定，检查误触发、延迟和零偏漂移。

## 6. STM32H757 落地改造与接口设计清单

### 6.1 头文件接口（设计目标）

以下头文件是完整接口草案；实现时应放入 `STM32H757/Middleware/Calibration/`，并由 CM7 CMake 显式加入。所有状态均由一个静态上下文拥有。

```c
#ifndef IMU_ADAPTIVE_VIBRATION_H
#define IMU_ADAPTIVE_VIBRATION_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define IMU_ADAPTIVE_PWM_NODE_COUNT 6U
#define IMU_ADAPTIVE_PWM_MIN_PERCENT 0.0f
#define IMU_ADAPTIVE_PWM_MAX_PERCENT 100.0f
#define IMU_ADAPTIVE_DEFAULT_TAU_S 0.15f
#define IMU_ADAPTIVE_ZUPT_WINDOW_SIZE 32U
#define IMU_ADAPTIVE_NOTCH_COUNT 2U

typedef enum {
    IMU_ADAPTIVE_OK = 0,
    IMU_ADAPTIVE_INVALID_ARG = -1,
    IMU_ADAPTIVE_NOT_READY = -2,
    IMU_ADAPTIVE_PROFILE_INVALID = -3,
    IMU_ADAPTIVE_INPUT_INVALID = -4
} imu_adaptive_status_t;

typedef enum {
    IMU_ADAPTIVE_STATE_UNINITIALIZED = 0,
    IMU_ADAPTIVE_STATE_WAIT_CALIBRATION,
    IMU_ADAPTIVE_STATE_PROFILE_READY,
    IMU_ADAPTIVE_STATE_RUNNING,
    IMU_ADAPTIVE_STATE_DEGRADED,
    IMU_ADAPTIVE_STATE_RECOVERING
} imu_adaptive_state_t;

typedef struct {
    float pwm_percent;
    float accel_rms[3];
    float gyro_rms[3];
    float accel_total_rms;
    float gyro_total_rms;
    uint32_t valid_samples;
    uint8_t quality_ok;
} imu_adaptive_vibration_node_t;

typedef struct {
    imu_adaptive_vibration_node_t node[IMU_ADAPTIVE_PWM_NODE_COUNT];
    float rpm_at_pwm[IMU_ADAPTIVE_PWM_NODE_COUNT];
    float static_var_base;
    float static_gyro_threshold;
    uint32_t calibration_generation;
    uint8_t verified;
} imu_adaptive_calibration_table_t;

typedef struct {
    float kp0;
    float kp_kappa;
    float ekf_r0[3];
    float ekf_beta[3];
    float ekf_r_min[3];
    float ekf_r_max[3];
    float fc_base_hz;
    float fc_min_hz;
    float fc_lambda_hz_per_percent;
    float notch_q;
    float tau_s;
    float zupt_gyro_threshold;
    uint16_t sample_rate_hz;
} imu_adaptive_config_t;

typedef struct {
    float pwm_percent;
    float rpm;
    float accel_rms[3];
    float gyro_rms[3];
    float accel_total_rms;
    float gyro_total_rms;
    float kp_runtime;
    float r_acc[3];
    float fc_runtime_hz;
    float notch_hz[IMU_ADAPTIVE_NOTCH_COUNT];
    float zupt_var_threshold;
    uint8_t pwm_clamped;
    uint8_t profile_valid;
    uint8_t notch_enabled;
    uint8_t accel_trusted;
    uint8_t zupt_stationary;
    imu_adaptive_state_t state;
} imu_adaptive_runtime_params_t;

typedef struct {
    uint32_t pwm_update_count;
    uint32_t invalid_input_count;
    uint32_t profile_miss_count;
    uint32_t notch_reject_count;
    uint32_t zupt_enter_count;
    uint32_t zupt_exit_count;
    uint32_t coefficient_update_count;
    uint64_t last_update_timestamp_us;
} imu_adaptive_diagnostics_t;

imu_adaptive_status_t imu_adaptive_init(const imu_adaptive_config_t *config);
imu_adaptive_status_t imu_adaptive_set_calibration(
    const imu_adaptive_calibration_table_t *table);
imu_adaptive_status_t imu_adaptive_get_rpm_for_pwm(float pwm_percent,
                                                   float *rpm);
imu_adaptive_status_t imu_adaptive_set_pwm(float pwm_percent,
                                           uint64_t timestamp_us);
imu_adaptive_status_t imu_adaptive_update(float dt_s, float rpm,
                                           uint64_t timestamp_us);
imu_adaptive_status_t imu_adaptive_update_zupt(float accel_norm,
                                               float gyro_norm,
                                               uint8_t sample_valid);
imu_adaptive_status_t imu_adaptive_get_runtime(
    imu_adaptive_runtime_params_t *runtime);
imu_adaptive_status_t imu_adaptive_get_diagnostics(
    imu_adaptive_diagnostics_t *diagnostics);
imu_adaptive_state_t imu_adaptive_get_state(void);
uint8_t imu_adaptive_is_ready(void);

#ifdef __cplusplus
}
#endif

#endif /* IMU_ADAPTIVE_VIBRATION_H */
```

实现文件中的静态上下文至少包含一份 calibration table、config、runtime、LPF/双 notch 状态、三轴 ZUPT 状态和 diagnostics；`set_calibration()` 不得保留调用者指针，必须复制固定大小结构体，并以 `calibration_generation` 让运行时发现表更新。若更新与 IMU 任务并发，使用短临界区/双缓冲静态结构，不在实时路径调用 FreeRTOS 长时间锁。

### 6.2 与现有模块的接入时序

1. `imu_boot_manager` 在 `VERIFY` 成功后读取 `imu_vibration_get_lsm_profile()`/BMI 结果和 30 s 静态统计，组装 `imu_adaptive_calibration_table_t`，调用 `imu_adaptive_set_calibration()`；失败则保持 `WAIT_CALIBRATION/DEGRADED`。
2. `s3_service.c` 的 `RADAR_STATUS` 解析保留当前 online、范围和 3000 ms stale 规则，调用 `imu_adaptive_set_pwm()`。调用只写静态输入快照；重算由 IMU 任务完成。无效/超时回到 0% 静态基线，并同时调用现有 `imu_filter_set_radar_pwm(0U)` 兼容路径。
3. `imu_manager.c` 继续负责 LSM303/BMI323 采样和锁保护，不在驱动 ISR 内计算 RMS 或滤波器系数。现有 `imu_update()` 发布统一快照后，由 `imu_runtime.c` 在滤波/姿态调用前执行 `imu_adaptive_update(dt,rpm,ts)`。
4. 现有 `imu_filter_update()` 在第一阶段仍运行，adaptive LPF/notch 作为其前置或显式旁路；未经回放证明不得直接替换 `imu_filter_alpha` 语义。
5. `attitude.c`/Mahony 或未来 EKF 在同一融合任务中读取 `imu_adaptive_get_runtime()`；加速度门控只影响重力修正和 `R_acc`，不改变原始传感器快照。
6. ZUPT 在滤波后输入上更新；只有 `imu_adaptive_update_zupt()` 返回稳定状态时，偏置/速度零化逻辑才可提交。BMI323 错误必须非阻塞，不能停止现有 LSM303 10 ms 路径。
7. 任何自适应诊断应走现有日志/调试通道。不要把 runtime 参数追加到 `0x0202` 或 `0x0208`；需要二进制遥测时另行分配 MSG_ID，并保持旧接收端可解析。

### 6.3 资源、实时性和电气边界

| 项目 | 设计约束 | 验证方式 |
| --- | --- | --- |
| RAM | 固定节点、双 notch、32 点 ZUPT 窗口；禁止堆分配 | `sizeof`、链接 map、静态扫描 `malloc|free` |
| Flash | `sinf/cosf` 只在系数更新路径；可选 CMSIS-DSP | CM7 map、`-ffunction-sections` map、周期统计 |
| CPU | 插值/slew 为 O(1)；每样本仅 biquad MAC 和窗口更新 | DWT cycle counter，报告平均/最大 WCET |
| 栈 | 不在实时函数声明大数组；诊断使用静态缓冲 | FreeRTOS stack high-water mark |
| 中断/DMA | ISR 只提交时间戳/样本，任务消费；不持锁、不打印 | ISR latency、DMA overrun、环形缓冲溢出计数 |
| 外设竞争 | 不新增 SPI/I2C 事务；RPM/PWM 来自已有服务 | 总线 trace、任务优先级审查 |
| 电源/硬件 | PWM、RPM 和振动响应必须上电实测；源码不能证明电机转速 | 示波器/逻辑分析仪/IMU 原始回放 |

### 6.4 验证清单

- **主机数学向量**：PWM `-1,0,10,20,30,80,100,101,NaN`；检查钳位、端点连续性、坏节点回退和有限值。
- **slew 阶跃**：20% -> 80% 和 80% -> 20%，验证单调、无过冲、时间常数误差和首次初始化行为。
- **Mahony/EKF**：静止、恒定 yaw rate、高振动合成数据；四元数范数、`R` 正值、加速度门控和偏置积分冻结。
- **Biquad**：对每个 PWM/RPM 点测中心频率、陷波深度、系数稳定性；验证 `f0>=0.45fs` 自动关闭和系数更新不产生爆音/尖峰。
- **ZUPT**：窗口长度、方差非负、静止进入/退出延迟、100% PWM 静止车体不误判运动。
- **静态检查/构建**：确认无 `malloc/free`、无协议结构体改动；CM7 CMake 编译新模块，检查告警、map、栈余量。
- **协议回归**：对 `0x0202` 和 `0x0208` 做字节级 golden vector；新模块不得改变长度、端序、CRC 或旧字段。
- **硬件/集成（单独证据）**：只有在明确授权烧录后，才验证 PWM 波形、RPM、IMU RMS、UART、BLE 和姿态效果。构建通过不能替代这些证据。

## 附录 A：开源系统对标与工业经验

当前环境无法稳定直连 GitHub，以下链接和文件路径是应固定 commit 后复核的上游定位；本附录只提炼通用设计经验，不宣称已在本机读取了指定 commit 的源代码。

| 系统/定位 | 可复核上游文件 | 工业级经验 | 本规范采用方式 |
| --- | --- | --- | --- |
| ArduPilot | [`libraries/AP_InertialSensor/AP_InertialSensor_HarmonicNotch.cpp`](https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_InertialSensor/AP_InertialSensor_HarmonicNotch.cpp) / `.h` | 谐波陷波按传感器实例维护状态，中心频率可由转速/FFT 驱动；滤波器重算与采样率、Nyquist 检查绑定，并保留状态/诊断。 | 双 Biquad、f0/RPM 表、限幅、系数更新计数和无效关闭。 |
| ArduPilot EKF3 | [`libraries/AP_NavEKF3/AP_NavEKF3_core.cpp`](https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_NavEKF3/AP_NavEKF3_core.cpp) | 振动/创新质量应影响观测融合门控和健康状态，不能把异常观测直接写进状态；阈值、延迟和 reset 需要可观测。 | `accel_trusted`、`R_acc` 上下界、ZUPT 状态和诊断位分离。 |
| PX4 EKF2/ECL | [`src/modules/ekf2`](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/ekf2) 与 ECL vibration metrics | 记录 delta-angle/delta-velocity clipping、振动指标和创新一致性；指标与估计器状态解耦，供故障诊断、观测拒绝和参数调度使用。具体字段/耦合须按锁定版本核对。 | 运行时 RMS、profile quality、NaN、notch reject 和 ZUPT 计数独立记录，不以日志字符串冒充姿态验收。 |
| Betaflight | [`src/main/flight/gyro_filter.c`](https://github.com/betaflight/betaflight/blob/master/src/main/flight/gyro_filter.c) 及 `filter.c` | dynamic gyro LPF 根据节流/频率输入得到有界 cutoff；Biquad 系数仅在参数变化时更新，热路径保持固定 MAC，实时系统避免频繁 trig/分配。 | PWM 事件只存快照；20 Hz/变化阈值重算；每样本 `process()` 为固定 O(1)。 |
| OpenVINS | [`ov_msckf/src/update/UpdaterZeroVelocity.cpp`](https://github.com/rpng/open_vins/blob/master/ov_msckf/src/update/UpdaterZeroVelocity.cpp) | ZUPT 通常是固定长度 IMU 窗口上的统计检验/创新门控，而不是单一瞬时加速度阈值；需要连续确认、观测协方差和失败回退。 | 滑动方差 + gyro gate + 连续 hold_count；PWM 仅改变噪声补偿，不直接强制静止。 |

### A.1 对标结论

1. **先测量再调度**：RPM/PWM 只是调度输入，不能替代加速度频谱、创新或 clipping 证据。
2. **参数平滑与状态连续性同等重要**：只平滑 `Kp/R/fc` 而不处理 Biquad 历史状态，仍会产生瞬态尖峰。
3. **估计器必须可退化**：profile 缺失、RPM 失锁、传感器无效或 CPU 过载时，应关闭未验证特性并回到静态基线。
4. **诊断不是验收**：vibration metric、滤波器更新次数和 ZUPT 计数只能说明代码路径运行，不能证明传感器、电机、姿态或通信链路正确。
5. **固定版本复核**：集成前将上游仓库 commit、文件 hash、所引用的数学公式和本地测试向量记录到设计决策记录，避免把上游主分支变化带入产品参数。

### A.2 Smart_Car 专用禁止事项

- 不修改 `SCBP_MSG_ID_IMU_CAL_STATUS (0x0202)`、`SCBP_MSG_ID_DUAL_IMU_STATUS (0x0208)` 的既有负载。
- 不把 BMI323 诊断数据未经明确融合契约直接塞入现有 LSM303 `attitude.c`。
- 不在 `s3_service` UART 回调中调用 `sinf/cosf`、打印长日志或等待互斥锁。
- 不把“PWM 已收到”“CM7 已编译”写成“雷达转速/振动补偿/姿态精度已验证”。
