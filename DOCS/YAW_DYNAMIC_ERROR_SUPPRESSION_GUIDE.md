# 四轮差速底盘 YAW 动态误差抑制：技术学习与实施指南

> 文档属性：GitHub 调研、算法设计和本地控制实现记录。本轮只修改列出的
> CM7 航向/轮速控制文件；没有修改轮子映射、轮距、`WHEEL_TRIM`、S3/App
> 协议或硬件配置。
>
> 证据分级：**源码事实**来自当前 checkout；**上游实现**固定到文末列出的 GitHub commit；**建议**是待评审、待台架验证的设计，不能等同于已集成或已实车验收。

## 0. 当前工程基线

| 项目 | 当前源码证据 | 含义/限制 |
| --- | --- | --- |
| 轮序与轮距 | `STM32H757/Application/Chassis/chassis_kinematics.h:10-21`：`RR/RF/LR/LF`，`193.0 mm` | 用户已确认底层配置、轮子映射和符号正确；本文将其作为固定契约，不把它列为本问题的排查项 |
| 运动学 | `STM32H757/Application/Chassis/chassis_kinematics.c:18-31`：`v_L=v-wL/2`、`v_R=v+wL/2` | 沿用现有正角速度/左右轮分配定义；上游资料不得覆盖本地已确认约定 |
| 航向任务 | `STM32H757/Application/Chassis/chassis_task.c:21-31, 189-400` | 任务周期 10 ms；已有线速度加速度斜坡；输出门控、姿态有效性和急停清零已存在 |
| 航向控制 | `chassis_task.c:328-402` | 当前误差为 `cur_yaw - target_yaw`；增益 `Kp=0.28, Ki=0.085, Kd=0.006`；保留固定 10 ms 航向积分，加入零默认 `w_ff`、负阻尼项和 `5 rad/s^2` 输出 slew 限幅 |
| 轮速反馈 | `STM32H757/Middleware/MotorBoard/motor_board_task.c:118-148, 569-618` | `$MSPD` 到帧时采样单调接收时刻；实际 `dt` 传入 Ramp、PI 和反馈低通；状态发布周期仍为 50 ms |
| 动态 dt | `motor_board_task.c:31-32, 118-148` | 移除控制路径对 `MB_PID_DT_SECONDS=0.05f` 的依赖；`dt` 限幅在 `2 ms..100 ms`，首帧/异常帧跳过一次控制更新并重置 PID 历史 |
| 轮速 PI/前馈 | `STM32H757/CM7/Core/Src/pid_controller.c:94-155`、`wheel_control_params.h:12-48` | 已有目标斜坡、反馈一阶低通、线性 `Kv=1.40`、平滑库仑项、单轮 trim 和死区；并非完全没有补偿 |
| IMU | `STM32H757/Middleware/Attitude/DualAHRS/dual_ahrs.c:718-855, 944-954` | BMI323 数据流注释为 200 Hz；gyro 先经 DualAHRS LPF，`dual_ahrs_get_heading_state()` 返回有效 yaw 和滤波后 `gyro_z` |
| 最终运动权威 | 根 README 的系统边界；`chassis_task.c` 通过 `motor_board_set_target_wheel_speeds()` 下发 | 建议只能接入现有 CM7 控制边界，不绕过 S3/IMU/急停门控 |

用户任务中给出的 100 Hz 航向环、现有增益及 50 ms 失配描述，与基线源码一致；本轮已消除控制路径的固定 50 ms 依赖，但实际 `$MSPD` 周期、串口抖动、执行延迟和电机物理响应仍需日志测量。

### 本地契约优先

本项目已经确认底层通信配置、轮子映射、编码器方向、运动学符号和轮距没有问题。它们在本文中只作为固定输入，不是“建议重新核对”的根因假设。左右两侧仍可能存在摩擦、负载、驱动增益或响应时间常数差异，但那属于被控对象动态不对称，不等于映射或底层配置错误。GitHub/PX4/ROS2 文档只用于借鉴限幅、时间戳、融合和分层控制模式，不具备否决本地实测契约的优先级。

## 一、运动跑偏核心机理

### 1. 控制理论：差速系统是两个执行器共同形成一个 YAW 通道

理想差速模型为

\[
v=\frac{v_R+v_L}{2},\qquad
\omega=\frac{v_R-v_L}{L},\qquad L=0.193\,\mathrm{m}
\]

当左右轮实际增益分别为 `G_R(s)`、`G_L(s)` 时，期望的共模/差模输入会被分裂为：

\[
v_R=G_R(s)(v+\tfrac{L}{2}\omega),\quad
v_L=G_L(s)(v-\tfrac{L}{2}\omega)
\]

若 `G_R != G_L`，即使命令 `omega=0`，也会产生

\[
\omega_{bias}\approx\frac{(G_R-G_L)v}{L}
\]

这解释了“直线时随速度增加而跑偏”。在加速/减速时，电机反电动势、静摩擦脱离和 PWM 饱和改变 `G_R/G_L`，所以偏差不是一个固定角度偏置，而是动态偏置。

航向环还存在两个常见稳定性风险：

1. **双环带宽重叠**：外层航向环在 100 Hz 计算，但轮速反馈只在 `$MSPD` 帧到达时更新；如果内环实际更新较慢或抖动，外环看到的是带延迟的 `omega` 执行器。
2. **积分与饱和**：差速输出受单轮速度上限、PWM 上限和最小可动速度约束。积分继续累加时会造成退出饱和后的过冲。当前航向积分有误差窗和限幅，但仍使用固定 10 ms，应把实际控制周期纳入递推并记录饱和状态。

### 2. 机械/电机响应：动态跑偏来自不匹配的时间常数

典型路径为 `PWM -> 电流/转矩 -> 减速箱 -> 轮胎接地 -> 编码器`。左右侧的以下差异会形成不同的幅值和相位：

- 静摩擦、库仑摩擦和 Stribeck 区域不同；
- 齿轮间隙、轮胎半径、接地压力和滚阻不同；
- 电机板 PWM 更新与 `$MSPD` 反馈不是同一时钟；
- 电池电压下降导致前馈模型整体漂移；
- 转向修正阶跃太大时，轮胎先滑移，编码器报告的速度不再代表车体瞬时速度。

因此补偿应优先作用于“目标变化率、左右同步误差和低速摩擦区”，不能只把 `Kp` 调大。

### 3. 传感器相位滞后：LPF 降噪可能换来相位损失

当前 DualAHRS 对 BMI323 gyro 做低通后才提供 `gyro_z`。低通截止频率越低，噪声越小，但相位延迟越大；外层航向控制再使用该信号作阻尼时，延迟后的阻尼可能在快速变向时变成“滞后修正”。另一方面，轮速差分角速度

\[
\omega_{wheel}=\frac{v_R-v_L}{L}
\]

对编码器量化、帧抖动和打滑敏感，但低频稳态时可校准陀螺偏置。适合的分工是：陀螺提供高频动态，轮速提供低频一致性校正，并在轮速异常或打滑时降低其权重。

## 二、GitHub 典型开源项目拆解

以下链接固定到已核验的 commit；上游项目不是本工程的依赖，代码只能作为模式参考。

### 1. ROS 2 `diff_drive_controller`：速度/加速度/jerk 限制与轮式里程计

- 仓库：[ros-controls/ros2_controllers](https://github.com/ros-controls/ros2_controllers)
- 固定 revision：[`94e74de35f9d04f313aca8f29df66c3a76004aa7`](https://github.com/ros-controls/ros2_controllers/tree/94e74de35f9d04f313aca8f29df66c3a76004aa7)
- 核心文件：[`speed_limiter.hpp`](https://github.com/ros-controls/ros2_controllers/blob/94e74de35f9d04f313aca8f29df66c3a76004aa7/diff_drive_controller/include/diff_drive_controller/speed_limiter.hpp)、[`diff_drive_controller.cpp`](https://github.com/ros-controls/ros2_controllers/blob/94e74de35f9d04f313aca8f29df66c3a76004aa7/diff_drive_controller/src/diff_drive_controller.cpp)、[`odometry.cpp`](https://github.com/ros-controls/ros2_controllers/blob/94e74de35f9d04f313aca8f29df66c3a76004aa7/diff_drive_controller/src/odometry.cpp)。
- 可借鉴模式：`SpeedLimiter::limit(v, v0, v1, period.seconds())` 同时处理速度、一次导数（加速度）和二次导数（jerk）；控制器将实际周期传入限制器，并可发布 limited velocity；里程计显式使用左右轮半径、轮距和周期。
- 对本项目的映射：将 `w_corr` 先经过角速度/角加速度/角 jerk 限制，再做轮速分解；`period.seconds()` 对应 `$MSPD` 或 CM7 控制时间戳。
- 不可直接照搬：ROS2 的实时线程、参数服务和硬件接口不适合直接移植到 FreeRTOS；本项目仍必须保留 `s3_service_is_synced()`、IMU ready、BUS_OFF/超时和急停边界。

### 2. PX4 Rover Differential：分层的速度、角速度和差速执行器控制

- 仓库：[PX4/PX4-Autopilot](https://github.com/PX4/PX4-Autopilot)
- 固定 revision：[`64cbe71af74ddf87b4209c1aedba587a3f345c43`](https://github.com/PX4/PX4-Autopilot/tree/64cbe71af74ddf87b4209c1aedba587a3f345c43)
- 核心目录：[`src/modules/rover_differential`](https://github.com/PX4/PX4-Autopilot/tree/64cbe71af74ddf87b4209c1aedba587a3f345c43/src/modules/rover_differential)，包含 `DifferentialSpeedControl`、`DifferentialRateControl`、`RoverDifferential` 等模块。
- 可借鉴模式：把“期望车速/期望角速度”“角速度控制”“左右执行器分配”分层；差速模式下分别限制速度通道和转向通道，最后再做执行器限幅。这样能避免航向 PI 直接写 PWM，也便于加入左右侧同步误差。
- 版本注意：PX4 主线目录和参数会变化；本条只确认目录和模块名，不把具体增益或参数当作本项目可用数值。

### 3. `robot_localization`：带时间和协方差门控的 EKF

- 仓库：[cra-ros-pkg/robot_localization](https://github.com/cra-ros-pkg/robot_localization)
- 固定 revision：[`7dfb6aa97b2082185d2fac3420888ae8474bfc1a`](https://github.com/cra-ros-pkg/robot_localization/tree/7dfb6aa97b2082185d2fac3420888ae8474bfc1a)
- 核心文件：[`src/ekf.cpp`](https://github.com/cra-ros-pkg/robot_localization/blob/7dfb6aa97b2082185d2fac3420888ae8474bfc1a/src/ekf.cpp)。
- 可借鉴模式：预测步骤接收真实 `delta`；校正步骤只更新有效字段，处理 NaN/Inf、极小协方差、角度创新归一化，并用 Mahalanobis 距离拒绝异常测量。
- 对本项目的轻量化映射：不建议在 CM7 直接移植 Eigen EKF；对于单一 `yaw_rate`，用一阶互补滤波加轮速异常门控即可覆盖主要收益。需要完整位姿、协方差和多传感器异步融合时，再在 ROS2/上位机使用完整 EKF。

### 4. 模式对比

| 能力 | ROS2 diff drive | PX4 Rover Differential | robot_localization | 本项目建议 |
| --- | --- | --- | --- | --- |
| 角速度平滑 | 速度、加速度、jerk，使用实际 period | 分层速度/角速度/执行器控制，参数随版本变化 | 不是执行器限幅器 | 在 CM7 保留 10 ms 外环，新增 `w_cmd` 的 `alpha_max`，可选 jerk |
| 左右轮同步 | 主要依靠半径/轮距标定和反馈里程计 | 差速执行器分配层清晰 | 通过测量协方差间接融合 | 新增 `e_sync=(vR-vL)-(vR_ref-vL_ref)` 的小增益交叉耦合，限幅且可关闭 |
| IMU/轮速融合 | `odometry` 可由轮速反馈更新 | 由 PX4 状态估计链承担 | EKF、创新角度归一化、Mahalanobis 门控 | CM7 采用 gyro 高通/高频 + wheel 低通/低频互补；异常时退回 IMU |
| 摩擦/死区 | 不提供通用 Stribeck 模型 | 执行器输出线性化依平台配置 | 不处理电机摩擦 | 复用当前 `Kv`、平滑库仑项和单轮 trim；增加分段静态 LUT 后再实车验证 |

## 三、算法改进与数学模型

### 3.1 带前馈和角加速度限幅的航向控制

建议把“目标角速度”和“航向反馈修正”分开：

\[
\omega_{raw}=\omega_{ff}+K_p e_\psi+K_i I_e+K_d\,\omega_{imu}
\]

其中：

- `e_psi = wrap_pi(psi - psi_target)`，保持当前工程的 `cur_yaw - target_yaw` 符号；
- `omega_ff` 来自上层期望转率、曲率或轨迹：`omega_ff = v * kappa`；无轨迹曲率时取 0；
- `I_e[k+1] = clamp(I_e[k] + e_psi[k] * dt, I_min, I_max)`，只在速度非零、未失去门控且误差未使输出继续饱和时积分；
- `omega_cmd[k] = clamp(omega_raw, omega_min, omega_max)`；
- 角加速度限幅：

\[
\omega[k]=\omega[k-1]+\operatorname{clamp}(\omega_{cmd}[k]-\omega[k-1],-\alpha_{max}dt,+\alpha_{max}dt)
\]

若仍有打滑，可增加 jerk 限制：先限制 `a=(omega_cmd-omega_prev)/dt` 的变化率，再积分得到 `omega`。`alpha_max` 应以低速台架测试确定，不能直接套用汽车级数值。

### 3.2 基于 `$MSPD` 时间戳的动态 `dt`

对相邻反馈帧时间戳 `t_k`：

\[
dt_k=\operatorname{clamp}(t_k-t_{k-1},dt_{min},dt_{max})
\]

若时间戳回退、为零或超出上限，应丢弃本次积分、重置滤波历史或进入安全降级，而不是把异常间隔直接用于 PID。轮速 PI 的离散递推：

\[
e_k=r_k-y_k,\quad I_k=\operatorname{clamp}(I_{k-1}+K_i e_k dt_k,I_{min},I_{max})
\]
\[
u_k=\operatorname{sat}(K_p e_k+I_k+u_{ff}(r_k))
\]

一阶反馈低通也应使用真实间隔：

\[
\alpha_k=1-e^{-dt_k/\tau},\qquad y^f_k=\alpha_k y_k+(1-\alpha_k)y^f_{k-1}
\]

工程实现上，保留固定周期作为“无时间戳回退值”只用于启动/兼容路径，并记录 `dt_clamped` 计数。

### 3.3 轻量级 IMU + 轮速互补融合

令 `omega_imu` 为去偏后的高频陀螺，`omega_wheel=(vR-vL)/L` 为轮速差分。一个适合 CM7 的标量互补滤波为：

\[
\omega_f=\beta\omega_{imu}+(1-\beta)\omega_{wheel}
\]

其中 `beta=exp(-dt/tau)`；速度越高、轮速越稳定时可降低 `beta`，检测到打滑、轮速不同步或 `$MSPD` 超时则令 `beta -> 1`。更稳健的偏置校正写法为：

\[
b_{k+1}=b_k+K_b(\omega_{wheel}-\omega_{imu})dt,\quad
\omega_f=\beta(\omega_{imu}-b_k)+(1-\beta)\omega_{wheel}
\]

偏置更新必须在低速、左右轮反馈有效且 `|omega_wheel-omega_imu|` 未超门限时进行，避免把打滑当成陀螺偏置。

### 3.4 左右轮交叉耦合控制（CCC）

先由运动学得到参考差速 `d_ref=vR_ref-vL_ref`，由反馈得到 `d_act=vR_act-vL_act`：

\[
e_{sync}=d_{ref}-d_{act},\quad
u_R=u_R^{base}+K_{cc}e_{sync},\quad
u_L=u_L^{base}-K_{cc}e_{sync}
\]

`K_cc` 必须远小于单轮 PI 的主增益，并在 `|v|`、反馈有效性、PWM 余量和打滑检测通过时启用。它补偿的是左右动态差异，不应替代单轮速度环，也不应直接改变航向符号约定。

### 3.5 动态前馈与死区/摩擦模型

建议分三层：

1. 线性项：`u_kv = Kv * v_ref`；
2. 平滑库仑项：`u_c = Uc * tanh(v_ref / v_s)`，避免零速附近阶跃；
3. 分段静态 LUT：按方向、轮号和速度区间标定 `u_static[wheel][direction][bin]`，线性插值后再叠加 PI。

若需要 Stribeck 形式，可用

\[
F(v)=F_c+(F_s-F_c)e^{-(|v|/v_s)^2}+Bv
\]

但参数辨识和温度/电压变化成本较高；本项目优先采用 `Kv + tanh` 和 LUT，等数据充分后再评估 Stribeck。

## 四、STM32 / FreeRTOS C 语言实装示例

以下代码是**可移植核心示例**，用于说明接口和边界；实际本地接入保留现有
Middleware/Application 结构、门控和映射，仍需结合本工程源码审阅和台架测试。

### 4.1 航向控制：前馈、积分、角加速度限幅

```c
#include <math.h>
#include <stdbool.h>

typedef struct {
    float kp, ki, kd;
    float i_term;
    float w_prev;
    float i_limit;
    float w_limit;
    float alpha_max;
    float i_error_limit_rad;
} yaw_ctrl_t;

static float clampf(float x, float lo, float hi)
{
    return x < lo ? lo : (x > hi ? hi : x);
}

static float wrap_pi(float x)
{
    while (x > (float)M_PI)  x -= 2.0f * (float)M_PI;
    while (x < -(float)M_PI) x += 2.0f * (float)M_PI;
    return x;
}

/* Return false when the caller must force zero output. */
bool yaw_ctrl_step(yaw_ctrl_t *c, float yaw_rad, float target_rad,
                   float gyro_z_rad_s, float w_ff_rad_s,
                   float dt_s, bool gate_open, bool moving,
                   float *w_out_rad_s)
{
    float error;
    float raw;
    float max_step;

    if (c == NULL || w_out_rad_s == NULL || !gate_open ||
        !isfinite(yaw_rad) || !isfinite(target_rad) ||
        !isfinite(gyro_z_rad_s) || !isfinite(w_ff_rad_s) ||
        !isfinite(dt_s) || dt_s <= 0.0f || dt_s > 0.100f) {
        if (c != NULL) c->i_term = 0.0f;
        if (w_out_rad_s != NULL) *w_out_rad_s = 0.0f;
        return false;
    }

    error = wrap_pi(yaw_rad - target_rad);
    if (moving && fabsf(error) < c->i_error_limit_rad) {
        c->i_term = clampf(c->i_term + c->ki * error * dt_s,
                           -c->i_limit, c->i_limit);
    } else {
        c->i_term = 0.0f;
    }

    /* Positive local body yaw rate is opposed by negative damping. */
    raw = w_ff_rad_s + c->kp * error + c->i_term - c->kd * gyro_z_rad_s;
    raw = clampf(raw, -c->w_limit, c->w_limit);
    max_step = c->alpha_max * dt_s;
    *w_out_rad_s = c->w_prev +
                   clampf(raw - c->w_prev, -max_step, max_step);
    c->w_prev = *w_out_rad_s;
    return true;
}
```

注意：当前工程的 `Kd` 符号已经与 `cur_yaw-target_yaw` 和车体 Z 轴定义绑定；本项目已确认该底层方向契约，不应依据官方资料或通用 PID 习惯改写。只有在控制律本身变更时，才用回放数据确认闭环极性。

### 4.2 动态 `dt` 轮速 PI 与时间戳保护

```c
typedef struct {
    float kp, ki;
    float integral;
    float feedback_f;
    float tau_s;
    float out_limit;
    uint64_t last_timestamp_us;
    bool timestamp_valid;
} wheel_pi_t;

static bool wheel_dt(wheel_pi_t *p, uint64_t now_us, float *dt_s)
{
    uint64_t delta_us;

    if (p == NULL || dt_s == NULL || now_us == 0U) return false;
    if (!p->timestamp_valid) {
        p->last_timestamp_us = now_us;
        p->timestamp_valid = true;
        return false; /* establish baseline; do not integrate */
    }
    if (now_us <= p->last_timestamp_us) return false;
    delta_us = now_us - p->last_timestamp_us;
    p->last_timestamp_us = now_us;
    *dt_s = clampf((float)delta_us * 1.0e-6f, 0.001f, 0.100f);
    return true;
}

float wheel_pi_step(wheel_pi_t *p, float target, float actual,
                    float feedforward, uint64_t timestamp_us)
{
    float dt_s;
    float alpha;
    float error;
    float output;

    if (p == NULL || !isfinite(target) || !isfinite(actual) ||
        !isfinite(feedforward) || !wheel_dt(p, timestamp_us, &dt_s)) {
        return 0.0f;
    }
    alpha = 1.0f - expf(-dt_s / fmaxf(p->tau_s, 1.0e-4f));
    p->feedback_f += alpha * (actual - p->feedback_f);
    error = target - p->feedback_f;
    p->integral = clampf(p->integral + p->ki * error * dt_s,
                         -p->out_limit, p->out_limit);
    output = feedforward + p->kp * error + p->integral;
    return clampf(output, -p->out_limit, p->out_limit);
}
```

接入 `$MSPD` 时需要在解析层保留帧时间戳或在 `motor_board_update_pid()` 入口采样单调时钟。若协议本身没有时间戳，使用 CM7 接收时刻是可行的近似，但必须记录 UART 接收抖动；不能把“50 ms 状态发布周期”当成每一帧实际周期。

### 4.3 左右轮死区与非对称前馈 LUT

```c
enum { WHEEL_RR = 0, WHEEL_RF = 1, WHEEL_LR = 2, WHEEL_LF = 3 };
typedef struct { float speed; float pwm; } ff_point_t;

/* Values are placeholders; obtain them from a static-load test. */
static const ff_point_t ff_table[4][2][4] = {
    /* Each wheel: forward points, reverse points. */
    {{{  0.0f,   0.0f}, { 80.0f, 260.0f}, {300.0f, 520.0f}, {800.0f, 1180.0f}},
     {{  0.0f,   0.0f}, {-80.0f,-270.0f}, {-300.0f,-535.0f}, {-800.0f,-1200.0f}}},
    {{{  0.0f,   0.0f}, { 80.0f, 250.0f}, {300.0f, 510.0f}, {800.0f, 1160.0f}},
     {{  0.0f,   0.0f}, {-80.0f,-255.0f}, {-300.0f,-525.0f}, {-800.0f,-1180.0f}}},
    {{{  0.0f,   0.0f}, { 80.0f, 265.0f}, {300.0f, 525.0f}, {800.0f, 1190.0f}},
     {{  0.0f,   0.0f}, {-80.0f,-280.0f}, {-300.0f,-540.0f}, {-800.0f,-1210.0f}}},
    {{{  0.0f,   0.0f}, { 80.0f, 255.0f}, {300.0f, 515.0f}, {800.0f, 1170.0f}},
     {{  0.0f,   0.0f}, {-80.0f,-265.0f}, {-300.0f,-530.0f}, {-800.0f,-1190.0f}}}
};

static float wheel_ff_lut(unsigned wheel, float target_mm_s)
{
    unsigned dir = target_mm_s < 0.0f ? 1U : 0U;
    const ff_point_t *p;
    unsigned i;

    if (wheel >= 4U || fabsf(target_mm_s) < 1.0f) return 0.0f;
    p = ff_table[wheel][dir];
    for (i = 1U; i < 4U; ++i) {
        if ((dir == 0U && target_mm_s <= p[i].speed) ||
            (dir == 1U && target_mm_s >= p[i].speed)) {
            const float span = p[i].speed - p[i - 1U].speed;
            const float ratio = span == 0.0f ? 0.0f :
                (target_mm_s - p[i - 1U].speed) / span;
            return p[i - 1U].pwm + ratio * (p[i].pwm - p[i - 1U].pwm);
        }
    }
    return p[3].pwm;
}
```

示例表中的数值只是编译/接口示例，不能直接用于车辆。实际标定应分别测量四个轮、正反转、不同电池电压和冷/热状态，并把 LUT 与当前 `WHEEL_TRIM_M1..M4` 的作用顺序固定下来，避免重复补偿。

### 4.4 互补滤波伪代码

```c
bool yaw_rate_fuse_step(yaw_fuse_t *f, float gyro_z, float v_r,
                        float v_l, float track_m, float dt,
                        bool wheel_valid, bool slip_suspected,
                        float *rate_out)
{
    float wheel_rate;
    float beta;
    if (f == NULL || rate_out == NULL || track_m <= 0.0f ||
        !isfinite(gyro_z) || !isfinite(v_r) || !isfinite(v_l) ||
        !isfinite(dt) || dt <= 0.0f) return false;
    wheel_rate = (v_r - v_l) / track_m;
    beta = expf(-dt / fmaxf(f->tau_s, 1.0e-3f));
    if (!wheel_valid || slip_suspected) beta = 1.0f;
    if (wheel_valid && !slip_suspected && fabsf(wheel_rate - gyro_z) < f->bias_gate) {
        f->gyro_bias += f->bias_gain * (wheel_rate - gyro_z) * dt;
    }
    *rate_out = beta * (gyro_z - f->gyro_bias) + (1.0f - beta) * wheel_rate;
    return isfinite(*rate_out);
}
```

## 五、实车调参与验证步骤

### 阶段 0：安全与仪器准备

- 固定 CM7/S3 镜像版本，确认 `STM32H757/CM7/build/Debug/Smart_Car_H757_CM7.elf` 与被测板一致。
- 保持已确认的底层配置、轮子映射、编码器符号和急停链路不变；低速实车前保留物理断电/急停。
- 记录单调时间戳：航向任务 tick、`$MSPD` 接收时间、PWM 发送时间、四轮目标/实际速度、`yaw/gyro_z`、电池电压、门控状态和丢帧计数。

### 阶段 1：开环双侧标定

1. 关闭航向环和 CCC，只给四轮相同目标；分别测正/反转 80、150、300、600、800 mm/s。
2. 得到每轮 `PWM -> 稳态速度` 曲线、静摩擦起动点和响应时间常数。
3. 先更新 `Kv`、平滑库仑项或 LUT，再调 `WHEEL_TRIM_M1..M4`；目标是同一 PWM 下左右平均速度差小于 5%，而不是先用航向 PI 掩盖差异。

### 阶段 2：轮速闭环频响对齐

1. 用阶跃和小幅 PRBS 测每轮上升时间、超调、稳态误差及 `$MSPD` 帧间隔分布。
2. 先把 `dt` 动态化，确认 `dt_clamped` 为 0 或有明确原因，再调 `Kp/Ki`。
3. 通过单轮闭环使四路带宽接近；若某轮动态明显慢，按已确认的底层配置前提，优先辨识摩擦、负载、驱动增益和时间常数，不直接增大该轮积分。

### 阶段 3：CCC 与打滑门控

1. 低速直线启用很小 `K_cc`，观察 `e_sync` 是否下降且不引入轮速振荡。
2. 做加速/减速和原地转向，验证 CCC 在 PWM 饱和、反馈超时、轮速异常和急停时自动旁路。
3. 以左右反馈差、IMU/轮速角速度差和加速度阈值组合定义 `slip_suspected`；阈值须由实测数据给出。

### 阶段 4：航向阻尼与平滑

1. 暂时令 `Ki=0`，固定低速直线，逐步增加 `Kd` 或阻尼权重，直到角速度振荡明显下降但不产生延迟反向修正。
2. 从较小 `alpha_max` 开始，逐步放宽；记录 `w_raw-w_cmd` 和车体横摆峰值，选择“不打滑且响应足够快”的折中点。
3. 若使用 `w_ff=v*kappa`，沿用已确认的曲率符号及 `RR/RF`、`LR/LF` 分配，再叠加反馈修正；不以外部项目约定覆盖本地符号。

### 阶段 5：航向 P/I 与前馈联调

1. 保持 `Kd` 和 `alpha_max` 固定，增加 `Kp` 直到出现轻微欠阻尼，再回退 20% 左右。
2. 只在低速、误差窗内启用 `Ki`，以消除恒定机械偏差；检查积分在停机、断链、姿态无效、BUS_OFF 和门控丢失时清零。
3. 最后加入 `w_ff`，比较“有/无前馈”时的稳态误差、轮速差、峰值横摆和能耗。

### 阶段 6：验收指标与证据边界

建议每次只改变一个参数组，并保存原始 CSV/串口日志。可采用以下初始验收门槛，再根据底盘能力调整：

| 指标 | 建议门槛 | 证据 |
| --- | --- | --- |
| `$MSPD` `dt` | 中位数、P95、丢帧数均有记录；无时间回退 | CM7 接收日志/分析脚本 |
| 直线稳态跑偏 | 目标速度区间内 yaw 斜率较基线下降 ≥50% | IMU + 轮速同步记录 |
| 加减速横摆峰值 | 不超过基线，且无持续轮滑 | IMU 峰值、轮速差、视频/目测 |
| 左右同步误差 | CCC 开启后 RMS 下降，不能引入振荡 | 四轮目标/实际速度 |
| 安全恢复 | 断链、姿态无效、BUS_OFF、急停均在规定窗口清零 | 故障注入日志和 PWM 抓包 |
| 资源 | CPU、堆、任务栈高水位无回归 | FreeRTOS runtime stats、map 文件 |

主机编译、静态检查和日志回放只能证明软件逻辑；不能证明 UART 电气质量、IMU 真实相位、轮胎打滑、无线链路或车辆安全。实车结论必须绑定具体刷写镜像、硬件配置和原始捕获文件。

## 六、实施状态与修改边界

| 优先级 | 修改位置（未来实施） | 修改原因 | 潜在影响 | 验证方法 |
| --- | --- | --- | --- | --- |
| P0 已实施 | `STM32H757/Middleware/MotorBoard/motor_board_task.c` 的 `$MSPD` 入口 | 消除固定 50 ms 与实际反馈失配 | PID 数值行为改变，需重新调参 | 帧间隔统计、阶跃响应、CPU/栈 |
| P1 已实施 | `STM32H757/Application/Chassis/chassis_task.c` 航向计算段 | 增加 `w_ff`、角加速度限制和负阻尼表达 | 可能增加响应延迟；需实车确认极性和打滑边界 | 正负误差仿真、低速台架、打滑测试 |
| P1 已实施 | `STM32H757/CM7/Core/Src/pid_controller.c` | 动态反馈低通、有限值保护 | 滤波相位/幅值随实际 `dt` 改变 | 单轮频响、丢帧和时间异常注入 |
| P1 后续 | `STM32H757/CM7/Core/Src/pid_controller.c` 或独立 CCC 模块 | 交叉耦合和饱和抗积分扩展 | 左右轮输出耦合，调参复杂度增加 | 单轮/双轮频响、CCC 旁路故障注入 |
| P2 | `STM32H757/Middleware/Attitude/DualAHRS` 旁路融合模块 | 融合 gyro 高频与轮速低频 | 轮速打滑时可能污染 yaw rate | 轮速异常门控、静态偏置和旋转测试 |
| P2 | `STM32H757/CM7/Core/Inc/wheel_control_params.h` | 分方向/分轮 LUT 标定 | Flash/参数维护成本增加 | 冷热、电压、正反转重复标定 |

任何实际代码修改都应单独提交，保持当前 SRP、安全接口、底层配置和轮子映射不变；先完成主机回放和台架测试，再申请低速实车验证。

## 七、已核验来源

1. ROS 2 控制器仓库与 revision：[`ros-controls/ros2_controllers@94e74de`](https://github.com/ros-controls/ros2_controllers/tree/94e74de35f9d04f313aca8f29df66c3a76004aa7)。
2. PX4 Autopilot Rover Differential 目录与 revision：[`PX4/PX4-Autopilot@64cbe71`](https://github.com/PX4/PX4-Autopilot/tree/64cbe71af74ddf87b4209c1aedba587a3f345c43/src/modules/rover_differential)。
3. `robot_localization` EKF 与 revision：[`cra-ros-pkg/robot_localization@7dfb6aa`](https://github.com/cra-ros-pkg/robot_localization/tree/7dfb6aa97b2082185d2fac3420888ae8474bfc1a)。
4. 当前工程源码：`STM32H757/Application/Chassis/`、`STM32H757/Middleware/MotorBoard/`、`STM32H757/Middleware/Attitude/DualAHRS/`、`STM32H757/CM7/Core/Src/pid_controller.c`。
