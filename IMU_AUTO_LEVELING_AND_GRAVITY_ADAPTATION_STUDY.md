# IMU 自动水平调平、局部重力自适应与动态权重方案调研

## 0. 结论摘要与证据边界

本报告将“机械安装外参”“开机静态测得的重力调平”“温度/零偏/比例因子校准”和“运行时姿态估计”分成四个层次。推荐的数据流为：

$$
raw_S \rightarrow bias/scale/temp_S \rightarrow R_{board,S}
\rightarrow R_{level,S} \rightarrow filter \rightarrow AHRS
$$

其中 R_board,S 是固定的板级安装/芯片轴向变换，R_level,S 是本次上电由静态重力估计得到的调平矩阵。两者不能混成一个不可解释的“校准角”。

针对当前 Smart_Car 的建议结论：

1. BMI323 与 LSM303 各自计算、保存和验证一个调平矩阵，不假设两颗器件安装倾角相同。
2. 调平复用现有双 IMU 启动静态窗口，在 IMU_READY 之前完成并冻结；正常运行不因短时加速度改变矩阵。
3. 用静态样本估计 g_local，用于归一化和运动门限；不要继续把 9.80665 m/s² 当成唯一真值。
4. 逆方差权重用于同一个姿态观测量的融合。LSM303 没有陀螺仪，不能与 BMI323 角速度逐轴直接平均；LSM303 应作为重力倾角/磁航向观测参与姿态校正。
5. 权重需要方差下限、有效性门限、时间平滑、过期检测和滞回，避免短时异常导致权重跳变。
6. 倾角超过约 30°、倒置启动、自由落体、振动过大或样本非有限时，报警并回退到上一次有效矩阵或标称板级矩阵。

证据分层如下：

| 标记 | 含义 |
| --- | --- |
| 工程事实 | 当前工作区源码和构建配置直接观察到的函数、宏、数据流 |
| 上游证据 | 固定提交的 PX4、ArduPilot、Betaflight、ROS 源码 |
| 理论推导 | 坐标变换、统计估计和误差传播的数学结论 |
| 建议/待验证 | 面向本工程的设计建议，不等同于设备运行结果 |

本次只新增本报告，不修改 STM32、ESP32、macOS 或其他工程源文件，不编译、不烧录、不宣称硬件运行验收。工作区原有脏改动保持不变。

## 1. 问题本质与坐标系机理分析

### 1.1 坐标系约定

定义右手系：

- S：传感器芯片坐标系，BMI323 或 LSM303 原始轴；
- B：底盘/车体坐标系，通常 x 向前、y 向左、z 向上；
- L：调平后的车体水平坐标系；
- E：地理坐标系，例如 ENU。

采用列向量和主动旋转约定。若 R_BS 把传感器坐标中的向量变换到车体坐标，则

$$
\mathbf v_B = R_{BS}\mathbf v_S,\qquad
R^{-1}=R^T,\qquad R^TR=I,\quad\det R=1.
$$

实际代码也可能采用被动旋转或行向量。实施前必须用已知轴向量做单元测试，不能仅凭矩阵名称判断乘法方向。

### 1.2 倾斜安装造成偏航到横滚/俯仰的窜轴

假设传感器绕车体 y 轴有安装倾角 θ，传感器到车体的旋转为

$$
R_y(\theta)=
\begin{bmatrix}
\cos\theta&0&\sin\theta\\
0&1&0\\
-\sin\theta&0&\cos\theta
\end{bmatrix}.
$$

车辆在水平面纯偏航，真实角速度为

$$
\boldsymbol\omega_B =
\begin{bmatrix}0\\0\\\omega_z\end{bmatrix}.
$$

在倾斜传感器系中：

$$
\boldsymbol\omega_S=R_y(-\theta)\boldsymbol\omega_B
=
\begin{bmatrix}
-\sin\theta\,\omega_z\\
0\\
\cos\theta\,\omega_z
\end{bmatrix}.
$$

如果算法把 ω_Sx 当车体 Roll 角速度，则得到

$$
\omega_{roll,false}\simeq|\omega_z|\sin|\theta|.
$$

例如 θ=10°、ω_z=2 rad/s 时，虚假横滚通道约为 0.347 rad/s；积分一秒约 19.9°。滤波器只能减轻随机噪声，不能消除这个确定性耦合。

绕 x 轴倾斜会把偏航映射到 y 轴；任意安装旋转可写为

$$
\boldsymbol\omega_S=R_{mount}^T\boldsymbol\omega_B.
$$

矩阵的非对角元素就是窜轴项。小角度下 R_mount ≃ I+[ε]×，于是

$$
\delta\boldsymbol\omega\simeq-[\boldsymbol\epsilon]_\times\boldsymbol\omega_B.
$$

误差与安装角和真实角速度成正比，因此它不是适合单纯 inverse-variance 的随机噪声。

### 1.3 加速度也需要调平

静止时加速度计测到的是重力的 specific force，驱动可能输出 +g 或 -g。统一符号后，静态均值为

$$
\bar{\mathbf a}_S=\frac1N\sum_{k=1}^N\mathbf a_{S,k},\quad
g_{local}=\|\bar{\mathbf a}_S\|,\quad
\hat{\mathbf a}_S=\bar{\mathbf a}_S/g_{local}.
$$

调平目标：

$$
R_{level}\hat{\mathbf a}_S=\mathbf g_0,\qquad
\mathbf g_0=[0,0,1]^T.
$$

若当前驱动静止输出 -g_0，目标必须统一改为 -g_0 或先翻转符号；不能让两颗传感器隐含不同约定。

## 2. 自动调平矩阵的数学构造

### 2.1 静态样本和鲁棒统计

简单平均只有在确实静止且没有离群点时成立。建议在现有静态窗口内维护：

$$
\bar{\mathbf a}=\frac{1}{M}\sum_{k\in\mathcal V}\mathbf a_k,\qquad
C_a=\frac{1}{M-1}\sum_{k\in\mathcal V}
(\mathbf a_k-\bar{\mathbf a})(\mathbf a_k-\bar{\mathbf a})^T,
$$

其中 V 是通过有效性门限的样本集合。可先做中值/截尾均值，再计算协方差。至少记录有效样本数、模长均值和标准差、三轴方差、陀螺 RMS、时间戳连续性和丢样数。

### 2.2 Rodrigues 两向量旋转

令 v=a_hat_S，t=g_0，c=v×t，s=|c|，c_0=v·t。s>0 时，旋转轴 u=c/s，满足 sinθ=s、cosθ=c_0。反对称矩阵：

$$
[\mathbf u]_\times=
\begin{bmatrix}
0&-u_z&u_y\\
u_z&0&-u_x\\
-u_y&u_x&0
\end{bmatrix}.
$$

Rodrigues 公式：

$$
R_{level}=I+[\mathbf u]_\times\sin\theta+
[\mathbf u]_\times^2(1-\cos\theta).
$$

实现应优先使用 s 和 c_0，避免先计算 θ=acos(c_0) 再重新求正弦。生成后验证：

$$
\|R_{level}\hat{\mathbf a}_S-\mathbf g_0\|<\epsilon_g,\quad
\|R^TR-I\|_F<\epsilon_R,\quad \det R>0.
$$

奇异点处理：

1. 同向：s<ε_s 且 c_0>0，返回单位阵。
2. 反向：s<ε_s 且 c_0<0，旋转轴不唯一。选择与 v 最不平行的坐标基向量 e_i，令 u=normalize(v×e_i)，使用 π 旋转；若产品不允许倒置启动，则直接拒绝。
3. 非有限或 g_local 太小：拒绝。

### 2.3 两向量四元数

不显式构造轴角时，可构造把 v 旋到 t 的最短弧四元数：

$$
q_{raw}=
\begin{bmatrix}
1+\mathbf v\cdot\mathbf t\\
\mathbf v\times\mathbf t
\end{bmatrix},\qquad q=q_{raw}/\|q_{raw}\|.
$$

这里约定标量在前。由 q 转成 R(q) 后满足 R(q)v=t。反向时 q_raw 接近零，走与 Rodrigues 相同的确定性正交轴分支。四元数便于和已有 AHRS 合成，Rodrigues 便于直接投影向量；两条路线必须用同一组轴向量测试验证。

### 2.4 固定板级旋转与运行时调平解耦

推荐明确表示

$$
R_{BS}^{effective}=R_{level,S}R_{board,S}
$$

或在明确坐标定义下采用等价顺序。关键原则：

- R_board,S 来自原理图、丝印、数据手册或工厂装配；
- R_level,S 只补偿本次上电的重力方向；
- 温度偏置、零偏和比例因子先在传感器轴上校准；
- 不把温漂造成的偏置错误地吸收到水平矩阵。

若把 R_level 写成永久板级校准，换安装位置或温度后会失去可追溯性；若先调平再减去在传感器轴标定的 bias，也会改变 bias 的物理含义。默认顺序应是“传感器轴校准 → 固定板级变换 → 重力调平”。

## 3. 局部重力自适应

### 3.1 g_local 估计

可采用模长中位数：

$$
g_{local}=\operatorname{median}_{k\in\mathcal V}\|\mathbf a_k\|
$$

或使用均值向量模长：

$$
g_{local}=\|\bar{\mathbf a}\|,\qquad
\operatorname{Var}(g)\approx\hat{\mathbf a}^TC_a\hat{\mathbf a}.
$$

两种估计差异超过阈值，通常意味着启动窗口有运动、冲击或饱和，不应静默接受。

### 3.2 运动加速度门限

归一化模长：

$$
\rho_a=\frac{\|\mathbf a_{level}\|}{g_{local}}.
$$

基本静态条件：

$$
|\rho_a-1|<\max(\tau_{abs}/g_{local},\tau_{rel}).
$$

建议分三层：

| 层 | 目的 | 初始策略 |
| --- | --- | --- |
| 硬拒绝 | 自由落体、冲击、饱和 | |a|<0.2g 或 |a|>1.8g |
| 观测降权 | 加减速、过坎 | ||a|-g|>0.1g 时降低加速度校正 |
| 正常校正 | 平稳运动 | ||a|-g|<0.05g 且陀螺 RMS 低 |

这些是初始建议，不是设备验收值。

### 3.3 不应在线漂移调平矩阵

运行中用每个加速度样本重算 R_level 会把纵向加速度解释成车体倾斜，造成姿态跳变。推荐：

1. 启动静态窗口估计 R_level 和 g_local；
2. IMU_READY 后冻结；
3. 仅在显式重新标定、车辆确认静止且完整满足门限时更新；
4. 需要慢速自适应时更新 bias 或观测权重，不追踪瞬时重力方向。

## 4. 动态方差加权

### 4.1 逆方差权重推导

设两个独立观测 z_i=x+n_i，n_i 方差为 σ_i²，线性无偏估计为

$$
\hat x=w_1z_1+w_2z_2,\qquad w_1+w_2=1.
$$

最小化

$$
\operatorname{Var}(\hat x)=w_1^2\sigma_1^2+w_2^2\sigma_2^2
$$

得到

$$
w_i=\frac{\sigma_i^{-2}}{\sum_j\sigma_j^{-2}}.
$$

双传感器形式：

$$
w_{BMI}=\frac{1/\sigma_{BMI}^2}
{1/\sigma_{BMI}^2+1/\sigma_{LSM}^2},\qquad
w_{LSM}=1-w_{BMI}.
$$

振动窗口可用

$$
RMS_i=\sqrt{\frac1M\sum_kx_{i,k}^2},\qquad
\sigma_i^2=\frac1{M-1}\sum_k(x_{i,k}-\bar x_i)^2.
$$

RMS 含零偏、安装误差和低频运动，未经去均值不能直接称为白噪声方差。

### 4.2 从加速度协方差传播到姿态协方差

调平后的倾角近似为

$$
\phi_a\simeq atan2(a_y,a_z),\qquad
\theta_a\simeq atan2(-a_x,\sqrt{a_y^2+a_z^2}).
$$

若 r=[a_x,a_y,a_z]^T、协方差为 C_a，则

$$
C_\eta\simeq JC_aJ^T,\qquad
J=\frac{\partial[\phi_a,\theta_a]}{\partial\mathbf r}.
$$

应使用 C_eta 的对角项或完整 2×2 协方差，而不是直接把原始加速度 RMS 当成 roll/pitch 方差。磁航向还应加入硬铁/软铁校准残差和磁场模长异常门限。

### 4.3 BMI323/LSM303 的非对称性

当前工程中 LSM303 提供加速度/磁力计，BMI323 通过独立采样路径提供陀螺和加速度输入。两者观测维度、采样率、延迟和误差模型不同：

- 不能把 LSM303 加速度和 BMI323 角速度逐轴平均；
- BMI323 陀螺先做各自 bias/scale/温漂校准和调平；
- LSM303 加速度用于 roll/pitch，磁力计用于 yaw；
- 权重进入 Mahony/Madgwick/EKF 的观测增益或残差协方差；
- 时间戳过期、样本质量下降或初始化失败时，权重归零并记录原因。

### 4.4 可落地权重

为防止方差接近零导致溢出：

$$
\tilde\sigma_i^2=clamp(\sigma_i^2,\sigma_{min}^2,\sigma_{max}^2),\qquad
\tilde w_i=\frac{1/\tilde\sigma_i^2}{\sum_j1/\tilde\sigma_j^2}.
$$

权重一阶平滑：

$$
w_i[k]=\alpha_w w_i[k-1]+(1-\alpha_w)\tilde w_i[k].
$$

再设置每周期最大变化量和滞回阈值。姿态融合不要直接平均欧拉角，应使用四元数误差 δq=q_ref⁻¹⊗q_i，在小角度误差向量或误差状态空间中加权。

## 5. 开源飞控/机器人方案横向对比

| 系统 | 数学方法与校准层次 | 开机耗时/采样 | 计算开销（估计） | 奇异点/异常保护 | 动态适应性 |
| --- | --- | --- | --- | --- | --- |
| PX4 | 热偏置、offset、scale 后乘 rotation；板级离散旋转与 sensor-level adjustment 分层 | 参数读取即时；校准由独立流程完成 | 3×3 向量乘约 9 乘法+6 加法；FPU 下很低 | 旋转枚举和参数边界；EKF 对 bias/device change 有 reset 逻辑 | EKF 学习 bias，不在线重算水平矩阵 |
| ArduPilot | 多姿态 accel calibration，旋转重力向量，50 点平均，约 0.05 m/s² 收敛阈值 | 单姿态迭代最多约 10 s | 校准阶段均值和向量旋转；运行时为常规矩阵/四元数 | 临时 ROTATION_NONE；失败恢复；运动检测 | 通过校准参数和 AHRS trim 解耦 |
| Betaflight | 四元数/DCM + Mahony 小角度反馈，快速反平方根 | 轻量启动对准 | 少量向量运算和近似三角函数 | 加速度模长 0.9g~1.1g 才启用反馈 | 由加速度健康门控调整反馈 |
| imu_filter_madgwick | 重力/磁场初始化四元数，运行中融合陀螺、加速度、磁力计 | 首帧/时间戳驱动 | 四元数乘法、归一化和梯度下降 | 自由落体拒绝；磁极奇异处理 | 按观测有效性更新姿态 |
| robot_localization | 假定 IMU 到 base_link 的静态 TF 外参；滤波器使用协方差 | TF 配置即时 | EKF/UKF 主要开销在状态维度 | 协方差 -1 拒绝；差分/相对/去重力参数 | 通过 TF 与 per-sensor covariance 调节 |
| 本工程建议 | 固定板级旋转 + 每传感器启动调平 + 等效姿态协方差权重 | 复用 IMU_READY 前静态窗口 | 每采样 15 FLOP 量级；两个传感器约 30 FLOP | 倾角、倒置、自由落体、方差、过期、非有限检查 | 权重慢速平滑，矩阵不随运动漂移 |

表中 FPU 指令数是算法层估计，不是编译器生成的精确指令清单。精确数目受内联、寄存器分配、加载/存储、CMSIS 优化以及 Debug/Release 选项影响。

## 6. 关键算法源码剖析与伪代码

### 6.1 PX4：校准、板级旋转与 sensor-level adjustment

固定提交：PX4-Autopilot f40090acf4872747660ea366ad3b25e53d7b0ab4。

相关路径：

- src/lib/drivers/accelerometer/
- src/lib/conversion/rotation.cpp
- src/modules/sensors/vehicle_acceleration/
- src/modules/ekf2/

核心数据流等价摘录：

~~~~cpp
Vector3f Accelerometer::Correct(const Vector3f &data)
{
    Vector3f corrected = data - _thermal_offset - _offset;
    corrected = corrected.emult(_scale);
    return _rotation * corrected;
}

void Accelerometer::set_rotation(Rotation rotation)
{
    _rotation = Dcmf(GetSensorLevelAdjustment())
              * get_rot_matrix(rotation);
}
~~~~

设计要点：

1. 热偏置和普通 offset 在旋转前处理。
2. scale 是逐轴比例因子，旋转矩阵不承担温漂补偿。
3. GetSensorLevelAdjustment() 从 SENS_BOARD_X/Y/Z_OFF 读取 sensor-level adjustment；离散板级方向来自 SENS_BOARD_ROT。
4. VehicleAcceleration 在校准后再低通。
5. EKF2 的 learned bias 是估计器状态，校准或设备变化时显式 reset。

PX4 的工业化思路不是每帧根据重力重算姿态角，而是把外参、温度校准和估计器 bias 放在不同层。

参考：

- https://github.com/PX4/PX4-Autopilot/blob/f40090acf4872747660ea366ad3b25e53d7b0ab4/src/lib/conversion/rotation.cpp
- https://github.com/PX4/PX4-Autopilot/blob/f40090acf4872747660ea366ad3b25e53d7b0ab4/src/modules/sensors/vehicle_acceleration/
- https://github.com/PX4/PX4-Autopilot/blob/f40090acf4872747660ea366ad3b25e53d7b0ab4/src/modules/ekf2/

### 6.2 ArduPilot：多姿态加速度校准和失败恢复

固定提交：ArduPilot 3b81960d3c6190b689e4be966b9dcc936d9c0b87。

相关路径：

- libraries/AP_InertialSensor/
- libraries/AP_AHRS/

AP_InertialSensor::simple_accel_cal() 逻辑可压缩为：

~~~~cpp
old_orientation = board_orientation;
old_offsets = accel_offsets;
old_scales = accel_scales;

board_orientation = ROTATION_NONE;
for (each requested calibration orientation) {
    average = average_of_50_accel_samples();
    rotated_gravity = rotate_gravity_for_orientation(average, requested_orientation);
    candidate_offsets = average - rotated_gravity;

    if (converged(candidate_offsets, 0.05f)) {
        save_candidate();
    }
}

if (all_orientations_succeeded) {
    save_offsets_scales_temperature();
    ahrs_trim = Vector3f(0, 0, 0);
} else {
    restore(old_orientation, old_offsets, old_scales);
}
~~~~

可提取的原则：

- 标定时暂时关闭已有 board rotation，避免重复应用；
- 多已知姿态区分 offset、scale 和重力方向；
- 每次取 50 点平均，收敛阈值约 0.05 m/s²，单姿态流程约 10 秒上限；
- 失败恢复旧参数，成功后清理 AHRS trim；
- 运动检测和数据有效性是前置条件。

当前固定提交中没有检索到名为 set_board_offset 的同名符号；不同分支把“板级偏移/安装外参”分散在 board orientation、accelerometer calibration 和 AHRS trim API 中，因此本报告不把该函数名当作当前版本事实。

参考：

- https://github.com/ArduPilot/ardupilot/blob/3b81960d3c6190b689e4be966b9dcc936d9c0b87/libraries/AP_InertialSensor/AP_InertialSensor.cpp
- https://github.com/ArduPilot/ardupilot/tree/3b81960d3c6190b689e4be966b9dcc936d9c0b87/libraries/AP_AHRS

### 6.3 Betaflight：小算力平台的健康门控

固定提交：Betaflight 087699dba08e3f518edef05770dea5ad8e054ac7。

关键片段：

~~~~c
static bool imuIsAccelerometerHealthy(void)
{
    return (0.9f < acc.accMagnitude) &&
           (acc.accMagnitude < 1.1f);
}
~~~~

随后 Mahony 更新使用四元数派生 DCM、快速反平方根和小角度误差。它先判断模长是否接近重力，再决定是否使用加速度反馈；这比把瞬时加速度当成绝对可靠的水平参考更稳健。

参考：

- https://github.com/betaflight/betaflight/blob/087699dba08e3f518edef05770dea5ad8e054ac7/src/main/flight/imu.c

### 6.4 ROS：静态 TF 外参与运行时滤波分离

imu_filter_madgwick 的范式是首帧用重力/磁场初始化，运行中使用时间戳计算 dt；自由落体和磁极附近奇异情形不更新无效观测，并发布固定坐标系到 IMU 坐标系的 TF。

robot_localization 的 IMU 回调明确假设用户提供 IMU 到 base_link 的变换：

~~~~cpp
// IMU 消息包含姿态、角速度和加速度，但共享一个 frame_id。
// 用户提供的 base_link <-> imu TF 用于变换到车体坐标。
poseCallback(..., base_link_frame_id_, base_link_frame_id_, true);
twistCallback(..., base_link_frame_id_);
accelerationCallback(..., base_link_frame_id_);
~~~~

它还通过 differential、relative、remove_gravitational_acceleration 和协方差拒绝参数处理不同传感器的动态可靠性。滤波器不猜机械安装角，静态外参由 TF 管理。

参考：

- https://github.com/CCNYRoboticsLab/imu_tools/blob/1bb40718c9bf54a8cd7f60c42fa88d88026e68b5/imu_filter_madgwick/src/imu_filter.cpp
- https://github.com/cra-ros-pkg/robot_localization/blob/7dfb6aa97b2082185d2fac3420888ae8474bfc1a/src/ros_filter.cpp

## 7. 针对 STM32H757 CM7 + BMI323 + LSM303 的架构适配

### 7.1 当前工程已确认的数据流

当前主要路径在 STM32H757/Middleware/Sensor/imu_manager.c：

1. 读取 LSM303 加速度/磁力计并形成 imu_raw_data_t 快照；
2. 通过 imu_boot_manager_is_ready() 门控启动阶段；
3. 调用 imu_calibration_apply()；
4. 调用 imu_filter_update()；
5. 调用 attitude_update()；
6. DualAHRS 分支输入独立 BMI323 样本以及最新 LSM303 加速度/磁力计，调用 dual_ahrs_update()。

当前工程还观察到：

- STM32H757/Middleware/Attitude/DualAHRS/dual_ahrs.c 中有 DUAL_AHRS_G 9.80665f；
- 校准头文件、部分传感器驱动和静态校准路径也存在 9.80665f；
- 振动采集已保存 double 类型 sum 和 sum_square，计算 RMS/variance，并有 90% 样本质量下限；
- 当前滤波器使用 LSM303 振动 RMS 选择 IIR alpha，但这不等价于双传感器 inverse-variance 融合；
- CM7 使用 -mcpu=cortex-m7 -mfpu=fpv5-d16 -mfloat-abi=hard；Debug 另有 -O0 -g3。

这些是源码结构事实，不代表 BMI323 已通过实体线路、设备日志或姿态验收。

### 7.2 最小侵入的切入点

建议在管理器的“校准后、滤波前”建立投影边界：

~~~~text
LSM303 raw
  -> LSM bias/scale/temp correction
  -> R_board_LSM
  -> R_level_LSM
  -> imu_filter_update
  -> attitude_update

BMI323 raw
  -> BMI bias/scale/temp correction
  -> R_board_BMI
  -> R_level_BMI
  -> BMI filter/decimator
  -> dual_ahrs_update

LSM accel/mag observation
  -> R_board_LSM * R_level_LSM
  -> gravity/magnetic residual
  -> DualAHRS observation weighting
~~~~

设计建议：

| 项目 | 建议 |
| --- | --- |
| 矩阵数量 | BMI323、LSM303 各自一个 3×3 float 矩阵，并保存来源、版本、校验和 |
| 计算时机 | 复用 IMU_READY 前静态窗口，静态数据通过后计算并冻结 |
| 更新频率 | 矩阵每样本应用；矩阵本身只在显式重新标定或启动成功时更新 |
| 旧接口兼容 | 保留现有 imu_raw_data_t 和 filter/AHRS API，在管理器中投影 |
| 权重位置 | DualAHRS 的加速度倾角、磁航向残差/增益 |
| 失败策略 | 保留上一次有效矩阵；无历史值则使用标称板级矩阵并上报 |

后续若允许增加状态字段，最少需要保存：

~~~~c
typedef struct {
    float r_level[3][3];
    float g_local_mps2;
    float accel_variance[3];
    float gyro_rms[3];
    uint32_t valid_samples;
    uint32_t total_samples;
    uint8_t valid;
    uint8_t fallback_reason;
} imu_leveling_state_t;
~~~~

这是接口设计草案，不是本次源码修改。

### 7.3 CM7 资源和时序

3×3 矩阵乘 3×1 向量需要 9 次乘法和 6 次加法。两个传感器每周期约 30 个标量浮点运算，另加矩阵读取。Cortex-M7 FPv5-D16 单精度硬件 FPU 可将其压到微秒量级甚至更低，但实际时间受内存位置、cache、内联、FMA、IRQ/DMA 抢占影响。

Debug -O0 的测量不能外推 Release WCET。建议在真实 Release 编译选项下使用 DWT_CYCCNT 测量 1000~10000 次，记录 cache、IRQ 屏蔽状态和编译器版本。

每个 float 矩阵占 36 字节；两颗传感器为 72 字节，加上质量状态、方差和时间戳通常小于几百字节。启动阶段的 double 累加只在低频标定窗口使用，不放进 100 Hz 热路径。

运行时投影使用 float 足够；静态窗口继续使用 double 累加可降低长窗口误差。方差计算优先用稳定形式或 Welford 算法，所有输入先做 isfinite 检查。

## 8. 异常保护与边界条件设计

### 8.1 倾角阈值、报警和回退

$$
\theta_{tilt}=\arccos(clamp(\hat{\mathbf a}\cdot\mathbf g_0,-1,1))
$$

建议策略：

| 条件 | 动作 |
| --- | --- |
| θ≤20° | 正常接受，记录角度和质量 |
| 20°<θ≤30° | 接受但 warning，要求更严格方差 |
| θ>30° | 报警并回退到历史有效矩阵/标称矩阵 |
| a_hat·g_0<0 | 倒置启动；无显式倒置模式时拒绝 |
| g_local 超出合理范围 | 拒绝并报告样本统计 |

30°不是数学奇点，而是产品安全策略：倾角越大，侧放/倒置误判风险越高。

### 8.2 开机运动误校准防呆

进入 LEVELING_COMMIT 必须同时满足：

1. 加速度模长在合理范围；
2. 三轴方差和模长方差低于阈值；
3. 陀螺 RMS 低，且最大绝对值没有持续超限；
4. 有效样本比例不低于现有 90% 下限；
5. 时间戳单调，采样间隔没有长间隙；
6. 没有饱和、SPI/I2C 错误或 stale；
7. 两个 IMU 在共享窗口中各自通过质量条件。

检测到冲击或运动时清空/重启静态窗口，而不是继续累加坏样本。

### 8.3 自由落体、饱和和非有限值

- |a| 远小于 g：自由落体，不能用于重力方向；
- 任一轴接近满量程：可能饱和，不能用于均值和方差；
- NaN/Inf：丢弃并增加计数；
- 时间戳重复或倒退：丢弃并标记时序故障；
- 单边传感器失效：另一侧进入降级姿态模式，但权重必须显式为单传感器。

### 8.4 回退优先级

1. 本次启动验证通过的新矩阵；
2. NVM 中带版本/CRC 且温度范围有效的上一次矩阵；
3. 编译期标称 R_board；
4. 单位阵作为最后安全模式，并上报“未调平”。

回退不能静默发生；SCBP/日志至少携带 reason、tilt、g_local、variance 和 sample ratio。

## 9. 推荐状态机与伪代码

### 9.1 启动状态机

~~~~text
BOOT -> SENSOR_INIT -> STATIC_COLLECT
  -> (norm/gyro/variance/timestamp gate)
  -> STATIC_COLLECT_RESET on invalid
  -> LEVELING_BUILD on valid window
  -> GRAVITY_COMMIT
  -> VIBRATION_COLLECT
  -> WEIGHT_COMMIT
  -> IMU_READY
  -> explicit recalibration returns to STATIC_COLLECT
~~~~

### 9.2 调平伪代码

~~~~c
bool build_leveling(const sample_set_t *set,
                    const mat3f_t *board_rotation,
                    level_result_t *out)
{
    if (set->valid_ratio < 0.90f ||
        set->gyro_rms > GYRO_STATIC_MAX ||
        set->accel_norm_std > ACC_NORM_STD_MAX) {
        return false;
    }

    vec3f mean = robust_mean(set->accel);
    float g = norm(mean);
    if (!isfinite(g) || g < G_MIN || g > G_MAX) {
        return false;
    }

    vec3f v = mean / g;
    float d = clamp(dot3(v, GRAVITY_TARGET), -1.0f, 1.0f);
    vec3f c = cross3(v, GRAVITY_TARGET);
    float s = norm(c);
    mat3f r_level;

    if (s < SINGULAR_EPS) {
        if (d > 0.0f) {
            r_level = mat3_identity();
        } else {
            if (!ALLOW_INVERTED_START) {
                return false;
            }
            vec3f axis = deterministic_orthogonal_axis(v);
            r_level = rodrigues(axis, PI);
        }
    } else {
        r_level = rodrigues(c / s, atan2f(s, d));
    }

    float tilt = acosf(d);
    if (tilt > TILT_MAX_RAD ||
        norm(r_level * v - GRAVITY_TARGET) > ROTATION_RESIDUAL_MAX ||
        !is_rotation_matrix(r_level)) {
        return false;
    }

    out->r_effective = r_level * (*board_rotation);
    out->g_local = g;
    out->tilt_rad = tilt;
    out->valid = true;
    return true;
}
~~~~

运行路径不应每次调用 atan2f/acosf；它们只在启动提交和诊断中使用，热路径只做矩阵乘法与门控。

### 9.3 动态权重伪代码

~~~~c
void update_attitude_observation_weights(const stats_t *bmi,
                                         const stats_t *lsm,
                                         weight_state_t *state)
{
    float vb = clamp(bmi->equivalent_attitude_var, VAR_FLOOR, VAR_CEIL);
    float vl = clamp(lsm->equivalent_attitude_var, VAR_FLOOR, VAR_CEIL);
    bool bmi_ok = bmi->online && !bmi->stale && bmi->valid_ratio >= 0.90f;
    bool lsm_ok = lsm->online && !lsm->stale && lsm->valid_ratio >= 0.90f;
    float ib = bmi_ok ? 1.0f / vb : 0.0f;
    float il = lsm_ok ? 1.0f / vl : 0.0f;
    float denom = ib + il;
    float target_bmi = denom > 0.0f ? ib / denom : 0.0f;
    float target_lsm = denom > 0.0f ? il / denom : 0.0f;
    state->bmi = slew_and_smooth(state->bmi, target_bmi);
    state->lsm = slew_and_smooth(state->lsm, target_lsm);
}
~~~~

输入必须是等效姿态观测方差，而非未经变换的加速度方差。BMI323 gyro 的积分状态和 LSM303 的重力/磁场校正保留各自时间戳。

## 10. 验证计划与验收边界

### 10.1 数学与主机回放

固定向量测试至少包括：

1. a_hat=[0,0,1] 必须得到单位阵；
2. a_hat=R_y(10°)^T[0,0,1] 调平误差小于阈值；
3. 反向向量走拒绝或显式倒置分支；
4. 旋转矩阵满足正交性和正行列式；
5. 纯 yaw + 10° 安装倾角时，投影后的 roll/pitch 窜动接近零；
6. g=9.78、9.80665、9.83 时归一化门限一致；
7. 方差相差 10 倍时权重符合 inverse-variance；
8. 方差为零、NaN、Inf、stale 时不产生 NaN 权重。

离线回放应输出原始/调平曲线、yaw 激励下交叉通道峰值、g_local 置信区间、等效方差/权重及回退原因。

### 10.2 固件静态和构建验证

本任务没有执行构建。后续实施应分层报告：

| 证据层 | 可证明内容 | 不能证明内容 |
| --- | --- | --- |
| 源码审查 | 数据流、状态机、矩阵顺序、常数来源 | 传感器实际输出 |
| 主机单元测试 | 数学和边界处理 | SPI/I2C 电气连接 |
| CM7 Release 构建 | 编译、链接、符号和尺寸 | 设备上电行为 |
| DWT/WCET | 指定构建和负载下时间 | 所有中断/温度/缓存组合 |
| 设备日志 | 状态机、质量、回退原因 | 仅凭日志不能证明真实水平度 |
| 实车/台架 | 窜轴、门控、温漂和融合效果 | 不能替代源码可追溯性 |

### 10.3 台架验收

在水平、已知 5°/10°/20° 垫片及倒置姿态分别启动，记录接受/拒绝状态、耗时、独立矩阵、g_local、方差、陀螺 RMS、纯 yaw/roll/pitch 交叉通道、权重变化、断开一颗传感器后的降级行为，以及矩阵版本和 CRC。

没有这些设备证据时，只能称为“数学/源码设计完成”，不能称为“自动调平已在车辆上生效”。

## 11. 风险、取舍和分阶段落地

### 11.1 主要风险

1. 旋转方向或乘法顺序反了：静态重力看似正确，但 yaw 交叉通道符号错误。
2. 把加速度 bias 吸收到调平矩阵：温度变化后水平角漂移。
3. 运行中追踪瞬时重力：车辆加速被误判为俯仰。
4. 把 LSM303 与 BMI323 原始量直接加权：观测模型不一致。
5. 方差来自不同带宽/采样率：数值小不一定代表物理上更可靠。
6. 只在 Debug 版本测时：不能代表 CM7 Release WCET。
7. 只检查“校准完成”标志：忽略有效样本比例、时间戳和具体失败原因。

### 11.2 推荐阶段

| 阶段 | 范围 | 退出条件 |
| --- | --- | --- |
| A | 主机端旋转、局部重力、奇异点测试 | 固定向量和 NaN 测试通过 |
| B | 只读接入启动窗口，记录不影响姿态的诊断 | 日志统计稳定，失败不阻塞 LSM303 |
| C | 启用 LSM303 调平投影，保留旧矩阵回退 | 台架静态和纯 yaw 验收 |
| D | 启用 BMI323 独立调平和 DualAHRS 观测权重 | 双传感器时间戳/状态一致 |
| E | Release DWT、温漂、振动和断链测试 | WCET、内存、降级和实车记录达标 |

各阶段保持 SCBP-V3、App BLE 和已有 30/26/14 字节兼容边界不变；本报告不授权任何协议或源码修改。

## 12. 参考资料与工程索引

### 12.1 当前工程

- STM32H757/Middleware/Sensor/imu_manager.c：LSM303 快照、启动门控、滤波/AHRS 和 DualAHRS 输入汇合点。
- STM32H757/Middleware/Filter/imu_filter.c：滤波更新和基于振动 RMS 的 IIR 参数选择。
- STM32H757/Middleware/Calibration/imu_boot_manager.c：双 IMU 启动状态、静态/振动窗口和 IMU_READY 门控。
- STM32H757/Middleware/Calibration/imu_vibration.c：sum、sum_square、RMS/variance 和样本质量统计。
- STM32H757/Middleware/Attitude/DualAHRS/dual_ahrs.c：当前 DualAHRS 及 DUAL_AHRS_G 固定常数。
- STM32H757/CM7/CMakeLists.txt：CM7 目标、BMI323 源文件选择和编译定义。

### 12.2 固定提交的上游实现

- PX4-Autopilot f40090acf4872747660ea366ad3b25e53d7b0ab4  
  https://github.com/PX4/PX4-Autopilot/commit/f40090acf4872747660ea366ad3b25e53d7b0ab4
- ArduPilot 3b81960d3c6190b689e4be966b9dcc936d9c0b87  
  https://github.com/ArduPilot/ardupilot/commit/3b81960d3c6190b689e4be966b9dcc936d9c0b87
- Betaflight 087699dba08e3f518edef05770dea5ad8e054ac7  
  https://github.com/betaflight/betaflight/commit/087699dba08e3f518edef05770dea5ad8e054ac7
- imu_filter_madgwick 1bb40718c9bf54a8cd7f60c42fa88d88026e68b5  
  https://github.com/CCNYRoboticsLab/imu_tools/commit/1bb40718c9bf54a8cd7f60c42fa88d88026e68b5
- robot_localization 7dfb6aa97b2082185d2fac3420888ae8474bfc1a  
  https://github.com/cra-ros-pkg/robot_localization/commit/7dfb6aa97b2082185d2fac3420888ae8474bfc1a

### 12.3 最终实施前审查清单

- [ ] 明确每颗器件的轴向、重力正负号和 R_board。
- [ ] 为同向、反向、30°边界、非有限值编写单元测试。
- [ ] 在同一时间基准下计算 RMS、方差和 stale。
- [ ] 将原始方差传播为等效 roll/pitch/yaw 观测方差。
- [ ] 为矩阵、g_local、权重和回退原因增加可观测字段。
- [ ] 使用 Release 构建和 DWT 测量 CM7 WCET。
- [ ] 完成静止、纯 yaw、加减速、振动、断传感器和倒置台架试验。
- [ ] 复核 dirty worktree，确保实施时没有误改不相关模块。

**报告状态：调研与设计建议完成；工程源码未修改；未执行构建、烧录或硬件运行验收。**
