# 当前小车底盘航向控制与运动执行审计

状态：`CONFIRMED（当前源码静态提取）`。本记录没有修改源码；未把源码、文档或日志推断为整车验收。

## 1. 航向闭环控制（HEADING_CTRL）

### 实现位置和入口

| 项目 | 当前实现 |
| --- | --- |
| 接收入口 | `STM32H757/Middleware/Communication/Services/s3_service.c:511-540`，处理 `SRP_MSG_ID_CHASSIS_HEADING_CMD` (`0x17`)；payload 是 `v_mm_s`, `target_yaw_deg`, `flags`，要求 `flags==0` |
| 控制任务 | `STM32H757/Application/Chassis/chassis_task.c:189-401` |
| 姿态输入 | `dual_ahrs_get_heading_state()`，返回 Primary yaw 和 `primary_gyro_z_rad_s`；`STM32H757/Middleware/Attitude/DualAHRS/dual_ahrs.c:944-955` |
| 任务周期 | `CHASSIS_TASK_PERIOD_MS=10`，`vTaskDelayUntil()`；FreeRTOS tick 为 1000 Hz，因此名义 100 Hz (`chassis_task.c:19-22,400`) |
| 运行门控 | S3 已同步、IMU boot ready、`g_attitude_is_ready`、MotorBoard task 存在；门控撤销时清目标、清积分并输出零 (`chassis_task.c:65-71,245-261`) |

### 当前参数

| 参数 | 当前值 | 代码位置 |
| --- | ---: | --- |
| `Kp` | `0.28` | `chassis_task.c:25-28` |
| `Ki` | `0.085` | 同上 |
| `Kd` | `0.006` | 同上 |
| 积分限幅 | `+/-8.0 deg*s` | `chassis_task.c:29,289-299` |
| 积分窗口 | `abs(err) < 15 deg` 且 `abs(v)>0.001 mm/s` | `chassis_task.c:289-298` |
| 修正硬限幅 | `+/-2.0 rad/s` | `chassis_task.c:31,313-321` |
| 轮速余量限幅 | `max_w=(1000-abs(v))/(193/2)`，再与 `2.0` 取小 | `chassis_task.c:308-321` |

### 计算公式和 D 项定义

当前误差为最短角差：

```text
current_yaw_deg = current_yaw_rad * 57.295779513
err_deg = wrap180(current_yaw_deg - target_yaw_deg)
I_deg_s <- clamp(I_deg_s + err_deg * 0.010, -8.0, +8.0)
w_corr_rad_s = 0.28 * err_deg + 0.085 * I_deg_s + 0.006 * gyro_z_rad_s
```

因此 `Kd` 项使用的是 `dual_ahrs_get_heading_state()` 提供的处理后、偏置校正后的车体 Z 轴角速度（单位 `rad/s`），不是 `(err[k]-err[k-1])/dt` 的角度误差差分。DualAHRS 对 BMI323 gyro 先去偏，再做车体 Z 极性修正和 80 Hz 低通；相关代码为 `dual_ahrs.c:800-808,847-853`。这里名称虽为 `Kd`，从量纲和实现上更接近角速度阻尼项。

正误差按当前注释定义为逆时针/左修正；经底盘运动学后为右侧加速、左侧减速。姿态无效会清除目标并输出零 (`chassis_task.c:265-279`)。

## 2. 底盘速度分配与轮速解算

### `CHASSIS_SPEED_CMD` / `CHASSIS_HEADING_CMD`

普通差速命令在 `s3_service.c:477-508` 解出 `v` 和 `w`，航向命令在 `:511-540` 解出 `v` 和 `target_yaw`，后者由 10 ms Chassis task 生成 `w_corr`。核心运动学在 `STM32H757/Application/Chassis/chassis_kinematics.c:6-32`：

```text
half_track = 193.0 / 2 = 96.5 mm
left  = v - w * 96.5
right = v + w * 96.5

RR = right
RF = right
LR = left
LF = left
```

所有输入和四个输出都要求 finite，任一轮绝对值超过 `1000 mm/s` 即拒绝。数组顺序固定为 `RR, RF, LR, LF` (`chassis_kinematics.h:10-23`)。当前 MotorBoard task 的源代码进一步声明 `M1=RR, M2=RF, M3=LR, M4=LF` (`motor_board_task.c:60-65`)。

### 补偿逻辑

- Chassis 层没有按左/右侧设置的独立前馈或死区补偿。
- Heading 模式有 `CHASSIS_MIN_SAFE_WHEEL_SPEED_MM_S=80`：当 `abs(v)>80` 且一侧低于 80 时，将同样的有符号增量加到两侧，保持左右差值 (`chassis_task.c:73-114,325-331`)。这是低速可控性保护，不是制造偏差标定。
- MotorBoard 层有每轮输出 trim：`M1=1.08`，`M2/M3/M4=1.00` (`wheel_control_params.h:36-41`；应用于 `motor_board_task.c:551-558`)。这是当前唯一显式的不平衡前馈/通道比例补偿。
- 轮速控制器还包含统一的速度前馈和库仑摩擦补偿：`Kv=1.40 PWM/(mm/s)`，摩擦峰值 `260 PWM`，在目标速度 `80 mm/s` 内线性过渡 (`wheel_control_params.h:24-29`, `pid_controller.c:143-152`)。

## 3. 加减速规划与速度平滑

### Chassis 层

`chassis_ramp_linear_velocity()` 是线性加速度限幅，不是梯形/S 曲线轨迹器：

```text
max_delta = 400.0 * dt_s
delta = target_v - s_ramped_v
s_ramped_v += sign(delta) * min(abs(delta), max_delta)
```

位置：`chassis_task.c:45-63`。`dt_s` 使用相邻任务唤醒的实际 tick 差，正常约 10 ms，因此名义速度步进约 `4 mm/s`。急停、门控撤销和姿态失效会把 ramp 状态清零。

### MotorBoard 层

每个轮子另有 `Ramp_Profile_t`，初始化为 `WHEEL_RAMP_MAX_ACCEL=800 mm/s^2` (`motor_board_task.c:801-808`)。`Ramp_Update()` 每次处理有效 `$MSPD` 时按固定 `MB_PID_DT_SECONDS=0.05 s` 更新 (`motor_board_task.c:21-30,521-555`; `pid_controller.c:21-45`)。因此每个反馈事件的目标变化上限名义为 `40 mm/s`。这是轮速目标平滑，不能等同为底盘角加速度限制。

### 角加速度

当前源码没有 `angular_accel`、角速度 ramp 或角速度 slew-rate 参数。普通 `CHASSIS_SPEED_CMD` 的 `w` 直接进入运动学；Heading 模式的 `w_corr` 每 10 ms 重算，但仅受最大修正和轮速余量约束。故“当前角加速度参数”是：`未配置/未发现显式限幅`。

其他相关平滑参数：反馈一阶低通 `alpha=0.35`，目标小于 `30 mm/s` 的非零值在 MotorBoard PID 前截断为零，误差死区 `6 mm/s` (`wheel_control_params.h:15-22`)。

## 4. MotorBoard 通信与 PWM 下发链路

### 调用链

```text
S3 SRP CHASSIS_* command
  -> chassis_task_set_velocity()/heading target
  -> chassis_kinematics_compute()
  -> motor_board_set_target_wheel_speeds()
  -> wait for incoming $MSPD:M1,M2,M3,M4#
  -> motor_board_update_pid()
  -> pid_controller_step() for 4 wheels
  -> per-wheel trim and +/-2500 clamp
  -> MB_Protocol_SendPwm()
  -> MB_Protocol_SendFour("pwm", ...)
  -> "$pwm:m1,m2,m3,m4#"
  -> MB_Transport_Send() TX ring
  -> USART6 TXE interrupt writes TDR
```

关键位置：`motor_board_task.c:440-461,521-579`；`motor_board_protocol.c:360-377,531-539`；`motor_board_transport_uart.c:161-187,231-265`。

### 周期、传输方式和配置

| 项目 | 当前实现 |
| --- | --- |
| MotorBoard task 调度 | `vTaskDelay(1 ms)` 轮询 (`motor_board_task.c:21-24,810-835`) |
| PID/PWM 更新触发 | 不是固定 1 ms 定时器；仅在 `MB_Protocol_Poll()` 解析到 `$MSPD` 帧时执行 (`:812-816,521-560`) |
| TX 方式 | 512 B 环形 TX buffer + USART6 TXE/TXFNF 中断；ISR 写 `TDR` (`motor_board_transport_uart.h:13-37`; `.c:231-265`) |
| DMA/阻塞 | 当前 MotorBoard TX 未使用 DMA，也没有 `HAL_UART_Transmit*()` 阻塞发送 |
| UART | USART6，PC6 TX / PC7 RX，115200 8N1，无硬件流控 (`CM7/Core/Src/main.c:741-773`; `stm32h7xx_hal_msp.c:354-374`) |
| 启动配置序列 | `$pwm:0,0,0,0#`；`$mtype:1#`、`$mline:11#`、`$mphase:30#`、`$wdiameter:65#`；随后 `$upload:0,1,1#` (`motor_board_task.c:681-775`) |
| 输出限幅 | `WHEEL_PID_MAX_OUT=2500`，trim 后再次 clamp；目标轮速上限 `1000 mm/s` (`wheel_control_params.h:43-48`) |

### 当前闭环归属判断

当前运行代码发送的是 `$pwm`，不是 `$spd`。`MB_Protocol_SendSpeed()` 只在 `motor_board_protocol.c:536-539` 提供了封装，整个工程没有找到运行时调用。因此当前实际设计是：电机板回传 `$MSPD`，CM7 在 `pid_controller.c` 执行轮速 PI + 速度/摩擦前馈，再以直接 PWM 下发。

需要注意，`pid_controller_t` 虽保存 `kd`、`previous_error` 等字段，但 `pid_controller_step_with_feedforward()` 的公式只使用 `kp*error + integral + feedforward`，没有微分计算 (`pid_controller.c:128-155`)；当前轮速 `Kd=0.00` 只是进一步掩盖了这一实现事实。

官方板卡资料说明 `$spd` 才是板上速度闭环、`$pwm` 是直接 PWM；这只能证明板卡能力和协议含义，不能证明当前板卡固件/硬件配置已启用自带速度 PID。按当前 Smart_Car 发送路径判断，板卡自带速度闭环未被本运行路径使用。

## 风险与验证边界

1. `M1..M4` 的当前项目映射 `RR,RF,LR,LF` 与资料指南中记录的厂商机械映射 `M1=LF,M2=LR,M3=RF,M4=RR` 不一致；必须以实际接线、编码器符号和轮子离地测试为准，不能仅靠数组名判定物理方向。
2. `MB_PID_DT_SECONDS=50 ms` 与板卡 MTEP 资料所称 10 ms 数据周期不是同一概念；当前源码没有对 MSPD 到达周期做测量或动态 dt，实测反馈周期应纳入调参验证。
3. 本审计没有重新构建、烧录或抓取 USART6 波形；因此只确认源码链路和参数，不能宣称 UART、电机板反馈、PWM 波形、轮速闭环或整车航向性能已验收。

建议的最小验证顺序：

```text
1. 静态检查并构建 STM32H757/CM7/build/Debug。
2. 轮子离地，逻辑分析仪确认 USART6 115200 8N1 和 $pwm:...# 字节流。
3. 记录 $MSPD 到达时间戳，核对实际 PID 更新频率与 50 ms 假设。
4. 单轮确认 M1..M4 方向、RF 编码器反号和 trim 生效情况。
5. 数值回放验证 err 正负 -> RR/RF 与 LR/LF 的差速方向，再进行低速带载航向测试。
```
