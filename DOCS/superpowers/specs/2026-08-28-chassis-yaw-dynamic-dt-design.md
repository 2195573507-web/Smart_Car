# 底盘 YAW 动态航向误差抑制设计

## 范围

本次重构只覆盖 CM7 MotorBoard 速度闭环和底盘 Target Yaw 控制周期。目标是
让有效 `$MSPD` 帧的真实时间间隔驱动速度 Ramp、反馈低通和 PID 积分，并让
航向修正具备明确的角速度阻尼、目标角速度前馈和角加速度 slew-rate 限制。

## 不变量

- 电机顺序保持 `[M1:RR, M2:RF, M3:LR, M4:LF]`，包括编码器符号、PWM 参数
  顺序和 `WHEEL_TRIM_M1..M4`。
- 底盘轮距保持 `193.0 mm`，半轮距仍由现有运动学宏计算为 `96.5 mm`。
- `s3_service_is_synced()`、IMU/姿态有效性、MotorBoard 任务存在性、BUS_OFF
  清除和急停归零路径保持有效。
- 现有 Target Yaw SRP 12 字节格式不变；`w_ff` 通过 CM7 本地接口预留，默认
  `0.0 rad/s`。

## 控制设计

### MotorBoard 时间步进

在解析出有效 `MB_FRAME_MSPD` 后读取 `bsp_timer_get_us()`。以相邻有效帧
时间戳计算 `dt`，并将其限制在 `[0.002 s, 0.100 s]`。首帧、时钟回退、超时
或无效时钟只更新基准、刷新实际速度并重置 PID 历史，不执行本次 Ramp/PID
更新。正常帧把同一个 `dt` 传给 `Ramp_Update()` 和
`pid_controller_step()`。PID 反馈低通使用该 `dt` 计算指数衰减系数；其名义
周期只用于把既有 alpha 参数转换为等效时间常数，不再作为控制步长。

### 航向环

航向误差保持 `err = cur_yaw - target_yaw`。DualAHRS 的 Body-Z 角速度经过
BMI323 轴变换后，正角速度使用 `-Kd * gyro_z` 形成负反馈阻尼。控制律为：

```
w_corr_raw = w_ff + Kp * err + Ki * integral(err * dt) - Kd * gyro_z
```

原始角速度先受现有轮速余量和 `2.0 rad/s` 限幅，再使用
`alpha_max = 5.0 rad/s^2` 施加 `|delta_w| <= alpha_max * dt`。底盘失能、姿态
无效、航向未激活、急停或运动学拒绝时清零积分和 `w_prev`。

## 异常与并发

- 所有时间差、控制结果和 PID 内部输出在使用前后检查有限性；异常值回到零
  或安全基准，不把 NaN/Inf 送入运动学、PWM 或积分器。
- 时间戳使用已有 DWT 扩展单调时钟，跨 32 位 CYCCNT 溢出由 `bsp_timer` 统一
  处理。
- 共享目标、停止序列和前馈值继续通过现有 FreeRTOS 临界区保护；急停序列
  优先于当前控制周期输出。

## 验证

1. 源码审计确认轮序、轮距、`WHEEL_TRIM`、同步/姿态/BUS_OFF/急停门控未改。
2. 主机测试覆盖 PID 动态 `dt`、首帧/异常时间戳、低通有限性和角速度 slew。
3. 在 `STM32H757/CM7/build/Debug` 执行 CM7 clean build，并检查
   `Smart_Car_H757_CM7.elf`。
4. 构建和主机测试只证明源代码集成；UART/BLE 实时链路和实车航向性能仍需
   使用匹配固件和现场采集单独验收。
