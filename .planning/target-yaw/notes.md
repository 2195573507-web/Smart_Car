# Target Yaw 研究记录

- 活动 STM-S3 协议为 Common/SRP SRP v4，现有 `0x06` 是 16 字节 `v + yaw_rate`。
- `chassis_task.c` 当前为未跟踪但已纳入 CM7 CMake 的 10 ms 底盘任务，直接目标保存在任务模块内。
- `chassis_kinematics_compute` 约束左右轮速度绝对值不超过 1000 mm/s，轮距 193 mm。
- DualAHRS 内部将 BMI323 gyro 转为 rad/s，去 bias、Z 轴符号变换、leveling 和 LPF 后用于姿态积分；当前 public output 只有 yaw，没有 gyro。
- CM7 `s3_service_step` 在 200 ms S3 接收超时时调用 `chassis_task_set_velocity(0,0)` 和 `motor_board_force_stop()`。
- S3 `command_bridge` 已有 motion pending/in-flight、BLE disconnect、V2 session expiry 和 SRP BUS_OFF 清除路径。

## App 对接阶段（2026-08-26）

- iOS 与 macOS 客户端都依赖 `Shared/SmartCarAppCore`，但各自维护 `BLEManager`、`SmartCarViewModel` 和控制卡片。
- 当前两端 FFE1 写队列统一使用 `.withResponse`；S3 `s3_ble.c` 的 FFE1 属性也只声明 `WRITE`，与冻结的 `0x2E` Write Without Response 要求不一致。
- 现有 `MotionIntent` 只有底盘速度/角速度和四轮目标，没有目标航向字段；`0x2D` 发送在 BLEManager 的 `encodeMotionIntent` 中集中生成。
- FFE2 姿态已进入共享 `TelemetryStore.attitude.snapshot.data.yawDeg`，包含有效性和超时状态，可用于当前航向对齐。
- 当前 CM7 `chassis_task_set_heading_target(0, 0)` 与 S3 motion stop 识别仍需核对并修正，确保全零 `0x2E` 不会保留 heading active 或被改写成 wheel-speed stop。

## 2026-08-28 动态航向误差抑制

- 用户已确认采用 MotorBoard `[2 ms, 100 ms]` MSPD `dt` 保护、航向
  `alpha_max = 5.0 rad/s^2`、`w_ff` 默认 `0.0 rad/s` 和 `-Kd * gyro_z`。
- 当前工作树已有局部动态 MSPD 时间戳与航向 `w_prev`/前馈雏形；实施时须
  只补齐真实周期积分、异常有限性保护和验证，不覆盖同一文件中的其他脏改动。
- 本次实现已通过 PID 动态 `dt`/极端值 host smoke、SRP codec 回归，以及
  `STM32H757/CM7/build/Debug` CM7 clean build；构建产物内存约 FLASH 18.51%、
  RAM 61.47%。硬件 UART/BLE 和实车航向仍未验收。
