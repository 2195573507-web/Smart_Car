# Target Yaw 研究记录

- 活动 STM-S3 协议为 Common/SRP SRP v4，现有 `0x06` 是 16 字节 `v + yaw_rate`。
- `chassis_task.c` 当前为未跟踪但已纳入 CM7 CMake 的 10 ms 底盘任务，直接目标保存在任务模块内。
- `chassis_kinematics_compute` 约束左右轮速度绝对值不超过 1000 mm/s，轮距 193 mm。
- DualAHRS 内部将 BMI323 gyro 转为 rad/s，去 bias、Z 轴符号变换、leveling 和 LPF 后用于姿态积分；当前 public output 只有 yaw，没有 gyro。
- CM7 `s3_service_step` 在 200 ms S3 接收超时时调用 `chassis_task_set_velocity(0,0)` 和 `motor_board_force_stop()`。
- S3 `command_bridge` 已有 motion pending/in-flight、BLE disconnect、V2 session expiry 和 SRP BUS_OFF 清除路径。
