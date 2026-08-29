# P1 ROS2 建图计划进度

## 会话：2026-08-29

### 阶段 2b：第二次严谨性审查（进行中）

- **审查范围：** 复核轮速极性/新鲜度、S3 组件依赖、TCP 端口所有权、时钟与序号语义、容器挂载和 Humble 开源组件版本。
- **已确认：** `ENCODER_DIR_SIGN` 已在 STM32 内部应用；`0x210` 仍是无时间戳的缓存重发；现有 TCP `8765` transport 只能由一个 gateway 实例拥有；SCBP parser payload 只在回调期间有效。
- **开源复用修正：** `ros2_controllers` Humble 固定为 `eb4ca17d610eb4315f7241c0134de1bdfc5748ea`（2.54.0），优先仅链接其公开 `diff_drive_controller::Odometry`；`ros/diagnostics` Humble 固定为 `ros2-humble` commit `de779cfd3bff7975f158971c58bedf0581148f9a`（4.0.7）。
- **计划修订：** 将 `smartcar_state_bridge` 定义为 gateway 内部 decoder/library；增加 telemetry sink 依赖反转、通用 envelope parser、有界 ready 队列、`Common/SCBP_CAN` 只读挂载和 `slam_toolbox`/EKF/map saver 的明确边界。
- **尚未执行：** 未修改固件或 ROS2 业务代码，未构建新镜像，未连接真实 Wi-Fi/车辆；以上仍属于实施前设计审查证据。

### 阶段 1：现有系统与成熟组件审计

- **状态：** completed
- **执行的操作：**
  - 检查当前工作树和已有 `ROS2_WIN` 文件，保留所有用户改动。
  - 阅读 ROS2 bridge、S3 radar uplink、STM32 SCBP/轮速/IMU 源码和现有 ROS2 架构文档。
  - 浅克隆并检查 Humble `slam_toolbox`、`robot_localization` 和 `diff_drive_controller`。
  - 区分构建/离线测试证据与真实雷达、Wi-Fi、TF、车辆证据。
- **创建/修改的文件：**
  - `.planning/ros2-p1-mapping/task_plan.md`
  - `.planning/ros2-p1-mapping/findings.md`
  - `.planning/ros2-p1-mapping/progress.md`
  - `.planning/ros2-p1-mapping/P1_MAPPING_PLAN.md`

### 阶段 2：P1 接口与实施计划

- **状态：** completed; 等待用户批准后进入实现
- **当前结论：**
  - 自动建图直接采用 `slam_toolbox online_async`。
  - P1 只读：增加 telemetry/odom/TF/SLAM；不接 `/cmd_vel`。
  - 轮速先由 `0x210` 经 S3 独立 telemetry 上行到 ROS2；新 message type 数值待协议评审，不猜测。
  - 姿态先作为只读 topic，确认坐标系和协方差后再考虑 `sensor_msgs/Imu`/EKF。
  - 复核后收紧为单个 `smartcar_state_bridge`，复用已有 S3 transport；补充 Docker 官方 ROS2 依赖和 `command_bridge.c` 集成入口。
  - 根据用户环境澄清：Mac 不安装 Docker；Windows Docker Desktop Linux 容器是 P1 目标运行环境。

## 测试结果

| 检查 | 结果 |
| --- | --- |
| ROS2 host bridge 现有 colcon 测试 | 已有记录：25 tests、0 failures；不代表真实设备 |
| S3 live telemetry | 未实现、未验证 |
| `/odom`、TF、姿态 ROS topic | 当前不存在 |
| `slam_toolbox` 实车建图 | 未运行 |
| ROS2 -> 车辆控制 | P1 明确不启用 |

## 错误日志

| 时间 | 错误 | 处理 |
| --- | --- | --- |
| 2026-08-29 | 旧文档与当前 S3 uplink 源码不一致 | 以当前源码为准，在计划中标记文档同步任务 |

## 五问重启检查

| 问题 | 答案 |
| --- | --- |
| 我在哪里？ | 阶段 2 已完成：P1 接口与实施计划 |
| 我要去哪里？ | 用户批准后先实现只读 telemetry/odom，再接 URDF/TF/SLAM，最后做人工驾驶验收 |
| 目标是什么？ | 人工驾驶时由 ROS2 自动生成并保存可靠 2D 地图 |
| 我学到了什么？ | 见 `findings.md`：当前 `/scan` 已有，`/odom`/姿态/TF/SLAM 尚缺 |
| 我做了什么？ | 只读审计并创建本计划文件，未修改固件、协议或 ROS2 源码 |
