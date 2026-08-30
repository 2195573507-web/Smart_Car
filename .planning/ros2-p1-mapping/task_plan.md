# P1 ROS2 人工驾驶建图计划

## 目标

在保留 STM32 最终运动和安全控制权的前提下，让车辆由现有人工驾驶方式低速移动，ROS2 在 Windows Docker Desktop 的 Linux 容器中稳定接收 `/scan`、只读轮速里程计和可选姿态信息，运行成熟的 `slam_toolbox` 自动生成并保存 2D 地图。

## 范围边界

- 本任务是 **P1：人工驾驶 + ROS2 自动建图**。
- 允许 ROS2 读取雷达、轮速和姿态；不允许 ROS2 在本任务内发布或执行车辆运动命令。
- 现有 STM32 的姿态准入、链路超时、BUS_OFF、急停和 `motor_board_force_stop()` 继续拥有最终停止权。
- 现有工作树有大量用户改动；实施只触及计划明确列出的 S3 telemetry/uplink 文件和对应测试、文档，不覆盖、不回退、不整理其它业务代码。

## 当前阶段

阶段 3 已完成源码与主机侧实现：S3 只读 telemetry 上行、Windows 通用 envelope 接收和 telemetry 旁路计数均已落地；真实设备链路、`/odom`、TF 和 SLAM 仍未验收。

## 阶段清单

| 阶段 | 状态 | 结果 |
| --- | --- | --- |
| 1. 读取现有系统与参考实现 | completed | 当前 ROS2、S3、STM32、协议和成熟组件边界已确认 |
| 2. P1 接口与 TF/里程计设计 | completed | 已形成协议、话题、坐标系、参数和文件变更清单 |
| 2b. 第二次严谨性审查 | completed | 已修正轮速 freshness/FIFO、RF 极性、单 TCP owner、S3 sink 依赖、时钟/序号、Docker 挂载和 Humble 开源库边界 |
| 2c. 状态文档一致性同步 | completed | 已修正 ROS2_WIN 中“radar_uplink_protocol.c 不存在”等过时表述，并保留实验/未验收证据边界 |
| 3. 只读遥测与里程计 bridge | completed (source/host) | S3 telemetry sink、SRPv4 有界队列、统一 S3RD 序号、Windows type=2 接收和旁路计数；live `/odom` 仍受 freshness 合同硬门约束 |
| 4. URDF/TF/SLAM bringup | pending | `map -> odom -> base_link -> laser_frame` 和 `slam_toolbox` |
| 5. 分层验证与人工驾驶建图 | pending | rosbag、地图保存/加载、低速车辆验收 |
| 6. 交付审计 | pending | 变更清单、证据边界、未完成项和后续 P2 控车入口 |

## 退出条件

只有满足以下全部条件，才能声称“P1 建图完成”：

1. `/scan` 来自真实 S3/YDLIDAR 链路或可追溯的真实 rosbag，不仅是离线 fixture。
2. `odom -> base_link` 连续、时间戳单调、方向和尺度经直线/原地旋转实测。
3. `base_link -> laser_frame` 来自实物测量，TF 树无冲突。
4. 轮速源提供可验证的 `sample_tick/sample_seq/valid` 或等价 stale 语义；旧版 `0x14` 只能作为实验数据，不能单独作为完成证据。
5. Windows 端只有一个 TCP `8765` owner，raw radar 和 telemetry 共用同一套 envelope parser/reassembly。
6. `slam_toolbox` 在线运行时没有持续的 message-filter 丢弃、TF extrapolation 或扫描过期错误。
7. 人工驾驶低速绕行后 `/map` 可保存为 YAML/PGM，slam_toolbox posegraph 可序列化，并分别通过官方 map server 与 slam_toolbox localization/继续建图重载。
8. 断开 Wi-Fi、S3 重启、雷达无数据、STM32 链路超时时，系统只停止更新或降级，不产生任何 ROS2 控车输出。

## 关键问题的当前答案

| 用户问题 | 当前结论 |
| --- | --- |
| 能否自动建图 | 可以。推荐直接复用 `slam_toolbox` 的 `online_async`，输入真实 `/scan`、有新鲜度证明的 `/odom` 和完整 TF；ROS2 不需要从零实现 SLAM。 |
| 现有 ROS 能否控制车辆 | 当前不能，也不应在 P1 直接接通。现有 `ROS2_WIN` 只有雷达 bridge；没有 `/cmd_vel` 到 S3/STM32 的受控协议、租约、超时和人工接管机制。人工驾驶仍走现有 App/车辆链路。 |
| ROS 能否看到姿态 | 当前 ROS2 没有姿态 topic。STM32 已产生 `ATTITUDE(0x11)` 和 `IMU_TELEMETRY(0x10)`，S3 目前主要转发给 App BLE；P1 可增加只读 telemetry 上行和 ROS topic，但姿态不是 2D 建图的硬依赖。`0x10` 的 LSM303 magnetometer 与 BMI323 gyro 必须分开映射。 |

## 已知风险和否决项

- S3 `S3RD` 雷达上行协议仍属于实验性实现；协议冻结前不得猜测新的 telemetry message type 数值。
- 当前没有 STM32 `Middleware/Odometry` 实现；轮速状态存在，但 ROS2 里程计需要单独 bridge 和实测标定。
- 轮序固定为 `[M1=RR, M2=RF, M3=LR, M4=LF]`。`(+1,-1,+1,+1)` 是 STM32 内部 raw encoder 校正，不是 `0x14` wire 极性；ROS2 默认倍率为 `[+1,+1,+1,+1]`，最终车辆方向仍必须实车验证。
- `0x14` 只有四个 float 且由 STM32 周期性重发缓存，未补充源采样时间/序号/valid 前不得把收包时间当作测量时间。
- Windows TCP `8765` 只能由一个 gateway 进程监听；state bridge 不得再创建 socket。
- S3 telemetry 通过 `smartcar_service` sink 反转依赖，并在 parser callback 内复制入队；不能从 `main` 组件反向 include 到 service，也不能保存 parser payload 指针。
- SRP inner sequence 是所有消息共享的 8 位序号；不能按 telemetry 类型直接计算连续性。
- 轮径约 65 mm、轮距 193 mm 是现有记录值，必须在 P1-0 用量具复核后写入参数。
- 构建成功不等于 Wi-Fi、UART、TF、SLAM 或车辆通过；每一类证据单独记录。
- 工作树现在同时包含 `Common/SRP` 共享实现和 active `srp_*` UART2 service；当前 telemetry 旁路按 active SRPv4 解码，协议联合冻结前仍不得把源码/主机测试通过误认为真实两端已验收。

## 遇到的错误

| 错误 | 次数 | 处理 |
| --- | --- | --- |
| 旧文档声称 `radar_uplink_protocol.c` 不存在 | 1 | 以当前源码为准；已同步 ROS2_WIN 状态文档，保留实验协议和未验收边界，不回退源码。 |
| 旧审计把 `ros2_controllers` master commit 当作 Humble | 1 | 已更正为 Humble `eb4ca17d610eb4315f7241c0134de1bdfc5748ea`（2.54.0），并补充公开 `Odometry` 库的使用边界。 |
| `TcpChunkAssembler` 的 `max_ready_frames` 仅保存参数、未实际限流 | 1 | 纳入 P1-1/P1-2 的阻断修订：实现有界 ready 队列、丢弃策略、计数和压力测试后才能进入 live。 |
