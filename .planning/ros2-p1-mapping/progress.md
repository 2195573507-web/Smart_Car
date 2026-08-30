# P1 ROS2 建图计划进度

## 会话：2026-08-29

### 阶段 2b：第二次严谨性审查（已完成）

- **审查范围：** 复核轮速极性/新鲜度、S3 组件依赖、TCP 端口所有权、时钟与序号语义、容器挂载和 Humble 开源组件版本。
- **已确认：** `ENCODER_DIR_SIGN` 已在 STM32 内部应用；`0x14` 仍是无时间戳的缓存重发；现有 TCP `8765` transport 只能由一个 gateway 实例拥有；SRP parser payload 只在回调期间有效。
- **开源复用修正：** `ros2_controllers` Humble 固定为 `eb4ca17d610eb4315f7241c0134de1bdfc5748ea`（2.54.0），优先仅链接其公开 `diff_drive_controller::Odometry`；`ros/diagnostics` Humble 固定为 `ros2-humble` commit `de779cfd3bff7975f158971c58bedf0581148f9a`（4.0.7）。
- **计划修订：** 将 `smartcar_state_bridge` 定义为 gateway 内部 decoder/library；增加 telemetry sink 依赖反转、通用 envelope parser、有界 ready 队列、`Common/SRP` 只读挂载和 `slam_toolbox`/EKF/map saver 的明确边界。
- **尚未执行：** 未修改固件或 ROS2 业务代码，未构建新镜像，未连接真实 Wi-Fi/车辆；以上仍属于实施前设计审查证据。

### 阶段 1：现有系统与成熟组件审计

- **状态：** completed
- **执行的操作：**
  - 检查当前工作树和已有 `ROS2_WIN` 文件，保留所有用户改动。
  - 阅读 ROS2 bridge、S3 radar uplink、STM32 SRP/轮速/IMU 源码和现有 ROS2 架构文档。
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
  - 轮速先由 `0x14` 经 S3 独立 telemetry 上行到 ROS2；新 message type 数值待协议评审，不猜测。
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
| 2026-08-29 | 更新 findings 时补丁锚点使用了被截断的行 | 改为读取文件尾部并以完整末行作为锚点，未改动错误目标 |
| 2026-08-30 | 独立 telemetry smoke 初次链接命令把 `.o` 当作 C++ 源文件 | 改为先单独编译 stdin `main.o` 再链接对象文件，smoke 通过 |

## 五问重启检查

| 问题 | 答案 |
| --- | --- |
| 我在哪里？ | 阶段 2 已完成：P1 接口与实施计划 |
| 我要去哪里？ | 用户批准后先实现只读 telemetry/odom，再接 URDF/TF/SLAM，最后做人工驾驶验收 |
| 目标是什么？ | 人工驾驶时由 ROS2 自动生成并保存可靠 2D 地图 |
| 我学到了什么？ | 见 `findings.md`：当前 `/scan` 已有，`/odom`/姿态/TF/SLAM 尚缺 |
| 我做了什么？ | 只读审计并创建本计划文件，未修改固件、协议或 ROS2 源码 |

### 会话续作：2026-08-29（快速收口）

- **阶段 2b：completed。** 将 `0x14` freshness 设为实施硬门；轮速改为有界 FIFO，overflow/序号断裂/stale 后锁存 `odom_invalid`，禁止 latest-only 积分；确认 ROS 默认 wire 极性 `[+1,+1,+1,+1]`。
- **架构边界：** 明确 Windows `8765` 只有一个 gateway owner；`smartcar_state_bridge` 只能作为同进程 library/component；S3 通过 `smartcar_service_set_telemetry_sink(callback, context)` 反转依赖；通用 envelope parser 按 message type 分流。
- **时间与消息：** 分开 outer S3RD sequence、inner 全局 SRP sequence、DualAHRS sample sequence 和新增 wheel sample sequence；live/replay 分别使用主机时间和 `--clock`；姿态先做合法性、`wxyz -> xyzw` 重排和 stale 检查。
- **开源复用：** 固定 Humble `slam_toolbox`、`robot_state_publisher`、Nav2 map saver、`diagnostic_updater` 和 `diff_drive_controller::Odometry` 的复用边界；不启动完整 controller、`controller_manager` 或 `/cmd_vel`。
- **状态文档：** 同步 `ROS2_WIN/README.md`、`ROS2_WIN/docs/final-report.md`、`ROS2_WIN/docs/s3-gateway-protocol-TODO.md`，说明上行源码已存在但仍是实验实现，历史 live `/scan` 记录不等于当前端到端验收。
- **未执行：** 未修改固件/ROS2 业务代码，未重建 Docker 镜像，未重新连接真实 S3/车辆；阶段 3-6 仍待批准和硬件证据。

### 会话续作：2026-08-29（S3 telemetry 实施）

- **阶段 3：in_progress。** 用户已明确要求继续，开始实现 S3 侧只读 SRP telemetry sink、有界队列、通用 S3RD 编码和 uplink 调度。
- **冻结边界：** 不修改现有 SRP 消息布局、UART2/BLE/车辆控制语义；不接 `/cmd_vel`；新 S3RD telemetry 类型在协议评审前仅作为实验值，不宣称正式兼容。
- **验证目标：** 先完成 host tests、默认 ESP-IDF build 和 uplink-enabled build；真实刷写、Wi-Fi、STM32 capture 与车辆验收继续单列为未验证。

### 会话续作：2026-08-30（阶段 3 收口）

- **阶段 3：source/host completed。** `smartcar_service` 在 parser callback 内将完整 SRPv4 frame 复制到 S3 telemetry sink；S3 使用轮速 FIFO、姿态/两路 IMU latest-slot 和非阻塞互斥，继续保持雷达 UART1 与 STM UART2 控制链隔离。
- **协议/调度修订：** S3RD envelope 允许实验 telemetry type `2`；雷达与 telemetry 共用一个外层发送序号，雷达 UART 原始序号只用于雷达连续性检查，避免 Windows 全局 sequence tracker 产生伪造跳号。
- **Windows 修订：** `S3FrameExtractor` 按 message type 选择 raw YDLIDAR 或 telemetry payload policy；`TcpChunkAssembler` 真正执行 `max_ready_frames`，超限消费并计数丢弃；bridge 对 telemetry 只做诊断计数，不送入 YDLIDAR 解码器。
- **已验证：** `sh main/radar/tests/run_host_tests.sh`；`git diff --check`；ESP-IDF 5.5.4 uplink-enabled `idf.py build` + `idf.py size`；从 `sdkconfig.defaults` 生成的临时默认关闭配置 `idf.py build`；独立 `framing.cpp` C++17 警告编译。
- **输出证据：** 活动构建树 `ESPS3/build/smartcar_s3_gateway.elf/.bin`；启用 uplink 镜像约 1.22 MB、应用分区余量 83%；默认关闭临时镜像约 0.70 MB、余量 90%。
- **仍未验证：** 未刷写；未取得 STM32 `0x14/0x11/0x10` 真实 capture；未完成协议联合冻结/golden capture；未在 Windows Docker 上重建 ROS2 镜像或运行真实 TCP、`/odom`、TF、slam_toolbox 和车辆验收。

### 会话续作：2026-08-30（telemetry decoder）

- **实现：** 新增 `s3_ydlidar_bridge::TelemetryDecoder`，直接链接 `Common/SRP` 的 `srp_decode()`/wire helpers，解码 `0x14`、`0x11` 和 `0x10`，校验完整 frame、STM32 来源、S3/广播目的、`STREAM_DATA` flags、payload 长度、DualAHRS schema、IMU sensor id 和所有浮点有限性。
- **安全边界：** bridge 仅对 telemetry 做解码和 diagnostics 计数；legacy `0x14` 明确标记 `freshness_available=false`，没有发布 `/odom` 或 `odom -> base_link`。
- **构建边界：** CMake 增加 `Common/SRP` 静态 codec，Compose 以 `/ws/Common:ro` 挂载共享协议定义；没有复制协议结构，也没有新增 TCP listener。
- **测试：** 增加 `test_telemetry_decoder.cpp`，覆盖轮速、DualAHRS、两类 IMU、错误 source/flags、NaN 和 unsupported message；本机独立 C/C++ smoke test 通过，S3 host tests 与 `git diff --check` 通过。
- **环境限制：** 本机无 Docker、ROS Humble 或 colcon，无法执行 Windows Docker 镜像重建及 gtest/colcon；需在目标 Windows Docker 环境补跑并保存日志。
- **CM7 复核：** 迁移后的 active `srp_*` service 与共享 `Common/SRP` 已一致；此前 Debug 重配置曾因错误的 `srp_protocol_defs.h` 文档引用失败，现已统一使用 `srp_registry.h`/`srp_codec.c`。规范 `STM32H757/CM7/build/Debug` 构建通过。真实刷写格式仍待联合确认。
