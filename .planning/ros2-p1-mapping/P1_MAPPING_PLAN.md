# P1：人工驾驶 + ROS2 自动建图详细实施计划

## 1. 交付目标

最终操作方式：

1. 操作者使用现有 App/人工驾驶链路让车辆低速移动。
2. YDLIDAR 仍由 ESP32-S3 独占 UART1/GPIO44，S3 经 Wi-Fi 上行到 Windows ROS2。
3. 唯一的 ROS2 gateway 接收真实 `/scan` 和 STM32 只读遥测；在同一进程内生成 `/odom` 并发布完整 TF。
4. `slam_toolbox` 在线估计位姿并发布 `/map`，操作者在 Windows 上的 RViz2/容器显示端观察质量。
5. 用 `slam_toolbox` 的 `save_map`/`serialize_map` 保存可继续建图的 posegraph，并用官方 `nav2_map_server map_saver_cli` 另存标准 YAML/PGM；之后分别验证继续建图/localization 和静态 map server 加载。

本计划不把 ROS2 运动控制接入车辆。自动导航和 `/cmd_vel` 属于 P2，需另行设计安全协议和车辆验收。

## 1.1 运行环境边界

- Mac 只用于源码阅读、配置编辑和计划维护；P1 不要求 Mac 安装 Docker、ROS2 或连接车辆。
- 目标运行机是 Windows。按当前仓库的 `ROS2_WIN/docker/Dockerfile` 和 Compose 设计，ROS2 Humble 运行在 Windows Docker Desktop 的 Linux 容器中，容器内为 Ubuntu 22.04；S3 通过同一局域网连接 Windows 主机映射的 TCP 端口。
- 所有构建和运行命令在 Windows PowerShell 进入 `ROS2_WIN/docker` 后执行；WSL2 是 Docker Desktop 的后端条件，不是 Mac 的依赖。
- “原生 Windows ROS2（不使用 Linux 容器）”不属于当前实现目标。若必须原生运行，需要另立兼容性门：Winsock transport、Windows SDK backend、Humble 二进制包可用性、`slam_toolbox`/Nav2/TF 依赖和 RViz 图形链路逐项验证。

## 2. 现状与差距

| 层 | 现有代码 | P1 需要补齐 |
| --- | --- | --- |
| 雷达采集 | S3 `radar_uart` + 官方 YDLIDAR parser | 真实设备抓包和上行合同验收 |
| S3->ROS 雷达 | `s3_ydlidar_bridge` 发布 `/scan`，默认安全模式，实验 TCP | 冻结 S3 实际合同；真实 `/scan` 端到端验证 |
| 轮速 | STM32 已发 `0x210` 四轮实际速度 | S3 独立 telemetry 转发；ROS2 解包和 odom 节点 |
| 姿态 | STM32 已发 `0x201` DualAHRS/`0x207` IMU telemetry，当前面向 App BLE | 可选只读 ROS telemetry bridge，暂不作为 SLAM 必需输入 |
| TF | 当前只有临时 RViz/`laser_frame` 使用，无车体 URDF/连续 odom TF | URDF、实测外参、`robot_state_publisher`、`odom -> base_link` |
| SLAM | 未启动 | 复用 Humble `slam_toolbox` `online_async` |
| 控车 | ROS2 无 `/cmd_vel` 安全下行 | P1 保持断开；后续 P2 单独评审 |

## 3. 推荐的软件包结构

在已有 `ROS2_WIN/src/` 下新增或扩展。推荐先把 state bridge 合并为一个包，避免为每种 telemetry 再维护一套 transport：

```text
ROS2_WIN/src/
  s3_ydlidar_bridge/                 # 唯一 gateway；唯一 TCP :8765 owner + 通用 reassembly
  smartcar_state_bridge/             # gateway 内部 decoder/odom library，不独立监听 TCP
  smartcar_description/              # URDF/xacro + meshes + TF 参数
  smartcar_bringup/                  # P1 mapping launch/config/RViz
```

`smartcar_state_bridge` 内部仍保持 `TelemetryDecoder`、`WheelOdom`、`AttitudePublisher` 三个独立模块，但 P1 以静态库或 composable component 形式由唯一 gateway 装载。若后续需要独立部署，只能订阅 gateway 已发布的内部/ROS 消息；不能复制 `TcpChunkAssembler`、重新实现 S3 socket，或让第二个进程监听 `8765`。

### 3.2 TCP 与消息所有权

```text
ESP32-S3 TCP client
        |
        v
唯一 s3_ydlidar_bridge / smartcar_gateway_bridge
  :8765 listener -> bounded TCP assembler -> generic S3RD envelope parser
                                      |
                         +------------+-------------+
                         |                          |
                 RAW_YDLIDAR_FRAME             SCBP_TELEMETRY
                 official decoder              scbp_can_decode()
                         |                          |
                       /scan                 odom/attitude/diagnostics
```

- 一个进程、一个 listener、一个连接生命周期和一个重组缓冲；雷达与 telemetry 只在解析后的 dispatcher 层分流。
- `smartcar_state_bridge` 不声明 `TcpServerTransport`，不读取控制命令，也不发布 `/cmd_vel`。
- 现有可执行文件名可以暂时保持 `s3_ydlidar_bridge_node` 以减少迁移；文档和诊断名称应标明它是唯一 gateway。若改名为 `smartcar_gateway_bridge`，必须保留兼容的 launch 参数而不增加第二个节点。

## 3.1 开源组件复用清单

| 能力 | 直接复用 | 项目特有代码只保留 |
| --- | --- | --- |
| YDLIDAR 解码 | 当前已导入的官方 YDLIDAR SDK | S3 payload 适配和参数校验 |
| S3 TCP 重组 | 当前 `s3_ydlidar_bridge` 的 `TcpChunkAssembler`、`TcpServerTransport`，配合通用 envelope parser | 修复 ready 队列上限，再按 message type 分流；不再写第二套 socket |
| 2D SLAM | ROS2 Humble `slam_toolbox` `online_async_launch.py` 和官方参数模板 | 仅项目参数、frame 名称和 launch 组合 |
| TF/URDF | `robot_state_publisher`、`xacro`、`tf2_ros`、`tf2_tools` | 实测车体/雷达外参 |
| 地图保存 | `slam_toolbox` `save_map`/`serialize_map` 服务和 `nav2_map_server map_saver_cli` | 地图目录和验收脚本 |
| 记录回放 | `rosbag2` | 录包 topic 清单和证据元数据 |
| 诊断 | `diagnostic_updater`、`diagnostic_msgs` | S3/SCBP 计数器到诊断键的映射 |
| 状态融合 | `robot_localization` EKF/UKF（通过审查后） | 只提供正确 frame、时间和协方差 |
| 里程计积分 | 直接链接 Humble `diff_drive_controller::Odometry` 公开库（2.54.0） | 轮序/极性/新鲜度 gate 和 m/s 到每周期位移的适配；不启动 controller |

不建议把完整 `diff_drive_controller` 直接接入 P1：它要求 `ros2_control` 硬件接口、command/state interface 和控制命令。可以只链接其公开 `Odometry` 类来复用成熟积分实现；必须通过容器编译探针确认 Humble ABI，并确保没有 `controller_manager`、plugin 或 `/cmd_vel` 运行路径。

## 4. 阶段 P1-0：冻结设备、网络和标定数据

### 修改位置

仅新增文档、配置和测试记录；不改业务代码。建议记录到 `ROS2_WIN/docs/p1-mapping-evidence/`，不要把凭据提交 Git。

### 修改原因

当前最大风险不是 SLAM 算法，而是雷达型号/包模式、安装外参、轮速符号和时钟语义未被真实数据确认。

### 修改内容

- 记录 X3/X3 Pro 型号、固件、雷达是否带强度、UART 电平、供电和共地。
- 记录 S3 与 Windows 主机的 IP、Docker 端口映射和防火墙策略；P1 近场先用同一 LAN。
- 保存连续原始 YDLIDAR 包和对应 S3RD 包；同时保存一组坏 CRC、拆包、粘包、未知 type、序号跳变和断线重连样本。
- 用量具测量 `base_link -> laser_frame` 的 xyz 和 yaw/pitch/roll。
- 复核轮径、轮距；初始候选 65 mm/193 mm，未经复核不得写成最终标定值。
- 抬轮和地面低速前进、后退、原地旋转，记录 `[RR, RF, LR, LF]` 的实际符号。特别记录：STM32 内部 `ENCODER_DIR_SIGN={+1,-1,+1,+1}` 已经应用，ROS2 wire 默认倍率必须是 `[+1,+1,+1,+1]`，不得再次反相 RF。
- 记录 `0x210` 的源采样新鲜度：若仍只有四个 float，则把它标为实验输入；只有新增 `sample_tick/sample_seq/valid`（或经过评审的等价 stale 合同）后，才允许进入 live `/odom`/SLAM 验收。
- 记录 S3 入 FIFO 时间、发送时间、Windows 收包时间和 ROS header 时间；明确每次 S3/STM32 重启后的 epoch 与 32 位毫秒回绕处理。

### 潜在影响

若型号、角度方向、轮速符号或 TF 外参错误，SLAM 可能仍“有画面”但地图会镜像、弯曲或漂移；因此本阶段是硬门槛。

### 验证方法

- `ros2 topic hz /scan`、原始包计数和雷达频率一致；S3 FIFO backlog 超过阈值时旧帧被丢弃并在 diagnostics 中计数。
- 定距障碍物在 RViz 中出现在正确方向和距离。
- 直线 1 m 和原地 360° 旋转的 odom 误差分别记录。
- `ros2 run tf2_ros tf2_echo base_link laser_frame` 与量具数据对照。
- 断开 MotorBoard 反馈后验证 ROS2 不会把周期性重发的旧 `0x210` 当成新测量继续积分。

## 5. 阶段 P1-1：冻结 telemetry 上行合同

### 修改位置

设计评审涉及：

- `ESPS3/main/radar/radar_uplink_protocol.h/.c`
- `ESPS3/main/radar/radar_uplink.c`
- `ESPS3/main/radar/radar_frame_fifo.*`
- `ESPS3/components/smartcar_service/command_bridge.c`（当前 `0x210/0x201/0x207` 的 S3 接收和 App relay 集成点）
- `ESPS3/components/smartcar_service/include/smartcar_service.h` 或新增 service-owned telemetry sink 头文件
- `Common/SCBP_CAN/include/scbp_protocol_defs.h`（仅在明确批准新增定义时）
- `ROS2_WIN/src/smartcar_state_bridge/`

### 修改原因

当前 S3RD 只覆盖 raw YDLIDAR frame；ROS2 需要轮速和可选姿态。必须新增受控 telemetry type，同时保持雷达 raw type 和 STM32 UART2/SCBP 不变。`0x210` 的新鲜度缺口是实施前置阻断，而不是由 ROS 收包时间掩盖。

### 推荐设计

- S3 上行 envelope 复用现有 header 的身份、sequence、时间和外层 CRC；但先把当前只懂 YDLIDAR 的校验拆成通用 envelope parser 和按 type 的 payload policy。
- telemetry payload 携带完整、已验证的 SCBP-CAN encoded frame，而不是在 S3/ROS 两端重新定义 `wheel[4]`、姿态字段和多个版本。
- 在 `command_bridge.c` 的现有 relay 边界旁通过 `smartcar_service_set_telemetry_sink(callback, context)` 增加 telemetry enqueue；parser callback 只做固定大小复制、白名单检查和非阻塞入队，不能同步 Wi-Fi send。
- `scbp_parser` 的 `frame->payload` 是 parser 内部缓冲，回调返回后立即失效；队列 entry 必须拥有自己的字节存储。
- telemetry 的最大 payload 不能继续假设为 YDLIDAR 最大帧；按 `SCBP_CAN_MAX_FRAME_SIZE=267` 重新计算外层上限（至少 `26 + 267 + 2 = 295` 字节），并给静态缓冲、FIFO 深度、heap/stack 水位留出明确预算。
- ROS2 decoder 只接受白名单 `0x210`、`0x201`、`0x207`，验证 source/destination、payload length、SCBP flags、CRC、validity 和时间语义；未知 type 只计数丢弃。
- 雷达 raw frame 与 telemetry 使用不同 message type。outer sequence 按 `(device_id, stream_id, message_type)` 独立时，必须写入协议；inner SCBP sequence 仍是全链路共享的 8 位序号，不能按消息类型单独判连续。
- 明确定义丢包、重复、乱序、wrap、断线重连、S3/STM32 重启 epoch、最大 payload、S3 monotonic timestamp 与 ROS host time 的换算及 stale timeout。
- S3 发送雷达前按 `dequeue_age_ms` 丢弃过期帧；轮速样本必须使用按时间顺序消费的有界 FIFO，不能 latest-only；纯观察姿态可 latest-only。任何队列溢出、源序号断裂或 stale 都要锁存对应状态无效，不能让状态积压阻塞雷达或 UART2。
- 任何新 type 数值、magic、字段布局必须在协议评审后冻结，不从旧文档猜测。

### 5.1 `0x210` 新鲜度决策（实施硬门）

| 方案 | 兼容性 | 能否支撑 live `/odom` | 决策 |
| --- | --- | --- | --- |
| 新增版本化只读 wheel-status，带 `sample_tick/sample_seq/valid` 和源端 stale 规则 | 保留旧 `0x210` | 可以，推荐 | P1-1 首选 |
| 保持 16 字节 `0x210`，只在新 MSPD 到达时发送，并增加独立 stale diagnostics | 较高 | 仅在实测证明发送间隔等价于采样年龄后 | 需协议评审批准 |
| ROS 仅按 TCP 收包时间判定新鲜 | 无固件改动 | 不可以 | 明确禁止 |

### 潜在影响

新增 telemetry 会占用 S3 Wi-Fi、RAM、CPU 和 ROS 网络带宽；错误的队列策略可能阻塞雷达或 STM32 控制链。轮速使用固定容量 FIFO（记录 overflow/drop 并触发 `odom_invalid`），姿态等纯观察流才可 latest-only；禁止可靠无限队列，并记录每类丢弃数。

### 验证方法

- Host golden tests：通用 envelope 完整帧、拆分/粘包、长度错误、CRC 错误、未知 type、身份错误、每流序号跳变、inner 全局序号交错和重连。
- S3 host tests：编码/解码、最大 267 字节 SCBP frame、轮速 FIFO 满/溢出、姿态 latest-only、过期雷达丢弃、Wi-Fi 断线期间 RAM/栈水位。
- 真实 capture：同一 SCBP frame 在 STM32、S3 和 ROS2 三端字节一致；断开 MotorBoard 反馈后 `valid/age` 按合同变化。

## 6. 阶段 P1-2：实现 gateway 内的 `smartcar_state_bridge` 模块

### 修改位置

建议文件（作为 gateway 内部库/component，不单独监听 TCP）：

```text
ROS2_WIN/src/smartcar_state_bridge/package.xml
ROS2_WIN/src/smartcar_state_bridge/CMakeLists.txt
ROS2_WIN/src/smartcar_state_bridge/include/.../telemetry_decoder.hpp
ROS2_WIN/src/smartcar_state_bridge/include/.../wheel_odom.hpp
ROS2_WIN/src/smartcar_state_bridge/src/state_node.cpp
ROS2_WIN/src/smartcar_state_bridge/test/
```

需要同步修改已有 gateway/protocol 的具体位置：

```text
ROS2_WIN/src/s3_ydlidar_bridge/include/s3_ydlidar_bridge/framing.hpp
ROS2_WIN/src/s3_ydlidar_bridge/src/framing.cpp
ROS2_WIN/src/s3_ydlidar_bridge/include/s3_ydlidar_bridge/transport.hpp
ROS2_WIN/src/s3_ydlidar_bridge/src/transport.cpp
```

修改目标是让一个 TCP owner 的外层重组、身份校验和 message dispatch 同时服务 raw radar 与 SCBP telemetry；雷达解码、telemetry 解码和 ROS publisher 仍保持独立。

### 修改内容

- 复用并泛化 `s3_ydlidar_bridge` 的 transport/reassembly；先将 envelope/parser/dispatcher 提取为同一 ROS package 的 `s3_gateway_protocol` library，禁止复制 socket 和拆包代码。
- `TcpChunkAssembler` 必须执行 `max_ready_frames` 上限；明确满队列丢弃 oldest（雷达）或 latest-only（状态）的策略，并发布 dropped-ready 计数。
- 解析后优先只发布标准消息：`nav_msgs/msg/Odometry`、`diagnostic_msgs/msg/DiagnosticArray`；原始四轮值放入 diagnostics，避免为内部数组创建不必要的自定义 ROS message。
- 可选发布 `sensor_msgs/msg/JointState` 作为只读车轮观测（速度换算为 rad/s，明确关节名和无 position 语义）。
- 姿态首版使用 `geometry_msgs/msg/QuaternionStamped` 供观察；确认 REP-103、协方差、时间基准和重力补偿后再发布 `/imu/data_raw`。
- `0x207` 的 LSM303 vector 映射为 `sensor_msgs/msg/MagneticField`，BMI323 vector 映射为 gyro；只有单位、轴向、协方差和时间确认后才发布 `sensor_msgs/msg/Imu`。
- 每个状态流使用 `diagnostic_updater::Updater`：最后一帧年龄、有效/丢弃/CRC/长度/序号计数、source timestamp/epoch、S3 与主机时钟差；不复制 diagnostics 源码。
- stale 或身份不匹配时停止发布新状态；不发布零速度伪造运动已经停止，除非协议明确给出安全状态。
- 轮速未具备源 `sample_tick/sample_seq/valid`（或批准的等价合同）时，节点只能发布实验诊断，不得发布作为 SLAM 输入的 live `/odom`。

### 潜在影响

ROS2 executor 中解析大 payload、日志和 TF 发布会消耗 CPU；需要限制日志频率、预分配/复用缓冲并避免阻塞 callback。

### 验证方法

- `colcon build --symlink-install --packages-select s3_ydlidar_bridge`
- `colcon test --packages-select s3_ydlidar_bridge`
- 在同一容器中对 `/ws/Common/SCBP_CAN` 的 `scbp_can_decode()` 做最大帧编译/链接探针；确认没有第二个 `bind(8765)`。
- `ros2 topic echo --qos-durability volatile /odom`
- 录制并回放 telemetry bag，检查时间单调和重复包被丢弃。

## 7. 阶段 P1-3：实现 state bridge 内的轮速里程计模块

### 修改位置

建议新增：

```text
ROS2_WIN/src/smartcar_state_bridge/
  include/.../wheel_kinematics.hpp
  src/wheel_kinematics.cpp
  src/odom_node.cpp
  config/odom.yaml
  test/test_wheel_kinematics.cpp
```

### 修改内容

- 输入带有效源采样语义的 wheel status；在新鲜度合同冻结前只允许离线 fixture，不接目标速度 `0x110`。
- 固定数组索引并在代码/测试中写出 `RR, RF, LR, LF`，避免以后按 `FL/FR/RL/RR` 重排。
- STM32 已将 `ENCODER_DIR_SIGN={+1,-1,+1,+1}` 应用于 raw feedback；ROS2 wire 默认倍率固定为 `[+1,+1,+1,+1]`。最终“前进为正”仍由 P1-0 实测确认，不能静默再次反相 RF。
- 对右侧和左侧取平均，计算 `v` 与 `omega`；单位从 mm/s 转 m/s。
- 直接链接 Humble `diff_drive_controller::Odometry`（2.54.0，commit `eb4ca17d610eb4315f7241c0134de1bdfc5748ea`），先在 wrapper 中验证 `dt`、finite、freshness 和重连重锚，再调用其 exact integration。Humble `updateFromVelocity()` 参数实际是每周期左右轮位移，不是 m/s；传入 `v_left*dt`、`v_right*dt`，并用编译探针锁定 API。
- `dt<=0`、`dt` 过小、超过 stale timeout、NaN/Inf、source sequence 回退时跳过积分并进入 diagnostics；不要让 `robot_localization` 在 stale 后继续外推。
- 发布 `nav_msgs/msg/Odometry`，设置合理但保守的 twist covariance；未标定前不要给出虚假的高精度 pose covariance。
- 独占发布 `odom -> base_link` TF。若后续启用 `robot_localization`，由 EKF 接管 odom TF，原始 odom 节点关闭 TF 发布。
- 以参数控制 `publish_tf`、`wheel_radius_m`/`wheel_diameter_m`、`track_width_m`、`timeout_ms`、`wheel_speed_sign[4]`、`frame_id`、`child_frame_id`；`wheel_diameter_m` 不参与已是 mm/s 的线速度换算，只用于可选 JointState rad/s。
- P1 默认不启动 EKF；若批准启用 `robot_localization`，显式 `use_control:false`、`two_d_mode:true`、`world_frame: odom`，并增加 stale supervisor/重置策略。

### 潜在影响

轮速打滑、wire 极性误读、轮距误差、缓存旧值和时钟跳变会使 odom 漂移；SLAM 可能依赖 scan matching 修正，但不会修复错误 TF 或反向运动学。EKF 若在 stale 后继续预测还会把旧速度扩散到地图，故不能默认启用。

### 验证方法

- 单元测试：直行、后退、左/右旋、零速、时间倒退、丢包、源 valid=false、sample sequence 跳变/wrap、SCBP inner 全局序号交错和 outer per-stream sequence。
- 静态台架：四轮抬起，验证每个输入通道对左右平均值的贡献。
- 地面测试：1 m 直线、原地 90°/360°、正反向，记录 odom 与量具/地标误差。
- `ros2 topic hz /odom`、`ros2 run tf2_ros tf2_echo odom base_link`。
- 断开反馈或 Wi-Fi 后确认无新鲜 wheel sample 时不继续积分；确认未启动 `controller_manager`，且 `ros2 topic info /cmd_vel` 没有 P1 publisher。

## 8. 阶段 P1-4：ROS 依赖、URDF、静态 TF 和 bringup

### 修改位置

建议新增：

```text
ROS2_WIN/src/smartcar_description/urdf/smart_car.urdf.xacro
ROS2_WIN/src/smartcar_description/config/sensors.yaml
ROS2_WIN/src/smartcar_bringup/launch/description.launch.py
ROS2_WIN/src/smartcar_bringup/launch/p1_mapping.launch.py
ROS2_WIN/src/smartcar_bringup/config/p1_mapping.yaml
ROS2_WIN/src/smartcar_bringup/rviz/p1_mapping.rviz
```

同时修改 `ROS2_WIN/docker/Dockerfile` 的 apt 依赖，优先使用 ROS2 官方二进制包，不把成熟包 vendor 进仓库：

```text
ros-humble-slam-toolbox
ros-humble-robot-state-publisher
ros-humble-xacro
ros-humble-tf2-tools
ros-humble-nav2-map-server
ros-humble-robot-localization
ros-humble-diagnostic-updater
ros-humble-diff-drive-controller
```

`robot-localization`、`diagnostic-updater` 和 `diff-drive-controller` 可作为运行/编译依赖；首版不启用 EKF 或完整 controller，但保留公开 `Odometry` 库的可复现编译环境。不要启动 `ros2_control`、`controller_manager`、controller plugin 或任何 `/cmd_vel` 下行节点。构建后记录 `docker image inspect` 的镜像 digest 及 `apt-cache policy` 的实际包版本。

Compose 还必须增加仓库共享协议的只读挂载，例如：

```yaml
- ../../Common/SCBP_CAN:/ws/Common/SCBP_CAN:ro
```

容器内 CMake 通过该路径编译/链接 `scbp_parser.c`、`scbp_crc.c`、`scbp_wire.c`（或一个已安装的共享 package），禁止复制一份 `scbp_protocol_defs.h`。地图、bag 和 evidence 使用明确的 named volume 或 Windows 主机目录，避免写入源码树。

### 修改内容

- 最小 TF 树：`base_link -> laser_frame`；若发布姿态，再增加 `base_link -> imu_link`。
- `robot_state_publisher` 只发布 URDF 中的静态/关节 TF；不要同时用 `static_transform_publisher` 发布同一对 frame。
- 雷达安装方向按 REP-103 和实测写入 xacro；不要复制资料包里未经测量的 `laser` 外参。
- 配置统一 frame 名：`map`、`odom`、`base_link`、`laser_frame`、`imu_link`。
- launch 启动顺序：唯一 gateway（含 state bridge）、`robot_state_publisher`、官方 `slam_toolbox` async node、RViz；默认不启动 Nav2 `map_server`、controller 或 `robot_localization`。
- `robot_state_publisher` 通过官方 xacro/`robot_description` 入口发布固定关节 `/tf_static`；只有实际活动关节才订阅 `/joint_states`，而仅有 velocity 的观测不能替代 position。
- 项目 slam 参数必须显式覆盖 `use_sim_time: false`、`base_frame: base_link`、`odom_frame: odom`、`map_frame: map`、`scan_topic: /scan`；不能依赖官方模板的 `base_footprint`/`true` 默认值。

### 潜在影响

TF 重复发布、frame_id 拼写差异或时间戳不一致会导致 SLAM 丢弃扫描；安装 pitch/roll 未测量时 2D 投影可能失真。

### 验证方法

```bash
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo base_link laser_frame
ros2 run tf2_ros tf2_echo odom base_link
ros2 topic echo /tf_static --qos-durability transient_local
```

要求 TF 树只出现一条 `map -> odom -> base_link -> laser_frame` 路径，且无 authority 冲突。

Windows Docker 运行 live gateway 时必须使用 `docker compose run --service-ports`（或 `docker compose up`），否则 `8765` 不会暴露到主机；RViz 只在同一 Linux 容器域内作为已验证的显示进程运行，WSLg/VcXsrv 的图形连通性另列证据。

## 9. 阶段 P1-5：复用 `slam_toolbox` 自动建图

### 复用方式

- 直接依赖 ROS2 Humble 的 `slam_toolbox`，优先 `online_async`；不复制其 scan matcher 或 pose graph。
- 最小输入：`/scan`、`odom -> base_link` TF、`base_link -> laser_frame` TF。
- 优先调用官方 `slam_toolbox/launch/online_async_launch.py`，只通过 `slam_params_file` 注入项目配置；初始配置从官方 `mapper_params_online_async.yaml` 逐项审查，不直接照搬机器人 frame、分辨率和扫描频率。
- 建图期间操作者用 RViz2 的 `/map`、`/scan` 和 TF 观察，不启用 Nav2 controller。

### 关键参数审查

| 参数 | P1 策略 |
| --- | --- |
| `odom_frame`/`map_frame`/`base_frame` | 固定为 `odom`/`map`/`base_link` |
| `scan_topic` | `/scan`，与 bridge 参数一致 |
| `use_sim_time` | 明确为 `false`；P1 不使用 `/clock` |
| `mode` | mapping |
| `resolution` | 先用 0.05 m，依据雷达噪声和场地调整 |
| `transform_publish_period` | 由 slam_toolbox 发布 `map -> odom`；不能由 odom bridge 重复发布 |
| `minimum_travel_distance`/`minimum_travel_heading` | 结合 X3 扫描频率、车辆最低可控速度和场地调参 |
| `scan_buffer_size`/`scan_queue_size` | 有界；出现积压时先降速或丢弃过期扫描 |
| `use_map_saver`/服务 | 优先使用官方 `slam_toolbox/save_map`、`serialize_map`；`nav2_map_server map_saver_cli` 作为独立备份，不另写地图导出器 |

### 潜在影响

没有可靠 odom 或 TF 时，SLAM 可能退化为错误匹配；扫描频率、Wi-Fi 延迟和车辆速度过高会造成 scan queue 堵塞。异步模式优先保证实时性，但仍需监控丢帧和处理延迟。

### 验证方法

```bash
ros2 launch smartcar_bringup p1_mapping.launch.py use_rviz:=true
ros2 topic hz /scan
ros2 topic hz /odom
ros2 topic echo /map --once
ros2 run tf2_tools view_frames
```

先用 rosbag replay 验证，再做低速人工驾驶；地图优先通过官方 `slam_toolbox/save_map`/`serialize_map` 服务保存。需要独立 YAML/PGM 备份时使用 `nav2_map_server` 的 `map_saver_cli`，固定 `/map`、格式和持久化路径：

```bash
ros2 run nav2_map_server map_saver_cli -t /map -f /ws/maps/p1_site_YYYYMMDD \
  --fmt pgm --mode trinary
```

保存前确认机器人停止、最后一圈扫描已处理、地图无明显重影；建图期间不要启动 `nav2_map_server map_server` 发布第二个 `/map`。随后在单独的加载阶段用保存的 YAML/PGM 启动官方 `map_server`，验证 topic、分辨率、原点和坐标系一致。官方 map-saver launch 的 `use_sim_time` 默认不能原样用于真实车辆，项目 launch 必须显式传 `false`。

## 10. 阶段 P1-6：分层验证矩阵

| 层级 | 输入 | 必须看到 | 不能据此声称 |
| --- | --- | --- | --- |
| H0 静态/单元 | golden YDLIDAR/telemetry bytes | 通用 envelope、SCBP decoder、序号、CRC、单位和 bounded queue 测试通过 | 真实无线或车辆可用 |
| H1 离线 ROS | rosbag `/scan` + synthetic wheel telemetry（含 source age/valid） | `/scan`、`/odom`、TF、`/map` 正常 | S3 现场链路通过 |
| H2 真实雷达 | S3 与 ROS2 同 LAN | 唯一 gateway 的实时 `/scan`、序号/年龄 diagnostics、过期 FIFO 丢弃 | 里程计或 SLAM 通过 |
| H3 真实遥测 | STM32 wheel-status（含 freshness）-> S3 -> 唯一 gateway | `/odom` 方向、频率、时间单调；RF 不二次反相；反馈中断时停止积分 | 地图质量通过 |
| H4 TF/时钟 | `/scan` + `/odom` + URDF + 三时钟记录 | 只有 `map -> odom -> base_link -> laser_frame`，无 message-filter/TF extrapolation 持续错误 | 车辆安全自动驾驶 |
| H5 传感器闭环 | `/scan` + `/odom` + TF rosbag replay | `slam_toolbox` 地图稳定，官方 save/load 通过 | 真实现场链路通过 |
| H6 低速人工驾驶 | 人工 `/App` 控制，物理急停在场 | 地图闭环、保存、重载；断 Wi-Fi/S3/雷达/反馈只降级 | ROS2 能控车 |

每层都记录：固件/ELF hash、ROS 镜像 digest、Git commit、apt 包版本、配置、命令、原始日志、原始 capture、bag、地图文件和结论。任何一层未通过都不得把下一层结果写成“完成”。

## 11. P1 不做的控车工作，以及未来 P2 入口

当前 ROS2 **不能控制车辆**，原因是没有经过审查的 `/cmd_vel` 下行链路。未来 P2 必须另立设计，至少包括：

- `geometry_msgs/Twist` 到车体速度的单位和上限；
- ROS2->S3 连接认证、命令 sequence、lease/控制权租约；
- 命令频率、超时归零、断线归零和人工接管优先级；
- S3 仲裁与限速，STM32 最终姿态/链路/急停门；
- 执行反馈（目标速度、实际速度、状态、故障）和故障注入；
- 物理急停、台架、封闭场地、观察者和最高速度门槛。

P1 的 launch 和代码应确保没有 publisher 连接到 `/cmd_vel`，不启动 `controller_manager`，也不把 `/scan` 或 ROS DDS 直接送进 STM32 UART2。可用 `ros2 topic info /cmd_vel -v` 和进程清单作为自动化否定检查。

## 12. 交付清单

- `smartcar_state_bridge`、`smartcar_description`、`smartcar_bringup` 的源码和测试。
- P1 mapping launch、参数、RViz 配置和地图保存/加载命令。
- S3 telemetry 协议评审记录和 golden capture；雷达 S3RD 版本/兼容性记录；唯一 TCP owner 和 sink API 的依赖图。
- 轮径、轮距、轮速极性、雷达外参和时间基准标定记录。
- rosbag、`/diagnostics`、TF tree、地图 YAML/PGM 和验证报告。
- Dockerfile/Compose 的 ROS Humble 官方包清单、`Common/SCBP_CAN` 只读挂载和镜像/apt 版本记录。
- 明确列出未完成的真实硬件证据和 P2 控车审批项。

## 13. 当前结论

最稳妥的实施顺序是：先冻结并验证唯一 gateway 的雷达/telemetry 合同和源新鲜度，再补只读 odom，随后补静态 TF 和 `slam_toolbox`；姿态作为可选观察数据，EKF 经过 stale/时间审查后再决定，控车完全后置。这样最大限度复用成熟 ROS2 组件（官方 YDLIDAR decoder、`diff_drive_controller::Odometry`、`robot_state_publisher`、`slam_toolbox`、Nav2 map saver、diagnostic_updater），新增代码只承担项目特有的协议适配、源状态 gate、轮序映射和硬件外参，不从零实现 SLAM 或底盘控制器。
