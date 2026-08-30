# P1 ROS2 建图审计发现

## 1. 确认事实

### 1.1 ROS2 主机侧

- 工作区为 `ROS2_WIN/`，目标运行机为 Windows；当前仓库提供 Docker Desktop Linux 容器方案，Mac 不属于运行依赖。
- [`bridge_node.cpp`](/Users/zhiqin/Projects/Smart_Car/ROS2_WIN/src/s3_ydlidar_bridge/src/bridge_node.cpp) 创建 `/scan` 和 `/diagnostics`，支持 `unconfigured`、`replay`、实验性 `tcp` 三种 transport；当前没有 `/odom`、TF、姿态或 `/cmd_vel`。
- [`scan_mapper.cpp`](/Users/zhiqin/Projects/Smart_Car/ROS2_WIN/src/s3_ydlidar_bridge/src/scan_mapper.cpp) 负责官方 YDLIDAR node 到 `sensor_msgs/msg/LaserScan` 的距离/角度映射和零位包分圈。
- 当前 `frame_id` 默认是 `laser_frame`，QoS 使用 `rclcpp::SensorDataQoS()`。
- 已有 Docker/colcon 证据：25 tests、0 failures；这只证明 host-side bridge 的构建和离线测试。仓库还保留过 Windows TCP bridge 的历史 live `/scan` 记录，但当前 full-revolution 版本与 S3 固件链路尚未重新端到端验证，不能把历史记录当作当前完成证据。

### 1.2 S3 雷达侧

- [`radar_uplink_protocol.c`](/Users/zhiqin/Projects/Smart_Car/ESPS3/main/radar/radar_uplink_protocol.c) 当前定义实验性 `S3RD`：magic、version、raw-frame message type、flags、device/stream ID、sequence、timestamp、payload length、CRC16-Modbus。
- [`radar_uplink.c`](/Users/zhiqin/Projects/Smart_Car/ESPS3/main/radar/radar_uplink.c) 负责 Wi-Fi/TCP 上行；`SMARTCAR_RADAR_UPLINK_ENABLED` 默认关闭。
- 当前上行内容是已校验的原始 YDLIDAR 帧，不包含 STM32 轮速或姿态遥测。
- [`radar_frame_fifo.c`](/Users/zhiqin/Projects/Smart_Car/ESPS3/main/radar/radar_frame_fifo.c) 提供有界 FIFO 和丢弃统计，可作为遥测通道的资源隔离参考，但不能让雷达拥塞影响 UART2 控制链。

### 1.3 STM32 状态和安全边界

- [`srp_registry.h`](/Users/zhiqin/Projects/Smart_Car/Common/SRP/include/srp_registry.h) 定义：
  - `SRP_MSG_ID_WHEEL_SPEED_STATUS = 0x14`，payload 16 字节，4 个 little-endian float。
  - `SRP_MSG_ID_ATTITUDE = 0x11`，payload 80 字节，schema 2 的 DualAHRS。
  - `SRP_MSG_ID_IMU_TELEMETRY = 0x10`，每个传感器 30 字节。
- [`motor_board_task.c`](/Users/zhiqin/Projects/Smart_Car/STM32H757/Middleware/MotorBoard/motor_board_task.c) 的 `motor_board_send_wheel_status()` 周期性发送实际轮速；`motor_board_get_actual_wheel_speeds()` 提供当前值。
- 当前轮序是 `[RR, RF, LR, LF]`；`RF` 的 raw encoder 校正在 STM32 内部完成，`0x14` 携带的是校正后的值，ROS2 不能再静默把 RF 设为负。
- [`imu_runtime.c`](/Users/zhiqin/Projects/Smart_Car/STM32H757/Application/RTOS/imu_runtime.c) 周期性打包并发送 DualAHRS 和 IMU telemetry。
- [`s3_service.c`](/Users/zhiqin/Projects/Smart_Car/STM32H757/Middleware/Communication/Services/s3_service.c) 是 STM32-S3 SRP 边界；不得把雷达原始包塞进这条控制/状态协议。
- STM32 仍保留最终运动安全：姿态 gate、链路 timeout、BUS_OFF 和 `motor_board_force_stop()`。

### 1.4 成熟组件参考

外部仓库已浅克隆到 `/tmp/smartcar-ros-refs.UasknS/`，只作参考：

| 组件 | 参考 commit | P1 用法 |
| --- | --- | --- |
| `slam_toolbox` Humble | `51a99767b3e2ed4076ae5763ff14b69343ffd884` | 直接使用 `online_async`、map save/load 和标准 TF 接口 |
| `robot_localization` Humble | `8696ee5a9e4f959fcaae37835dcf2ed12ead581b` | P1 先保留为可选融合层，确认协方差/时间后再启用 EKF |
| `ros2_controllers/diff_drive_controller` Humble | `eb4ca17d610eb4315f7241c0134de1bdfc5748ea`（2.54.0） | 优先直接链接公开的 `diff_drive_controller::Odometry` 库；不启动 controller/plugin |

## 2. 推荐数据流

```text
YDLIDAR -> S3 UART1/GPIO44 -> S3 radar_uplink -> ROS2 s3_ydlidar_bridge -> /scan
STM32 0x14 wheel status -> S3 telemetry uplink -> smartcar_state_bridge -> /odom
STM32 0x11/0x10 attitude (optional read-only) -> smartcar_state_bridge -> /attitude or /imu/data_raw
URDF + robot_state_publisher -> base_link -> laser_frame (+ imu_link)
/scan + odom -> slam_toolbox online_async -> /map + map -> odom
```

禁止的数据流：

```text
ROS2 /cmd_vel -> 车辆
ROS2 /scan -> STM32
```

## 3. 协议设计结论

推荐新增一个**独立、受控的 telemetry 上行类型**，其 payload 传输完整且已验证的 SRPv4 frame（type、priority、flags、8 位 sequence、length 和 payload）；SRP 外层本身没有 timestamp。ROS2 端复用 `Common/SRP` 的 wire 定义解析 `0x14/0x11/0x10`，时间戳分别按具体 payload 或 S3 外层语义处理。这样可以避免在 S3 和 ROS2 两端重新发明轮速/姿态字段。

但当前不能写入具体新 type 数值或声称兼容：必须先完成协议评审并提供至少一组真实 golden capture、错误 CRC、长度错误、序号跳变和断线重连样本。雷达 `RAW_YDLIDAR_FRAME` 与 telemetry 必须保持不同 message type；不修改已有雷达 type，不把雷达 payload 混进 STM32 SRP 控制队列。

## 4. 里程计数学

1. 接收带源采样年龄/序号/有效位的轮速状态；现有 `0x14` 只有四个 float，不能单独证明数据新鲜，因此在补充合同前只能做实验回放。
2. 解包四个实际轮速，单位按现有代码视为 mm/s，固定轮序为 `[RR, RF, LR, LF]`。
3. `ENCODER_DIR_SIGN={+1,-1,+1,+1}` 是 STM32 对 MotorBoard raw feedback 的内部校正，校正后的值已进入 `0x14`；ROS2 默认 wire 极性必须为 `[+1,+1,+1,+1]`，不得再次把 RF 设为负。
4. 应用经台架确认的车辆极性表，将每个轮速归一到“车辆向前为正”。不能只依据变量名推断。
5. 计算：`v_right = mean(RR, RF)`，`v_left = mean(LR, LF)`；`v = (v_right + v_left)/2`；`omega = (v_right - v_left)/track_width`。
6. 使用 `track_width_m`、`wheel_diameter_m` 参数，默认候选值为 0.193 m 和 0.065 m，但在实测前标记为未确认；`0x14` 已是 mm/s，轮径不参与线速度换算，只用于可选关节 rad/s。
7. 优先直接链接 Humble `diff_drive_controller::Odometry` 的公开库，适配层先将 m/s 乘以有效 `dt` 变成该版本 `updateFromVelocity()` 实际需要的每周期轮位移；处理时间倒退、`dt` 过小、非有限值和状态过期后再调用库。
8. 发布 `nav_msgs/msg/Odometry`：`header.frame_id=odom`、`child_frame_id=base_link`、twist 为车体速度；默认只发布 `odom -> base_link` TF，避免与 EKF/SLAM 产生第二个发布者。

## 5. 姿态结论

P1 可先发布只读 `geometry_msgs/msg/QuaternionStamped`，但必须先检查 schema、primary-valid/fault/stale、有限值、单位四元数，并把 wire `w,x,y,z` 重排为 ROS `x,y,z,w`；不满足时只发 diagnostics。`0x10` 中 LSM303 的 vector 是 magnetometer，BMI323 的 vector 是 gyro，不能把两者统一塞进一个 `/imu/data_raw`。只有确认 ROS REP-103 坐标轴、协方差、时间基准和重力补偿后，才分别映射到 `sensor_msgs/Imu`/`sensor_msgs/MagneticField` 并考虑 `robot_localization`。

P1 建图不依赖姿态 topic；2D SLAM 先依赖 `/scan`、`odom -> base_link` 和静态雷达 TF。姿态若用于 TF，必须明确只由一个节点发布，不能与 odom yaw 或 SLAM 的 `map -> odom` 冲突。

## 6. 参考资料边界

- 官方 YDLIDAR SDK 和 ROS2 driver 已用于当前 `/scan` 解析；driver 的串口/DTR 控制代码不能作为 S3 runtime。
- `slam_toolbox`、`robot_localization`、`diff_drive_controller` 均为成熟开源组件，计划只复用标准接口和算法，不复制其无关控制或硬件层。
- 所有网页、仓库和旧文档内容均视为参考数据；本项目当前源码优先，特别是 S3 uplink 文件已经存在时，旧的“不存在”说明必须标记为过时。

## 7. 必须在实测中确认的项目

| 项目 | 确认方法 | 未确认时的处理 |
| --- | --- | --- |
| 轮径、轮距 | 量具 + 直线/原地旋转实测 | 只允许离线 odom 测试，不运行 SLAM |
| 轮速正负 | 抬轮低速和地面前进/后退抓包 | 不发布可用于 SLAM 的 `/odom` |
| S3/主机时钟 | 发送时间与 ROS 接收时间对照、NTP/SNTP 记录 | 初版使用主机接收时间并禁用严格融合 |
| 雷达安装外参 | 量具、水平仪、静态 TF 对照 | 不进入 slam_toolbox |
| 雷达角度方向 | 定距障碍物 + RViz + 原始包 | 只保存 bag，不保存地图 |
| telemetry 上行合同 | 协议评审 + golden captures | 不实现 live telemetry parser |

## 8. 复核后的严谨性修订

- 当前 `ROS2_WIN/docker/Dockerfile` 只安装 bridge、RViz 和 rosbag2 相关依赖；要运行 P1，必须显式加入 `ros-humble-slam-toolbox`、`ros-humble-robot-state-publisher`、`ros-humble-xacro`、`ros-humble-tf2-tools`、`ros-humble-nav2-map-server`、`ros-humble-robot-localization`、`ros-humble-diagnostic-updater` 和（若采用公开里程计库）`ros-humble-diff-drive-controller`，并记录镜像 digest 与 apt 包版本。
- Windows 端只能有一个 TCP `8765` owner：保留/扩展现有 `s3_ydlidar_bridge` 为唯一 gateway 进程；`smartcar_state_bridge` 只能作为同进程 library/component 或订阅 gateway 内部输出，不能再创建 listener。
- telemetry 不应创建第二套 TCP server/reassembly。应泛化已有 `s3_ydlidar_bridge` transport，或把它提取成共享 library，再由同一 gateway 分发 raw radar 与 SRP telemetry。
- S3 的实际接入点是 `ESPS3/components/smartcar_service/command_bridge.c` 的现有 SRP relay，而不是只修改 `main/radar`；通过 `smartcar_service_set_telemetry_sink(callback, context)` 做依赖反转，parser callback 内完成有界复制/入队，不能做 Wi-Fi 阻塞操作或保存 `frame->payload` 指针。
- telemetry 若复用 S3RD 外层，容量必须按共享定义 `SRP_MAX_FRAME_SIZE` 重新预算；当前值为 `8 + 500 + 4 = 512` 字节，不能沿用仅适用于 YDLIDAR raw frame 的上限。
- ROS2 侧应修改已有 `s3_ydlidar_bridge` 的 `framing.*`/`transport.*` 为通用 envelope parser + message dispatcher，避免新增 state bridge 时重复 TCP reassembly；`TcpChunkAssembler` 必须真正执行 `max_ready_frames` 上限并报告丢弃。
- ROS 侧优先使用标准 `nav_msgs/Odometry`、`sensor_msgs/JointState`、`geometry_msgs/QuaternionStamped`、`sensor_msgs/Imu` 和 `diagnostic_msgs`；只有标准消息无法表达的稳定跨节点合同才新增自定义 message。
- `diff_drive_controller` 完整控制器适合已有 `ros2_control` 硬件接口和控制命令的底盘；当前只读轮速不满足其运行前提。P1 可只链接 Humble `diff_drive_controller::Odometry`（commit `eb4ca17d610eb4315f7241c0134de1bdfc5748ea`，2.54.0），绝不启动 `controller_manager`、plugin 或 `/cmd_vel`。
- `slam_toolbox` 官方 `online_async_launch.py` 已提供标准启动入口；建图完成优先调用其 `save_map`/`serialize_map`，`nav2_map_server map_saver_cli` 作为独立备份/自动化入口。建图过程中不要同时启动 `map_server` 发布第二个 `/map`。
- `robot_localization` 的官方 EKF 在 sensor timeout 后仍会 predict；P1 不默认启用。若经评审启用，必须显式 `use_control: false`、`two_d_mode: true`、`world_frame: odom`，并由 stale supervisor 停止/重锚其输出。

## 9. Windows 运行边界复核

- 当前 Compose 的基础镜像是 `ros:humble-ros-base-jammy`，并把源码挂载到容器；因此推荐定义为“Windows Docker Desktop 上运行 ROS2 Linux 容器”，而不是声称已经有原生 Windows ROS2 支持。
- Mac 不需要安装 Docker，也不参与 P1 运行验收。Windows 端负责 Docker Desktop Linux engine、端口 `8765`、RViz/WSLg 或 X11 显示和真实网络抓包。
- 若用户要求 ROS2 节点直接运行在 Windows 原生 Python/C++ 环境，必须重新验证 POSIX socket、YDLIDAR Unix backend、Humble 的 `slam_toolbox`/Nav2 二进制包，当前计划不能直接覆盖该场景。

## 10. Humble slam_toolbox 启动参数实证

- 参考仓库 `slam_toolbox` Humble commit `51a99767b3e2ed4076ae5763ff14b69343ffd884` 的 `launch/online_async_launch.py` 将 `use_sim_time` 默认声明为 `true`，并把 `slam_params_file` 默认指向官方 `mapper_params_online_async.yaml`。
- 同一官方配置模板的默认 `base_frame` 是 `base_footprint`，`odom_frame=odom`、`map_frame=map`、`scan_topic=/scan`、`use_map_saver=true`。
- 因此真实车辆的项目参数必须显式覆盖 `use_sim_time: false` 和 `base_frame: base_link`，不能只调用官方 launch 而依赖默认值；这条覆盖应加入 launch 验证和配置单元检查。

## 11. robot_localization 与 diff_drive_controller 边界实证

- Humble `robot_localization` 官方 `params/ekf.yaml` 示例包含 `use_control: true`、`stamped_control: false` 和 `control_timeout`，即会把控制命令纳入预测；该示例不能原样用于 P1。其 sensor timeout 仍会继续 predict，故 P1 不默认启用；若启用 EKF，应显式设置 `use_control: false`、`two_d_mode: true`、`world_frame: odom`，增加 stale supervisor，并仅在真实姿态/协方差/时间基准审查后选择输入变量。
- 同一示例要求明确 `map_frame`、`odom_frame`、`base_link_frame`、`world_frame`，并说明连续轮速应以 `world_frame=odom` 融合；计划中的 TF 所有权必须遵守这一 REP-105 约束。
- `ros2_controllers/diff_drive_controller` Humble 参考实现同时声明左右轮的 command/state interface、hardware interface 和 controller update 生命周期；当前项目没有 ros2_control hardware interface，也没有 P1 控制命令，因此不能加载完整控制器。但其公开 `diff_drive_controller::Odometry` 库可被 state bridge 直接链接，输入单位和 stale/重连 gate 由项目 wrapper 负责。

## 12. GitHub 分支与可复现引用

- `ros-navigation/navigation2` Humble 分支：`3c3db59d6969d8ecee8e68468693d006397f4a0c`。
- `ros/robot_state_publisher` Humble 分支：`8b5af4f31e0f754c7c262eb487ccd3956cb788cc`。
- `ros-controls/ros2_control` Humble 分支：`401158e2ab17bc856ad7a0dc8f8890d0f5b1af89`。
- `ros-drivers/transport_drivers` Humble 分支：`d3f510ce1b4be12967064d8251ebcac530921a04`。
- `ros/diagnostics` Humble 分支为 `ros2-humble`，commit `de779cfd3bff7975f158971c58bedf0581148f9a`（4.0.7）；P1 使用 ROS Humble 二进制包并记录 apt 包版本，不复制源码。
- 引用仓库只用于核对 API/参数和测试思路；不把整套导航栈或控制器源码复制进 `ROS2_WIN`，避免许可证、依赖和升级负担。

## 13. transport_drivers 与 robot_state_publisher 实证

- `ros-drivers/transport_drivers` Humble commit `d3f510ce1b4be12967064d8251ebcac530921a04` 的仓库结构只有 `udp_driver`、`serial_driver` 和 `io_context`，没有 TCP driver 或通用 TCP 粘包/重组层。当前项目的 TCP 服务器不能直接替换为该仓库；可参考其 ASIO `IoContext`、异步回调和生命周期测试，但引入整个仓库会增加依赖。
- `robot_state_publisher` Humble commit `8b5af4f31e0f754c7c262eb487ccd3956cb788cc` 的官方 launch 示例先用 `xacro.process_file()` 或 `launch.substitutions.Command('xacro ...')` 生成 URDF 字符串，再以 `robot_description` 参数启动 `robot_state_publisher`。P1 应沿用该标准入口，固定关节由 RSP 发布 `/tf_static`，移动关节才依赖 `/joint_states`。

## 14. Navigation2 map_saver_cli 实证

- Navigation2 Humble commit `3c3db59d6969d8ecee8e68468693d006397f4a0c` 的 `nav2_map_server/CMakeLists.txt` 明确构建并安装 `map_saver_cli`，安装路径为 `lib/nav2_map_server/map_saver_cli`；因此计划中的 `ros2 run nav2_map_server map_saver_cli -f ...` 使用的是官方入口，不需要自写 YAML/PGM 导出器。
- `nav2_map_server` 的 map IO 依赖 `yaml_cpp_vendor`、`nav_msgs`、`nav2_util`、TF2 和 GraphicsMagick；Docker 镜像应通过 `ros-humble-nav2-map-server` 安装并执行 `ros2 pkg prefix nav2_map_server`、`ros2 run nav2_map_server map_saver_cli --help` 验证，不能只根据 bridge 编译成功推断可保存地图。
- 建图阶段优先使用 `slam_toolbox` 自带的 `save_map`/`serialize_map`；`map_saver_cli` 只作为独立备份入口并显式指定 `-t /map`。建图过程中不要同时启动 `map_server` 发布第二个 `/map`。
- 该 Humble CMake 对 `WIN32` 有专门的编译定义，但当前目标是 Linux 容器；这项源码事实不能替代原生 Windows ROS2 的完整兼容性验证。

## 15. 外部核验限制

- 一次 GitHub raw 文件请求在 20 秒超时且没有返回内容；另一次 GitHub API URL 未引用，zsh 将查询字符串解释为 glob。后续核验应优先读取已完成的本地浅克隆，或对 URL 使用引号和超时；不能把网络请求失败当作项目源码缺失。

## 16. 第二次严谨性审查（2026-08-29）

本节把“可以实施的设计”和“必须先补证据的阻断项”分开记录。下列结论来自当前工作树源码和已固定的 Humble 参考提交；未完成设备实测的内容仍保持未确认。

### 16.1 阻断：RF 不得在 ROS2 二次反相

- `motor_board_task.c` 的 `ENCODER_DIR_SIGN={+1,-1,+1,+1}` 是 MotorBoard raw feedback 的 STM32 内部校正表（`[motor_board_task.c:60](/Users/zhiqin/Projects/Smart_Car/STM32H757/Middleware/MotorBoard/motor_board_task.c:60)`）。
- 校正后的值写入 `s_actual_wheel_speed`，`0x14` 只把该数组原样编码发送（`[motor_board_task.c:539](/Users/zhiqin/Projects/Smart_Car/STM32H757/Middleware/MotorBoard/motor_board_task.c:539)`、`[motor_board_task.c:565](/Users/zhiqin/Projects/Smart_Car/STM32H757/Middleware/MotorBoard/motor_board_task.c:565)`）。
- 因此 ROS2 的 wire 默认倍率必须是 `[+1,+1,+1,+1]`。实车抬轮、前进、后退抓包只用于确认“车辆前进为正”的最终合同，不能预置 `RF=-1`。

### 16.2 阻断：当前 `0x14` 没有新鲜度证明

- `0x14` payload 固定为 16 字节四个 float，没有源采样时间、sample sequence、valid 或数据年龄（`[srp_registry.h:93-101](/Users/zhiqin/Projects/Smart_Car/Common/SRP/include/srp_registry.h:93)`）。
- STM32 约每 50 ms 重发缓存，而缓存只在新的 `MSPD` 反馈到达时更新；反馈中断后仍可能重复发送最后一个非零值。ROS 收包时间只能代表重发时间，不能代表测量时间。
- P1 必须先在协议评审中选择并冻结一种方案：新增版本化只读 wheel-status（含 `sample_tick/sample_seq/valid` 和 stale 语义，具体 message ID 不在评审前猜测），或明确“仅新反馈发送 + 源端 stale 清零/诊断”的兼容方案。前者是推荐方案。
- 在新鲜度合同、golden capture 和断反馈实测完成前，`0x14 -> /odom` 只能用于离线/实验，不能作为 P1 SLAM 完成证据。

### 16.3 阻断：Windows TCP `8765` 只有一个 owner

- 当前 `TcpServerTransport` 直接 `bind/listen`，每个实例只服务一个客户端（`[transport.cpp:58](/Users/zhiqin/Projects/Smart_Car/ROS2_WIN/src/s3_ydlidar_bridge/src/transport.cpp:58)`、`[transport.cpp:142](/Users/zhiqin/Projects/Smart_Car/ROS2_WIN/src/s3_ydlidar_bridge/src/transport.cpp:142)`）。
- 因此不能让 `s3_ydlidar_bridge` 和 `smartcar_state_bridge` 各自监听 `8765`。推荐保留现有包并把它扩展为唯一 `smartcar_gateway_bridge` 进程：一个 socket/reassembly、通用 S3RD envelope parser、按 message type 分发到雷达 decoder 和 SRP telemetry/odom 模块。
- `smartcar_state_bridge` 可以保留为同进程静态库或 composable component；若做成独立进程，必须改为订阅 gateway 已发布的内部消息，不能再打开 TCP listener。

### 16.4 阻断：S3 组件依赖必须反转

- `radar_uplink.c` 属于 ESP-IDF `main` component，而 `command_bridge.c` 属于 `smartcar_service`；让后者直接包含前者会形成反向依赖。
- 推荐在 `smartcar_service` 公共头文件声明 `smartcar_service_set_telemetry_sink(callback, context)`。`command_bridge` 收到 SRP frame 后在 parser callback 内完成固定大小复制并入队；轮速使用有界 FIFO，姿态等观察流才可 latest-only，回调立即返回；`main/radar` 注册 sink，由低优先级 uplink task 取出并发送。
- `srp_parser_t` 的 `frame->payload` 指向 parser 内部缓冲（`[srp_codec.h:31](/Users/zhiqin/Projects/Smart_Car/Common/SRP/include/srp_codec.h:31)`），不能在回调返回后保存指针，也不能在该回调内阻塞 Wi-Fi。

### 16.5 高风险：三个时钟域必须显式定义

- SRP frame 只有 priority、type、flags、8 位 sequence、length 和 payload，没有通用 timestamp；`0x11` 自带 `timestamp_ms/sample_sequence`，`0x10` 自带传感器 timestamp，当前 `0x14` 没有。
- S3 radar FIFO 在 UART 入队时记录 `esp_log_timestamp()`，S3RD 外层 timestamp 是 S3 uptime 毫秒并会回绕；ROS `rclcpp::Clock` 是主机时钟。三者重启和 wrap 不能直接比较。
- 初版建议以 ROS host receipt time 发布 ROS header，保留 source timestamp/epoch/age 作为 diagnostics；只有 NTP/SNTP 或 offset 估计与误差上限被实测后，才把源时间用于融合。S3 发送前必须按 dequeue age 丢弃过期雷达帧；轮速按源采样顺序进入有界 FIFO，姿态等纯观察流才可 latest-only。

### 16.6 高风险：SRP sequence 是全链路共享序号

- `srp_link_send()` 对所有 message ID 共用一个 8 位 `next_sequence`（`[srp_link.c:170](/Users/zhiqin/Projects/Smart_Car/Common/SRP/srp_link.c:170)`），不是每个 telemetry 类型独立计数器。
- `0x14` 之间出现 `0x11`、`0x10`、日志或状态帧时，inner sequence 不连续是正常现象。验证必须区分 inner SRP sequence、outer S3RD stream sequence 和 DualAHRS `sample_sequence`；不能把 inner 跳变直接当作轮速丢包。
- 若协议需要每流丢包率，outer sequence 应按 `(device_id, stream_id, message_type)` 独立，具体字段和 reset/wrap 规则在协议评审中冻结。

### 16.7 高风险：通用 envelope parser 和 ready 队列边界

- 当前 `S3FrameExtractor` 硬编码 YDLIDAR 最小 10 字节及 CT/flags 校验（`[framing.cpp:72](/Users/zhiqin/Projects/Smart_Car/ROS2_WIN/src/s3_ydlidar_bridge/src/framing.cpp:72)`、`[framing.cpp:101](/Users/zhiqin/Projects/Smart_Car/ROS2_WIN/src/s3_ydlidar_bridge/src/framing.cpp:101)`），不能直接解析 SRP telemetry。
- 应拆成只负责 magic/version/header/length/CRC 的通用 parser，再按 type 执行 payload policy；未知 type 必须计数并丢弃，不能交给 YDLIDAR decoder。
- `TcpChunkAssembler` 当前构造函数忽略 `max_ready_frames`，ready deque 可能在一次恶意大 chunk 中增长。实施时必须执行上限（明确丢弃 oldest/newest 策略）、增加 dropped-ready 计数和压力测试，才可称为 bounded transport（`[framing.cpp:140](/Users/zhiqin/Projects/Smart_Car/ROS2_WIN/src/s3_ydlidar_bridge/src/framing.cpp:140)`）。

### 16.8 高风险：共享协议源码的容器边界

- 当前 Compose 只挂载 `ROS2_WIN/src`，容器看不到仓库根目录 `Common/SRP`。若 ROS2 直接复用 `srp_decode()`/`srp_wire`，必须添加只读挂载（例如 `../../Common/SRP:/ws/Common/SRP:ro`）并在 CMake 中引用该路径，或先建立可复现的共享 package；不能复制一份协议定义。
- 外层 telemetry buffer 按共享 `SRP_MAX_FRAME_SIZE=512` 加 header/CRC 重新预算，并在 Windows 容器中做最大帧编译探针、ASan/边界测试（若工具链允许）。

### 16.9 开源复用的最终边界

| 能力 | 采用方式 | 明确不采用 |
| --- | --- | --- |
| YDLIDAR | 当前官方 SDK 的 host decoder | 不把其串口/DTR 驱动搬到 S3 runtime |
| 里程计积分 | Humble `diff_drive_controller::Odometry` 公开库，commit `eb4ca17d610eb4315f7241c0134de1bdfc5748ea` | 不加载 `diff_drive_controller` controller/plugin，不启动 `controller_manager` |
| SLAM | Humble `slam_toolbox` `online_async`、官方参数和服务 | 不自写 scan matcher/pose graph |
| TF/URDF | `robot_state_publisher` + `xacro` + `tf2_ros` | 不让 RSP 生成 `odom -> base_link` |
| 地图保存 | `slam_toolbox` `save_map/serialize_map`；`nav2_map_server map_saver_cli` 作备份 | 建图过程中不启动第二个 `map_server` |
| 诊断 | `diagnostic_updater`/`diagnostic_msgs`（Humble `ros2-humble`，commit `de779cfd3bff7975f158971c58bedf0581148f9a`） | 不复制 diagnostics 源码 |
| 融合 | `robot_localization` 仅作审查后的可选 EKF，显式 `use_control:false` | 不因“多用开源”默认启用会在超时后继续 predict 的 EKF |

### 16.10 文档和证据边界

- `ROS2_WIN/README.md`、`docs/final-report.md` 和 `docs/s3-gateway-protocol-TODO.md` 已同步为“上行源码存在但实验/默认关闭”；不以旧文档推翻当前源码。
- 当前仅有 host-side bridge 离线测试记录；Windows Docker 镜像重建、真实 Wi-Fi/TCP、真实 telemetry、TF、SLAM、地图保存和车辆验收均未执行。任何交付报告都必须逐层列出固件/ELF hash、镜像 digest、配置、原始 capture、bag 和测试命令。

## 17. 收口后的执行边界

- 轮速里程计的 FIFO 与 stale latch 是实施前置条件；`0x14` 仍只有四个 mm/s float，未完成新 freshness 合同前不得作为 live SLAM 输入。`wheel_diameter` 不再次缩放线速度，只用于可选 `JointState` 的 rad/s。
- 姿态 topic 只能在 valid/fault/stale、有限性、单位和 `wxyz -> xyzw` 字节序检查通过后发布；`0x10` 的 BMI323 gyro 与 LSM303 magnetometer 分开映射。
- 建图产物分两类：标准 YAML/PGM 供 Nav2 map server/AMCL，`slam_toolbox serialize_map` 的 posegraph 供 localization/继续建图；两类都必须持久化并分别重载验证。
- live 模式所有节点显式 `use_sim_time=false`；replay 模式使用 `ros2 bag play --clock` 且所有参与节点显式 `use_sim_time=true`，禁止混用。
- 阶段 3 已完成受控的 S3 telemetry 与主机 envelope 实现；后续 `/odom`、TF、SLAM 和真实设备验收仍需单独批准与证据，不因本阶段代码通过而自动放行。

## 18. S3 telemetry sink 实施核对（2026-08-29）

- `smartcar_service_task -> srp_parser_feed -> srp_link_receive -> command_bridge_on_frame` 在同一个 service task 中同步执行；`frame->payload` 指向 parser 内部缓冲，只在当前 callback 内有效。
- sink 只能在 `smartcar_service_init()` 前注册，回调内仅允许固定大小复制和零等待入队；网络发送继续由低优先级 uplink task 执行。
- service 对 ROS sink 的白名单固定为 `0x11/0x10/0x14`；`0x12/0x13/0x20` 保持现有 App relay，不因本次实现扩大 ROS 数据面。
- 有效 SRP view 使用现成 `srp_encode_frame()` 重建完整 frame，保留 type、priority、flags、inner sequence 和 payload，并重新生成 CRC/EOF；最大编码帧为 512 字节。
- BLE 与 ROS sink 是两个独立消费者；BLE 未连接、notify 失败或 sink 队列满，均不能互相切断，也不能阻塞 UART2、BUS_OFF 恢复或 BLE 断连零速命令。
- legacy `0x14` 仍只是缓存重发。S3 可以原样上行供抓包与离线诊断，但不能生成不存在的 `sample_tick/sample_seq/valid`，也不能据此宣称 live `/odom` 已具备有效新鲜度。

## 19. 阶段 3 实施结果（2026-08-30）

- `radar_telemetry_queue` 已改用 `SRP_MAX_FRAME_SIZE`，对 `0x14`、`0x11`、`0x10` 做完整 wire decode、来源/目的/flags、长度和有限值校验；轮速为有界 FIFO，姿态与两路 IMU 为独立 latest slot。
- `radar_uplink` 注册 `smartcar_service` sink，在 S3 parser callback 外复制数据，使用零等待互斥和 PSRAM 轮速 FIFO；雷达 UART1、STM UART2 和 BLE 控制路径未改变。
- 外层 S3RD 的 `sequence` 现由雷达和 telemetry 共用单一发送计数器；雷达 UART 原始序号只用于检测雷达缺帧和重同步。
- Windows `S3FrameExtractor` 已按 type 分离 raw/telemetry payload policy；telemetry 默认上限直接绑定共享 `SRP_MAX_FRAME_SIZE`（当前 512 字节），`TcpChunkAssembler` 现在消费并统计超过 `max_ready_frames` 的完整帧，bridge 对 telemetry 走独立 SRP decoder/diagnostics 路径，不交给 YDLIDAR decoder。
- 证据仍限于源码、host tests 和交叉编译；没有真实 S3/STM32 capture、协议冻结、Windows Docker live TCP、`/odom`、TF、SLAM 或车辆证据。

## 20. ROS telemetry decoder 实施结果（2026-08-30）

- 新增 `ROS2_WIN/src/s3_ydlidar_bridge/include/s3_ydlidar_bridge/telemetry_decoder.hpp` 与对应实现，直接调用 `Common/SRP` 的 C wire/parser，不复制 SRP wire 定义。
- decoder 接受完整 SRPv4 frame，限制来源为 STM32H757、目的为 ESP32-S3/广播、flags 为 `STREAM_DATA`；按 `0x14`、`0x11`、`0x10` 的固定长度和 schema/sensor id 解码，并拒绝非有限浮点。
- `bridge_node` 将 telemetry 分流到 decoder，发布 diagnostics 计数；`0x14` 的 `freshness_available` 固定为 false，因此当前仍不能安全地产生 live `/odom`。
- ROS CMake 通过 `srp_codec` 静态库链接 `Common/SRP/srp_crc.c`、`srp_wire.c`、`srp_codec.c`；Windows Compose 只读挂载 `../../Common:/ws/Common`，保持单 TCP owner。
- 本机已完成 C/C++ 编译和 wheel decode smoke；因为没有 ROS Humble/colcon/Docker，gtest 与 Windows Docker 验证仍待目标环境执行。

## 21. CM7 协议构建边界复核（2026-08-30）

- 工作树中的 `Common/SRP` 由 active `s3_service.c`/`s3_service.h` 直接使用。CM7 首次重配置曾因共享 codec/include 路径不完整而失败。
- 已在 `STM32H757/CM7/CMakeLists.txt` 纳入 SRPv4 codec 源和 include 路径；CM7 service 与 UART2 均只编译 SRP 实现，不保留兼容协议。
- 修复后 `cmake --build STM32H757/CM7/build/Debug -j2` 通过，ELF 为 `Smart_Car_H757_CM7.elf`，`arm-none-eabi-size` 为 text 175720、data 500、bss 59928 字节。
- 该构建通过只证明当前源码可编译；真实刷写两端仍需用 UART capture 确认 SRPv4 wire format、序列、ACK 和恢复行为。
