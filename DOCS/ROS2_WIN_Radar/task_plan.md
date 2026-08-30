# ROS2_WIN 雷达资料学习与移植文档计划

## 目标

在不修改源码、固件、压缩包或工程配置的前提下，建立 ROS2_WIN 的雷达学习、驱动移植和 Docker 部署准备文档。

## 阶段

- [x] 读取项目治理、架构与索引文档，明确 DOCS 为唯一可写范围。
- [x] 记录 X3/X3 Pro 本地资料、SDK 与 ROS1/ROS2 driver 归档的静态清单。
- [x] 对齐独立 Docker 架构审计的设备归属与 Windows 设备转交风险。
- [x] 建立四份指定学习/移植/部署文档。
- [x] 更新开发索引并执行文档静态检查。

## 证据边界

本任务只提供静态资料、归档目录清单和项目文档交叉检查。未解压、修改或编译资料；未安装或运行 Docker、ROS 2、SDK、driver、SLAM Toolbox 或 Nav2；未连接雷达。

## 2026-08-28：S3-ROS2-STM32 构建审计计划

- [x] 对齐当前 S3 雷达输入、活动构建清单、S3-STM32 SRPv4 边界与 CM7 Debug 构建路径。
- [x] 新增分阶段的 S3、Windows ROS2 与 STM32 构建/审计计划，明确雷达不上 STM32 与 ROS2 暂不控制车辆的范围。
- [x] 记录当前 S3 源码与旧雷达测试文档存在的 PWM/监测描述漂移，作为实施前的审计门。
- [x] P1 已接入 S3 雷达帧定界、官方 XOR 校验、统计和 latest-only 原始帧槽；host 测试与 ESP-IDF 构建通过。
- [x] P2 S3 Wi-Fi 上行实验性源码：已加入 STA、TCP 主动连接、重连退避、latest-only 上行、网关包头和 CRC16-Modbus 校验；未提交真实凭据。
- [x] P3 S3 有效帧可靠上行：以固定八槽 FIFO 替换 latest-only 单槽；满载丢最旧帧、发送失败保留待发送包，S3RD 字段不变。
- [ ] P2 现场验收：仍需真实 Wi-Fi、Windows bridge、丢包/重连、长时间资源和跨网方案验证；当前协议仍需独立评审。
- [ ] 真实 S3 刷写、串口/BLE 台架验证和长时间压力证据。

## 2026-08-28：S3 与 ROS2_WIN 协议冻结前实施计划（仅计划）

### 当前结论

- S3 当前存在实验性 LAN/TCP 上行草案，但外层网关协议尚未冻结；它不能作为 Windows live receiver 或 ROS2 `/scan` 的正式输入合同。
- 本计划不修改 S3、ROS2_WIN、STM32、网络配置或协议代码；`SMARTCAR_RADAR_UPLINK_ENABLED` 在协议冻结和现场验收前保持关闭。
- 数据边界保持为 `YDLIDAR -> S3 UART1/GPIO44 -> S3 校验 -> Wi-Fi -> ROS2_WIN`；雷达原始数据不进入 STM32 UART2/SRPv4，ROS2 暂不获得车体运动控制权。

### 阶段与退出条件

| 阶段 | S3 工作 | ROS2_WIN 工作 | 退出条件 |
| --- | --- | --- | --- |
| G0 基线审计 | 固定 UART1 GPIO44、115200 8N1、GPIO4 PWM 和 latest-only 约束；确认雷达型号、固件和强度模式 | 清点官方 ROS2 教程、SDK、driver 归档和 Windows/WSL2/Docker 运行边界 | 有实物型号、至少一组完整串口抓包、运行环境路线明确 |
| G1 协议冻结 | 不写实现；提供 S3 能力和资源约束，评审原始帧最大长度、发送频率和时间来源 | 提出接收、解码和 `/scan` 所需字段 | 冻结 TCP/UDP、魔数、版本、头长、消息类型、device/stream、序号、时间戳、payload 长度、CRC、最大帧长、分片/粘包、重复/丢包和错误状态 |
| G2 S3 传输实现 | 将协议编码、Wi-Fi 连接、重连、背压和凭据配置按冻结合同实现；与 UART task 隔离 | 提供协议 golden packet 和异常输入，暂不宣称 live | S3 host/firmware 构建通过，协议单测覆盖，资源预算和控制链隔离审计通过 |
| G3 ROS2_WIN bridge | 提供固定格式包的接收证据需求，不再临时变更字段 | 在 `ROS2_WIN` 建立可复现 workspace；复用官方 ROS2/SDK 的消息和雷达解码约定，新增最小网关接收节点、参数、launch 和 diagnostics | 本地回放能处理 TCP 字节流分片、CRC/版本/长度错误、序号跳变和超时，并产生明确状态 |
| G4 `/scan` 回放 | 提供带序号、时间戳和原始 YDLIDAR 帧的录包 | 将合法帧解码为 `sensor_msgs/msg/LaserScan`；冻结 `frame_id`、角度单位、距离单位、量程、无效值和 intensity 语义；与官方 SDK 输出对比 | 离线 golden/replay 测试通过；不依赖真实 Wi-Fi 或车辆 |
| G5 同网端到端 | 刷写匹配镜像，验证 UART 持续接收、Wi-Fi/TCP 连接、断线恢复和资源水位 | Windows bridge 接收并发布 `/scan`，记录延迟、序号间隔、CRC/丢包/重复和 stale 状态 | 同一 LAN 下形成 S3、Windows、ROS2 日志和抓包证据；仍不做车辆运动测试 |
| G6 跨网与运维 | 若 S3 与 Windows 不在同一 LAN，接入经批准的 MQTT/TLS 或 WSS relay；不把 DDS multicast 当跨网方案 | 配置 relay 客户端、设备身份、ACL、TLS、启动/停止和故障诊断 | NAT、断网、重连、证书/权限和长期运行通过；正式发布前完成安全审计 |

### G1 协议冻结清单

1. 传输：TCP 或 UDP；是否允许公网/跨 NAT；连接方向、端口和 keepalive。
2. 帧格式：魔数、协议版本、固定头长度、消息类型、字节序、device/stream ID、序号。
3. 时间：S3 单调时间戳的单位、回绕处理、Windows/ROS 时间映射和 stale 判定。
4. 负载：原始已校验 YDLIDAR 帧还是规范化扫描；强度/无强度标志、最大 payload、分片和重组规则。
5. 完整性：内部 YDLIDAR XOR 与外层 CRC 的职责、覆盖范围、错误响应和丢弃策略。
6. 可靠性：粘包/半包、重复、乱序、丢包、序号回绕、重连后的首帧和背压行为。
7. 安全：凭据与设备身份、TLS/证书、ACL、日志脱敏和版本兼容策略。

### 实施约束

- 在 G1 完成前，不新增或修改 live 网络接收器，不把实验性字段当正式 API，不发布 `/scan` 已打通的结论。
- S3 解析、Wi-Fi 和 ROS2 bridge 都必须使用有界缓冲；网络拥塞只能丢弃旧雷达数据，不得阻塞 UART、BLE 或 STM32 控制链路。
- 所有构建、回放、抓包、硬件和车辆证据分开记录；构建通过不等于串口、Wi-Fi、ROS2 或 `/scan` 验收。

## 2026-08-28：Windows ROS2 环境基线后的下一步

### 已知基线（用户提供）

- Docker/WSL2 Linux engine 已恢复；`docker version` 为 Client/Server `29.7.2`，上下文为 `desktop-linux`。
- Windows 端 `docker compose build`、`colcon build`、`colcon test` 均通过，测试为 16 个、0 failures，`git diff --check` 通过；用户报告仅修改了 `ROS2_WIN/`。
- 以上结果证明 ROS2 工程环境和离线测试链可用；尚未证明 `rviz2`/`rqt` GUI 显示、真实 S3 Wi-Fi、网关报文或 `/scan`。
- 当前唯一阻塞是 S3 Wi-Fi 网关协议和真实抓包缺失；ROS2_WIN 继续保持 `unconfigured`，不猜测字段。

### 双端执行顺序

| 顺序 | S3 端 | Windows/ROS2_WIN 端 | 依赖与产物 |
| --- | --- | --- | --- |
| N1 并行准备 | 保持上行关闭；采集 UART1/GPIO44 的原始二进制帧，记录波特率、强度模式、时间和断线情况；不改 GPIO4 PWM、不接 STM32 | 在已通过构建的容器内做 `rviz2`、`rqt` GUI smoke test，确认 WSLg 显示、ROS graph 和容器启动方式 | S3 录包 + Windows GUI 截图/日志；不产生 live `/scan` 结论 |
| N2 共同冻结 | 根据抓包确认帧边界、频率、最大长度和 S3 时间能力 | 根据 `/scan` 需求审查接收、解码、诊断和 UI 字段 | 一份双方签字的网关协议表 |
| N3 并行实现 | 按冻结协议实现 transport/codec/Wi-Fi；保留有界 latest-only 和故障隔离 | 按冻结协议实现 receiver、YDLIDAR 解码、`LaserScan` publisher、diagnostics、launch/config；复用官方 SDK/ROS2 代码约定 | S3 构建 + Windows 构建/测试 + golden packet |
| N4 离线回放 | 提供有效、强度/无强度、坏 CRC、截断、粘包和序号跳变录包 | 回放网关包，验证 `/scan`、`rviz2`、`rqt` 和错误状态 | replay 结果与官方 SDK 输出对照 |
| N5 同网台架 | 刷写匹配镜像，验证 Wi-Fi 连接、TCP/UDP 行为、重连和资源水位 | 运行 bridge 和 UI，记录 packet、延迟、丢包、重复、stale 和 `/scan` 频率 | 同一 LAN 的双端日志、抓包和屏幕证据 |
| N6 跨网发布 | 仅接入批准的 MQTT/TLS 或 WSS relay；不把 DDS multicast 暴露到跨网 | 配置 relay、TLS、设备身份、ACL、启动和故障诊断 | NAT、证书、权限和长期运行报告 |

### 下一步的实际先后

1. Windows 先完成 `rviz2`/`rqt` 显示验证，但只验证 UI 环境，不接入猜测的 S3 协议。
2. S3 采集一组可复现原始 UART 录包，并登记雷达型号、强度模式、供电、M_CTR 状态和采集时间。
3. 双端依据录包和 ROS2 需求冻结网关协议；在此之前不实现正式 live receiver。
4. 协议冻结后，S3 transport/codec 与 ROS2_WIN receiver/decoder 可以并行开发，但必须共享同一版协议文档和 golden packet。
5. 先做离线回放，再做同一 LAN，最后才做跨 Wi-Fi/NAT；每一级都有独立退出条件。

### 当前不可宣称事项

- Docker/WSL2、ROS2 构建和 16 个单元测试通过，不等于 `rviz2`/`rqt` 已显示真实数据。
- 现有实验性 TCP 草案不等于正式网关协议。
- 没有真实 S3 Wi-Fi 报文和双端抓包，不能宣称真实设备 `/scan` 已打通。
- STM32 端本阶段不需要修改，也不承载雷达原始数据。
