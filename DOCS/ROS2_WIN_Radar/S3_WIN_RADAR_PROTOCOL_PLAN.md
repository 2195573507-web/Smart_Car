# S3 与 Windows ROS2 雷达协议计划

## 1. 文档状态

| 项目 | 内容 |
| --- | --- |
| 状态 | 协议冻结前的设计与实施计划 |
| 目标 | 建立 `YDLIDAR -> ESP32-S3 -> Wi-Fi -> Windows ROS2 -> /scan` 的可审计数据链 |
| 本轮范围 | S3 雷达数据上行、Windows 接收/解码、ROS2 `LaserScan`、诊断、离线回放和联调验收 |
| 明确排除 | 不修改 GPIO4 PWM；不把雷达原始数据发给 STM32；不开放 ROS2 直接控制车辆；不把 DDS 组播作为跨网传输 |
| 当前结论 | 可以开始准备和采集证据，但正式 live transport 必须等待协议冻结 |

本文件是计划，不代表真实 Wi-Fi、Windows 网络接收器或 ROS2 `/scan` 已经打通。

## 2. 已确认事实与证据边界

### 2.1 当前系统边界

```text
YDLIDAR
  -> S3 UART1 RX / GPIO44 / 115200 8N1
  -> S3 帧定界与 YDLIDAR 校验
  -> S3 Wi-Fi 上行
  -> Windows Docker/WSL2 Linux ROS2
  -> bridge 解码
  -> sensor_msgs/msg/LaserScan (/scan)
```

- S3 是雷达 UART 的唯一拥有者；GPIO4 PWM 保持现状。
- STM32 继续负责既有控制、安全和执行链路，雷达数据不上行到 STM32 UART2/SRPv4。
- Windows 端 Docker/WSL2、`colcon build`、`colcon test` 和 16 个测试通过，是用户提供的环境基线；`rviz2`/`rqt` 显示和真实网络仍待验证。
- 最新 S3 日志只证明 IO44 持续收到 UART 数据；其中多次出现 `AA 55`，但日志是截断 HEX，不能证明完整帧校验、Wi-Fi 传输或 `/scan`。
- S3 当前存在实验性 LAN/TCP 上行代码。其字段只能作为候选设计参考，不能直接视为发布接口。

### 2.2 不得混淆的证据层级

| 证据 | 可以说明 | 不能说明 |
| --- | --- | --- |
| host 单元测试 | 编解码和异常输入的逻辑正确性 | S3 实机、Wi-Fi 或 ROS2 行为 |
| S3 固件构建 | 源码可编译链接 | 已刷写、串口连续性或网络可用 |
| WIN `colcon` 构建/测试 | 容器工作区和离线测试链可用 | `rviz2` 显示真实数据或跨主机连接 |
| 抓包和两端日志 | 某次台架链路的实际行为 | 未测试网络或车辆场景的安全性 |

## 3. 协议目标与设计原则

1. 传输边界与雷达解析边界分离：S3 只上行完整、已校验的 YDLIDAR 帧；Windows 负责协议重组和 ROS2 语义转换。
2. 所有输入使用有界缓冲。网络拥塞只丢弃旧雷达数据，不阻塞 UART、BLE 或 STM32 控制链路。
3. TCP 必须按协议长度处理半包和粘包；一次 `recv()` 不等于一个网关包。
4. 每个包携带设备身份、流身份、序号和 S3 时间，用于去重、丢包、延迟和过期诊断。
5. 内层 YDLIDAR 校验负责雷达帧完整性，外层 CRC 或 TLS 完整性负责网关包完整性；两层职责不能互相替代。
6. 协议版本、最大长度和错误行为必须先冻结，再实现正式 live receiver。
7. 协议只承载雷达数据和诊断，不承载急停、速度指令或其他执行器控制。

## 4. 候选网关包格式（待评审，不是正式合同）

当前 S3 实验代码使用以下候选格式，供双方评审和生成 golden packet。协议冻结前允许调整。

| 字段 | 长度 | 候选含义 |
| --- | ---: | --- |
| `magic` | 4 | ASCII `S3RD`，用于重同步 |
| `version` | 1 | 协议版本，当前候选为 `1` |
| `message_type` | 1 | 当前候选 `RAW_YDLIDAR_FRAME` |
| `flags` | 2 | 强度/无强度、零位包等标志；未知位必须拒绝 |
| `device_id` | 4 | S3/雷达设备身份 |
| `stream_id` | 4 | 数据流身份，用于重启和多流区分 |
| `sequence` | 4 | 单调递增序号，允许回绕并定义回绕规则 |
| `timestamp_ms` | 4 | S3 单调时间，主要用于诊断和 stale 判定 |
| `payload_length` | 2 | 原始 YDLIDAR 帧长度，接收前检查上限 |
| `payload` | 可变 | 完整 `AA 55` YDLIDAR 帧，保留原协议校验字段 |
| `crc` | 2 | 候选为 CRC16-Modbus；覆盖范围需冻结 |

候选格式采用小端字段，候选固定头长为 26 字节。`payload` 不得是任意 UART 读取块，必须先通过 S3 的 YDLIDAR 帧校验。Windows 端仍须再次验证内层帧，不能只信任外层 CRC。

## 5. 协议冻结门（G1）

以下问题全部有明确答案并记录评审结论后，才能进入正式网络接收器实施：

| 类别 | 必须冻结的内容 |
| --- | --- |
| 传输 | LAN 使用 TCP 还是 UDP；连接方向；端口；keepalive；远程是否使用 relay |
| 帧格式 | 魔数、版本、固定头长、字节序、消息类型、未知字段处理 |
| 身份 | `device_id`、`stream_id` 的分配、重启后的变化和冲突处理 |
| 序号 | 起始值、递增点、回绕、重复、乱序和丢包判定 |
| 时间 | 单调时间单位、回绕、Windows/ROS 时间映射、stale 阈值 |
| 负载 | 原始 YDLIDAR 帧或规范化扫描；强度/无强度标志；最大长度 |
| 完整性 | 内层 XOR 与外层 CRC 的覆盖范围、错误码和丢弃策略 |
| 可靠性 | 半包、粘包、断线、重连首帧、背压、重复发送和过期包处理 |
| ROS 语义 | `frame_id`、角度方向、角度单位、距离单位、量程、无效值、intensity 语义 |
| 安全 | LAN 凭据、远程 TLS、设备身份、ACL、证书轮换和日志脱敏 |

在 G1 前，`SMARTCAR_RADAR_UPLINK_ENABLED` 保持关闭；Windows 保持 `unconfigured`，不声称真实设备 `/scan` 已打通。

## 6. S3 端实施计划

### S3-P0：采集与资源基线

- 采集 IO44 原始 UART 二进制数据，而不是只保存 BLE/控制台 HEX 日志。
- 至少覆盖正常连续帧、强度/无强度模式、启动/停止、雷达断电恢复和异常字节流。
- 每个录包登记雷达型号、固件（若可得）、供电、M_CTR 状态、波特率、采集时间和硬件版本。
- 记录有效帧数、校验错误、UART 溢出、任务栈余量、内部 RAM/PSRAM 水位。

退出条件：有可复现的完整帧和异常帧输入，且没有改变 GPIO4 PWM、UART2/SRPv4 或 BLE 控制含义。

### S3-P1：协议 codec 与 transport

- 按冻结合同编码网关头、长度和完整性校验。
- 由独立低优先级任务按序发送有界 PSRAM FIFO 中的完整帧；不得在 UART 任务中执行阻塞网络操作。
- 明确 Wi-Fi STA、DNS、连接超时、断线、指数退避、重连和发送失败计数。
- 多 Wi-Fi 凭据继续使用本地列表配置；真实密码不进入日志或公开提交。
- 发送前只取完整、通过 YDLIDAR 校验的帧；网络慢时只在有界 FIFO 满载时丢弃最旧帧，重连后等待零位包重新对齐扫描边界。

退出条件：host/固件构建和 codec 单测通过；断网/恢复期间 S3 不阻塞 UART、不重启、不影响 STM32 控制链路。

## 7. Windows ROS2 端实施计划

### WIN-P0：运行环境和 GUI smoke

- 在现有 Ubuntu 22.04/ROS2 Docker 容器中确认 `rviz2`、`rqt`、WSLg 显示和 ROS graph。
- 固定镜像、ROS 发行版、RMW 实现、`ROS_DOMAIN_ID` 和依赖版本。
- 先验证容器内 UI 和离线节点，不接入未冻结的 S3 网络包。

退出条件：可重复启动容器，RViz/rqt 有截图和日志证据。

### WIN-P1：bridge 与官方解码复用

建议工作区新增最小 bridge 包，例如 `s3_ydlidar_bridge`，只负责：

1. TCP/relay 字节流接收和半包/粘包重组；
2. 网关版本、长度、CRC、设备身份和序号检查；
3. 复用资料中官方 YDLIDAR SDK/ROS2 driver 的协议解码约定；
4. 将合法数据发布为 `sensor_msgs/msg/LaserScan`；
5. 发布连接、CRC、丢包、重复、过期、帧率和最后有效时间等 diagnostics。

不直接复制一套新的 YDLIDAR 解析器，不让 receiver 与 ROS publisher 共享无界缓存，也不让 bridge 获得车体运动控制权。

退出条件：离线 golden packet 可稳定发布 `/scan`；错误输入能进入明确的诊断状态；`rviz2` 能显示回放数据。

## 8. 共享测试与联调顺序

### 8.1 Golden packet / replay

双方共享同一版协议文档和测试向量，至少包含：

- 合法无强度帧；
- 合法强度帧；
- 最大合法 payload；
- 半包、粘包和多个包一次读出；
- 魔数、版本、消息类型、长度、未知 flags 错误；
- 外层 CRC 错误和内层 YDLIDAR 校验错误；
- 重复、乱序、序号跳变、序号回绕；
- 断线、重连和 stale 超时。

回放验收需要同时检查原始 payload、`LaserScan` 字段、诊断计数和时间语义，不能只看 RViz 画面。

### 8.2 同一 LAN 台架

1. 刷写匹配的 S3 镜像，Windows 运行固定容器镜像。
2. 记录 S3 连接/发送计数、Windows 接收/丢包/重复计数和网络抓包。
3. 比较 S3 序号/时间与 Windows 接收时间，计算延迟、抖动和扫描年龄。
4. 注入 Wi-Fi 断开、S3 重启、雷达断电、Windows bridge 重启，验证恢复和降级。
5. 用固定障碍物检查 `/scan` 角度方向、距离、量程、`frame_id` 和频率。

### 8.3 跨网部署

同网验收通过后，才评估 MQTT over TLS 或 WSS relay。S3 与 Windows 均主动出站连接，使用设备身份、ACL、TLS 校验和独立凭据。不得用 DDS multicast、宽泛端口转发或未认证 TCP 作为正式跨网方案。

## 9. STM32 边界

本协议计划不新增 STM32 接口。以下不变量必须保持：

- 雷达原始帧、网关包和 `/scan` 不进入 STM32 UART2/SRPv4；
- GPIO4 PWM 不因网络上行而改变；
- STM32 仍是底盘执行和最终安全停止的所有者；
- ROS2 未来若需要控制车辆，必须另立控制协议和安全评审，不能复用雷达上行通道。

## 10. 交付物与审计证据

| 阶段 | 必须交付 |
| --- | --- |
| P0 | 原始 UART 录包、元数据、网络拓扑、协议评审记录 |
| P1 | 协议文档、字段表、golden packet、异常向量和 S3 codec/transport 测试 |
| P2 | S3 构建日志、固件哈希、资源水位、断网/恢复记录 |
| P3 | WIN 容器定义、镜像标识、`colcon build/test`、bridge 回放报告 |
| P4 | 同网抓包、两端日志、rosbag、`/scan`/RViz/rqt 证据和延迟统计 |
| P5 | 跨网 TLS/ACL/NAT/重连/长期运行报告 |

每项证据须注明日期、Git commit、固件/镜像版本、配置来源、命令和“已证明/未证明”结论。

## 11. 当前立即行动

1. S3 端先完成真实 UART 原始录包和元数据登记。
2. WIN 端先完成 `rviz2`/`rqt` GUI smoke，不接真实网络。
3. 双方依据录包冻结第 4 节协议字段和第 5 节所有决策。
4. 协议冻结后，S3 codec/transport 与 WIN bridge/decoder 并行实现。
5. 先离线 replay，再同一 LAN，最后跨网 relay。

详细阶段进度仍记录在 [`task_plan.md`](../history/context/task_plan.md)、[`findings.md`](../history/context/findings.md) 和 [`progress.md`](../history/context/progress.md)。
