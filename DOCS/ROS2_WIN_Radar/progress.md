# S3-ROS2-STM32 构建计划进度

## 2026-08-28

- 已按只读方式核对 S3 雷达 UART/PWM 当前源码、S3-STM32 SRPv4 边界、CM7 规范构建路径、ROS2 Windows 部署资料和工作树状态。
- 已识别活动源码与旧 S3 雷达测试文档的 PWM/监测描述漂移；计划以活动源码为准并将文档同步列为审计门。
- 已新增端到端构建与审计计划；本次未修改 S3、STM32 或 ROS2 业务代码，也未运行构建、网络或硬件测试。

## 2026-08-28：P1 S3 雷达帧接入

- 修改 `ESPS3/main/CMakeLists.txt`，将现有 `radar/radar_parser.c` 纳入实际组件构建，并声明 `esp_driver_uart` 依赖。
- `radar_parser.c` 默认采用 YDLIDAR X3/X3 Pro 官方文档的 AA 55 帧格式和 XOR checksum，支持 2 字节无强度样本、3 字节强度样本的帧定界。
- 新增 `radar_parser_stats_t`，统计有效帧、校验错误、非法帧、重同步和环形缓冲溢出。
- `radar_uart.c` 已将任意 UART 读取块送入解析器；校验通过的完整原始帧写入有界 latest-only 槽，提供序号、帧长和年龄读取接口；GPIO4 PWM、UART2/STM32 链路未改动。
- BLE 日志增加限流的 `RADAR_STATS`，仍保留原始 UART HEX 日志。
- UART 驱动事件队列增加 FIFO/driver-buffer 溢出观测；统计日志同时包含每秒有效帧增量、latest 帧年龄、UART 缓存水位、雷达任务栈余量和 internal/PSRAM 空闲量。
- 新增 `ESPS3/main/radar/tests/test_radar_parser.c` 与 `run_host_tests.sh`，覆盖拆包、粘包、强度/无强度、坏校验、非法 LSN、噪声和拆分包头。
- 验证通过：`sh main/radar/tests/run_host_tests.sh`；`git diff --check`；ESP-IDF 5.5.4 下 `idf.py build`。固件输出为 `ESPS3/build/smartcar_s3_gateway.elf` 与 `.bin`。
- 未执行：刷写、真实 UART/BLE 运行、Wi-Fi 上行、Windows ROS2、车辆或 STM32 行为验收。

## 2026-08-28：P2 S3 Wi-Fi 上行实现与复验

- 新增 `ESPS3/main/radar/radar_uplink_protocol.h/.c`：固定小端网关头、版本/消息类型、device/stream/sequence、S3 单调时间戳、payload 长度和 CRC16-Modbus；只接受已通过 YDLIDAR 校验的完整原始帧。
- 新增 `ESPS3/main/radar/radar_uplink.h/.c`：独立低优先级 Wi-Fi STA/TCP 上行任务，断线重连采用有界指数退避，网络慢时保持 latest-only，不阻塞 UART1 接收、BLE 日志或 STM UART2。
- `main.c` 仅调用可选上行初始化；默认 Kconfig 关闭，SSID、密码、Windows 主机和端口为空时拒绝启动；GPIO4 PWM、UART2/SRPv4、BLE 命令含义未改动。
- 收紧 TCP 连接行为：非阻塞 `connect()` 加 `select()` 超时，发送使用独立 `SO_SNDTIMEO`；Wi-Fi SSID/密码超过芯片字段容量时拒绝而不是静默截断。
- 主机验证通过：`sh main/radar/tests/run_host_tests.sh`、两个 `-std=c11 -pedantic -Wall -Wextra -Werror` 测试、ASAN（`detect_leaks=0`）和 UBSAN；`git diff --check` 通过。
- 上行协议测试补充了零位包标志、版本/消息类型拒绝和最大合法强度帧（255 点）round-trip，严格编译、ASAN 和 UBSAN 复验均通过。
- ESP-IDF 5.5.4 验证通过：默认关闭配置构建 `ESPS3/build/smartcar_s3_gateway.elf/.bin`；临时启用配置在 `/tmp/smartcar_s3_uplink_build` 构建通过，测试配置未进入仓库。
- `idf.py size`：默认关闭镜像约 723561 bytes、应用分区余量 90%；临时启用上行镜像约 1217549 bytes、应用分区余量 83%；两者均通过分区大小检查。
- 未执行：固件刷写、真实 Wi-Fi/TCP、Windows bridge、完整雷达连续帧、BLE 实时日志、跨 NAT/公网中继和车辆行为验收；不能据此宣称端到端链路已验收。

## 2026-08-28：Wi-Fi 多网络凭据配置

- 新增 `ESPS3/main/radar/radar_wifi_credentials.h`，以 SSID/密码结构体数组维护多个可用 Wi-Fi；不在仓库中写入真实凭据。
- `radar_uplink.c` 启动时校验列表中每一项，断线后按列表顺序循环切换；日志只输出 SSID 和错误名，不输出密码。
- 移除 Kconfig 中单 SSID/密码字段，Windows bridge host/port、device/stream ID 和上行开关保持原配置方式；UART、PWM、BLE、STM32 链路未改动。

## 2026-08-28：S3 与 ROS2_WIN 协议冻结前规划

- 按用户要求暂停代码实施，仅建立 S3 与 ROS2_WIN 的分阶段计划；本次未修改源码、构建配置、协议字段或硬件连接。
- 将协议冻结设为共同前置门，明确 TCP/UDP、帧头、版本、长度、序号、时间戳、payload、CRC、可靠性和安全策略未定前，不实现正式 live receiver，也不宣称真实 `/scan` 已打通。
- 规划覆盖 S3 资源/隔离审计、ROS2_WIN 官方资料复用、离线 golden/replay、同网端到端、跨网中继和运维安全验收；STM32 继续不承载雷达数据。

## 2026-08-28：Windows ROS2 环境恢复后的双端计划

- 用户报告 Windows Docker/WSL2 engine 已恢复，Docker 29.7.2、`docker compose build`、`colcon build`、`colcon test` 和 16/0 测试通过；该证据尚未在本机复核。
- 将下一步拆为 N1 GUI smoke 与 S3 原始 UART 录包并行、N2 共同冻结协议、N3 双端并行实现、N4 离线回放、N5 同网台架、N6 跨网发布。
- 明确当前仍不修改或启用 live receiver；缺少网关规范和真实抓包前，ROS2_WIN 保持 `unconfigured`，不宣称 `/scan` 已打通。

## 2026-08-29：S3-WIN 雷达协议计划文档

- 新增 `DOCS/ROS2_WIN_Radar/S3_WIN_RADAR_PROTOCOL_PLAN.md`，汇总 S3 与 Windows ROS2 的协议目标、候选包格式、协议冻结门、双端实施顺序、golden packet/replay、同网台架和跨网 relay 验收要求。
- 更新 `DOCS/DEVELOPMENT_INDEX.md`，增加协议计划文档入口。
- 本次仅修改文档；未修改 S3、ROS2_WIN、STM32 源码或构建配置，也未启用实验性 Wi-Fi 上行或声称真实 `/scan` 已打通。

## 2026-08-29：S3 有效雷达帧 FIFO 上行

- 将 `ESPS3/main/radar/radar_uart.c` 的单槽 latest-frame 改为固定八槽完整帧 FIFO；满载只丢弃最旧帧并记录 `fifo_drop_oldest`，不执行动态分配或网络阻塞。
- `radar_uplink.c` 按 FIFO 顺序发送所有已接收的校验有效帧，保留原始 parser 时间戳；TCP 发送中断时保留一个待发送包，重连后先重试；发送轮询由 20 ms 收紧为 5 ms，避免人为限制正常 X3PRO 包速率。
- S3RD 26 字节头、device/stream ID、flags、CRC、UART1/GPIO44、GPIO4 PWM、UART2/SRPv4 和 BLE 原始 UART 日志开关均未改变。
- 新增 `radar_frame_fifo.[ch]` 及主机测试，覆盖顺序、元数据、满载丢旧、空队列和短缓冲行为。
- ESP-IDF 5.5.4 上行启用构建、`idf.py size`、主机测试、ASAN/UBSAN 和 `git diff --check` 均通过；未刷写、未做真实 Wi-Fi/TCP 或 Windows 整圈组帧验收。

## 2026-08-29：S3 雷达 PSRAM FIFO 与非阻塞发送优化

- 将完整帧 FIFO 扩展为 256 槽，并使用 `heap_caps_calloc()` 从 PSRAM 分配，约占 200 KiB；分配失败会明确返回错误，不静默退回小容量。
- 有效帧入队后通过 FreeRTOS task notification 唤醒 uplink，FIFO 锁为零等待；UART 接收缓冲、解析器和任务栈仍保留在内部 RAM。
- TCP 连接建立后保持 `O_NONBLOCK`；新增发送状态机，保留部分写入偏移，`EAGAIN` 不关闭 socket，并将每次尝试限制为 16 次 `send()` 调用。
- 初次连接或重连后等待下一次 YDLIDAR 零位包，期间丢弃非零位旧帧，避免 Windows/ROS 将断线前后的半圈拼接；新增发送等待、部分写入、重试、重同步丢弃和编码失败诊断。
- 主机测试、ASAN/UBSAN、ESP-IDF 5.5.4 `idf.py build`、`idf.py size` 和 `git diff --check` 均通过；未刷写，未执行真实 Wi-Fi/TCP、Windows bridge 或 ROS2 整圈验收。
