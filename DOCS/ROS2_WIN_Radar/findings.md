# S3-ROS2-STM32 构建计划事实记录

## 2026-08-28 静态基线

- 当前 S3 雷达输入是 UART1、GPIO44 RX、115200 8N1；GPIO4 PWM 当前源宏为 0%。
- （P1 实施前基线）`ESPS3/main/radar/radar_parser.c` 已存在，但当时未被当前 `ESPS3/main/CMakeLists.txt` 编译且 UART 任务未调用它；该问题已在本轮修复。
- S3 与 STM32 的现有控制链是 UART2 GPIO17/18、921600 8N1、SCBP-CAN；雷达数据不应接入此链路。
- CM7 规范构建目录是 `STM32H757/CM7/build/Debug`，唯一固件 ELF 为 `Smart_Car_H757_CM7.elf`。
- Windows ROS2 目录尚无实际工作区；本计划选择先使用同一 LAN 的自定义 TCP 网关，跨 Wi-Fi/NAT 远程模式再使用 MQTT/TLS 或 WSS 中继。
- 旧 `ESPS3/docs/S3_YDLIDAR_X3PRO_TEST.md` 与当前活动源码存在 PWM/监测描述漂移；实施前必须按源码和实物重新审计后同步文档。

## 2026-08-28 P1 实施发现

- 现有日志中的数据块包含 `AA 55`、`CT=0`、`LSN=0x28` 等 X3/X3 Pro 扫描包特征；此前仅能证明 UART 收到原始字节，不能证明完整帧校验。
- 官方 SDK 协议文档的 checksum 为 `PH ^ FSA ^ each Si ^ (LSN << 8 | CT) ^ LSA`，强度模式的每个三字节样本拆为一个强度字节与一个小端距离字；S3 解析器已按此规则实现默认校验。
- 解析器保留拆分 `AA` 包头、按 `LSN * sample_bytes + 10` 计算长度，并在校验通过后才更新 latest-only 槽；异常流不会把 UART 读取块直接视为有效帧。
- Wi-Fi 上行尚未实现：当前计划没有冻结外层网关报文字段、最大长度和网络端点，继续编码会引入不可审计的协议猜测。
- S3 构建验证确认 `radar_parser.c` 和 `radar_uart.c` 均已进入 `main` 组件；本次新增代码未修改 GPIO4 PWM 宏、UART2/SCBP-CAN 或 STM32 文件。

## 2026-08-28 P2 代码复验

- 当前 S3 上行实现采用实验性 LAN TCP：S3 主动连接 Windows bridge，TCP 字节流由接收端按固定 26 字节头、payload 长度和 2 字节 CRC16-Modbus 重组；不能把一次 `recv()` 视为一帧。
- 网关 payload 是 S3 已通过 YDLIDAR XOR 校验的完整 `AA 55` 原始帧；S3 不向 STM32 发送雷达原始数据，也不改变 GPIO4 PWM、BLE 或 UART2/SCBP-CAN 所有权。
- 上行任务保留最新序号帧；断线或发送失败时关闭 socket 并退避重连，恢复连接后可能重复发送最新帧，Windows 端必须依据 `sequence` 去重并统计间隔。
- TCP connect 已改为非阻塞加 `select()` 超时；发送超时独立于连接超时。该实现只完成源代码级边界，仍未证明真实网络下的时延、丢包和恢复行为。
- Wi-Fi 凭据现在由 `ESPS3/main/radar/radar_wifi_credentials.h` 的列表维护，启动前逐项检查非空和长度，过长配置返回参数错误；真实密码不打印，也不应提交到公开仓库。
- P2 仍是实验性协议实现：device/stream 身份、最大长度、认证/TLS、跨 Wi-Fi/NAT 传输和 Windows bridge 端合同需要独立评审后才能作为发布接口。

## 2026-08-29 S3 FIFO 上行复验

- `radar_uart.c` now copies each checksum-valid AA55 frame into an eight-entry
  fixed FIFO. The FIFO uses complete-frame storage, preserves sequence and
  parser timestamp, and drops only the oldest entry when full.
- `radar_uplink.c` consumes FIFO entries oldest-first. A packet removed from the
  FIFO remains in a fixed pending buffer until `send_all()` succeeds, so a
  short TCP write followed by reconnect is retried as one complete S3RD packet.
- The current active `ESPS3/sdkconfig` has uplink enabled for
  `192.168.31.101:8765`, device/stream IDs `1/1`; credentials were not printed
  or changed by this work.
- Host parser/FIFO/protocol tests, ASAN/UBSAN, `git diff --check`, ESP-IDF
  5.5.4 build, and `idf.py size` passed. No flash or live Wi-Fi/TCP test was
  performed in this turn.

## 2026-08-28 协议冻结门

- 当前源码中的 S3 上行仅是实验性 LAN/TCP 草案；魔数、版本、头长、消息类型、序号、时间戳、payload 边界、外层 CRC、分片/粘包、重连和安全策略尚未获得正式评审结论。
- 因此不能安全实现正式的 Windows live receiver，也不能声称真实设备已经发布 ROS2 `/scan`。现有上行开关应保持关闭，直到 G1 协议冻结阶段完成。
- 后续建议先做离线 golden packet/replay，再做同一 LAN 的 S3 -> Windows -> `/scan` 台架验收，跨 Wi-Fi/NAT 只使用经过批准的 MQTT/TLS 或 WSS 中继；不把 DDS multicast 作为跨网传输。

## 2026-08-28 Windows 环境基线（用户报告）

- Docker/WSL2 Linux engine 已恢复，Docker Client/Server 为 `29.7.2`，上下文为 `desktop-linux`。
- `docker compose build`、`colcon build`、`colcon test` 均通过；测试结果为 16 个、0 failures，`git diff --check` 通过。用户报告该次仅修改 `ROS2_WIN/`。
- 该证据只覆盖 Windows ROS2 工程的构建和离线测试；`rviz2`/`rqt` 显示、S3 Wi-Fi、网关协议和真实 `/scan` 仍未验证。
- Windows 端默认 `unconfigured` 是正确状态，必须等到网关协议规范和有效/异常抓包输入后，才能实现 live transport。

## 2026-08-29 S3 端优化复验

- 源码已从八槽内嵌 FIFO 改为 256 槽 PSRAM FIFO；最大完整帧槽位约 200 KiB，仍保持有界 oldest-drop 策略。
- UART 有效帧入队后通知独立低优先级 uplink task；TCP socket 连接后保持非阻塞，部分写入和 `EAGAIN` 由状态机保留偏移，不再因短暂背压立即重连。
- 重连后按原始 `CT` bit0 等待下一次零位包，并丢弃等待期间的旧非零位帧；S3RD 头、flags、序号、时间戳、CRC 和 UART1/GPIO44 均未修改。
- 本轮通过 host/ASAN/UBSAN、ESP-IDF 5.5.4 构建、`idf.py size` 和差异检查；PSRAM 实机分配、持续 Wi-Fi/TCP goodput、Windows/ROS2 整圈完整性仍未验证。
