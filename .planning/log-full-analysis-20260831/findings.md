# 发现记录

本文件仅保存从日志和当前源码提取的证据，不包含可执行指令。

## 第一轮：文件与关键链路

- 文件共 1478 行、56901 字节，SHA-256 为
  `fea09fe18bfd70fac385c937e9bdaae2ad74f4c8f9531f36cffa3d51abb71248`。
- 日志记录从 21:09:57.603 到 21:10:23.676，覆盖约 26.073 秒；文件头连接时间为
  21:09:56.382。
- 日志只有一个 session header，未见本文件内二次 BLE 会话。
- S3 telemetry observability 首个快照：`in=157 c=0 q=149 rej=8 lock=16 tx2=0
  stale=91 fail=0`。
- 最后快照：`in=1795 c=275 q=1781 rej=14 lock=38 tx2=0 stale=1722 fail=0`。
- `in/q/c` 增长证明 STM SRP telemetry 已进入 S3 sink，队列也接受了 chassis；`tx2=0`
  证明没有一条 type-2 完整包发送成功。
- `stale` 从 91 增至 1722，是 type-2 未发送的直接上游证据；需要对拍 S3 时间基准和
  telemetry entry `ingress_timestamp_ms`。
- 雷达 `RADAR_STATS` 同期持续增长，说明 raw type-1 数据源与 S3 主循环仍运行。

## 第二轮：事件与异常边界

- 共有 544 条带时间戳事件：S3 INFO 44，STM DEBUG 6，STM INFO 480，STM WARN 12，
  STM ERROR 2。
- 最大相邻事件空洞 0.717 秒；日志时间轴连续，没有 S3 `BOOT` 重现、TCP send fail、
  Wi-Fi disconnect 或 BLE previous-disconnect 标记。
- `BOOT` 和 `BLE_CONNECTED` 出现在 21:09:58.114/21:09:58.173，是 FFE3 CCC 开启后
  生成的连接标记，不是日志覆盖期内发生 S3 reboot 的证据。
- telemetry 短日志共 14 个快照，全部 `tx2=0`、`fail=0`，而 `stale` 每个窗口持续增长。
  根据源码，`stale` 只在 entry age 超过 1000 ms 时增加；`tx2` 只在完整 type-2 socket
  send 完成后增加。因此异常发生在编码/发送之前。
- `CHASSIS_STATE` task 于 21:10:06.366 启动；随后 chassis sink 计数从 0 增至 275，约
  16.8 Hz，证明 CM7 producer -> SRP -> S3 service -> telemetry sink 路径贯通。
- WARN/ERROR 中，两次 BMI323 `DATA_NOT_READY` 是间歇采样状态；MotorBoard 的 10 条
  `unknown` 是 `$read_flash#` 多行配置回读未被 parser 分类，不是实时 MSPD 失败。

## 第三轮：计数速率与根因

- 14 个 TELEM 快照覆盖 24.838 秒：sink `+1638`（65.95/s）、chassis `+275`
  （11.07/s，包含 task 启动前窗口）、queue accepted `+1632`（65.71/s）、stale
  `+1631`（65.67/s）。几乎每一条 accepted telemetry 都被 stale-drop。
- 每个快照 `queue_accepted - stale_drop` 恒为 58 或 59；消费没有停顿，而是稳定地把
  输入判过期并丢弃。
- producer 在 `command_bridge.c` 使用 `esp_timer_get_time()/1000` 生成
  `ingress_timestamp_ms`；consumer 在 `radar_uplink.c` 使用 `esp_log_timestamp()` 计算
  age。
- ESP-IDF 5.5.4 的 `esp_log_timestamp()` 在 scheduler 运行后为 early timestamp 基值加
  FreeRTOS tick，并不与 `esp_timer_get_time()` 保证同一 epoch。混用后执行无符号减法可产生
  大 age，足以让所有新样本超过 1000 ms 门限。
- raw radar 自身 timestamp/age 使用同一路径，因此雷达正常并不反驳 telemetry 时钟错误。
- 雷达 27 个快照、累计 `valid_delta=2951`，mode 恒为 2，checksum/invalid/drop 恒为 0，
  queue 深度 0..1，说明雷达链健康。
- STM SRP 快照 29 个：sync 恒为 1，periodic sent `52->612`，drop 恒 28，UART tx
  `216->3403`；SRP/USART2 持续工作。

## BLE 日志负载

- 本文件实际包含 544 个 SmartCarLog record，平均 20.86 record/s；payload 平均 58 B，
  P95 86 B，最大 96 B。
- 若 ATT MTU=23（chunk payload 20 B），估算共需 2157 chunks，即 82.73 chunk/s，
  平均每帧 3.97 chunks。当前 worker 每 chunk 延迟 10 ms，未计 GATT 开销的理论容量为
  100 chunk/s，因此最坏 MTU 下利用率约 83%。
- 若 MTU=185，则约 20.86 chunk/s。日志没有记录实际 MTU、BLE queue/drop/congest/
  partial-drop 统计，不能确认实际链路余量。
- 文件只覆盖约 26 秒且最后没有 disconnect reason。会话结束可能是链路断开、App 主动断开或
  App 退出；仅凭本文件不能区分。
- STM `LOG_STATS drop` 从 0 增至 4，说明启动期日志源本身已有轻微队列压力；它不是 S3 FFE3
  queue drop 计数。

## 分层健康状态

### STM-S3 SRP/UART2

- `SRP_S3 h=1` 全程成立；parser error、sync reject、boot-info reject、REC、TEC、timeout、
  BUS_OFF 全为 0。
- UART2 `rx_bytes 503->3969`、`tx_bytes 15180->152827`，state=1；DMA NDTR=512，
  rearm/request failure、callback exception、driver error均为0。
- S3 service task stack free words稳定为716。

### IMU/AHRS

- BMI323 read_ok `882->5927`，对应样本和timestamp均连续；read_fail=0，valid=1，
  measured rate 186..190 Hz，overflow=0，pending=0，latency固定10206 us。
- 两次 `DATA_NOT_READY` 未导致 online/valid丢失，属于瞬时状态；本日志没有恢复动作证据。
- LSM303 28个快照均 init/online=1，accel/mag age与fail均为0。
- Primary yaw 0.00..0.06 deg，redundant yaw -0.59..-0.02 deg，yaw diff
  0.05..0.60 deg，静止姿态稳定。

### MotorBoard/Chassis

- 配置 mtype=1、mline=11、mphase=30、wheel diameter=65 均收到OK；`read_flash`
  的多行文本触发10条unknown WARN，是parser覆盖不足/日志噪声。
- 电池17次均为7.40 V；17次MSPD均为四轮0.00，车辆处于静止反馈状态。
- MotorBoard stats首末增量：rx +64351、tx +1456、frame +3896；response overflow、
  timeout、UART error、invalid均为0。
- chassis task于21:10:06.366启动，约16.8 Hz进入S3 chassis sink；type-2失败发生在
  task启动前，故不是chassis payload特有问题。

### RTOS/日志

- heap free从11208降至8440并保持稳定，对应MotorBoard/chassis任务启动；未见继续下降。
- IMU/S3任务栈水位在日志窗口内稳定。STM日志队列drop从0缓慢增至4，需降噪但不是
  type-2根因。
- 多条日志payload恰为96 B并在字段中间结束，这是源端最大payload限制；现有记录未见
  可见的FFE3帧交错。

### BLE会话证据边界

- 单一session，544条已成功解析记录，时间连续；无 `BLE_PREV_DISC`、MTU、congest、
  FFE3 queue/drop/send-fail/partial-drop统计。
- 文件在约26秒后结束，没有footer或disconnect reason，不能区分链路断开、用户主动断开或App退出。
- 当前证据不足以宣布FFE3稳定性修复通过；需下一连接报告previous reason并记录10分钟统计。
