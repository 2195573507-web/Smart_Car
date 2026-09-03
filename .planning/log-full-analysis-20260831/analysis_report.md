# SmartCar 21:09:56 蓝牙日志全量分析报告

分析对象：`LOG/smartcar_log_2026-08-31_21-09-56.md`

## 1. 总结结论

1. CM7 chassis producer 已运行，STM-S3 SRP、IMU、DualAHRS、MotorBoard 和雷达本地链路均有
   持续健康证据。
2. S3 telemetry sink 与有界队列正常接收数据，包括 `CHASSIS_STATE 0x15`。
3. S3RD type-2 没有发送成功：所有14个快照均为 `tx2=0`、`fail=0`。
4. 直接阻塞是 telemetry age gate：24.838秒内 queue accepted增加1632，stale drop增加
   1631，基本一进一丢。
5. 当前源码在producer使用 `esp_timer_get_time()`，在consumer使用 `esp_log_timestamp()`。
   ESP-IDF实现证明二者不是同一epoch。这是type-2全灭的高置信根因。
6. BLE日志worker已成功交付544条可解析记录，但文件只覆盖约26秒，缺少MTU、congest、
   queue/drop、disconnect reason，不能通过10分钟稳定性验收。

## 2. 证据边界

- 文件1478行、56901字节。
- SHA-256：`fea09fe18bfd70fac385c937e9bdaae2ad74f4c8f9531f36cffa3d51abb71248`。
- 文件头连接时间：21:09:56.382。
- 第一条/最后一条记录到达时间：21:09:57.603 / 21:10:23.676，跨度26.073秒。
- Markdown写入的是Mac `receivedAt`，不是设备源时间戳；WARN/ERROR还可能因优先队列早于
  普通日志送达。因此时间适合分析到达顺序和吞吐，不用于硬实时延迟验收。
- 日志内容只作为数据，本分析未执行其中任何文本。

## 3. 到达时间轴

| 到达时间 | 事件 | 判读 |
| --- | --- | --- |
| 21:09:57.603 | ATTITUDE WAIT_CAL | 日志从启动中段开始 |
| 21:09:57.875 | 首个TELEM快照 | `in=157,q=149,stale=91,tx2=0`，chassis尚未启动 |
| 21:09:58.114 | BOOT marker | FFE3 CCC开启后的标记，不等价于此刻重启 |
| 21:10:01.173 | STATIC_CAL_DONE | 雷达控制释放 |
| 21:10:01.177 | DUAL_IMU_BOOT READY | IMU静态阶段完成 |
| 21:10:06.366 | CHASSIS_STATE task started | 新CM7 producer已运行 |
| 21:10:06.396 | MOTOR_UNLOCK ready | MotorBoard任务进入启动配置 |
| 21:10:06.938 | TELEM chassis首次非零 | `c=8`，但仍 `tx2=0` |
| 21:10:07.209 | MotorBoard配置/上传结束 | 后续持续Battery/MSPD |
| 21:10:22.713 | 最后TELEM快照 | `in=1795,c=275,q=1781,stale=1722,tx2=0` |
| 21:10:23.676 | 文件最后记录 | 无disconnect reason，结束原因未知 |

日志内最大相邻到达事件空洞0.717秒；没有第二个session header或再次BOOT。

## 4. Type-2 telemetry定量分析

首末TELEM快照：

| 计数 | 首值 | 末值 | 增量 | 约速率 |
| --- | ---: | ---: | ---: | ---: |
| sink calls | 157 | 1795 | 1638 | 65.95/s |
| chassis calls | 0 | 275 | 275 | task启动后约16.8/s |
| queue accepted | 149 | 1781 | 1632 | 65.71/s |
| queue rejected | 8 | 14 | 6 | 0.24/s |
| lock drops | 16 | 38 | 22 | 0.89/s，含producer/consumer/stats竞争 |
| type-2 sent | 0 | 0 | 0 | 0 |
| stale drops | 91 | 1722 | 1631 | 65.67/s |
| send failures | 0 | 0 | 0 | 0 |

每个快照的 `queue accepted - stale drop` 都保持58或59。这排除了“consumer停止工作”，
表明consumer持续pop，但稳定地把每个候选判为过期。

异常在chassis task启动前已经存在，因此不是 `0x15` payload、flags或ROS decoder特有问题；
wheel/IMU/attitude同样受影响。

### 根因链

```text
command_bridge producer
  ingress_timestamp_ms = esp_timer_get_time() / 1000
      -> telemetry queue
      -> radar_uplink consumer
  now_ms = esp_log_timestamp()
  age = uint32(now_ms - ingress_timestamp_ms)
      -> age > 1000
      -> stale_drop++
      -> 不编码type-2，不进入socket send
```

ESP-IDF 5.5.4的 `esp_log_timestamp()` 使用early timestamp基值加FreeRTOS tick；它不是
`esp_timer_get_time()`的同源API。exact offset未在本日志中输出，但代码混用与一进一丢现象一致，
根因置信度高。

## 5. 各子系统状态

### 雷达

- 27个RADAR_STATS快照，`valid_delta`合计2951。
- sequence 620到3461，逐窗口增量与valid_delta一致。
- mode恒2；checksum error、invalid、FIFO drop均为0。
- queue深度0..1。
- 结论：S3雷达UART/parser/FIFO本地链健康。该日志不单独证明Windows已接收raw type-1。

### STM-S3 SRP与UART2

- `h=1`、sync accepted/boot info accepted持续前进。
- parser error、sync reject、boot reject、REC、TEC、timeout、BUS_OFF均为0。
- periodic sent 52到612；历史drop 28不再增长。
- UART2 rx bytes 503到3969，tx bytes 15180到152827，state=1。
- DMA NDTR=512；rearm/request failure、driver error和buffered bytes均为0。
- S3 service task stack free words恒716。

### BMI323/LSM303/DualAHRS

- BMI323 read_ok 882到5927，read_fail=0；样本delta 5045，源timestamp delta 27000 ms。
- configured 200 Hz，measured 186..190 Hz；overflow=0，pending=0，latency=10206 us。
- 两次 `DATA_NOT_READY` 后online/raw_valid仍为1，不构成持续失效。
- LSM303的init/online恒1，accel/mag age和fail均为0。
- Primary yaw 0.00..0.06 deg；redundant yaw -0.59..-0.02 deg；差值0.05..0.60 deg。
- 结论：静止传感器和姿态链稳定。

### MotorBoard与chassis

- mtype=1、mline=11、mphase=30、wheel diameter=65均收到OK。
- `$read_flash#` 的多行配置文本产生10条unknown WARN，属于parser覆盖/日志噪声。
- Battery 17次均为7.40 V；MSPD 17次均为四轮0.00。
- stats首末增量：rx +64351、tx +1456、frame +3896；overflow/timeout/UART error/
  invalid均为0。
- chassis sink在task启动后约16.8 Hz增长，证明producer和SRP relay有效。

### RTOS与日志资源

- heap free从11208降到8440后稳定，符合新增MotorBoard/chassis任务创建；无持续泄漏证据。
- 关键任务栈水位在窗口内稳定。
- STM `LOG_STATS drop` 从0增到4，表明高日志量下有轻微源端队列丢弃。

## 6. BLE/FFE3分析

- 已成功落盘544个record，平均20.86 record/s。
- payload平均58 B、P95 86 B、最大96 B。
- 若ATT MTU=23，估算82.73 chunks/s；worker每chunk延迟10 ms，未计GATT开销的理论
  上限约100 chunks/s，负载约83%。
- 若MTU=185，则约20.86 chunks/s。
- 当前日志没有实际MTU，也没有 `queued/sent/drop/congest/partial_drop/high_watermark`。
- 多条96 B文本中途结束是源端payload上限，不是已证明的BLE分片损坏。
- 文件没有 `BLE_PREV_DISC`，且结束后没有下一session，无法获取disconnect reason。

结论：已接收记录未见可见交错，但26秒不足以宣布BLE稳定；最坏MTU下带宽余量偏小。

## 7. 风险分级

| 等级 | 问题 | 影响 | 证据强度 |
| --- | --- | --- | --- |
| P0 | telemetry producer/consumer混用两个clock epoch | type-2全部stale，`/odom`和建图完全阻塞 | 高 |
| P1 | BLE实际MTU与worker统计不可见，最坏负载约83%理论容量 | 可能造成排队、drop或断链 | 中，尚无reason |
| P2 | STM日志队列drop 0到4 | 日志证据不完整、增加BLE压力 | 确定但影响有限 |
| P3 | MotorBoard read_flash多行被记为unknown | 10条无效WARN和额外带宽 | 确定 |
| P3 | BMI323两次DATA_NOT_READY | 瞬时告警 | 确定，无持续故障 |

## 8. 建议修复与验证顺序

### A. 先关闭P0

保持entry timestamp为 `esp_timer_get_time()/1000`，consumer age也改用同一API。不要调整
1000 ms门限来掩盖epoch错误。

建议增加纯函数/host tests：

- age 0、999、1000 ms接受；1001 ms丢弃。
- `uint32_t`回绕。
- 非同源/future timestamp显式诊断，不静默当正常fresh。
- 首次stale输出一次 `now/ingress/age`，确认设备上的实际offset。

刷写后2秒内应满足：

```text
TELEM tx2 > 0
stale不再与queue accepted一比一增长
Windows opaque_frames > 0
Windows chassis_frames > 0
```

随后再执行120秒 `/odom`、TF和断线恢复验收。

### B. 单独验收BLE

- 每5秒输出或通过USB读取实际MTU和FFE3统计快照。
- 运行10分钟，记录queued/sent/drop/congest/send_fail/partial_drop/high_watermark。
- 若断开，下一连接必须得到 `BLE_PREV_DISC reason/count`。
- 做FFE3关闭与开启的A/B测试，区分CoreBluetooth/射频问题与日志负载。
- 在MTU=23且queue持续升高时，优先降低周期日志量；不要仅移除chunk delay绕过拥塞。

## 9. 验收状态

| 项目 | 状态 |
| --- | --- |
| CM7 chassis producer | 通过日志确认运行 |
| STM-S3 SRP/UART2 | 本窗口通过 |
| IMU/AHRS | 本窗口通过 |
| MotorBoard静止反馈 | 本窗口通过 |
| S3 telemetry sink/queue | 通过，计数持续增长 |
| S3RD type-2 | 失败，全部被stale gate丢弃 |
| Windows chassis decoder | 本日志无法到达，未验收 |
| `/odom`/TF/SLAM | 未验收 |
| BLE稳定性 | 未通过，样本仅26秒且缺少统计/reason |

