# STM32H757 / ESP32-S3 当前工作树深化审计：第 28-37 轮

> 审计日期：2026-08-31（Asia/Shanghai）
> 起始快照：分支 `codex/s3-stm-cn-comments`，HEAD `f703453727a136d15ff7cacea4530beab6e9c08a`，工作树 147 项
> 证据级别：当前脏源码、CMake/Kconfig、锁定依赖和主机消费者静态审计
> 边界：仅修改审计 Markdown；未构建、测试、烧录、抓包、连接目标板或运行车辆。

## 1. 审计规则

- 第 28-37 轮只记录新的 ABI/并发/时间/错误传播/内存/ISR/安全/UI/测试证据，或对前 27 轮结论的独立关闭/纠偏。
- 不将 packed host struct 当作 wire，不将 CRC 当作认证，不将 App 调用计数当作物理交付。
- 每个修复建议包含位置、原因、最小内容、潜在影响和验证，但本轮不实施。

## 2. 第 28 轮：ABI、字节序、alignment 与序列化

### `R28-APP-TYPE-001` - Medium - S3 已转发的 App telemetry type 与 Swift 注册表不同步

**确定事实（CONFIRMED_SOURCE）**

- S3 App 类型表在 `app_parser.h:31,34` 定义 `CHASSIS_STATE=0x29`、`WHEEL_CONTROL_STATUS=0x2C`，`command_bridge.c:591-623` 会把对应 SRP 遥测编成这两类 V1 帧。
- macOS `SmartCarProtocol.FrameType` 不包含 `0x29/0x2C`；`DecodedMessage.init` 在 switch 前要求 `FrameType(rawValue:)` 成功，否则抛 `unsupportedType`。
- App 的 `DecodedMessage` 也没有 chassis state/wheel control status case。

**影响与边界**

- S3 可成功 notify，但 App 将两类帧计为解码失败，不会更新 UI/状态。这是遥测可观测性缺陷，不直接改变 STM 最终运动权。

**建议（本轮未实施）**

- 位置：S3 `app_parser.h/command_bridge`、Swift `FrameType/DecodedMessage/TelemetryStore`和 App 协议文档。
- 原因：类型注册表必须是双端联合合同。
- 内容：从单一 schema 生成/验证 App type 表，补齐 0x29/0x2C decoder，或在应用未支持时明确禁止 S3 转发并计数。
- 潜在影响：增加 App model/UI 字段，需冻结 schema、单位和 freshness。
- 验证：两类 golden frame 在 S3 和 Swift 逐字节一致，App 能解码有效帧并拒绝错 schema/非有限值。

### `R28-PACK-002` - Medium - 非 wire 逻辑对象被 `#pragma pack(4)` 降低 64-bit host 指针自然对齐

**确定事实（CONFIRMED_SOURCE）**

- `srp_def.h:84-111` 的 `#pragma pack(push,4)` 同时覆盖 wire header/trailer 和含 payload 指针的逻辑 `srp_frame_t`，尽管注释声称后者保持自然对齐。
- `srp_link.h:88-121` 将 config/pending/link 整体 pack(4)，其中含多个函数指针和 `void *`。
- 当前 CM7/S3 是 32-bit 指针；ROS2 当前只使用 codec 逻辑帧，未在共享内存或线缆上发送这些 struct。

**推断风险（INFERRED_RISK）**

- 将这些类型放入其他 pack(4) 容器/数组或移植到严格 64-bit 对齐架构时，数据/函数指针可只有 4 B 对齐。当前 x86_64 host 往往容忍，不等于合同正确。

**建议与验证**

- 位置：`srp_def.h/srp_link.h`。原因：packing 只应限定 wire description，不应污染内存内逻辑对象。
- 内容：在 wire struct 后立即 `pack(pop)`，logical/link struct 使用自然对齐；用 static assert 锁定 wire size/offset，不锁死 host pointer struct sizeof。
- 潜在影响：`srp_link_t` RAM 大小/对齐可变，但不改线缆字节。
- 验证：32-bit M7/XTensa 和 64-bit host 编译 `_Alignof/offsetof`，codec golden bytes 不变，UBSan alignment 无报错。

### `R28-S3RD-003` - Medium - S3RD ABI/CRC 在 S3 C 与 ROS2 C++ 中重复手写

- S3 定义 `RADAR_UPLINK_HEADER_SIZE=26`、magic/offset/CRC16-MODBUS；ROS2 `framing.cpp` 独立定义 `kHeaderBytes=26`、`kMagic` 和另一份 CRC 实现。
- ROS2 测试又自行组帧/实现 CRC，未直接消费 S3 encoder 生成的共享 golden corpus。
- 建议位置：S3RD schema 和 host tests。原因：实验协议已出现三份常量/CRC 真值。内容：生成版本化 schema 头/共享 golden binary corpus，两端同时测试。
- 潜在影响：需调整 ROS2 build 输入；未冻结前不改当前字节。验证所有 offset、长度、CRC、回绕、未知 type/flags 向量。

### `R28-FLOAT-004` - Low/Medium - binary32 宽度有断言，但 IEEE-754 表示合同未完整锁定

- `srp_wire.c:6` 已 `_Static_assert(sizeof(float)==sizeof(uint32_t))`，float 通过 `memcpy` 到 u32 再显式小端写入，避免 strict-aliasing/未对齐读写。
- 尺寸为 4 不单独证明 IEC 60559 binary32；当前 M7 GCC、Xtensa GCC 和 x86_64 host 实际都使用 binary32，所以它是可移植性合同缺口，不是当前观测到的数值错误。
- 建议增加 `FLT_RADIX==2`/`FLT_MANT_DIG==24`/`FLT_MAX_EXP==128` 编译断言或在工具链合同明确 binary32。验证 `+0/-0`、有限边界和 NaN/Inf 拒绝由业务层一致处理。

### 第 28 轮已确认保护

- SRPv4 codec、payload helper、S3RD 和 Swift float payload 都按字节显式写入，没有直接发送 C/Swift struct 内存。
- SRP wire header/trailer 有 size/offset/alignment static assert，关键 payload 有 size assert；IMU cal/telemetry 明确区分 host struct sizeof 和 11/30 B wire payload。
- ROS2 telemetry decoder 先查 frame length/schema/reserved/priority/flags，再用 `srp_wire_*` 读非对齐 float/u32，并拒绝非有限值。

## 3. 第 29 轮：锁顺序、callback 重入与共享状态

### `R29-S3-OWNER-001` - High - BLE callback 直接写 service-owned motion struct，“只置事件”注释与实现不符

**确定事实（CONFIRMED_SOURCE）**

- `s_motion_inflight/s_motion_pending_target/s_motion_pending_scale/s_motion_tx_in_flight` 由 smartcar service task 和 SRP completion 路径管理，未宣告 volatile，无 mutex/critical section。
- Bluedroid disconnect callback `command_bridge_on_ble_disconnect():1143-1150` 在另一任务上下文直接清两个 pending struct 的 `valid`，然后置 volatile stop flag；注释却称 callback 只向 service task 发信号。
- service 可同时在 completion 中复制/清 pending 并启动下一命令。

**推断风险（INFERRED_RISK）**

- callback 与 service/completion 交错时可丢失新 pending、在 stop 后启动旧复制，或产生 C memory model 数据竞争。volatile stop flag 不会使整个 motion struct 变成原子。

**建议（本轮未实施）**

- 位置：S3 BLE disconnect callback 与 command bridge event loop。
- 原因：motion state 必须单 writer。
- 内容：callback 只往有界 control-event queue 写 `{DISCONNECT, connection_epoch}`；service 原子处理 queue/parser/session/pending/inflight/zero。
- 潜在影响：增加一个高优先级事件槽和最多一个 10 ms service tick 延迟。
- 验证：在 completion/pending overwrite/断连的每个写点强制调度交错，旧 epoch 不得再发，最后 motion 必为 zero。

### `R29-MB-CRITICAL-002` - High/Medium - MotorBoard 全局临界区内执行浮点 PID、ramp、`snprintf` 和 TX 入队

- `motor_board_update_pid():535-566` 从读目标开始持 `taskENTER_CRITICAL`，在其中对 4 轮执行 ramp/PID/限幅，并调 `MB_Protocol_SendPwm()`。
- `motor_board_force_stop():491-501` 也在临界区内重置 4 轮 PID/ramp 并发 zero。
- `MB_Protocol_SendPwm -> MB_Protocol_SendFour` 使用 `snprintf` 生成文本，再调用会再次进入 critical section 的 `MB_Transport_Send()`。

**推断风险**

- CM7 全局中断屏蔽时间包含浮点计算和 libc 格式化，会增加 SysTick、UART2 DMA/IDLE、USART6 RX/TX 和传感器 IRQ 延迟；正好可加剧已知链路/feedback 风险。

**建议与验证**

- 位置：MotorBoard target/PID publish 和 transport stop primitive。原因：临界区只保护最小快照/所有权转换。
- 内容：短临界区复制 target/stop generation，在区外计算/格式化，再用短临界区提交整帧；stop generation 在提交前后复核以防旧 PWM 后发。
- 潜在影响：需防止计算期间 stop/target 更新造成 TOCTOU。
- 验证：DWT/GPIO 测量最长关中断时间，在 PID 计算中注入 stop、UART burst 和 SysTick，确认 zero 不被旧 PWM 覆盖。

### `R29-LINK-REENTRANT-003` - Medium - shared link callback 不可重入合同仍被 S3 motion completion 违反

- `srp_link.h` 明确规定 completion/frame/bus-off callback 不得递归操作同一 link；S3 motion completion 会直接调 `start_motion_command()` 再进 `srp_link_send()`。
- 该路径当前在单 service owner 中运行，未已确认死锁，但会在 link receive 迭代期间改写 pending/sequence，使 shared API 实际重入语义与注释不一致。
- 建议 callback 只记 completion event，下一 service iteration 再 send。验证 pending 满、ACK/BUS_OFF/recover 与新 motion 交错。

### `R29-BLE-STATE-004` - Medium - BLE 多字段状态只用 volatile，通知分片无一致 connection snapshot

- Bluedroid callback 写 `connected/notify_enabled/log_notify_enabled/ready`、`conn_id`和 MTU；service/radar/log 任务直接读并分片发送，没有 mutex/epoch snapshot。
- 单个 bool/16-bit 读写往往是字长原子，但多字段不是同一时刻；分片中断连/重连可把旧 `conn_id` 与新 ready/MTU 混合。
- 建议单 BLE TX owner 使用 `{epoch,conn_id,mtu,ccc}` snapshot，每片提交前复核 epoch。验证分片中 disconnect/reconnect/MTU 更新。

### 第 29 轮关闭的死锁候选

- IMU boot manager 在调用 manager/calibration/transport 前通常先复制状态并释放 boot mutex；calibration 不在持锁时反向取 boot lock。
- IMU filter 的 median/IIR 在自己单 mutex 内执行，当前无反向锁顺序；radar FIFO/telemetry mutex 在 socket/BLE 发送前已释放。
- 因此本轮不报告“IMU/radar 已确认死锁”；仍需运行时 lock wait/WCET 量测。

## 4. 第 30 轮：时间、序号与回绕算术

### `R30-DWT-001` - Medium - DWT 64-bit 扩展只能观测一次 `CYCCNT` 回绕，长停顿会漏计时间

**确定事实（CONFIRMED_SOURCE）**

- `bsp_timer_get_us()` 保存上次 32-bit `DWT->CYCCNT`，仅当当前值小于上次值时累加一个 `2^32`。
- 480 MHz 时单次回绕约 8.95 s；两次调用间若跨过两次或更多回绕，算法无法恢复遗失的高位。
- 正常 BMI/IMU 任务以 200 Hz/高频读该时基，所以正常调度下有充足裕量。

**风险与建议**

- 调试器长时间 halt、超长关中断/高优先级饿死或任务长期不运行后，IMU 时间可少计多个 8.95 s，使 freshness/标定窗口/dt 失真。它不是正常 200 Hz 路径已观测故障。
- 位置：CM7 monotonic time service。原因：扩展计数必须不依赖业务调用频率。内容：用 64-bit hardware timer/周期性高优先级回绕服务，或将 HAL tick + 子 tick 组合并显式处理 debug halt。
- 潜在影响：时基切换会影响所有 freshness/dt，必须一次性迁移而不混用。验证跨 1/2/3 次 CYCCNT 回绕、debug halt 和时钟变更。

### `R30-V2-SEQ-002` - Low/Medium - V2 command sequence 使用普通 `<=`，不支持模 2^32 回绕

- S3 在 session 内以 `meta.sequence <= last_sequence` 判 stale；sequence 从 `0xffffffff` 回到 0/1 后，会把所有新命令视为 stale，直到新 HELLO/session。
- 当前 App 未使用 V2，即使 10 Hz 持续命令也需多年才回绕，因此是协议完整性缺口而非当前现场风险。
- 建议使用 half-range modulo comparison 或强制在序号高水位前重建 session，并测试 duplicate/out-of-order/wrap/session reset。

### `R30-PHASE-SENTINEL-003` - Low - boot phase timing 使用 0 作未设置哨兵，恰在 32-bit ms 回绕点有歧义

- `imu_boot_manager` 的 `phase_timing.start/end_timestamp` 和 `phase_end_time` 使用 32-bit `imu_time_now_ms()`，并以 0 判断 end 尚未设置。
- deadline 判断已用 `(int32_t)(now-deadline)>=0`，elapsed 使用无符号差，短超时本身回绕安全；仅当阶段恰好在约 49.7 天回绕到 0 附近进入/结束时，诊断哨兵有歧义。
- 建议使用显式 `valid` 位或 64-bit 内部时间，验证 start/end 跨 0 时的状态与进度遥测。

### 第 30 轮已确认保护

- CM7 S3 timeout、status 周期、MotorBoard deadline 和 IMU boot deadline 主路径使用无符号差或 half-range `time_reached/tick_due`，短周期跨 32-bit 回绕安全。
- calibration 窗口使用 64-bit us；S3 App/session 使用 64-bit `esp_timer_get_time`；ROS2 使用 steady clock 判本地 age。
- ROS2 `SequenceTracker` 已按模 2^32 half-range 区分 forward/out-of-order/wrap，新 connection epoch 会 reset，不应重复报错。

## 5. 第 31 轮：返回值、ACK 关联与错误传播

### `R31-CONTEXT-001` - High - PID/SYS_CONFIG 多事务共用可变 callback context，ACK 可归属错 App 命令

**确定事实（CONFIRMED_SOURCE）**

- S3 对 PID 命令使用单一全局 `s_pid_tx_context`，每次发送前覆写其 app type/version/session/sequence，并把同一指针保存到 SRP pending callback。
- SYS_CONFIG 同样共用单一 `s_baud_tx_context`，还共用 `s_baud_change_value`。
- `srp_link` 有 4 个 pending slot，并不拒绝同一 type 的多个序号同时在途。

**推断风险（INFERRED_RISK）**

- 两条 PID/SYS_CONFIG 在第一条 ACK 前到达时，第一个 callback 会使用第二条的 App sequence/session 发 ACK。波特率路径还可在第一条成功 ACK 时调度切换当前全局最新值，而非该事务的值。

**建议（本轮未实施）**

- 位置：S3 PID/SYS_CONFIG transaction owner 和 shared pending context。
- 原因：callback context 必须与每个 pending 事务一对一且不可变。
- 内容：每个事务槽内嵌 app metadata 和 baud value，或显式限制同类只有一个 in-flight；callback 按自己槽位完成/切换。
- 潜在影响：需要更改 pending/context RAM 布局和 busy 结果。
- 验证：两条同类命令乱序 ACK/超时/远端拒绝，每个 App sequence 只收到自己的结果，波特率只在对应 commit 后切换。

### `R31-COALESCE-002` - Medium/High - motion pending overwrite 丢失被取代命令的完成结果

- 已有 motion in-flight 时，后续 target 和 master-scale 分别只保留最新一条；新命令直接 struct 赋值覆盖旧 pending 并返回 true。
- 被覆盖的 V2 App sequence 不会收到 `SUPERSEDED/BUSY/REJECTED`；重发时因 session `last_sequence` 已前进，可只得到 stale-sequence。V1 ACK 无 sequence，更无法区分。
- 建议覆盖前对旧命令发明确 `SUPERSEDED` 结果，或使用有界 FIFO/单一 latest-intent 合同并让 App 知道只保留最新值。
- 潜在影响：ACK 通量增加，必须保证 stop 不被普通 target 合并。验证 target/scale/stop burst 和乱序 ACK。

### `R31-V1-ACK-003` - Medium - V1 ACK 只含 type/result，App 只处理 PID ACK

- S3 V1 ACK payload 仅 `{acknowledged_type,result}`，无 command sequence/stage/session。同类快速命令无法精确关联。
- `SmartCarViewModel` 只在 ACK type 是 PID 时更新 apply status；wheel/radar/control/stop 拒绝不影响控制 UI。
- 建议位置：App V2 迁移/ACK state machine。原因：操作者需要区分 BLE write、gateway admitted、STM accepted 和 actuator applied。内容：使用 V2 sequence/stage 或为 V1 定义不可混淆的单 in-flight 规则。
- 影响：UI 增加 pending/accepted/rejected/expired 状态；验证所有可操作命令的成功/拒绝/超时。

### `R31-ACK-DROP-004` - Medium - CM7 fast response 发送失败被业务 handler 静默忽略

- `s3_service_send_response_locked()` 无返回值，多个 command handler 在执行或拒绝业务后调用它，不知道 ACK/ERROR 是否进 UART。
- ACK 丢失将触发发送端重试；第 19 轮已确认接收端无 replay cache，因此错误传播失败会放大重复副作用。
- 建议返回/计数 fast-response TX 结果，副作用命令仅在 replay cache 记录后执行，ACK 可重发而业务不重做。

### `R31-DIAG-005` - Medium - 日志 API 返回 OK 不代表 SRP LOG 已交付

- `uart_log_write_usart2()` 忽略 `s3_service_send_log()` 结果，`bsp_uart_log_write_link_level()` 对所有合法输入返回 `BSP_STATUS_OK`。
- logger 任务和 RTOS health 输出因此无法从返回值区分“已编码”、“链路未同步/锁超时”和“UART 已发”。
- 建议区分 accepted/dropped/not-synced/transport-failed 并在日志统计中暴露；不应让日志失败阻塞控制，但也不应伪报 OK。

### 第 31 轮已确认边界

- 对轮速/底盘/航向命令，S3 motion pipeline 已将一个 in-flight 与 latest pending 分开，zero 会取消普通 motion pending；问题是覆盖结果/跨线程 owner、不是无任何调度结构。
- 遥测/日志本来允许 best-effort，可以忽略单帧失败，但必须有真实 drop/result 计数，不能与必达控制 ACK 使用相同“成功”语义。

## 6. 第 32 轮：缓冲区、生命周期与复杂度上界

### `R32-ROS-COMPLEXITY-001` - Medium/High - 未认证 TCP 无 magic 流可触发 `vector.erase(begin)` 二次复杂度

- `TcpChunkAssembler` 缓冲使用 `std::vector<uint8_t>`；extractor 对 magic 错误每次返回 `consumed=1`，feed loop 每次从 `buffer_.begin()` erase 一个字节。
- 当缓冲中是长无 magic 输入时，每个字节都会移动剩余 vector，成本近似 O(n^2)。buffer 有 262144 B 上限，可防无界内存，但不防 CPU DoS。
- 当前 TCP 又无认证且单 client 占用，该路径是 live PoC 启用后的可达攻击面；默认 `unconfigured` 仍是保护。
- 建议位置：ROS2 TCP assembler/extractor。原因：重同步应线性扫描而不重复前删。内容：使用 read offset/ring/deque，一次搜索下一 magic，达阈值后统一 compact。
- 潜在影响：assembler 内部索引和计数语义变化。验证 262 KiB 无 magic/部分 magic/黏包，设 CPU 时间上界和计数一致性。

### `R32-ROS-LENGTH-002` - Medium - host raw payload 上限 65535 远大于 S3 发送合同

- S3 雷达最大帧是 `10 + 255*3 = 775 B`，S3RD 发送上限取 `max(775, SRP_MAX_FRAME_SIZE=512)`，即 775 B。
- ROS2 `S3ProtocolConfig.max_payload_bytes` 和 YAML `s3_max_payload_bytes` 默认 65535，extractor 会为该声明长度等待完整帧并复制 payload，之后才交 official decoder 拒绝不可能的 YDLIDAR 帧。
- 建议位置：ROS2 S3ProtocolConfig/YAML 和联合 schema。原因：receiver 应使用发送端真实上限。内容：raw max 固定/生成为 775，telemetry 继续绑 SRP max，不允许配置放大超过 schema。
- 潜在影响：未来更大雷达帧需要协议版本升级，不能只改 YAML。验证 775/776/65535 边界和半帧超时。

### `R32-APP-QUEUE-003` - Medium - App decode/log/disk 工作队列无 backlog 上限

- `BLEReceivePipeline` 和 `BLELogReceivePipeline` 对每个通知直接 `DispatchQueue.async`，无 pending count/drop/coalescing/backpressure。
- `SessionLogWriter.append()` 又对每条日志向独立串行 queue 追加 async 任务，文件每条同步写入无会话大小/时间轮转、磁盘配额或自动禁用。
- 高频 FFE2/FFE3 输入超过 decode/main/disk 速度时，GCD pending block 和文件可持续增长，而 UI 只看不到 backlog/drop。
- 建议位置：App receive pipelines/SessionLogWriter。原因：订阅端也必须有资源上界。内容：有界 item/byte queue、遥测 latest/coalesce、日志 FIFO/drop counter，文件按大小/时间转存并检查磁盘失败。
- 潜在影响：过载时将丢日志/合并遥测，必须显示 drop 而不静默。验证持续超速通知、慢磁盘/磁盘满和长时会话内存/文件上界。

### `R32-CM7-STACK-004` - Medium（调试配置）- 512-word IMU 任务可调用 768 B 局部日志块

- `imu_debug_task` 栈为 512 words（约 2048 B）；开启 `IMU_RUNTIME_ENABLE_RAW_DATA_LOG` 时调用 `imu_data_print()`，其局部 `char block[768]` 外还有多个快照 struct/函数调用。
- 该宏默认关闭，当前安全默认不触发该局部块；打开后无静态 stack budget 断言，只能依赖 HWM/溢出 hook。
- 建议将原始日志 block 改为分段有界输出或调试专用静态 scratch，并为开/关两种构建记录静态 stack-usage 和目标板 HWM。

### 第 32 轮已确认保护

- ROS2 assembler 总 buffer 有上界，ready frame 数有上界；S3 radar raw FIFO/telemetry queue 均有明确 PSRAM 容量和失败返回。
- Swift `SmartCarLogParser` 缓冲限 2048 B 并用 read index 分批 compact，`DeviceLogStore` UI ring 限 500 条，telemetry/history 也有容量上限。
- SRP/S3 App parser 使用固定最大帧，不保留输入 payload 指针跨 callback。

## 7. 第 33 轮：ISR、临界区、DMA/cache 与最坏延迟

### `R33-UART-ISR-001` - Medium/High - USART2 RX ISR 的最坏执行量包含 512 B copy、cache 与 HAL 重装

**确定事实（CONFIRMED_SOURCE）**

- `UART_LINK_RX_DMA_SIZE=512`，Rx event callback 在 ISR 中先对 DMA 区做 D-cache invalidate，再在 FreeRTOS ISR critical section 内逐字节复制整块到 2048 B ring、更新统计。
- 同一 callback 随后直接调用 `HAL_UARTEx_ReceiveToIdle_DMA()` 重装 normal-mode DMA、通知任务、读取 DMA/DMAMUX/GPIO 状态快照；error callback 也可在 ISR 中清错误并尝试相同重装。
- USART2/DMA1 Stream0 与 USART6 都配置为 NVIC priority 5，恰好等于 `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY=5`。因此当前 `vTaskNotifyGiveFromISR()`/FreeRTOS ISR critical API 的优先级用法一致，但同级 IRQ 不能互相抢占。

**推断风险（INFERRED_RISK）**

- 512 B 满块、ring 满时的逐字节 modulo/drop、cache 操作和 HAL re-arm 会延长 priority-5 ISR；它可推迟同级 MotorBoard USART6 搬运。第 29 轮确认的 MotorBoard 长 task critical section又会屏蔽 priority 5 及更低 IRQ，二者形成可叠加的最坏延迟。源码没有 ISR cycle/WCET 或 DMA inactive gap 上界，因此不能静态证明 921600 UART2 与 MotorBoard RX 在峰值时无丢失。

**建议（本轮未实施）**

- 位置：CM7 `uart_link` Rx event/error callback、MotorBoard 临界区和 IRQ 观测层。
- 原因：ISR 应只完成有界 ownership 转换，copy/cache/re-arm 的最坏时间必须量化。
- 内容：优先用 ping-pong DMA/固定 block descriptor 将 bulk copy 与诊断快照移到任务；若 HAL re-arm 必须留在 ISR，锁定 HAL 合同并为 callback cycle、re-arm gap、ring waterline 和同级 IRQ latency 计数。缩短 MotorBoard task critical section时用 stop generation 防旧 PWM 后发。
- 潜在影响：增加 DMA buffer/descriptor RAM 和任务唤醒；ownership/cache 边界错误会造成更严重数据损坏，必须分阶段验证。
- 验证：DWT/GPIO 同时测 0/短/512 B callback、ring 满、UART error、PID+stop 并发下的最长 ISR/关中断时间；以 UART2 921600 burst 和 USART6 feedback 同时压测，检查丢字节、re-arm failure、MotorBoard feedback age 与最终 stop 延迟。

### 第 33 轮已确认保护与边界

- DMA buffer 32 B 对齐并位于 `.dma_buffer`，ISR copy 前按 cache line invalidate；DMA request、normal mode 和 priority 均显式配置。
- USART6 ISR 只清硬件错误、搬运 RX/TX 字节，不解析协议或写日志；但 RX `while` drain 没有显式每次字节预算，仍需实测连续输入下的 ISR 上界。
- 上述是静态时延风险，不是已确认丢帧；本轮没有逻辑分析仪/DWT trace/目标板统计。

### `R33-BLE-SELFQUEUE-002` - High（条件性）- FFE3 CCC callback 可用无限等待回投并填满自身 BTC queue

- FFE3 CCC write 在 Bluedroid GATT callback 中同步调用 `s3_ble_log_flush_pending()`；它可遍历 48 条 pending log。单条最大 108 B，默认 MTU 23 时会拆成最多 6 个 notification chunk。
- IDF 5.5.4 在 BTC task 上同步调用项目 GATT callback；每个 `esp_ble_gatts_send_indicate()` 又通过 `btc_transfer_context()` 投递同一个 BTC work queue。queue0 默认容量 100，post timeout 为 `OSI_THREAD_MAX_TIMEOUT`，满队列时进入 `portMAX_DELAY`。
- 因 callback 返回前 BTC task 无法消费自己新投递的 work item，足够大的启动日志积压可形成 self-queue deadlock；断连/拥塞 callback 也会随 BTC task 一起停滞。条件是首次开启 FFE3 CCC 时 pending/chunk 数足以填满 queue，当前未在设备复现。
- 建议位置：S3 BLE log pending/CCC handler。原因：stack callback 不得同步生产无界数量的同队列工作。内容：CCC callback 只唤醒独立 BLE TX owner；按有界 burst、congestion/completion、connection epoch 发送，禁止在 BTC callback 中 flush。
- 潜在影响：日志排空延迟增加并可能显式丢低优先级日志；必须为 stop/断连保留独立高优先级路径。验证 MTU 23、48 条满积压、queue 拥塞、分片中断连，BTC task 必须持续前进且 disconnect stop 可处理。

### `R33-UART-TASK-003` - Medium - UART service 每 1 ms 可在全局临界区复制 128 B

- `uart_link_read()` 在 task critical section 内逐字节复制调用方容量；S3 service scratch 为 128 B，并以 1 ms 周期调用。该 critical 会屏蔽 priority 5 的 UART2/USART6 及更低 IRQ。
- 单次 128 B、2048 B ring 和单生产者/消费者使工作量有界，但它会叠加 MotorBoard PID critical 与 UART2 ISR WCET。建议先测 BASEPRI 持续时间；若超预算，用短临界区提交 head/tail/count snapshot 或经验证的 SPSC ring，不可直接无协议去锁。
- 验证 UART2 921600 满线速叠加 USART6 burst，记录 critical pulse、ring overflow、feedback age 和控制周期抖动。

### `R33-RAWDIAG-004` - Medium/High（仅诊断配置）- raw USART1 输出按整段文本全局关 IRQ

- `SMARTCAR_RAW_DIAGNOSTICS` 默认 0 且声明只用于静止台架；开启后 `cm7_raw_diag.raw_write()` 保存 PRIMASK、全局关 IRQ、逐字节轮询 USART1 并等待 TC，每字节还有 20000 spins 上限。
- USART1 为 115200 时，几十字符的成功输出本身即可使全部 IRQ 关闭数毫秒；UART2 TX phase 可多次调用该诊断。风险是 UART2/USART6 丢字节、SysTick 抖动和控制周期破坏，但生产默认关闭。
- 建议 production profile 在构建层强制该宏为 0；诊断改用有界 ring + task TX。验证诊断开启时最长 PRIMASK、ORE/drop、1 ms task jitter，并禁止车辆运动。

### 第 33 轮明确关闭/保留

- 锁定 STM HAL 的 IDLE normal-mode、DMA TC 和 error-abort 路径均在调用项目 callback 前把 Rx/DMA state 置 READY/unlock；因此旧“callback 内 re-arm 必然因 HAL BUSY/内部锁失败”候选关闭。仍开放的是 callback copy/cache/HAL 调用的 WCET 和物理 RX inactive gap。
- CM7-only UART2 DMA buffer 当前 32 B 对齐、独占 `.dma_buffer`、链接到 DMA 可达 D2 SRAM，pre-arm/实际长度 invalidate 和 CMSIS barrier 完整；旧“当前 buffer 未对齐/不可达/缺 barrier”误报关闭。CM4 对 D2 物理别名覆盖仍是独立开放项。

## 8. 第 34 轮：秘密、固件安全、日志隐私与网络暴露

### `R34-SECRET-000` - Critical - 当前活动 Wi-Fi 凭据已存在于公开 GitHub、构建产物和受跟踪历史

**确定事实（CONFIRMED_REPOSITORY，不披露原值）**

- 活动 `ESPS3/main/radar/radar_wifi_credentials.h` 被 `.gitignore` 排除，去除注释后有 1 组有效 SSID/password 字面量。
- 只用内存集合/不可见原值比较确认：这 1 组与受跟踪的 `S3-radartest/archive/legacy_modules_20260610/ESPS3/components/Middlewares/wifi/wifi_credentials.h` 中一组完全相同；该 tracked 文件当前与 HEAD 一致。
- 另一个受跟踪的 `S3-radartest/分支项目/ESP1/ESPS3/components/Middlewares/gateway_config/gateway_wifi_credentials.h` 也含有效字面量。去注释集合比较确认这两份 tracked 文件合计至少 5 组唯一凭据对；本报告不记录任何 SSID、password 或其可逆表示。
- 两个 tracked 路径均由初始提交 `6de387a1134e6862a3deb86c86404d26b5161b24` 引入；该提交仍被当前分支、`main` 及多个本地/远端分支包含。
- GitHub 当前元数据确认 `2195573507-web/Smart_Car` 可见性为 `PUBLIC`，公开默认分支 `codex/dual-imu-lifecycle` 包含上述初始提交。三份 tracked `esp111_protocol_common.h` 还重复保存一组非占位 SoftAP SSID/password。
- 不输出内容的二进制匹配确认：活动 header 的 SSID 与 password 两个字段都已嵌入现有 `ESPS3/build/smartcar_s3_gateway.elf/.bin`。这些旧本机构建产物不是当前 release 证据，但属于 secret 扩散面。

**影响与立即处置**

- `.gitignore` 只阻止当前活动文件新增跟踪，不能撤销已经进入 HEAD、Git objects、远端分支、clone、CI cache 或备份的秘密。与活动值相同的凭据必须按“已泄露”处理，不能等待历史清理后再轮换。
- 立即在对应网络侧轮换/撤销所有受影响 Wi-Fi 与 SoftAP 凭据，检查未知客户端和访问日志；新凭据不得写入仓库、审计文档、issue、日志或命令输出。轮换属于外部运维动作，本轮没有代替用户执行。
- 历史清理需要单独授权、通知所有 clone/远端/CI 使用者并协调强制更新；本审计不删除文件、不 rewrite history、不 push。即使获批清理，轮换仍是必要动作。

**验证**

- 轮换后验证旧凭据全部拒绝，新凭据只通过受控 secret injection 使用；用 secret scanner 覆盖 HEAD、all refs、Git objects、构建产物、coredump/日志和 CI cache，报告只保留命中路径/secret ID，不保留原值。

### `R34-FW-TRUST-001` - High（发布门）- 当前 S3 配置未建立固件真实性与静态秘密保护

**确定事实（CONFIRMED_CONFIG）**

- 当前被忽略的 `ESPS3/sdkconfig` 明确为 `CONFIG_SECURE_BOOT is not set`、`CONFIG_SECURE_FLASH_ENC_ENABLED is not set`、`CONFIG_FLASH_ENCRYPTION_ENABLED is not set`、`CONFIG_NVS_ENCRYPTION is not set`；tracked `sdkconfig.defaults` 只固定 BLE、分区、flash/PSRAM 和日志等，没有锁定上述发布安全项。
- uplink 启用时 `radar_uplink.c` 直接 include 被 `.gitignore` 排除的 `radar_wifi_credentials.h`，再把 SSID/password 拷入 Wi-Fi STA config。报告未读取或复制该头的任何实际值。
- 当前分区含双 OTA app slot、NVS、SPIFFS 和 coredump；静态源码/配置没有提供“产线烧录已启用 eFuse secure boot/flash encryption”的证据。

**推断风险与边界**

- 若按当前配置生成并部署镜像，编译进固件的 Wi-Fi 凭据可被离线镜像/flash 读取，未验证签名的固件也可替换命令网关逻辑。这里确认的是构建配置缺口，不等同于已读取真实设备 eFuse 或证明当前车辆已部署该配置。

**建议（本轮未实施）**

- 位置：S3 release defaults、烧录/eFuse 流程、secret injection 和发布清单。
- 原因：网关持有网络凭据并转发运动命令，调试配置不能直接成为产品发布配置。
- 内容：分离 bench/release profile；release 强制 secure boot v2、flash encryption、签名/密钥轮换和不可回退策略；secret 只在受控构建注入，禁止进入日志/coredump，记录 eFuse attest 但不记录密钥。
- 潜在影响：eFuse 动作不可逆，影响 JTAG、量产、RMA、OTA 和灾难恢复；必须先在专用样机演练，不能在本审计中启用。
- 验证：clean release build 检查 config/镜像签名，专用样机验证未签名/回滚镜像拒绝、加密 flash 离线不可读、密钥轮换/RMA；设备证据与源码审计分开归档。

### `R34-BLE-AUTH-002` - High - SMP 被编译不代表 FFE1 命令特征已要求加密/授权

- 当前 sdkconfig 启用 Bluedroid BLE SMP 和 bond NVS，但项目没有 `esp_ble_gap_set_security_param()`、`esp_ble_set_encryption()` 或 peer allowlist/admission 调用。
- FFE1 RX value 使用普通 `ESP_GATT_PERM_WRITE`；FFE2/FFE3 与 CCC 也只用普通 READ/WRITE，不要求 encrypted/MITM permission。连接事件立即把 transport 标为 connected，V1 motion 又不要求 V2 session。
- 因此第 15/25 轮的“未配对 central 可尝试写 motion”保持开放。最小方案应先冻结产品配对/换机/RMA策略，再要求加密+MITM/bond/peer authorization，并让 command-ready 依赖认证完成；不能仅改一个 permission 常量而不处理 bond lifecycle。
- 验证未配对、Just Works/MITM、旧 bond、bond database reset、MAC randomization、重连和授权撤销；所有失败状态必须保持 zero，且诊断/恢复仍可用。

### `R34-APP-ID-003` - High - 官方 App 只按固定名称和公开 UUID 选择车辆

- App 扫描所有 peripheral，只以广播名 `SmartCar_S3` 选中设备；连接后只匹配公开 FFE0/FFE1/FFE2/FFE3 UUID。没有已批准 peripheral identifier、bond identity、设备证书或 challenge-response。
- 同名、同 UUID 外设可诱导 App 连接、接收操作者命令并伪造 ACK/遥测；反向也无法证明当前 central 连接的是目标车辆。固定 name/UUID 是筛选，不是身份认证。
- 建议首次配对明确显示并确认设备身份，持久化受信 bond/peripheral，并用设备持有私钥的认证握手绑定 session。影响是增加 onboarding/换板/RMA 流程；验证同时广播两个同名服务，App 必须拒绝未批准设备。

### `R34-PRIVACY-004` - Low/Medium - 运行日志输出完整 BLE 地址和 Wi-Fi SSID

- S3 BLE connect log 输出 remote BDA；radar uplink 的轮换/连接失败日志输出当前 credential 的 SSID。未发现打印 password 的项目代码。
- device log 又可经 BLE log characteristic 转发并进入 App session file，因此地址/SSID 会扩大到主机日志和问题归档。
- 建议默认对 peer 地址和 SSID 做稳定短哈希/尾部掩码，诊断构建按需显式开启原值；定义 session log/coredump 的保留期、权限和删除流程。验证正常/失败日志中不含 password、完整 BDA/SSID 或其他 secret。

### 第 34 轮已有保护

- radar uplink tracked Kconfig 默认关闭，credentials 文件被 Git 忽略；代码只记录 SSID、不记录 password，并检查字段长度。
- 这些保护降低源码泄密概率，但不能代替已编译镜像加密、TLS/对端认证或 BLE command authorization。live TCP 无认证/加密仍沿用第 26 轮发布阻断，不重复计为新问题。

## 9. 第 35 轮：App/UI 状态、反馈真实性与失联操作

### `R35-READY-001` - High - UI 用物理 BLE connected 代替 command/telemetry/safety ready

- `BLEManager.didConnect` 在 service/characteristic discovery 和 CCC 完成前即设 `.connected`；Control/Joystick/Wheel/PID 控件仅以该枚举启用，没有 command-ready、telemetry-ready、V2-session-ready 或 STM/MotorBoard admission 状态。
- service/characteristic/notify 失败虽会设置 `lastError`，却不撤销 `.connected`；操作者可看到 Connected 并操作控件，但 `sendFrame()` 因无 write characteristic 只返回 false/设置 notConnected。
- 建议位置：BLE capability state、ViewModel 和所有控制卡。原因：连接、可写、可收 ACK、STM accepted、actuator ready 是不同阶段。内容：显式能力/epoch 状态机，运动控件只在 authenticated command-ready + fresh safety status 时开放；zero/断开始终可达。
- 潜在影响：连接后控件会延迟启用，旧设备缺少 capability/ACK 时需显示受限兼容模式。验证 service/characteristic/CCC/认证/首 ACK 每阶段失败与重连。

### `R35-FEEDBACK-002` - High - 红色 STOP 与“已发送”计数不能证明命令交付/执行

- 控制页醒目的红色 STOP 调 `emergencyStop() -> send(.stop)`；该 V1 type 仍被 S3 拒绝，且 App 只对 PID ACK 更新 apply status。STOP 按钮本身不按连接状态禁用，也不显示 pending/rejected/expired/applied。
- wheel/control/radar 路径在调用 BLE send 前后直接递增 `transmittedFrameCount`；其中 `sendWheelSpeeds/sendControl` 丢弃 `sendFrame()` 的 Bool。`sendFrame()` 返回 true 也只表示 CoreBluetooth 已接受 `.withResponse` 调用。
- `BLEManager` 没有实现 `didWriteValueFor`，V1 ACK 又无 command sequence/stage。因此 UI 无法区分 local enqueue、GATT response、S3 admission、STM ACK 和 MotorBoard applied。
- 建议统一为有 sequence 的 command state：`queued -> gatt_acked -> gateway_admitted -> stm_accepted -> applied/expired/rejected`；急停按钮必须先清本地 timer/target并走有效零四轮/专用安全合同。超时和拒绝要持续可见，不能只闪现通用错误。
- 验证 write spy、didWrite error、S3 reject、STM timeout、被 supersede 和 feedback stale；每个 UI 状态只由对应证据推进，最终物理 stop 单独台架验收。

### `R35-PID-003` - Medium - PID sending 可重复提交且无 sequence/timeout

- PID 是唯一显示 ACK 结果的控制项，但 Apply 只按 connected/字段有效禁用；进入 `.sending` 后仍可再次点击。ACK 仅按 type 更新最后状态，没有 sequence，且没有发送 deadline 将永久 sending 变为 timeout。
- 这与第 31 轮“同类事务共用 callback context”叠加，可让第一条 ACK 显示为后一次操作结果。建议同类单 in-flight 或 V2 sequence；sending 时禁重复提交，显式显示 timeout/superseded。

### `R35-LIFECYCLE-004` - Medium/High - hide/resign/terminate zero 是异步 best-effort，无退出完成屏障

- `didResignActive`、`didHide`、`willTerminate` observer 都异步创建 MainActor task 调 `sendZeroWheelSpeeds()`；该函数会清 timer/target并提交一次 wheel zero，这是正确的本地意图清理。
- 但 `.withResponse` 写没有 completion owner，`willTerminate` 也不会等待 task/GATT response；主动 disconnect 同样先调用一次 zero 后立即 cancel connection。进程退出、系统杀死或 BLE stall 时只能依赖下游 motion lease，而当前 end-to-end lease 尚未建立。
- 建议把本地生命周期 zero 作为第一层保护，并把 S3/CM7/MotorBoard 独立 lease/feedback watchdog 作为权威停机；允许正常退出短时等待 zero write result，但不能依赖桌面进程完成回调才安全。

### 第 35 轮已有保护

- App 在 BLE 状态离开 connected 时停止 heartbeat、清 wheel target/joystick/PID 状态；BRAKE、摇杆回中和失焦/隐藏使用有效零四轮路径。
- connection panel 显示 BLE 错误，telemetry stores 对断连/stale 有降级显示，PID 有 sending/applied/rejected 状态。缺口集中在非 PID 控制命令和跨层 readiness/交付证明。

## 10. 第 36 轮：测试、CI、fault injection 与证据复现

### `R36-APP-CHECKOUT-001` - High - App clean checkout 源图不闭合

- `BLEManager` 必须实例化 `SessionLogWriter`，`Package.swift` 又把整个 `Sources/SmartCar_Control_MAC` 作为 executable target。
- 当前 `SessionLogWriter.swift` 存在于工作树但未被 Git 跟踪。当前本机源码可能可编译，但 fresh clone/`git archive`/CI 会缺类型定义；旧本机 bundle 不能证明当前仓库可复现发布。
- 建议位置：App 版本边界。原因：生产 target 的必需源必须受版本控制。内容：先审查其日志/隐私/接口，再纳入版本控制并检查全部资源。潜在影响：文件接口若尚未冻结需先评审。验证从 `git archive` 临时目录 clean Swift build/test/bundle。

### `R36-COVERAGE-002` - High - 安全控制主链没有项目级可执行测试

- App `Package.swift` 只有 executable target，无 `testTarget` 和 `Tests/`。
- S3 生产构建包含 command bridge/App parser/STM UART/BLE；现有 radar host 脚本只覆盖 radar parser/FIFO/uplink codec/tx/queue 与 shared codec，不覆盖 command bridge、BLE lifecycle、motion session/lease。
- CM7 生产构建包含 chassis、S3 service、MotorBoard、PID 和姿态安全，但没有项目级 `enable_testing/add_test` 或 HAL/FreeRTOS fake 测试。
- 仓库未发现 tracked `.github/workflows`、GitLab CI、Jenkins 或 Azure pipeline。急停、准入、coalescing、ACK 乱序、BUS_OFF、feedback timeout 和安全门撤销没有提交级回归门。
- 建议最小增加 App write/readiness/lifecycle、S3 parser/command bridge、shared SRP link、CM7 gate/zero/MotorBoard feedback 的纯逻辑/fake 测试；每个已报告 P0 至少有一个可重复 fault-injection case。影响是需要少量 test doubles 和构建时间，但不应借机重构生产接口。

### `R36-CI-003` - Medium - 现有验证入口不形成发布门或产物溯源

- radar host 脚本使用环境裸 `cc` 和临时目录，未锁工具链/输出报告；App `build_and_run.sh` 的 build 仅执行默认 Swift build/复制二进制，verify 只看进程一秒后仍存在。
- 没有一个自动入口联合执行 SRP/radar/Swift/CM7/S3，并记录 commit、工具链、source/config hash、产物 hash和证据层级。
- 建议建立统一但分层的验证入口：source/host/build 可以 CI 自动化；device/UART/BLE/vehicle 必须单独签收，不能由 build 绿灯冒充。验证同一 commit 两次 clean run 的输入清单一致，缺源/漏测硬失败。

### 第 36 轮已确认保护与对齐边界

- radar host tests 直接编译生产 `radar_parser/frame_fifo/uplink_protocol/uplink_tx/telemetry_queue` 源；CM7/S3 也共享同一 `Common/SRP` 实现，避免维护测试副本。
- 文档 host 命令虽把 `srp_link.c` 编进 binary，但 `test_srp_codec.c` 不调用 `srp_link_*`，只能证明编译覆盖，不能证明 pending/retry/BUS_OFF/recovery 行为。radar tests 也不覆盖生产 UART/task/socket/reconnect 集成。
- 本轮没有运行任何现有测试；“测试文件存在”和“当前快照测试通过”仍严格分开。

## 11. 第 37 轮：跨域终审与验收矩阵

### 11.1 去重后的根因结论

第 28-36 轮共记录 36 项详细发现（1 Critical、10 High、其余为 Medium/Low 及组合等级）。终审不把前 27 轮已知 STOP/lease/BUS_OFF/feedback 问题重复计数，而按根因归并：

| 根因族 | 本十轮新增/加强证据 | 与旧风险关系 | 终审判断 |
|---|---|---|---|
| 秘密与发布信任 | tracked Git 历史含活动凭据；secure boot/flash encryption 未锁发布配置；BLE command 未要求认证 | 比旧“secret 文件 ignored/TCP 无认证”更严重 | secret 先轮换；release 与 network gate 阻断 |
| 单 writer/事务身份 | BLE callback 跨 owner 写 motion；PID/SYS_CONFIG 共用 context；coalesce 无 superseded；UI 无 sequence/stage | 加强旧 callback 重入和 App ACK 缺口 | 先建立 epoch + immutable transaction slot，再讨论吞吐优化 |
| 实时/资源上界 | MotorBoard 长 critical；UART2 ISR 512 B copy/cache/re-arm；App queue/磁盘无界；ROS O(n²) resync | 与旧 UART/feedback/WCET 风险叠加 | 需 host 压测 + DWT/HWM/heap/IRQ 目标板量测 |
| ABI/时间/错误语义 | packed logical pointer struct、DWT 多回绕、V2 32-bit seq、fast ACK/log result 丢失 | 多数是中长期可移植性/诊断缺口 | 不改变当前 wire bytes；用 compile assert、wrap/fault vectors 关闭 |
| 操作者与暴露面 | physical connected 解锁控制、STOP/计数不代表执行、退出 zero 无 barrier、BDA/SSID 日志 | 独立验证旧盲控/断连安全问题 | UI 只能展示已取得的阶段证据，不能把 enqueue 标为 applied |
| 测试与可复现性 | App 必需源未跟踪；无 App test target/CI；motion/SRP link/fault 主链缺项目测试 | 与 R18 clean-checkout 阻断合并 | 当前脏树不是可复现 release，旧本机产物不是当前证据 |

### 11.2 独立停止门

| 门 | 必须先关闭的事实/风险 | 通过证据 | 未通过时动作 |
|---|---|---|---|
| S0 secret incident | `R34-SECRET-000` | 旧凭据撤销、全部 refs/artifacts/caches 扫描、新 secret injection 审计；报告不含原值 | 不再分发含秘密的 refs/产物；历史清理另行授权协调 |
| R0 reproducible release | R18 build graph + `R36-APP-CHECKOUT-001`、S3 target/security profile | clean archive 的 App/S3/CM7 构建与 host tests，工具链/config/source/产物 hash 可追溯 | 仅称开发快照，不签 release |
| P0 protocol/transaction | ACK strict/replay/epoch、callback non-reentrant、immutable context、wrap/timeout/fault tests | shared link + S3 command bridge fault-injection 全通过 | 不开放产品 motion 或动态 baud 切换 |
| M0 motion safety | 有效急停、BLE auth/connection epoch、operator lease、CM7统一非零准入、MotorBoard RUNNING/feedback/priority stop、独立 CPU-fault stop | 匹配 App/S3/CM7 镜像的 FFE1 -> UART2 -> USART6 抓取与 fault matrix；最后输出为 zero | 禁止车辆运动验收，只做无执行器/架空轮验证 |
| T0 realtime/resources | ISR/critical WCET、任务 HWM、heap minimum、queue/ring waterline、BLE/TCP backlog | 目标板 DWT/GPIO/RTOS counters 和长时峰值压测 | 不声明时延/容量余量 |
| N0 ROS2 live | TCP auth/encryption、client/frame deadline、source epoch/time、linear bounded assembler | 未授权/重放/半开/漂移注入，过期或伪造数据不进 `/scan` | 保持 `transport: unconfigured` 或受控 replay/PoC |
| D0 dual core | option bytes、HSEM boot/timeout、D2物理分区、CM4F RTOS、versioned mailbox/cache/reset/heartbeat、单核回退 | 双 map/readelf、CM4 缺失/卡死/复位/cache stress，CM7 stop 权不下降 | 保持 CM7-only；CM4 不迁移 sensor/UART/MotorBoard/safety owner |
| V0 vehicle acceptance | S0/R0/P0/M0/T0 全通过且匹配镜像已烧录 | 架空轮 -> 低速台架 -> 受控实车分阶段签收，记录物理停机上界和恢复不重放 | 不宣称 READY/车辆验收完成 |

### 11.3 双核与冻结合同终审

- 第 28-36 轮没有证据支持提前启用 CM4。推荐仍是：先关闭 CM7-only 单核运动/故障链，再做 CM4 no-op + heartbeat，最后仅评估低风险日志/诊断消费者。
- 轮序 `[M1:RR,M2:RF,M3:LR,M4:LF]`、`193.0 mm`、`WHEEL_TRIM`、PWM/编码器符号和 sync/attitude/BUS_OFF/emergency-stop 门仍为冻结合同；本审计没有建议借安全修复改变它们。
- ROS2 仍无 `/cmd_vel`/执行器控制，默认 `unconfigured` 是保护；它不构成车辆 READY，也不消除 live 感知链认证/新鲜度风险。

### 11.4 证据边界

- 本十轮是当前脏工作树的静态源码/配置/Git 元数据审计；凭据比较只输出路径、数量、相等关系，不输出原值或可逆表示。
- 审计期间 HEAD 未变，但 `git status --porcelain` 从 147 增至 151，且 command bridge、MotorBoard、chassis/safety/service 等脏源码在同一 HEAD 下继续演化；终稿行号和哈希以最后一次复核锚点为准，不把 HEAD 相同误写成工作树稳定。
- 未执行 build、host test、烧录、option-byte/eFuse 读取、UART/BLE/TCP 抓包、DWT/HWM 测量、目标板或车辆测试。
- 因此本轮可以关闭“第 28-37 轮文档审计”，不能关闭任何需要 build/device/vehicle 的验收门。

## 12. 终稿检查

### 12.1 最终源码锚点

- 复核时间：2026-08-31 04:48:23 CST；HEAD `f703453727a136d15ff7cacea4530beab6e9c08a`；`git status --porcelain` 151 项。
- 下列 13 个非 secret 关键源码连续两次 SHA-256 一致；secret 文件/值不进入哈希清单。

| 路径 | SHA-256 |
|---|---|
| `Common/SRP/include/srp_def.h` | `c65fd6468767776258912730c942f75cb0913525163adeeda1c17b5eee80eec2` |
| `Common/SRP/include/srp_link.h` | `2c5a65c42febf976bd07b317d1832c5c7d4cc931c63cccff60474918c20e79c0` |
| `Common/SRP/srp_link.c` | `3e68dc5de9ae8a3a0338a2b919f1c25b0bc4e4b890625e684de7bfcd4ba180b8` |
| `ESPS3/components/smartcar_service/command_bridge.c` | `84401441cb38254d458eb2095f2a9ce84d1e88cad013e6c24e3e9732cec9e0d8` |
| `ESPS3/components/s3_ble/s3_ble.c` | `256bee30e1aefe02f7252de920c9facbdd6da1e5f1032018efb87d15210913dd` |
| `ESPS3/main/radar/radar_uplink.c` | `799ae62b21c8545e52960d66df373e11751b84ce64a6bfc273d62982dd09f640` |
| `STM32H757/Middleware/Communication/UART_Link/uart_link.c` | `b7b16ee01e5789817a28754e1e08f2adc04640b3eb6efdbc393720d7da108467` |
| `STM32H757/Middleware/MotorBoard/motor_board_task.c` | `7793dfcd66a78734a7dc63d071035c4bd67ed0f2c470cdb6e0ab2a807fdbde0e` |
| `STM32H757/BSP/TIMER/bsp_timer.c` | `5225489761952a9eb7445cc99d9be7f5ada947b71bc8043666cefc87c8915702` |
| `IOS_APP/SmartCar_Control_MAC/Sources/SmartCar_Control_MAC/BLE/BLEManager.swift` | `77ae84d610fa8461e001f77fd8fd66071a308c4ea6bd908faa3dcd98779645b0` |
| `IOS_APP/SmartCar_Control_MAC/Sources/SmartCar_Control_MAC/ViewModels/SmartCarViewModel.swift` | `ec587045dfc4c87a8a48fb6452a8c26d48de7b114d875dd56d5711e5b8b61eed` |
| `IOS_APP/SmartCar_Control_MAC/Package.swift` | `d2753932e9601b56c40d8844504440d63f33c671a9ea53e3f58ad1e046b694db` |
| `ROS2_WIN/src/s3_ydlidar_bridge/src/framing.cpp` | `b218ea9ea4632d825393ec99f4a9bd29c955fedad8e85f691aaafd6d7de21fd6` |

### 12.2 检查结果

- [x] 第 28-37 轮均有独立证据/关闭记录，无重复文字充数。
- [x] 所有 High 发现在终稿源码快照上重查行号和调用图。
- [x] 正式计划、总报告、findings/progress/task plan 与详细证据一致。
- [x] 已知 secret 字面量检查 12 个，六份审计文档命中 0；本轮未写非审计文件。
- [x] 关键源码哈希、本地链接、状态标记、限定/全树 `git diff --check` 和 Git 写入边界通过。
- [x] 终稿明确未构建/测试/烧录/抓包/目标板/车辆验收。
