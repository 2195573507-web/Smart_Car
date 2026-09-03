# STM32H757 / ESP32-S3 当前工作树深化审计：第 18-27 轮

> 审计日期：2026-08-31（Asia/Shanghai）
> 起始快照：分支 `codex/s3-stm-cn-comments`，HEAD `f703453727a136d15ff7cacea4530beab6e9c08a`
> 证据级别：当前脏工作树源码/CMake/Kconfig/链接脚本静态审计
> 边界：未构建、未运行主机测试、未烧录、未抓取 UART/BLE/Wi-Fi，未执行目标板/车辆验收。

## 1. 本轮规则

- 本文仅记录第 18-27 轮的新证据、旧结论修正或独立交叉验证；不以重复第 13-17 轮文字充数。
- 源码在同一 HEAD 下存在并行未提交修改；终稿必须重查关键文件行号和 Git 状态。
- 标签继续使用 `CONFIRMED_SOURCE`、`CONFIRMED_CONFIG`、`INFERRED_RISK`、`UNVERIFIED_RUNTIME` 和 `RECOMMENDED`。
- 建议不是已实施事实；每项建议需列出位置、原因、内容、潜在影响和分层验证。

## 2. 第 18 轮：活动构建图与 clean checkout 可复现性

### `R18-BUILD-001` - High - 未跟踪统一调试头使 CM7/S3 clean checkout 构建图不闭合

- **确定事实：** CM7 CMake、S3 main/smartcar_service/stm_uart 组件都无条件加入 `Common/SmartCarDebug`；15 个活动源直接 include `smartcar_debug_config.h`。`git ls-files Common/SmartCarDebug` 为空，当前只有未跟踪头文件。
- **影响：** 当前本机旧 build/cache 不能证明仓库可复现；该头还控制容量、日志周期和条件编译。
- **建议：** 位置是 `Common/SmartCarDebug` 版本边界与两端 CMake；以当前默认值纳入版本控制，原因是关闭 clean source graph。保持默认值时不应改变运行行为。
- **验证：** `git archive` 临时目录中分别 configure CM7/S3，检查预处理宏和 compile commands；本轮未执行。

### `R18-BUILD-002` - High - fresh S3 checkout 没有固定 `esp32s3` target

- **确定事实：** `ESPS3/sdkconfig.defaults:1-36` 和项目 CMake 都没有 `CONFIG_IDF_TARGET/IDF_TARGET`；只有 ignored `sdkconfig` 和旧 build cache 保存 `esp32s3`。项目文档直接给出 `idf.py build`，未先 `idf.py set-target esp32s3`。
- **影响：** ESP-IDF 5.5.4 在没有 env/cache/config target 时可回落到默认 `esp32`，与 N32R16 的 OPI/PSRAM/外设合同不同。`dependencies.lock` 中的 target 不替代前置 target 选择。
- **建议：** 位置是 tracked defaults 或 S3 项目 CMake；显式固定 `esp32s3`并修正构建文档。影响是 clean configure 行为被固定，不再依赖本机状态。
- **验证：** 清空 `IDF_TARGET`、sdkconfig 和 build cache 后配置，确认生成 target/链接脚本均为 ESP32-S3。

### `R18-BUILD-003` - Medium - uplink 启用态依赖 ignored 凭据头，仓库无可构建模板

- tracked Kconfig 默认关闭 uplink；当前 ignored `sdkconfig` 开启后，`radar_uplink.c` 必须 include 被 `.gitignore` 忽略的 `radar_wifi_credentials.h`。
- 建议提交不含秘密的类型/示例模板，真实值继续由本地/secret include 注入；CI 用 dummy secret 构建启用态。不得把真实 SSID/密码写入报告或 Git。

### `R18-BUILD-004/005` - Medium - 死选项、陈旧 cache 和生成 CMake 中重复 BSP test 使构建语义漂移

- `SMARTCAR_SCHEDULER_PROBE` 仅在 CMake 定义/传递，当前源码无消费者；Debug cache 仍保留已删除的 GPIO18/UART2 试验键，但 compile commands 不再传递它们。
- 声明“不应修改”的 `mx-generated.cmake:95-113` 同时把 `BSP_TEST/bsp_test.c` 编入主固件并再创建一个未链接 OBJECT target，compile commands 有两条实例且宏集不同。
- 建议删除死选项/用 `cmake -U` 清旧键，并把 BSP test 接线移出生成文件，只保留单一 target 归属。验证两种选项预处理结果和单一 compile command。

### 第 18 轮修正/确认

- 两端活动 CMake 均编译 `Common/SRP/{crc,wire,codec,link}.c`，无活动 SCBP-CAN 源路径。
- 活动 BMI323 是 `Middleware/Sensor/BMI323`；旧 `Drivers/IMU/BMI323` 仅残留 include path，当前限定 include 避免了误解析，但建议清理死 include 路径。
- CM7 toolchain 仅查 PATH 中 `arm-none-eabi-*`，未锁定版本；当前本机 GCC 不是仓库合同。

## 3. 第 19 轮：SRPv4 codec/link 与对抗输入

### `R19-BUSOFF-001` - High - S3 BUS_OFF 恢复前没有 motion stop 屏障，且可遗留 in-flight 软件状态

- **确定事实：** `command_bridge_bus_off():402-418` 将 sync 状态返回 UART_READY 并计划 100 ms 后 recover，但不取消 motion transaction、不清 App session，也不发 zero。CM7 S3-frame timeout 是 200 ms。
- `srp_link_recover():275-285` 直接 `memset` pending 而不执行 completion callback；S3 `s_motion_tx_in_flight` 可永久保留 true，后续命令只覆盖 pending slot 而无人再启动。
- **风险：** S3 可在 CM7 200 ms 失联强停之前重新同步，旧非零目标不一定被清；同时 S3 自身 motion pipeline 可卡住。
- **建议：** 位置是 S3 BUS_OFF/recover 事务和 shared `srp_link_recover`；原因是 recover 必须先收敛 motion。内容包括原子取消/回调全部 pending，清 App epoch，有界发 zero，再重建 UART/sync。影响是重连后必须重新建 session/目标。
- **验证：** 在非零 in-flight/pending 各状态注入 BUS_OFF，检查最后 motion 为 zero、所有 callback 只完成一次且恢复后旧命令不重放。

### `R19-RETRY-002` - High - transport retry 失败不受 `max_retries` 限制，BUS_OFF 也不拦截 send/tick

- `srp_link_tick():246-260` 只在 transport 返回成功时增加 `retry_count` 和更新 `last_tx_ms`；失败只增 TEC，下一个 tick 立即再试。
- `srp_link_send/tick` 不因 `SRP_LINK_BUS_OFF` 停止新发送/重试；`SRP_LINK_TX_TRANSPORT_FAILURE` 和 `SRP_LINK_TX_BUS_OFF` 枚举当前从未交付给 callback。
- 建议将“传输接受尝试”和“成功重传”分开计数，两者均有硬上限；BUS_OFF 拒绝非恢复流量并完成所有 pending。压测永久 transport failure、间歇成功和 tick 频率差异。

### `R19-ACK-003` - High - ACK/ERROR 语义校验过宽，旧 ACK 可与回绕后 pending 混淆

- `srp_link_receive():193-218` 只要 payload `>=4`，不检查精确长度、reserved=0、ACK/ERROR type 与 flag/status 自洽，并在语义校验前降低 REC。
- pending 只用 8-bit type+sequence 匹配，无 session/epoch；序号回绕后迟到旧 ACK 可清理新事务。
- 建议严格固定 4 B/reserved/flag/status，将 pending 绑定 link epoch 并对未匹配/过期 ACK 计数。验证超长、错 flag/status、旧 epoch、序号回绕和 ACK 重放。

### `R19-REPLAY-004` - High - ACK-required 当前是 at-least-once，接收端无 replay cache

- 重试保留同一 type/sequence/帧，但两端业务 callback 没有已执行序号缓存，每次收到都可重复执行副作用。
- SYS_CONFIG 两端在安排 20 ms 后切换波特率时忽略 ACK 发送结果；ACK 丢失可造成重复副作用或两端分频。
- 建议位置是 SRP receiver/session 层；对有副作用命令实现 `(epoch,type,sequence)` replay cache，重复帧只重发原 ACK 不重做业务。对波特率等双端事务使用 prepare/ack/commit/rollback。

### `R19-LIVE-005` - Medium/High - 业务校验前续期 control liveness

- S3 `command_bridge.c:699-707` 和 CM7 `s3_service.c:557-568` 对任意 codec-valid 帧先刷新对端 liveness，然后才进入 link/业务类型。
- 未知 type、语义畸形 ACK 或业务无效帧可延迟停机。建议只有明确的 heartbeat/受支持控制帧刷新 control lease，遥测/日志和未知帧不续期。

### `R19-CALLBACK-006` - Medium - S3 completion callback 实际递归 send，违反 shared link 不可重入注释

- `srp_link.h:100-115` 要求 completion callback 不递归 send 同一 link；S3 motion completion `command_bridge.c:1042-1106` 会在 callback 中立即 `start_motion_command()` 并再调 `srp_link_send()`。
- 建议 callback 只投递完成事件，下一个 service loop 再发送；验证 callback 中取消/发送、pending 满和 BUS_OFF 交错。

### `R19-PARSER-007` - Medium - parser 无半帧超时，priority/reserved 等到整帧才拒绝

- parser 读到 8 B header 时只查 payload length，priority/reserved flags 由完整帧 `srp_decode()` 检查；一个声明大 payload 的截断帧可长期占住 parser。
- 建议 header 尽早校验且 transport/session 层提供半帧超时/discontinuity generation。验证截断帧、缓慢逐字节和断线跨 epoch 半帧。

### 第 19 轮测试缺口和非缺陷

- `Common/SRP/tests/test_srp_codec.c` 仅覆盖 codec/CRC/TLV；“sequence wrap”只测 header 宏，没有 link ACK/pending/retry/BUS_OFF/replay/wrap 测试。
- 小端 header 下 `data[7]=priority`、`data[6]=type` 是正确的；当前点对点 UART 不序列化 destination 是已声明设计，不重复报错。

## 4. 第 20 轮：STM/S3 UART、DMA、背压与恢复

### `R20-TX-001` - High - S3 TX 的 100 ms 等待不包含入队，recover 也没有清理旧 TX

- `stm_uart_send():417-448` 取外层 mutex 后调用 `uart_write_bytes()`，然后才 `uart_wait_tx_done(100 ms)`。当前 IDF 5.5.4 UART 实现向 TX ring 填写时使用 `portMAX_DELAY`，ring 停滞可无界阻塞唯一 service task。
- `stm_uart_recover():474-504` 调用的 `uart_flush()` 是 RX flush 别名，连续调用不会取消已排队/在途 TX。超时后旧 motion/ACK 仍可发出，link retry 又可再排一份。
- 建议位置是 S3 UART TX owner/driver lifecycle；建立单一有界 TX worker，enqueue 和物理完成分别有 deadline，recover 在 TX owner 内废弃旧 epoch 并重置 driver/FSM。
- 影响是增加 TX 队列和调度复杂度；验证 TX ring 满、对端断开、超时后恢复和 stop 抢占，确认旧 epoch 不会后发。

### `R20-RX-002` - Medium/High - 两端软件丢字节没有完整 discontinuity 合同

- CM7 ring 满时丢最旧字节，recover 会清 ring，但 parser 只初始化一次且恢复后不 reset。
- S3 硬件 FIFO/driver/线路错误会置 discontinuity，但 software mutex 失败或 8 KiB ring 溢出只累计 drop。
- 建议在两端增加单调 discontinuity generation；overflow、recover、rearm failure、baud switch 均推进，service 在下一次 feed 前 reset parser。验证半帧 + 溢出/复位/切波特率。

### `R20-RECOVER-003` - High - CM7 worker recovery 可与活动 TX 并发 abort/deinit 同一 HAL handle

- CM7 `uart_link_send():519-585` 在 TX mutex 内执行 blocking HAL TX；`uart_link_recover():650-687` 不获取该 mutex，worker 可直接禁 IRQ、abort、deinit/reinit UART/DMA。
- service 的 link mutex 不覆盖 UART worker；已通过 ready 检查的发送可与 deinit 交错。
- 建议先关闭 admission，然后只由持 TX mutex 的 owner 做 abort/deinit/baud switch；无法取锁时延后恢复而不并发改 HAL handle。
- 验证长 TX + RX error/BUS_OFF/baud switch 交错，要求无死锁/use-after-deinit，恢复首帧完整。

### `R20-INIT-004` - Medium - UART 分阶段初始化失败缺少完整逆序回滚/重试

- CM7 UART/DMA/mutex 配置失败时 worker 不一定创建，只有首次 RX arm 失败显式请求 restart。
- S3 driver install 后 threshold/timeout 失败不删 driver，RX task 创建失败也不释放 driver/两个 mutex。
- 建议统一 staged init + reverse cleanup，能力状态标明 retryable/degraded/fatal，运动一直 zero。逐阶段注入失败并检查资源计数/重试。

### `R20-DIAG-005` - Medium - 部分 UART 诊断字段从不更新

- CM7 `tx_dma_starts/errors`、`tx_preemptions`、TX DMA IRQ 统计只初始化/读取，无实际增量；S3 `tx_queue_drop` 不更新，`tx_queue_pending` 固定输出 0，却出现在周期诊断。
- 建议删除不适用字段或连接真实 driver 状态，并给 snapshot 失败增 valid 标志。

### 第 20 轮已确认保护/修正

- CM7 DMA buffer 已 32 B 对齐放入 D2 `.dma_buffer`，pre-arm 和 ISR copy 前均 invalidate cache；DMA/USART IRQ priority 5 与 FreeRTOS syscall 边界一致。
- 旧“DMA buffer 在 DMA1 不可达 DTCM”结论已关闭；CM4 对同一 D2 SRAM 的别名覆盖风险仍开放。
- CM7 1 kHz 每轮最多 128 B，静态 drain 约 128 KB/s，高于 921600 8N1 线速约 92.16 KB/s；S3 100 Hz x 256 B 约 25.6 KB/s，低于线速。这只证明 S3 burst/backpressure 风险，不证明现场已溢出。

## 5. 第 21 轮：运动命令所有权、租约与最终准入

### `R21-STOP-001` - High - 主控制页红色急停无效且会继续非零 heartbeat

- `ControlModeView.swift:36-40` 调用 `emergencyStop()`；该函数仅发 V1 `CONTROL/STOP`，不清 wheel target/timer，见 `SmartCarViewModel.swift:258-265,314-337`。
- S3 motion bridge 无 `CONTROL 0x01` 分支，最终返回 rejected；轮速卡 `BRAKE` 才调用有效零四轮路径。
- 建议位置是 App 急停命令/UI；原因是当前可见急停与线上合同断开。最小内容是复用 `sendZeroWheelSpeeds()`、停表/清目标；正式 E-stop 还需优先级、锁存和确认合同。
- 影响是改变 App 急停帧和 ACK UI；验证两个红色按钮后 500 ms 内只有 zero motion，S3/CM7/MotorBoard 最终输出为零。

### `R21-EPOCH-002` - High - BLE 断连 stop 后可执行旧队列，V2 session 也未绑定物理连接 epoch

- 断连回调不清 BLE RX queue/parser/V2 session；service 同一轮先 stop，再消费最多 4 个旧 queue item。V2 HELLO 直接覆盖 session ID，不撤销旧 inflight/pending motion。
- 建议位置是 S3 disconnect/HELLO/RX item；原因是 owner 必须与 BLE connection epoch 一致。在 service task 原子 reset queue/parser/session/motion，RX item 携 epoch，新 HELLO 先撤销旧 owner。
- 影响是重连必须重建 session/发新目标；验证 8 项满队列、半帧跨连接和 session 轮换，最后 UART motion 必须为 zero。

### `R21-LEASE-003` - High - V1 无 motion deadman，V2 `valid_for_ms` 只防止排队过期

- App 不实现 V2；V2 valid-until 只在发起 SRP 前检查，一旦 CM7 接受就不在到期时清零。V2 heartbeat 可无限续 session，S3 又独立用 100 ms SRP sync heartbeat 维持 CM7 liveness。
- 建议建立独立 operator-motion lease：V1 非零短周期刷新，V2 在 command TTL 到期强制零，SRP sync 不续 operator lease。
- 影响是旧 App 若不按频率刷新会停机；验证 BLE 保持但 App 冻结，在链路遥测仍正常时 motion lease 仍必须导致 zero。

### `R21-GATE-004` - High - CM7 direct wheel 可重新打开姿态门已强停的 MotorBoard

- coordinator freshness 失效会清 ready 并强停；CM7 wheel handler 仅有 sync/BUS_OFF 公共门，`motor_board_set_target_wheel_speeds()` 接收非零后直接清 forced-stop。
- 建议在最终执行器入口要求统一 control-admission token；姿态门关闭后普通 setter 不得清 stop latch，仅保留显式 motor-board-only 诊断例外。
- 影响是 direct-wheel 准入时机改变；验证非零与 freshness-loss 并发，直到新 epoch + 连续稳定周期前 PWM 均为零。

### `R21-CHASSIS-005` - High - chassis task 仍未启动，零 chassis 命令也不是物理停机

- 全树 `chassis_task_start()` 只有声明/定义；chassis speed 仍可写状态并 ACK。`chassis_task_force_stop()` 只清本地状态，不调用 MotorBoard。
- 建议在 coordinator 确认 MotorBoard task 创建后幂等启动 chassis；协议层全局 stop/零 chassis 直接强停 MotorBoard，不依赖周期任务。
- 影响是增加 512 words 栈和 10 ms 控制周期；验证 direct/chassis/heading 三种模式发各类零命令都有最终零 PWM。

## 6. 第 22 轮：MotorBoard 闭环与执行器失效安全

### `R22-STATE-001` - High - 初始化 FAILED/未 RUNNING 不是非零输出门

- 初始化失败只排一次 zero 并置 `MB_SEQUENCE_FAILED`；setter 不检查 sequence state，任意 MSPD 都执行 PID，任务入口还会提前清 stop latch。
- 建议仅 `RUNNING + fresh MSPD + control admission` 允许非零；FAILED 原子清目标/PID 并锁住到完整重初始化。
- 影响是初始化期间所有调参/运动命令被拒绝；验证每个初始化步骤 NACK/timeout + 杂散 MSPD/非零命令，PWM 一直为零。

### `R22-FEEDBACK-002` - High - 运行期无 MSPD feedback watchdog

- PID 只在收到 MSPD 时运行；RUNNING 后不保存最后有效 MSPD 时间，1 s response timeout 只服务初始化 sequence。
- 建议非零目标下 MSPD 超时立即锁止/强停，恢复必须重建连续 feedback READY。影响是板端上传抖动阈值需实测。
- 验证运动中分别断 USART6 RX/TX、板复位，确认规定时限内 zero/硬件 disable，且旧目标不自动恢复。

### `R22-DT-003` - Medium/High - 当前 MSPD PID 已回退为固定 50 ms dt

- `MB_PID_DT_SECONDS=0.05f`，Ramp/PID 每帧都使用该值；源码无 20 Hz 强制保证。旧“动态 dt 已完成”对当前工作树不成立。
- 建议用相邻有效 MSPD 单调时间计算 dt；首帧、`<2 ms`、`>100 ms`、回绕/非有限时重锚并跳过积分。
- 影响是 PID/斜坡动态变化；验证 2/20/50/100 ms、丢帧和回绕回放，输出有界且不跨长间隔积分。

### `R22-STOP-004` - High - 强停/故障路径无可靠物理优先通道

- `motor_board_force_stop()` 把 zero 追加到普通 TX FIFO，满时只返失败；`MB_Transport_Send()` 还把 TX 成功绑定到 RX 能否 re-arm。TX ring 无 stop 预留/优先级。
- HardFault/BusFault 记录后关 IRQ/WFI，普通 USART6 IRQ stop 无法排空。
- 建议为 stop 保留不受普通流量占用的完整帧槽，并使用独立 motor-enable/板端 watchdog 覆盖 CPU fault。验证 TX 满、RX arm 失败、PID TX 中和 HardFault 四种场景的物理停机上界。

### 第 22 轮冻结合同/现有保护

- 轮序仍是 `[M1:RR,M2:RF,M3:LR,M4:LF]`，RF encoder 仅在 PID 输入反相；几何 `193.0 mm`，trim `1.08/1.00/1.00/1.00`。
- PID 有 finite/目标/积分/输出限幅、零目标积分清除和抗饱和。这些保护不替代 lifecycle/feedback/fatal-stop 门。

## 7. 第 23 轮：传感器、标定、DualAHRS 并发与 freshness

### `R23-SNAPSHOT-001` - High - DualAHRS 全局 context 无锁，高优先级读者可得到撕裂安全状态

- 全部 estimator 状态在无锁 `s_dual`；BMI 200 Hz task 写 `dual_ahrs_update()`，attitude gate、telemetry 和预期 chassis task 并发读。
- `get_output/get_heading_state/is_primary_fresh` 直接复制/读 float、flags 和 64-bit 时间戳，无 mutex/seqlock。
- 建议单 writer 保持 estimator 私有，用 sequence counter/双缓冲原子发布 compact snapshot；bias/leveling/reset 通过 writer 事件串行，不用长临界区包三角函数。
- 影响是增加快照复制/屏障；验证在每个字段写点强制抢占，读者只能看到完整 N 或 N+1 snapshot。

### `R23-BMI-HEALTH-002` - Medium - BMI323 持续 SPI 故障会安全锁车，但没有自动恢复闭环

- BMI task read 失败只计数并喂 invalid sample；`imu_update()` 返回值只反映 LSM，1 s recovery 条件不由 BMI failure 触发。`bmi323_is_online()` 为 init ready latch，遥测 ONLINE 也直接使用它。
- 现有保护是 DualAHRS primary 50 ms freshness + coordinator 20 ms 检查会强停。
- 建议为 BMI 建立动态 read-fail/stale 健康和有界 recovery；恢复前撤销 admission，停 producer、锁 driver、重 init/ODR/自检，再经稳定门。ONLINE 来自动态健康而非 ready latch。
- 影响是增加传感器恢复期停机；验证持续/间歇 SPI 失败、mutex contention 和 recovery failure，确认先锁车且成功稳定后才解锁。

### `R23-FRAME-003` - Medium - BMI sensor-to-body 变换合同自相矛盾，不能静态证明坐标一致

- manager 注释声明 BMI accel/gyro 共用同一 frozen leveling rotation、禁止轴特定符号变化；DualAHRS 又只额外翻转 `gyro.z`。LSM 则明确使用 determinant `+1` 的 Body-Z 180 度旋转。
- 这不能证明现有 yaw 极性错误，单轴修正可能来自历史实车证据；但当前不是单一刚体旋转合同。
- 建议冻结 BMI `R_sensor_to_body` 正交矩阵并同时用于 accel/gyro，或明确记录 gyro 例外的物理原因；不在无实物证据时擅改极性。
- 影响是可能改变航向符号，必须以六面静置、Body X/Y/Z 正负旋转和 LSM/BMI 同步回放验证右手系/正 yaw。

### 第 23 轮关闭/修正的旧结论

- LSM303 DRDY 掩码当前正确：accel `ZYXDA=0x08`、mag `DRDY=0x01`，NOT_READY 不发布重复样本；旧掩码问题关闭。
- `imu_manager` 的 BMI/LSM/snapshot/dual-init 数据、calibration 和 boot manager 已有各自 mutex，不再笼统报“整个 IMU 链无锁”；未关闭的是 DualAHRS publish 层。
- BMI323 已逐样本驱动 Primary DualAHRS；`imu_manager.c` 中“BMI telemetry-only”注释过时。
- 姿态 freshness 已持续检查并强停；旧“只在启动检查”结论关闭，direct wheel 绕过最终门仍开放。
- 标定对 BMI producer 使用 try-lock 丢样并有质量计数，LSM 按单调时间戳去重；这些是现有保护。

## 8. 第 24 轮：RTOS、资源、watchdog、fatal path 与可观测性

### `R24-S3-SCHED-001` - Medium - S3 service 实际周期是 10 ms，源码预算和旧报告容易按 1 ms 理解

**确定事实（CONFIRMED_CONFIG/SOURCE）**

- `ESPS3/sdkconfig:1529` 定义 `CONFIG_FREERTOS_HZ=100`，一个 tick 是 10 ms。
- `ESPS3/components/smartcar_service/command_bridge.c:41,1523` 使用 `SMARTCAR_SERVICE_TASK_DELAY_TICKS=1` 直接调用 `vTaskDelay(1)`。
- service 每轮最多消费 4 个 BLE RX item，并且每轮最多从 STM UART 软件 ring 取一个固定缓冲；因此吞吐上界和断连 stop 调度粒度必须按 100 Hz 而不是 1 kHz 估算。

**推断风险（INFERRED_RISK）**

- 高频 SRP 遥测/日志 burst 更容易把 S3 软件 ring 推向丢字节；BLE 断连到 zero 请求的任务调度延迟正常为一个 10 ms 粒度，还需加上 TX 等待。
- 这不等于已量得 10 ms 物理停机时间；任务抢占、BLE callback、UART 锁和 MotorBoard 均未实测。

**建议（本轮未实施）**

- 位置：S3 service 调度常量和资源文档。
- 原因：消除 ticks/ms 混淆并给 RX/stop 建立真实预算。
- 内容：显式使用 `pdMS_TO_TICKS()` 表达目标周期，或使用任务通知唤醒；增加 loop overrun/RX high-water 计数。
- 潜在影响：改变 service CPU 占用、队列吞吐和 Wi-Fi/BLE 任务调度关系。
- 验证：当前 100 Hz 与候选配置下测量 loop period/WCET、RX ring 水位、BLE queue drop 和断连到 UART zero 的上界。

### `R24-S3-WDT-002` - Medium - S3 task watchdog 不直接监督项目关键任务，且当前不以 panic 收敛

**确定事实（CONFIRMED_CONFIG/SOURCE）**

- `ESPS3/sdkconfig:1414-1419` 启用 task WDT，5 s，自动检查 CPU0/CPU1 idle task，但 `CONFIG_ESP_TASK_WDT_PANIC` 未启用。
- 项目源中无 `esp_task_wdt_add/reset/delete/reconfigure`，STM UART RX、smartcar service、radar UART 和 uplink 没有各自的 deadline/heartbeat WDT 注册。

**推断风险（INFERRED_RISK）**

- 关键任务若阻塞但仍让 idle task 运行，当前 task WDT 可能不报该任务未前进；即使 idle 被饿死，当前配置也不能仅凭源码声称必然复位到安全态。

**建议（本轮未实施）**

- 位置：S3 系统监督层和各关键任务进度点。
- 原因：idle liveness 不等于协议/安全任务前进。
- 内容：为关键任务建立 deadline/progress heartbeat，由独立监督者决定 stop/restart；是否启用 panic/reboot 必须与 STM/MotorBoard 失联停机联合设计。
- 潜在影响：错误阈值会造成误复位；复位前必须先确保 STM/MotorBoard 独立停机。
- 验证：分别卡住 service/UART/radar/uplink，以及制造 CPU 饿死，核对 WDT 检测、复位、BLE/UART 断链和最终 zero-PWM 时间。

### `R24-STARTUP-003` - Medium - 两端仍允许部分资源创建失败后进入长期降级态，缺少统一 admission 记录

**确定事实（CONFIRMED_SOURCE）**

- CM7 `main.c:216-232` 在各任务启动 API 后不收集统一结果，且在 `srp_uart/s3_service/attitude_gate` 创建和 scheduler 开始前已打印 READY。
- `log_service_start()` 仅通过 handle 判断创建失败并累加 drop；`imu_runtime_start()` 可在 sample/logger 中任一创建失败后保留另一任务。
- S3 `stm_uart_init():375-383` 在 UART driver 和两个 mutex 已创建后，若 RX task 创建失败，只清 `s_initialized`，不删除已建 driver/mutex。
- S3 `radar_uplink_init():930-1008` 在 netif/Wi-Fi/event group/event handler 逐步建立后，多个失败分支只返回错误，没有完整逆序回滚。

**影响与边界**

- 单次启动中不一定造成立即内存泄漏放大，因为 `app_main` 当前不重试这些 init；但能力位、资源归属和后续恢复语义不闭合，全局 READY 可误导。

**建议（本轮未实施）**

- 位置：CM7/S3 启动 admission 表与各 init rollback。
- 原因：使“关键能力失败”不再只是一条日志。
- 内容：记录每个 driver/mutex/queue/task 的建立结果和 owner，失败时逆序释放或明确进入不可重试的 DEGRADED 状态；运动链关键项失败始终保持 zero。
- 潜在影响：需理清 ESP-IDF 全局 singleton 资源和任务重试语义。
- 验证：逐个注入 driver/mutex/queue/task 创建失败，检查资源数、能力位、日志、重试和 zero-PWM 不变式。

## 9. 第 25 轮：BLE 安全、session、CCC/拥塞、分片与重连

### `R25-APP-READY-001` - High - App 将“物理连接”当成“命令/遥测已就绪”，允许盲控窗口

**确定事实（CONFIRMED_SOURCE）**

- `BLEManager.centralManager(_:didConnect:)` 在调用 `discoverServices()` 前就执行 `updateStatus(.connected)`，见 `BLEManager.swift:332-341`。
- `writeCharacteristic` 只在后续 characteristic discovery 回调中赋值，FFE2/FFE3 只是调用 `setNotifyValue(true)`，App 没有实现 `didUpdateNotificationStateFor` 来记录 CCC 成功/失败，见 `BLEManager.swift:390-413`和当前 delegate 方法清单。
- 控制 UI 只根据 `status == .connected` 启用；`sendFrame()` 若尚无 write characteristic 才返回 false并设错。
- S3 运动 V1 入口不要求 FFE2 CCC ready，因此“可写命令但无遥测/ACK 通道”是当前允许状态。

**推断风险（INFERRED_RISK）**

- 连接初期操作可被 App 计为发送尝试却没有写入；若 FFE2 订阅失败而 FFE1 可写，操作者可在没有 ACK/状态反馈时继续发送 motion。

**建议（本轮未实施）**

- 位置：App `BLEManager` 连接状态机、控制 UI admission，S3 V1/V2 命令准入。
- 原因：物理连接不等于 service/FFE1/FFE2/session 就绪。
- 内容：分离 `connected/discovering/commandReady/telemetryReady/sessionReady`；只在写特征可用且必要的 CCC/session 成功后允许非零输入，zero/stop 保留最高可达性。
- 潜在影响：连接 UI 时序和 V1 兼容行为改变。
- 验证：延迟/拒绝 service discovery、缺 FFE1、FFE2 CCC 失败、断线重连，确认非零控件禁用且 stop 仍有明确结果。

### `R25-APP-TX-002` - High - App 主动断连的零速写没有完成屏障，发送计数也不是交付证据

**确定事实（CONFIRMED_SOURCE）**

- `BLEManager.disconnect()` 依次调用 `sendWheelSpeeds([0,0,0,0])` 和 `cancelPeripheralConnection()`，中间不等待写完成，见 `BLEManager.swift:172-177`。
- `sendFrame()` 调用 `.withResponse` 的 `writeValue()` 后立即返回 true，见 `BLEManager.swift:211-219`；当前无 `peripheral(_:didWriteValueFor:error:)` delegate 实现，也无单一 in-flight 写入/超时队列。
- `SmartCarViewModel` 多个路径在调用 BLE API 前就累加 `transmittedFrameCount`；该计数只代表 App 调用尝试，不代表 GATT response、S3 admission 或 STM ACK。

**推断风险（INFERRED_RISK）**

- CoreBluetooth 取消连接可以早于零速 write response；当前不能把主动断连前的 `sendWheelSpeeds(0)` 当成已交付保证。S3 断连 stop 仍是后备保护，但第 15 轮已证明旧 BLE 队列可在 stop 后重放。

**建议（本轮未实施）**

- 位置：App BLE TX 队列/完成回调、disconnect 状态机和 UI 计数。
- 原因：区分“提交给 CoreBluetooth”、“GATT write response”、“S3 准入”和“STM 接受”。
- 内容：串行化 with-response 写入，用 `didWriteValueFor` 推进；disconnect 等待有界 zero 写入结果后再断链，超时则依赖已验证的 S3/CM7 独立停机。
- 潜在影响：写入吞吐和断连等待时间变化，必须保证旧非零命令不排在 zero 之后。
- 验证：CoreBluetooth test double 控制 write success/error/timeout/out-of-order，并在真实 FFE1 抓取中确认断链前后最后 motion 为 zero。

### `R25-GATT-003` - Medium - S3 GATT 属性宣告上限超出当前 ESP-IDF 公开上限，与 App 实际需求也不一致

**确定事实（CONFIRMED_SOURCE/DEPENDENCY）**

- `ESPS3/components/s3_ble/s3_ble.c:19,236-265` 将 FFE1/FFE2/FFE3 `max_length` 都设为 `S3_BLE_MAX_RX_LEN=1032`。
- 当前 ESP-IDF 5.5.4 `esp_gatt_defs.h:475-481` 定义 `ESP_GATT_MAX_ATTR_LEN=517`。Bluedroid 建表路径会把应用填入的 `max_length` 直接传到 `attr_max_len`，其公开 API 上限与应用常量不一致。
- S3 App 帧 parser 和 macOS App 都限制 payload `128 B`，完整帧最大 `136 B`，见 `app_parser.h:12-20` 和 `SmartCarProtocol.swift:3-7`。
- 当前 GATT callback 只处理 non-prepared write，并不实现跨 MTU 的 prepare/execute write 组装。

**影响与边界**

- 当前 Bluedroid 内部路径可能为该值分配内存，但这超出公开上限，不能仅凭源码断言 GATT table 创建和各 peer 长度语义已受支持。它不是已确认的内存越界，而是依赖合同/可移植性风险。

**建议（本轮未实施）**

- 位置：S3 BLE 属性常量和 App frame 合同。
- 原因：让 GATT 容量符合 IDF 公开上限和真实帧上限。
- 内容：用 `SC_APP_FRAME_MAX_SIZE` 或不超过 `ESP_GATT_MAX_ATTR_LEN` 的明确上限；若确需超长命令，先定义 prepare-write/应用层分片合同。
- 潜在影响：可能改变历史客户端对特征最大长度的观察。
- 验证：ESP-IDF 构建 + GATT table event，MTU 23/247/517，136 B 最大 App 帧与超长/准备写入拒绝。

### `R25-BLE-STATE-004` - Medium - BLE init/service/notify 状态仍缺少一致发布和拥塞完成语义

**确定事实（CONFIRMED_SOURCE）**

- `s3_ble_init()` 在异步 `esp_ble_gatts_app_register()` 返回后立即设 `s_initialized=true`，此时 GATT table/service/advertising 尚未完成，见 `s3_ble.c:405-449`。
- GATT table event 中 `esp_ble_gatts_start_service()` 的返回值未检查，紧接着就设 `s_service_ready=true` 并启动广播，见 `s3_ble.c:333-344`。
- FFE2/FFE3 通知从多任务直接发送，没有 TX 串行化；事件分支没有处理 `ESP_GATTS_CONGEST_EVT` 或建立 notification completion 队列。
- CCC 开启 FFE3 时在 GATT callback 中同步 flush 最多 48 条 pending log；发送失败时保留队首，但没有后续拥塞解除事件来自动继续 flush。

**建议与验证**

- 位置：S3 BLE lifecycle/TX owner。原因：异步资源就绪、连接状态和 TX 拥塞必须是同一可观测状态机。
- 内容：单一 BLE TX worker 串行 FFE2/FFE3 分片，处理 congestion/complete/disconnect epoch；只在 table+service+advertising 成功后发布 capability ready。
- 潜在影响：增加队列 RAM 和上行延迟，需区分可丢遥测/日志和必达控制 ACK。
- 验证：注入 table/service/advertising 失败，MTU 分片并发，congest/uncongest，CCC 重复切换和分片中断连。

## 10. 第 26 轮：雷达、S3RD、Wi-Fi/TCP 与 ROS2 边界

### `R26-NET-AUTH-001` - Medium（未来自主导航前阻断）- live S3RD TCP 没有对端身份或机密性

**确定事实（CONFIRMED_SOURCE/CONFIG）**

- S3 `connect_endpoint()` 使用普通 IPv4 TCP `socket/connect`，没有 TLS、证书、token 或对端 allowlist，见 `ESPS3/main/radar/radar_uplink.c:344-412`。
- ROS2 `TcpServerTransport` 默认绑定 `0.0.0.0:8765`，`accept()` 不保留 peer address，没有认证/TLS，见 `ROS2_WIN/.../transport.cpp:58-109,142-188` 和 `config/bridge.yaml:4-11`。
- S3RD 的 CRC16-MODBUS 覆盖 version 到 payload，可检测随机损坏，但没有密钥或 MAC，不是发件人认证，见 `radar_uplink_protocol.c:7-18,95-112,150-155`。

**风险与现有保护**

- 在 live TCP 启用后，同网络攻击者可伪造正确 CRC/device/stream 的扫描或遥测。当前 ROS2 无 `/cmd_vel`/执行器控制，因此它当前是感知完整性风险，不是已确认车辆失控。
- tracked Kconfig 和 ROS2 YAML 分别默认 uplink 关闭、`transport: unconfigured`，这是有效安全默认；当前本机 ignored `sdkconfig` 已开启 uplink，不能将两者混为一个状态。

**建议（本轮未实施）**

- 位置：S3RD 传输层、Windows/ROS2 TCP listener 和部署网络。
- 原因：CRC 不能防伪造，未来感知/导航不能信任任意 LAN peer。
- 内容：最少绑定明确接口/防火墙 allowlist；产品化使用双向认证 TLS 或带密钥的帧 MAC，并将 device/stream 身份绑定到证书/密钥。
- 潜在影响：增加 S3 RAM/CPU、证书配置和 Windows 部署复杂度。
- 验证：非授权 peer、中间人、重放、证书过期和密钥轮换；授权失败不得产生 `/scan`。

### `R26-STALE-002` - Medium/High - S3 半开发送与 ROS2 接收时 freshness 组合后，旧包可被当成新包

**确定事实（CONFIRMED_SOURCE）**

- S3 雷达帧只在从 FIFO 首次取出时检查 `dequeue_age_ms <= 500`，然后编码为 pending packet，见 `radar_uplink.c:600-658,776-839`。
- 已连接 socket 返回 `RADAR_UPLINK_TX_WAIT` 时，pending packet 保留并只延迟一个 tick，没有重查 source timestamp/age，见 `radar_uplink.c:841-898` 和 `radar_uplink_tx.c:18-56`。
- S3RD 包携带 S3 `timestamp_ms`，ROS2 extractor 会解码保存，但 bridge 业务不使用它；全源搜索只在 `framing.cpp` 写入 `ReceivedFrame.timestamp_ms`。
- ROS2 `stale_after_ms` 根据帧在主机 assembler 中完成解析时记录的 `received_steady_ns` 计算，见 `framing.cpp:190-223` 和 `bridge_node.cpp:234-247`。

**推断风险（INFERRED_RISK）**

- 一个在 S3 socket 内等待很久的包最终到达主机时，`received_steady_ns` 是新的，可通过 host stale 门并进入 sequence/scan accumulation。当前无自主控制，但这是 live 地图/导航前的时效阻断。

**建议（本轮未实施）**

- 位置：S3 pending TX deadline、S3RD 时间合同、ROS2 host freshness。
- 原因：传输接收时间不是传感器采集时间。
- 内容：S3 对 pending 包持续重查 age 并过期丢弃/重建 socket；联合冻结单调时间基准、回绕和 S3-host 时钟映射；host 同时检查 source age 和 local queue age。
- 潜在影响：时钟同步/漂移管理和重连后 epoch 更复杂。
- 验证：对端长时间不读后恢复，注入时钟漂移/回绕/网络延迟，确认过期包不进 `/scan`。

### `R26-TCP-DOS-003` - Medium - ROS2 live server 的单客户端阻塞模型允许未认证连接长期占用唯一入口

**确定事实（CONFIRMED_SOURCE）**

- listener backlog 为 1；接受一个 client 后，同一 transport thread 在 blocking `recv()` 中持续服务它，直到返回 `<=0`，见 `transport.cpp:97,142-207`。
- client socket 没有 `SO_RCVTIMEO`、`SO_KEEPALIVE` 或应用层 idle timeout，也没有身份校验。
- 缓冲和 ready frame 队列有上界，可防无界内存增长，但不解决连接占用。

**建议与验证**

- 位置：ROS2 TCP transport。原因：一个静默/恶意 peer 不应阻止真实 S3 重连。
- 内容：认证后才占有 active slot，增加 handshake/idle/frame deadline 和 keepalive；新合法 epoch 到来时能中止旧连接。
- 潜在影响：超时过短可误断弱网连接。
- 验证：静默 client、slowloris、高频无效帧、S3 同时重连和 server stop/restart。

### `R26-TIME-004` - Medium - `/scan` 时间戳是主机处理时间，当前没有 S3/ROS 时钟合同

**确定事实（CONFIRMED_SOURCE）**

- S3 在雷达帧进 FIFO 时使用 `esp_log_timestamp()` 记 32-bit ms，并写入 S3RD header。
- ROS2 的 scan accumulation 使用 host `steady_clock` 算 revolution duration，用 `now()` 在 zero-packet 边界给 `LaserScan.header.stamp`，见 `bridge_node.cpp:332-360` 和 `scan_mapper.cpp:175-212`。
- S3 `timestamp_ms` 未转换到 ROS time，也没有对 32-bit 回绕和 S3 reset epoch 的时间合同。

**影响、建议与验证**

- 当前足以做有界 PoC/可视化，但不能声称 scan 与姿态/里程计已时间对齐。
- 位置：S3RD 时间字段和 ROS2 time mapper。原因：SLAM/传感器融合需要可比较的采集时间。内容：冻结 boot epoch + 64-bit monotonic/source time + host offset/drift 估计，或明确只用 host receipt 且标注不确定度。
- 潜在影响：协议 header 和 host 状态增加。验证：延迟/抖动/回绕/S3 复位下与参考时钟对比，给出 stamp error 上界。

### 第 26 轮已确认保护和非缺陷

- tracked 默认 `transport: unconfigured` 且 uplink Kconfig `default n`；没有显式 opt-in 时不启动 live network。
- ROS2 只创建 `/scan` 和 `/diagnostics` publisher，无 `/cmd_vel`、`/odom`、controller subscription 或 `controller_manager`；telemetry type 2 只作诊断。
- raw 和 telemetry 共用 S3RD sequence，ROS2 在分类前统一跟踪，与 S3 统一 uplink sequence 一致，不是缺陷。
- frame assembler 的 buffer/ready frame 数量有上界，无效长度/CRC/device/stream/flags 都有计数和拒绝。

## 11. 第 27 轮：双核、文档与跨域终审

### 跨域 liveness/lease 语义矩阵

| 信号 | 所有者/周期 | 当前能证明 | 不能证明 / 缺口 |
|---|---|---|---|
| App wheel heartbeat | macOS App / 100 ms | App main run loop 仍在定时调用发送 | GATT 已交付、S3/STM 已接受；App 卡住后下游无 V1 deadman |
| App V2 heartbeat/session | S3 / 建议 500 ms、TTL 3 s | V2 session 客户端仍活着 | 已执行 motion 仍在 command TTL 内；App 当前只用 V1 |
| S3 service loop | S3 / 1 tick = 10 ms | service task 获得调度的理想粒度 | 最坏执行/阻塞时间；TX enqueue 可在 IDF 内无界等待 |
| SRP sync heartbeat | S3 -> CM7 / 100 ms | S3-STM UART/SRP 传输仍能往返 | App 运动意图仍新鲜；不应续 operator-motion lease |
| CM7 S3-frame timeout | CM7 / 200 ms | 最近收到 codec-valid S3 帧 | 帧业务有效/受支持；当前业务校验前就续期 |
| S3 STM-frame timeout | S3 / 1500 ms | 最近收到 STM 帧 | 车轮、MotorBoard 或 App 健康 |
| MotorBoard init response timeout | CM7 / 1000 ms，仅 init sequence | 初始化某一步等待是否超时 | RUNNING 后 MSPD feedback 仍新鲜；当前无 runtime feedback watchdog |
| S3 BUS_OFF recovery delay | S3 / 100 ms | 安排 link recover 的时间 | 已发 zero/旧 pending 已完成；当前可早于 CM7 200 ms 强停重连 |

**终审结论：** 当前没有一个跨 App -> S3 -> CM7 -> MotorBoard 的 end-to-end motion lease。传输 heartbeat、session TTL、业务 command TTL 和执行器 feedback watchdog 必须分层，不能互相代替。

### 当前优先级与停止门

| 门 | 必须关闭的当前问题 | 未关闭时的允许范围 |
|---|---|---|
| 车辆 motion P0 | App 红色急停、BLE disconnect epoch、operator lease、CM7 统一非零准入、MotorBoard RUNNING/feedback watchdog/优先 stop、CPU fault 独立停机 | 只做 source/host/无执行器回放；不进入车辆运动验收 |
| 固件发布 P0 | clean checkout 调试头、S3 target 固定、uplink 启用模板、死 CMake/cache、启动 admission/READY 能力位 | 当前脏本机树只能作开发快照，不作可复现 release |
| SRP/UART P1 | BUS_OFF stop/recover、transport retry 上限、ACK/replay/epoch、parser half-frame、S3 TX 无界入队、CM7 recover/TX 串行 | 可做静态和主机 fault model；不声称 UART 长时稳定/失联必停 |
| IMU/control P1 | DualAHRS atomic publish、BMI dynamic health/recovery、BMI frame 合同、chassis task 接入和动态 MotorBoard dt | 保持静态诊断/无车台架；极性/单位不无证据修改 |
| BLE 产品化 P1 | 配对/加密/peer 授权、App commandReady/telemetryReady、with-response TX completion、GATT 长度合同、单 TX owner/congestion | 只限制受控 bench 连接，不声称未授权写入已阻断 |
| ROS2 live P0（启用自主前） | TCP 认证/机密、单 client DoS、source timestamp/freshness、pending TX deadline、S3RD type/schema 联合冻结 | 保持 `transport: unconfigured` 或受控 replay/PoC，不用于导航/安全感知 |
| CM4 启用前 P0 | option bytes、HSEM boot-ready/timeout、D2 物理分区、CM4F RTOS、版本化 mailbox/cache/reset epoch/heartbeat、单核回退 | 继续 CM7-only；不构建/烧录产品 CM4 任务 |

### 双核结论复核

- 第 18-26 轮没有产生足以改变旧推荐的新证据：当前仍应保持 CM7-only，先修单核 motion/fault 安全链，不用 CM4 转移掩盖单核 deadline/可观测缺口。
- CM4 Reset_Handler 仍在 HSEM 等待前初始化 `.data/.bss`，linker 仍从 D2 别名 `0x10000000` 开始，CM7 DMA 仍从同一物理 SRAM 的 `0x30000000` 别名开始。
- CM4 仍等 HSEM0 而 CM7 无 release，startup 仍错标 cortex-m7，无完整 RTOS/IPC/heartbeat。首阶段仍只允许 no-op + heartbeat，不迁移 sensor/UART/MotorBoard/安全 owner。

### canonical 文档漂移仍未关闭

- `.codex/MEMORY.md` 仍保留旧 `sc_frame`/BMI paused。
- `PROJECT_STATUS.md`、`DOCS/architecture/system.md`、`DOCS/esp32s3/ble.md` 仍声称 BLE callback/command relay 未连接，或 BMI323 paused。
- `DOCS/code_map.md` 仍引用不存在的 `Shared/SmartCarAppCore` V2 session 路径，并称 ROS2 无 runtime entry；`DOCS/ros2/ros2.md` 也仍把已存在的 bridge 完全归为未实现。
- `DECISION_LOG.md` 将 BMI323 paused 作为已决策事实，与当前 BMI Primary DualAHRS 源码冲突。
- 文档专项应以当前源码为真值更新 current status，但保留迁移前记录为明确 historical/deprecated；本轮未扩大到修改这些非审计文档。

### 结论去重、关闭与纠偏

- 保留并加强：App 急停/lease、S3 disconnect/BUS_OFF、CM7 direct wheel gate、MotorBoard feedback/stop/fatal、DualAHRS publish、BLE 授权、clean build、CM4 阻断。
- 新增独立发现：S3 target 未固定、SRP replay/retry/recover callback、S3 TX `portMAX_DELAY`、CM7 recover/TX 并发、App GATT-ready/with-response completion、GATT 1032/517 合同、S3 WDT 覆盖、S3RD 认证/时效/时间合同。
- 关闭旧误报：LSM303 DRDY 掩码已正确；姿态 freshness 已持续撤销 ready；BLE RX callback/遥测 relay 已连接；`data[7]/data[6]` 是正确 SRPv4 priority/type；DMA buffer 不在 DMA1 不可达 DTCM。
- 重新打开：当前 MotorBoard PID 已回退固定 50 ms dt，因此历史“动态 dt 完成”不适用当前工作树。

### 第 27 轮证据限制

- 本十轮只得到源码/配置/依赖实现静态证据。没有当前源码的 clean build，没有匹配烧录镜像，没有 UART/BLE/TCP 抓取，没有 RTOS HWM/WCET、传感器实物、MotorBoard watchdog 或车轮停机证据。
- 因此本文不宣称任何一个 High 风险已在实车触发，也不宣称现有保护已经台架验收。

## 12. 终稿检查表

- [x] 第 18-27 轮均有独立输入、证据和结论变化/关闭记录。
- [x] 所有 High 项回到当前源码重查行号。
- [x] 16 个关键 SRP/S3/ROS2/CM7/App 源文件在终稿前通过 SHA-256 稳定性复核。
- [x] 正式计划、总报告、findings/progress/task plan 一致。
- [x] 新章节不把静态证据升级为构建/设备/车辆验收。
- [x] `git diff --check` 与最终 Git 写入边界通过。
