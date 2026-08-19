# Smart Car 系统架构审查报告

## 审查结论

本报告是对当前 dirty checkout 的只读系统审查，汇总并复核：

- `VERIFY_ROOT_CAUSE_REPORT.md`
- `STM32_PROTOCOL_REPORT.md`
- `ESP32_CAL_REPORT.md`
- `APP_BMI_REPORT.md`
- 当前 STM32、ESP32-S3 和 macOS App live source

当前三端的静态接口仍可对齐，且本轮明确的最小修改没有改变 SCBP ID、payload、BLE UUID、校准状态转换或传感器采样节奏。`VERIFY_TIMEOUT` 的设备根因仍未闭环；现有日志改动是诊断改动，不是状态机修复。

重要架构事实：当前实现是“LSM303 主 AHRS + BMI323 独立采集/校准/振动/telemetry”，不是 BMI323 已接管的主姿态融合，也没有静态证据证明 LSM/BMI 已实现故障切换。若“BMI 主姿态、LSM 冗余”是目标架构，它仍是未实现/未验收的设计目标；本轮没有扩大到该架构变更。

## 证据分类

| 分类 | 当前结论 |
| --- | --- |
| 确定事实 | 源码中的调用、状态、ID、payload 长度、字段映射、日志位置和 UI 数据源。 |
| 已有设计 | SCBP-V3 STM-S3 传输、S3 独立 App BLE 帧、FFE2 telemetry、FFE3 日志；STM LSM303 AHRS 与双 IMU 数据模型。 |
| 推测/风险 | ACK 丢失或错配、任务调度、混合固件、物理链路和 `pendingTelemetry` 丢帧等运行时原因；均未被设备捕获证明。 |
| 未验证 | Flash、UART 原始帧、S3 运行状态、传感器质量、雷达 PWM、BLE 接收、UI 运行和车辆安全。 |

## 三端完整链路

```text
STM32H757
  imu_boot_manager / imu_runtime
    ├─ CAL_EVENT legacy type 0x18, payload {event_id}
    │    └─ SCBP-V3 MSG_ID 0x0401 -> UART -> S3 command_bridge
    └─ IMU_TELEMETRY legacy type 0x27, payload 30 bytes per sensor
         └─ SCBP-V3 MSG_ID 0x0207 -> S3 relay_telemetry

ESP32-S3
  SCBP parser/command_bridge
    ├─ calibration event -> radar_calibration_manager -> generic ACK 0x0005
    └─ telemetry -> App frame type 0x27 -> s3_ble_notify_send()
         └─ BLE service FFE0, TX notify characteristic FFE2

macOS App
  BLEManager FFE2 notification
    -> SmartCarProtocol decode (30-byte 0x27)
    -> TelemetryStore.IMUState.ingest(IMUTelemetry)
    -> pendingTelemetry -> snapshot.model.lsm303 / snapshot.model.bmi323
    -> ControlModeView / DeveloperModeView
```

### 链路静态证据

| 边界 | 当前 live source 事实 | 审查判断 |
| --- | --- | --- |
| STM 校准事件 | `imu_boot_manager.c` 的 `send_cal_event()` 发送一个字节；`s3_service.c` 调用 `sc_frame_encode()` 后 `uart_link_send()`。 | wire contract 未改变；传输成功仍需硬件捕获。 |
| STM-S3 CAL_EVENT | `SC_TYPE_CAL_EVENT=0x18` 映射 `SCBP_MSG_ID_CAL_EVENT=0x0401`；ACK 由 generic `0x0005` 携带原消息 sequence 相关联。 | ID、payload 和 ACK correlation 保持兼容。 |
| S3 事件接收 | `command_bridge.c` 对 `0x0401` 强制 `length==1`，设置 ACK context 后交给 `radar_calibration_manager_on_frame()`。 | 没有发现 id=1/2/3 的静态协议漂移。 |
| STM telemetry | `imu_runtime.c` 发送两个 30-byte `SC_TYPE_IMU_TELEMETRY=0x27` 帧，source 分别为 `IMU_SENSOR_LSM303` 和 `IMU_SENSOR_BMI323`；旧 LSM `SC_TYPE_IMU_STATUS` 仍保留。 | 新双 IMU 数据和历史 LSM 状态并存，不应混为同一 payload。 |
| S3-App telemetry | `command_bridge.c:relay_telemetry()` 将 SCBP `0x0207` 严格检查为 30 bytes，再编码 App type `0x27`，经 `s3_ble_notify_send()` 发送。 | STM-S3 SCBP 与 App BLE frame 是两个独立 framing contract。 |
| App decode/UI | `VehicleState.swift` 对 0x27 解码 source/valid flags；`TelemetryStore.flush()` 按 sensor 写入 `snapshot.model`；Control/Developer 页面读取模型。 | BMI 显示修复位置正确；物理接收尚未验证。 |

## 双 IMU、姿态与冗余边界

### 确定事实

1. `STM32H757/Middleware/Sensor/imu_manager.c` 明确标注 BMI323 sample 为 manager-local，不进入 LSM303 AHRS path；BMI 数据保存在 `bmi_data`/capture ring，并独立参与 dual calibration/vibration 结果。
2. `imu_manager.c` 的正常路径对 LSM 数据执行 `imu_calibration_apply()`、`imu_filter_update()`，随后在 filter ready 时调用 `attitude_update()`。
3. `imu_runtime.c` 的 ATTITUDE payload 标注为 `LSM303-derived AHRS`；`Middleware/Attitude/attitude.c` 消费的是 `imu_filter_get_output()`。
4. 两颗传感器都拥有初始化、有效性、校准和 vibration completion 状态，因此“同时采集/校准”成立；没有源码证据显示 BMI 已进入姿态融合或在 LSM 失败时自动接管。

### 架构判断

- 本轮 STM 诊断日志只读取 completion getter 和 transport 结果，不改变双 IMU ownership、采样频率、filter、AHRS 或校准状态机。
- App 只把 Control Mode BMI availability 从旧兼容字段切换到 `imu.model.bmi323.online`，不改变传感器主从关系。
- 因此本轮没有违反现有“LSM 主 AHRS + BMI 独立 telemetry/校准”实现边界；但不能把当前实现宣称为“BMI 主姿态 + LSM 冗余已完成”。实现该目标需要另行设计、审计和硬件验证。

## 校准状态机审查

ESP32 源码实际枚举为 `RADAR_WAIT_SYNC`、`RADAR_WAIT_ACK`、`RADAR_WAIT_EVENT`、`RADAR_CAL_DONE`、`RADAR_CAL_ERROR`；事件等待子状态为 static done、vibration done、cal complete。当前路径保持：

```text
BOOT_READY -> PWM=0 / WAIT_RADAR_ACK -> static calibration -> CAL_EVENT id=1 -> ACK
  -> each PWM level / ACK -> vibration capture -> CAL_EVENT id=2 -> ACK
  -> final id=2 ACK -> WAIT_CAL_COMPLETE -> CAL_EVENT id=3 -> ACK -> CAL_DONE
```

静态审查确认：

- id=1、id=2 仅在对应阶段和 payload length 1 条件下处理；重复/过早事件有专门分支。
- id=3 仍使用 `RADAR_CAL_COMPLETE_TIMEOUT_MS=5000`；STM `DUAL_IMU_VERIFY_WINDOW_MS` 也是 5000 ms。相等不等于运行时充分，当前没有捕获依据扩大窗口。
- 工作树中 `RADAR_CAL_VIBRATION_EVENT_TIMEOUT_MS` 已为 35000 ms；四份报告将 25 -> 35 s 视为既有 timing 修复。S3 recovery、错误回 `WAIT_SYNC`、最终完成 ACK 顺序等更大 dirty 改动必须单独归因，不能假设属于本轮最小修复。
- `VERIFY_TIMEOUT` 在 STM 只有到达 VERIFY deadline 且本地 flags/ACK 条件仍未满足时触发；新日志可区分 completion flags、pending event 与 event waiting，但没有运行时证据说明是哪一项实际失败。

## 本轮修改范围与影响

> 以下是针对本轮任务的最小修改记录；工作树中其他 dirty 文件不自动归入本轮。

| 修改文件 | 修改函数/位置 | 修改原因 | 潜在影响 | 验证方法 |
| --- | --- | --- | --- | --- |
| `STM32H757/Middleware/Calibration/imu_boot_manager.c` | `boot_log_verify_enter()`、`boot_log_verify_timeout()`；`imu_boot_manager_step()` 捕获 timeout 字段；`imu_boot_manager_on_cal_event_ack()` 触发 VERIFY entry log | 暴露 `static_lsm/static_bmi/vibration_lsm/vibration_bmi` 与 `verify_*_done/pending_event_id/event_waiting`，区分本地门槛和 id=3 transport | 一次 entry/timeout 的 bounded log；可能带来少量 log/调度扰动，不改状态、5 s VERIFY 窗口或采样 | 静态检查日志字段；CM7 configure/build；设备上关联 entry、id=3 TX、ACK RX 和 timeout |
| `STM32H757/Middleware/Communication/Services/s3_service.c` | `s3_service_send_boot_frame()` | 暴露 CAL_EVENT encode、UART HAL 状态与编码帧 sequence | 增加有限日志；保留 void transport callback，不能把日志当成端到端送达证明 | 静态确认 `0x0401`/一字节 payload；CM7 build；UART 原始帧和 ACK sequence capture |
| `IOS_APP/SmartCar_Control_MAC/Sources/SmartCar_Control_MAC/UI/ControlModeView.swift` | `VehicleCard.body` BMI `KeyValueRow` | 控制页使用权威 `IMUStateSnapshot.model.bmi323` | 只改变 BMI availability 显示源，不动 BLE/SCBP/model schema | `rg` 清除旧引用；Swift build/xcodebuild；FFE2 真实 0x27 帧验证 UI |
| `ESPS3/components/smartcar_service/radar_calibration_manager.c/.h` | 当前工作树既有 timeout/state/recovery 改动 | 审查记录，不在本系统审查中追加修改 | 35 s 与 recovery/final-ordering 会影响运行行为，需独立归因 | 目标快照 build；S3/STM 时间戳、PWM、id=1/2/3 capture |

禁止项复核：未发现本轮新增协议 ID、payload 格式、BMI/LSM 采样频率、强制 id=3 或绕过校准状态机的修改。

## 构建与验证证据

以下结果来自当前工作树的既有构建报告，属于 source/build evidence：

| 目标 | 结果 | 不能证明 |
| --- | --- | --- |
| STM32 CM7 | `cmake --preset Debug` + `cmake --build build/Debug --target Smart_Car_H757_CM7 -j2` 通过；FLASH 118604 B，RAM 54768 B | UART、电气链路、ACK、传感器和校准行为 |
| ESP32-S3 | ESP-IDF 5.5.4 `idf.py build` 通过；gateway binary 生成 | STM UART、雷达、PWM、状态机运行 |
| macOS SwiftPM | `swift build` 通过 | 完整 App bundle、BLE 接收和 UI 运行 |
| `xcodebuild` | 因 active developer directory 为 Command Line Tools 失败，见 `BUILD_ERROR_REPORT.md` | 不能把该命令失败解释为 Swift 源码语义失败 |
| `swift test` | 当前 package 没有 Tests target | 没有 XCTest 覆盖 |

未执行或未取得：flash、monitor、STM/S3 UART 抓包、FFE2/FFE3 BLE 抓包、LSM/BMI 采样质量、雷达 PWM 示波/测速、车辆运动与安全测试。

## 风险、剩余问题与运行验证计划

### 风险

1. `VERIFY_TIMEOUT` 根因仍未由设备日志证明。即使 S3 记录发送/ACK，也必须以 STM 的 sequence/source/destination correlation 和本地 flags 为准。
2. STM 与 S3 的 id=3 deadline 同为 5 s；在没有时间戳证据前扩大窗口可能掩盖 ACK 丢失、错配或任务饥饿。
3. SCBP pending ACK 为单槽位；其他 ACK-required 事务交错时存在覆盖 correlation 的结构性风险，当前没有证明它是本次故障根因。
4. App `pendingTelemetry` 是单一 optional 槽位；同一 200 ms flush 周期内连续收到 LSM/BMI 可能丢掉先到的一帧。它不是本次 BMI Disable 显示根因，不应借本轮重构。
5. 根工作区存在大量既有 dirty 改动；未锁定快照时，构建和设备结果无法可靠归因到本轮三处最小修改。
6. 架构文档本身说明 UART/BLE/硬件拓扑是 intended boundary，不是运行验收；实际物理路由、供电、DMA/RTOS 调度和车辆安全仍需专门测试。

### 最小运行验证顺序

1. 固定同一 STM32/S3/App 构建快照并记录 hash；在静止安全台架上烧录后采集 `IMU_VERIFY_ENTER`、`CAL_EVENT_TX`、S3 RX/ACK、STM ACK RX。
2. 验证每个 id=1/2/3 的 CAL_EVENT 帧均为 `MSG_ID=0x0401`、payload length 1、ACK_REQUIRED、CRC 正确，ACK sequence/source/destination/result 全部匹配。
3. 将最终 id=2 ACK 时间作为零点，检查 id=3 TX、S3 complete deadline、STM `IMU_VERIFY_TIMEOUT` 的相对时间；按 flags/pending/event_waiting 分支定位根因。
4. 通过 FFE2 注入或接收 source=LSM303/BMI323 的 30-byte 0x27 帧，确认 Control Mode 与 Developer Mode 同时读取 `snapshot.model`，valid flags 改变时显示随之变化。
5. 完整 Xcode 可用后重跑：

   ```bash
   DEVELOPER_DIR=/Applications/Xcode.app/Contents/Developer \\
   xcodebuild -scheme SmartCar_Control_MAC -destination 'platform=macOS' build
   ```

6. 硬件验证通过前，不宣称三端运行链路、校准成功、BMI 主姿态切换或车辆安全验收完成。

## 最终架构审查结论

本轮可接受的最小范围是：STM32 两处 bounded diagnostics 与 App 单字段状态源修复；ESP32 35 s vibration timeout 作为已有 dirty 改动单独复核，id=3 5 s 不因猜测扩大。协议 ID/payload、BLE FFE2/FFE3 接口、LSM303 AHRS ownership、BMI323 独立采集/校准边界均保持不变。

当前可以确认的是源码和构建层面的兼容性，不能确认 VERIFY_TIMEOUT 已在设备上修复，也不能确认 BMI Disable 已在 BLE/UI 运行时闭环。下一步必须依赖同一固件快照的 UART、S3、FFE2 和传感器硬件证据。
