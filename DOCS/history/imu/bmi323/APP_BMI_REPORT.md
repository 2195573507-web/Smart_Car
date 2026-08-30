# macOS App BMI 状态审计

> 历史快照说明：本文中的 STM32-S3 UART 术语来自 SRPv4 切换前的审计。
> App BLE `0x27` 是独立的 App envelope，仍不等同于 STM32-S3 UART message ID。

## 范围与证据边界

本报告只读检查了：

- `IOS_APP/SmartCar_Control_MAC/Sources/SmartCar_Control_MAC/UI/ControlModeView.swift`
- `.../Model/VehicleState.swift`
- `.../Stores/TelemetryStore.swift`
- `.../Model/SmartCarProtocol.swift`
- `.../UI/DeveloperModeView.swift`

未修改 Swift 源码、协议常量或数据模型。结论是当前源码静态事实；BLE 接收、S3 转发和运行中的 UI 未验证。

## 当前数据流

```text
App BLE IMU telemetry type `0x27`
  -> ESP32-S3 STM frame relay
  -> BLE FFE2 notification
  -> SmartCarProtocol decode
  -> TelemetryStore.ingest
  -> IMUStateSnapshot.model
  -> ControlModeView / DeveloperModeView
```

`TelemetryStore.swift` 同时保留旧兼容字段 `IMUStateSnapshot.bmi323` 和双 IMU 模型
`IMUStateSnapshot.model.bmi323`。`TelemetryStore.flush()` 中，`pendingBMI323` 写入旧兼容
快照字段，`pendingTelemetry` 按传感器写入 `next.model.lsm303` 或 `next.model.bmi323`，
两条写入路径不是同一字段覆盖。

## 发现

| 位置 | 现状 | 影响 |
| --- | --- | --- |
| `ControlModeView.swift:195` | `imu` 是 `IMUStateSnapshot`，BMI 行读取 `imu.bmi323.online` | 使用旧兼容字段，可能显示 `Disable`/离线，即使双 IMU 模型已收到有效 BMI 数据 |
| `DeveloperModeView.swift:31` | 使用 `telemetryStore.imu.snapshot.model.bmi323` | 已使用正确的新数据源 |
| `TelemetryStore.swift:10-14` | `IMUStateSnapshot` 同时存在 `bmi323` 与 `model.bmi323` | 兼容结构保留是现状，不建议本次重构删除 |
| `TelemetryStore.swift:122-171` | `pendingBMI323` 与 `pendingTelemetry` 分开提交 | 未发现 LSM/BMI 互相覆盖 `model` 的路径 |
| 全局 BMI/online 搜索 | 未发现其他 `imu.bmi323.online` UI 引用；其余入口读取各自模型或本地传感器值 | 修复面可限制在 `ControlModeView.swift` 一处 |

## 最小修复建议

- 修改文件：`IOS_APP/SmartCar_Control_MAC/Sources/SmartCar_Control_MAC/UI/ControlModeView.swift`
- 修改位置/函数：`VehicleCard.body`
- 修改内容：将 `imu.bmi323.online` 改为 `imu.model.bmi323.online`。
- 原因：使控制页与双 IMU telemetry 的权威模型一致。
- 潜在影响：仅改变 BMI 状态显示来源；不改变 BLE、SRPv4 UART payload、采样频率、LSM/BMI 数据接收或校准状态机。
- 验证：静态搜索确认无旧引用；`swift build`/`xcodebuild`；接收一帧 BMI telemetry 后确认控制页与 Developer Mode 一致。运行 UI/BLE 仍需设备证据。

## 不应扩大范围

- 不删除 `IMUStateSnapshot.bmi323` 兼容字段。
- 不重构 `TelemetryStore` 的 pending 合并机制。
- 不修改 App BLE `0x27`、BLE FFE2 或 payload 格式。

## 2026-08-18 独立复核增补

以下结论由当前 dirty checkout 的源码复核得到，未把既有报告或构建结果当作运行时证据。

### 确定事实与证据

| 文件/行 | 确定事实 |
| --- | --- |
| `Model/SmartCarProtocol.swift:9-27` | App 保留 `FrameType.imuTelemetry = 0x27`；协议常量未因本次显示修复改变。 |
| `Model/VehicleState.swift:645-671` | 0x27 仅接受 30-byte payload；source `0x01/0x02` 分别解码为 LSM303/BMI323，BMI 的 `online` 由 accel 与 gyro valid flags 同时为真得到。 |
| `BLE/BLEManager.swift:73-76,188-194,348-365` | FFE2 (`txCharacteristicUUID`) 的通知进入 receive pipeline，经 `DecodedMessage` 后调用 `TelemetryStore.ingest`。这里证明 App 端接收/分发路径；STM32/ESP32 实际转发仍需设备捕获。 |
| `Stores/TelemetryStore.swift:118-176,531-541` | `IMUState` 分别维护旧 `pendingBMI323`/`pendingLSM303` 和新的 `pendingTelemetry`；0x27 消息进入后者，flush 时按 enum case 写入 `snapshot.model.bmi323` 或 `snapshot.model.lsm303`。 |
| `UI/ControlModeView.swift:183-196` | BMI 行当前读取 `imu.model.bmi323.online`；这是本次显示修复的实际生效位置。LSM 行仍读兼容字段 `imu.lsm303.online`，不在本次 BMI 修复范围内。 |
| `UI/DeveloperModeView.swift:30-33,170-221` | BMI/LSM 调试卡均从 `snapshot.model` 取数据，并按 `BMI323Data`/`LSM303Data` 的 `online` 字段显示；未发现旧 `imu.bmi323.online` 引用。 |

### BMI/online/Disable 全局审计

- 在 `IOS_APP/SmartCar_Control_MAC/Sources/SmartCar_Control_MAC` 内未发现其他 `imu.bmi323.online` 旧 UI 引用。
- `DeveloperModeView` 的 `.disabled(...)` 仅用于连接状态下的雷达控制，不是 BMI 可用性显示。
- `TelemetryStore.swift` 的 `IMUStateSnapshot` 兼容字段（`bmi323`/`lsm303`）仍由旧 `IMU_STATUS` 路径维护；保留它们不会改变新 0x27 模型的写入目标，也不建议在本轮删除。

### pendingTelemetry 覆盖判定

没有发现 LSM/BMI 交叉写入：`flush()` 的 `.lsm303` 分支只写 `model.lsm303`，`.bmi323` 分支只写 `model.bmi323`。但是 `pendingTelemetry` 是单一 optional 槽位（`ingest` 每次直接赋值），因此同一个 200 ms flush 周期内若先收到 LSM 再收到 BMI，LSM 那一帧会被丢弃；反向顺序同理。该行为是 UI 抽样/丢帧风险，不是 BMI 显示为 Disable 的根因，也不应在本次协议兼容修复中扩展为重构。

### 本次修改记录

- 修改文件：`IOS_APP/SmartCar_Control_MAC/Sources/SmartCar_Control_MAC/UI/ControlModeView.swift`。
- 修改函数/位置：`VehicleCard.body`，BMI `KeyValueRow`。
- 修改内容：`imu.bmi323.online` -> `imu.model.bmi323.online`。
- 原因：旧兼容 `IMUData` 不接收 0x27 dual-IMU telemetry；权威 BMI 状态在 `IMUDataModel.bmi323`。
- 潜在影响：仅影响控制页 BMI availability 文本；不改变 SRPv4 UART type/payload、BLE characteristic、采样频率、校准状态机或传感器数据接收。
- 验证方法：静态 `rg` 确认无旧 UI 引用；使用真实 FFE2 notification 注入 source=0x02 且两 valid flags=1 的 0x27 帧，确认 Control Mode 与 Developer Mode 同时显示 ONLINE；断开 BLE 或发送任一 valid flag=0 的帧，确认显示回到离线/Disable。Swift 编译只能证明源码集成，不能替代 BLE/设备验证。

### 证据边界

本审计未连接 STM32、ESP32 或车辆硬件，未 flash、未采集 UART/BLE 原始帧；因此 STM32 -> ESP32 的物理转发和运行时频率/丢帧情况仍未验证。
