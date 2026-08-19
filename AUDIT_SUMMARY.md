# Smart Car 双问题修复审计汇总

日期：2026-08-18  
阶段：第一阶段审计已完成；用户随后确认继续，第二阶段最小修改与构建验证已执行。

## 1. 审计输入

| 专家 | 报告 | 结论状态 |
| --- | --- | --- |
| STM32 Calibration | `VERIFY_ROOT_CAUSE_REPORT.md` | 已完成，只读 |
| STM32 Communication | `STM32_PROTOCOL_REPORT.md` | 已完成，只读 |
| ESP32 Radar Calibration | `ESP32_CAL_REPORT.md` | 已完成，只读 |
| macOS App SwiftUI | `APP_BMI_REPORT.md` | 已完成，只读 |
| System Architecture | `SYSTEM_REVIEW_REPORT.md` | 已完成，只读 |

## 2. 问题一：VERIFY_TIMEOUT

### 确定事实

- `verify_lsm_done` / `verify_bmi_done` 是 `imu_boot_manager_step()` 的局部值。
- 它们分别由以下条件计算：
  - `imu_calibration_is_lsm_complete() && imu_vibration_is_lsm_complete()`
  - `imu_calibration_is_bmi_complete() && imu_vibration_is_bmi_complete()`
- `static_lsm/static_bmi/vibration_lsm/vibration_bmi` 不是当前源码中的实际变量，而是上述四个 completion getter 的逻辑别名。
- 最终 id=2 ACK 被 STM 相关联接受后进入 VERIFY；当前源码不会因进入 VERIFY 清除校准/振动模块已保留的完成结果。
- 若四个完成结果为真但 id=3 未被 STM 接受，正常路径应进入 `CAL_EVENT_TIMEOUT` 分支；仅凭现有源码无法证明 `VERIFY_TIMEOUT` 的具体运行时原因。
- `SC_TYPE_CAL_EVENT=0x18` 仍映射到冻结的 `SCBP_MSG_ID_CAL_EVENT=0x0401`，payload 仍为单字节 event id。
- ESP32 当前工作区已有 `RADAR_CAL_VIBRATION_EVENT_TIMEOUT_MS` 从 25 秒到 35 秒的 dirty 改动；静态时序（约 2 秒稳定 + 30 秒采集）支持该改动，但它不是本次审计新增的修改。
- STM 与 ESP32 的 id=3 完成等待窗口均为 5 秒；静态证据不足以直接扩大该窗口。

### 可能根因（未证实）

1. S3 侧“ACK 成功”日志不等于 STM 严格按 `msg_id/seq/src/dst` 相关联接受。
2. 编码失败、UART 未就绪/超时、ACK 丢失或错配当前缺少分层日志。
3. 运行中的 STM/S3 固件可能混合或不是当前源码构建物。
4. VERIFY 期间任务调度、复位/并发状态破坏等运行时因素尚无设备证据。

上述均为假设，不能在没有捕获的情况下当作确定根因。

## 3. 问题二：BMI Disable 显示

### 确定事实

- `ControlModeView.swift:195` 已改为读取 `imu.model.bmi323.online`；修改前读取的是 `IMUStateSnapshot` 的旧兼容字段。
- 双 IMU 权威显示模型是 `imu.model.bmi323`；`DeveloperModeView.swift` 已从该模型读取 BMI 状态。
- `TelemetryStore.flush()` 中 `pendingBMI323` 写入兼容字段，`pendingTelemetry` 写入 `next.model.bmi323`/`next.model.lsm303`，没有发现 LSM/BMI 互相覆盖模型的路径。

### 根因

控制页 BMI 状态显示使用了旧兼容字段，而不是双 IMU telemetry 的 `model` 字段，导致 UI 可能显示 `Disable`/离线。

## 4. 批准候选修改

| 文件 | 函数/位置 | 修改原因 | 修改内容 | 潜在影响 |
| --- | --- | --- | --- | --- |
| `STM32H757/Middleware/Calibration/imu_boot_manager.c` | `enter_verify_phase_locked()`、`imu_boot_manager_step()`（必要时 ACK 接收边界） | 暴露 VERIFY 进入时四个完成源及超时分支状态 | 增加一次 `IMU_VERIFY_ENTER static_lsm=... static_bmi=... vibration_lsm=... vibration_bmi=...`；超时时增加 `IMU_VERIFY_TIMEOUT verify_lsm_done=... verify_bmi_done=... pending_event_id=... event_waiting=...` | 少量日志 CPU/栈/UART 带宽；不改状态、超时、采样或协议 |
| `STM32H757/Middleware/Communication/Services/s3_service.c` | `s3_service_send_boot_frame()` | 区分 CAL_EVENT 编码失败与 UART/HAL 失败 | 仅对 CAL_EVENT 增加 `CAL_EVENT_TX`、`CAL_EVENT_ENCODE_FAIL`、`CAL_EVENT_UART_FAIL` 有界诊断；从现有帧读取 seq | 增加日志量；不改 `0x0401`、单字节 payload、CRC、ACK 相关逻辑 |
| `IOS_APP/SmartCar_Control_MAC/Sources/SmartCar_Control_MAC/UI/ControlModeView.swift` | `VehicleCard.body` | 使用双 IMU 权威模型显示 BMI 在线状态 | `imu.bmi323.online` → `imu.model.bmi323.online` | 仅改变 UI 数据源；不改 BLE、模型、协议或采样 |
| ESP32 | 暂不新增文件 | 35 秒 id=2 timeout 已在工作区，需单独确认归属 | 不扩大 `RADAR_CAL_COMPLETE_TIMEOUT_MS=5000`；等待运行时捕获 | 避免用扩大窗口掩盖 ACK/调度问题 |

## 5. 明确不修改

- 不修改任何协议 ID、payload 格式、CRC、BLE UUID 或三端接口。
- 不改变 BMI/LSM 采样频率、校准状态机、状态转换或超时策略（除非后续确认明确授权必要的 ESP32 timeout 修复）。
- 不强制发送 CAL_EVENT id=3，不绕过完成标志或校准状态。
- 不删除兼容字段，不重构 `TelemetryStore`，不合并 App BLE 与 STM-S3 帧封装。
- 不回滚工作区既有 dirty 改动，不进行大范围重构，不提交 Git，不烧录设备。

## 6. 第二阶段修改与验证结果

1. `git diff --check` 通过；协议常量仍为 `SCBP_MSG_ID_CAL_EVENT=0x0401`、`SC_TYPE_CAL_EVENT=0x18`，event payload 仍为单字节。
2. STM32：`cmake --preset Debug` 与 `cmake --build build/Debug --target Smart_Car_H757_CM7 -j2` 通过；本次复核为 `ninja: no work to do`。
3. ESP32：ESP-IDF 5.5.4 `idf.py build` 通过，gateway binary 为 `0xB25C0` bytes。
4. App：`swift build` 通过；`xcodebuild` 因本机 active developer directory 为 Command Line Tools 失败，详见 `BUILD_ERROR_REPORT.md`。
5. 全局搜索未发现旧的 `imu.bmi323.online` UI 引用。
6. 设备阶段仍需捕获 VERIFY entry/timeout、CAL_EVENT TX seq、UART 状态、S3 ACK seq、id=3 TX/RX、BLE FFE2 telemetry 和 BMI UI。构建成功不等于硬件/运行时验收。

## 7. 当前状态与剩余门槛

- 审计报告：已生成。
- 源码修改：已执行最小范围诊断日志和 App 单字段状态源修复；未修改协议 ID、payload、状态转换、采样频率或 BLE 接口。
- 构建：STM32、ESP32、SwiftPM 通过；完整 `xcodebuild` 受工具链缺失阻断。
- 硬件/设备：未执行。
- 下一步：使用同一固件快照进行 flash 后 UART/S3/BLE/传感器运行验证；在取得证据前不宣称 VERIFY_TIMEOUT 根因已闭环。
