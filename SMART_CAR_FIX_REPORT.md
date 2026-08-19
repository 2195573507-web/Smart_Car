# Smart Car 双问题修复报告

日期：2026-08-18

## 1. 问题一：VERIFY_TIMEOUT

### Root cause status

静态源码没有证明一个确定的本地状态机根因。当前实现中：

- `verify_lsm_done` / `verify_bmi_done` 在
  `STM32H757/Middleware/Calibration/imu_boot_manager.c::imu_boot_manager_step()`
  中由四个 completion getter 组合得到。
- `static_lsm/static_bmi/vibration_lsm/vibration_bmi` 不是独立变量，分别对应
  `imu_calibration_is_{lsm,bmi}_complete()` 和
  `imu_vibration_is_{lsm,bmi}_complete()`。
- 最终 id=2 ACK 被 STM 严格相关联接受后进入 VERIFY，不会清除已保留的校准/振动完成结果。
- 因此“id=1 ACK、id=2 ACK、进入 VERIFY、随后 VERIFY_TIMEOUT”无法仅由正常静态源码流程解释。
  仍需区分 ACK 误观察/错配、编码或 UART 失败、任务调度、混合固件或运行时状态破坏。

### Files/functions changed

| 文件 | 函数 | 修改 |
| --- | --- | --- |
| `STM32H757/Middleware/Calibration/imu_boot_manager.c` | `boot_log_verify_enter()`、`imu_boot_manager_on_cal_event_ack()` | 最终 id=2 ACK 进入 VERIFY 后，记录 `IMU_VERIFY_ENTER static_lsm=... static_bmi=... vibration_lsm=... vibration_bmi=...`。仅增加日志，不改变 phase/deadline。 |
| `STM32H757/Middleware/Calibration/imu_boot_manager.c` | `imu_boot_manager_step()`、`boot_log_verify_timeout()` | VERIFY deadline 分支在 `fail_locked()` 前捕获 `verify_lsm_done`、`verify_bmi_done`、`pending_event_id`、`event_waiting`，解锁后记录 `IMU_VERIFY_TIMEOUT ...`。不改变原失败分支。 |
| `STM32H757/Middleware/Communication/Services/s3_service.c` | `s3_service_send_boot_frame()` | 对 `SC_TYPE_CAL_EVENT` 增加 `CAL_EVENT_TX`、`CAL_EVENT_ENCODE_FAIL`、`CAL_EVENT_UART_FAIL`，序号来自已编码帧的 byte 8，结果来自现有编码/HAL 状态。 |

### Reason and impact

这些日志把本地 completion、pending event、编码结果、UART HAL 结果和 wire sequence 分层暴露，能够区分本地 VERIFY 门控与跨端 ACK 问题。日志使用现有有界队列，STM32 构建中无新增 warning；没有改变状态转换、超时常量、采样频率、协议 ID、payload 或 ACK 关联规则。

## 2. 问题二：BMI Disable

### Root cause

`ControlModeView` 的 `VehicleCard` 原先读取 `imu.bmi323.online`，这是 `IMUStateSnapshot` 的旧兼容字段。双 IMU telemetry 的权威模型是 `imu.model.bmi323`；`DeveloperModeView` 已使用该模型。

### Files/functions changed

| 文件 | 函数/位置 | 修改 |
| --- | --- | --- |
| `IOS_APP/SmartCar_Control_MAC/Sources/SmartCar_Control_MAC/UI/ControlModeView.swift` | `VehicleCard.body` | `imu.bmi323.online` 改为 `imu.model.bmi323.online`。 |

`TelemetryStore.flush()` 中 `pendingBMI323` 仍写兼容字段，`pendingTelemetry` 按源分别写 `model.lsm303`/`model.bmi323`；本次没有重构或改变其覆盖语义。

## 3. Protocol impact

- `SCBP_MSG_ID_CAL_EVENT` 保持 `0x0401`。
- `SC_TYPE_CAL_EVENT` 保持 `0x18`，event payload 仍为单字节 id=1/2/3。
- `SC_CAL_EVENT_STATIC_CAL_DONE`、`SC_CAL_EVENT_VIBRATION_STEP_DONE`、`SC_CAL_EVENT_COMPLETE` 均未改变。
- frame sequence、CRC、ACK envelope、BLE FFE2、`SC_TYPE_IMU_TELEMETRY=0x27` 和 payload 格式均未改变。
- BMI UI 修复不修改 STM32、ESP32 或 App 的接口定义。
- 当前工作区已有 ESP32 `RADAR_CAL_VIBRATION_EVENT_TIMEOUT_MS` 25 秒→35 秒改动；本轮未重写该 dirty 变更，也未扩大 `RADAR_CAL_COMPLETE_TIMEOUT_MS=5000`。

## 4. Build and validation evidence

| Target | Command/result | Evidence boundary |
| --- | --- | --- |
| STM32 CM7 | `cmake --preset Debug`; `cmake --build build/Debug --target Smart_Car_H757_CM7 -j2` 通过；FLASH 118604 B / 1 MB，RAM 54768 B / 128 KB | 证明当前源码可配置/编译；不证明 UART、传感器或校准运行 |
| ESP32-S3 | `source .../esp-idf/export.sh && idf.py build` 通过；`smartcar_s3_gateway.bin` 0xB25C0 bytes | 证明当前工作树可构建；不证明雷达、UART 或 ACK 运行 |
| macOS App | `swift build` 通过；`swift test` 因无 Tests target 失败 | SwiftPM 源码构建证据；不证明 staged bundle/UI/BLE，也没有 XCTest 覆盖 |
| `xcodebuild` | 失败，见 `BUILD_ERROR_REPORT.md` | 当前 active developer directory 是 Command Line Tools，不是完整 Xcode |
| Static | `git diff --check` 通过；无 `imu.bmi323.online` 旧 UI 引用 | 证明文本/引用一致性 |

未执行 flash、monitor、UART/BLE 抓包、传感器采样、雷达 PWM、车辆测试或 staged App 运行验证。

## 5. Runtime validation plan

1. 烧录同一批当前 STM32 与 ESP32 构建物，在安全静止台架上采集 `CAL_EVENT_TX` 的 id/seq/result。
2. 对照 STM32 UART 原始帧确认 `MSG_ID=0x0401`、单字节 payload、ACK_REQUIRED、CRC 和 sequence。
3. 对每个 id=1/2/3 记录 S3 接收、ACK source/destination/seq/result 与 STM 端 `imu_boot_manager_on_cal_event_ack()` 结果。
4. 在最终 id=2 ACK 后检查 `IMU_VERIFY_ENTER` 四个 completion 值；若 0，追查对应校准/振动质量；若全 1 但无 id=3，追查 pending/调度/传输；若 id=3 已发送仍 timeout，追查 ACK 相关性或固件身份。
5. 检查 `IMU_VERIFY_TIMEOUT` 的四个字段与 S3 deadline 时间戳，确认是否为本地 flag、pending event 或跨端传输问题。
6. 通过 BLE FFE2 注入/接收一帧 BMI telemetry，确认 Control Mode 与 Developer Mode 使用同一 `model.bmi323.online` 状态。
7. 配置完整 Xcode 后重跑 `xcodebuild`，并按需要执行 staged-bundle 验证。

## 6. Remaining risks

- VERIFY 的实际运行时根因尚未被设备日志证明；本次补丁是诊断修复，不是未经证据支持的状态机改写。
- STM/S3 id=3 都是 5 秒窗口，当前没有实测依据扩大窗口。
- 根工作区有大量既有 dirty 改动，尤其 S3 recovery/final-completion 变更未归入本次补丁；后续运行验证需锁定构建快照。
- `xcodebuild` 受本机 Xcode 工具链缺失阻断；`swift build` 不能替代完整 App bundle/UI 验证。
- 未烧录和未做硬件捕获，因此不能宣称三端运行链路或车辆安全验收完成。
