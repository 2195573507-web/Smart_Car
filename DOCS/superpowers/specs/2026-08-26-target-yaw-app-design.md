# Target Yaw App 控制对接设计

## 状态

- 状态：已获用户确认，待实现
- 范围：iOS SwiftUI 与 macOS SwiftUI 控制客户端；为满足 BLE 写入契约，包含 S3 FFE1 特征能力声明的最小兼容修正，以及 CM7/S3 全零 Target Yaw 停止语义修正
- 不包含：UUID 迁移、控制模式重构、底盘 PD 参数调节、自动导航或车辆硬件验收

## 目标与约束

App 在现有 `WHEEL SPEED` 控制卡片中增加紧凑的 Target Yaw 小版块。用户可以设置 `-180...+180` 度目标角、查看 FFE2 实时航向，并用当前航向对齐。底盘差速模式下，启用锁航并给出非零线速度时，App 下发 App-BLE `0x2E`；停止时下发精确的 12 字节全零载荷。

既有 FFE0/FFE1/FFE2/FFE3 UUID、App-BLE V1/V2 外层帧、独立轮 `0x15`、MasterScale `0x2B` 和底盘差速 `0x2D` 保持兼容。Target Yaw 不成为新的 `ChassisControlMode`，而是底盘差速模式下的附加锁航状态。

## 协议与数据流

```text
SwiftUI control card
    -> SmartCarViewModel targetYawDeg / headingLockEnabled
    -> Shared MotionIntent
    -> BLEManager motion scheduler
    -> App-BLE type 0x2E, payload 12 bytes
    -> FFE1 Write Without Response (ordered ATT chunks)
    -> S3 App parser / command bridge
    -> SRP 0x17, ACK_REQUIRED, payload unchanged
    -> CM7 s3_service / chassis_task
```

`0x2E` 载荷只通过显式字节写入构造：

| 偏移 | 字段 | 编码 |
| ---: | --- | --- |
| 0 | `v_mm_s` | little-endian Float32 |
| 4 | `target_yaw_deg` | little-endian Float32，规范化到 `[-180, 180]` |
| 8 | `flags` | little-endian UInt32，固定为 `0` |

共享协议层新增 `chassisHeadingCommand = 0x2E` 和一个只接受有限浮点数、有限角度、固定零 flags 的构造函数。V1 外层帧总长度为 20 字节；V2 会话帧继续由现有 session wrapper 生成，并按协商 ATT 长度分片。

FFE1 当前源代码只声明 `WRITE`。由于本命令的冻结传输要求是 Write Without Response，S3 GATT 属性最小改为 `WRITE | WRITE_NR`；接收回调仍沿用已有复制入队路径，不在 GATT 回调中解析或控制电机。App 仅对 `0x2E` 的物理写入使用 `.withoutResponse`，其它命令继续 `.withResponse`，保留现有 ACK 顺序和错误处理。

## 共享状态与控制语义

`MotionIntent` 增加：

- `targetYawDeg: Float`
- `headingLockEnabled: Bool`

默认目标角为 `0`，锁航关闭。`chassisBaseSpeed` 仍是现有线速度来源；`wheelIndependent` 模式忽略这两个字段。

状态转换规则：

1. 目标角输入实时夹紧到 `[-180, 180]`；无效文本或非有限值不进入发送路径。
2. “当前航向对齐”只在 FFE2 姿态有效且未超时的情况下更新本地目标角。
3. 静止时对齐不发送非零巡航帧；之后第一次非零底盘速度帧携带新目标角。
4. 行驶中对齐且锁航已启用时，立即刷新当前 `0x2E` 帧，使用当前线速度。
5. 锁航关闭时，非零底盘差速仍发送原 `0x2D`；锁航打开时，非零底盘速度发送 `0x2E`。
6. 速度回中、摇杆释放、急停、断链、后台/终止和会话失效都清除本地锁航状态，并排队一个 `0x2E` 全零停止帧。既有四轮零速安全帧继续保留，但不得替代该 Target Yaw 清除帧。
7. 全零 Target Yaw 帧必须按原命令类型转发为 SRP `0x17`，并在 CM7 `chassis_task` 清除 active target、清零角速度和轮速；不能被 S3 改写成 SRP wheel-speed 帧，也不能被 CM7 当作“零线速度但仍锁航”的目标。

## UI 设计

不增加页面、不改变原卡片布局。两端在现有速度控制区后插入同一紧凑版块：

- 标题：`定向巡航 / TARGET YAW`
- 当前状态：`AHRS VALID/WAITING`，实时 `当前航向 xx.x°`
- 角度输入：`TextField` 或等价数字输入，步进控件范围 `-180...180`
- 快捷操作：`当前航向对齐`
- 开关：`巡航锁航`

控件在 BLE 未连接、姿态无效或处于独立轮模式时按现有禁用规则处理。原有底盘模式选择、速度滑块、四轮目标和速度图表位置不变。急停按钮继续保持独立、醒目且可随时触发。

## 代码边界

主要修改位置：

- `Shared/SmartCarAppCore/Sources/SmartCarAppCore/AppBLECore.swift`：类型 `0x2E` 与显式 LE payload builder。
- `Shared/SmartCarAppCore/Sources/SmartCarAppCore/AppStores.swift`：扩展 `MotionIntent`。
- 两端 `BLEManager.swift`：Target Yaw motion 编码、命令级 `.withoutResponse`、停止识别与队列分片。
- 两端 `SmartCarViewModel.swift`：目标角、锁航、对齐、回中/生命周期清除。
- iOS `ControlDashboardView.swift` 与 macOS `WheelSpeedControlCard.swift`：紧凑 Target Yaw 小版块。
- `ESPS3/components/s3_ble/s3_ble.c`：FFE1 `WRITE_NR` 能力声明。
- `ESPS3/components/smartcar_service/command_bridge.c`、`STM32H757/Application/Chassis/chassis_task.c`：全零 `0x2E` 停止语义的最小修正。

不修改：FFE UUID、外层帧布局、DualAHRS 数据格式、底盘 PD 增益、轮速映射、CubeMX/IOC 和硬件引脚。

## 错误与安全处理

- App 不发送 NaN、无穷大、越界 yaw 或非零 flags。
- BLE 队列满、会话未就绪或连接丢失时不发送非零运动；停止帧仍按现有安全队列优先级尝试发送。
- `.withoutResponse` 不提供 ATT 写确认，协议层 V2 ACK/FFE2 诊断仍作为网关/STM 接收证据；UI 不把本地排队当作车辆已执行。
- S3 收到错误长度、非有限数或非零 flags 拒绝并保留现有 ACK/日志路径。
- CM7 端链路、姿态 admission 或输出 gate 失效时清除 Target Yaw 并输出零轮速。

## 验证计划

### 主机/源代码

- Shared 单元测试：`0x2E` 类型、12 字节 payload、LE float、flags=0、全零停止 payload 和角度边界。
- Swift 编译：Shared package、iOS package/source target、macOS package。
- macOS 运行验证：`SmartCar_Control_MAC/script/build_and_run.sh --verify`，确认 staging bundle 包含新控件。
- S3 隔离 ESP-IDF 构建，确认 GATT 属性和命令桥接编译通过。
- CM7 `STM32H757/CM7/build/Debug` clean build，确认停止清除修正无回归。

### 设备/集成（不由本次构建自动证明）

- BLE capture：确认 FFE1 对 `0x2E` 使用 Write Without Response，V1/V2 分片可重组。
- S3/STM UART capture：确认 App `0x2E` -> SRP `0x17` 且 flags=0、ACK_REQUIRED。
- FFE2 telemetry：确认对齐读取的 yaw 与 CM7 使用同一角度约定。
- 台架安全测试：回中、急停、断链、会话失效后轮速为零且 heading lock 不恢复。
- 车辆测试：仅在上述证据完成后验证直线航向保持和 `[-180,180]` 跨界。

## 风险与取舍

- 命令级无响应写入会降低 ATT 往返延迟，但缺少链路层确认；V2 协议 ACK 和安全停止队列承担上层观测与恢复。
- 共享 `MotionIntent` 扩展会触及两端 ViewModel，但消除双端协议漂移，且不改变既有公开 BLE UUID/帧接口。
- 现有工作树包含大量用户未提交修改；实施时只触碰上述文件并在构建前后检查目标文件差异。
