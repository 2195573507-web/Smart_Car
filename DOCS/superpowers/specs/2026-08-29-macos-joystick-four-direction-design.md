# macOS 摇杆四方向轮速控制设计

## 状态

实现已完成：macOS SwiftPM 构建与打包 bundle 启动验证通过；BLE、UART、
STM32 应用和车辆行为仍为 `UNVERIFIED`。本文只覆盖当前
`IOS_APP/SmartCar_Control_MAC` 可执行目标中的现有圆形摇杆。

## 背景与现状

`ControlModeView` 已包含 `VirtualJoystick`，但它当前发送 App BLE
`CONTROL(0x01)` 帧。现行 ESP32-S3 `command_bridge` 处理的是 App
`WHEEL_SPEED_CMD(0x15)`、PID 和雷达速度，未处理旧 `CONTROL(0x01)`，所以
当前摇杆的方向命令不能沿已实现的 S3 -> STM32 轮速路径得到确认。

现有轮速路径保持不变：App 发送四个 little-endian `float32` 轮速，S3
转发为 SCBP `WHEEL_SPEED_CMD(0x110)`，STM32 接收顺序为
`[M1:RR, M2:RF, M3:LR, M4:LF]`。

## 目标

1. 保留现有摇杆外观、拖动手势和四方向档位体验。
2. 将摇杆方向转换为现有 BLE `0x15` 四轮速度命令。
3. 使用现有速度滑块控制摇杆输出幅值。
4. 回中、方向切换、释放、断链和 App 生命周期变化均保持零速安全行为。
5. 不新增协议类型，不修改 S3、STM32、GPIO、轮序和底层控制参数。

## 非目标

- 不实现连续二维速度/角速度控制。
- 不新增摇杆页面或替换现有 UI 布局。
- 不在本次工作中修改全局急停协议或宣称硬件急停能力。
- 不根据构建结果宣称 BLE、UART、电机或车辆行为已验收。

## 控制语义

### 四方向档位

设 `v = speed_percent / 100 * 800.0 mm/s`，其中 `speed_percent` 来自
现有 `0...100` 速度滑块。轮速目标顺序固定为
`[M1:RR, M2:RF, M3:LR, M4:LF]`：

| 摇杆状态 | M1 RR | M2 RF | M3 LR | M4 LF |
| --- | ---: | ---: | ---: | ---: |
| 前进 | `+v` | `+v` | `+v` | `+v` |
| 后退 | `-v` | `-v` | `-v` | `-v` |
| 左转原地 | `+v` | `+v` | `-v` | `-v` |
| 右转原地 | `-v` | `-v` | `+v` | `+v` |
| 回中/释放 | `0` | `0` | `0` | `0` |

正负号是当前软件控制约定。电机安装方向、板卡通道方向和编码器方向仍
需要台架实测确认；若物理左右相反，只能在明确的映射层修正，不能修改
`WHEEL_TRIM`、轮序或底层 PID 来掩盖问题。

### 死区与状态转换

- 保留现有 `18 pt` 拖动阈值。
- 阈值内从有方向状态回到中心时立即发送一次全零 `0x15`，停止轮速心跳。
- 方向档位变化时立即切换目标，并由现有发送合并逻辑发送新目标。
- 手势结束时再次确保全零目标。
- 摇杆控件只有在 BLE 状态为 `connected` 时可操作。

## 数据流

```text
VirtualJoystick
  -> SmartCarViewModel.applyJoystickCommand()
  -> wheelTargets[4]
  -> BLEManager.sendWheelSpeeds()
  -> App BLE 0x15 (4 x f32 LE)
  -> S3 command_bridge
  -> SCBP 0x110
  -> STM32 wheel targets / MotorBoard
```

View 只报告四方向意图；四轮映射、速度缩放和停止调用由
`SmartCarViewModel` 负责。现有 50 ms 首次发送合并、100 ms 轮速心跳、
断链/后台归零逻辑继续复用，不在 View 中创建新的 Timer。

## 修改边界

允许修改：

- `IOS_APP/SmartCar_Control_MAC/Sources/SmartCar_Control_MAC/UI/ControlModeView.swift`
- `IOS_APP/SmartCar_Control_MAC/Sources/SmartCar_Control_MAC/ViewModels/SmartCarViewModel.swift`

不修改：

- `SmartCarProtocol` 的 App BLE 外层格式和 `0x15` payload 布局
- `ESPS3`、`STM32H757`、GPIO、UART、MotorBoard、PID 和 `WHEEL_TRIM`

## 生命周期与失败处理

- 未连接时不发送运动命令。
- 回中、释放、断链、后台、终止均把四轮目标清零。
- 非有限值不进入 `sendWheelSpeeds`；现有有限值检查继续生效。
- 轮速心跳只在存在非零目标且 BLE 仍连接时运行。
- App ACK、S3 转发、STM32 应用和车辆运动分别记录为独立验证层级。

## 验证计划

### 静态验证

- 确认 `VirtualJoystick` 不再调用旧 `sendControl()` 作为运动路径。
- 确认四个方向的四轮数组和轮序与本设计一致。
- 确认中心/释放调用全零目标，且没有遗留 Timer。

### 主机验证

- 运行 `swift build`（`IOS_APP/SmartCar_Control_MAC`）。
- 用现有 `build_and_run.sh --verify` 刷新并启动 macOS bundle。
- 在日志或注入式 BLE manager 中检查 `0x15` payload、方向切换和零速帧。

### 台架/车辆验证（本设计不代替）

- 断开车轮负载，分别验证四轮正负方向。
- 验证前进、后退、原地左转、原地右转和释放停止。
- 检查 100 ms 内有轮速心跳、断链后目标清零、STM32 watchdog 行为。
- 低速、额定速度和紧急停止需要独立的受控车辆测试记录。

## 风险

最大剩余风险是电机板安装方向与软件正负号不一致。构建只能证明 Swift
代码和协议编码集成；在匹配的 App/S3/STM32 映像、BLE/UART 抓包和受控
车辆测试完成前，摇杆仍标记为 `UNVERIFIED`。
