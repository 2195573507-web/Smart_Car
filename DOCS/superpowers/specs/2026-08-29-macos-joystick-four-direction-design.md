# macOS 摇杆 A+ 连续差速控制设计

## 状态

代码实现已完成，Swift 主机编译和活动 bundle 启动验证通过；BLE、UART、
STM32 应用和车辆行为仍需分别验证，当前不能据此宣称物理验收完成。

本文记录 `IOS_APP/SmartCar_Control_MAC` 现有圆形摇杆的连续差速控制语义。

## 背景与现状

原实现按拖动的主轴把摇杆量化为前进、后退、原地左转或原地右转。这样斜向
拖动会丢失幅度信息，无法表达缓转和急转。

当前实现保留原有界面和 BLE 轮速路径，新增 `JoystickIntent`，把归一化的
水平/垂直输入转换为四轮目标。底盘仍是左右两侧差速结构，不支持纯横移。

## 目标

1. 保留现有摇杆外观、拖动手势和速度滑块。
2. 纵向偏移决定前进/后退线速度，横向偏移幅度决定转弯速度。
3. 保持向左输入始终表示左转、向右输入始终表示右转，前进和倒车不翻转
   横向极性。
4. 复用现有 App BLE `0x15` 四轮速度命令、心跳和生命周期归零逻辑。
5. 不新增协议类型，不修改 S3、STM32、GPIO、轮序、PID 或 `WHEEL_TRIM`。

## 非目标

- 不实现麦克纳姆轮/全向轮所需的纯左移、纯右移或任意横移。
- 不在 View 中创建独立发送定时器。
- 不修改 STM32-S3 SRPv4 或 App BLE 外层帧格式。
- 不根据主机编译或打包结果宣称 BLE、UART、电机或车辆行为已验收。

## 控制语义

### 输入归一化

摇杆平面使用 SwiftUI 坐标：水平向左为负，向右为正；垂直向上为负，
向下为正。每个轴先限制到 `[-1, 1]`，再扣除现有 `18 pt` 死区并重新
归一化到满量程。

### 速度和转向

设速度滑块为 `speed_percent`，则最大轮速为：

```text
Vmax = speed_percent / 100 * 800.0 mm/s
v = -vertical * Vmax
```

横向输入采用幂函数曲线，中心附近保留微调空间：

```text
turn_fraction = abs(horizontal) ^ 1.5
```

当前软件约定 `w > 0` 为左转，因此：

```text
horizontal < 0 -> w = +turn_fraction * Vmax / 96.5
horizontal > 0 -> w = -turn_fraction * Vmax / 96.5
```

其中 `96.5 mm = 193.0 mm / 2`。轮速目标按差速运动学计算：

```text
right = v + w * 96.5
left  = v - w * 96.5
[RR, RF, LR, LF] = [right, right, left, left]
```

计算后若任一轮的绝对值超过 `Vmax`，则同时缩放左右轮，保持转弯半径不变。

### 方向示例

| 摇杆状态 | 操作含义 | 轮速趋势 |
| --- | --- | --- |
| 上 | 最大前进 | 四轮同向 |
| 上左 | 前进缓慢左转 | 右侧更快、左侧更慢 |
| 上左（偏移更大） | 前进急左转 | 左右差值更大 |
| 左 | 原地左转 | 右正、左负 |
| 下 | 最大后退 | 四轮反向 |
| 下左 | 倒车并保持左转输入 | 右侧负速较小、左侧负速较大 |
| 右 | 原地右转 | 右负、左正 |
| 回中/释放 | 停止 | 四轮为零 |

倒车时横向输入的极性不翻转。这符合真实遥控车“左打仍是左打”的习惯；
由于差速车的运动几何，倒车轨迹的弯曲方向可能与前进时的视觉直觉不同，
台架测试记录应区分“车头转向方向”和“地面轨迹方向”。

## 数据流与调度

```text
VirtualJoystick
  -> JoystickIntent(horizontal, vertical)
  -> SmartCarViewModel.setJoystickInput()
  -> wheelTargets[RR, RF, LR, LF]
  -> BLEManager.sendWheelSpeeds()
  -> App BLE 0x15 (4 x f32 LE)
  -> S3 command_bridge
  -> SRPv4 0x02
  -> STM32 MotorBoard
```

View 只负责手势和死区归一化；轮速公式、限幅、状态和发送节奏由
`SmartCarViewModel` 负责。

- 首次/变化命令继续使用约 `50 ms` 合并窗口。
- 连续拖动事件不会反复重置同一个待发送定时器，定时器发送最新目标。
- 稳定目标继续使用 `100 ms` 轮速心跳。
- 回中、释放、断链、后台和终止发送全零目标并停止心跳。
- 未连接或非有限输入不产生非零运动命令。

## 修改边界

允许修改：

- `IOS_APP/SmartCar_Control_MAC/Sources/SmartCar_Control_MAC/UI/ControlModeView.swift`
- `IOS_APP/SmartCar_Control_MAC/Sources/SmartCar_Control_MAC/ViewModels/SmartCarViewModel.swift`

保持不变：

- `SmartCarProtocol` App BLE 外层格式和 `0x15` payload 布局
- `ESPS3`、`STM32H757`、GPIO、UART、MotorBoard、PID 和 `WHEEL_TRIM`
- `[M1:RR, M2:RF, M3:LR, M4:LF]` 轮序

## 验证计划

### 静态和主机验证

- 确认 `VirtualJoystick` 不再以主轴择一方式丢弃横向幅度。
- 检查 `JoystickIntent` 的死区、符号、幂函数和四轮数组顺序。
- 检查连续拖动不会产生未限频的 BLE 写入。
- 运行 `git diff --check`。
- 在 `IOS_APP/SmartCar_Control_MAC` 运行 `swift build`。
- 运行 `script/build_and_run.sh --verify` 刷新并检查活动 bundle。

### 台架验证

轮子离地，使用低速档逐项记录：

1. 前进、后退、原地左转、原地右转。
2. 前左/前右和后左/后右的轻转、急转变化。
3. 摇杆回中和释放后的全零帧。
4. 断链、后台切换后的零速行为。
5. 电机板实际正负方向、左右转向极性和轮速反馈。

### 车辆验证

仅在台架方向确认后进行受控低速、额定速度和紧急停止测试；分别保存
App 日志、BLE/UART 抓包、STM32 日志和硬件测试记录。

## 风险

- 电机安装方向、板卡通道方向或编码器符号可能与软件正负约定不一致。
- `Vmax` 同时作为线速度和最大转向轮速分量，实际转弯手感需要台架调参。
- 倒车时“车头左转”和“轨迹向左”不是同一概念，必须在验收记录中明确。
- 主机编译只证明 Swift 集成，不证明 BLE、UART、STM32 或车辆运行。
