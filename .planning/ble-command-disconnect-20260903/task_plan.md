# BLE 指令无响应与断联修复计划

## 目标

修复 macOS App 在 BLE GATT 尚未准备完成时误报可操作、导致指令无响应的
生命周期问题，并补齐 App/S3 的有界链路诊断日志，以区分写入未提交、FFE1
接收/解析拒绝、SRP ACK 失败和真实 BLE 断开。

## 范围

- macOS App BLEManager 与现有控制调用点
- ESP32-S3 `s3_ble` GATT 写入和 `smartcar_service` App parser/ACK 路径
- 不恢复旧 `CONTROL(0x01)` 运动分支，不改变 SRP/FFE1/FFE2 UUID 和帧格式
- 不修改 STM32 安全门、MotorBoard 协议、ROS2 或硬件定义

## 阶段

| 阶段 | 状态 | 验收条件 |
| --- | --- | --- |
| 1. 日志、规则、历史上下文核对 | 完成 | 两份日志和当前源码边界明确 |
| 2. App GATT readiness 修复 | 完成 | 三项特征发现且 FFE2 通知状态确认后才进入 connected |
| 3. App/S3 有界诊断日志 | 完成 | 覆盖写提交、写回调、FFE1 RX、parser、ACK、断开原因 |
| 4. Host/App/S3 构建验证 | 完成 | 2026-09-03 续验：Swift staged bundle、S3 host tests、ESP-IDF build 均通过 |
| 5. 实机验证 | 待开始 | 先 ACK/零速，再非零指令；记录真实 disconnect reason |

## 证据边界

- 两份 2026-09-03 日志证明底层 STM-S3/SRP 在线、MotorBoard 反馈为零，不能证明 App 写入 FFE1 成功。
- `BLE_PREV_DISC reason=0x13` 是后续连接上报的历史原因，不等于当前会话实时断开。
- 构建通过不等于 BLE、UART 或车辆动作通过。
- 续验时本机未枚举 ESP32-S3 USB/JTAG 串口；`/dev/cu.Bluetooth-Incoming-Port` 与
  `/dev/cu.debug-console` 均不可作为刷写目标。
