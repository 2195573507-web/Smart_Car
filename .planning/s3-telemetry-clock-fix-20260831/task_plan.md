# S3 telemetry clock 修复计划

## 目标

修复 ESP32-S3 telemetry producer 与 consumer 使用不同时间基准导致所有
S3RD type-2 帧被 stale gate 丢弃的问题，同时保持协议、队列、1000 ms 门限、
STM32 和 ROS2 行为不变。

## 范围与约束

- 仅修改 `ESPS3` 内与 telemetry age 判定和对应 host test 直接相关的文件。
- 不修改 SRP/S3RD wire contract，不提高 stale 门限。
- 不修改 STM32H757、ROS2_WIN 或 macOS App。
- 保留现有脏工作树，不执行 reset、checkout、clean 或无关格式化。
- 构建成功只证明源码和产物，不等于已刷写或实机链路通过。

## 阶段

| 阶段 | 状态 | 验收条件 |
| --- | --- | --- |
| 1. 审计规则、现有差异和测试入口 | 完成 | 确认目标代码、现有改动和可复用测试结构 |
| 2. 设计并加入 age 边界测试 | 完成 | 覆盖 0/999/1000/1001 ms 和 uint32 回绕 |
| 3. 实施同源单调时钟修复 | 完成 | consumer 与 producer 同用 esp_timer 时间基准 |
| 4. 执行 host 回归与 ESP-IDF 构建 | 完成 | 目标测试、既有 radar tests、IDF build 均通过 |
| 5. 产物核验与实机门槛 | 完成 | 记录 BIN、SHA-256、刷写状态及 30 秒验收项 |
| 6. 分析 22:34:50 实机日志 | 完成 | 核对固件标识、tx2/stale/chassis、队列与链路健康 |
| 7. 确定下一阶段联调动作 | 完成 | 明确 S3、Windows ROS2 与 BLE 的验收顺序 |

## 错误记录

| 错误 | 次数 | 处理 |
| --- | ---: | --- |
| 新 age 测试首次编译找不到 `radar_telemetry_age.h` | 1 | 预期的红灯阶段；下一步加入 helper 实现 |
| 未发现 ESP32-S3 USB/JTAG 串口 | 1 | 不猜测端口、不刷写；等待设备以明确 USB 端口枚举 |
