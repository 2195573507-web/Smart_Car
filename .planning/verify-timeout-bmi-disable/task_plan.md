# Smart Car 双问题修复计划

## 目标

审计并按批准范围处理 `VERIFY_TIMEOUT` 与 macOS App BMI 状态显示问题，保持
SCBP ID/payload、STM32/ESP32/App 接口、采样频率和校准状态机兼容。

## 阶段

- [x] 读取项目规则、架构和当前 dirty 工作区
- [ ] 五个专家完成独立只读审计并提交报告
- [ ] 汇总生成 `AUDIT_SUMMARY.md`，等待用户确认
- [ ] 仅在确认后实施最小源码修改
- [ ] 执行 STM32、ESP32、App 构建验证
- [ ] 生成 `SMART_CAR_FIX_REPORT.md`；如构建失败生成 `BUILD_ERROR_REPORT.md`

## 硬边界

- 不修改协议 ID、payload 格式、BMI/LSM 采样频率或校准状态机逻辑。
- 不强制发送 `CAL_EVENT` id=3 掩盖问题。
- 保留用户既有 dirty 改动，不 reset/checkout/大范围重构。
- 构建、设备运行、UART/BLE/传感器捕获分别记录证据等级。

