# Target Yaw 定向巡航任务计划

## 目标

实现 SRP 0x17 目标偏航命令从 App BLE 经 S3 到 CM7 的转发，并在 CM7
底盘周期中完成 DualAHRS 驱动的闭环控制，完成 CM7 构建验证。

## 阶段

- [x] 分析现有协议、S3 事务和 CM7 底盘/姿态边界
- [x] 确认协议设计：0x17、12 字节、flags=0、S3 ACK_REQUIRED
- [x] 更新共享注册表和主机测试
- [x] 更新 S3 BLE 映射、事务和安全清除
- [x] 更新 DualAHRS getter 与 CM7 heading 控制
- [x] 执行 CM7 clean build、S3 隔离构建和变更审计

## 风险

- 现有工作树有大量不相关用户修改，所有编辑必须保持局部。
- 仅源码和构建不能证明 BLE/UART 实物链路或车辆航向性能。

## 2026-08-28 动态误差抑制重构

- [x] 用户确认 `CHASSIS_HEADING_MAX_ALPHA_RAD_S2 = 5.0 rad/s^2`
- [x] MotorBoard 有效 MSPD 帧动态 `dt` 与异常基准重置
- [x] PID/反馈低通动态 `dt` 有限性保护
- [x] 航向环实际周期积分、负反馈阻尼、前馈和 `w_prev` slew 生命周期
- [x] CM7 Debug clean build、主机测试和安全不变量审计
