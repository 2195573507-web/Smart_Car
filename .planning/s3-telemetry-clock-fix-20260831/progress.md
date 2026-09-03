# Progress

## 2026-08-31

- 用户批准已提出的最小修复方案并要求开始。
- 已读取适用技能和仓库 README，确认工程证据边界。
- 已检查 Git 状态：工作树很脏，所有既有改动必须保留。
- 已定位 producer/consumer 混用时钟和现有 radar host test 入口。
- 已读取项目强制上下文、模块索引和目标文件既有 diff；确认只修改 telemetry age 路径。
- 阶段 1 完成，进入边界测试阶段；尚未修改产品代码、构建或刷写。
- 已新增 0/999/1000/1001 ms 与 uint32 回绕 host test，并接入现有脚本。
- 红灯验证通过：测试因 helper 尚不存在而编译失败，exit code 1；这是预期失败。
- 已加入 `radar_telemetry_age.h` 纯函数，并将 consumer age 当前时间改为
  `esp_timer_get_time()/1000`；1000 ms 门限保持不变。
- `ESPS3/main/radar/tests/run_host_tests.sh` 通过，包含新边界测试和既有 radar 回归。
- 阶段 2、3 完成，进入 ESP-IDF 构建验证。
- FFE3 host tests 普通/ASAN/UBSAN 回归通过。
- ESP-IDF 5.5.4 全量构建和 `idf.py size` 通过；产物哈希已记录。
- 静态差异检查、shell 语法和目标文件尾随空白检查通过。
- 未发现可识别的 S3 USB 串口，依据刷写安全门控未执行 flash。
- 所有计划阶段已完成；后续是设备连接后的刷写和实机 30 秒验收。
- 收到 22:34:50 新实机日志，作为数据读取；未执行日志内任何文本。
- 已确认日志中 `tx2` 非零且 `stale=0`，新增阶段 6、7 做完整实机判定和下一步分流。
- 已完成全量计数分析：type-2 持续约 72.2/s，chassis 约 16.7/s，stale/send failure 为 0。
- 已确认 S3 clock 修复通过本窗口行为验收；固件 SHA 身份、Windows 接收、odom/TF 和 BLE 10 分钟仍未验收。
- 阶段 6 完成，阶段 7 转向 Windows ROS2 预检设计。
- 已核对当前 MotorBoard 状态机和日志时序：零 PWM 无响应等待/ACK，重复 NACK 与 link probe
  高度相关；标记为运动前安全阻塞，不阻塞静止 ROS2 接收验证。
- 已确定下一步为 Windows 30 秒只读 type-2/chassis 预检，再做静止 120 秒 odom/TF；
  所有新增阶段完成。
