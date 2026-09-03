# S3 telemetry clock 修复实施报告

## 结论

S3RD type-2 telemetry stale gate 的混用时钟问题已在源码中修复，并通过 host
回归与 ESP-IDF 5.5.4 全量构建。实施阶段没有可识别的 ESP32-S3 USB 串口，因此本代理
当时未执行刷写；后续用户提供的运行日志已证明 type-2 行为恢复，但没有镜像 hash，且仍未
证明 Windows chassis、`/odom` 或 TF 已恢复。

## 修改位置、原因与内容

| 文件 | 原因 | 修改内容 |
| --- | --- | --- |
| `ESPS3/main/radar/radar_uplink.c` | consumer 使用 `esp_log_timestamp()`，与 producer 的 `esp_timer_get_time()` 不同 epoch | telemetry age 当前时间改用 `esp_timer_get_time()/1000`，调用纯 age helper |
| `ESPS3/main/radar/radar_telemetry_age.h` | 门限与回绕策略需要可独立测试 | 新增 header-only `radar_telemetry_age_is_stale()` |
| `ESPS3/main/radar/tests/test_radar_telemetry_age.c` | 防止门限和 uint32 回绕回归 | 覆盖 0、999、1000、1001 ms 和跨回绕边界 |
| `ESPS3/main/radar/tests/run_host_tests.sh` | 新测试必须进入现有入口 | 编译并执行 telemetry age 测试 |

## 兼容边界与潜在影响

- 未改变 1000 ms stale 门限、SRP/S3RD wire bytes、队列容量、调度或 TCP 发送状态机。
- 未修改 `radar_uplink.c` 中只用于同源日志节流的其他 `esp_log_timestamp()`。
- 新 helper 无动态内存、无锁、无栈数组、无 RTOS/DMA/ISR 副作用。
- 新固件只完成 build 证据；刷写后仍需真实 producer/consumer 和 Windows 端联合验收。

## 验证

- `sh ESPS3/main/radar/tests/run_host_tests.sh`：PASS。
- `sh ESPS3/components/s3_ble/tests/run_host_tests.sh`：PASS。
- ESP-IDF 5.5.4 `IDF_BUILD_JOBS=2 idf.py build`：PASS。
- `idf.py size`：DIRAM 52.31%，`.bss` 55648 bytes。
- BIN：`ESPS3/build/smartcar_s3_gateway.bin`，1236064 bytes。
- BIN SHA-256：
  `dc4c5fe85cd5784e156bddac8a73b9bbf92890c2a6ed414e2cedab2bf5e7f242`。

## 刷写后验收

1. 30 秒内确认 `telemetry_type2_sent` 开始增长。
2. 确认 `telemetry_stale_drops` 不再与 `queue_accepted` 一比一增长。
3. Windows 确认 `opaque_frames > 0`、`chassis_frames > 0`。
4. 通过预检后再启用 live odom/TF，并执行 120 秒窗口。
5. BLE 稳定性仍需独立连续 10 分钟验收。

## 22:34:50 运行日志复核

`LOG/smartcar_log_2026-08-31_22-34-50.md` 提供约 30 秒运行证据：TELEM
queue accepted 增加 2084，type-2 sent 增加 2083，stale/send failure 均增加 0；
chassis task 启动后约 16.7 Hz。该结果通过本窗口行为验收，但文件未包含 source SHA8 或
BIN hash，不能单独证明刷写镜像身份。

下一步转到 Windows 做 30 秒 type-2/chassis 接收预检，随后在车辆静止且不启用任何运动
接口的前提下执行 120 秒 `/odom`/TF 验收。MotorBoard 对零 PWM 的重复 NACK 另列为实际
运动和建图前安全阻塞。
