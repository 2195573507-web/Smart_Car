# 发现记录

## 当前日志事实

- `BLE_CONNECTED` 出现在两份日志中；`SRP_S3 h=1`、`rec=0`、`tec=0`、`timeout=0`、`bus_off=0`。
- STM32 与 MotorBoard 持续通信，但 `MTEP/MSPD` 四轮均为 0。
- 当前两次日志未出现 `CLIENT DISCONNECTED`；第二份的 `BLE_PREV_DISC reason=0x13` 是历史断开记录。
- 日志没有 App 写 FFE1、CoreBluetooth `didWriteValueFor` 或 App parser ACK 证据。

## 当前代码风险

- `BLEManager.centralManager(_:didConnect:)` 在服务/特征发现之前调用 `updateStatus(.connected)`。
- FFE1、FFE2、FFE3 的特征是在后续发现回调中才赋值/订阅。
- App 的发送计数在调用前递增，不能作为 BLE 写入成功证据。
- S3 FFE1 使用 `ESP_GATT_AUTO_RSP`，写入会进入 RX 队列；旧 `CONTROL(0x01)` 没有当前运动消费者。

## 本轮实施

- App `BLEManager` 在 `didConnect` 后保持 `.connecting`；只有 FFE1/FFE2/FFE3 均发现且 FFE2 `didUpdateNotificationStateFor` 确认 `isNotifying=true` 才发布 `.connected`。
- App 输出限频 `[BLE_DIAG]`：`WRITE_SUBMIT`、`WRITE_COMPLETE`、GATT discovery、FFE2/FFE3 notify 状态和真实 `didDisconnectPeripheral` 错误；不改变 BLE UUID、帧格式或发送队列安全语义。
- S3 `smartcar_service` 每秒输出 `APP_BLE_RX`/`APP_BLE_TX` 摘要，分别覆盖 FFE1 入队/丢弃、合法帧/命令帧/parser error，以及 ACK 成功/拒绝/通知丢失、真实断开次数和 reason。
- 诊断计数使用固定宽度饱和计数；未逐包打印，避免调试输出影响服务任务实时性。

## 续验（2026-09-03）

- `IOS_APP/SmartCar_Control_MAC/script/build_and_run.sh --verify` 已重新打包当前
  源码；`.build` 与 `dist/SmartCar_Control_MAC.app` 二进制 SHA-256 一致，且
  `Info.plist` 通过 `plutil -lint`。此前 dist 二进制时间早于 BLEManager 源码，不能
  用作本轮修复已部署的证据，现已消除该差异。
- ESP-IDF 不在默认 PATH；显式加载
  `/Users/zhiqin/.espressif/v5.5.4/esp-idf/export.sh` 后，`idf.py -B build build`
  通过。雷达脚本无可执行权限但可经 `sh main/radar/tests/run_host_tests.sh` 通过。
- 当前仅枚举 `/dev/cu.Bluetooth-Incoming-Port` 与 `/dev/cu.debug-console`，未确认
  ESP32-S3 USB/JTAG 设备，故不执行 flash，也没有新的 BLE、UART 或车辆运行证据。
