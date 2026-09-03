# Findings

## 已确认事实

- 工作树已有大量未提交修改，目标文件 `ESPS3/main/radar/radar_uplink.c` 也已修改。
- producer 在 `command_bridge.c` 中以 `esp_timer_get_time()/1000` 写入
  `ingress_timestamp_ms`。
- consumer 在 `radar_uplink.c` 中以 `esp_log_timestamp()` 计算 age。
- 日志窗口显示 queue accepted 与 stale drop 基本一比一增长，type-2 sent 恒为 0。
- `RADAR_UPLINK_MAX_TELEMETRY_AGE_MS` 当前为 1000 ms，本任务不改变该门限。

## 风险

- 必须区分 telemetry ingress 时间戳与雷达 UART、BLE 日志所用时间戳，避免扩大修改范围。
- `uint32_t` 时间差需要保留模运算回绕语义。
- 目标文件已有用户改动，补丁前后都要核对局部 diff。

## 现有架构与实施决策

- `radar_uplink.c` 中 721/790 行附近的 `esp_log_timestamp()` 仅用于同源日志节流，
  不与 producer timestamp 相减，本轮保留。
- 只有 telemetry packet 准备路径在跨模块比较 `ingress_timestamp_ms`，因此只替换该处
  当前时间采集。
- 使用独立 header-only 纯函数 `radar_telemetry_age_is_stale()` 表达 `age > limit`
  策略；host test 不依赖 ESP-IDF/FreeRTOS。
- consumer 通过 `esp_timer_get_time()/1000` 获取当前时间，和 `command_bridge.c`
  producer 保持同源单调时钟。
- 目标文件修改前 SHA-256：
  `deef64ac288274f927856f6ad4b864b47e7c0f769643b2623e177fedc64d643e`。

## 验证结果

- Radar host tests：PASS，包含新增 age 边界测试、既有 queue/protocol/tx/observability
  及现有 sanitizer 组。
- FFE3 host tests：普通与 sanitizer 组均 PASS，固定状态仍为 5456 bytes。
- ESP-IDF 5.5.4 `IDF_BUILD_JOBS=2 idf.py build`：PASS。
- `idf.py size`：DIRAM 178759/341760 bytes（52.31%），`.bss` 55648 bytes。
- BIN：1236064 bytes，SHA-256
  `dc4c5fe85cd5784e156bddac8a73b9bbf92890c2a6ed414e2cedab2bf5e7f242`。
- ELF：SHA-256
  `2d1aae1ed86af2c4eed3bce7eb18f6f2965facd100d872896428a24781dbdf0a`。
- ELF 包含 `S3_TELEM_TCP_READY chassis=0x15 outer=2`、`S3_TELEM_OBS_V1`
  和 source SHA8 `4eb568fb`。
- 当前 `/dev` 仅有 Bluetooth Incoming Port 与 debug-console；没有 ESP32-S3
  USB/JTAG/USB-serial 端口，未刷写设备。

## 22:34:50 实机日志初步证据

- 文件：`LOG/smartcar_log_2026-08-31_22-34-50.md`，1720 行、69925 bytes，
  SHA-256 `78b83471bfbbb448e4d07af49389a73fb80852e93ae5ddef1abefaf21bc1c090`。
- 首个可见 TELEM：`in=317 c=0 q=315 rej=2 lock=20 tx2=254 stale=0 fail=0`。
- 末段可见 TELEM：`in=2406 c=407 q=2399 rej=7 lock=63 tx2=2337 stale=0 fail=0`。
- 初步判断：type-2 已发送且 stale 为 0，时钟修复方向得到实机日志支持；仍需核对
  source SHA8、完整窗口增量和断连/队列证据。

## 22:34:50 实机日志完整判读

- 可解析记录 671 条；第一/最后记录跨度 29.977 秒，S3 记录 49 条、STM32 记录 622 条。
- 16 个 TELEM 快照覆盖 28.864 秒：sink `+2089`（72.374/s）、queue accepted
  `+2084`（72.201/s）、type-2 sent `+2083`（72.166/s）、rejected `+5`、
  stale `+0`、send failure `+0`。queue 与 sent 只差 1 条在途项。
- chassis task 于 22:34:56.567 启动；到末个 TELEM `c=407`，约 24.393 秒，
  平均 16.685/s。
- 30 个 RADAR_STATS 快照覆盖 29.046 秒；outer/raw sequence `1121 -> 4287`，
  约 109 packet/s，checksum/invalid/drop 均为 0，queue 最大 2。
- App 日志没有 `S3_TELEM_OBS_V1 src=4eb568fb` 或 BIN hash，故只能确认行为与修复一致，
  不能仅凭该文件完成烧录镜像身份对拍。
- BLE 从 `BOOT/BLE_CONNECTED` 后持续到文件末尾，约 29.5 秒无断开记录；没有 MTU、
  FFE3 worker statistics 或 disconnect reason，不能替代 10 分钟稳定性验收。
- STM-S3 SRP 保持 sync，parser error/REC/TEC/timeout/BUS_OFF 为 0；periodic drop 累计 28
  在窗口内不增长，`LOG_STATS drop=0`。
- MotorBoard 启动阶段出现一次 response timeout/retry、两次 `Command not found` 和多行
  read_flash unknown WARN；随后 mtype/mline/mphase/wdiameter ACK、read_vol=7.10V、upload=OK，
  之后电池与四轮 0 速反馈持续，无 UART timeout/invalid 增长。
- BMI323 有 4 次瞬时 DATA_NOT_READY，但 read_fail/overflow 均为 0，采样继续前进；
  不构成持续传感器故障。

## 下一阶段决策

1. Windows 先保持四个 live 参数为 false，执行 30 秒只读预检；要求真实 type-2/
   telemetry 与 chassis 计数增长、decoder 无 reject，同时 `/scan` 不回归。
2. 预检通过后，在唯一 bridge owner 上临时启用 `allow_live_telemetry`、
   `enable_live_odom`、`publish_odom`、`publish_tf`，车辆静止执行 120 秒 `/odom`、
   `odom -> base_link`、时间戳、finite 和首帧基线验证；仍不启动 `/cmd_vel` 或 controller。
3. BLE 10 分钟验收与 Windows TCP/odom 分开进行，不作为本次 type-2 预检的前置条件。
4. 日志中每次 link probe 的 `$pwm:0,0,0,0#` 后均出现未标记命令来源的
   `Command not found`，且没有零 PWM ACK；结合源码时序，高度怀疑 MotorBoard 固件不接受
   当前零 PWM 命令。静止 ROS 接收/odom 可继续，但任何车辆运动或建图前必须单独确认物理
   stop 命令和板端协议。
5. `/odom`/TF 通过后仍需实测 `base_link -> laser_frame` 外参，才进入 slam_toolbox/map 验收。
