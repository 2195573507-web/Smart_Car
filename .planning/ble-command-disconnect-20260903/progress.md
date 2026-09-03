# 进度记录

- 2026-09-03：读取项目规则、Codex 知识库、指定日志和历史任务；未回滚既有 dirty worktree。
- 2026-09-03：确认本阶段最小修复边界为 App GATT readiness 与跨层有界诊断，不改旧 CONTROL 兼容分支。
- 2026-09-03：完成 App GATT readiness 修复和 `[BLE_DIAG]` 写入/断开诊断；完成 S3 `APP_BLE_RX/TX` 每秒摘要及 ACK/断开计数。
- 2026-09-03：`swift build`、S3 BLE log TX host tests、雷达 observability host tests 和 ESP-IDF v5.5.4 `idf.py -B build build` 均通过；未烧录、未执行实机 BLE/车辆验收。
- 2026-09-03（续）：完整读取前序任务与当前计划；重新执行 macOS
  `build_and_run.sh --verify`，确认 staged App 与 Swift 构建产物哈希一致、bundle
  plist 合法、空闲进程运行正常。显式加载 ESP-IDF 5.5.4 后重跑 `idf.py -B build build`
  成功；S3 BLE log TX 与雷达 host tests 通过。未发现可确认的 ESP32-S3 串口，保持
  阶段 5 待开始，不刷写、不宣称实机修复。
