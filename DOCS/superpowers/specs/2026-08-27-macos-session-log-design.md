# macOS Session 日志持久化设计

## 状态

- 状态：已获用户确认，待实施
- 范围：`SmartCar_Control_MAC` 的 BLE Session 日志持久化、连接时 UI 日志清空和日志页状态入口
- 不包含：BLE UUID、FFE3 数据格式、日志解析器、S3/STM32 固件、现有手动导出功能或车辆控制逻辑修改

## 目标与约束

每次 macOS App 收到 `SmartCar_S3` 的 CoreBluetooth `didConnect` 成功回调时，建立一个新的日志 Session：清空 STM32/S3 两个 `DeviceLogStore` 的 UI 历史，并在仓库根目录的 `LOG/` 创建一个新的 Markdown 文件。

文件绝对目录固定为 `/Users/zhiqin/Projects/Smart_Car/LOG/`；目录不存在时创建。文件名使用连接成功时的本地时间：

```text
smartcar_log_YYYY-MM-dd_HH-mm-ss.md
```

文件创建后立即写入：

```markdown
# SmartCar Session Log
- **Connected Time**: 2026-08-27 13:38:30 +0800
- **Device**: ESP32-S3 Gateway
---

```

`Connected Time` 实际写入创建该 Session 时的完整本地时间；上例仅说明格式。

每个 Session 只创建一个合并文件。FFE3 解析出来的 STM32 与 S3 记录均追加到这一文件，并保留来源和等级标签。这样一个 BLE 连接与一个可回放的完整日志工件一一对应。

## 当前数据流与修改后数据流

当前 FFE3 数据流保持不变：

```text
CoreBluetooth FFE3 notification
    -> BLELogReceivePipeline utility serial queue
    -> SmartCarLogParser
    -> BLEManager MainActor
    -> stmLogStore 或 s3LogStore
    -> SwiftUI DeviceLogView
```

修改后，在保留上述 UI 路径的同时追加一个独立磁盘输出支路：

```text
BLEManager didConnect
    -> clear two DeviceLogStore instances
    -> SessionLogWriter.beginSession(timestamp)
    -> LOG/smartcar_log_YYYY-MM-dd_HH-mm-ss.md

decoded FFE3 SmartCarLogRecord
    -> existing source-specific DeviceLogStore
    -> SessionLogWriter.append(record) on serial I/O queue

BLEManager didDisconnectPeripheral
    -> SessionLogWriter.closeAndSynchronize()
```

磁盘 I/O 不在 CoreBluetooth delegate、日志解析队列或 SwiftUI 主 actor 中同步执行。写入器以专有串行队列持有 `FileHandle`，并按到达顺序创建、追加、关闭，避免多条 FFE3 通知产生并发写入或交叉关闭。

## 组件与代码边界

新增：

- `Sources/SmartCar_Control_MAC/Support/SessionLogWriter.swift`：唯一拥有 `LOG` URL、命名/头部格式化、目录创建、`FileHandle`、异步追加和同步关闭的文件生命周期组件。

修改：

- `BLE/BLEManager.swift`：持有 writer；在 `didConnect` 建立 Session 并清空两个 UI store；每个已解析 FFE3 `SmartCarLogRecord` 送入 writer；在断开回调和显式断开路径安全关闭。对 SwiftUI 公开只读当前文件名和写入错误状态。
- `ViewModels/SmartCarViewModel.swift`：透传当前录制文件名，供两个开发者日志页使用；不改变连接、运动或协议状态逻辑。
- `UI/LoggerSTMView.swift`、`UI/LoggerS3View.swift`、`UI/DeviceLogView.swift`：把当前录制文件名传入日志工具栏/状态栏，并增加 Finder 打开固定 `LOG` 目录的按钮。没有活动 Session 时显示明确的未录制状态，按钮仍可打开目录。

不修改：

- `SmartCarLogParser`、`SmartCarLogRecord`、FFE0--FFE3 UUID 和任何 BLE 帧/协议定义。
- S3、STM32、CM7、GPIO、DMA、RTOS 和安全控制代码。
- `DeviceLogStore` 的容量/批量发布策略以及现有 Copy/Export TXT 操作。

## 文件内容与错误策略

写入器使用可读的 Markdown 文本行，例如：

```text
2026-08-27 13:38:31.240 [STM32][INFO] IMU ready
```

时间取日志抵达 App 的本地时间；原始 `timestampMilliseconds` 不是 wall-clock 时间，仍保留在 UI 的现有显示中。若创建目录、创建文件或追加发生错误，错误只记录到 writer 的状态并可在 App 控制台诊断；不得阻塞、丢弃或改变 FFE3 的解析和 UI 显示流程。无可用文件句柄时的追加是安全的 no-op。

断开时 `closeAndSynchronize()` 会排在同一 I/O 队列尾端，确保此前已经入队的写入先完成，再调用同步和关闭。该关闭动作不阻塞 CoreBluetooth callback：callback 只触发异步关闭；进程的明确退出路径补充同步关闭，降低尾部日志丢失风险。

## UI 设计

两个已有日志页面不新增页面或卡片。在工具栏标题右侧显示：

- 正在录制时：`Recording: smartcar_log_YYYY-MM-dd_HH-mm-ss.md`
- 未建立 Session 时：`Not recording`

工具栏加入带 Finder 图标的无文字按钮，hover 提示为 `Open LOG Folder`，调用 `NSWorkspace.shared.open()` 打开 `/Users/zhiqin/Projects/Smart_Car/LOG/`。它不要求用户手动选择目录，也不取代原有的 TXT 导出按钮。

## 生命周期与边界条件

1. 一次成功 `didConnect` 创建一次新 Session；重复扫描、发现设备、连接中、服务/特征发现不会创建文件。
2. 每次成功连接先关闭残留 writer（若有），再清空 UI store 并建立新的文件，防止重连时串写旧文件。
3. FFE3 通知可以分片；只有既有 parser 产出的完整 `SmartCarLogRecord` 写入文件和 UI，保持两者一致。
4. 断开、连接失败、蓝牙不可用、手动断连以及 App 终止均请求关闭当前 Session；多次关闭必须幂等。
5. 不以 host build 或本地文件生成断言 S3 BLE 推送已被硬件验证。

## 验证计划

### 主机验证

- 编译：在 `SmartCar_Control_MAC` 运行 `swift build`。
- 已打包 App：运行 `./script/build_and_run.sh --verify`，确认 staged bundle 能启动。
- 静态检查：确认文件名格式、目录创建、头部精确文本、FFE3 record 追加、来源标签、断开关闭和 UI folder reveal 调用均在目标文件中。
- 手工 macOS 验证：连接已可用的 `SmartCar_S3` 后确认 UI 两个 store 清空、`LOG/` 生成一个新 `.md`、文件包含头部和后续 FFE3 文本；断开后确认文件可以读取和重命名，表明句柄已关闭。

### 证据限制

Swift 编译和 bundle 启动只能证明目标源码集成与宿主进程启动。它们不能证明实际 S3 连接、FFE3 通知、BLE 传输、文件追加时序或车辆状态；这些需要匹配 App/S3 映像和真实连接捕获。

## 风险与取舍

- 单一合并文件避免用户在一个 BLE Session 后需要拼合 STM32/S3 两个文件，同时保留来源标签以便诊断。
- 串行后台 I/O 牺牲极小的持久化延迟，换取不阻塞主线程/解析队列和确定的写入顺序。
- 固定绝对路径满足当前工作流，但目录不可写时只能降级为 UI 继续显示、文件记录失败；不改变 BLE 通信或控制行为。
