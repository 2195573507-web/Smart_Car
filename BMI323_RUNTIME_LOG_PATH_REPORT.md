# BMI323 WHOAMI 运行日志路径检查报告

## 范围与结论

本次为只读源码与既有构建/Flash 证据审查。没有修改 C/C++、CMake、协议、S3、App 或 BMI323 文件，也没有烧录。

结论先行：

1. `bmi323_init()` 的启动调用链存在，且没有被 IMU Manager 条件跳过。
2. `WHOAMI_VALUE_ERROR` 表示 WHO_AM_I SPI 事务已经返回 `BSP_STATUS_OK`，但返回值不是 `0x43`；因此该状态不是“没有进入 WHOAMI 检查”的证据。
3. `[BMI323][WHOAMI]` 和 `[BMI323][SPI]` 是 BMI323 初始化阶段的一次性日志，发生在 STM32 启动早期、BLE 客户端连接和 FFE3 LOG Notify 开启之前。
4. STM32 端的 `log_bridge_handle()` 将 STM32 LOG 直接交给 `s3_ble_log_notify_send()`。该函数在 BLE 未连接或 LOG CCC Notify 未启用时返回 `ESP_ERR_INVALID_STATE`，桥接代码忽略返回值且不入 pending 队列。因此早期的一次性 BMI323 日志可能已在 S3 日志桥丢弃；之后周期性的 `[BMI323][DEBUG]` 仍可能出现。

这解释了“运行日志有 `[BMI323][DEBUG]`，但没有 `[BMI323][WHOAMI]`/`[BMI323][SPI]`”的最可能原因。仅凭源码不能区分 STM32 USART2 物理链路、S3 parser/CRC 丢帧和 BLE 连接时序的实际发生点；需要运行抓包或 S3 `STM_LOG_RX`/`LOG_DROP` 证据才能继续细分。

## 1. `bmi323_init()` 调用路径

当前 CM7 启动路径如下：

```text
main()
  STM32H757/CM7/Core/Src/main.c:195
  -> imu_runtime_start()
  -> STM32H757/Application/RTOS/imu_runtime.c:501 imu_init()
  -> STM32H757/Middleware/Sensor/imu_manager.c:526 imu_init()
  -> imu_init_internal(1U)
  -> imu_manager.c:498 lsm303_init()
  -> imu_manager.c:510 bmi323_init()
  -> bmi323.c:294 bmi323_port_init()
  -> bmi323.c:298 bmi323_read_reg(BMI323_REG_CHIP_ID, ..., 1U)
  -> bmi323.c:300 bmi323_log_whoami_raw(who_am_i)
```

`imu_init_internal(1U)` 先初始化 LSM303，再独立调用 BMI323，并通过
`imu_mark_bmi323_initialized()` 记录 BMI323 成功/失败；BMI323 失败不会改变 LSM303 的初始化返回值或调用方接口。

## 2. WHO_AM_I 日志触发条件

代码位置：`STM32H757/Middleware/Sensor/BMI323/bmi323.c`。

| 条件 | 当前行为 | 证据 |
| --- | --- | --- |
| `bmi323_port_init()` 失败 | 在进入 `bmi323_read_reg()` 前返回，不产生 WHOAMI/SPI 原始日志 | `bmi323.c:294-296` |
| port 初始化成功 | 读取寄存器 `0x00`，随后无条件调用 `bmi323_log_whoami_raw()` | `bmi323.c:298-300` |
| WHO_AM_I 读取成功但值错误 | 记录 `whoami_fail`，状态为 `WHO_AM_I_VALUE_ERROR`，然后初始化失败 | `bmi323.c:307-311` |
| WHO_AM_I 读取超时或 SPI RX 失败 | 设置相应错误状态，然后初始化失败 | `bmi323.c:223-235` |
| 一次性限制 | `static whoami_trace_done` 初值为 0；日志函数先检查再置 1，两个记录最多各发送一次 | `bmi323.c:27,82-115` |
| 后续 `imu_recover()` | `bmi323_init()` 可再次被调用，但没有清零 `whoami_trace_done`，不会重发这两条日志 | `imu_manager.c:532-546`、`bmi323.c:274-300` |

用户给出的周期日志格式：

```text
[BMI323][DEBUG]
read_ok=...
read_fail=...
last_status=WHO_AM_I_VALUE_ERROR
```

对应 `imu_bmi323_debug_log()`（`imu_manager.c:275-290`），该函数由 IMU 任务每个健康周期调用（`imu_manager.c:651-665`），不是一次性 WHOAMI 原始日志。`WHO_AM_I_VALUE_ERROR` 又来自 BMI323 初始化的数值比较分支，所以它反向证明初始化至少走到了 WHO_AM_I 判断。

## 3. STM32 日志出口与 `SC_TYPE_LOG`

BMI323 日志的实际调用路径：

```text
bmi323_log_whoami_raw()
  -> uart_log_write()                         bmi323.c:99,115
  -> bsp_uart_log_write()                    bsp_uart.h:68-70
  -> bsp_uart_log_write_level(INFO, text)    bsp_uart.c:246-248
  -> uart_log_write_usart1()                 bsp_uart.c:230
  -> uart_log_write_usart2()                 bsp_uart.c:232
  -> sc_frame_encode(SC_TYPE_LOG, ...)       bsp_uart.c:215-217
  -> uart_link_send()                        bsp_uart.c:217
  -> HAL_UART_Transmit(USART2, ...)          uart_link.c:125-162
```

`SC_TYPE_LOG` 定义为 `0x30`（`STM32H757/Middleware/Communication/SmartCar_Frame/sc_frame.h:125`），映射为 `SCBP_MSG_ID_LOG=0xF000`（`sc_frame.c:136`、`sc_frame.h:68`），编码时目标为默认 S3 节点（`sc_frame.c:311-315`）。USART2 链路在 BMI323 初始化前已经建立：`main.c:178-182` 依次初始化 USART1、`uart_link_init()` 和 `bsp_uart_init()`；`uart_link_task_start()` 在 `main.c:198`，晚于 IMU 初始化，但发送函数本身直接调用 `HAL_UART_Transmit()`，不依赖 RX task。

`bmi323_log_whoami_raw()` 对 `uart_log_write()` 的返回值使用 `(void)` 丢弃（`bmi323.c:99,115`），所以当前源码不会把 USART2/HAL 发送失败反馈到 BMI323 诊断状态。

## 4. S3 与 BLE 运行时序

STM32 侧可确定的时间顺序是：

```text
Reset
  |
  v
HAL_Init / 时钟 / GPIO
  |
  v
USART1 + uart_link_init(USART2) + bsp_uart_init  (main.c:174-185)
  |
  v
imu_runtime_start -> imu_init -> bmi323_init -> WHO_AM_I
  |
  v
一次性 [BMI323][WHOAMI]、[BMI323][SPI]
  |
  v
创建 uart_link_task / s3_service_task
  |
  v
vTaskStartScheduler (main.c:198-200)
```

S3 侧是另一个独立复位/启动序列：

```text
app_main
  -> stm_uart_init()                 ESPS3/main/main.c:37
  -> s3_ble_init()                   main.c:42
  -> smartcar_service_init()         main.c:68
  -> smartcar_service_task 读取 STM UART 缓存并解析
```

BLE GATT 服务就绪、客户端连接和 FFE3 LOG CCC 写入都是异步事件。`ESP_GATTS_CONNECT_EVT` 只设置 `connected=true`，并明确把 `log_notify_enabled` 设为 false（`ESPS3/components/s3_ble/s3_ble.c:340-345`）；只有客户端写入 LOG CCC 后，`log_notify_enabled` 才为 true（`s3_ble.c:371-383`）。

STM32 LOG 的 S3 处理链为：

```text
STM32 USART2 TX
  -> S3 UART2 RX(GPIO18, 115200)
  -> stm_uart_task() 存入 4 KiB storage
  -> smartcar_service_task()
  -> sc_frame_parser_feed()
  -> command_bridge_on_frame()
  -> log_bridge_handle()
  -> s3_ble_log_notify_send()
  -> FFE3 Notify
```

关键差异在 `log_bridge.c:49-52`：STM32 source 的帧只打印 `STM_LOG_RX`，然后直接调用 `s3_ble_log_notify_send()`。该 API 在 `s3_ble.c:472-480` 要求 `s_initialized && connected && log_notify_enabled`，不满足即返回 `ESP_ERR_INVALID_STATE`。`log_bridge_handle()` 忽略返回值，且没有把 STM32 日志写入 `s_pending_logs`。后者只被 `s3_ble_log_emit()` 使用（`s3_ble.c:521-543`），并在 LOG CCC 开启时刷新（`s3_ble.c:376-382`）。

因此启动早期的一次性 BMI323 记录有明确的丢失窗口：STM32 已发送，但 S3 解析/桥接发生在 BLE Notify 准备好之前，记录不会等待客户端。周期性 `[BMI323][DEBUG]` 通过 STM32 `LOG_INFO` -> `log_service` 队列（`log_service.c:46-55`）在 scheduler 后继续产生，若此时 BLE 已 ready，则可见。

## 5. 证据边界与根因判断

| 项目 | 判断 |
| --- | --- |
| BMI323 调用是否存在 | 已确认；无 Manager 条件跳过 |
| WHOAMI 判断是否执行 | `WHO_AM_I_VALUE_ERROR` 证明已执行并收到非 `0x43` |
| `[WHOAMI]`/`[SPI]` 是否只一次 | 是；静态标志且 recovery 不清零 |
| STM32 是否使用 `SC_TYPE_LOG=0x30` | 是；经 USART2/SCBP LOG 帧发送 |
| 是否有 BMI323 日志过滤 | 未发现；日志函数直接调用 UART，Manager 周期 DEBUG 走 LOG_INFO 队列 |
| S3 是否保证早期 STM32 LOG 等待 BLE | 否；桥接直接发送，失败返回值被忽略 |
| 当前 ELF/Flash 是否含该代码 | 既有 `STM32_FLASH_VERIFY_REPORT.md` 记录 ELF 与目标 CM7 Flash 区域 SHA-256 一致；该报告同时记录当时核心为 halted、无运行采集 |
| 是否已证明 USART2 物理线、S3 CRC、BLE Notify 成功 | 未证明；本任务未做运行抓包 |

### 根因

**最可能根因是日志启动时序/桥接丢弃：WHOAMI/SPI 在 BLE LOG Notify 订阅建立前只发送一次，而 STM32 LOG bridge 没有 pending 缓存；不是 BMI323 初始化调用缺失，也不是 `WHO_AM_I_VALUE_ERROR` 分支阻止了日志生成。**

如果需要把软件桥接丢弃与物理链路问题进一步区分，应在不改变协议的前提下采集同一次启动的三处证据：

1. STM32 USART2 TX 上是否出现 `SC_TYPE_LOG` 帧（或检查 `uart_link` TX 计数/错误计数）。
2. S3 是否输出 `STM_LOG_RX`，以及是否有 `LOG_DROP`/CRC 错误。
3. BLE 连接后是否写入 FFE3 LOG CCC，并观察 App 的 FFE3 Notify。

本报告不提出或实施代码修复；以上是下一步定位所需的观测点。
