# STM32H757 Runtime Log Debug Record

更新时间：2026-08-03
范围：CM7 启动、USART1 调试输出、FreeRTOS 调度、IMU runtime 任务和
SmartCar Logger 接收链路。

本文记录本次“reset 后有启动日志、没有 `imu_runtime` 运行日志”排查的
静态证据和硬件验收边界。源码检查、编译检查和串口工具检查不能替代烧录后
的板级运行证据。

## 结论摘要

- 当前源码路径包含 BOOT 输出、`imu_runtime_start()` 和
  `vTaskStartScheduler()`；它们按顺序位于 `STM32H757/CM7/Core/Src/main.c`
  的启动路径中。
- `imu_runtime_start()` 先同步调用 `imu_init()`，再创建 `imu_task` 和
  `imu_debug_task`。初始化返回值当前不会阻止后续创建任务，但初始化内部的
  SPI/I2C 访问和有限等待仍可能延迟启动日志。
- `imu_task` 和 `imu_debug_task` 都是永久循环任务，依赖 FreeRTOS 调度器运行。
  仅看到 BOOT 不能区分“任务创建失败、调度器未运行、初始化仍在阻塞、UART
  发送失败”或“随后发生 reset”。
- 目标运行标记为 `TASK_CREATE_OK`、`SCHEDULER_RUNNING` 和连续的
  `[DATA][IMU_RAW]` / `[DATA][IMU_FILTER]`。这些标记必须以实际串口接收为准；
  源码中是否已加入对应标记，以当前 Implementation Agent 的变更和构建结果为准。
- 2026-08-03 已将该 CM7 ELF 写入目标，并在串口监听已建立后执行软件 reset，
  连续观察 60 秒。原始输出止于 `IMU_INIT_BEGIN`；`IMU_INIT_DONE`、任务创建、
  调度器和心跳标记均未出现。因此本次故障发生在 `imu_init()` 内部、scheduler
  启动之前，而不是 IMU task 运行后由 BMI323 `online: FAIL` 引发的静默。

## BOOT

### 源码路径

`main()` 的静态顺序为：

1. `HAL_Init()` 和时钟配置；
2. GPIO、I2C4、TIM1、TIM3、TIM2 初始化；
3. USART1 初始化（调试适配器使用 PA9/PA10）；
4. `bsp_uart_init(BSP_UART_USART1, 115200U)`；
5. 输出 `SMART CAR H757 BOOT` 和 `USART1 DEBUG READY`；
6. 调用 `imu_runtime_start()`；
7. 调用 `vTaskStartScheduler()`。

对应证据：`STM32H757/CM7/Core/Src/main.c` 的 `main()`，以及
`STM32H757/CM7/Core/Src/stm32h7xx_hal_msp.c` 的 USART1 GPIO 初始化。

### 运行记录

| 观察 | 状态 | 证据等级 |
| --- | --- | --- |
| reset 后可见 BOOT 文本 | 已验证 | 2026-08-03 原始串口捕获 |
| 烧录后持续输出 | 未通过 | 首次 60 秒监听无后续字节；随后 reset 捕获止于 `IMU_INIT_BEGIN` |
| BOOT 后出现任务创建标记 | 未通过 | 60 秒内未见 `TASK_CREATE_OK` |
| BOOT 后出现调度器标记 | 未通过 | 60 秒内未见 `SCHEDULER_RUNNING` |

## UART

- 运行时日志发送链路为 `uart_log_write()` -> `bsp_uart_log_write()` ->
  `HAL_UART_Transmit()`。
- 目标配置是 USART1、115200 bps、8 data bits、1 stop bit、无校验、无硬件流控。
- `bsp_uart_init()` 还会检查 USART 实例、波特率和 HAL ready 状态，并创建 TX
  mutex；任一条件失败会使启动日志或后续日志不可见。
- 日志发送是带超时的阻塞 TX，并由 mutex 串行化。多个初始化诊断块连续输出时，
  TX timeout 或串口未 ready 可能造成日志缺口；这不等同于 FreeRTOS 任务没有运行。

注意：`STM32H757/BSP/UART/README.md` 仍描述“当前 IOC 仅配置 USART2”，而
当前 `main.c`/生成的 MSP 路径已包含 USART1。该文档差异是静态文档一致性问题，
不能作为 USART1 板级收发成功的证明。

## RTOS

- CM7 CMake 已纳入 FreeRTOS kernel、heap4 和 ARM CM7 port；`main()` 直接调用
  `vTaskStartScheduler()`。
- scheduler 启动前不能通过任务上下文证明“正在运行”。因此应在一个已运行的
  task 中输出 `SCHEDULER_RUNNING`，并用 1 Hz 心跳证明 tick、任务切换和 UART
  发送链路持续工作。
- `vTaskStartScheduler()` 正常启动后通常不会返回。若 BOOT 后完全没有任务日志，
  应按以下顺序区分：初始化阻塞 -> `xTaskCreate()` 失败 -> scheduler 启动异常
  -> task 进入 fault/reset -> UART TX 失败。

## IMU task

### 创建和初始化顺序

`imu_runtime_start()` 的顺序是：

```text
imu_init()
  -> bmi323_init()
  -> lsm303_init()
  -> imu_update()
  -> xTaskCreate(imu_task, ...)
  -> xTaskCreate(imu_debug_task, ...)
```

`imu_task` 以约 10 ms 周期读取 IMU，并在失败后按约 1 s 周期尝试恢复；
`imu_debug_task` 以 100 ms 周期输出一条 RAW 和一条 FILTER 数据日志。两者都包含永久循环，但循环
本身依赖 `vTaskDelayUntil()` 让出 CPU，不应在正常 scheduler 状态下独占 CPU。

### BMI323 FAIL 的边界

- `imu_init()` 的返回值在 `imu_runtime_start()` 中被忽略，因此 BMI323 初始化
  返回 FAIL 不会直接跳过两个 `xTaskCreate()` 调用。
- BMI323 初始化包含 SPI 访问及有限的毫秒/微秒等待；SPI HAL 超时、总线异常或
  其他底层错误仍可能让“启动阶段”耗时，需以实际串口时间戳确认是否是阻塞点。
- 任务启动后，BMI323 读失败应体现在 `IMU STATUS` 的 `online: FAIL`、任务统计
  或恢复日志中；不能把 FAIL 自动解释为 scheduler 未启动。
- 本次任务禁止修改 BMI323/LSM303 驱动；若需继续定位，应使用已有状态日志、
  任务创建标记和硬件逻辑分析仪/电气检查分别取证。

### 目标运行标记

建议在本次 Implementation 变更中保留以下可检索文本，并由烧录测试确认：

```text
TASK_CREATE_OK task=imu_task
TASK_CREATE_OK task=imu_debug_task
SCHEDULER_RUNNING
[DATA][IMU_RAW] ax=... ay=... az=... mx=... my=... mz=...
[DATA][IMU_FILTER] ax=... ay=... az=... mx=... my=... mz=...
```

两条 `DATA` 日志应每 100 ms 成对出现。当前 Filter 尚未实现算法时，RAW 与
FILTER 数值一致是正常现象；后续加入低通算法后，FILTER 应相对 RAW 更平滑。

## LOGGER

- STM32 侧 Logger 边界是 UART 文本发送，不负责持久化；macOS SmartCar Logger
  仅接收、按行缓存和显示串口字节。
- Logger UI 的过滤、环形缓存和 Copy 导出不会修复设备端缺失的字节，也不能证明
  设备 task 正在运行。
- 本次排查不修改 Logger 工具、不修改 UART 协议。验证时应保存原始串口文本，
  不只保存经过 UI 过滤后的显示内容。
- 同时记录设备端可见日志和 Logger 端统计；若设备端已确认发送而 UI 无显示，
  再单独排查 USB-UART、端口占用、波特率和 Logger 接收链路。

## 硬件验收记录

以下项目必须在同一固件、同一 UART 接线和同一 Logger 配置下执行：

| 场景 | 通过条件 | 当前状态 |
| --- | --- | --- |
| 烧录后不按 reset，等待 60 s | BOOT 后持续看到 10 Hz 采样链运行和 10 Hz 成对 `DATA` 日志 | 未通过：首次监听未收到后续字节 |
| 软件 reset 后 | 出现完整 `BOOT` -> `TASK_CREATE_OK` -> `SCHEDULER_RUNNING` -> `DATA` 日志序列 | 未通过：仅到 `IMU_INIT_BEGIN`，连续观察 60 秒 |
| BMI323 不在线 | 任务仍存活并明确报告 `online: FAIL`，不因单个传感器 FAIL 卡死启动 | 未验证：流程未离开 `imu_init()`，尚未到达任务运行态 |
| reset/watchdog 原因 | 记录复位寄存器、fault 或 watchdog 证据；不能凭“日志停止”推断 | 待硬件调试 |

## 证据边界

本文件的源码路径、日志字符串、初始化顺序和循环检查属于静态证据；即使
CM7 clean build 通过，也不能证明板上 scheduler、task、UART、电源、SPI/I2C
或 watchdog 行为正确。烧录、串口原始日志、60 秒连续观察和 reset 原因记录
由 Agent 4 单独完成后，才能把对应项目从“待验证”改为“已验证”。
