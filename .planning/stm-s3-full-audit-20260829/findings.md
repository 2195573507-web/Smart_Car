# STM32H757 与 ESP32-S3 全量只读审计证据账本

> Historical/deprecated audit evidence from before the SRPv4 full switch.
> Current STM32-S3 UART2 implementation is defined by `Common/SRP`.

## 需求与边界

- 用户要求全量审计 STM 与 S3，至少覆盖任务调度、底层、应用层独立性和代码效率。
- 用户要求计划 STM32H757 双核任务拆分并说明最优方案。
- 用户要求至少 10 轮审计、编写和修改，最终交付逻辑清晰、表达明确的计划与总报告。
- 禁止修改任何代码；本任务只允许写本任务专用 Markdown 文档。

## 已确认基线

- 当前 Git 分支为 `4...origin/4`。
- 开始时已存在 `.planning/ros2-p1-mapping/` 与 `ROS2_WIN/` 未提交项；审计期间另有 S3、CM7、Common/SRP、ROS2 改动进入工作树。本审计不回滚、不归因这些并行改动，只维护本任务 Markdown。
- 根级 `task_plan.md`、`findings.md`、`progress.md` 属于 2026-07-30 的 iOS/S3 架构规划，不属于本任务。
- Codex 目标系统已自动建立本次审计目标，状态为 active。
- 历史项目约束提示当前源代码优先，必须保持 SRP/BLE 契约、外设所有权、传感器/标定边界和运动安全门控，并严格区分静态/构建/硬件证据。

## 逐轮发现

### 第 1 轮：范围与基线

- 状态：已完成静态证据收集；运行时验证按正式报告矩阵保留为未执行。
- 初始 `git ls-files STM32H757 ESPS3 Common/SRP` 返回 1695 个受跟踪路径，其中 `STM32H757` 1626 个、`ESPS3` 69 个；`Common/SRP` 当前不存在。
- 后续 `git ls-files --stage Common` 纠正了初步假设：实际构建使用的 `Common/SCBP_CAN` 和 `Common/SmartCarLog` 共 12 个文件全部受 Git 跟踪。主范围合计为 1707 个受跟踪路径，不存在“共享源未纳入 Git”的问题。
- `STM32H757` 的 762 个 `.c`、391 个 `.h`、134 个 `.s` 等统计包含 CMSIS、HAL、静态库、工具链工程和生成材料，不能等同于 762 个项目自研 C 文件。
- 全量审计口径：项目自研代码按当前构建源集与模块逐文件覆盖；供应商/生成代码审查版本、选取、编译参数、中断/缓存/链接配置和项目集成面，不宣称重新验证供应商算法实现。
- 当前入口文档声明 CM7 是主实时核；`STM32H757/CM4/README.md` 明确 CM4 CMake 目标只是 build-structure check，不产出固件。双核拆分目前主要是目标设计而非已经运行的事实。
- `STM32H757/System/Task`、`System/Memory`、`System/Watchdog` README 仍写为 deferred/未实现边界；需以当前 CMake 和源码验证哪些陈述已过时。
- 现有 `STM32H757/Docs/ARCHITECTURE_AUDIT.md` 是早期 scaffold 审计，内容称多个接口为 placeholder；当前仓库已有实际 IMU、通信、控制与安全实现，因此只能作为历史基线，不能复用为当前结论。
- `.codex/BOOT.md`、`.codex/RULES.md` 和 `.codex/WORKFLOW.md` 明确规定 STM 是最终运动/安全权威、S3 是网关/雷达端，并要求把静态、构建、设备和整车证据分开。
- `.codex/MEMORY.md` 与 `PROJECT_STATUS.md` 含有日期较早的运行路径陈述；例如后者日期为 2026-08-07。它们用于约束和漂移检查，当前实现结论必须重新核对 2026-08-29 工作树。
- 工作区存在大量未跟踪/忽略的历史构建目录；本审计只读取必要的当前配置/现有产物证据，不把任何旧构建目录当成规范源，也不运行构建更新它们。
- CM7 与 S3 当前 CMake 均直接编译 `Common/SCBP_CAN/{scbp_crc,scbp_wire,scbp_parser,scbp_link}.c`；S3 还编译 `Common/SmartCarLog/smartcar_log.c`。历史 SRP v4 文档不是当前构建事实。
- CM7 当前 CMake 同时把 `CM7/BSP_TEST/bsp_test.c` 加入主固件源集，并另建 `BSP_TEST` object target；源注释称其为 compile-only smoke，但主固件因此也会编译这些可触发 I2C 扫描/PWM/UART 的测试入口。没有调用者，运行风险低，仍是构建边界漂移候选。
- 已补齐：完整自研构建源文件口径和生成/第三方明确边界，详见正式计划第 3 节。

### 第 2 轮：架构、启动、链接与双核现状

- `CM7/Core/Src/main.c:56` 明确当前镜像为 CM7-only；`main.c:185-232` 在 CM7 初始化外设、通信、IMU、控制/安全任务和 scheduler。
- `CM4/Core/Src/main.c:35-45,85-100` 等待 HSEM0 后进入 D2 STOP/WFE；CM7 项目代码不存在对应 release。CM4 醒后只 `HAL_Init()` 并空循环（104-129）。
- CM4 当前 CMake 只纳入 generated Core/startup 和基础 HAL，不含 FreeRTOS/OpenAMP/IPCC/应用；IOC 却在 15-16 行列出这些 IP，形成配置治理漂移。
- CM7 Flash `0x08000000`/1024 KiB、DTCM `0x20000000`/128 KiB、D2 DMA RAM `0x30000000`/288 KiB；CM4 Flash `0x08100000`/1024 KiB、RAM `0x10000000`/288 KiB。
- CM7 开 I/D cache并有 `.dma_buffer`/`.noinit`；两核没有 `.shared`、mailbox、ABI、cache ownership 或 heartbeat。
- 已确认 `STM-DUAL-001` High（双核启用阻断）、`STM-DUAL-002` Medium（IOC/CMake 漂移）、`STM-DUAL-003` Medium（共享内存/cache 合约缺失）。当前 CM7-only 路径不依赖 CM4，不能把这些前置缺口描述为当前车辆正在发生的故障。
- 审计基线提交 `d8b80c917af035415aa5a2bdf6886ba0984a17ed`，分支 `4`；审计期间出现其他任务的 4 个 S3 源文件改动，终稿按最终工作树复核。

### 第 3 轮：STM 调度、同步与时间基准

#### 任务清单（`CONFIRMED_SOURCE`）

| 任务名 | 创建位置 | 优先级 | 栈（word） | 主要周期/阻塞 | 生命周期 |
|---|---|---:|---:|---|---|
| `attitude_gate` | `Application/Safety/attitude_startup_coordinator.c:164-170` | 3 | 256 | `vTaskDelayUntil`, 20 ms | 常驻，先于电机任务 |
| `imu_task` | `Application/RTOS/imu_runtime.c:572-581` | 2 | 512 | `vTaskDelayUntil`, 10 ms | 常驻 |
| `s3_service` | `Middleware/Communication/Services/s3_service.c:381-385` | 2 | 512 | `vTaskDelay`, 1 ms；链路锁最多 20 ms | 常驻 |
| `motor_board` | `Middleware/MotorBoard/motor_board_task.c:826-830` | 2 | 384 | `vTaskDelay`, 1 ms；轮询协议 | 通过姿态门后创建 |
| `imu_data_logger` | `Application/RTOS/imu_runtime.c:583-594` | 1 | 512 | `vTaskDelayUntil`, 10 ms | 常驻 |
| `uart_link` | `Middleware/Communication/UART_Link/uart_link.c:383-387` | 1 | 384 | `vTaskDelay`, 1 ms | 常驻（前提是链路 ready） |
| `logger` | `Middleware/Communication/Services/log_service.c:127-131` | 1 | 384 | 队列接收最多 250 ms | 常驻 |
| `bmi323_task` | `Middleware/Sensor/imu_manager.c:1354-1361` | 1 | 384 | 按 ODR 相位调度；正常为 200 Hz | 双 IMU 初始化成功后创建 |
| `imu_lsm_init` / `imu_bmi_init` | `imu_manager.c:1212-1250` | 1 | 各 384 | 通知闸门，完成后 `vTaskDelete` | 初始化期间临时存在 |

稳态应用任务栈请求合计为 3328 words（约 13.3 KiB），不含 FreeRTOS idle task、TCB、队列、互斥锁和 allocator 元数据；双 IMU 初始化期间两个临时任务会短时占用额外 768 words。该数值是源码预算，不是运行时高水位。

#### `STM-RTOS-001`：健康监测名单与真实任务集不一致

- **等级/置信度：** Medium / High。
- **确定事实：** `STM32H757/System/Task/rtos_health.c:27-35` 监测名单包含 `protocol`，而当前项目自研源的任务创建名中不存在该任务；名单没有 `attitude_gate`、`motor_board`、`imu_lsm_init`、`imu_bmi_init`。`rtos_health_sample()` 在 `rtos_health.c:170-187` 通过名称查找任务，找不到时不更新该槽的当前/最小栈值。
- **机制/影响：** 诊断输出可能把不存在的 `protocol` 当成有效槽位，并遗漏姿态门和电机执行任务。BMI 任务又是条件创建的，初始化前后监测语义不同；因此不能用当前 `RTOS_HEALTH` 行证明关键执行链健康。
- **现有保护：** 栈溢出 hook 仍会记录并停机；这是越界保护，不是任务存在性覆盖。
- **建议：** 以集中式任务注册表生成监测名单，记录创建结果、生命周期、最后进度戳和 deadline miss；对条件任务明确 `ABSENT/STARTING/RUNNING` 状态。
- **验证：** 逐个禁用/延迟任务创建，确认诊断能区分缺失、阻塞和运行；检查所有实际任务的高水位和进度计数。

#### `STM-RTOS-002`：健康采样绑定低优先级 logger，缺少调度运行时度量

- **等级/置信度：** Medium / High。
- **确定事实：** `log_service.c:50-95` 只有 `logger` 任务调用 `rtos_health_sample()`，其优先级为 1；`FreeRTOSConfig.h:29-30` 关闭 run-time stats 和 trace，`configQUEUE_REGISTRY_SIZE` 为 0。
- **机制/影响：** 当优先级 2/3 任务持续占用 CPU、发生锁竞争或 logger 队列拥塞时，健康采样会延迟；现有数据无法区分“任务没运行”“采样者没运行”和“任务阻塞”。关闭运行时统计也使 CPU 占用、响应抖动和最坏执行时间无法由源码配置直接观测。
- **现有保护：** 任务使用绝对周期的路径仍可在正常负载下运行；日志队列满时会丢弃而不阻塞调用者。
- **建议：** 把健康采样放入独立、受保护的监测上下文，或让每个关键任务更新无锁进度戳，由监测者检查；保留低开销的执行计数和 deadline miss 计数，按需启用 trace/runtime stats 做验证构建。
- **验证：** 人为拉长高优先级任务、阻塞 UART/IMU 锁并测量采样延迟、进度戳年龄、CPU 利用率和恢复动作。

#### `STM-RTOS-003`：1 ms 相对延时轮询带来周期漂移和无效唤醒

- **等级/置信度：** Medium / High。
- **确定事实：** `s3_service_task()`（`s3_service.c:358-372`）、`uart_link_task()`（`uart_link.c:359-375`）和 `motor_board_task()`（`motor_board_task.c:792-818`）在可变工作量之后使用 `vTaskDelay(pdMS_TO_TICKS(1U))`；相同模块中的 IMU/姿态任务使用 `vTaskDelayUntil()`。
- **机制/影响：** 相对延时把本轮处理时间叠加到下一周期，导致轮询频率随报文数量、日志和 HAL 延迟变化；1 kHz 空轮询还增加上下文切换和 tick 唤醒，可能挤压同优先级 BMI/日志任务。电机任务同时在一次循环内 `while (MB_Protocol_Poll())` 清空所有可用帧，输入突发时其执行时间无静态上界。
- **现有保护：** 电机和链路均有超时/强停路径；FreeRTOS 抢占可打断低优先级任务。
- **建议：** 后续实施阶段优先改为事件/通知唤醒；若必须轮询，使用绝对 deadline、限制每次处理帧数并记录 overrun，不改变现有协议和安全门。
- **验证：** 对 UART/MotorBoard 注入最大突发帧，测量循环 WCET、周期抖动、队列/环形缓冲水位和电机命令响应延迟。

#### `STM-RTOS-004`：动态堆与任务创建失败处理分散，关键能力没有统一 admission 结果

- **等级/置信度：** Medium / High。
- **确定事实：** `FreeRTOSConfig.h:16,32-34` 将 heap_4 动态堆固定为 32 KiB、关闭静态分配；所有应用任务和 `xSemaphoreCreateMutex()`/`xQueueCreate()` 均走动态分配。`imu_runtime_start()` 是 `void` 且仅记录失败（`imu_runtime.c:572-595`）；`uart_link_task_start()`、`s3_service_start()`、`log_service_start()` 失败也只写日志或丢弃计数。`configUSE_MALLOC_FAILED_HOOK=1`，hook 在 `stm32h7xx_it.c:364-368` 记录后永久停机。
- **机制/影响：** 启动时资源不足会形成部分任务集，调用者没有统一的“关键任务未就绪”结果；诊断任务本身也可能因同一堆耗尽而无法创建。当前静态源码不能证明 32 KiB 在所有编译选项、日志长度和初始化时序下都有余量。
- **现有保护：** 电机姿态门和各任务内部有若干创建失败分支；heap_4 会合并空闲块并提供最小剩余量。
- **建议：** 在实施阶段建立启动 admission 表：逐项记录 TCB/栈/队列/锁分配、失败等级和安全回退；关键运动任务失败必须明确保持 zero-PWM，非关键日志可降级。优先评估静态对象或按内存域预留，但不在本次审计中改代码。
- **验证：** 记录每次分配的大小、峰值和最小剩余堆；逐个注入分配失败，确认 CM7 仍保持停机权且能保留故障原因。

#### `STM-RTOS-005`：时间基准与中断屏蔽边界需要运行时量测

- **等级/置信度：** Info / High。
- **确定事实：** FreeRTOS 与 HAL 共用 1 kHz SysTick（`CM7/Core/Src/stm32h7xx_it.c:326-333`）；IMU 周期/状态多使用 DWT 扩展的 `imu_time_now_ms/us()`，UART、S3、姿态门和 MotorBoard 多使用 `HAL_GetTick()` 或 `xTaskGetTickCount()`。DWT 读数在 `BSP/TIMER/bsp_timer.c:42-52` 通过临时 `__disable_irq()` 保证回绕原子性。
- **机制/影响：** 三种读取路径在时钟重配、调试暂停、DWT 不可用或长时间关中断时可能有不同观测；源码没有 jitter/WCET 或关中断时长计数，因此实时裕量不能静态下结论。
- **建议/验证：** 统一记录任务释放时间、完成时间和 tick/DWT 对照；在目标板测量 SysTick、DMA/USART 中断延迟及 `bsp_timer_get_us()` 关中断窗口，作为双核迁移门。

#### `STM-DUAL-004`：CM4 链接起始地址会与 CM7 DMA 别名重叠（条件性 High）

- **等级/置信度：** High / High（触发条件为真正释放并运行当前 CM4 镜像）。
- **确定事实：** CM7 linker 将 `.dma_buffer` 放在 `0x30000000`（`STM32H757/CM7/stm32h757xx_flash_CM7.ld:214-221`；现有 Debug map 显示 `0x30000000-0x30000200`，由 `uart_link.c:26-28` 使用）。器件头文件同时定义 `D2_AXISRAM_BASE=0x10000000` 与 `D2_AHBSRAM_BASE=0x30000000`（`Drivers/CMSIS/Device/ST/STM32H7xx/Include/stm32h757xx.h:2297-2298`）。CM4 linker 将普通 `RAM` 从 `0x10000000` 开始（`CM4/stm32h757xx_flash_CM4.ld:38-42`）；现有 CM4 Debug map 的 `.data/.bss` 占用 `0x10000000-0x10000030`。
- **机制/影响：** 两个地址是同一 D2 SRAM 的不同总线别名；CM4 启动时清零/初始化其 `.data/.bss`，在 CM4 被释放时可能覆盖 CM7 UART DMA 缓冲的前 0x30 字节，造成帧损坏、丢包或链路恢复异常。当前 CM7-only 路径不触发该条件，但这不是可接受的双核默认布局。
- **建议：** 双核实施前为 CM4/CM7 划定不重叠的物理 SRAM 子区，并在 linker、MPU、cache 属性和启动清零范围上做一致性审计；DMA 缓冲不得与 CM4 普通 `.data/.bss` 共用别名区。将 map overlap 检查设为发布门。
- **验证：** 用最终双镜像 map 对物理地址而非虚拟别名做区间检查；CM4 启动/复位期间持续观测 UART DMA 内容、cache clean/invalidate 和 IPC 序号。

## 双核设计候选

- 当前最先需要的是 CM4 boot-ready/heartbeat/no-op 阶段，不是直接迁移任务。
- 后续候选继续比较：维持单核；CM7 实时安全 + CM4 低风险服务；传感/计算拆分；独立安全核。当前不预设最终答案。

### 第 4 轮：STM BSP、驱动、中断、DMA 与外设所有权

#### `STM-BSP-001`：BSP GPIO 初始化会覆盖 USART1 RX 复用

- **等级/置信度：** High / High；触发条件是启用 BMI323/`bsp_gpio_init()` 后继续依赖 USART1 接收。
- **确定事实：** `STM32H757/CM7/Core/Src/main.c:185-191,220-226` 先调用 `MX_USART1_UART_Init()`，再启动 IMU runtime；`STM32H757/Middleware/Sensor/BMI323/bmi323_port.c:103-117` 的初始化路径调用 `bsp_gpio_init()`。`STM32H757/BSP/GPIO/bsp_gpio.c:44-56` 把 PA10 配置为普通 `GPIO_MODE_INPUT`，而 `STM32H757/CM7/Core/Src/stm32h7xx_hal_msp.c:319-329` 已把 PA10 配置为 USART1 alternate function；`main.h:78-79` 同时将该脚标为 `LF_INT1`。
- **机制/影响：** 后续 BSP 初始化覆盖了 USART1 RX 的 AF 配置。USART1 发送可能仍工作，但 CH340/调试日志回传、命令接收或故障导出会出现单向失效；这是确定的资源所有权冲突，不是仅凭旧文档推测。
- **现有保护：** IOC 将 PA10 锁定为 LF_INT1，且 LF_INT2/PA13 的 SWD 冲突有注释；没有运行时 pin-owner 检查，也没有在 `bsp_gpio_init()` 后恢复 USART1 AF 的保护。
- **建议：** 在后续实施前建立单一 pin ownership 表，明确 USART1 RX 与 LF_INT1 的互斥关系；选择保留调试 UART 或重新分配传感器脚位，禁止多个初始化器无条件重配同一 GPIO。建议保留当前协议和日志接口不变。
- **验证：** 仅在获批的验证分支/目标板上读取 PA10 MODER/AFR 前后值，做 USART1 RX 回环和 IMU 初始化前后对照；静态审计不宣称已发生现场故障。

#### `STM-BSP-002`：SPI/I2C 公共 BSP 没有设备级串行化合同

- **等级/置信度：** Medium / High。
- **确定事实：** `STM32H757/BSP/SPI/bsp_spi.c:145-197`、`BSP/I2C/bsp_i2c.c:37-87` 直接调用 HAL 传输，没有 BSP 层 mutex/所有者参数。BMI323 manager 另有 `bmi_driver_mutex`，但 BSP_TEST 和未来调用者可直接进入同一接口；I2C 的 `write_read` 由独立 transmit/receive 组成。
- **机制/影响：** 当前调用集合主要由单一 IMU 生命周期串行执行，因此现状未证明有并发故障；然而接口本身允许多个任务同时改变 SPI 状态/CS 或交错 I2C 事务，迁移到 CM4 或增加诊断任务时会产生撕裂事务、错误寄存器地址或 HAL 状态竞争。它也是应用层独立性和双核迁移的隐藏耦合。
- **现有保护：** BMI323 manager 的 driver mutex 覆盖其已知路径；LSM303 当前没有 DMA/IRQ 并发路径。
- **建议：** 把总线所有权写入接口契约，采用每总线单一 worker 或在 BSP 层统一锁定，并明确重复起始（repeated-start）语义；不要让调用方直接操作 CS/寄存器。
- **验证：** 两个任务交错读写、插入超时和复位，检查完整事务、HAL error、CS 波形和设备寄存器回读；记录锁等待和最坏占用。

#### `STM-BSP-003`：UART2 ReceiveToIdle DMA 在 UART IRQ 回调中立即重装

- **等级/置信度：** Medium / Medium（依赖所用 STM32 HAL 版本对该 API 的 ISR 安全保证）。
- **确定事实：** `STM32H757/CM7/Core/Src/stm32h7xx_it.c:348-351` 在 `USART2_IRQHandler` 调用 HAL IRQ；`STM32H757/Middleware/Communication/UART_Link/uart_link.c:308-320` 的 `HAL_UARTEx_RxEventCallback()` 在回调中执行 `dcache_invalidate()`、环形队列临界区，并直接调用 `uart_link_start_dma_receive()`；后者 `:146-168` 再调用 `HAL_UARTEx_ReceiveToIdle_DMA()` 和 DMA 中断配置。
- **机制/影响：** 若 HAL 接收重装路径获取内部锁、修改与当前 IRQ 共享的状态或依赖任务上下文，回调中的递归/并发访问可能返回 BUSY、丢失下一帧或留下 DMA inactive 窗口。源码没有 deferred rearm、重装次数上限或明确的 HAL ISR 合同，因此只能判为静态风险。
- **现有保护：** `s_dma_active`、错误计数、重启请求和 DMA 缓冲 cache line 对齐；重装失败会设置 `s_restart_requested`。
- **建议：** 以“IRQ 只确认事件、任务上下文重装”为候选方案；若保持当前做法，应锁定 HAL 版本并用源码/厂商文档确认 ISR 安全，增加可观测的 rearm failure/idle gap 计数。
- **验证：** 以连续满帧、IDLE/满缓冲交替、UART 错误和高优先级负载压测，测量重装延迟、DMA inactive 时间、丢帧和 HAL 状态。

#### `STM-BSP-004`：USART6 中断接收无背压，突发时只计数丢弃

- **等级/置信度：** Medium / High。
- **确定事实：** `STM32H757/Middleware/MotorBoard/motor_board_transport_uart.c:86-95` 在 RX ring 满时丢弃字节并递增 `s_rx_overflow`；`STM32H757/Middleware/MotorBoard/motor_board_transport_uart.c:246-264` 在 IRQ 中循环排空所有 RXNE 字节，但没有通知任务、流控或帧级丢弃标记；`motor_board_task.c:792-818` 任务随后以 1 ms 轮询解析。
- **机制/影响：** 输入突发或任务被高优先级抢占时，512 字节环形缓冲可能丢掉帧中间字节，协议层只能看到未知/无效文本；计数器可观测但不会触发链路重同步或运动降级。实际阈值取决于波特率和负载，需测量。
- **现有保护：** 协议解析器有长度上限和 invalid frame 统计；MotorBoard 有超时/强停序列。
- **建议：** 保留当前 UART/文本契约，补充帧边界丢失标志、有限批处理预算和 overflow -> link recovery/zero-PWM 策略；不要在 IRQ 中执行不可界定的协议解析。
- **验证：** 注入最大响应突发、同时施加 CM7 高负载，记录 ring 水位、overflow、invalid frame、恢复时间和电机输出。

#### `STM-BSP-005`：BMI323 初始化/事务包含忙等待，影响实时预算

- **等级/置信度：** Medium / High。
- **确定事实：** `STM32H757/Middleware/Sensor/BMI323/bmi323_port.c:195-234` 与 `Drivers/IMU/BMI323/bmi323.c:180-185` 以 DWT 时间忙等待微秒/毫秒延时；这些路径可在调度器启动前和初始化任务中执行。
- **机制/影响：** 忙等待占用 CM7 计算资源并延长其他任务/中断响应；复位重试或日志开启时成本会放大。源码给出了延时上限，但没有 CPU 利用率、IRQ 延迟或初始化最坏时长测量。
- **现有保护：** 延时函数使用单调时间基准，诊断日志有次数限制；初始化失败会被上层记录。
- **建议/验证：** 将忙等待纳入启动和任务 WCET 预算；后续可评估硬件定时器/阻塞等待，但不得在本审计中直接改动传感器时序。目标板测量延时误差、IRQ 延迟和启动峰值。

#### `STM-BSP-006`：GPIO/调试脚位的历史兼容枚举仍暴露不可用资源

- **等级/置信度：** Low / High。
- **确定事实：** `BSP/GPIO/bsp_gpio.c:14-16` 保留 LF_INT1/2 映射，`bsp_gpio_init():44-56` 明确 PA13 仍由 SWD 保留；`Smart_Car_H757.ioc:102-107` 将 PA13 锁为 Serial_Wire。`BSP/PWM/bsp_pwm.c:9-15` 也保留 TIM3 CH1/CH2 枚举但拒绝使用。
- **机制/影响：** 接口表面看似可用，调用方若只依据枚举而忽略返回值，会把调试脚或已退役 PWM 当成真实外设；这增加应用独立性和测试误用风险，但当前已返回错误，未判为运行故障。
- **建议/验证：** 在模块索引和接口契约中标明 `UNSUPPORTED/DEBUG_CONFLICT/RETIRED` 状态，并在静态检查中禁止生产路径调用；验证所有调用者检查返回值。

## 证据限制

- 本任务不烧录、不连接设备、不抓取 UART/BLE/Wi-Fi、不进行车辆测试。
- 静态检查可以确认实现路径和潜在缺陷，不能确认实际 CPU 占用、最坏执行时间、栈高水位、链路误码率或物理控制效果。
- 文件数量只是范围证据，不是质量或覆盖率结论；最终报告必须说明覆盖口径和排除项。

### 第 5-12 轮终审整合（2026-08-30）

- **第 5 轮：** 回读 DualAHRS、BMI323 双实现、fast-zero 标定和高速标定锁竞争；形成 `STM-AHRS-001`、`STM-IMU-001`、`STM-ATT-001`、`STM-CAL-001`。
- **第 6 轮：** 回读 attitude gate、MotorBoard 控制/遥测和恢复路径；形成 `STM-SAFE-001`、`STM-ARCH-001`、`STM-CTRL-001`，并关闭旧的 `imu_recover` 过时候选。
- **第 7 轮：** 回读 S3 `app_main`、STM UART、BLE、command bridge、雷达 UART/parser/FIFO、telemetry queue 和 Wi-Fi/TCP uplink；形成 `S3-BOOT-001`、`S3-UART-001`、`S3-BLE-001`、`S3-SVC-001`、`S3-RADAR-001..002`。
- **第 8 轮：** 回读 App `AA 55` 与 SCBP-CAN `5A A5` 双层编解码、CRC、ACK pending、重试、BUS_OFF 和启动 ready；形成 `X-STATE-001`、`X-PROTO-001..002`。
- **第 9 轮：** 交叉检查 HAL/IDF、BSP、transport、driver、protocol、service、app 依赖方向，全局状态和 telemetry sink ownership；确认反向依赖和服务汇合点。
- **第 10 轮：** 汇总 CM7 任务栈/heap、S3 UART/BLE/雷达/ACK 容量、复制和轮询边界；形成 `EFF-001`，不推导未经测量的 CPU 百分比。
- **第 11 轮：** 将姿态 freshness、BLE 断连、BUS_OFF、CM4 失联、过期 telemetry 纳入统一验证矩阵；形成 `SAFE-REC-001`。
- **第 12 轮：** 以 CM4/CM7 内存别名、外设 owner、IPC 缺口和安全最终权为约束，确定“CM7-only 基线 -> CM4 heartbeat -> 低风险服务”的推荐迁移路线；终审回读报告行号、发现编号、事实/建议边界和工作树状态。

上述轮次均为源码/配置静态审计或文档交叉复核；未烧录、未抓包、未运行车辆，详见正式总报告验证矩阵。

## 资源

- 项目根：`/Users/zhiqin/Projects/Smart_Car`
- 审计主范围：`STM32H757/`、`ESPS3/`、`Common/SCBP_CAN/`、`Common/SmartCarLog/`
- 接口上下文：仅在验证跨模块契约时读取 App/ROS2/工具侧，不扩展为其全量审计。
- 正式计划：`DOCS/STM_S3_FULL_CODE_AUDIT_PLAN.md`。
