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

## 2026-08-31 当前 SRPv4 第 13-17 轮增量证据

### 快照和前提

- 分支 `codex/s3-stm-cn-comments`，HEAD `f703453727a136d15ff7cacea4530beab6e9c08a`。
- CM7 和 S3 当前 live CMake 共享 `Common/SRP/{srp_crc,srp_wire,srp_codec,srp_link}.c`；活动 STM-S3 合同是 SRPv4。
- 工作区有大量并行未提交改动。本审计仅写审计 Markdown，不回滚、不归因、不提交。
- 本次未构建、未运行主机测试、未烧录、未读 option bytes、未做 UART/BLE/Wi-Fi/目标板/车辆验收。

### 第 13 轮：SRPv4、CM7 控制与安全

- `chassis_task_start()` 除声明/定义外无调用者；`CHASSIS_SPEED_CMD`/`CHASSIS_HEADING_CMD` 可保存目标和 ACK，但 10 ms 底盘任务不运行。
- `WHEEL_SPEED_CMD` 在同步后直达 `motor_board_set_target_wheel_speeds()`，后者仅查 finite/范围；新非零命令可在姿态门撤销后清掉 forced-stop。
- MotorBoard 无 WAIT_FEEDBACK/READY、MSPD 时间戳和 200 ms feedback watchdog；PID 仍固定 `dt=0.05 s`。
- `motor_board_force_stop()` 依赖普通 512 B USART6 TX FIFO；队列满时 zero-PWM 可丢失，已排队旧 PWM 可先发。
- CM7 fault/assert/stack/malloc 致命路径最终关中断并永久停机，没有 IWDG/WWDG 或独立执行器 disable/brake。
- DualAHRS `s_dual` 由低优先级 BMI task 分步写，gate/chassis/telemetry 无锁读多字段和 64-bit 时间戳。
- PA9 依次被 USART1/TIM1 重配，PA10 依次被 USART1/LF_INT1 重配；最终 owner 与启动注释不一致。
- SRPv4 codec/header/注册表/boot-info 锁边界一致；ACK/ERROR 接收仍只要求 payload `>=4`，不查精确长度、reserved 和 flag 一致性。当前 host 测试主要覆盖 codec，没有 link ACK/重试/BUS_OFF 状态测试。
- 详细证据见 `round13-14-stm-current.md`。

### 第 14 轮：CM4 启动、D2 内存、RTOS/IPC 和 owner

- CM4 无条件等 HSEM0，CM7 没有释放；CM4 醒后仅 `HAL_Init()` 和空循环。
- CM4 Reset_Handler 在进入 HSEM 等待前已经初始化 `.data/.bss`。CM4 `0x10000000` 与 CM7 DMA `0x30000000` 是 D2 物理别名；旧 map 还显示 `uwTick` 在 DMA 前 512 B 范围内的等价偏移。
- 两核无 `.shared`、版本化 mailbox、cache clean/invalidate 规则、heartbeat、age timeout 或 reset epoch。
- CM4 source list 无 FreeRTOS/OpenAMP/watchdog/业务源，SVC/PendSV 空，无已集成 CM4F port；startup 还错标 `.cpu cortex-m7`。
- SPI/I2C、IMU、DualAHRS、姿态门、USART2/6、MotorBoard 和最终 zero-PWM 必须继续属于 CM7。CM4 第一阶段只能是 no-op/heartbeat 和低风险诊断消费者。

### 第 15 轮：S3 UART、BLE、雷达、Wi-Fi/TCP 和资源

- BLE 断连回调只清 motion pending 并置 stop flag，没有清 BLE RX 队列或撤销 V2 session。service 先发 zero，再消费断连前旧队列，因此非零命令可在 stop 后再次下发。
- FFE1 是普通 write 权限，当前无 BLE 加密/配对/peer 授权强制；V1 写入不要求 FFE2 CCC 或 V2 session。
- `S3 SYSTEM READY` 在 UART/BLE/radar/uplink/service 失败后仍可打印，不等价于系统能力就绪。
- UART 硬件错误会设 discontinuity，软件 ring/mutex 丢字节只计数，service 不会立即 reset parser。
- FFE2/FFE3 多生产者直接分片 send-indicate，无 TX 锁/队列和 congestion 状态机；MTU 23 下多片日志可交错。
- TCP connect/backoff/burst 有界，但已连接 socket 持续 WAIT 无 deadline/keepalive/pending stale 重检。
- S3 task stack 单位是 bytes：STM RX 3072、radar UART 4096、service 16384、uplink 6144；部分 HWM 日志错标为 words。
- Kconfig 默认关闭 uplink，当前本机 ignored `sdkconfig` 已开；ignored 凭据头内容未复制到审计文档。S3RD type 2 仍是实验 ID。

### 第 16 轮：App-S3-STM、ROS2、控制权与失联

- 控制页红色急停调用 `emergencyStop() -> send(.stop)`，不清 wheel target、不停 100 ms heartbeat。该帧是 V1 `CONTROL 0x01 / STOP 0x01`，S3 无该 type 分支并返回 rejected；App 不展示该拒绝。
- `BRAKE -> emergencyWheelBrake() -> sendZeroWheelSpeeds()`、摇杆回中、失焦/隐藏和主动断连是不同的有效零四轮路径。
- App 编解码固定 V1，完全未使用 S3 已实现的 V2 500 ms heartbeat、3 s TTL、sequence 去重和 valid-for。S3 独立 SRP heartbeat 会继续维持 CM7 链路，它不能代替 App motion lease。
- ROS2 bridge 只发 `/scan` 和 `/diagnostics`；telemetry type 2 只做诊断，无 `/odom`、`/cmd_vel`、控制 subscription 或 `controller_manager`。默认 `transport: unconfigured`，是安全边界而不是缺陷。

### 第 17 轮：RTOS、调试配置、文档与终审

- CM7 RTOS health 表使用不存在的 `uart_link/protocol`，漏 `attitude_gate/motor_board/chassis_task`；找不到任务只保持 0 且无 missing 标志，日志又未输出全部槽位。
- CM7 在创建 `srp_uart/s3_service/attitude_gate` 和启动 scheduler 前打印 `RTOS READY/SYSTEM READY`；S3 对大多数初始化失败也仍打印 READY。
- `Common/SmartCarDebug/` 当前未跟踪，但 CM7/S3 已直接 include。普通 `cmake -D<name>=<value>` 只建 cache 变量；当前只有显式 `target_compile_definitions()` 的少数开关真正进 compiler。`SMARTCAR_SCHEDULER_PROBE` 当前无源码使用点。
- `.codex/MEMORY.md`、`PROJECT_STATUS.md`、`DOCS/architecture/system.md`、`DOCS/esp32s3/ble.md`、`DOCS/ros2/ros2.md`、`DOCS/code_map.md` 仍混有旧协议、BMI paused、BLE relay 未连接、ROS2 无 runtime 和不存在的 shared App V2 路径。
- `git diff --check` 在终稿前的只读快照无输出；最终文档修订后需再次执行。

### 当前停止条件

- 在 App 急停、S3 断连队列、end-to-end motion lease、CM7 统一非零准入、MotorBoard feedback watchdog/优先 stop 和 CPU hang 独立停机证据取得前，不进入车辆运动验收。
- 在 CM4 option bytes、启动握手、物理内存分区、CM4F RTOS、IPC/cache/reset 和故障注入全部通过前，继续保持 CM7-only。
- 静态审计完成不代表当前脏源码已构建，也不代表已烧录或完成 UART/BLE/Wi-Fi/目标板/车辆验收。

## 2026-08-31 第 18-27 轮深化审计摘要

### 快照与覆盖

- 分支 `codex/s3-stm-cn-comments`，HEAD `f703453727a136d15ff7cacea4530beab6e9c08a`；当前结论基于未提交脏工作树，不是 release tag。
- 详细证据见 `round18-27-current.md`；终稿前 16 个关键 SRP/S3/ROS2/CM7/App 源文件通过 SHA-256 稳定性复核。
- 本十轮没有构建、测试、烧录、读 option bytes、抓 UART/BLE/TCP、连目标板或运行车辆。

### 十轮新证据

| 轮次 | 新证据/独立修正 |
|---:|---|
| 18 | `Common/SmartCarDebug` 未跟踪；S3 fresh target 未固定，IDF 可回落 `esp32`；uplink 启用态无 tracked secret 模板；死 CMake/cache 和重复 BSP test |
| 19 | S3 BUS_OFF 没有 stop 屏障；transport failure 不受 3 次 retry 限制；ACK 过宽/无 epoch；ACK-required 无 replay cache；callback 重入和 parser 半帧超时缺失 |
| 20 | S3 `uart_write_bytes()` TX ring 入队用 IDF `portMAX_DELAY`；recover 不清旧 TX；CM7 worker 可在不取 TX mutex 时 abort/deinit HAL handle；两端 discontinuity 不完整 |
| 21 | BLE disconnect/V2 HELLO 未绑 connection epoch；V2 valid-for 只防排队过期，不终止已执行目标；零 chassis 命令只清本地状态 |
| 22 | MotorBoard FAILED/未 RUNNING 不拦非零；无 MSPD feedback watchdog；PID 当前固定 50 ms dt；普通 FIFO/RX re-arm/IRQ 无法构成可靠物理 stop |
| 23 | DualAHRS publish 层仍无原子快照；BMI failure 会 freshness 锁车但无 dynamic recovery；BMI gyro-Z 单轴翻转与共用刚体变换注释矛盾 |
| 24 | S3 service 一 tick 实为 10 ms；task WDT 只监 idle task 且不 panic；关键任务未注册 progress watchdog；两端 partial-start 缺统一 admission/回滚 |
| 25 | App 在 GATT service/characteristic/CCC 前即设 connected 并启用控制；disconnect zero 没等 `.withResponse`；S3 GATT `1032` 超 IDF 公开 `517`；BLE ready/TX 状态分散 |
| 26 | live TCP 无 TLS/认证，CRC 不是 MAC；单静默 client 可占 listener；S3 pending 旧包到 host 后按新 receive time 通过 stale 门；S3 timestamp 未映射 ROS time |
| 27 | operator/session/transport/feedback 四类 lease 被明确分层；当前无 end-to-end motion lease；车辆、release、SRP/UART、BLE、ROS2 live、CM4 分别设立停止门 |

### 新增高优先级链路

- `S3 BUS_OFF -> 100 ms recover -> 重新 sync` 可早于 CM7 `200 ms` 失联强停，而 S3 BUS_OFF callback 未发 zero/未清 motion/session。
- `srp_link_recover()` 清 pending 不回调，S3 `s_motion_tx_in_flight` 可永久卡住；transport failure 又不增 retry_count。
- S3 所谓 100 ms UART TX 上限不覆盖 IDF TX-ring 入队；旧 motion/ACK 可在 recover 后继续物理发送。
- App “connected”不等于 command/telemetry ready，“transmitted”不等于 GATT/S3/STM 交付；主动 disconnect 前 zero 也无完成屏障。
- MotorBoard lifecycle state 不参与非零准入，MSPD 停流无 watchdog，CPU fault 时普通 UART IRQ stop 无法保证。

### 关闭、纠偏与仍开放项

- **关闭：** LSM303 DRDY 掩码已正确；姿态 freshness 已持续撤销 ready；BLE RX callback/当前 telemetry relay 已连接；`data[7]/data[6]` 是 SRPv4 priority/type；CM7 DMA buffer 位于 DMA 可达 D2 SRAM。
- **纠偏：** 不得笼统称“整个 IMU 链无锁”，manager/calibration/boot 已有各自 mutex；开放问题是 DualAHRS publish 层。
- **重开：** 当前 MotorBoard 使用固定 `dt=0.05 s`，历史“动态 dt 完成”不适用当前工作树。
- **仍开放：** CM4 D2 别名/HSEM/RTOS/IPC；App 急停/lease；CM7 direct wheel gate；DualAHRS snapshot；BLE 授权/拥塞；canonical 文档漂移。

### 十轮后的停止条件

- 车辆运动前 P0：急停、BLE epoch、operator lease、CM7 统一非零 gate、MotorBoard RUNNING/feedback/priority stop、CPU fault 独立停机。
- 可复现发布前 P0：clean checkout 调试头、S3 target、uplink 模板、CMake/cache/admission 关闭。
- ROS2 live/自主前 P0：TCP 认证/对端完整性、client/frame deadline、source age/time mapping 和 S3RD schema 联合冻结；否则保持 `unconfigured`/replay/PoC。
- CM4 启用前 P0：option bytes、HSEM boot-ready/timeout、物理内存分区、CM4F RTOS、mailbox/cache/reset/heartbeat 和 CM7 单核回退。

## 2026-08-31 第 28-37 轮深化审计摘要

### 快照与边界

- 分支/HEAD 仍为 `codex/s3-stm-cn-comments` / `f703453727a136d15ff7cacea4530beab6e9c08a`，但同一 HEAD 下并行脏源码在审计期间继续演化；启动 status 147 项，终稿前一度增至 151 项。
- 详细证据见 `round28-37-current.md`。本轮只修改审计 Markdown，没有修改、格式化、回滚、提交或 push 固件/App/config/generated/secret/build 文件。
- 未执行 build、host test、烧录、eFuse/option-byte 读取、UART/BLE/TCP 抓取、RTOS/WCET 量测、目标板或车辆验收。

### 十轮独立结论

| 轮次 | 核心结果 |
|---:|---|
| 28 | App 缺 0x29/0x2C type；logical SRP struct 受 pack(4)；S3RD 三份常量/CRC；binary32 合同不完整 |
| 29 | BLE callback 跨 owner 写 motion；MotorBoard 长 critical；link callback 重入；BLE 多字段无 epoch snapshot |
| 30 | DWT 多回绕漏高位；V2 sequence 不支持 32-bit wrap；boot zero-sentinel；多数短 deadline 已 wrap-safe |
| 31 | PID/SYS_CONFIG 共用 callback context/baud；motion overwrite 无 superseded；V1/fast ACK/log 返回值语义不足 |
| 32 | ROS resync 近 O(n²)且 payload max 过大；App queue/disk 无界；CM7 raw-log 调试栈风险 |
| 33 | UART2/USART6 同级 ISR/WCET + task critical；FFE3 CCC 可向自身 BTC queue 无限等待回投；raw diagnostics 整段关 IRQ |
| 34 | 活动凭据进入公开 GitHub/产物/历史；release 未锁 secure boot/flash encryption；BLE/App 身份未认证；日志隐私 |
| 35 | physical connected 解锁控制；STOP/计数不代表执行；PID 无 sequence/timeout；terminate zero 无 barrier |
| 36 | App 必需 `SessionLogWriter.swift` 未跟踪；无 App test target/仓库 CI；motion/fault 主链无项目测试 |
| 37 | 36 项去重为 1 Critical/10 High；建立 S0/R0/P0/M0/T0/N0/D0/V0 门；CM4 建议不变 |

### Critical：秘密事件

- ignored 活动 Wi-Fi 凭据的一组值与 tracked 历史文件完全相同；GitHub 当前元数据确认仓库是 PUBLIC，公开默认分支包含引入秘密的初始提交。
- 两个 tracked Wi-Fi 文件至少含五组唯一凭据对；三份 tracked shared header 还重复保存一组非占位 SoftAP 凭据。活动 SSID/password 两个字段均能在现有 S3 ELF/BIN 中匹配。
- 所有比较只输出路径、条目数、相等/命中关系；账本、计划和报告均不保存 SSID/password、可逆编码或可用于复原的内容。
- 必须先在网络侧轮换/撤销并审计未知客户端；history rewrite/远端 ref 清理需单独授权和多方协调，不能替代轮换，本轮没有执行。

### 去重后的 High 根因

- **owner/transaction：** BLE disconnect callback 跨 owner 写 motion；PID/SYS_CONFIG 共用可变 callback context。
- **BLE progress：** FFE3 CCC 在 BTC task 内同步 flush，默认 MTU 23 和足够积压可填满容量 100 的自身 queue 并进入无限等待。
- **firmware/device trust：** 当前配置未启 secure boot/flash encryption/NVS encryption；FFE1 不要求 encrypted/MITM/peer authorization；官方 App 只按 name/公开 UUID 选车。
- **operator truth：** App 以 physical connected 解锁控制；红色 STOP 无效，非 PID 命令无分阶段结果。
- **release/test：** App 必需源未跟踪，安全控制主链无项目测试或 tracked CI。

### 明确关闭/保留

- 锁定 STM HAL 的 IDLE/TC/error callback 在项目 callback 前已恢复 READY/unlock；关闭“re-arm 必然因 HAL BUSY/内部锁失败”，保留 ISR WCET/RX inactive gap。
- CM7-only UART2 DMA 当前 line alignment、D2 placement、pre-arm/actual invalidate 和 barrier 完整；关闭当前 cache/可达性误报，保留 CM4 D2 物理别名覆盖。
- live TCP 无 TLS/auth、单 client、source-time/freshness 问题沿用第 26 轮，不重复计为第 34 轮新 High；tracked 默认 uplink off 和 ROS `unconfigured` 仍是保护。
- 轮序 `[M1:RR,M2:RF,M3:LR,M4:LF]`、193.0 mm、trim、符号和安全门继续冻结；第 28-36 轮没有支持提前启用 CM4 的证据。

### 独立停止门

- S0：轮换/撤销秘密并扫描 all refs/objects/artifacts/caches。
- R0：clean archive App/S3/CM7、固定 target/security profile、工具链/config/source/artifact 可追溯。
- P0/M0：严格 ACK/replay/epoch/context；有效急停、operator lease、CM7 final gate、MotorBoard READY/feedback/priority stop/CPU-fault stop。
- T0：ISR/critical WCET、HWM、heap、queue/ring waterline 长时实测。
- N0：ROS2 live 认证、新鲜度、deadline、source time/epoch 和线性 assembler；否则保持 `unconfigured`。
- D0：CM4 boot/物理内存/CM4F RTOS/mailbox/cache/reset/heartbeat/单核回退；否则保持 CM7-only。
- V0：前置门通过且匹配镜像完成架空轮、低速台架、受控实车签收后，才可声明车辆 READY。

## 2026-08-31 第 38-47 轮深化审计摘要

### 快照与覆盖

- 起始快照：2026-08-31 04:54:08 CST，`codex/s3-stm-cn-comments` / `f703453727a136d15ff7cacea4530beab6e9c08a` / status 151项。
- 详细证据见 `round38-47-current.md`；本轮只写审计Markdown，不修改/回滚/提交源码、App、配置、secret、generated或build。
- 第38-46轮共33项：0新增Critical、2 High、5 High/Medium组合；第34轮公开secret Critical仍开放。

### 十轮结论

| 轮次 | 新证据/独立关闭 |
|---:|---|
| 38 | fault strict-alias、ROS窄化/索引、C/C++ header、当前不可达vendor lock UB、诊断profile |
| 39 | peer boot epoch/re-arm、early physical stop、reset cause、retained fault交付 |
| 40 | NVS整擦、OTA rollback owner、16/32 MB分区表、session日志原子完成 |
| 41 | heading min-wheel保持差速却改变平均applied v |
| 42 | BMI/LSM timestamp/quality保护；内部cal result无活动端到端合同；新odometry wheel/yaw时刻不一致 |
| 43 | STM UART/uplink/service init失败rollback与commit |
| 44 | MotorBoard通用ACK不绑定当前step；Swift control parser无上限/O(n²) |
| 45 | wheel source freshness、ONLINE/READY反向语义、counter/snapshot/log loss |
| 46 | ROS/GCC锁定、LICENSE/SBOM、YDLidar来源、CubeMX/Swift generator provenance |
| 47 | 八根因族去重、停止门更新、冻结合同和CM4结论 |

### 带High等级根因

- **Reset/re-arm：** peer无boot epoch，CM7单独重启不使BLE断开，旧App heartbeat可在zero后未经新授权恢复非零。
- **Earliest stop：** physical zero晚于clock/GPIO/USART6，VOS/PLL/UART早期hang没有板端watchdog/enable-brake证据。
- **Control numeric：** heading min-wheel给两侧同加偏置，低速requested v可能被显著提升。
- **Lifecycle：** S3 STM UART task失败残留driver/mutex；uplink中后段失败残留network/event资源。
- **Response correlation：** MotorBoard任意OK/ACK/Set文本可推进当前配置step，未绑定事务。
- **Feedback truth：** MSPD停流后最后wheel数组仍50 ms重发，App receipt time把旧值伪装为新鲜。

### 关键Medium与关闭项

- NVS整区擦、双OTA无owner/rollback、旧分区表误选和App session截断不可辨识属于release/persistence门；当前没有项目OTA入口。
- 标定内部timestamp/motion/quality/leveling gate存在；Common/CM7/S3无cal-result发送合同，App 0x25 decoder孤立，内部结果与provenance均无法对外验收。
- S3 App/SRP C parser固定容量线性、App log parser有上限；Swift control parser无上限且坏候选前删。
- App ONLINE可与SRP sync相反，Radar/TCP READY只表示本地阶段，多项counter无epoch/出口，radar snapshot失败伪装零值。
- IDF lock、YDLidar version/license、CMSIS/HAL/FreeRTOS许可和无submodule是保护；ROS image/apt、CM7 GCC、generator、vendor import与artifact-specific SBOM未锁。
- 当前HAL ReceiveToIdle READY顺序和CM7 UART DMA cache边界继续关闭；CM4 D2 alias、旧motion/secret安全链继续开放。

### 停止门更新

- S0不变：先轮换公开secret；history cleanup另行授权。
- R0加入immutable toolchain/image/deps/vendor/generator、NVS/partition/OTA/log、LICENSE/NOTICE/SBOM。
- P0加入peer boot epoch、精确MotorBoard ACK、parser budget与strict C/C++ build。
- M0加入earliest hardware stop/reset re-arm、applied-v合同与feedback source freshness。
- T0加入O0/Os/sanitizer、init失败资源守恒和snapshot有效性。
- N0加入ROS参数硬边界/immutable image/source-time；D0/V0增加reset epoch/brownout/不自动rearm证据。
- CM4推荐不变：CM7-only -> CM4 no-op+heartbeat/boot epoch -> 低风险服务；sensor/UART/MotorBoard/safety owner不迁移。
