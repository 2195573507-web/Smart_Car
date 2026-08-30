# CM7 第 4-6 轮审计证据

> 审计对象：当前工作树中的 STM32H757 CM7 代码及其直接共享模块。  
> 审计性质：只读静态审计；本文件没有修改任何固件、配置、生成文件或构建产物。  
> 证据标签：`CONFIRMED_SOURCE` 表示源码/构建清单直接确认；`INFERRED_RISK` 表示由代码机制推断、仍需运行时触发；`UNVERIFIED_RUNTIME` 表示本轮没有板级、链路或整车证据；`RECOMMENDED` 仅表示后续方案。

## 轮次总览

| 轮次 | 范围 | 结果 | 运行时边界 |
|---:|---|---|---|
| 4 | BSP、外设、IRQ、DMA、总线和测试构建边界 | 7 项候选问题 | 未读寄存器、未做回环/突发/竞争测试 |
| 5 | 传感器、标定、姿态和 DualAHRS | 5 项候选问题 | 未验证量程读回、并发快照和标定故障注入 |
| 6 | 应用、安全、恢复和层间独立性 | 3 项风险 + 1 项复核结论 | 未做传感器拔除、链路拥塞或车辆测试 |

本轮所有建议均为后续实施输入，不是本任务授权的代码修改。

## 第 4 轮：BSP、外设、中断和 DMA

### STM-BSP-001 - PA10 的 USART1 RX 与 LSM303 INT1 所有权冲突

- **等级/置信度：** High / High（机制为静态推断）。
- **证据：** `STM32H757/CM7/Core/Src/main.c:185-200` 先初始化 GPIO、USART1 和其他外设；`STM32H757/CM7/Core/Src/stm32h7xx_hal_msp.c:298-329` 为 PA9/PA10 设置 `GPIO_MODE_AF_PP`、`GPIO_AF7_USART1`。`STM32H757/BSP/GPIO/bsp_gpio.c:11-18` 又把 `BSP_GPIO_LF_INT1` 映射到 PA10，`bsp_gpio_init()` 在 `:35-52` 将 PA10 配成普通输入。正常 BMI 初始化由 `STM32H757/Middleware/Sensor/BMI323/bmi323.c:513-521` 调用 `bmi323_port_init()`，后者在 `STM32H757/Middleware/Sensor/BMI323/bmi323_port.c:103-122` 调用 `bsp_gpio_init()`；该路径发生在 `main.c` 的 USART1 初始化之后。
- **事实与触发机制：** 正常 IMU 启动若进入 `bmi323_port_init()`，同一 GPIO 配置函数会重写 USART1 RX 的 AF 配置。源码没有按运行模式拒绝重复所有权，也没有在后续初始化后做 PA10 的 AF 读回。
- **潜在影响：** CH340/USART1 RX 可能在 IMU 启动后停止作为串口输入；调试命令、回环和依赖该 RX 的诊断会出现静默失效。当前源码不能据此断言车辆控制已经受影响。
- **现有保护：** `main.c` 有“USART1 owns PA9/PA10”的注释；GPIO API 有枚举校验，但没有跨模块所有权校验。故障状态仍可由 `bsp_uart` 返回错误或目标板寄存器读回发现。
- **建议（未实施）：** 建立单一引脚所有权表，将 LF_INT1 与 USART1 RX 设为互斥配置；在不同镜像/硬件变体中显式选择其一，并在启动诊断中记录最终 MODER/AFR。
- **验证方法：** 目标板上分别在 `MX_USART1_UART_Init()` 后、BMI 初始化后读回 GPIOA MODER/AFR；执行 CH340 回环和 IMU 启动序列，记录 RX 字节、错误计数和 PA10 电平。此项当前为 `UNVERIFIED_RUNTIME`。

### STM-BSP-002 - I2C BSP 使用阻塞 HAL，缺少总线所有权和恢复层

- **等级/置信度：** Medium / High（`CONFIRMED_SOURCE`）。
- **证据：** `STM32H757/BSP/I2C/bsp_i2c.c:25-99` 的写、读、写读和 probe 直接调用 `HAL_I2C_Master_Transmit/Receive` 与 `HAL_I2C_IsDeviceReady`；未见 mutex、事务序列号、SDA/SCL 恢复或错误后重新初始化。`STM32H757/Drivers/IMU/LSM303/lsm303.c:77-109,161-184,227` 通过同一 BSP 访问多个地址。
- **事实与触发机制：** 任何调用者都可在同一 `hi2c4` 上发起阻塞事务；器件掉线、SDA 被拉低或 HAL 状态非 READY 时，调用会一直占用调用任务直到传入 timeout。当前 BSP 只把 HAL 状态映射成项目状态码。
- **潜在影响：** LSM303 更新或初始化任务的执行时间取决于总线故障和调用次数，可能造成周期超限；未来增加第二个调用者时还会出现交叉事务或错误恢复竞态。
- **现有保护：** 每个 API 有地址、指针、长度和 `i2c_ready` 检查，并传播 `HAL_TIMEOUT`；这不是总线恢复或互斥。
- **建议（未实施）：** 明确 I2C 唯一 owner，外层串行化事务；为 stuck-bus、HAL ERROR 和重新初始化定义有限状态机，并把失败原因和恢复次数纳入健康监测。
- **验证方法：** 在目标板逐步断开 LSM303、保持 SDA 低、注入 HAL 超时并测量任务 WCET、恢复时间和后续成功率；静态上核对所有 `bsp_i2c_*` 调用者。

### STM-BSP-003 - SPI 事务和 BMI323 raw probe 没有 BSP 层锁

- **等级/置信度：** Medium / High（共享状态风险为 `INFERRED_RISK`）。
- **证据：** `STM32H757/BSP/SPI/bsp_spi.c:17-22,104-197` 使用单一静态 `hspi1_bsp`，普通 transmit/receive/write-read 没有 mutex。`bsp_spi_bmi323_raw_transaction()` 在 `:254-323` 直接清除 `SPI_CR1_SPE`、修改 `SPI1->CFG1.MBR`，执行一次或两次 HAL 事务，再无条件清除 SPE 并恢复 MBR。正常 BMI 驱动和诊断路径均通过该 BSP。
- **事实与触发机制：** 只检查 `HAL_SPI_GetState()==READY` 不能防止两个任务在检查后交错；raw probe 与普通事务之间没有所有权协议。若诊断在正常读取期间运行，CS、MBR、SPE 和 HAL 状态可能被另一调用者改变。
- **潜在影响：** 传输帧截断、错误速率、CS 时序错误或后续事务卡在 disabled 状态；姿态数据和初始化结果可能出现非确定性。是否存在同时调用者需要运行时确认。
- **现有保护：** raw 路径显式拉高/拉低 CS、保存并恢复 MBR，并检查 HAL 状态；这不能替代互斥和事务级 owner。
- **建议（未实施）：** 由 BMI323 驱动持有 SPI 事务锁，普通和 raw 操作共享同一锁；把诊断采样安排在安全窗口，并对 CR1/CFG1/CS 做 postcondition 检查。
- **验证方法：** 在主机替身或目标板并发触发普通读、raw probe 和恢复；采集 CS、SCK、SPE、MBR、HAL error 及 WHO_AM_I 连续成功率，确认前后寄存器与事务序列一致。

### STM-BSP-004 - 退役 TIM3 仍暴露 PWM API

- **等级/置信度：** Medium / High（当前运行影响未确认）。
- **证据：** `STM32H757/BSP/PWM/bsp_pwm.c:5-68` 仍引用 `htim3` 并导出 init/start/stop/set-duty；仅允许枚举 3/4。`STM32H757/CM7/Core/Src/stm32h7xx_hal_msp.c:201-220` 保留 TIM3 时钟初始化，但 `HAL_TIM_MspPostInit()` 明确注释 TIM3 local PWM 已退役；`STM32H757/CM7/Core/Src/main.c:185-199` 没有 `MX_TIM3_Init()` 调用，全文搜索也未找到该调用。
- **事实与触发机制：** 当前主路径没有建立 `htim3` 的 PWM 状态，调用遗留 BSP 通常只会得到 `NOT_READY`；未来测试或新应用若误用该 API，则可能与 USART6/电机板的引脚契约冲突。
- **潜在影响：** 接口边界与实际外设所有权漂移，增加误启用本地 PWM、错误诊断和维护成本；不把当前未发现的调用描述成运行故障。
- **现有保护：** `pwm_validate()` 拒绝 CH1/CH2，注释声明 PC6/PC7 由 USART6 所有；`bsp_pwm_init()` 检查 `htim3.Instance` 和 HAL 状态。
- **建议（未实施）：** 将退役 API 标为构建期兼容层或移入明确的诊断目标；在生产配置中用编译期断言/依赖检查阻止误用，保持现有电机板协议不变。
- **验证方法：** 对每个 `bsp_pwm_*` 调用者做静态清单；在 Debug/生产配置检查未解析的 TIM3 初始化和 map 中是否出现 PWM 输出；只在获批后做硬件引脚示波验证。

### STM-BSP-005 - BSP_TEST 同时进入生产目标和未使用 OBJECT 目标

- **等级/置信度：** Medium / High（`CONFIRMED_SOURCE`）。
- **证据：** `STM32H757/CM7/mx-generated.cmake:95-113` 将 `BSP_TEST/bsp_test.c` 直接加入 `${CMAKE_PROJECT_NAME}`，随后又建立 `BSP_TEST` OBJECT library。`STM32H757/CM7/BSP_TEST/bsp_test.c:11-24` 的 compile smoke 函数会触及 GPIO、SPI、I2C、PWM、USART6 和 ADC API；`:37-67` 还提供 I2C scan、PWM 和 UART 测试入口。当前 CMake 没有把 `BSP_TEST` OBJECT library 链接到主目标。
- **事实与触发机制：** 同一源至少被两个 target 编译；直接加入主固件意味着测试符号和其依赖进入生产编译边界，OBJECT target 又会产生额外编译和潜在编译定义漂移。静态搜索未发现这些入口的应用调用，因此不能断言它们当前被执行。
- **潜在影响：** Flash/编译时间增加，测试代码可能因链接保留、LTO 或未来调用而改变资源预算；测试入口与正式外设所有权边界不清。
- **现有保护：** 函数没有自动注册或构造器，当前未见调用；主目标仍由显式源列表控制。
- **建议（未实施）：** 把 smoke 源只放入独立 test/compile target；若必须保留生产编译，至少在报告和 map 中明确其不应被调用，并对未使用符号做链接检查。
- **验证方法：** 只读检查生成的 compile commands/map（不重建本仓库）；比较主目标和 OBJECT target 的预处理宏、包含路径和最终符号，确认是否存在重复编译或意外保留。

### STM-BSP-006 - UART DMA RX 回调在 ISR 内做 cache、复制和重启

- **等级/置信度：** Medium/High（高负载影响为 `INFERRED_RISK`）。
- **证据：** `STM32H757/Middleware/Communication/UART_Link/uart_link.c:80-97` 的 `dcache_invalidate()` 可覆盖整段对齐范围；`:42-60,63-77` 在 ISR 临界区逐字节写入 2048 字节 ring 并更新计数；`:308-320` 的 `HAL_UARTEx_RxEventCallback()` 在中断上下文依次 invalidate、复制最多 512 字节、重启 DMA。DMA 缓冲在 `:26-28` 位于 `.dma_buffer` 且 32 字节对齐。
- **事实与触发机制：** 每次 IDLE/full 事件都执行上述完整路径，工作量随 `size` 和 ring 溢出而变；回调还直接调用 `HAL_UARTEx_ReceiveToIdle_DMA()`。IRQ 优先级为 5（`:13-17,117-120`），与 `Config/FreeRTOSConfig.h:58-63` 的 FromISR 边界一致，未发现可据此判定的优先级违规。
- **潜在影响：** 中断占用和临界区延长，可能推迟 SysTick、USART2 后续字节或其他 DMA/控制事件；高突发时 ring 溢出和丢帧概率增加。源码不能给出实际 WCET。
- **现有保护：** DMA 缓冲对齐、禁用半传输中断、计数器记录溢出/丢字节，且 worker 任务有恢复标志。
- **建议（未实施）：** ISR 只记录 buffer descriptor/事件，复制和 cache 维护移至受控任务；为每次事件设置上界，保留丢帧计数和背压策略，不改变 UART2 契约。
- **验证方法：** 目标板用最大长度和连续突发帧测量 ISR 脉宽、SysTick/USART 延迟、ring 水位、丢字节和恢复次数；比较空闲、日志高峰和链路错误三种工况。

### STM-BSP-007 - UART 同步发送可阻塞调用者约 20 ms

- **等级/置信度：** Medium / High（`CONFIRMED_SOURCE`，控制影响为 `INFERRED_RISK`）。
- **证据：** `STM32H757/Middleware/Communication/UART_Link/uart_link.h:13-17` 将 `UART_LINK_TX_TIMEOUT_MS` 设为 20 ms；`uart_link.c:213-243` 先等待 TX mutex，再调用阻塞 `HAL_UART_Transmit`。`STM32H757/Middleware/Communication/Services/s3_service.c:56-60,267-277` 通过链路锁调用该发送路径，链路锁本身也等待 20 ms。
- **事实与触发机制：** 日志、启动消息、遥测和协议响应共享同一发送路径；发送器忙、S3 未响应或 HAL 卡住时，调用任务可能在锁和 HAL 中累计接近超时。
- **潜在影响：** 非通信任务被拖长，造成控制/IMU 周期抖动或同优先级任务排队；具体是否超过控制 deadline 仍需量测。
- **现有保护：** mutex 和 HAL 均有超时，失败会增加计数而不会无限等待；日志队列对生产者采用非阻塞入队。
- **建议（未实施）：** 将实时响应与日志/遥测分离为有界发送队列；对调用者定义“可丢弃/必须送达”策略和独立 deadline，保留现有超时和 zero-PWM 门。
- **验证方法：** 注入发送端忙、锁持有和 UART 断线，测量各任务阻塞时间、控制周期、`tx_timeout_count` 和丢弃率。

## 第 5 轮：传感器、标定、姿态和 DualAHRS

### STM-AHRS-001 - DualAHRS 全局上下文的更新、重置和读取未显式同步

- **等级/置信度：** High / High（并发后果为 `INFERRED_RISK`）。
- **证据：** `STM32H757/Middleware/Attitude/DualAHRS/dual_ahrs.c:17-67` 把全部滤波器、时间戳、偏置、状态和输出放在静态 `s_dual`；`dual_ahrs_update()` 在 `:775-932` 写入，`dual_ahrs_get_output()`/`dual_ahrs_pack_payload()` 在 `:934-989` 读取，`dual_ahrs_set_bias()`/`set_leveling()` 在 `:732-773` 可重置或替换状态。未见 mutex、临界区或双缓冲。
- **事实与触发机制：** BMI producer 在 `STM32H757/Middleware/Sensor/imu_manager.c:843-877,1032-1122` 更新；`STM32H757/Application/RTOS/imu_runtime.c:121-125,182-200` 的遥测/日志任务读取；`imu_manager.c:771-827` 还可能在启动/恢复期间调用 setter。多个任务可在字段复制或状态重置中交错。
- **潜在影响：** 发送半新半旧的四元数/时间戳，或在 bias/leveling 重置时读取到不一致状态，导致姿态跳变、诊断误判和跨模块时间序列破坏。该条不等同于已观测的姿态失控。
- **现有保护：** BMI/LSM 各自的数据锁和启动生命周期锁存在；`dual_ahrs` 输入有有限值/陈旧样本判断，但这些保护不覆盖整个输出快照。
- **建议（未实施）：** 采用单 writer + 双缓冲版本号，或为 update/setter/getter 定义统一锁；输出带 sequence/version，消费者只接受完整快照。
- **验证方法：** 在主机替身和目标板同时施加 200 Hz 更新、50/100 ms 遥测读取及恢复/重新标定，检查 sequence 单调性、四元数范数、状态切换和异常快照计数。

### STM-IMU-001 - BMI323 活动实现与 dormant driver/文档的陀螺量程契约漂移

- **等级/置信度：** High / Medium（`INFERRED_RISK`，当前活动硬件比例为 `UNVERIFIED_RUNTIME`）。
- **证据：** 当前 CMake 在 `STM32H757/CM7/CMakeLists.txt:33-45` 编译 `Middleware/Sensor/BMI323/bmi323.c`，其 `:20-21,401-432,795-799` 使用 500 dps 换算并以 `0x4028` 作为 GYR_CONF 基值。并存但未列入该 target 的 `STM32H757/Drivers/IMU/BMI323/bmi323.c:33-34,392-398` 使用 2000 dps；`STM32H757/Docs/DRIVER/IMU_DRIVER.md:21-24` 也仍描述 +/-2000 dps。
- **事实与触发机制：** 当前活动代码会读回并校验 GYR_CONF，说明其内部契约自洽；但 include/source 树保留两套 API 和不同缩放常量。若切换配置、复用 dormant header/driver，或硬件寄存器仍为 2000 dps，软件输出会产生约 4 倍比例偏差。
- **潜在影响：** 角速度、积分姿态、标定 RMS 和控制补偿均可能按错误比例运行；这是契约漂移风险，不是本轮已确认的比例错误。
- **现有保护：** 活动实现保存 `ctrl_gyr/gyr_conf` 并在初始化后读回；CMake 当前明确只编译 middleware 实现。
- **建议（未实施）：** 选择一个权威驱动/量程契约，统一文档、寄存器构造、换算和诊断字段；在发布门强制比对实际 GYR_CONF 与换算常量。
- **验证方法：** 通过目标板寄存器读回确认 GYR_CONF，施加已知角速率并比较 raw、rad/s 和日志；做活动/诊断配置的 include 解析和独立链接检查。

### STM-IMU-002 - 两套 BMI323 实现同时可见，头文件/API 边界易漂移

- **等级/置信度：** Medium / High（`CONFIRMED_SOURCE`）。
- **证据：** `STM32H757/Drivers/IMU/BMI323/{bmi323.c,bmi323.h}` 与 `STM32H757/Middleware/Sensor/BMI323/{bmi323.c,bmi323.h}` 同名实现并存。`STM32H757/CM7/CMakeLists.txt:33-36` 只编译后者，但 `:67-80` 同时暴露 `Drivers/IMU/BMI323` 和 `Middleware/Sensor/BMI323` 相关 include 根；两套 header 的返回类型、API 集合和宏守卫不同。
- **事实与触发机制：** 当前 `imu_manager.c` 使用带目录的 `BMI323/bmi323.h`，但其他新模块若使用裸 `bmi323.h` 或把 dormant `.c` 加入目标，可能解析到另一套契约；两套实现同时链接还会产生同名符号冲突。
- **潜在影响：** 编译在不同 target/顺序下得到不同 ABI 或单位，维护者难以判断实际驱动；配置切换可能在链接期失败或更隐蔽地改变数据解释。
- **现有保护：** CMake 当前的 source list 只有 middleware 实现；活动 source 自带 `bmi323_reg.h` 和诊断 API。
- **建议（未实施）：** 将活动驱动移到唯一公共 include 根，给 dormant/实验驱动使用显式命名空间或独立 target；为每个配置输出 include trace 和符号清单。
- **验证方法：** 使用现有 compile commands 做 `-H`/预处理头解析（不改文件），比较每个 BMI323 调用者实际包含的 header；对两套实现分别做隔离链接检查。

### STM-ATT-001 - 启动 fast-zero 绕过 500 样本累计零点流程

- **等级/置信度：** Medium / High（`CONFIRMED_SOURCE`，运动影响为 `INFERRED_RISK`）。
- **证据：** `STM32H757/Middleware/Attitude/attitude.c:93-123` 的 `attitude_zero_capture_current()` 用单个当前样本直接写入 roll/pitch/yaw offset 并置 ready；同文件 `:142-163` 定义了 `ATTITUDE_ZERO_SAMPLE_COUNT` 的累计和圆周 yaw 平均。`STM32H757/Middleware/Sensor/imu_manager.c:695-701` 在每次 filter ready 后先调用 `attitude_update()`，随后在 latch 未置位时直接调用 fast-zero。
- **事实与触发机制：** 首个有效 LSM 样本可能先进入累计流程，随后同一更新路径立即执行单样本 fast-zero；因此正常启动不会稳定等待完整 500 样本窗口。
- **潜在影响：** 启动时若平台有振动、磁场瞬态或单帧异常，零点偏置会被锁定；与代码中多样本平均的设计意图和标定说明不一致，可能造成初始姿态偏差。
- **现有保护：** fast-zero 检查校准完成、online、有限值；姿态门还要求 `attitude_zero_is_ready()` 和状态 READY。
- **建议（未实施）：** 明确 fast-zero 是有意的诊断旁路还是生产路径；若生产需要平均，应让单一路径负责 ready，并记录样本数/运动状态。
- **验证方法：** 目标板启动时人为施加已知静态角度和短时振动，记录 zero sample count、offset、ready 时间及重复上电方差；对 debug-only 和正常镜像分别验证。

### STM-CAL-001 - BMI 高速标定在锁竞争时丢样本但原因不显式

- **等级/置信度：** Medium / High（`CONFIRMED_SOURCE`）。
- **证据：** `STM32H757/Middleware/Calibration/imu_calibration.c:514-525` 的 `imu_calibration_update_bmi323()` 使用 `try_lock_calibration()`；锁不可得时立即返回。BMI 200 Hz producer 在 `STM32H757/Middleware/Sensor/imu_manager.c:1049-1065` 调用该函数。质量统计在 `imu_calibration.c:190-207,263-269` 只依据累计 `sample_count` 判定 `quality_ok`。
- **事实与触发机制：** 100 Hz 管理路径或其他读者短暂持锁时，高速样本被有意跳过；样本计数会少于目标，但诊断只显示数量/质量，不区分“传感器无效”“时间窗外”和“锁竞争丢弃”。
- **潜在影响：** 标定可能因数量不足失败，或工程人员将锁竞争误判为硬件噪声；在接近阈值时，标定时长和成功率随任务调度变化。
- **现有保护：** 设计注释明确不让 ODR 任务等待；实际计数和质量阈值会阻止不完整结果进入完成态。
- **建议（未实施）：** 增加 contention-drop 计数、时间窗拒绝计数和原因字段；在质量报告中区分采集源、锁竞争和运动检测。
- **验证方法：** 注入可控锁持有时间，比较 configured/actual/minimum count、quality flag、标定结果和日志原因；测量 producer 是否保持 200 Hz deadline。

## 第 6 轮：应用、安全、恢复与模块独立性

### STM-SAFE-001 - 姿态 ready 置位后没有持续 IMU 新鲜度撤销门

- **等级/置信度：** High / High（`INFERRED_RISK`）。
- **证据：** `STM32H757/Application/Safety/attitude_startup_coordinator.c:57-91` 在启动阶段检查生命周期和 LSM/BMI `update_count`；但任务在 `:105-112` 发现 `g_attitude_is_ready != 0` 后只 `vTaskDelayUntil()` 并跳过后续检查。ready 在 `:132-146` 置位并启动 `motor_board`。电机任务随后持续处理 `STM32H757/Middleware/MotorBoard/motor_board_task.c:792-818`。
- **事实与触发机制：** 运行中若 IMU 停止更新、时间戳冻结、姿态状态降级或 calibration 状态被重置，源码没有从该 coordinator 路径持续清除 ready 并强制停止；电机任务仍可能处理已有命令。
- **潜在影响：** 传感器失效到 zero-PWM 的响应依赖其他间接路径（例如 S3 超时/BUS_OFF），不具备明确的本地姿态新鲜度 deadline；在链路仍在线时尤其需要关注。
- **现有保护：** 启动前有稳定周期、zero-PWM、BUS_OFF/链路超时和 `motor_board_force_stop()`；电机任务有 forced-stop 状态和协议超时。它们不能替代运行中 IMU freshness gate。
- **建议（未实施）：** 将姿态有效、样本年龄和 update sequence 纳入电机输出的每周期准入；任一超限立即锁定 zero-PWM，恢复必须重新经过启动稳定门。
- **验证方法：** 运行中拔除/停读 BMI 或 LSM、冻结 update counter/timestamp、模拟 AHRS 降级，测量故障检测至 PWM 归零的最大时间，并确认恢复需重新稳定周期。当前为 `UNVERIFIED_RUNTIME`。

### STM-ARCH-001 - BSP 反向依赖 S3 服务和通信日志

- **等级/置信度：** Medium / High（`CONFIRMED_SOURCE`）。
- **证据：** `STM32H757/BSP/UART/bsp_uart.c:6-10,186-238` 直接包含 `s3_service.h` 并调用 `s3_service_send_log()`；`STM32H757/BSP/SPI/bsp_spi.c:3-10,24-42,213-234` 从 SPI BSP 直接调用 UART/S3 日志函数。
- **事实与触发机制：** 低层 BSP 的错误/诊断路径依赖上层服务初始化、链路 mutex 和 UART 发送；SPI/BSP 初始化期间若服务尚未 ready，日志调用会失败或改变时序。依赖方向不再是单向 `HAL -> BSP -> driver -> service -> app`。
- **潜在影响：** 初始化顺序、错误处理和测试替身被跨层耦合；未来把 BSP 复用于 CM4、裸机诊断或双核迁移时，需要携带 S3 service 依赖，增加循环依赖和阻塞风险。
- **现有保护：** 日志发送有空指针/状态检查和超时，失败通常被忽略；不影响当前已确认的链接契约，但会降低独立可测性。
- **建议（未实施）：** BSP 只上报轻量诊断事件/计数，由 service 或 app 统一消费；保留现有日志格式和协议 ID，在适配层完成映射。
- **验证方法：** 生成静态 include/调用依赖图；用最小 HAL/BSP 替身链接 SPI/UART，不链接 S3 service，确认接口可独立测试；在目标板测量初始化阶段日志失败与阻塞。

### STM-CTRL-001 - MotorBoard 控制任务同步发送轮速/电源遥测

- **等级/置信度：** Medium/High（阻塞机制 `CONFIRMED_SOURCE`，deadline 影响 `INFERRED_RISK`）。
- **证据：** `STM32H757/Middleware/MotorBoard/motor_board_task.c:565-588` 在控制任务中直接调用 `s3_service_send_message()`；调用点位于主循环 `:792-817`，轮速状态周期 50 ms、电源状态周期 500 ms（`:23-29`）。S3 服务 `STM32H757/Middleware/Communication/Services/s3_service.c:267-277` 等待 20 ms 链路锁，底层 `uart_link.c:213-243` 还可能等待/发送 20 ms。
- **事实与触发机制：** 轮速或电源状态到期时，MotorBoard 任务同步完成编码、链路锁和 HAL 发送，然后才回到 1 ms 轮询；S3 堵塞或 UART 错误会把遥测发送时延带入控制任务。
- **潜在影响：** 控制循环周期漂移、MSPD 响应延迟和 PID 更新抖动；当前没有 WCET/锁等待上限的运行时证据，不能直接宣称已超出控制要求。
- **现有保护：** 发送失败返回值被丢弃但不会无限等待；PID/forced-stop 使用临界区和 zero-PWM 保护，链路/BUS_OFF 也有停止路径。
- **建议（未实施）：** 把非安全遥测放入独立有界队列或低优先级发送任务；控制任务只写入快照并记录 drop/deadline，不改变安全命令和轮序契约。
- **验证方法：** 让 S3 链路锁持续占用、UART 断线和高日志负载，测量 MotorBoard 1 ms 循环 WCET、PID 周期、MSPD 响应、telemetry drop 和 zero-PWM 时限。

### STM-RECOVERY-001 - 旧结论“imu_recover 未重置 Boot Manager”在当前源码中不成立

- **结论类型：** 静态复核关闭旧候选；仍有 `UNVERIFIED_RUNTIME` 边界。
- **证据：** 当前 `STM32H757/Middleware/Sensor/imu_manager.c:1367-1392` 的 `imu_prepare_lifecycle()` 在非 debug 路径调用 `imu_boot_manager_init()`；`imu_recover()` 在 `:1429-1443` 先调用该生命周期准备函数，再记录 dual lifecycle reset。
- **判断：** 早期账本中关于恢复后 Boot Manager 状态未同步重置的描述不能直接继承到当前工作树；源码层面已看到重置调用，因此不把它列为当前 High/Medium 缺陷。
- **仍需验证：** 设备复位、任务并发和 S3 断链期间是否存在旧任务/旧快照继续运行，仍需故障注入和日志/状态序列验证。该复核不代表已完成硬件恢复验收。

## 本轮交叉结论

1. CM7 已有 zero-PWM、BUS_OFF、链路超时、DMA 对齐和启动姿态门等保护，但这些保护不能替代外设唯一所有权、连续 IMU freshness 和运行时 WCET 证据。
2. 高风险项均保留事实/推断边界：PA10、DualAHRS 并发、BMI 量程和 ready 撤销门需要目标板或故障注入才能升级为运行结论。
3. 第 7-12 轮应把上述接口作为跨核迁移的冻结边界：SPI/I2C/UART owner、姿态快照版本、标定质量原因、MotorBoard 安全权威和 S3 遥测背压不能在迁移中隐式改变。
