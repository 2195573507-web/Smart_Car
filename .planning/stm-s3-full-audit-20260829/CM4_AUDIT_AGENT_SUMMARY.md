# CM4 / 双核静态审计代理摘要

> 只读证据摘要，供主线总报告合并。未修改任何固件、配置或生成源码。
> 快照日期：2026-08-30。所有结论均为当前工作树静态证据；目标板行为仍未验证。

## 结论总览

当前 H757 运行责任仍集中在 CM7。CM4 源码是 CubeMX 启动外壳：先等待 HSEM0/WFE，唤醒后只执行 `HAL_Init()` 并进入空循环。CM7 源码没有对应释放、IPC、共享 mailbox、心跳或跨核故障回退。CM4 linker 使用 D2 SRAM 的 `0x10000000` 别名，而 CM7 DMA 区使用同一物理 SRAM 的 `0x30000000` 别名；双核释放前必须完成物理区间重划分。

## 发现

### STM-DUAL-001（High，置信度 High）：CM4 等待但没有完成启动握手

- **证据：** `STM32H757/CM4/Core/Src/main.c:35-45` 定义 `DUAL_CORE_BOOT_SYNC_SEQUENCE` 与 HSEM ID；`:85-100` 开 HSEM 时钟、激活 HSEM0 通知、进入 `HAL_PWREx_EnterSTOPMode(..., PWR_D2_DOMAIN)`，唤醒后清 HSEM 标志。`:102-129` 之后只调用 `HAL_Init()`，无限循环为空。
- **对照：** `STM32H757/CM7/Core/Src/main.c:148-232` 完成 CM7 外设、通信、IMU、MotorBoard、安全任务初始化并启动 scheduler；CM7 源码搜索未发现 `HAL_HSEM_Release`/`HAL_HSEM_FastTake`、IPCC、OpenAMP/RPMsg、共享 mailbox 或 CM4 heartbeat。
- **影响：** 当前 CM7-only 路径不依赖 CM4，因此不能描述为现有车辆故障；若直接启用当前 CM4 镜像，CM4 无法承载任何服务，也不能向 CM7 证明存活。迁移任务会失去明确的失联检测和安全回退。
- **建议：** 第一个双核阶段只交付版本化 boot-ready/heartbeat/no-op；CM7 超时继续保留单核 zero-PWM/停机权。通过复位、CM4 缺失、CM4 卡死和反复复位注入后再迁移低风险服务。
- **验证：** 双镜像构建和启动地址/option bytes 核对；用共享 mailbox 或 GPIO 测量握手；拔除/停住 CM4 时确认 CM7 仍能确定进入安全态。

### STM-DUAL-002（Medium，置信度 High）：IOC 声明能力与 CM4 CMake/源码集不一致

- **证据：** `STM32H757/Smart_Car_H757.ioc:15-16` 为 CortexM4 列出 `FREERTOS_M4`、`OPENAMP_M4`、`IWDG2`（并列出 DMA/MDMA/USB 等）；`STM32H757/CM4/mx-generated.cmake:15-67` 的源集只有 `main.c`、中断/MSP、sysmem/syscalls、启动汇编和基础 HAL/双核 system 文件，没有 FreeRTOS、OpenAMP、IPCC、watchdog 或应用源。
- **证据：** `STM32H757/CM4/README.md:3-4` 仍称 CM4 CMake target 只是 build-structure check 且不产出 firmware；但当前工作树存在 `STM32H757/CM4/build/Debug/Smart_Car_H757_CM4.elf`（文件时间 2026-08-23，大小约 1.97 MB，属于既有产物，未在本轮生成）。
- **影响：** 配置、文档和实际 CMake 目标表达三种不同状态。CubeMX 再生成可能改变 IRQ、middleware 和源清单；审计者或迁移者可能误把 IOC 名称当成已集成 IPC/RTOS。ELF 的存在也说明 README 的“无固件”描述已漂移。
- **建议：** 实施前建立 IOC-to-CMake 机器可检查清单，明确唯一 IPC 方案（轻量 HSEM/mailbox 或 OpenAMP）；把 README 改为准确的构建/部署状态（本次不执行修改）。
- **验证：** 独立分支重新生成并做源清单、编译宏、IRQ 表、linker section 和 map 的差异审计；以当前 CMake 实际目标作为构建证据，不以 IOC 名称代替。

### STM-DUAL-003（Medium，双核启用前阻断，置信度 High）：没有共享内存/cache/ABI 合约

- **证据：** `STM32H757/CM7/stm32h757xx_flash_CM7.ld:37-53,214-231` 定义普通 RAM、`RAM_D2`、`.dma_buffer` 和 `.noinit`；`STM32H757/CM4/stm32h757xx_flash_CM4.ld:31-42` 只定义单一 `RAM`，完整 sections 中没有 `.shared`、mailbox 或版本化 IPC 区域。项目自研源搜索不到共享结构版本、producer/consumer 所有权、跨核 cache clean/invalidate、DMB/DSB 或 heartbeat ABI。
- **对照：** `STM32H757/CM7/Core/Src/main.c:157-163` 开启 I/D cache；`STM32H757/CM4/Core/Src/main.c:137-145` 只启用 privileged default MPU，未定义共享区属性。CM4 当前没有 RTOS/IPC 代码。
- **影响：** 未来在普通 SRAM 直接交换结构体会遇到 cache 陈旧、撕裂快照、无版本兼容和复位后残留数据；共享 HAL handle/指针还会造成所有权与生命周期错误。
- **建议：** 采用固定大小、版本化、单写者 mailbox；字段至少含 magic/version/sequence/length/timestamp/valid/CRC/commit；共享区按 32-byte cache line 对齐并配置 non-cacheable/shareable MPU，或严格定义 clean/invalidate 与 DMB/DSB；禁止跨核共享指针。
- **验证：** cache 开/关组合、随机复位、并发序号/CRC 压测、IPC 延迟和丢包测试；发布前自动检查 map 物理区间、section 属性和 ABI 版本。

### STM-DUAL-004（条件性 High，置信度 High）：D2 SRAM 别名可能覆盖 CM7 DMA

- **证据：** `STM32H757/CM7/stm32h757xx_flash_CM7.ld:44-53,214-221` 将 `RAM_D2` 放在 `0x30000000`，`.dma_buffer` 位于该区；`STM32H757/CM7/Middleware/Communication/UART_Link/uart_link.c:26-28`（当前路径实际为 `STM32H757/Middleware/Communication/UART_Link/uart_link.c:26-28`）把 UART DMA 缓冲放入 `.dma_buffer`。`STM32H757/CM4/stm32h757xx_flash_CM4.ld:31-42` 将 CM4 普通 RAM 从 `0x10000000` 开始。`STM32H757/Drivers/CMSIS/Device/ST/STM32H7xx/Include/stm32h757xx.h:2297-2298` 同时定义 `D2_AXISRAM_BASE=0x10000000` 与 `D2_AHBSRAM_BASE=0x30000000`。
- **静态产物交叉证据：** 既有 `STM32H757/CM4/build/Debug/Smart_Car_H757_CM4.map` 显示 `.data/.bss` 从 `0x10000000` 起占用约前 0x30 字节；CM7 Debug map 的 `.dma_buffer` 从 `0x30000000` 起。两者是同一 D2 SRAM 物理窗口的别名，不能按虚拟地址不重叠来放行。
- **影响：** 只有在 CM4 真正释放、执行启动代码时触发：CM4 `Reset_Handler`（`CM4/Core/Startup/startup_stm32h757xx_CM4.s:60-101`）会复制 `.data` 并清零 `.bss`，可能写坏 CM7 DMA 缓冲头部，导致 UART CRC/帧丢失和链路恢复异常。当前 CM7-only 不触发该条件。
- **建议：** 双核实施前按物理 SRAM 区间重划 CM4、CM7 DMA 和 mailbox；对别名做归一化区间检查，禁止 CM4 `.data/.bss` 与 CM7 DMA/共享区重叠。将 map overlap 检查设为发布门。
- **验证：** 用最终双镜像 map 做物理地址归一化；CM4 启动/复位期间持续校验 DMA 内容、UART 计数、CRC 和 IPC 序号，并覆盖 cache 开/关组合。

### STM-DUAL-005（Medium，置信度 High）：CM4 的异常与看门狗路径没有系统级回退

- **证据：** `STM32H757/CM4/Core/Src/stm32h7xx_it.c:69-138` 的 NMI/HardFault/MemManage/BusFault/UsageFault 均在本核无限循环；`:183-191` SysTick 仅 `HAL_IncTick()`。CM4 CMake 源集没有 FreeRTOS port、watchdog 初始化或共享 fault record；CM4 `main.c:148-160` 的 `Error_Handler()` 关中断后永久循环。IOC 虽列 `IWDG2/WWDG2`（`:15`），当前 CM4 源没有对应初始化/喂狗调用。
- **影响：** 一旦未来把服务任务放到 CM4，核卡死/异常不会自动向 CM7 报告；CM7 也没有可见心跳/故障原因，可能继续使用过期服务数据或等待无界。当前 CM7-only 运行不受 CM4 故障路径影响。
- **建议：** 双核第一阶段定义独立 CM4 watchdog（或由 CM7 监督并可复位 CM4）、周期 heartbeat/sequence 和 age timeout；CM4 fault handler 只写最小 `.noinit` 记录并触发可观测复位/停机，不在中断里做复杂日志。CM7 保持执行器最终停机权，服务数据过期即丢弃。
- **验证：** CM4 HardFault/死循环/看门狗超时/复位注入；确认 CM7 在一个消息有效期内停止消费并进入安全降级，且故障记录跨复位可读。

### STM-DUAL-006（Medium，置信度 Medium）：CM4 启动同步依赖 WFE/HSEM，但未定义时钟、电源和中断恢复契约

- **证据：** `STM32H757/CM4/Core/Src/main.c:85-100` 在 `HAL_Init()` 之前进入 D2 STOP/WFE；注释假定 CM7 已完成 system clock/external memory initialization。CM4 唤醒后没有 `SystemClock_Config`、外设初始化、`HAL_HSEM_IRQHandler` 或 HSEM IRQ handler 实现；startup 向量仅弱绑定 `HSEM1_IRQHandler/HSEM2_IRQHandler`（`CM4/Core/Startup/startup_stm32h757xx_CM4.s:273-274,701-705`），没有 HSEM0 专用 handler。
- **影响：** 当前示例依靠 WFE 事件唤醒而非完整 HSEM 中断服务；未来若改为中断/低功耗或 CM7 时钟重配置，可能出现 pending event、SysTick 时基、D2 电源状态和 HSEM 清除顺序不一致。该风险是设计缺口，不等同于已发生的唤醒失败。
- **建议：** 明确启动状态机（RESET -> WAIT_CM7_CLOCK -> HSEM_RELEASED -> CLOCK_READY -> IPC_READY -> SERVICE_READY），规定谁配置 D2 时钟、谁清事件、谁负责 HSEM IRQ、超时和复位；不要把 CubeMX 示例同步代码直接当作产品握手。
- **验证：** 冷启动、软件复位、仅 CM4 复位、D2 STOP 唤醒和调试暂停场景下测量事件/信号顺序与启动超时。

### STM-DUAL-007（Low/Medium，置信度 High）：CM4 存在未被选择机制约束的第二套 linker 内存布局

- **证据：** `STM32H757/CM4/CMakeLists.txt:11-15` 固定 `STM32_LINKER_SCRIPT=stm32h757xx_flash_CM4.ld`；同目录还保留 `stm32h757xx_sram2_CM4.ld:37-42`，其代码/向量放在 `RAM_EXEC 0x10000000/128K`、数据放在 `RAM 0x10020000/160K`。当前 CMake preset (`CM4/CMakePresets.json:1-37`) 没有 linker 选择变量或互斥检查。
- **影响：** 不同 IDE/手工命令若选择另一份脚本，会产生完全不同的向量、代码执行和数据物理区间；与 CM7 D2 DMA/mailbox 划分的 overlap 结论也会改变。现有文档没有说明该脚本是废弃、RAM 下载模式还是受支持部署模式。
- **建议：** 在双核发布清单中只允许一个明确 linker；将备用脚本标记为历史/实验并纳入自动 map/物理区间检查，禁止靠 IDE 默认值隐式切换。
- **验证：** 对所有 preset/CI 命令打印实际 `-T` 参数，并比较两套 map 的向量地址、`.data/.bss`、stack/heap 和共享区；未声明的 linker 选择应使发布检查失败。

### STM-DUAL-008（Medium，置信度 High）：顶层 ExternalProject 的 CM4 输出路径与 preset 不一致

- **证据：** `STM32H757/mx-generated.cmake:3-17` 为 CM4 设置 `BINARY_DIR ${CMAKE_SOURCE_DIR}/CM4/build`，并把 `ST_DUAL_CORE_CM4_PROJECT_BUILD_TARGET` 指向 `CM4/build/Smart_Car_H757_CM4.elf`；同文件 `:19-33` 对 CM7 明确使用 `CM7/build/Debug`。而 `STM32H757/CM4/CMakePresets.json:7-18,28-36` 的唯一标准 preset 将 binary dir 设为 `CM4/build/Debug`。
- **影响：** 顶层多核构建、CM4 preset 和现有 Debug ELF 可能指向三个不同的配置/产物位置；打包或烧录脚本若依赖顶层 cache，可能找不到 CM4、误用旧 ELF，或在 `BUILD_ALWAYS true`（`mx-generated.cmake:13`）下生成未审查目录。该结论是构建集成风险，不代表已执行顶层构建。
- **建议：** 选择一个规范的 CM4 binary/artifact 路径并在 CMake、preset、README、CI 和发布脚本中统一；将产物路径作为显式发布门，禁止依赖默认 ExternalProject 目录。
- **验证：** 仅在隔离构建目录中检查 `ExternalProject` 的 `-B`、target path 和最终 map/ELF 绝对路径；确认 CM4/CM7 双镜像来自同一源码快照和配置。

### STM-DUAL-009（Info/Medium，置信度 High）：IOC 将全部实际引脚与外设归属 CM7，CM4 没有可迁移的硬件所有权清单

- **证据：** `STM32H757/Smart_Car_H757.ioc:23-38` 声明双 context，但实际 `PA2/PA3` USART2、`PA5-PA7` SPI1、`PA8/PA9` TIM1、`PC6/PC7` USART6、`PD12/PD13` I2C4 及 GPIO 均标记 `PinAttribute=CortexM7`（例如 `:119-157,201-216,227-243`）；`CortexM4.IPs`（`:15`）仍列出 DMA/MDMA/USB/FATFS/FreeRTOS/OpenAMP 等能力。
- **影响：** 目前 CM4 不能直接接管任何已配置板级外设；若按“把传感器或通信搬到 CM4”直觉迁移，会引入 GPIO/时钟/DMA/IRQ 重分配和 HAL handle 双所有权，且没有静态表能证明冲突已消除。
- **建议：** 双核设计先建立外设、DMA stream、IRQ、GPIO、时钟域和 RAM 的唯一 owner 表；推荐服务类 CM4 不触碰 CM7 的传感/执行器外设，只消费版本化快照。
- **验证：** 从最终 IOC/CMake 生成物提取每项外设 owner；对迁移候选做 IRQ/DMA/clock conflict 检查，并通过故障注入确认 CM7 安全路径独立。

### STM-DUAL-010（Medium，条件性 High，置信度 High）：CM4 中断骨架不能直接承载 IOC 声明的 FreeRTOS

- **证据：** `STM32H757/Smart_Car_H757.ioc:15,86-96` 声明 `FREERTOS_M4` 与 CM4 的 SysTick/NVIC 配置；但 `STM32H757/CM4/Core/Src/stm32h7xx_it.c:170-192` 的 `PendSV_Handler()` 为空，`SysTick_Handler()` 只调用 `HAL_IncTick()`，没有 `xPortPendSVHandler`/`xPortSysTickHandler`。`STM32H757/CM4/Core/Inc/stm32h7xx_hal_conf.h:163-170` 还将 `USE_RTOS` 设为 0；`STM32H757/CM4/mx-generated.cmake:15-67` 未纳入任何 FreeRTOS port/kernel。
- **影响：** 当前 CM4 无 RTOS，因此不构成现行运行故障；若按 IOC 名称直接在 CM4 添加任务，tick 递增、上下文切换和 HAL/RTOS 时基不会自动成立，表现可能是任务不调度或系统停在启动路径。
- **建议：** 先明确 CM4 是否采用 FreeRTOS；若采用，单独审查 CM4 port、SysTick/PendSV、优先级边界、heap 和 HAL tick，不要只复制 CM7 的任务源。若不采用，应从双核配置/文档中删除未实现能力或明确为预留。
- **验证：** 在隔离分支加入最小 CM4 no-op RTOS 任务，测 tick、PendSV、任务切换和看门狗；确认生成文件再生不会撤销手工中断集成。

## 双核任务拆分结论

### 候选评分（静态设计评分，不是运行时证明）

| 方案 | CM7 | CM4 | 安全隔离 | 实时确定性 | IPC/迁移复杂度 | 结论 |
|---|---|---|---:|---:|---:|---|
| A 单核 | 全部现有任务 | 禁用 | 5/5 | 3/5 | 5/5 | 当前最稳，可作为基线/回退 |
| B 实时/服务分离 | 传感、姿态、标定、运动控制、MotorBoard、所有安全门 | 日志、遥测、健康统计、低频协议预处理 | 5/5 | 4/5 | 3/5 | **推荐候选**，前提是 mailbox/失联安全先验收 |
| C I/O/计算分离 | 控制闭环 | 高频传感采集或融合 | 2/5 | 2/5 | 1/5 | 不推荐，IPC 高频且耦合闭环 |
| D 独立安全核 | 常规控制/融合 | 安全监督/停机仲裁 | 4/5 | 4/5 | 1/5 | 需独立外设、时钟、复位和安全论证，当前阶段过重 |

### 推荐分工（候选 B）

- **CM7 保留：** IMU/BMI323/LSM303 采样和 SPI/I2C owner；DualAHRS/leveling/标定状态机；attitude gate；MotorBoard UART/PWM/执行器；S3 UART transport 的最终准入、BUS_OFF/超时/zero-PWM 和所有安全状态；最终故障记录与复位策略。
- **CM4 先承载：** 日志格式化、低频 telemetry 聚合、非安全协议预解析、诊断统计；CM4 不直接写 MotorBoard、PWM、姿态安全状态，也不持有 CM7 HAL handle 或 DMA owner。
- **不迁移：** 高频 IMU->AHRS->控制闭环、UART DMA 中断 owner、任何最终 stop/arming/established-link gate。

### 分阶段迁移门

1. **阶段 0：单核基线。** 固定当前 CM7-only 版本和安全行为，测 CPU/WCET/栈/heap/IPC 需求。
2. **阶段 1：CM4 no-op。** 解决物理内存重叠；加入版本化 mailbox、boot-ready、heartbeat 和 CM7 超时回退；CM4 无业务副作用。
3. **阶段 2：低风险服务。** 只迁移日志/遥测/诊断；每条消息有序号、长度、时间戳、有效期和 CRC/commit；过期即丢弃。
4. **阶段 3：压力与故障。** 量测 IPC 延迟、总线占用、cache 一致性、CM4 栈/heap；注入 CM4 卡死、复位、乱序、CRC 错误和 CM7 单核运行。
5. **阶段 4：评估扩展。** 只有在关键控制 deadline 仍有明确收益且回退路径已实测时，才考虑更多服务迁移；任何控制闭环迁移需重新安全评审。

## 证据边界

- 本摘要未运行构建、未生成或改写 build/cache、未烧录/复位、未抓取 UART 或车辆数据。
- 既有 ELF/map 只作为历史产物的静态交叉证据；不能证明当前源码与产物完全一致，也不能证明目标板已启用 CM4。
- 所有 High 项均标明触发条件；当前 CM7-only 运行的实际安全状态仍需设备证据确认。
