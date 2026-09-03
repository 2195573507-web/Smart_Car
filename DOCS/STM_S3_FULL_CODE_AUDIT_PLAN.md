# STM32H757 与 ESP32-S3 全量只读代码审计计划

> 历史快照说明：本文记录 2026-08-29 SRP v4 硬切换前的只读审计基线。
> 当前 UART2 活动协议已迁移到 `Common/SRP/`；本文中的旧协议名称和路径
> 仅用于解释当时的审计证据，不得作为现行实现或构建依据。

> 2026-08-31 已针对当前 SRPv4 工作树启动第 13-17 轮增量审计，
> 方法、范围和完成条件见第 11 节。冲突时以该增量计划和当前源码为准。

> 用户随后要求再增加 10 轮；第 18-27 轮深化计划见第 12 节。
> 用户再次增加 10 轮；第 28-37 轮深化计划见第 13 节。
> 用户继续增加 10 轮；第 38-47 轮深化计划见第 14 节。
> 它不替换旧快照，但对当前脏工作树的结论优先级最高。

> 日期：2026-08-29  
> 审计对象：当前工作树 `STM32H757/`、`ESPS3/` 及其实际共享协议依赖  
> 审计性质：只读静态审计与目标架构规划  
> 硬边界：不修改、格式化、生成、构建、烧录或提交任何固件代码

## 1. 目标

本审计回答五个工程问题：

1. 当前 STM32H757 与 ESP32-S3 实际编译、启动和运行哪些项目代码。
2. 任务调度、时间基准、中断、DMA、队列、锁、缓存和内存所有权是否清楚且可控。
3. BSP、驱动、中间件、应用、安全和通信层是否保持合理独立，是否存在反向依赖或隐式共享状态。
4. CPU、RAM、Flash、栈、总线带宽、日志和数据复制是否存在明确低效或只能通过实测确认的风险。
5. 在保持现有硬件、协议和安全契约的前提下，STM32H757 的 CM7/CM4 任务应如何拆分，迁移顺序怎样风险最低。

最终输出为 [总审计报告](STM_S3_FULL_CODE_AUDIT_REPORT.md)。报告必须把事实、已有设计、推测、建议和未验证项分开。

## 2. 禁止项与保护契约

### 2.1 禁止项

- 不修改 `.c`、`.h`、汇编、链接脚本、CMake、Kconfig、IOC、分区表或 SDK 配置。
- 不执行会更新构建目录、缓存、生成源码或固件产物的命令。
- 不烧录、不复位设备、不连接串口/BLE/Wi-Fi、不运行车辆。
- 不提交 Git，不覆盖用户现有未提交改动。
- 不把历史文档、旧构建结果或日志升级为当前源码事实。

### 2.2 冻结契约

本审计不授权调整以下内容；建议涉及这些边界时必须单独列为后续决策门：

- STM32 继续持有最终运动输出和本地安全门控。
- S3 继续持有 BLE/Wi-Fi 网关与雷达采集/上行职责。
- STM-S3、App-S3 各自现有消息 ID、包格式、确认语义和连接状态机。
- STM USART2 与 S3 UART2、S3 雷达 UART1/GPIO44、雷达 PWM GPIO4 等当前外设所有权。
- 电机顺序、PWM 通道、轮侧、底盘几何、控制方向和停机门控。
- LSM303/BMI323、标定、姿态就绪和传感器失效边界。
- BUS_OFF、失联、姿态无效、未同步、紧急停止和看门狗路径。

## 3. 审计口径

### 3.1 全量的定义

| 类别 | 覆盖方式 | 结论边界 |
|---|---|---|
| 项目自研且参与当前构建的代码 | 按构建源清单逐文件、逐入口、逐共享状态审计 | 可形成代码级发现 |
| 项目自研但当前未参与构建的代码 | 检查用途、漂移、误启用风险与测试状态 | 不宣称其运行 |
| CubeMX/ESP-IDF 生成代码 | 检查项目配置结果、IRQ/DMA/时钟/MPU/缓存/句柄集成 | 不重审生成器本身 |
| CMSIS/HAL/FreeRTOS/ESP-IDF 等第三方代码 | 检查版本、编译选择、配置、补丁和调用假设 | 不宣称逐行验证供应商实现 |
| 静态库/二进制 | 检查链接参与、接口和可审计性 | 无源码处标记黑盒 |
| 历史文档与旧构建目录 | 仅用于定位漂移和过去证据 | 不作为当前实现真相 |

### 3.2 主范围

- STM32H757：`CM7/`、`CM4/`、`BSP/`、`Drivers/` 中项目设备驱动、`Middleware/`、`Application/`、`System/`、`Config/`、启动/链接/生成集成。
- ESP32-S3：根 CMake/配置、`main/`、项目 `components/`、雷达链和当前测试代码。
- 共享通信：实际被 STM 与 S3 引用的 SRP/帧编解码、注册表、CRC、ACK/重试和状态机实现。
- 接口上下文：仅为核对契约读取 App、ROS2 或工具消费者；不扩展为这些模块的全量审计。

### 3.3 初始规模基线

初始命令 `git ls-files STM32H757 ESPS3 Common/SRP` 返回 1695 个受跟踪路径：`STM32H757` 1626 个，`ESPS3` 69 个；其中 `Common/SRP` 当前不存在。复核 `git ls-files --stage Common` 确认实际构建使用的 `Common/SCBP_CAN` 和 `Common/SmartCarLog` 共 12 个文件均受 Git 跟踪，因此主范围合计为 1707 个受跟踪路径。STM 树包含大量 CMSIS/HAL、静态库和生成材料，最终覆盖率按“当前构建自研源集”计算，不按目录中文件总数计算。

## 4. 证据模型

### 4.1 事实标签

| 标签 | 含义 |
|---|---|
| `CONFIRMED_SOURCE` | 当前源码、构建清单或配置直接确认 |
| `CONFIRMED_STATIC` | 静态脚本/搜索交叉检查确认，但未运行目标 |
| `EXISTING_DESIGN` | 当前文档或接口明确表达的设计意图 |
| `INFERRED_RISK` | 基于代码机制推断，需要运行时触发验证 |
| `RECOMMENDED` | 本审计提出的后续方案，不是当前实现 |
| `UNVERIFIED_RUNTIME` | 必须通过目标板、链路或车辆测试确认 |

### 4.2 发现字段

每个问题至少记录：

| 字段 | 内容 |
|---|---|
| ID | 稳定编号，例如 `STM-SCHED-001` |
| 严重度 | Critical / High / Medium / Low / Info |
| 置信度 | High / Medium / Low |
| 证据 | 文件、行号、符号、调用者和配置入口 |
| 触发条件 | 哪种任务/中断/状态/输入使问题出现 |
| 机制 | 为什么会出现 |
| 影响 | 实时、安全、资源、数据、恢复或维护影响 |
| 现有保护 | 已有门控、超时、检查或降级路径 |
| 建议 | 最小兼容方案；本任务不实施 |
| 验证 | 静态、构建、目标板、链路和车辆分层步骤 |

### 4.3 严重度

| 等级 | 判定标准 |
|---|---|
| Critical | 可现实触发不受控运动、系统性内存破坏、绕过最终安全权，或确认活动秘密已进入受跟踪仓库/历史并需立即轮换 |
| High | 可能导致死锁、持续错误输出、关键任务饿死、失联不停车或关键恢复失败 |
| Medium | 明显降低实时性、可靠性、效率、扩展安全性或长期可维护性 |
| Low | 局部一致性、诊断、测试或文档质量问题 |
| Info | 已有良好设计、度量缺口或非紧急改进机会 |

## 5. 12 轮审计与文档修订

用户要求至少 10 轮。本计划执行 12 轮，每轮都包含源码读取、证据交叉核验、问题去重/定级和总报告修订。

| 轮次 | 审计主题 | 主要检查 | 本轮报告动作 |
|---:|---|---|---|
| 1 | 范围与基线 | 工作树、入口、构建源、第三方/生成边界 | 建立范围、口径和排除项 |
| 2 | 架构与启动 | CM7/CM4/S3 启动、链接、内存域、MPU/cache、核唤醒 | 写当前部署图和启动风险 |
| 3 | STM 调度 | 任务创建、优先级、周期、deadline、锁、通知和时间源 | 写任务表和调度问题 |
| 4 | STM 底层 | BSP、HAL 句柄、IRQ、DMA、UART/I2C/SPI/TIM、错误回调 | 写外设/中断所有权矩阵 |
| 5 | STM 中间件 | 传感器、标定、滤波、姿态、里程计、PID、数据有效性 | 写数据流和闭环审计 |
| 6 | STM 应用与安全 | 命令准入、运动状态、停止、BUS_OFF、失联、看门狗 | 写安全链与应用独立性 |
| 7 | S3 内部 | FreeRTOS、核心亲和、BLE、UART、Wi-Fi、雷达、内存/队列 | 写 S3 任务与资源审计 |
| 8 | 跨芯片协议 | 帧、CRC、ID、ACK、重试、去重、超时、背压、启动同步 | 写端到端状态机和契约风险 |
| 9 | 模块独立性 | 依赖方向、全局状态、内部头暴露、跨层调用、测试替身 | 写耦合与可维护性评估 |
| 10 | 效率 | CPU、栈、堆、Flash、复制、日志、轮询、带宽和算法复杂度 | 写可证实低效与测量缺口 |
| 11 | 故障恢复 | 看门狗、超时、重连、复位、降级、故障注入和配置一致性 | 统一风险等级和验证矩阵 |
| 12 | 双核与终审 | 任务迁移候选、IPC/cache、时序预算、回退、全文反查 | 写推荐方案并完成终稿 |

## 6. 分工与交叉审计

| 审计线 | 独立范围 | 主线复核点 |
|---|---|---|
| CM7/共享 STM | 当前 CMake 源集、任务、控制、传感器、通信、安全 | 所有 High/Critical、任务周期、控制和停机门控 |
| CM4/双核 | CM4 构建能力、启动、链接、内存、HSEM/IPC、cache/MPU | 是否真的可启动；共享内存和外设所有权是否成立 |
| S3 | ESP-IDF 组件、任务、BLE/Wi-Fi/UART/雷达、内存和恢复 | STM 协议对称性、背压、连接/CCC、资源回收 |
| 主线 | 项目规则、构建清单、共享协议、跨芯片状态机、报告 | 路径/行号反查、去重、分级、事实/建议边界 |

独立审计结果只能作为候选发现。主线必须回读当前文件后才能进入终稿。

## 7. 重点检查清单

### 7.1 STM32H757

- [ ] CM7 启动顺序、时钟、MPU、I/D cache、CM4 唤醒和失败路径。
- [ ] CM4 是否生成可运行固件，向量表、链接区和启动握手是否完整。
- [ ] 每个任务的创建者、入口、优先级、栈、周期、阻塞点、deadline 和健康上报。
- [ ] SysTick/FreeRTOS tick/硬件计时器/微秒时间戳的一致性和溢出处理。
- [ ] ISR 中允许的 API、优先级与 `configMAX_SYSCALL_INTERRUPT_PRIORITY` 约束。
- [ ] DMA 缓冲区位置、cache 一致性、对齐、生命周期和回调竞争。
- [ ] UART/I2C/SPI/TIM/GPIO/ADC/PWM 句柄与引脚唯一所有权。
- [ ] 传感器离线、非有限值、陈旧样本、标定失败和姿态未就绪处理。
- [ ] PID 积分/微分、动态 dt、限幅、斜坡、轮序、符号和停机清零。
- [ ] 连接、同步、BUS_OFF、紧停、故障恢复和看门狗是否形成闭环。
- [ ] 全局/静态共享状态的互斥、原子性、可见性和快照一致性。
- [ ] 任务栈、静态/动态分配、日志格式化和浮点开销的可测量性。

### 7.2 ESP32-S3

- [ ] `app_main` 初始化顺序、失败回滚和部分启动状态。
- [ ] 每个 FreeRTOS 任务的优先级、栈、核心亲和、阻塞和删除/重启语义。
- [ ] UART2 STM 链与 UART1 雷达链完全独立，缓冲区和事件队列不互相占用。
- [ ] BLE 连接、MTU、CCC、拥塞、通知返回值、断连清理和命令准入。
- [ ] Wi-Fi/TCP 上行的连接生命周期、发送背压、超时和凭据/默认配置。
- [ ] 雷达 UART/parser/FIFO/uplink 的所有权、容量、丢弃策略和时间戳。
- [ ] ISR/回调到任务之间的数据复制、锁顺序和回调重入。
- [ ] 内部帧与 App 帧的类型、长度、有限值、保留字段、ACK 和错误映射。
- [ ] PSRAM/内部 RAM 的 DMA 能力、栈位置和高水位测量。
- [ ] 日志速率、敏感数据、阻塞风险和 FFE3 上行资源隔离。

### 7.3 模块独立性

- [ ] 依赖方向是否保持 `HAL/IDF -> BSP/transport -> driver/protocol -> service -> app`。
- [ ] 应用层是否绕过公共接口直接访问 HAL、UART、全局句柄或协议内部结构。
- [ ] 安全状态是否由单一权威维护，还是存在多份可漂移副本。
- [ ] 数据生产者/消费者是否有明确快照、所有权和生命周期。
- [ ] 测试是否使用真实公共接口，是否存在未进入当前构建的重复实现。
- [ ] CMake/Kconfig 是否能误启用诊断路径并替代正常安全链。

## 8. 双核任务拆分评估方法

### 8.1 必须先确认的当前事实

双核方案不能只看任务计算量。第 2、3、4、5、6 轮先确认：

- CM4 当前是否具备独立可烧录镜像与可靠启动/复位握手。
- 各外设属于 D1/D2/D3 哪个域，DMA master 与目标 RAM 的可达性。
- CM7 D-cache 对共享 SRAM/DMA 缓冲的影响以及 MPU 属性。
- 哪些任务直接依赖外设句柄、ISR、控制周期或同一状态机。
- 跨核 IPC 可采用何种 HSEM/IPCC/shared-memory 合约及最坏延迟。

### 8.2 候选方案

| 方案 | CM7 | CM4 | 初始判断 |
|---|---|---|---|
| A：维持单核 | 全部现有实时与服务任务 | 不启用 | 最低迁移风险，隔离和余量有限 |
| B：按实时性拆分 | 传感器采样、姿态、控制、安全、执行器 | 通信编解码、遥测、日志、健康统计 | 通常是首选候选，但必须解决状态快照与失联安全 |
| C：按 I/O/计算拆分 | 运动控制与执行器 | 传感器采集/融合或全部通信 | IPC 高频且耦合闭环，需证明收益大于时序风险 |
| D：安全核 | 常规控制/融合/通信 | 独立安全监督和停机仲裁 | 隔离价值高，但外设仲裁、独立时钟/看门狗和失效独立性要求最高 |

初始不直接宣布最优方案。最终推荐由测量需求、外设亲和、共享数据频率、安全所有权和迁移成本共同决定。

### 8.3 评分维度

| 维度 | 权重 | 通过条件 |
|---|---:|---|
| 安全与失效隔离 | 30% | 任一核故障时执行器可进入确定安全态 |
| 实时确定性 | 25% | 关键周期和最坏抖动有预算、有监测、有超限策略 |
| 外设/内存亲和 | 15% | 无不必要跨域访问；DMA/cache 合约明确 |
| IPC 复杂度 | 10% | 消息有限、有界、版本化、可恢复、无隐式共享指针 |
| 资源余量 | 10% | 两核 CPU、栈、RAM 和总线均保留目标余量 |
| 迁移与回退 | 10% | 可分阶段启用并一键回退至已验证单核路径 |

### 8.4 推荐方案必须包含

- 每核任务表、建议优先级层级、周期/deadline、栈预算和核心亲和。
- IPC 消息、生产者/消费者、最大频率、有效期、序号、超时和清零规则。
- 共享内存段、对齐、cache 属性、所有权转换和内存屏障规则。
- 外设/IRQ/DMA 唯一所有权，不允许两核直接控制同一执行器或 HAL 句柄。
- 启动、停止、复位、固件版本不匹配、单核卡死和 IPC 中断时的安全状态机。
- 从当前单核到目标双核的最小迁移阶段、验收门和回退条件。

## 9. 验证路线

本次只执行不会写源码/构建产物的检查：

- 当前文件、符号、调用和 CMake/Kconfig/链接配置反查。
- 源文件清单、重复编译、未引用模块、跨层 include 和公共 API 搜索。
- Markdown 结构、链接、发现编号和证据字段检查。
- 结束时 Git 状态与本任务文件清单核对。

后续实施前必须补充：

| 层级 | 建议验证 | 本审计能否完成 |
|---|---|---|
| 静态 | 编译源覆盖、调用图、配置一致性 | 是 |
| 主机构建/测试 | CM7 Debug、CM4 固件、S3、协议/控制单测 | 否，本次禁止生成/修改产物 |
| 目标板 | 栈高水位、CPU runtime stats、任务抖动、DMA/cache、复位 | 否 |
| 链路 | UART 双向抓包、CRC/重试、BLE 拥塞、Wi-Fi 背压 | 否 |
| 故障注入 | 单核挂起、IPC 断开、传感器离线、BUS_OFF、看门狗 | 否 |
| 车辆 | 架空轮、低速、负载、紧停和恢复 | 否 |

## 10. 完成条件

- 12 轮审计和报告修订均在进度账本中有记录，不少于用户要求的 10 轮。
- 当前自研构建源集、任务/中断/外设/协议/状态所有权可追溯。
- 所有 Critical/High 发现有当前路径、行号、触发机制、影响、建议和验证方法。
- 双核建议说明“为何最优、依赖哪些未测数据、如何迁移和如何回退”。
- 总报告明确列出审计排除项、未执行的构建/硬件测试和残余风险。
- 最终 Git 状态证明本任务没有修改任何源代码。

## 11. 2026-08-31 当前 SRPv4 五轮增量审计

### 11.1 快照与边界

- 分支：`codex/s3-stm-cn-comments`。
- 基准 HEAD：`f703453727a136d15ff7cacea4530beab6e9c08a`。
- 当前活动 STM-S3 UART2 合同：`Common/SRP/` 的 SRPv4；旧 `Common/SCBP_CAN/` 结论只保留为历史。
- 工作区已有大量其他任务的未提交修改；增量审计不回滚、不归因、不提交。
- 仅允许修改本审计 Markdown；不构建、不烧录、不连接 UART/BLE/Wi-Fi、不运行车辆。

### 11.2 第 13-17 轮

| 轮次 | 主题 | 核心输出 | 状态 |
|---:|---|---|---|
| 13 | SRPv4、ACK/重试、同步、CM7 控制与安全 | 活动构建图、chassis/MotorBoard/姿态门和链路状态 | complete |
| 14 | CM4 启动、D2 内存、RTOS/IPC 和 owner | 双核启用阻断项、物理别名与迁移边界 | complete |
| 15 | S3 UART、BLE、雷达、Wi-Fi/TCP 和资源 | 断连顺序、RX 不连续、BLE 拥塞、TCP deadline 和本机配置 | complete |
| 16 | App-S3-STM、ROS2、控制权与失联停机 | 急停编码、V1/V2 租约、失联链和 ROS2 安全边界 | complete |
| 17 | RTOS/栈/调试配置、文档一致性与终审 | 任务清单、可复现构建、文档漂移和最终边界检查 | complete |

### 11.3 增量完成条件

- 每轮都回到当前源码/CMake/Kconfig/链接脚本并标注静态证据等级。
- 所有 High 项都给出修改位置、原因、最小内容、潜在影响和分层验证方法，但本轮不实施。
- 总报告追加当前 SRPv4 章节，不把迁移前 SCBP 章节伪装成现行事实。
- 执行 Markdown `git diff --check`、路径/行号复核和最终写入边界检查。
- 终稿明确未执行构建、烧录、目标板、UART/BLE/Wi-Fi 和车辆验收。

## 12. 2026-08-31 当前 SRPv4 第 18-27 轮深化审计

### 12.1 方法与边界

- 快照仍为分支 `codex/s3-stm-cn-comments`、HEAD `f703453727a136d15ff7cacea4530beab6e9c08a`；终稿前重查关键脏源码哈希和行号。
- 每轮需有独立输入、新证据、旧结论修正或关闭记录；不以重复第 13-17 轮文字充数。
- 允许读取锁定的 ESP-IDF 5.5.4 依赖源来验证 API 语义，但不审计第三方库本身的通用正确性。
- 仍只修改审计 Markdown；不构建、测试、烧录、抓包、连接设备或运行车辆。

### 12.2 第 18-27 轮

| 轮次 | 主题 | 独立核心输出 | 状态 |
|---:|---|---|---|
| 18 | 构建图/clean checkout | 未跟踪头、S3 target、secret 模板、死 CMake/cache、重复 BSP test | complete |
| 19 | SRPv4 对抗输入 | BUS_OFF stop/recover、retry 上限、ACK/epoch/replay、callback 重入、半帧超时 | complete |
| 20 | STM/S3 UART/DMA | S3 无界 TX 入队、两端 discontinuity、CM7 recover/TX 并发、回滚/诊断 | complete |
| 21 | motion owner/lease | App 急停、BLE connection epoch、V1/V2 deadman、CM7 统一 gate、chassis 启动 | complete |
| 22 | MotorBoard 闭环 | RUNNING 门、MSPD watchdog、动态 dt、物理优先 stop 和冻结轮序/几何 | complete |
| 23 | 传感器/DualAHRS | atomic snapshot、BMI dynamic health/recovery、sensor-to-body 合同、旧 IMU 结论关闭 | complete |
| 24 | RTOS/资源/watchdog | S3 10 ms 调度粒度、task WDT 覆盖、两端 partial-start admission | complete |
| 25 | BLE/session/TX | App GATT-ready 盲控窗口、with-response 完成、GATT 1032/517 合同、BLE lifecycle/TX owner | complete |
| 26 | radar/S3RD/ROS2 | TCP 认证/DoS、跨端 stale 漏洞、S3/ROS 时钟合同、默认安全边界 | complete |
| 27 | 跨域终审 | liveness/lease 矩阵、发布/车辆/双核/ROS2 停止门、文档漂移和结论去重 | complete |

### 12.3 完成条件

- 详细证据写入 `.planning/stm-s3-full-audit-20260829/round18-27-current.md`，正式报告只保留高优先级发现、结论修正和最小修复/验证门。
- 所有建议列出修改位置、原因、最小内容、潜在影响和分层验证；不在本审计实施。
- 区分 tracked 默认、ignored 本机配置、旧 build/cache 和当前源码，不复制本地秘密。
- 重新执行关键行号/哈希、本地链接、状态标记、`git diff --check` 和 Git 写入边界检查。
- 终稿明确没有 clean build、匹配镜像、物理链路、RTOS 实测或车辆验收证据。

## 13. 2026-08-31 当前 SRPv4 第 28-37 轮深化审计

### 13.1 方法与只读边界

- 快照仍为 `codex/s3-stm-cn-comments` / `f703453727a136d15ff7cacea4530beab6e9c08a`，结论覆盖该 HEAD 上的当前脏工作树，不代表 release tag。
- 只修改本审计 Markdown；不修改/格式化/回滚/提交固件、App、配置、生成物或构建产物，不运行 build/test/flash/capture/vehicle 命令。
- 第 28-37 轮必须覆盖前两组十轮未单独展开的 ABI、并发、时间、错误传播、复杂度、ISR、安全、UI、测试和终审角度；旧发现只能作为交叉验证，不重复计数。
- 若发现秘密，只允许记录路径、条目数量、相等关系、Git 提交/refs 和处置状态；禁止把 SSID、password、token、可逆编码或可用于离线猜测的摘要写入报告。

### 13.2 第 28-37 轮

| 轮次 | 主题 | 独立核心输出 | 状态 |
|---:|---|---|---|
| 28 | ABI/序列化 | App type 表漂移、logical struct packing、S3RD 多份真值、binary32 合同 | complete |
| 29 | 并发/锁/callback | BLE callback 跨 owner、MotorBoard 长 critical、link callback 重入、BLE epoch snapshot | complete |
| 30 | 时间/回绕 | DWT 多回绕、V2 32-bit sequence、boot timing 哨兵及已确认 wrap-safe 路径 | complete |
| 31 | 错误传播/关联 | PID/SYS_CONFIG callback context、coalesce 结果、V1 ACK、fast response/log 返回值 | complete |
| 32 | 内存/复杂度 | ROS O(n²) resync/长度上限、App queue/disk 上界、CM7 调试栈 | complete |
| 33 | ISR/DMA/WCET | USART2 ISR/critical 叠加；FFE3 CCC 向自身 BTC queue 无限等待回投 | complete |
| 34 | secret/固件/BLE/network | 公开 tracked/产物凭据、secure boot/flash encryption、BLE/App identity、日志隐私 | complete |
| 35 | App/UI/lifecycle | readiness、分阶段反馈、PID transaction、hide/terminate zero 边界 | complete |
| 36 | test/CI/reproducibility | App 必需源未跟踪、motion/fault 测试空白、无 CI/产物溯源 | complete |
| 37 | 跨域终审 | 根因去重、S0/R0/P0/M0/T0/N0/D0/V0 独立停止门、CM4 建议复核 | complete |

### 13.3 安全事件与验收门

| 门 | 目标 | 静态审计后的动作 |
|---|---|---|
| S0 secret incident | 撤销已进入 HEAD/history 的活动凭据，扫描全部 refs/artifacts/caches | 先轮换/撤销；history rewrite 必须另行授权并协调所有 clone/远端 |
| R0 release | clean checkout 源图、固定 target/security profile、可追溯产物 | 当前脏树只作开发快照，不签 release |
| P0 protocol | ACK/replay/epoch/context/callback/wrap/fault test 闭环 | 未通过前不开放产品 motion/动态 baud |
| M0 motion | App-S3-CM7-MotorBoard 分层 lease、最终非零准入和物理优先 stop | 未通过前不进入车辆运动验收 |
| T0 realtime | ISR/critical WCET、HWM/heap/queue/ring 实测 | 未量测不声明时延或资源余量 |
| N0 ROS2 live | 认证、新鲜度、client/frame deadline、时间映射 | 保持 `transport: unconfigured` 或受控 PoC/replay |
| D0 CM4 | boot/物理内存/CM4F RTOS/mailbox/cache/reset/heartbeat/单核回退 | 保持 CM7-only；CM4 首阶段仅 no-op + heartbeat |
| V0 vehicle | 前置门通过且匹配镜像完成分阶段台架/实车签收 | 源码/构建绿灯不得替代车辆 READY |

### 13.4 完成条件

- 详细证据写入 `.planning/stm-s3-full-audit-20260829/round28-37-current.md`；正式报告第 18 节只保留 Critical/High、根因族、最小建议和验收门。
- 所有 Critical/High 结论在终稿快照复查路径、行号或 Git 关系；秘密复核不得显示原值。
- 关键源码哈希、本地 Markdown 链接、轮次/状态、`git diff --check` 和 audit-only 写入边界通过。
- 明确本轮没有 build、host test、烧录、eFuse/option-byte 读取、UART/BLE/TCP 抓取、RTOS/WCET 量测、目标板或车辆验收。

## 14. 2026-08-31 当前 SRPv4 第 38-47 轮深化审计

### 14.1 方法与边界

- 起始快照为 `codex/s3-stm-cn-comments` / `f703453727a136d15ff7cacea4530beab6e9c08a` / status 151项；同一HEAD下脏源码可继续漂移，终稿重拍行号与非secret hash。
- 只修改本审计Markdown；不修改/格式化/回滚/提交源码、App、配置、secret、生成物或构建产物，不运行build/test/flash/capture/vehicle。
- 已知公开secret事件保持S0阻断，本十轮禁止读取/输出值，只允许记录路径、数量、命中/处置状态。
- 每轮必须提供新证据、独立关闭或纠偏，不重复前37轮的ACK/lease/BUS_OFF/watchdog文字充数。

### 14.2 第38-47轮

| 轮次 | 主题 | 独立核心输出 | 状态 |
|---:|---|---|---|
| 38 | compiler/UB/转换 | fault strict alias、ROS窄化、C/C++ header、vendor lock UB、诊断profile | complete |
| 39 | reset/brownout/re-arm | peer boot epoch、earliest physical stop、reset cause、retained fault交付 | complete |
| 40 | persistence/OTA | NVS整擦、OTA rollback owner、分区表、App session原子完成 | complete |
| 41 | control numeric | heading min-wheel对requested/applied linear speed的改写 | complete |
| 42 | sensor/cal evidence | timestamp/quality保护、cal-result端到端合同、新odometry时间对齐 | complete |
| 43 | lifecycle/rollback | STM UART、uplink、service staged init与重试资源守恒 | complete |
| 44 | parser budget | MotorBoard ACK相关性、Swift parser容量/复杂度及固定容量保护 | complete |
| 45 | diagnostic truth | wheel source freshness、ONLINE/READY、counter/snapshot/log loss语义 | complete |
| 46 | supply chain | ROS/GCC锁定、LICENSE/SBOM、YDLidar provenance、generator环境 | complete |
| 47 | cross-domain closeout | 33项去重、停止门更新、冻结合同与CM4结论 | complete |

### 14.3 完成条件

- 详细证据写入 `.planning/stm-s3-full-audit-20260829/round38-47-current.md`；正式报告第19节保留带High等级发现、关键Medium根因、建议与停止门。
- 所有High/Medium-High在最终工作树重新定位；33项统计、轮次、计划/报告/findings/progress/task plan一致。
- secret字面量审计文档命中0，本地Markdown链接、非secret关键源码hash、状态、`git diff --check`和audit-only写入边界通过。
- 明确没有O0/Os build、sanitizer/fuzz、NVS/OTA/掉电/reset、UART/BLE/TCP、目标板或车辆验收证据。
