# STM32H757 与 ESP32-S3 全量只读审计进度

> Historical/deprecated progress log. It records the pre-SRPv4 audit baseline,
> not the current active protocol or build graph.

## 会话：2026-08-29

### 阶段 1：边界、基线与清单

- **状态：** complete
- 已执行：
  - 完整读取 `using-superpowers` 与 `planning-with-files-zh` 技能规则。
  - 确认已有目标就是本次全量只读审计，状态 active。
  - 检查初始 Git 状态并识别用户现有 ROS2 文档改动。
  - 恢复根级旧规划上下文，确认不能覆盖。
  - 建立本任务独立规划作用域和 12 轮审计框架。
  - 读取根 README、模块/文档索引、STM/S3 架构材料与 STM 局部 README。
  - 统计受跟踪路径并区分自研、供应商、生成与二进制/工程材料的审计口径。
  - 完整读取 `.codex` 启动/规则/工作流/索引与项目状态、决策和规范文档。
  - 创建正式审计计划，明确 12 轮修订、证据分级、双核评分与完成条件。
  - 创建总报告第 1 轮基线稿，建立后续 11 轮实际修订入口。
  - 复核共享代码 Git 状态，否定“Common 未跟踪”的初步假设，并修正计划/报告范围为 1707 个受跟踪路径。
  - 确认当前两端构建使用 `Common/SCBP_CAN`，而不是历史 `Common/SRP`。
  - 第 2 轮回读 CM7/CM4 main、CMake、linker、startup、IOC 和 HSEM/IPC 搜索结果。
  - 总报告第 2 次修订：确认 CM7-only，记录 CM4 启动外壳、内存图、配置漂移和双核基础设施阻断项。
  - 记录审计中并行出现的 4 个 S3 源文件改动；不回滚，终稿复核最终工作树。
- 阶段 2 / 第 3 轮已执行：
  - 读取 `Config/FreeRTOSConfig.h`、CM7 主入口、全部项目 `xTaskCreate` 调用、任务入口、周期和同步点。
  - 建立 8 个稳态应用任务与 2 个双 IMU 临时初始化任务的优先级/栈/周期表；估算稳态任务栈请求为 3328 words（约 13.3 KiB），不含 TCB、队列、互斥锁和 idle task。
  - 核对 `configTICK_RATE_HZ=1000`、动态 heap `32 KiB`、栈溢出/ malloc hook、关闭 run-time stats/trace，以及 SysTick、HAL tick、DWT 三类时间读取路径。
  - 发现并记录 `STM-RTOS-001..005`：健康名单含不存在的 `protocol`、关键任务未覆盖；采样绑定低优先级 logger；1 ms 相对轮询存在周期漂移/无效唤醒；动态资源 admission 分散；时间基准和关中断窗口缺少量测。
  - 复核 CM4/CM7 D2 SRAM 两个地址别名与现有 map，记录条件性 High 发现 `STM-DUAL-004`（CM4 `.data/.bss` 可能覆盖 CM7 UART DMA 区）。
  - 报告第 3 次修订：补充任务表、调度风险、资源预算、健康监测缺口和双核内存别名风险。
- 本任务创建/修改的文件：
  - `.planning/stm-s3-full-audit-20260829/task_plan.md`
  - `.planning/stm-s3-full-audit-20260829/findings.md`
  - `.planning/stm-s3-full-audit-20260829/progress.md`
  - `DOCS/STM_S3_FULL_CODE_AUDIT_PLAN.md`
  - `DOCS/STM_S3_FULL_CODE_AUDIT_REPORT.md`
- 源代码修改：无。

## 审计轮次记录

| 轮次 | 主题 | 报告修订状态 | 交叉复核 | 状态 |
|---:|---|---|---|---|
| 1 | 范围、目录、构建入口、工作树基线 | 报告第 1 轮基线稿已完成 | CMake、Git、规则和共享源已复核 | completed |
| 2 | 架构、启动、链接、存储、双核现状 | 报告第 2 次修订完成 | 主线与 CM4 独立审计一致 | completed |
| 3 | STM 调度、同步、时间基准 | 报告第 3 次修订完成 | CM7 任务创建、配置、时间路径和 CM4 别名证据已复核 | completed |
| 4 | STM BSP/驱动/中断/DMA/外设 | 报告第 4 次修订完成 | GPIO/IRQ/DMA/总线调用链已反查 | completed |
| 5 | STM 中间件/传感器/标定/控制 | 已写入证据与风险 | 已交叉复核 | completed |
| 6 | STM 应用/安全/恢复/独立性 | 已写入证据与风险 | 已交叉复核 | completed |
| 7 | S3 调度/通信/内存/雷达 | 已写入证据与风险 | 已交叉复核 | completed |
| 8 | 跨芯片协议/状态机/背压 | 已写入证据与风险 | 已交叉复核 | completed |
| 9 | 分层独立性/耦合/接口治理 | 已写入证据与风险 | 已交叉复核 | completed |
| 10 | CPU/RAM/Flash/栈/带宽/日志效率 | 已写入预算与测量缺口 | 已交叉复核 | completed |
| 11 | 安全/看门狗/故障恢复/配置 | 已写入验证矩阵 | 已交叉复核 | completed |
| 12 | 双核最优拆分/迁移/终稿一致性 | 已写入推荐方案与回退门 | 已终审 | completed |

## 验证结果

| 检查 | 预期 | 实际 | 状态 |
|---|---|---|---|
| 初始 Git 边界 | 识别用户已有改动 | 仅发现 ROS2 规划/文档改动 | pass |
| 源代码写入 | 不得发生 | 截至当前无源代码写入 | pass |
| 正式计划覆盖 | 至少 10 轮、双核、证据和边界 | 已定义 12 轮及完成条件 | pass |
| 第 2 轮双核事实 | 当前核职责、启动、存储、IPC 边界可追溯 | CM7-only；CM4 shell；无 IPC/shared ABI | pass |
| 第 3 轮调度事实 | 任务、优先级、周期、栈、健康监测和时间基准可追溯 | 已建立任务表并记录 5 项调度/观测风险 | pass |
| 第 3 轮边界 | 不误称运行时性能已验证 | 未构建、未测量 CPU/WCET/HWM，均标记待验证 | pass |
| 第 4 轮 BSP 边界 | 底层所有权、DMA/IRQ 风险可追溯 | 已记录 6 项 STM-BSP 发现；HAL ISR 语义和吞吐仍待目标板验证 | pass |

## 错误日志

| 时间 | 错误 | 次数 | 处理 |
|---|---|---:|---|
| 2026-08-29 | 新建目标失败：已有 active 目标 | 1 | 确认现有目标内容等于本次请求，不重试 |
| 2026-08-29 | 初始路径限定造成 Common 跟踪状态误判 | 1 | 改用 `git ls-files --stage Common`，立即修正文档，不保留错误结论 |
| 2026-08-29 | 合并修订补丁找不到账本标题，原子拒绝 | 1 | 读取准确上下文后拆分修订；无文件被部分修改 |

## 会话：2026-08-30

### 第 4 轮：STM BSP/驱动/中断/DMA/外设

- **输入：** CM7 当前 CMake 源集、`main.c`/MSP/IRQ、BSP GPIO/UART/SPI/I2C/TIMER/PWM、UART link、MotorBoard transport、BMI323/LSM303 port。
- **输出：** 报告第 4 次修订；新增 `STM-BSP-001..006` 和 USART/SPI/I2C/TIM 资源所有权矩阵。
- **关键结论：** PA10 在 USART1 RX 与 LF_INT1 之间存在实际重配冲突；UART2 DMA 回调重装和 USART6 ring 背压属于需要运行时/厂商语义确认的静态风险；SPI/I2C 总线所有权目前依赖调用顺序。
- **结论变化：** 底层章节从“待填写”变为已审计；双核建议增加“传感器和实时 UART 不迁移，先固定 owner”的前置条件。
- **未验证项：** 未构建、未烧录、未测 PA10 AF、DMA inactive 窗口、UART 突发丢帧、总线锁等待和 IRQ 延迟。
- **代码边界：** 只修改本任务 Markdown；未修改源代码、配置或生成物。

## 五问重启检查

| 问题 | 答案 |
|---|---|
| 我在哪里？ | 阶段 5：静态验证与交付已完成 |
| 我要去哪里？ | 本轮结束；后续仅按总报告验证矩阵开展设备/链路/车辆验收 |
| 目标是什么？ | 在不修改源代码的条件下完成 STM/S3 全量、可追溯审计 |
| 我学到了什么？ | 见本作用域 `findings.md` |
| 我做了什么？ | 见本文件的阶段与轮次记录 |

## 会话：2026-08-30（快速收尾）

### 终审与交付

- **状态：** complete
- 已将 CM7 第 5/6 轮证据、CM4 双核摘要、S3 任务/BLE/UART/雷达/Wi-Fi 源码检查和 SCBP/App 双层协议检查合并进正式总报告。
- 已完成第 5-12 轮主题记录：传感器/姿态、应用安全、S3、跨芯片协议、模块独立性、资源效率、故障恢复、双核拆分与终稿一致性。
- 总报告现在包含：发现清单、触发机制、现有保护、建议、验证矩阵、双核推荐分工、迁移门和回退方案。
- 计划文件、总报告、findings/progress 账本均已标记 complete；本轮不再继续扩展审计轮次。
- `git diff --check` 已通过；本任务未修改任何 `.c`、`.h`、汇编、CMake、Kconfig、IOC、链接脚本或生成物。
- 工作区仍有其他任务产生的固件、雷达、ROS2 和 Common/SRP 改动；未回滚、未归因、未提交。

### 最终验证边界

- **已完成：** 当前工作树静态源码、构建清单、协议定义、状态/资源所有权和文档一致性复核。
- **未执行：** CM7/S3/CM4 构建、烧录、UART/BLE/Wi-Fi 抓包、故障注入、目标板实时测量和车辆验收。
- **交付判断：** 静态审计文档已完成；High 项和双核迁移仍须按总报告第 11 节验证矩阵取得设备证据后才能进入实施。
