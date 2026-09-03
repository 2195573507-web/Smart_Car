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

## 会话：2026-08-31（当前 SRPv4 五轮增量审计）

### 第 13-15 轮

- **状态：** complete
- 快照：分支 `codex/s3-stm-cn-comments`，HEAD `f703453727a136d15ff7cacea4530beab6e9c08a`，工作区存在大量其他任务的未提交修改。
- 第 13-14 轮详细证据已写入 `round13-14-stm-current.md`；仅新增该 Markdown，未修改任何 STM 源码或配置。
- 第 15 轮已完成 S3 UART/BLE/雷达/Wi-Fi/TCP/资源复核，包括 BLE 断连后遗留队列命令可重新解除零速、未加密 FFE1 写入、RX ring 丢字节不报 discontinuity、BLE 多生产者拥塞和 TCP WAIT 无应用级 deadline。
- 当前本机忽略的 `sdkconfig` 启用了雷达 uplink，而 Kconfig/fresh clone 默认关闭；忽略的本地凭据内容未写入审计报告。
- `Common/SmartCarDebug/` 当前未跟踪，但 S3/CM7 构建图已引用其头文件；记录为 clean checkout 可复现性阻断。

### 第 16-17 轮与文档合并

- **状态：** complete
- 已确认控制页红色急停按钮发送 App V1 `CONTROL/STOP`，S3 当前无该类型分支并返回拒绝；零四轮 BRAKE 路径与此不同。
- 已合并 ROS2 安全默认、V1/V2 租约、RTOS 任务名单、调试配置跟踪状态和文档漂移证据。
- 本轮误用无效 exec cell 等待协作任务；已改用协作任务消息/状态，该错误没有写入工程源码。

### 第 13-17 轮终稿

- **状态：** complete
- 正式计划已追加第 11 节，总报告已追加第 16 节；旧第 1-12 轮保留为 SRPv4 切换前历史。
- findings 已写入第 13-17 轮证据、历史结论修正、P0/P1 停止条件和未验证项。
- 总报告已给出 High 发现、S3 中等风险/资源修正、App 急停/租约、ROS2 安全边界、后续最小修复位置/原因/内容/影响/验证。
- 本审计没有修改固件、Swift、CMake、Kconfig、IOC、linker、startup 或生成文件，也没有构建、烧录、抓包或车辆验收。
- 终稿 `git diff --check` 在审计 Markdown 限定路径和全工作树上均无输出；第 13-17 轮状态和章节标记检查通过。
- 本轮可归因的写入仅有正式计划、总报告、三个规划账本和 `round13-14-stm-current.md`；最终工作区其余源码/配置脏改动未被本审计修改、回滚或提交。

## 会话：2026-08-31（第 18-27 轮深化审计）

### 启动快照

- **状态：** in_progress
- 用户要求在已完成第 13-17 轮后再增加 10 轮，本次编号为第 18-27 轮。
- 边界不变：只修改审计 Markdown，不修改/格式化/回滚/提交固件、App、配置和生成文件，不构建、烧录、抓包或运行车辆。
- 起始快照：分支 `codex/s3-stm-cn-comments`，HEAD `f703453727a136d15ff7cacea4530beab6e9c08a`，`git status --porcelain` 为 145 项。
- 同一 HEAD 下未提交源码仍在演化；第 18-27 轮不沿用旧行号，终稿前必须重拍关键源码快照。
- 十轮已按构建/SRP/UART、motion/MotorBoard/IMU、RTOS/BLE/radar-ROS2 和跨域终审分组，每轮需产生独立输入、新证据或对旧结论的关闭/修正，不以重复文字充数。

### 第 18-23 轮：构建、SRP/UART、motion/MotorBoard/IMU

- **状态：** complete
- 第 18 轮确认 clean checkout 的两个 High 阻断：未跟踪 `Common/SmartCarDebug`、S3 tracked 配置未固定 `esp32s3` target；同时记录 uplink secret 模板、死 CMake/cache 和重复 BSP test。
- 第 19 轮新增 BUS_OFF 无 stop 屏障、transport retry 失败无上限、ACK/epoch/replay 不严、callback 重入和半帧超时缺口。
- 第 20 轮对照 IDF 5.5.4 确认 S3 UART TX-ring 入队使用 `portMAX_DELAY`，并确认 CM7 recover 未取 TX mutex、两端 discontinuity/init rollback/诊断不完整。
- 第 21-22 轮将 App 急停/BLE epoch/operator lease/CM7 gate 与 MotorBoard FAILED/feedback/dt/priority stop 串成完整执行器风险链，并显式冻结轮序、193 mm、trim 和符号。
- 第 23 轮保留 DualAHRS publish 无锁发现，新增 BMI dynamic health/recovery 和 sensor-to-body 合同矛盾；关闭 LSM DRDY 掩码和“整条 IMU 无锁”旧误报。

### 第 24-27 轮：RTOS/BLE/radar-ROS2 与跨域终审

- **状态：** complete
- 第 24 轮确认 S3 service 一 tick 是 10 ms，task WDT 未直接监督项目关键任务，两端部分初始化失败缺统一 admission/回滚。
- 第 25 轮新增 App GATT-ready 盲控窗口、disconnect zero 不等 write response、S3 GATT `1032` 与 IDF 公开 `517` 上限冲突，并交叉确认 BLE lifecycle/TX 无单一 owner。
- 第 26 轮确认 live TCP 无认证，单 client 可占 listener，S3 pending 旧包可在 ROS2 按新 receive time 通过 stale 门，S3 source timestamp 未映射 ROS time；仍保留 `unconfigured` 为安全默认。
- 第 27 轮建立 operator/session/transport/feedback lease 矩阵，区分车辆、release、SRP/UART、BLE、ROS2 live 和 CM4 六类停止门，双核推荐仍为 CM7-only -> CM4 no-op/heartbeat -> 低风险服务。
- 详细证据已写入 `round18-27-current.md`，正式计划第 12 节和总报告第 17 节已完成合并。
- 终稿前 16 个关键源文件 SHA-256 复核全部通过；审计期间这些证据源未漂移。

### 终稿验证

- **状态：** complete
- 第 18-27 轮在正式计划第 12 节、总报告第 17 节和 `round18-27-current.md` 中均有独立记录，无待审计/等待回传/TODO 占位。
- 16 个关键源文件 SHA-256 在终稿前全部 `OK`，关键 High 行号未漂移。
- Markdown 本地链接、章节编号、第 18-27 轮 complete 状态和未验证声明检查通过。
- 限定审计文件与全工作树 `git diff --check` 均无输出；本轮最终 Git 快照为 147 项，其中其他任务新增了未跟踪 `.planning/s3-stm-cn-comments/`，本审计未读写/回滚该目录。
- 本十轮只新增 `round18-27-current.md`并继续修订本任务正式计划、总报告和三个规划账本；未修改任何固件、App、CMake、Kconfig、IOC、linker、startup 或生成文件。
- 未执行构建、主机测试、烧录、UART/BLE/TCP 抓包、RTOS 量测、目标板或车辆验收；终稿不再继续扩展轮次。

## 会话：2026-08-31（第 28-37 轮深化审计）

### 启动快照

- **状态：** in_progress
- 用户再次要求增加 10 轮，本次编号第 28-37 轮；不重复第 18-27 轮的构建/SRP/motion/BLE/S3RD 文字。
- 起始快照仍为分支 `codex/s3-stm-cn-comments`、HEAD `f703453727a136d15ff7cacea4530beab6e9c08a`、`git status --porcelain` 147 项。
- 边界不变：只写本审计 Markdown，不修改/格式化/回滚/提交固件、App、CMake/Kconfig/IOC/linker/startup/生成文件，不构建、测试、烧录、抓包或运行车辆。
- 第 28-37 轮主题为 ABI/并发/时间/错误传播/内存/ISR/安全/交互/测试/终审；每轮必须产生新发现、独立交叉验证或明确关闭旧结论。

### 第 28-37 轮证据整理

- **状态：** complete
- 第 28-32 轮已写入 `round28-37-current.md`：ABI/类型表、锁与 callback、DWT/序号回绕、ACK context/错误传播、缓冲与复杂度。
- 第 33 轮已写入 ISR 初稿：USART2 priority-5 callback 的 512 B copy/cache/HAL re-arm 与 MotorBoard 同级 IRQ/长临界区存在叠加 WCET 风险；当前仅为静态风险，仍待独立回读和目标板时延量测。
- 第 34 轮已写入发布安全初稿：当前配置未启 secure boot/flash encryption，BLE SMP 未被 GATT 命令准入实际要求；记录 BDA/SSID 日志隐私边界，未读取或复制任何本地凭据值。
- 第 34 轮安全线升级一项 Critical：活动 ignored 凭据与 HEAD/历史中的 tracked 凭据匹配；已用不输出原值的集合比较复核。文档只记录路径、数量和提交关系，要求先轮换/撤销；本轮未改 secret、未清历史、未 push。
- 安全/ISR 独立线补充并复核：远端仓库为 PUBLIC，活动 secret 已进入现有 S3 ELF/BIN，tracked shared header 另含 SoftAP secret；官方 App 只按 name/UUID 选车；FFE3 CCC 同步 flush 可 self-post 填满 BTC queue；另记录 task critical/raw diagnostics 和 HAL/cache 关闭项。总计更新为 36 项、1 Critical/10 High。
- 第 35-36 轮经独立 App/测试审计线回读：确认 UI readiness/反馈/lifecycle 缺口，并新增未跟踪 `SessionLogWriter.swift`、无 App test target/仓库 CI、motion/fault 主链测试缺口；现有 radar/shared-source host test 对齐被记录为保护。
- 第 37 轮已完成风险去重和 S0/R0/P0/M0/T0/N0/D0/V0 独立停止门初稿；双核推荐保持 CM7-only -> CM4 no-op/heartbeat -> 低风险服务。
- 正式计划已追加第 13 节，列出第 28-37 轮完成记录、secret 安全记录规则和独立验收门；Critical 定义已覆盖确认活动秘密进入 Git 历史的场景。
- 正式总报告已更新头部/执行摘要并追加第 18 节：十轮记录、Critical 凭据事件、10 个 High、Medium/保护边界、含五要素的最小建议、独立停止门和 CM4 结论。
- 终稿前首次哈希复核发现同一 HEAD 下并行脏源码仍在变化，status 从 147 增至 151；已重定位第 28 轮 command bridge 行号，最终哈希/写入边界需在最后一次复核时重新锚定。
- 本阶段仍只修改审计 Markdown；未修改源码/配置，未构建、测试、烧录或抓取运行数据。

### 第 28-37 轮终稿验证

- **状态：** complete
- 最终锚点：2026-08-31 04:48:23 CST，HEAD 不变，status 151 项；13 个关键非 secret 源文件连续两次 SHA-256 一致。
- 详细文件共有第 28-36 轮 36 项发现（1 Critical、10 High）和第 37 轮去重/验收门；正式计划第 13 节、总报告第 18 节、findings/task plan 状态一致。
- 已知 secret 字面量 12 个对六份审计文档的精确命中为 0；本地 Markdown 链接缺失 0。
- 限定审计文件与全工作树 `git diff --check` 均无输出；本轮可归因写入仅为正式计划、总报告、三个规划账本和 `round28-37-current.md`。
- 未执行 build、host test、烧录、eFuse/option-byte 读取、UART/BLE/TCP 抓取、RTOS/WCET 量测、目标板或车辆验收。

## 会话：2026-08-31（第 38-47 轮深化审计）

### 启动快照

- **状态：** in_progress
- 用户在第 28-37 轮完成后再次增加 10 轮，本次编号为第 38-47 轮。
- 起始快照：2026-08-31 04:54:08 CST，分支 `codex/s3-stm-cn-comments`，HEAD `f703453727a136d15ff7cacea4530beab6e9c08a`，`git status --porcelain` 151 项。
- 边界不变：只写本审计 Markdown；不修改/格式化/回滚/提交固件、App、配置、secret、生成物或构建产物，不运行 build/test/flash/capture/vehicle。
- 已知凭据事件仍为 S0 阻断；本轮只记录路径、数量、相等/命中与处置状态，不读取或输出原值。
- 新十轮按 UB/复位/持久化、数值/传感/生命周期、解析预算/可观测性/供应链和终审分组，不重复前 37 轮文字充数。

### 第 41 轮主线初稿

- **状态：** in_progress
- 新增航向min-wheel-speed策略改变左右平均速度、使applied linear显著高于requested/ramped v的控制语义风险；单轮1000上限仍是保护。
- 已确认kinematics finite/范围、heading dt/integral/correction/slew与PID有限值/饱和保护，不重复旧dt/watchdog发现。

### 第 38 轮独立审计回传

- **状态：** complete
- 新增CM7 fault record strict-alias UB、ROS参数窄化/索引前float-to-int、SRP C/C++ header拼写、当前不可达vendored serial lock UB和diagnostic profile缺口。
- 已按当前可达性分级：fault/ROS参数为Medium，vendor serial仅Low/Medium；未把编译进静态库等同于运行调用。
- 本轮未编译O0/Os、未运行sanitizer或fault injection。

### 第 39 轮独立审计回传

- **状态：** complete
- 新增peer boot epoch/re-arm缺失、CM7早期初始化hang早于物理zero、两端reset cause/boot id缺失、retained fault记录清除早于日志accepted四项。
- 已明确旧motion不在noinit；reset后再运动来自仍连接App heartbeat/外板保持输出，而非CM7 BSS自动恢复。
- 未读取设备reset/eFuse/option状态，未执行brownout/reset/fault注入。

### 第 40 轮独立审计回传

- **状态：** complete
- 新增NVS错误整区擦除、双OTA分区无rollback/health owner、16/32 MB分区表误选和App session非原子完成四项。
- 已明确当前没有项目OTA入口、CM7/S3无其他自研Flash文件写；“分区存在”不升级为“OTA已支持”。
- 未运行掉电、磨损、NVS/OTA或文件系统故障注入。

### 第 42 轮主线初稿

- **状态：** in_progress
- 纠正为更基础的当前调用图：内部result/quality存在，但Common无CAL_RESULT ID、CM7无发送、S3无relay，只有App 0x25 decoder；端到端标定结果/来源完全缺失。
- 已确认BMI/LSM独立source timestamp、dt/freshness、LSM pair、calibration去重/motion/quality/leveling保护，不重复旧恢复/极性发现。
- 并行源码新增chassis state/odometry后完成增量复核：wheel snapshot有timestamp/sequence，但yaw为当前无timestamp值，新增odometry time-skew Medium；总数更新为33项。

### 第 43 轮主线初稿

- **状态：** in_progress
- 新增S3 STM-UART task失败残留driver/mutex导致直接重试不可恢复、radar uplink中后段失败不逆序释放network/event资源、smartcar service非幂等/commit顺序三项生命周期风险。
- 确认radar UART完整rollback、多个task handle guard和IMU recovery复用长期BMI task等保护。

### 第 44 轮主线初稿

- **状态：** in_progress
- 新增 MotorBoard 通用 ACK 未绑定当前命令/read-flash `Set ` 可提前推进，以及 Swift 控制帧 parser 无 buffer 上限、坏帧逐字节前删的复杂度风险。
- 确认 S3 App/SRP C parser 固定容量且线性，App log parser 有 read-index/上限；旧半帧 timeout 和 Poll drain 缺口只引用、不重复计数。
- 仅修改审计 Markdown；未运行 parser test/fuzz、构建或目标板输入注入。

### 第 45 轮主线初稿

- **状态：** in_progress
- 新增 wheel status source freshness 缺失：MSPD停流后旧数组仍以50 ms周期转发，App receipt time会把旧反馈显示成新鲜。
- 新增 BLE notify failure counter 多生产者非原子、FFE2/FFE3混合且漏 invalid-state/异步未交付的诊断语义缺口。
- 已区分其他使用 critical/mutex/single-owner 的 stats getter，不扩大为“所有计数器均撕裂”。

### 第 46 轮主线初稿

- **状态：** in_progress
- 新增 ROS Docker mutable base/unpinned apt、CM7 PATH-only compiler和项目级LICENSE/NOTICE/SBOM不完整三类供应链/复现缺口。
- 已记录保护：IDF lock固定5.5.4/esp32s3，YDLidar 1.0.6 inventory/license/local patch说明，CMSIS/HAL/FreeRTOS源内许可，无活动submodule，Swift无外部包。
- 许可证结论仅是材料/溯源缺口，不推断法律违规；本轮未联网下载依赖或构建镜像。
- 独立线补充 App ONLINE/SRP sync反向语义、Radar/TCP本地READY命名、隐藏计数/epoch、radar invalid snapshot和日志端到端loss budget；供应链补充YDLidar不可重建来源与CubeMX/Swift生成环境。

### 第 47 轮终审初稿

- **状态：** complete
- 第38-46轮共33项：0新增Critical、2 High、5 High/Medium组合；第34轮secret Critical仍开放。
- 已按编译、reset、持久化、数值/标定、init事务、parser、诊断和供应链八类去重，更新S0/R0/P0/M0/T0/N0/D0/V0要求。
- CM4推荐不变；冻结轮序/193 mm/trim/符号/协议ID和安全门未被修改建议触碰。
- 正式计划已追加第14节，总报告已更新头部并追加第19节；findings已写入十轮摘要、带High根因、关闭项和停止门更新。
- 下一步只做最终源码hash/行号、secret 0命中、链接、状态和audit-only写入边界复核。
- 首次终稿检查时并行工作树已从151增至161项，新增chassis state/odometry且部分SRP/S3/MotorBoard源码漂移；本审计未写这些路径，已暂停complete标记并在当前源码重核7项带High发现。
- 13:49复核确认7项仍开放；MotorBoard已新增内部wheel timestamp/sequence/valid，但legacy 0x14仍16 B且App仍用receipt time，所以仅新chassis state路径受freshness保护。

### 第38-47轮终稿验证

- **状态：** complete
- 14:01 provisional锚点因后续漂移废止；最终源码静止后于14:29重拍35路径，连续manifest为`1de69c579ecfcfb4dd5d1ba2bf16e4fac9d5d5e1cd9de85589742ab9f8203ce6`，HEAD不变、status 161项。
- 第38-46轮33项：0新增Critical、2 High、5 High/Medium组合；第47轮完成根因去重/停止门，旧secret Critical仍开放。
- 已知12个secret字面量对七份审计文档精确命中0；本地Markdown链接缺失0。
- secret、链接、统计、限定/全树`git diff --check`和hash稳定性均通过；本轮仅写审计Markdown。
- 未执行O0/Os build、sanitizer/fuzz、NVS/OTA/掉电/reset、UART/BLE/TCP、目标板或车辆验收。
