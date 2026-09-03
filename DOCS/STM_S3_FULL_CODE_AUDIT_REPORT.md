# STM32H757 与 ESP32-S3 全量代码审计总报告

> 历史快照说明：本文是 SRP v4 硬切换前的只读审计报告，旧协议描述仅保留
> 作为迁移前证据。当前活动 UART2 合同以 `DOCS/SRP_v4_Spec.md`、
> `Common/SRP/` 和现行源代码为准。

> 本文后续出现的 `SCBP`、`HCS`、`FCS`、旧消息 ID 和旧路径，全部属于
> 迁移前审计快照，已废弃且不构成当前构建输入或运行时合同。

> 2026-08-31 当前 SRPv4 工作树的第 13-17 轮增量审计见第 16 节。
> 该节与前文冲突时，以第 16 节和当前源码为准。

> 用户后续要求再增加 10 轮；第 18-27 轮深化审计见第 17 节。
> 用户再次增加 10 轮；第 28-37 轮深化审计见第 18 节。
> 用户继续增加 10 轮；第 38-47 轮深化审计见第 19 节。
> 该节对当前工作树的结论优先级高于第 16-18 节及迁移前历史。

> 状态：迁移前 12 轮历史终稿 + 当前 SRPv4 第 13-47 轮增量/深化终稿；未执行构建、烧录、链路和车辆验收
> 日期：历史快照 2026-08-30；当前增量快照 2026-08-31
> 审计方式：当前工作树只读静态审计  
> 代码修改：无  
> 计划：[STM_S3_FULL_CODE_AUDIT_PLAN.md](STM_S3_FULL_CODE_AUDIT_PLAN.md)

## 1. 执行摘要

本报告审计 Smart_Car 当前 STM32H757 与 ESP32-S3 固件的构建边界、任务调度、底层驱动、中间件、应用独立性、跨芯片协议、效率、安全和故障恢复，并给出 STM32H757 双核目标拆分方案。

本报告是基于当前工作树的只读静态终稿。审计期间没有修改固件、协议、配置或生成代码；当前已经确认：

- STM32H757 是最终运动与本地安全权威，ESP32-S3 是 BLE/Wi-Fi 网关和雷达端点；本审计不改变该责任边界。
- 当前入口文档表明 CM7 是主实时核；CM4 现有 CMake 目标只声明为构建结构检查，不应把系统描述成已完成双核运行。
- 受跟踪的 STM 树包含大量供应商、生成和二进制材料；全量覆盖以“当前构建中项目自研代码”为主集合，供应商代码只审计集成面。
- 旧架构/状态文档与当前实现之间可能存在漂移；最终结论只接受当前文件、符号和配置反查。
- 本次不构建、不烧录、不做 UART/BLE/Wi-Fi/车辆验证，运行时指标和物理效果会明确列为未验证。

第 34 轮确认一个必须先于普通修复处理的 Critical 安全事件：当前 ignored 活动 Wi-Fi 凭据与公开 GitHub 的 HEAD/历史中受跟踪凭据完全匹配，且两个字段都已嵌入现有 S3 ELF/BIN；tracked shared header 另含 SoftAP 凭据。报告没有记录原值。相关凭据必须先在网络侧轮换/撤销；删除当前文件或新增 `.gitignore` 不能撤销已经传播到 Git refs/objects、clone、缓存、备份和产物的秘密，history rewrite 需要单独授权与协同。

第 2 轮进一步确认，当前固件是明确的 CM7-only 路径：CM7 初始化全部传感、通信、控制和安全任务；CM4 镜像只有 CubeMX 启动外壳，先等待 HSEM 唤醒，醒后没有应用任务并进入空循环。当前代码没有 CM7 侧释放序列、跨核 IPC、共享内存 ABI、心跳或故障回退，因此任何“双核优化”都必须先完成基础设施验证，不能直接迁移控制任务。

第 3 轮完成 CM7 FreeRTOS 任务、周期、栈、同步和时间基准复核。当前稳态任务使用动态堆，应用任务栈参数合计约 13.3 KiB（不含 TCB、队列、互斥锁和 idle task）；健康监测名单与真实任务集不一致，低优先级 logger 是唯一采样者，通信/电机任务使用 1 ms 相对轮询。上述结论是源码级风险判断，尚未替代目标板上的 CPU、WCET、栈高水位和抖动测量。

第 4 轮补齐 STM BSP、驱动、中断和 DMA。第 5/6 轮补齐传感器、姿态、标定、应用安全和恢复；第 7/8 轮补齐 S3、雷达、BLE、UART、跨芯片协议和背压；第 9/11 轮完成层间独立性、资源效率和故障恢复复核；第 12 轮给出双核拆分与迁移门。所有运行时结论仍标为 `UNVERIFIED_RUNTIME`，不能用源码审计替代目标板、链路或车辆验收。

## 2. 审计范围与覆盖口径

### 2.1 主范围

| 域 | 路径 | 覆盖内容 | 当前状态 |
|---|---|---|---|
| STM32H757 CM7 | `STM32H757/CM7/` | 启动、生成集成、CMake、主控制核 | 静态完成 |
| STM32H757 CM4 | `STM32H757/CM4/` | 启动、链接、可构建性、双核准备 | 静态完成 |
| STM 共享自研层 | `STM32H757/BSP/`、`Drivers/` 项目驱动、`Middleware/`、`Application/`、`System/`、`Config/` | BSP、设备驱动、算法、任务、安全、配置 | 静态完成 |
| ESP32-S3 | `ESPS3/main/`、`ESPS3/components/`、项目配置 | FreeRTOS、BLE/Wi-Fi、STM UART、雷达、服务 | 静态完成 |
| 跨芯片协议 | 实际构建引用的共享/本地 SCBP 与帧模块 | ID、布局、CRC、ACK、状态机、背压 | 静态完成 |

### 2.2 文件规模

审计开始时：

| 路径 | 受 Git 跟踪路径数 | 说明 |
|---|---:|---|
| `STM32H757/` | 1626 | 包含 CMSIS/HAL、生成、工程、库和自研文件 |
| `ESPS3/` | 69 | 包含源码、配置、文档与主机测试 |
| `Common/` | 12 | 当前实际构建使用 `SCBP_CAN` 与 `SmartCarLog`；不存在 `Common/SRP` |
| 合计 | 1707 | 不是 1707 个自研源文件 |

### 2.3 排除与限制

- 不逐行重新审计 CMSIS、STM32 HAL、FreeRTOS、ESP-IDF 等供应商实现。
- 不反编译静态库；无源码依赖按黑盒风险记录。
- 不把历史/忽略的构建目录作为规范源。
- App、ROS2、工具只用于跨边界契约核对。
- 不执行会写构建产物、缓存或生成文件的验证。

### 2.4 审计快照边界

- 基线提交：`d8b80c917af035415aa5a2bdf6886ba0984a17ed`，分支 `4`。
- 审计开始后，工作区出现其他任务产生的 4 个未提交 S3 源文件改动：`command_bridge.c/.h` 与 `radar_uplink_protocol.c/.h`。本审计不回滚它们，S3 结论以终稿时当前工作树为准。
- 因为存在并行写入，所有高风险发现会在第 12 轮重新核对路径、行号和最终文件哈希；终稿之后发生的改动不在覆盖范围内。

## 3. 证据与术语

| 标签 | 含义 |
|---|---|
| `CONFIRMED_SOURCE` | 当前源码/配置直接确认 |
| `CONFIRMED_STATIC` | 多处静态交叉检查确认 |
| `EXISTING_DESIGN` | 当前文档或接口表达的既有设计 |
| `INFERRED_RISK` | 代码机制支持，但需运行触发验证 |
| `RECOMMENDED` | 后续建议，不是当前实现 |
| `UNVERIFIED_RUNTIME` | 需要目标板、链路或车辆证据 |

严重度定义见审计计划。最终发现不会仅凭旧文档定级。

## 4. 当前系统架构

### 4.1 已确认责任边界

```text
App / ROS2 (上层意图与观察)
             |
             v
ESP32-S3 (BLE/Wi-Fi 网关、雷达、STM 传输)
             |
             v
STM32H757 CM7 (传感、姿态、控制、安全、执行器权威)
             |
             v
MotorBoard / physical plant

STM32H757 CM4: 当前没有运行职责；仅保留启动/链接外壳
```

### 4.2 STM 当前启动与存储图

| 项目 | CM7 | CM4 | 证据状态 |
|---|---|---|---|
| Flash | `0x08000000`, 1024 KiB | `0x08100000`, 1024 KiB | `CONFIRMED_SOURCE`，两份 linker script |
| 主 RAM | `0x20000000`, 128 KiB | `0x10000000`, 288 KiB | `CONFIRMED_SOURCE` |
| D2 DMA RAM | CM7 额外声明 `0x30000000`, 288 KiB | 未声明独立共享段 | `CONFIRMED_SOURCE` |
| Cache | CM7 开启 I/D cache | IOC 配置 CM4 I/D cache 关闭 | `CONFIRMED_SOURCE`/IOC |
| MPU | CM7 仅 CubeMX 默认 4 GiB guard region | CM4 仅启用 privileged default | `CONFIRMED_SOURCE` |
| RTOS | CM7 启动全部当前任务 | 无 FreeRTOS 源进入 CM4 CMake | `CONFIRMED_SOURCE` |
| 核启动同步 | CM7 明确不等待/释放 CM4 | CM4 使用 HSEM0 后进入 D2 STOP/WFE | `CONFIRMED_SOURCE` |
| IPC/共享内存 | 无 | 无 | `CONFIRMED_STATIC` 搜索 |

CM7 linker 把 `.dma_buffer` 以 32 字节对齐放到 D2 SRAM，并把故障/健康保留记录放入 `.noinit`；这是当前 DMA 与诊断的良好基础，但不是跨核共享内存合约。CM4 linker 没有 `.shared`、mailbox 或版本化 ABI 段。

## 5. 总体发现清单

> 表中发现均已回读当前工作树；运行时触发结果仍按 `UNVERIFIED_RUNTIME` 标记。

| ID | 等级 | 域 | 标题 | 状态 |
|---|---|---|---|---|
| STM-DUAL-001 | High | 双核准备 | CM4 等待 CM7 HSEM 释放，但 CM7 明确不释放；无 IPC/心跳/回退 | 已确认，当前 CM7-only 不受影响，启用双核前阻断 |
| STM-DUAL-002 | Medium | 配置治理 | IOC 声明 CM4 FreeRTOS/OpenAMP/IWDG 等能力，当前 CM4 CMake/源码未集成 | 已确认，属于再生成/双核迁移风险 |
| STM-DUAL-003 | Medium | 内存/缓存 | 当前没有共享内存段、所有权、cache 属性或跨核 ABI | 已确认，双核迁移阻断项 |
| STM-DUAL-004 | High | 双核内存 | CM4 `0x10000000` 起始 `.data/.bss` 与 CM7 `0x30000000` DMA 别名可能重叠 | 条件性确认，释放 CM4 前必须修复布局 |
| STM-RTOS-001 | Medium | 调度/观测 | 健康监测含不存在的 `protocol`，遗漏姿态门/电机等关键任务 | 已确认，诊断覆盖不足 |
| STM-RTOS-002 | Medium | 调度/观测 | 健康采样绑定优先级 1 logger，且关闭运行时统计/trace | 已确认，无法证明无饿死/抖动 |
| STM-RTOS-003 | Medium | 实时性/效率 | S3/UART/电机任务使用 1 ms 相对轮询，周期随工作量漂移 | 已确认机制，WCET 待测 |
| STM-RTOS-004 | Medium | 资源/启动 | 32 KiB 动态堆和任务创建失败结果分散，缺少统一 admission | 已确认设计风险，峰值余量待测 |
| STM-RTOS-005 | Info | 时间基准 | HAL tick、FreeRTOS tick、DWT 并用，关中断窗口无量测 | 已确认观测缺口 |
| STM-BSP-001 | High | BSP/引脚 | `bsp_gpio_init()` 将 USART1 RX/PA10 重配为 GPIO 输入 | 已确认，需硬件回环复核 |
| STM-BSP-002 | Medium | BSP/总线 | SPI/I2C 公共接口没有统一总线串行化合同 | 静态设计风险 |
| STM-BSP-003 | Medium | UART/DMA | ReceiveToIdle 回调内直接调用 HAL DMA 重装 | HAL ISR 语义待确认 |
| STM-BSP-004 | Medium | UART/背压 | USART6 RX ring 溢出只丢字节，不触发帧级恢复 | 已确认机制，阈值待测 |
| STM-BSP-005 | Medium | 实时性 | BMI323 初始化含 DWT 忙等待 | 已确认机制，时长待测 |
| STM-BSP-006 | Low | 资源治理 | 调试冲突/退役 PWM 仍以兼容枚举暴露 | 已确认接口风险 |
| STM-AHRS-001 | High | 姿态并发 | DualAHRS 全局更新/读取未显式同步 | 静态确认，运行后果待注入 |
| STM-IMU-001 | High | 传感器契约 | 活动 BMI323 与 dormant 驱动量程/换算可能漂移 | 静态确认，实际配置待读回 |
| STM-ATT-001 | Medium | 标定 | fast-zero 绕过 500 样本累计流程 | 静态确认，意图与运动影响待确认 |
| STM-CAL-001 | Medium | 标定实时性 | BMI 高速标定锁竞争时丢样本 | 静态确认 |
| STM-SAFE-001 | High | 安全门 | ready 置位后没有持续 IMU freshness 撤销门 | 静态机制确认，故障检测时间待测 |
| STM-ARCH-001 | Medium | 分层 | BSP 反向依赖 S3 service/日志 | 静态确认 |
| STM-CTRL-001 | Medium | 控制实时性 | MotorBoard 控制循环同步发送遥测 | 静态确认，deadline 影响待测 |
| S3-BOOT-001 | Medium | 启动 | STM UART/BLE/雷达部分初始化失败后仍可能继续启动其他服务 | 静态确认 |
| S3-UART-001 | Medium | 通信背压 | UART2 字节环溢出按字节丢弃，可能破坏 SCBP 帧 | 静态确认，阈值待测 |
| S3-BLE-001 | Medium | BLE | 通知按 MTU 分片且逐片同步发送，无拥塞/发送队列 | 静态确认，返回值有计数 |
| S3-SVC-001 | Medium | 服务调度 | BLE 队列深度 8、每轮预算 4，满载时丢弃命令 | 静态确认 |
| S3-RADAR-001 | Medium | 雷达资源 | UART FIFO、解析器、遥测队列多级复制，容量/丢弃策略需统一 | 静态确认 |
| S3-RADAR-002 | Medium | 上行 | Wi-Fi/TCP 重连与发送有界，但实验协议尚未冻结 | 静态确认，网络行为待测 |
| X-STATE-001 | High | 跨芯片状态 | ACK/重试、BUS_OFF、BLE ready、boot-ready 分属不同状态机 | 静态确认，端到端时序待抓包 |
| X-PROTO-001 | Medium | 协议治理 | App `AA 55` 帧与 SCBP-CAN `5A A5...0D 0A` 双层契约并存 | 静态确认 |
| X-PROTO-002 | Medium | 协议恢复 | SCBP ACK 槽仅 4 个，重试由 tick 驱动，突发时会返回 busy/timeout | 静态确认 |
| EFF-001 | Medium | 资源 | S3 静态/动态缓冲与高水位统计不形成统一 admission | 静态确认，峰值待测 |
| SAFE-REC-001 | High | 恢复 | 传感器 freshness、BLE 断链、BUS_OFF 和 CM4 失联需统一安全仲裁 | 设计缺口，运行时待验证 |
| BASE-001 | Info | 范围 | CM4 不能按现有入口文档视为已部署运行核 | 已确认 |
| BASE-002 | Info | 文档 | 早期 scaffold/状态文档可能滞后于当前实现 | 已确认方法风险 |
| BASE-003 | Info | 协议治理 | 历史材料指向 `Common/SRP`，当前两端 CMake 实际使用 `Common/SCBP_CAN` | 已确认当前构建事实，第 8 轮已复核 |
| STM-BSP-001 | High | BSP/引脚 | `bsp_gpio_init()` 将 USART1 RX/PA10 重配为 GPIO 输入 | 已确认，需硬件回环复核 |
| STM-BSP-002 | Medium | BSP/总线 | SPI/I2C 公共接口没有统一总线串行化合同 | 静态设计风险 |
| STM-BSP-003 | Medium | UART/DMA | ReceiveToIdle 回调内直接调用 HAL DMA 重装 | HAL ISR 语义待确认 |
| STM-BSP-004 | Medium | UART/背压 | USART6 RX ring 溢出只丢字节，不触发帧级恢复 | 已确认机制，阈值待测 |
| STM-BSP-005 | Medium | 实时性 | BMI323 初始化含 DWT 忙等待 | 已确认机制，时长待测 |
| STM-BSP-006 | Low | 资源治理 | 调试冲突/退役 PWM 仍以兼容枚举暴露 | 已确认接口风险 |

### 5.1 `STM-DUAL-001`：现有 CM4 不是可直接承载任务的运行核

- **等级/置信度：** High / High；等级针对“启用双核或迁移任务”场景，当前 CM7-only 固件没有被该缺口直接破坏。
- **确定事实：** `STM32H757/CM7/Core/Src/main.c:56` 明确写明不等待或释放 CM4；主初始化在 `main.c:185-232` 启动 IMU、UART link、S3 service、MotorBoard/attitude coordinator 和 FreeRTOS scheduler。
- **确定事实：** `STM32H757/CM4/Core/Src/main.c:35-45,85-100` 启用 HSEM0 通知并进入 D2 STOP/WFE，等待 CM7；`main.c:104-129` 醒后只执行 `HAL_Init()` 并进入空循环。
- **确定事实：** 项目自研源搜索没有 CM7 侧 `HAL_HSEM_FastTake/Release`、IPCC、OpenAMP/RPMsg、共享 mailbox、CM4 heartbeat 或跨核故障状态机。
- **机制/影响：** 直接烧录并启用当前 CM4 镜像时，CM4 要么持续等待未发生的释放，要么醒后空转；它不能安全承载任务，也不能向 CM7 证明健康。若在此基础上把通信或控制从 CM7 移走，CM7 没有确定的失联降级依据。
- **现有保护：** CM7 当前明确保持单核所有权，所以现有运行路径不依赖 CM4。
- **建议：** 第一阶段 CM4 只实现可版本校验的 boot-ready/heartbeat/no-op 镜像；CM7 超时后继续单核安全运行。通过复位、单核缺失和心跳丢失测试后，才允许迁移低风险服务。
- **验证：** 双镜像构建；option-byte/boot-address 核对；逻辑分析或共享 mailbox 观测启动延迟；CM4 缺失、卡死、反复复位故障注入；确认 CM7 始终保留停机权。

### 5.2 `STM-DUAL-002`：IOC 与当前 CM4 集成边界不一致

- **等级/置信度：** Medium / High。
- **确定事实：** `STM32H757/Smart_Car_H757.ioc:15-16` 为 CM4/M7 列出 FreeRTOS、OpenAMP、watchdog、DMA 等多项 IP；CM4 当前 CMake 在 `CM4/mx-generated.cmake:20-47` 只纳入启动、基础 Core 和部分 HAL，没有 FreeRTOS、OpenAMP、IPCC、watchdog或应用源。
- **机制/影响：** IOC、生成源清单和手工维护源码表达了三种不同状态。未来 CubeMX 再生成可能引入未审查组件或覆盖当前手工集成，双核计划也可能错误假设 OpenAMP 已可用。
- **建议：** 实施前建立 IOC-to-CMake 资源清单和受保护 USER CODE 边界；先决定采用轻量共享 mailbox/HSEM 还是 OpenAMP，禁止两套 IPC 同时成为权威。
- **验证：** 在独立分支导出 CubeMX 生成差异；对 CM4/M7 CMake 源集、IRQ、linker section 和 middleware 配置做机器可读一致性检查。

### 5.3 `STM-DUAL-003`：没有跨核内存和 cache 合约

- **等级/置信度：** Medium / High；当前单核为潜伏设计缺口，双核启用前为阻断项。
- **确定事实：** CM7 linker 只有 `.dma_buffer` 和 `.noinit` 特殊段（`CM7/stm32h757xx_flash_CM7.ld:214-231`）；CM4 linker 没有共享段（`CM4/stm32h757xx_flash_CM4.ld:37-47` 及完整 `SECTIONS`）。项目代码没有共享结构版本、生产者/消费者所有权、cache clean/invalidate 或内存屏障协议。
- **机制/影响：** CM7 开启 D-cache，而 CM4 当前关闭 cache。未经定义地共享普通 SRAM 会产生陈旧数据、撕裂快照、伪共享和复位后 ABI 不匹配。
- **建议：** 用固定大小、版本化、单向 mailbox；每个方向单写者，包含序号、长度、时间戳、CRC/commit 字；共享区要么由 MPU 设为 non-cacheable/shareable，要么严格执行 cache line 对齐和 clean/invalidate。禁止跨核共享指针或 HAL handle。
- **验证：** cache 开/关组合测试、随机复位、并发序号/CRC 压测、总线负载与 IPC 最坏延迟测量。

### 5.4 `STM-DUAL-004`：D2 SRAM 地址别名造成潜在 DMA 覆盖

- **等级/置信度：** High / High；只在当前 CM4 真正释放并执行其启动初始化时触发。
- **确定事实：** CM7 linker 的 `.dma_buffer` 位于 `0x30000000`，现有 Debug map 显示大小 `0x200`；CM4 linker 将整块 RAM 从 `0x10000000` 起放置普通 `.data/.bss`。器件头文件同时给出 `D2_AXISRAM_BASE=0x10000000` 和 `D2_AHBSRAM_BASE=0x30000000`。现有 CM4 Debug map 的 `.data/.bss` 实际占用 `0x10000000-0x10000030`。
- **机制/影响：** 这两个地址是同一 D2 SRAM 的不同访问别名。若 CM4 被释放，启动阶段对 `.data/.bss` 的初始化可能写入 CM7 UART DMA 缓冲的前 0x30 字节，进而造成帧损坏、丢包或链路恢复异常。当前 CM7-only 路径不触发该条件，不能据此声称现在线上已发生故障。
- **建议：** 双核实施前按物理 SRAM 区间重新划分 CM4、CM7 DMA 与共享 mailbox；对两个 linker、启动清零范围、MPU 和 cache 属性做同一份机器可检查的内存表。发布门必须拒绝任何物理区间重叠。
- **验证：** 以最终双镜像 map 做别名归一化后的区间检查；在 CM4 启动、复位和 cache 开/关组合下观察 DMA 内容、CRC 和 UART 解析计数。

## 6. STM32H757 审计

### 6.1 启动、链接和双核现状

当前 CM7-only 架构与链接图已确认，核心结论见第 4.2 节和 `STM-DUAL-001..003`。CM4 `CMakeLists.txt:17-19` 技术上声明 `add_executable`，因此“README 说只做结构检查”不是编译器强制；但本次没有运行 CM4 构建，不能声称该目标目前成功产出可烧录 ELF。

第 12 轮双核建议必须基于上述前置门，不把 IOC 中存在 OpenAMP/FREERTOS 名称误当成已经集成。

### 6.2 任务调度与实时性

#### 6.2.1 当前任务表

| 任务 | 优先级 | 栈 word | 调度方式 | 关键依赖/备注 |
|---|---:|---:|---|---|
| `attitude_gate` | 3 | 256 | `vTaskDelayUntil`, 20 ms | 姿态/传感器生命周期门；门未开时反复强停 |
| `imu_task` | 2 | 512 | `vTaskDelayUntil`, 10 ms | 调用 boot manager、LSM303 更新、恢复和发布 |
| `s3_service` | 2 | 512 | `vTaskDelay`, 1 ms | 解析 STM-S3 链路；链路锁等待最多 20 ms |
| `motor_board` | 2 | 384 | `vTaskDelay`, 1 ms | 清空可用 MotorBoard 帧并运行控制序列 |
| `imu_data_logger` | 1 | 512 | `vTaskDelayUntil`, 10 ms | 10 ms 轮询，周期性 telemetry/日志 |
| `uart_link` | 1 | 384 | `vTaskDelay`, 1 ms | DMA 恢复与栈监测；RX 主路径在 ISR/ring |
| `logger` | 1 | 384 | 队列接收，250 ms | 唯一调用 `rtos_health_sample()` |
| `bmi323_task` | 1 | 384 | ODR 相位（正常 200 Hz） | 双 IMU 初始化成功后才创建；竞争 SPI driver lock 时丢样本 |
| `imu_lsm_init` / `imu_bmi_init` | 1 / 1 | 各 384 | 通知闸门 | 初始化期间临时创建，完成后删除 |

稳态应用任务栈请求为 `3328 words = 13312 bytes`；初始化临时任务使峰值再增加 `768 words`。`configMINIMAL_STACK_SIZE=256` 的 idle task、TCB、FreeRTOS queue/mutex 和 allocator 元数据另计，32 KiB heap 的实际峰值只能由 map/运行时测量确认。

#### 6.2.2 调度结论

1. `FreeRTOSConfig.h:13-34` 使用 1 kHz tick、最多 7 个优先级、动态 heap 32 KiB、栈溢出和 malloc hook；运行时统计、trace 和 queue registry 关闭。
2. `rtos_health.c:27-35` 的监测名单含不存在的 `protocol`，却没有 `attitude_gate` 与 `motor_board`；`rtos_health_sample()` 对缺失任务不产生明确状态。详见 `STM-RTOS-001`。
3. `logger`（优先级 1）是唯一采样者（`log_service.c:50-95`），因此采样延迟本身可能被误认为所有任务健康；没有 CPU time、deadline miss 或进度戳。详见 `STM-RTOS-002`。
4. S3、UART 和 MotorBoard 任务在可变工作量之后使用相对 1 ms 延时；MotorBoard 还以无静态上界的 `while (MB_Protocol_Poll())` 清空突发帧。详见 `STM-RTOS-003`。
5. `imu_task`/姿态门使用绝对周期，BMI323 任务用 1 kHz tick 上的相位累加近似 200 Hz；这是合理的源码意图，但没有目标板 jitter/WCET 证据。HAL tick、FreeRTOS tick 与 DWT 时间读取并用，详见 `STM-RTOS-005`。

#### 6.2.3 调度建议（不属于本次代码修改）

- 先建立任务 admission 表和关键任务进度监测，再讨论提高优先级或迁移 CM4；不能用“多一个核”掩盖未量测的单核 deadline。
- 事件/通知优先于 1 ms 空轮询；若保留轮询，应设每轮最大帧数和 overrun 计数，并保持现有 zero-PWM、BUS_OFF、姿态门和链路超时保护。
- 运行时验证至少采集任务 release/finish 时间、CPU 利用率、栈最小余量、heap 最小余量、ISR 延迟、环形缓冲溢出和命令端到端延迟。

### 6.3 BSP、驱动、中断和 DMA

#### 6.3.1 资源所有权矩阵

| 资源 | 当前拥有者 | 初始化/数据路径 | 主要风险 | 双核结论 |
|---|---|---|---|---|
| USART2 PA2/PA3 + DMA1 Stream0 | CM7 UART link | `uart_link.c` DMA RX、IRQ/ring、SCBP service | IRQ 内 HAL 重装；D2 cache/别名需保持 | 必须留在 CM7 或定义 mailbox 代理 |
| USART6 PC6/PC7 | CM7 MotorBoard | 寄存器 RX/TX ring + `motor_board_task` | 无背压，突发丢字节；IRQ 批处理无上限 | 迁移前先固定唯一 owner |
| USART1 PA9/PA10 | CM7 调试/日志 | HAL blocking TX/RX、`bsp_uart` | `bsp_gpio_init` 重配 PA10 | 不宜与传感器脚位共享 |
| SPI1 PA5/6/7 + PC4 CS | CM7 BMI323 | HAL blocking SPI + 软件 CS | BSP 无统一锁；忙等待 | 传感器实时链应留 CM7 |
| I2C4 PD12/PD13 | CM7 LSM303 | HAL blocking probe/read/write | 公共 BSP 无锁；write/read 事务语义未显式化 | 迁移需定义总线代理 |
| TIM1/TIM2 encoder | 当前生成初始化，应用读取路径有限 | CubeMX encoder handles | 采样/溢出和所有权缺少统一说明 | 不作为首批迁移对象 |
| TIM3 PWM | 退役/兼容接口 | `bsp_pwm` 仅允许 CH3/CH4 | 枚举可诱导误用 | 保持 CM7 资源禁用 |

#### 6.3.2 关键发现

`STM-BSP-001` 是当前最明确的底层所有权缺陷：USART1 初始化配置 PA10 为 AF7，而 BMI323 初始化链随后调用 `bsp_gpio_init()` 把同一脚设置成普通输入。该结论来自当前调用顺序和 GPIO 寄存器配置，不能用“USART1 只发日志”来消除，因为接口仍宣称 TX/RX 且故障导出/调试接收会受影响。

`STM-BSP-003` 和 `STM-BSP-004` 共同说明 UART 数据路径的实时边界尚未形成闭环。UART2 使用 DMA 和 cache 对齐是正确方向，但回调中执行 HAL 重装是否被当前 HAL 版本保证为 ISR-safe 尚未有证据；USART6 则在 IRQ 侧排空输入，ring 满时只计数丢字节。两者都需要目标板注入测试，不能从静态代码推导实际误码率。

SPI/I2C 访问目前由传感器生命周期的调用顺序隐式串行。BMI323 manager 有 driver mutex，但它不构成 BSP 公共接口的通用合同；后续若将诊断、校准或服务任务拆到 CM4，必须先定义每条总线的单一 owner、CS/重复起始语义和 cache/IPC 边界。

`STM-BSP-005` 影响效率而非直接安全逻辑：BMI323 CS setup/hold 和 reset settling 使用忙等待，初始化任务期间会占用一个 CM7 时间片。应将其作为 WCET/启动预算项，运行时用 DWT/逻辑分析仪测量，而不是凭常数估算 CPU 利用率。

### 6.4 传感器、姿态、控制与应用安全

#### 传感器与姿态数据流

CM7 的传感器链为 `BMI323/LSM303 -> imu_manager -> DualAHRS -> attitude_gate -> MotorBoard`。现有证据确认：DualAHRS 上下文由多个路径更新、重置和读取，但没有统一锁或版本化快照；活动 BMI323 路径与 dormant driver 同时存在，陀螺量程和换算常量存在契约漂移风险；启动 fast-zero 路径可绕过 500 样本累计；高速标定在驱动锁竞争时会丢样本。

这些问题的共同影响是数据可能“有值但不一定同一时刻、同一配置或满足质量门”。建议后续采用单写者双缓冲/序号快照，统一 BMI323 权威实现和量程读回，并把标定样本数、锁竞争、运动拒绝和 freshness 编入状态。当前未做传感器拔除、振动、量程寄存器读回或并发恢复注入。

#### `STM-SAFE-001`：姿态 ready 后缺少 freshness 撤销

- **确定事实：** `attitude_startup_coordinator.c:57-91` 在启动阶段检查生命周期和 `update_count`，`:132-146` 置位 ready 并启动 MotorBoard；ready 分支之后只按周期延时，未继续检查 IMU 样本年龄/序号。
- **机制/影响：** 传感器停止更新、总线卡住或 AHRS 快照陈旧时，启动门仍保持已通过；MotorBoard 的 BUS_OFF、链路超时和 forced-stop 保护不能证明姿态数据仍新鲜。这是静态机制风险，不等同于已发生失控。
- **建议：** 将有效状态、样本年龄和序号纳入每周期输出准入；超限立即 zero-PWM，恢复重新经过稳定门。
- **验证：** 运行中冻结 update counter、拔除 BMI/LSM、注入 AHRS 降级，测量故障到 PWM 归零的最大时间及重新 arming 条件。

#### 应用层和控制闭环

命令桥在 S3 侧验证 App payload、映射 SCBP message ID 后才送入 CM7；CM7 保留姿态、链路、BUS_OFF、MotorBoard 和 zero-PWM 权威。MotorBoard 任务同时同步发送轮速/电源遥测，可能把 UART 锁等待和发送耗时带入控制循环。该路径保留现有 `[RR, RF, LR, LF]`、底盘几何、PWM 和 stop 约束，审计没有建议改接口或符号。

`STM-ARCH-001` 的反向依赖来自 BSP/驱动直接调用 service/log 接口，使底层难以脱离 S3 服务做替身测试，也增加初始化失败时的阻塞面。建议将底层事件降为计数/轻量事件，由 service 统一消费。

## 7. ESP32-S3 审计

### 7.1 启动、任务与资源

`ESPS3/main/main.c:34-95` 的顺序是 NVS、STM UART2、BLE、雷达 UART/PWM/uplink、SmartCar service；NVS 失败直接返回，STM UART 失败仅记录并跳过 service，BLE/雷达失败也没有统一 rollback 或总的 degraded 状态。因此 `S3 SYSTEM READY` 日志不能单独证明 STM 链和雷达链都可用。

STM UART 任务在 `components/stm_uart/stm_uart.c:11-18,62-90,148-161` 使用 4096 字节驱动缓冲、4096 字节应用环、3072 word 栈、优先级 9 和 100 ms 读取超时。应用环按字节保留新数据，溢出时丢弃旧字节；这对日志流可接受，但对 SCBP 帧会造成帧中间截断。发送由 `s_tx_mutex` 串行化，并在 `stm_uart.c:167-210` 同步等待发送完成，调用方会受到固定超时影响。

`command_bridge.c:30-75,491-543,614-621` 建立静态 BLE RX 队列深度 8，每轮最多消费 4 项，并以任务轮询同时处理 BLE、STM UART、ACK tick、标定和雷达状态。队列满时记录 drop；没有把“命令必须送达”和“遥测可丢弃”分离成独立调度预算。

### 7.2 BLE

`s3_ble.c:189-218,342-395` 将 ready 定义为 connected 且 TX CCC 已开启，断连时清除 ready 并回调 service；RX 回调只在锁内复制函数指针，要求下游同步把 GATT buffer 放入队列。MTU 在 `:391-394` 更新，通知在 `:447-474` 按 `MTU-3` 分片调用 `esp_ble_gatts_send_indicate`，错误只递增失败计数并立即返回；没有独立通知队列、拥塞窗口或重试策略。日志另有 FFE3 pending ring（容量 48），满载时返回 `ESP_ERR_NO_MEM`。

这形成 `S3-BLE-001`：数据分片和回调状态是明确的，但高频姿态/日志上行会与同步发送竞争，实际拥塞、通知顺序和丢包率只能通过 BLE MTU/CCC/拥塞测试确认。断连 stop 回调已存在，仍需测量从 GATT 事件到 CM7 zero-PWM 的上界。

### 7.3 雷达与 Wi-Fi/TCP

雷达 UART 在 `main/radar/radar_uart.c` 通过事件队列、parser、frame FIFO 和 mutex 分层，FIFO 满时按“丢最旧帧”策略；parser 对帧头、长度、校验和状态做边界检查。`radar_telemetry_queue.c/.h` 将 wheel 作为有限 FIFO、姿态和两路 IMU 作为单槽最新值，并设置 wheel burst 上限 4，所有 push/pop 由上层 mutex 串行化。该设计改善了资源上界，但存在多级复制和不同流的丢弃语义，必须在报告/上位机端解释计数。

`radar_uplink.c:39-58,314-336,345-418,650-760` 使用事件组管理 Wi-Fi 连接/凭据轮换，TCP connect timeout 500 ms、重试退避 500 ms 到 10 s、发送前 `select`，任务栈 4096 word、优先级 4、等待 100 ms、每轮最多 4 帧，并丢弃超过 500 ms 的雷达帧和超过 1000 ms 的遥测。`S3RD` envelope 的版本、长度和 CRC 有编码/解码检查；协议注释明确 telemetry message type 仍是 experimental，不能宣称 Windows/ROS2 已冻结或已产生 `/scan`。

### 7.4 S3 发现

| ID | 等级 | 静态结论 | 后续验证 |
|---|---|---|---|
| S3-BOOT-001 | Medium | 部分初始化失败无统一降级状态；READY 日志可能早于完整能力可用 | 注入 NVS/UART/BLE/Wi-Fi/雷达失败，检查服务准入和恢复 |
| S3-UART-001 | Medium | UART2 应用环按字节溢出，可能截断 SCBP 帧 | 最大突发、高日志负载下测 ring 高水位、CRC/恢复 |
| S3-BLE-001 | Medium | 通知同步分片，无拥塞队列；失败有计数但无重试 | MTU 23/247、CCC 开关、拥塞/断连和时序捕获 |
| S3-SVC-001 | Medium | BLE RX 深度 8、每轮预算 4，命令与遥测共用服务周期 | 持续命令/遥测压力下测 drop、延迟和 stop 上界 |
| S3-RADAR-001 | Medium | 雷达多级 FIFO/队列有界但复制和丢弃语义不同 | UART 突发、FIFO 满、Wi-Fi 断线恢复和计数一致性 |
| S3-RADAR-002 | Medium | TCP 有界重连/发送，但 S3RD telemetry ID 尚未联合冻结 | 与 Windows/ROS2 联测、版本/CRC/序号和延迟 |

## 8. 跨芯片协议与状态机

### 8.1 两层协议

App-S3 层使用 `AA 55` 头、版本 `0x01`、16-bit little-endian 长度、CRC16 和 `0x55` 尾（`ESPS3/components/smartcar_protocol/include/app_parser.h:7-24`，`app_parser.c:35-111`）。STM-S3 层当前构建使用 `Common/SCBP_CAN`：`5A A5` 头、ID/flags/length/HCS/sequence、payload CRC16 和 `0D 0A` 尾（`scbp_protocol_defs.h:10-17`，`scbp_parser.c:64-128`）。两层职责不同，但文档、测试和抓包必须明确封装边界，不能把历史 `Common/SRP` 当成当前构建事实。

### 8.2 ACK、重试和 BUS_OFF

`scbp_link.c:139-207,226-269,281-333` 对需要 ACK 的发送保留 4 个 pending slot，默认 ACK timeout 500 ms、最多 3 次重试；ACK 同时匹配原始 CAN ID 和 sequence，BUS_OFF 时释放 pending、调用回调并由 command bridge flush UART、延迟恢复。解析器在 HCS/FCS 错误时重置并回放可能的帧头后缀，长度/节点/保留 flag 有边界检查。

`X-STATE-001` 的风险不在 CRC/ACK 编解码本身，而在状态机分散：BLE ready、S3 service ready、SCBP link active/BUS_OFF、STM boot-ready、雷达 PWM ready 和 CM7 attitude ready 由不同模块维护。任何一方恢复时，另一方可能仍持有旧 pending 或旧数据。建议后续建立端到端状态表，明确每个状态的 owner、有效期、序号、清零动作和安全结果。

`X-PROTO-002`：4 个 ACK 槽与 4 项重试是明确资源上界，但突发控制命令可能得到本地 busy，超时由 1 ms service tick 驱动；必须证明 busy/timeout 被 App 映射为拒绝而不是继续运动。当前无跨设备抓包，不能证明重连期间序号、ACK 顺序和重复帧行为。

## 9. 应用层独立性与可维护性

当前依赖方向大体为 `HAL/IDF -> BSP/transport -> driver/protocol -> service -> app`，但存在三类反向/隐式耦合：

1. STM BSP/驱动直接调用 S3 service/log，底层不能独立链接测试（`STM-ARCH-001`）。
2. DualAHRS、链路计数、BLE ready、标定 ready 和 telemetry sink 由多个模块持有全局/静态副本，缺少统一版本/生命周期。
3. S3 命令桥同时承担 App 解码、SCBP 发送、雷达标定、BLE 断连 stop、遥测分发和 BUS_OFF 恢复，服务任务成为多种实时等级的汇合点。

现有公共头文件已对 telemetry sink 的“回调期间有效、必须复制、不得阻塞”作出约束（`smartcar_service.h:9-18`），这是可维护性基础；`radar_telemetry_queue` 也用调用者提供的存储避免隐藏分配。后续建议以接口级 ownership 表、单向事件、版本化快照和独立 test target 继续拆分，保持现有 ID、线序和 stop 权威不变。

## 10. 效率与资源

### 10.1 已知预算

| 区域 | 源码可见预算 | 解释 |
|---|---:|---|
| CM7 应用任务栈参数 | 3328 words，约 13.3 KiB | 不含 TCB、queue/mutex、idle 和 allocator 元数据 |
| CM7 动态 heap | 32 KiB | 创建失败有局部处理，但无统一 admission |
| S3 STM UART | RX/TX driver 各 4096 B，应用环 4096 B | 溢出按字节丢弃 |
| S3 BLE RX | 静态队列 8 项 | 每轮最多处理 4 项 |
| S3 雷达 uplink | 任务栈 4096 words；wheel FIFO 32 | 另有姿态/IMU 单槽和 parser/FIFO |
| SCBP ACK pending | 4 项，每项保存最大帧 | 500 ms timeout，最多 3 retries |

### 10.2 效率判断边界

CM7 的 1 ms 相对轮询、MotorBoard 无静态批处理上限、BMI323 DWT 忙等待和同步遥测发送是可审计的效率风险；S3 的队列复制、BLE 分片和 TCP 发送也会增加 CPU/内存带宽。当前没有运行时 CPU load、WCET、栈高水位、heap minimum、DMA inactive、IRQ latency 或链路吞吐数据，因此不报告百分比利用率、不声称“性能足够”。

建议的最小测量集：关键任务 release/finish 时间、ISR 脉宽、栈最小余量、heap 最小余量、队列高水位/丢弃、SCBP 端到端延迟、BLE notification failure、Wi-Fi send timeout 和雷达 dequeue age。

## 11. 安全、故障恢复与验证矩阵

### 11.1 当前保护

当前源码已经存在若干保护：CM7 姿态启动门和 zero-PWM、MotorBoard forced-stop/协议超时、SCBP BUS_OFF 计数/回调/恢复、S3 BLE 断连 stop、App/SCBP 长度与 CRC 校验、雷达 parser/FIFO 边界、S3 Wi-Fi/TCP 有界重试。它们是已有设计，不等价于已通过硬件验收。

### 11.2 闭环缺口

`SAFE-REC-001`：传感器 freshness、BLE 断链、SCBP BUS_OFF、CM4 失联和雷达/网络过期数据尚未由一张系统级安全状态机统一仲裁。尤其需要确认：

- BLE 断连回调到 CM7 zero-PWM 的实际最大延迟；
- UART2 ring/SCBP 丢帧是否一定导致命令拒绝或链路恢复，而不是保留旧目标；
- CM7 attitude ready 后 IMU 停止更新是否在一个控制周期内撤销输出；
- BUS_OFF/重连期间旧 ACK、旧 telemetry、旧 calibration 状态是否被清零；
- CM4 卡死/复位是否绝不影响 CM7 执行器停机权。

### 11.3 分层验证矩阵

| 阶段 | 操作 | 通过标准 |
|---|---|---|
| 静态 | 反查 High 项路径、owner、长度/序号和建议边界 | 行号、触发条件和事实/推断标签一致 |
| 主机 | SCBP/App/radar codec、队列和错误注入 | CRC、长度、重同步、FIFO 和 drop 计数符合契约 |
| 固件构建 | 在独立构建目录验证 CM7/S3/CM4 产物 | 本轮未执行；不得把旧产物当本轮证据 |
| 目标板 | 传感器拔除、UART 突发、BLE 断连、BUS_OFF、CM4 复位 | 输出进入 zero-PWM，计数/状态可追踪，恢复需重新准入 |
| 链路 | UART/BLE/Wi-Fi 抓包和压力测试 | ID、长度、CRC、ACK、序号、时限和重连一致 |
| 车辆 | 抬轮、低速、紧停、失联和姿态故障 | 方向/轮序正确，任何故障都可确定停机 |

## 12. STM32H757 双核最优拆分

### 12.1 推荐结论

在当前证据下，最优不是立即把高频闭环拆到 CM4，而是“阶段 A 继续 CM7-only，阶段 B 让 CM4 先做低风险服务”。候选 B 的静态评分最高：CM7 保留传感、姿态、标定、运动控制、MotorBoard、S3 UART 最终准入、BUS_OFF/超时/zero-PWM 和所有安全状态；CM4 只承载日志格式化、低频 telemetry 聚合、非安全协议预解析和诊断统计。

CM4 不得直接写 PWM/MotorBoard，不持有 CM7 HAL handle/DMA owner，不成为 attitude/arming/stop 的权威。高频 `IMU -> DualAHRS -> attitude_gate -> control`、UART DMA IRQ owner 和最终停机权都不迁移。

### 12.2 迁移前阻断项

1. 完成 CM4 boot-ready/heartbeat/no-op；修复 HSEM 释放、异常/看门狗回退和启动状态机。
2. 以物理地址归一化检查 CM4 `.data/.bss` 与 CM7 D2 DMA 区，消除 `0x10000000`/`0x30000000` 别名重叠。
3. 选择唯一 IPC 方案，定义固定大小、版本化、单写者 mailbox；字段至少含 magic/version/sequence/length/timestamp/valid/CRC/commit，并定义 cache/MPU/DMB/DSB 规则。
4. 建立外设、DMA、IRQ、时钟域和 RAM owner 表；CM4 只消费快照，不共享指针或 HAL handle。
5. 建立 CM7 对 CM4 的 age timeout：消息过期立即丢弃，CM7 继续单核安全运行。

### 12.3 分阶段路线

| 阶段 | CM4 内容 | 验收门 | 回退 |
|---|---|---|---|
| A | 不启用 CM4，基线 CM7-only | 保留现有构建/设备证据 | 永久可回退 |
| B | CM4 no-op + heartbeat | 冷启动、单核缺失、卡死、复位注入 | CM7 忽略过期 heartbeat |
| C | 日志/低频 telemetry/诊断统计 | mailbox 序号/CRC/cache、CPU/栈/延迟压力 | 编译开关回到 CM7 本地 |
| D | 非安全协议预解析 | 乱序、重复、过期、IPC 满载 | CM7 丢弃预解析结果 |
| E | 任何更高频迁移 | 只有在 WCET/抖动/故障隔离达标后评审 | 不满足即禁止 |

### 12.4 双核设计硬约束

CM7 始终是执行器和安全最终仲裁者；CM4 故障只允许减少诊断/遥测能力，不能改变 stop/arming 结果。每个 mailbox 消息必须有生产者、消费者、最大频率、有效期、序号和丢弃动作；共享 RAM 必须有物理区间、cache 属性和发布前自动 overlap 检查。

## 13. 12 轮审计与报告修订记录

| 轮次 | 主题 | 本轮报告修订 | 结论状态 |
|---:|---|---|---|
| 1 | 范围、目录、构建入口、工作树基线 | 建立报告结构、覆盖口径、证据限制和待审章节 | 完成 |
| 2 | 架构、启动、链接、存储、双核现状 | 补充当前核部署、内存图、3 项双核阻断发现和快照边界 | 完成 |
| 3 | STM 调度、同步、时间基准 | 补充任务表、资源预算、健康监测缺口、轮询漂移和 D2 别名风险 | 完成 |
| 4 | STM BSP/驱动/中断/DMA/外设 | 补齐 pin、总线、DMA、ring 与资源矩阵 | 完成 |
| 5 | STM 中间件/传感器/标定/控制 | 补齐 DualAHRS、BMI323、标定和闭环风险 | 完成 |
| 6 | STM 应用/安全/恢复/独立性 | 补齐 freshness、反向依赖和遥测阻塞 | 完成 |
| 7 | S3 调度/通信/内存/雷达 | 补齐启动、BLE、UART、雷达、Wi-Fi 资源审计 | 完成 |
| 8 | 跨芯片协议/状态机/背压 | 补齐 App/SCBP 双层帧、ACK、重试和 BUS_OFF | 完成 |
| 9 | 分层独立性/耦合/接口治理 | 补齐依赖方向、全局状态和测试边界 | 完成 |
| 10 | CPU/RAM/Flash/栈/带宽/日志效率 | 给出源码预算和必须实测的指标 | 完成 |
| 11 | 安全/看门狗/故障恢复/配置 | 汇总已有保护、闭环缺口和验证矩阵 | 完成 |
| 12 | 双核拆分/迁移/终稿一致性 | 给出推荐分工、迁移门、回退和终审声明 | 完成 |

## 14. 当前验证状态

| 验证层 | 本次状态 | 能证明 | 不能证明 |
|---|---|---|---|
| 源码静态 | 完成 | 当前实现路径、配置和潜在机制 | 编译、时序、硬件行为 |
| 构建/主机测试 | 本轮未执行 | 无 | 当前构建是否通过；避免改写产物/缓存 |
| 目标板 | 未执行 | 无 | CPU/栈/IRQ/DMA/复位行为 |
| UART/BLE/Wi-Fi | 未执行 | 无 | 实际链路、吞吐、丢包和重连 |
| 车辆 | 未执行 | 无 | 运动方向、制动、稳定性和安全验收 |

## 15. 代码修改声明

本任务只创建或修改本计划、总报告和专用规划账本，不修改任何 STM32、ESP32-S3、共享协议或其他应用源代码。工作区中同时存在其他任务产生的固件/雷达改动，本报告不将其归因于本审计，也不回滚它们。

## 16. 2026-08-31 当前 SRPv4 五轮增量审计

### 16.1 快照、优先级与证据边界

| 项目 | 当前快照 |
|---|---|
| 分支 / HEAD | `codex/s3-stm-cn-comments` / `f703453727a136d15ff7cacea4530beab6e9c08a` |
| 活动 STM-S3 协议 | `Common/SRP/` 的 SRPv4；两端 live CMake 编译同一 codec/link 源 |
| 工作区 | 大量其他任务的未提交改动；本审计不回滚、不归因、不提交 |
| 审计性质 | 当前源码/CMake/Kconfig/链接脚本的静态复核 |
| 未执行 | 构建、主机测试、烧录、option-byte 读回、UART/BLE/Wi-Fi 抓包、目标板与车辆验收 |

本次没有仅按旧报告继续编号，而是重新核对当前调用图。因此，旧报告中以下结论已被更正：当前 STM-S3 不是 SCBP；S3 BLE RX callback 已注册；当前遥测 relay 不再以空 `imu_bridge_handle()` 为活动路径；姿态 freshness 会持续撤销 ready；BMI323 已成为 DualAHRS primary 生产者，不是旧文档所说的 paused 路径。

### 16.2 第 13-17 轮完成记录

| 轮次 | 复核范围 | 当前结论 |
|---:|---|---|
| 13 | SRPv4、ACK/重试、同步、CM7 启动、控制与安全 | 发现 chassis 任务未启动、直通 wheel 门控不一致、MotorBoard/fatal-stop/姿态快照缺口；SRPv4 主合同已统一 |
| 14 | CM4 启动、D2 内存、RTOS/IPC、owner | CM4 仍是启动壳；HSEM、D2 别名、shared ABI/cache/heartbeat 和 CM4F RTOS 均是启用阻断 |
| 15 | S3 UART、BLE、雷达、Wi-Fi/TCP、资源 | 断连后旧命令可重新解除零速；BLE 授权、RX 不连续、多生产者发送和 TCP deadline 不闭环 |
| 16 | App-S3-STM、ROS2、控制权、失联 | 红色急停当前会被 S3 拒绝且不会停 heartbeat；App 未使用 V2 租约；ROS2 仍保持无控制权的安全默认 |
| 17 | RTOS/栈/调试配置、文档一致性 | 健康任务表、READY 语义、S3 HWM 单位、未跟踪调试头和 canonical 文档均有漂移 |

第 13-14 轮的逐项证据、触发条件、现有保护和验证方法见 [round13-14-stm-current.md](../.planning/stm-s3-full-audit-20260829/round13-14-stm-current.md)。

### 16.3 当前 High 发现

| ID | 域 | 确定事实 | 推断影响 / 边界 |
|---|---|---|---|
| `CUR13-CTRL-001` | CM7 chassis | `chassis_task_start()` 无调用者；`0x06/0x17` 可保存目标并 ACK | 10 ms 运动学/航向任务不运行，与直通 wheel 命令行为分叉；倾向于静止，不是已确认车辆失控 |
| `CUR13-SAFE-001` | CM7 motion gate | 同步后 `WHEEL_SPEED_CMD` 直达 MotorBoard，不查 IMU/姿态 freshness/MotorBoard READY | 姿态门撤销并强停后，新非零命令可清除 forced-stop |
| `CUR13-MB-001` | MotorBoard | 无 WAIT_FEEDBACK/READY、200 ms MSPD watchdog 和动态 dt；固定 `0.05 s` | 反馈停止时不主动 zero-PWM；板端是否自带超时未知 |
| `CUR13-STOP-001` | MotorBoard TX | 强停是 512 B FIFO 尾部的普通消息，满队列可失败 | 旧非零 PWM 可先发，或物理 zero-PWM 根本未入队 |
| `CUR13-FAULT-001` | CM7 fatal path | fault/assert/stack/malloc 致命路径最终关中断并 WFI/死循环，无 IWDG/WWDG/独立 brake | 已到达 MotorBoard 的非零输出是否停止依赖未证明的板端行为 |
| `CUR13-AHRS-001` | DualAHRS | 低优先级 200 Hz producer 分步写 `s_dual`，高优先级 gate/chassis 无锁读多字段和 64-bit 时间戳 | 可读到混合 sample 或撕裂时间戳；需目标板抢占测试证实触发率 |
| `CUR14-BOOT-001` | CM4 boot | CM4 等 HSEM0，CM7 明确不释放；醒后仅 `HAL_Init()` + 空循环 | 当前不是可承载任务的第二核；option bytes 未读回 |
| `CUR14-MEM-001` | CM4/CM7 D2 | CM4 Reset_Handler 在 HSEM 前写 `0x10000000` `.data/.bss`；CM7 DMA 从物理别名 `0x30000000` 开始 | CM4 启动/复位可破坏 UART DMA；醒后 `uwTick` 可持续写别名偏移 |
| `CUR14-IPC-001` | 双核 | 无 shared section、版本 ABI、cache 合同、heartbeat、reset epoch 和 CM4F RTOS port | 任一项未关闭都不允许启用 CM4 产品任务 |
| `CUR15-DISC-001` | S3 BLE disconnect | 断连只置 stop pending，不清 BLE RX 队列/不撤销 V2 session；service 先发零速后消费旧队列 | 断连前已入队非零命令可在 stop 之后再次下发，S3 heartbeat 又可维持 CM7 同步 |
| `CUR15-AUTH-001` | S3 BLE authorization | FFE1 是普通 write 权限，无加密/配对/peer 授权强制；V1 不要求 CCC 或 session | 射频范围内客户端可构造合法 V1 motion；STM 本地门仍是最后保护 |
| `CUR16-STOP-001` | macOS App | 红色急停只发 V1 `CONTROL/STOP`，不清 wheel target/不停 100 ms heartbeat；S3 无 `0x01` 分支并返回 rejected | 按键后没有零速下发，已有非零 heartbeat 还可继续；BRAKE/摇杆回中是另一条有效零速路径 |
| `CUR16-LEASE-001` | App-S3-CM7 | App 固定 V1，未使用 S3 已实现的 V2 session/heartbeat/TTL；S3 独立 100 ms SRP heartbeat | App 主线程卡住但 BLE 未断时，motion 无 deadman 过期，CM7 链路仍可保持活跃 |
| `CUR17-RTOS-001` | CM7 health | 健康表监测不存在的 `uart_link/protocol`，漏 `attitude_gate/motor_board/chassis_task`，日志又漏最后一槽 | 诊断可给出关键任务健康的错误印象，但不直接改变控制输出 |
| `CUR17-CFG-001` | build reproducibility | `Common/SmartCarDebug/` 未跟踪，CM7/S3 已直接 include；只有少数 CMake cache 变量显式转为 compiler define | 当前本地树可见头文件，fresh clone/CI 静态确定缺文件；本轮未构建复现 |
| `CUR17-DOC-001` | canonical docs | 根 MEMORY/status/system/BLE/ROS2/code map 仍混有旧 frame、BMI paused、BLE relay 未连接、ROS2 无入口和不存在的 App V2 shared path | 后续开发可选错协议/运行路径；当前源码优先级已在项目规则中明确 |

### 16.4 S3 中等风险和资源修正

| ID | 当前静态结论 | 验证重点 |
|---|---|---|
| `CUR15-READY-001` | STM UART、BLE、雷达、uplink 或 service 失败后仍可无条件打印 `S3 SYSTEM READY` | 逐项失败注入，READY 改为 capability/degraded 位 |
| `CUR15-RX-001` | 硬件 UART 错误会置 discontinuity，软件 ring/mutex 丢字节只计数，parser 不立即 reset | 输入高于 service 消费能力的 burst，检查 parser reset、sync 和 stop |
| `CUR15-BLE-TX-001` | FFE2/FFE3 多生产者直接分片 send-indicate，无 TX 串行化/拥塞状态机 | MTU 23/247，并发 STM/radar/uplink 日志，检查分片不交错 |
| `CUR15-TCP-001` | connect/backoff/burst 有界，但已连接 socket 持续 WAIT 无应用级 deadline/keepalive | 对端不读不断，检查 pending stale drop 与 socket 重建 |
| `CUR13-SRP-001` | ACK/ERROR 只要长度 `>=4` 即可按 type+sequence 清 pending，不查精确长度/reserved/flag 一致性 | link-level fuzz，覆盖超长 ACK、错 flag/reserved、pending 满和 BUS_OFF |
| `CUR13-IO-001` | USART1 先配 PA9/PA10，TIM1 后把 PA9 改 AF1，BMI GPIO 又把 PA10 改普通输入 | 原理图/IOC 冻结 owner，分阶段读 MODER/AFR 并抓波形 |

S3 资源单位在旧报告中有过时描述。ESP-IDF 当前 `xTaskCreate()` 栈参数按字节使用：STM RX `3072 B`、radar UART `4096 B`、service `16384 B`、uplink `6144 B`。但 service/radar 日志仍把 HWM 标成 `words`，必须按 bytes 解释。CM7 当前预期稳态任务栈参数合计约 `4480 words`（17.5 KiB，不含 TCB/queue/mutex/idle）；未启动的 chassis 任务若接入还会增加 `512 words`，因此必须在 32 KiB heap 上重测 minimum-ever-free，不能仅靠静态加法判定余量。

Kconfig/fresh clone 默认关闭 radar uplink，但当前本机被 Git 忽略的 `ESPS3/sdkconfig` 已开启它；本地 Wi-Fi 凭据头同样被忽略且未在报告中复制其内容。这说明“fresh clone 默认”与“当前本机运行配置”必须分开报告。S3RD telemetry type `2` 仍是实验 ID，不构成已冻结的 Windows/ROS2 产品合同。

### 16.5 App 急停、控制租约与 BLE 断连

`CUR16-STOP-001` 是当前优先级最高的跨端缺陷之一。`ControlModeView.swift:36-40` 的红色按钮调用 `emergencyStop()`；`SmartCarViewModel.swift:258-265` 只发 `send(.stop)`，没有调用 `sendZeroWheelSpeeds()`，也没有清目标或停止 `:314-327` 的 100 ms wheel heartbeat。它最终编成 App V1 `type=0x01, payload=0x01`，而 S3 `app_parser.h:22-37` 和 `command_bridge.c:1275-1399` 没有 CONTROL 分支，会返回 rejected。App 又只对 PID ACK 更新 UI 状态，用户不会从急停按钮获得这个拒绝结果。

已有的 `BRAKE -> emergencyWheelBrake() -> sendZeroWheelSpeeds()`、摇杆回中、App 失焦/隐藏和主动断连都使用零四轮路径，这是可复用的现有保护，但不能证明红色急停按钮有效。

`CUR15-DISC-001` 与 `CUR16-LEASE-001` 使单纯“补一个 App 按钮”不足以关闭安全链：

1. S3 断连回调只置标志；service 下一轮先发 zero，再从深度 8 队列消费断连前的非零命令。
2. V1 没有 session/sequence/valid-for/TTL，S3 的 V2 `500 ms heartbeat + 3 s TTL` 已实现却没有被 App 使用。
3. S3 自己的 100 ms SRP heartbeat 只证明 S3-STM 传输活着，不证明 App 运动意图仍新鲜。

因此验证必须覆盖“非零已入队 -> BLE 断连 -> S3 stop -> 旧队列消费”和“BLE 保持连接 -> App 主线程冻结”。通过标准必须是最后一条运动输出仍为 zero，且旧 session/epoch 任何命令都不得重放。

### 16.6 ROS2 边界复核

当前 `ROS2_WIN/src/s3_ydlidar_bridge` 只创建 `/scan` 和 `/diagnostics` publisher；telemetry type `2` 只解码并累计诊断，不进入 scan 或 odom。源码中没有 `nav_msgs/Odometry`、`/odom`、`/cmd_vel`、控制 subscription 或 `controller_manager`。`bridge.yaml` 默认 `transport: unconfigured`，因此默认不打开串口、不启用 live TCP、不发布扫描。

这是已确认的安全边界，不是缺陷。S3 raw/telemetry 共用 uplink sequence，ROS2 在分类前统一跟踪 sequence，两端语义一致，也不应误报。本机 uplink 配置开启、host 测试或历史 `/scan` 证据都不能升级为当前 S3 -> Windows -> ROS2 实时链验收。

### 16.7 后续最小修复门（本轮未实施）

| 顺序 | 修改位置 | 修改原因 | 最小内容 | 潜在影响 | 验证方法 |
|---:|---|---|---|---|---|
| P0 | App `ControlModeView/SmartCarViewModel`；S3 `command_bridge` | 红色急停无效，断连旧队列可重放 | 按键先停 timer/清目标并发零四轮；断连增加 epoch、清 RX/parser/session，保证 stop 是最后 motion | 改变 App ACK/UI 和 S3 断连时序，需兼容 V1/V2 迁移 | App write-spy + S3 host state test + FFE1/UART2/MotorBoard 抓取最终输出 |
| P0 | App V2 编码/ACK；S3 V1 admission；CM7 motion lease | App 卡住时无 end-to-end deadman | 复用已有 V2 session/TTL；迁移期对 V1 motion 强制短租约；CM7 不用 S3 heartbeat 续期 motion | 影响旧 App 兼容和命令到达时限 | 冻结 App、替换 session、序号重放、TTL 过期；测最后 FFE1 到物理停机上界 |
| P0 | CM7 最终 motion admission / MotorBoard target API | 直通 wheel 绕过姿态门 | zero 始终可达；非零统一要求 sync+IMU+attitude fresh+MotorBoard READY，门撤销后清旧目标 | 改变当前 wheel 直通准入时机 | READY/未标定/stale/BUS_OFF/timeout 状态矩阵，持续非零输入下仍 zero-PWM |
| P0 | MotorBoard task/transport/protocol | 无反馈 READY/watchdog，强停可排队/失败 | 两帧有效 MSPD 后 READY；动态 dt；200 ms 无反馈 zero/清目标；stop 抢占普通 TX | 改变启动和恢复时序，需冻结轮序/符号/trim/几何 | 配置乱序、MSPD 间隔、拔线、板复位、满 TX 队列，测 zero-PWM 最坏时间 |
| P0 | CM7 watchdog/系统安全层 + MotorBoard 硬件合同 | CPU halt 后没有独立停机 | IWDG 仅由全部关键 heartbeat 喂狗；板端 command timeout 或独立 enable/brake | 影响复位策略和硬件验收 | 非零运行注入 HardFault/关中断/饿死/通信断开，测车轮停机时间 |
| P1 | CM7 启动/姿态协调层 | chassis 命令 ACK 但任务未运行 | 在安全门内单次、幂等启动 chassis task；任务创建失败保持 zero | 增加 512 words 栈和 10 ms 高优先级周期 | canonical CM7 Debug、HWM/WCET；`0x06/0x17` 到 `[RR,RF,LR,LF]` 时延和门撤销清零 |
| P1 | DualAHRS publish/read API | 控制和安全读者可获得混合快照 | 单写者 seqlock/双缓冲或短临界区发布 compact snapshot | 增加快照复制和内存屏障，必须控制中断关闭上界 | 高频抢占/sequence 前后一致性，时间戳/flags/state 自洽 |
| P1 | S3 BLE GATT 安全配置 | 任意未配对客户端可写 V1 motion | 在产品策略确认后要求加密/配对/bond/peer 授权，或明确限定 bench-only | 影响 App 连接、配网和售后流程 | 未配对 central、未开 CCC、旧 bond、重连和授权失效测试 |
| 双核前 P0 | CM4 startup/linker/RTOS/IPC | 当前启动、内存和通信基础不成立 | 修正 CPU 属性，物理内存分区，boot-ready/heartbeat/no-op，版本化 mailbox/cache/reset 合同，正确 CM4F port | 会引入双镜像发布和故障注入面 | option bytes、双 map overlap、readelf/vector、CM4 缺失/卡死/复位/cache stress，CM7 始终保持单核安全 |

### 16.8 文档、构建与验收结论

当前 canonical 文档的漂移不应在本轮通过顺手修改源码来“对齐”。后续文档专项应以当前源码为真值，更新 `.codex/MEMORY.md`、`PROJECT_STATUS.md`、`DOCS/architecture/system.md`、`DOCS/esp32s3/ble.md`、`DOCS/ros2/ros2.md` 和 `DOCS/code_map.md`，并保留迁移前结论为明确历史快照。

本轮只完成静态审计文档。在下列证据全部取得前，不得声称“急停已闭环”、“BLE 断连必停”、“MotorBoard 失联必停”、“CM4 可启用”、“ROS2 live `/scan` 已验收”或“车辆 READY”：

- 当前源码的 canonical CM7 `build/Debug` 和 ESP-IDF 构建/主机测试证据。
- 配对 CM7/S3/App 镜像的 FFE1 -> S3 UART2 -> CM7 -> USART6/MotorBoard 抓取。
- 断连队列残留、App 冻结、传感器 stale、BUS_OFF、MSPD 停流、TX 满和 CPU fault 注入。
- 任务 HWM/WCET、heap minimum、ISR/DMA 延迟、BLE 拥塞和 TCP 半开连接实测。
- 架空轮/低速台架上的最终停机时间和恢复后不自动重放旧目标。

增量审计未修改任何 `.c/.h`、Swift、CMake、Kconfig、IOC、linker、startup、生成文件或构建产物；工作区中这些路径的脏改动均属于其他并行工作，本审计没有回滚或提交它们。

## 17. 2026-08-31 第 18-27 轮深化审计

### 17.1 快照和十轮完成记录

| 项目 | 当前证据 |
|---|---|
| 分支 / HEAD | `codex/s3-stm-cn-comments` / `f703453727a136d15ff7cacea4530beab6e9c08a` |
| 源码状态 | 大量并行脏改动；第 18-27 轮对关键文件重读和哈希复核，不回滚/归因 |
| 新证据范围 | 活动构建图、ESP-IDF 5.5.4 UART/GATT 依赖语义、SRP link、motion/MotorBoard/IMU、RTOS/BLE/S3RD/ROS2 |
| 详细证据 | [round18-27-current.md](../.planning/stm-s3-full-audit-20260829/round18-27-current.md) |
| 未执行 | clean build、主机测试、烧录、UART/BLE/TCP 抓取、RTOS 量测、目标板/车辆验收 |

| 轮次 | 独立主题 | 主要新证据/结论修正 |
|---:|---|---|
| 18 | 构建图/可复现性 | 未跟踪调试头；S3 target 未固定；uplink 启用态无 secret 模板；死选项/重复 BSP test |
| 19 | SRPv4 link/对抗输入 | BUS_OFF 无 stop 屏障；retry 失败无上限；ACK/epoch/replay、callback 重入、半帧超时 |
| 20 | UART/DMA/背压/恢复 | S3 TX 入队 `portMAX_DELAY`；旧 TX 跨 recover；CM7 recover/TX 并发；discontinuity/诊断合同 |
| 21 | motion owner/lease | V2 TTL 只防排队过期；connection epoch 未绑定；chassis 零命令不物理停机 |
| 22 | MotorBoard | FAILED 不是非零门；无 MSPD watchdog；固定 50 ms dt 重新打开；无物理优先 stop |
| 23 | IMU/DualAHRS | DualAHRS publish 无原子快照；BMI 无 dynamic health/recovery；BMI 刚体变换合同矛盾；关闭 LSM DRDY 旧误报 |
| 24 | RTOS/资源/watchdog | S3 service 一 tick 实为 10 ms；task WDT 不直接覆盖关键任务；两端 partial-start 无统一 admission |
| 25 | BLE/session/TX | App 物理连接即启用控制；disconnect zero 不等 with-response；GATT 1032 超 IDF 公开 517 上限 |
| 26 | radar/S3RD/ROS2 | live TCP 无认证；S3 pending + host receipt freshness 使旧包变新；单 client DoS；source time 未映射 ROS time |
| 27 | 跨域终审 | 区分 operator/session/transport/feedback lease；给出车辆、发布、BLE、ROS2、CM4 独立停止门 |

### 17.2 第 18-27 轮新增或显著加强的 High 发现

| ID | 确定事实 | 影响 / 证据边界 |
|---|---|---|
| `R18-BUILD-001` | `Common/SmartCarDebug` 未跟踪，两端和 15 个活动源无条件依赖 | clean checkout 构建图静态不闭合；本轮未通过实际 clean build 复现 |
| `R18-BUILD-002` | S3 tracked defaults/CMake 未固定 `esp32s3`，ignored sdkconfig/cache 才保存 target | IDF 5.5.4 无线索时默认 `esp32`，fresh build 可选错芯片 |
| `R19-BUSOFF-001` | S3 BUS_OFF 不取消 motion/session/不发 zero，100 ms 后 recover；shared recover 清 pending 不回调 | 可早于 CM7 200 ms timeout 重连而不强停，且 S3 motion-in-flight 状态可永久卡住 |
| `R19-RETRY-002` | transport retry 只在发送成功时计数，失败每 tick 再试，BUS_OFF 不拦 send/tick | “最多 3 次重试”不覆盖 transport failure；可占用 service 并累加 TEC |
| `R19-ACK-003` | ACK/ERROR 不查精确长度/reserved/flag/status，pending 只用 8-bit type+sequence | 过宽 ACK 可清 pending；旧 ACK 可与回绕后新事务混淆 |
| `R19-REPLAY-004` | ACK-required 重试会重复进入业务，接收端无 replay cache | 实际是 at-least-once；SYS_CONFIG/波特率副作可重复或造成两端分频 |
| `R20-TX-001` | S3 `uart_write_bytes()` 的 IDF TX-ring 路径用 `portMAX_DELAY`；100 ms 只在入队后等物理完成 | TX stall 可无界阻塞唯一 service；RX flush/recover 不取消旧 TX |
| `R20-RECOVER-003` | CM7 blocking TX 持 mutex，worker recovery 不取锁即 abort/deinit/reinit 同一 HAL handle | 活动 TX 可与 deinit 并发；未经目标板并发注入验证 |
| `R21-EPOCH-002` | BLE disconnect/HELLO 不 reset queue/parser/session/inflight，旧命令未绑物理 connection epoch | stop 后可重放旧非零；新 session 可接管前一 owner 的在途状态 |
| `R21-LEASE-003` | V1 无 deadman，V2 valid-for 只在开始发 SRP 前检查，执行后不到期清零 | App 卡住时最后非零目标可被 S3 transport heartbeat 间接长期保留 |
| `R22-STATE-001` | MotorBoard FAILED/未 RUNNING 不拦非零 setter/MSPD PID | 初始化 NACK/timeout 后杂散 MSPD + 目标仍可产生 PWM |
| `R22-FEEDBACK-002` | RUNNING 后无最后 MSPD 时间和 feedback watchdog | 反馈停流后是否物理停机依赖未证明的外部板语义 |
| `R22-STOP-004` | zero 走普通 TX FIFO，TX 成功还绑 RX re-arm；CPU fault 后 IRQ TX 停止 | 满队列/RX fault/HardFault 下无可靠物理优先 stop |
| `R23-SNAPSHOT-001` | DualAHRS 单 writer 与高优先级读者共享无锁多字段/64-bit 时间戳 | freshness/yaw/gyro 可来自混合更新；未经 M7 强制抢占验证 |
| `R25-APP-READY-001` | App 连接回调在 service/characteristic/CCC 前即设 `.connected`，UI 据此启用控制 | 存在无 FFE1 写特征或无 FFE2/ACK 的早期/盲控窗口 |
| `R25-APP-TX-002` | 主动 disconnect 提交 zero `.withResponse` 后立即取消连接，无 `didWriteValueFor`/in-flight queue | zero 可未交付就断链；App transmitted counter 只代表调用尝试 |

上表中的 motion/MotorBoard/DualAHRS 多为对第 13-17 轮的独立交叉验证和触发面扩展；构建 target、SRP replay/BUS_OFF、S3 TX 无界入队、App GATT-ready/with-response 是本十轮新增证据。

### 17.3 Medium 发现、资源修正和旧结论关闭

| 类别 | 当前结论 |
|---|---|
| build | uplink 启用态无 tracked secret template；`SMARTCAR_SCHEDULER_PROBE` 无消费者；生成 CMake 对 BSP test 重复编译；toolchain 版本未锁定 |
| SRP | control liveness 在业务校验前刷新；completion callback 违反不可重入注释；parser 无半帧超时；link-level 主机测试缺失 |
| UART | 两端 software drop/recover 无统一 discontinuity generation；分阶段 init 不完整回滚；多个统计字段永远为 0 |
| MotorBoard | PID 当前固定 `dt=0.05 s`，历史“动态 dt 已完成”对当前树已失效；轮序、193 mm、trim/符号仍冻结 |
| IMU | BMI 持续 read-fail 会被 freshness 锁车，但不进入 dynamic recovery；ONLINE 仍是 init latch；BMI gyro-Z 单轴翻转与“accel/gyro 共用刚体变换”注释冲突 |
| RTOS | S3 service 当前 100 Hz，“1 tick”是 10 ms；task WDT 只自动看 idle task 且 panic 未启；两端任务/资源失败无统一 startup admission |
| BLE | S3 `max_length=1032` 超出 IDF 5.5.4 公开 `ESP_GATT_MAX_ATTR_LEN=517`，而 App 完整帧最大 136 B；BLE init/service ready 提前，无单 TX owner/congestion completion |
| radar/ROS2 | live TCP 无 TLS/对端认证，CRC 可伪造；单未认证 client 可占住 blocking recv；S3 pending 旧包到主机后按新 receive time 通过 stale 门；S3 timestamp 未映射 ROS time |

本十轮明确关闭以下旧误报：LSM303 DRDY 掩码已正确；姿态 freshness 已持续撤销 ready；BLE RX callback/当前 telemetry relay 已连接；SRP `data[7]/data[6]` 确实是 priority/type；CM7 DMA buffer 已放在 DMA1 可达的 D2 SRAM。未关闭的是 CM4 对该 D2 物理区的别名覆盖。

### 17.4 跨域 lease 结论

| 层 | 当前周期/超时 | 实际语义 | 不得代替 |
|---|---|---|---|
| App wheel heartbeat | 100 ms | App run loop 在尝试发 motion | GATT/S3/STM 交付和下游 deadman |
| V2 heartbeat/session | 500 ms / 3 s | V2 client session 活着 | 已执行 command TTL；App 当前只用 V1 |
| S3 service | 1 tick = 10 ms | service 理想调度粒度 | TX 无界入队/WCET |
| S3 SRP heartbeat | 100 ms | S3-STM transport 活着 | App operator-motion lease |
| CM7 S3 timeout | 200 ms | 最近有 codec-valid S3 帧 | 业务有效/受支持的 motion 刷新 |
| MotorBoard response timeout | init 阶段 1000 ms | 初始化响应等待 | RUNNING 后 MSPD feedback watchdog |

终审确认：当前无任何一个信号构成 App -> S3 -> CM7 -> MotorBoard end-to-end motion lease。session、transport liveness、command TTL 和 actuator feedback 必须分层，并且任一失效都清旧目标而不自动重放。

### 17.5 第 18-27 轮后的最小修复/验证门（本轮未实施）

| 优先级 | 修改位置 | 原因 | 最小内容 | 潜在影响 | 验证 |
|---:|---|---|---|---|---|
| 发布 P0 | `Common/SmartCarDebug`、S3 CMake/defaults、uplink secret 模板、CM7 CMake | clean checkout 不闭合/target 可选错 | 跟踪默认头，固定 esp32s3，提供 dummy secret 模板，清死选项/重复 target | clean configure 行为被固定，需维持默认宏不变 | `git archive` 构建两端、预处理宏/target/compile command 比对 |
| SRP P0 | shared `srp_link`，S3/CM7 BUS_OFF/SYS_CONFIG | BUS_OFF 不停、retry 无上限、ACK/replay 不严 | BUS_OFF 先 zero/完成 pending；严格 ACK + epoch/replay cache；transport 尝试硬上限；双端事务 commit | 重连/重试时序改变，旧 peer 需同步 | link host fuzz，BUS_OFF/inflight/ACK 丢失/回绕/重放/切波特率注入 |
| UART P0 | S3 UART TX owner/driver，CM7 UART worker/recover | 入队可无界，旧 TX 跨 recover，CM7 deinit 与 TX 并发 | 单一有界 TX owner + epoch；恢复取 TX owner；两端 discontinuity generation/reset parser | 增加队列 RAM/延迟，stop 必须有优先槽 | TX ring 满、长 TX+RX fault/BUS_OFF/baud switch、半帧溢出，首帧完整/旧 epoch 不后发 |
| motion P0 | App BLE TX/state，S3 connection/session/lease，CM7 final gate | 物理连接早于 command-ready，zero 不等 write response，无 end-to-end lease | commandReady/telemetryReady/sessionReady；with-response 队列；connection epoch；operator lease；CM7 统一非零 token | 影响 V1 兼容、UI 时序和命令刷新频率 | discovery/CCC/write 失败，App 冻结，满队列断连，freshness/BUS_OFF 并发，最后 motion 为 zero |
| actuator P0 | MotorBoard lifecycle/PID/transport，外部硬件 stop | FAILED 可输出，无 feedback watchdog，dt 固定，zero 无优先 | RUNNING+feedback+admission 门，动态 dt，超时清目标，优先 TX stop，板端 watchdog/enable | 启动/恢复/PID 动态改变，轮序/193 mm/trim/符号不变 | 每步 NACK、MSPD 停流、2-100 ms dt、TX 满/RX fault/HardFault，测物理停机上界 |
| IMU P1 | DualAHRS publish、BMI health/recovery/frame contract | 无原子快照，BMI failure 只锁车不恢复，变换注释矛盾 | seqlock/双缓冲 snapshot；BMI dynamic health + 有界 re-init；冻结正交刚体变换或明确例外 | 增屏障/恢复停机；极性不能无实物证据改 | 强制抢占、SPI 持续/间歇故障、六面/三轴正负旋转和 LSM/BMI 同步回放 |
| RTOS/BLE P1 | S3 task supervision、BLE lifecycle/TX/GATT | idle WDT 不等关键任务前进，BLE ready/TX 语义分散 | 关键 progress deadline；单 BLE TX worker/congestion；table+service+CCC 能力位；属性上限回到 IDF/App 合同 | 增队列和监督资源，WDT 阈值过短可误复位 | 卡住各任务，table/service/CCC 失败，MTU 23/247/517、congest/disconnect epoch |
| ROS2 live P0 | S3RD/TCP/ROS time mapper | 无认证，单 client 占用，旧包按收到时间变新，source time 未使用 | 认证 TLS/MAC + allowlist；client/frame deadline；S3 pending age；boot epoch + source/host time 映射 | 增 S3 CPU/RAM/证书和 ROS 时间状态 | 非授权/静默 peer、半开恢复、重放、时钟漂移/回绕，过期包不得进 `/scan` |
| CM4 前 P0 | startup/linker/RTOS/IPC | HSEM、D2 别名、CM4F port、ABI/cache/reset/heartbeat 仍缺 | 继续 CM7-only；先 no-op heartbeat/物理分区/版本 mailbox/单核回退 | 引入双镜像发布和故障注入面 | option bytes、map overlap、CM4 缺失/卡死/复位/cache stress，CM7 不丧失 stop |

### 17.6 终审声明

- 十轮后双核推荐不变：继续 CM7-only，先关闭单核 motion/fault 安全链；CM4 首阶段仍只允许 no-op + heartbeat，不迁移 sensor/UART/MotorBoard/安全 owner。
- canonical 文档仍混有旧 `sc_frame`、BMI paused、BLE relay 未连接、ROS2 无 runtime 和不存在的 shared App V2 路径；本审计只报告漂移，未扩大到修改这些文档。
- 在车辆 motion P0 未关闭前不进入运动验收；在 ROS2 live P0 未关闭前保持 `transport: unconfigured` 或受控 replay/PoC；在发布 P0 未关闭前不将当前脏本机树称为可复现 release。
- 本十轮没有修改任何固件、App、CMake、Kconfig、IOC、linker、startup、生成文件或构建产物；未构建、测试、烧录、抓包或车辆验收。

## 18. 2026-08-31 第 28-37 轮深化审计

### 18.1 快照、边界与十轮记录

| 项目 | 当前证据 |
|---|---|
| 分支 / HEAD | `codex/s3-stm-cn-comments` / `f703453727a136d15ff7cacea4530beab6e9c08a` |
| 源码状态 | 最终锚点 status 151 项；同一 HEAD 下并行源码曾漂移，本审计只维护审计 Markdown，不回滚/归因其他路径 |
| 详细证据 | [round28-37-current.md](../.planning/stm-s3-full-audit-20260829/round28-37-current.md) |
| 结果 | 36 项详细发现：1 Critical、10 High、其余为 Medium/Low 及组合等级 |
| 未执行 | build、host test、烧录、eFuse/option-byte 读取、UART/BLE/TCP 抓取、RTOS/WCET 量测、目标板/车辆验收 |

| 轮次 | 独立主题 | 主要输出 |
|---:|---|---|
| 28 | ABI/序列化 | App type 表、logical packing、S3RD 多份真值、binary32 合同 |
| 29 | 并发/锁/callback | BLE 跨 owner 写 motion、MotorBoard 长 critical、link 重入、BLE epoch snapshot |
| 30 | 时间/回绕 | DWT 多回绕、V2 sequence、boot timing 哨兵与 wrap-safe 路径 |
| 31 | 错误传播/关联 | PID/SYS_CONFIG context、coalesce 结果、V1 ACK、fast response/log 返回值 |
| 32 | 内存/复杂度 | ROS O(n²) resync、payload 上限、App queue/disk、CM7 调试栈 |
| 33 | ISR/DMA/WCET | UART ISR/task critical；FFE3 CCC 对自身 BTC queue 的无限等待回投 |
| 34 | secret/固件/BLE/network | 公开 tracked/产物凭据、secure boot/flash encryption、BLE/App identity、日志隐私 |
| 35 | App/UI/lifecycle | readiness、分阶段反馈、PID transaction、hide/terminate zero |
| 36 | test/CI/release | App 必需源未跟踪、motion/fault 测试空白、无 CI/产物溯源 |
| 37 | 跨域终审 | 根因去重、独立停止门和 CM4 推荐复核 |

### 18.2 Critical：活动凭据已进入公开 GitHub、构建产物与 Git 历史

`R34-SECRET-000` 已通过不显示原值的结构/集合比较确认：

- ignored 活动 `ESPS3/main/radar/radar_wifi_credentials.h` 去除注释后有一组有效字面量；它与 tracked `S3-radartest/archive/legacy_modules_20260610/ESPS3/components/Middlewares/wifi/wifi_credentials.h` 中一组完全相同。
- tracked `S3-radartest/分支项目/ESP1/ESPS3/components/Middlewares/gateway_config/gateway_wifi_credentials.h` 也含有效字面量；两个 tracked 路径合计至少五组唯一凭据对。本文没有保存任何 SSID、password、可逆编码或可用于复原的内容。
- 两个 tracked 路径由初始提交 `6de387a1134e6862a3deb86c86404d26b5161b24` 引入；该提交仍被当前分支、`main` 和多个本地/远端分支包含。
- GitHub 元数据确认仓库 `2195573507-web/Smart_Car` 当前为 `PUBLIC`，公开默认分支 `codex/dual-imu-lifecycle` 包含初始提交；三份 tracked shared header 还重复保存一组非占位 SoftAP 凭据。
- 不输出值的二进制比较确认活动 SSID/password 两个字段都出现在现有 `ESPS3/build/smartcar_s3_gateway.elf/.bin`；旧产物不是当前 release 证据，但必须纳入 secret 事件清理范围。

处置顺序必须是：

1. 在对应网络侧轮换/撤销所有受影响 Wi-Fi/SoftAP 凭据，检查未知客户端和访问日志。
2. 新凭据只经受控 secret injection 使用，不进入仓库、日志、issue、构建报告或 coredump。
3. 扫描 HEAD、all refs、Git objects、远端、clone、CI cache、备份和产物；报告只保存 secret ID/路径/处置状态。
4. history rewrite、删除远端 refs 和强制更新 clone 必须另行授权并协调；它不能替代第一步轮换。本轮没有执行这些破坏性操作。

### 18.3 第 28-36 轮 High 发现

| ID | 确定事实 | 影响与边界 |
|---|---|---|
| `R29-S3-OWNER-001` | BLE disconnect callback 跨上下文直接写 service-owned motion pending struct，注释却称只置事件 | C data race/旧 epoch motion 交错风险；未做强制调度复现 |
| `R31-CONTEXT-001` | PID/SYS_CONFIG 多个 pending 共用可变 callback context，baud 还共用全局目标值 | 乱序 ACK 可归属错 App 命令或切换错误 baud |
| `R33-BLE-SELFQUEUE-002` | FFE3 CCC callback 最多把 48 条日志分片回投自身容量 100 的 BTC queue，满时 `portMAX_DELAY` | 默认 MTU 23 的足够积压可让 BTC task 等待自己，断连 callback 随之停滞；未设备复现 |
| `R34-FW-TRUST-001` | 当前 sdkconfig 未启 secure boot/flash encryption/NVS encryption，tracked defaults 未锁 release profile | 若按该配置发布，镜像真实性/静态 secret 无产品级保护；未读取设备 eFuse |
| `R34-BLE-AUTH-002` | SMP 组件已编译，但项目未设置 security/encryption/allowlist，FFE1 是普通 WRITE permission | 未配对/未授权 central 可尝试写 V1 motion；需实机认证矩阵验证 |
| `R34-APP-ID-003` | 官方 App 只按固定广播名和公开 FFE UUID 选车，无受信 peripheral/bond/certificate challenge | 同名同 UUID 外设可冒充车辆并伪造 ACK/遥测 |
| `R35-READY-001` | App 在 service/characteristic/CCC 前即标 Connected，所有控制只看该状态 | GATT/telemetry/safety 未 ready 时出现盲控窗口 |
| `R35-FEEDBACK-002` | 红色 STOP 仍走 S3 不支持的 V1 type；非 PID 命令无 sequence/stage，计数只表示调用尝试 | UI 不能证明 GATT/S3/STM/MotorBoard 已接受或执行 |
| `R36-APP-CHECKOUT-001` | `BLEManager` 必需的 `SessionLogWriter.swift` 当前未跟踪 | clean clone/archive 缺生产类型，App 发布源图不闭合 |
| `R36-COVERAGE-002` | App 无 test target；S3/CM7 motion、安全门、SRP link fault 主链无项目测试；仓库无 tracked CI | P0 缺陷没有提交级回归门，只能靠人工审计/设备试验发现 |

### 18.4 中等风险、已有保护与旧结论边界

| 类别 | 当前结论 |
|---|---|
| ABI | S3 已转发 0x29/0x2C 而 Swift type 表缺失；`srp_frame/link` logical pointer struct 受 pack(4)；wire 仍显式 little-endian，不直接发送 host struct |
| time | DWT 扩展跨两次以上约 8.95 s wrap 会漏高位；V2 sequence 不支持 32-bit wrap；主流短 deadline/ROS sequence 已使用 wrap-safe 算术 |
| transaction | motion overwrite 无 superseded；V1 ACK 无 sequence/stage；CM7 fast response/log send failure 被折叠；遥测 best-effort 可以丢但必须真实计数 |
| resources | ROS no-magic resync 近 O(n²)，host payload 上限过大；App decode/log/disk queue 与文件无配额；IMU raw-log 调试路径有 768 B 局部 block |
| realtime | USART2 ISR 最坏含 512 B copy/cache/re-arm 且与 USART6 同级；task read 又可在 critical 内复制 128 B；raw diagnostics 会整段关 IRQ；当前 HAL re-arm 状态顺序和 CM7 DMA cache 边界已确认成立 |
| UI | PID sending 可重复提交且无 timeout/sequence；hide/resign 会本地清目标并尝试 zero，但 terminate/主动断连没有 write completion barrier |
| privacy | BLE log 输出完整 remote BDA，uplink error log 输出 SSID；未发现项目代码打印 password |
| tests | radar tests 直接编译生产源、两端共享同一 SRP 是保护；但 codec binary 包含 `srp_link.c` 不等于 link 行为已测试 |

这些结论不改变已冻结的 `[M1:RR,M2:RF,M3:LR,M4:LF]`、`193.0 mm`、`WHEEL_TRIM`、PWM/编码器符号和 sync/attitude/BUS_OFF/emergency-stop 合同。第 28-36 轮也没有提供提前启用 CM4 的依据。

### 18.5 后续最小修改建议（本轮未实施）

| 优先级 | 修改位置 | 修改原因 | 最小内容 | 潜在影响 | 验证方法 |
|---:|---|---|---|---|---|
| S0 | 网络凭据管理、Git/CI/备份治理 | 活动秘密已进入 tracked history | 先轮换/撤销，建立 secret injection/scanner；历史清理单独授权 | 网络客户端需更新；rewrite 会影响所有 clone/refs | 旧凭据拒绝、未知 client 审计、all-ref/artifact scan 不含活动 secret |
| 发布 P0 | S3 release defaults/eFuse 流程、App/SmartCarDebug 版本边界 | secure boot/flash encryption 未锁，必需源未跟踪 | 分离 bench/release；锁签名/加密/target；跟踪经审查的必需源 | eFuse 不可逆，影响 JTAG/RMA/OTA；clean build 输入变化 | 专用样机签名/回滚/加密验证；git archive clean App/S3/CM7 build |
| auth P0 | S3 BLE GATT/GAP、App pairing/readiness | SMP 未形成 command authorization | 定义 pairing/bond/RMA；FFE1 要求 encrypted/MITM/peer admission；command-ready 等认证完成 | 旧 App/设备兼容和换机流程变化 | 未配对、旧 bond、reset、random address、授权撤销全部保持 zero |
| transaction P0 | S3 command bridge/SRP pending、App V2 ACK | callback context 可变、overwrite/V1 ACK 无身份 | 每事务 immutable slot；single in-flight 或 sequence；明确 superseded/timeout | 增 RAM/ACK 通量，需定义 stop 最高优先级 | 同类乱序 ACK、丢 ACK、重放、wrap、BUS_OFF、baud commit fault injection |
| realtime P1 | CM7 uart_link/MotorBoard、S3 BLE log TX | ISR/task bulk work、长 critical、BTC self-queue flush | short snapshot + stop generation；bulk copy/诊断移任务；CCC 只通知有界 BLE TX owner | 增 buffer/descriptor/task RAM，日志排空延迟 | DWT/GPIO 测 ISR/critical；MTU23+48 logs+disconnect 验证 BTC 持续前进 |
| UI P0 | BLEManager/ViewModel/Control/PID UI | Connected/计数/STOP 与真实交付不一致 | link/write/notify/auth/vehicle readiness；有 sequence 的阶段状态；急停清 timer/target走有效 zero | 控件启用延迟，兼容模式需明确 | discovery/CCC/write/S3 reject/STM timeout/supersede/feedback stale 状态矩阵 |
| test P0 | App Tests、S3 host adapter、shared SRP link、CM7 fakes、CI | motion/fault 主链无回归门 | 每个 P0 一个可执行 fault case；CI 记录 commit/toolchain/config/source/artifact hash | 增 test doubles 与构建时间，不应重构冻结接口 | clean archive 自动运行；漏源/漏测硬失败，source/host/build/device 分层报告 |
| resources P1 | ROS assembler/config、App queues/log writer、CM7 debug logging | CPU/内存/磁盘上界不一致 | read offset/ring linear resync；schema max 775；有界 queue/drop/rotation；分段 raw log | 过载时显式丢日志/合并遥测 | 262 KiB junk、慢盘/满盘、长会话、debug stack-usage/HWM |

### 18.6 独立停止门与双核结论

| 门 | 必须取得的证据 | 未通过时状态 |
|---|---|---|
| S0 secret | 凭据撤销、all-ref/artifact/cache 扫描、新 secret injection 审计 | 不再分发含秘密 refs/产物 |
| R0 release | clean archive App/S3/CM7 + host tests + 工具链/config/source/产物 hash | 只称开发快照，不签 release |
| P0 protocol | strict ACK/replay/epoch/context/callback/wrap/fault tests | 不开放产品 motion/动态 baud |
| M0 motion | 有效急停、BLE auth/epoch、operator lease、CM7 final gate、MotorBoard READY/feedback/priority stop/CPU-fault stop | 禁止车辆运动验收 |
| T0 realtime | ISR/critical WCET、task HWM、heap minimum、queue/ring waterline 和长时压力 | 不声明时延/资源余量 |
| N0 ROS2 live | 认证、新鲜度、client/frame deadline、source epoch/time、线性 assembler | 保持 `transport: unconfigured` 或受控 PoC/replay |
| D0 CM4 | option bytes、HSEM timeout、物理内存、CM4F RTOS、mailbox/cache/reset/heartbeat、单核回退 | 保持 CM7-only，CM4 首阶段仅 no-op + heartbeat |
| V0 vehicle | 前置门通过且匹配镜像完成架空轮、低速台架、受控实车签收 | 不宣称 READY/车辆验收完成 |

双核推荐不变：先关闭 CM7-only 单核 motion/fault 安全链；再做 CM4 no-op/heartbeat 和物理内存/IPC/cache 故障注入；最后只评估低风险日志/诊断消费者。传感器、实时 UART、MotorBoard 和最终安全权继续留在 CM7。

### 18.7 终审声明

- 第 28-37 轮详细审计和正式报告已完成；Critical secret 事件已被安全地记录，但外部轮换/远端清理尚未由本审计执行。
- 当前结论来自脏工作树静态审计和 Git 元数据；旧本机 build/bundle、测试文件存在或历史设备日志均未升级为当前通过证据。
- 最终锚点为 2026-08-31 04:48:23 CST；13 个关键非 secret 源文件连续两次 SHA-256 一致，完整 manifest 见详细证据第 12 节。
- 本十轮只修改审计 Markdown；未修改固件、Swift、CMake、Kconfig、IOC、linker、startup、sdkconfig、secret、生成文件或构建产物，未提交或 push。
- 未执行 build/test/flash/capture/target/vehicle，因此可以关闭文档轮次，不能关闭 S0/R0/P0/M0/T0/N0/D0/V0 中需要外部、构建或硬件证据的门。

## 19. 2026-08-31 第 38-47 轮深化审计

### 19.1 快照与十轮记录

| 项目 | 当前证据 |
|---|---|
| 起始分支 / HEAD | `codex/s3-stm-cn-comments` / `f703453727a136d15ff7cacea4530beab6e9c08a` |
| 起始工作树 | status 151项；大量并行脏源码，本审计只写审计Markdown |
| 审计中漂移 | 同一HEAD下增至至少161项并新增chassis state/odometry；带High项按当前源码重核 |
| 详细证据 | [round38-47-current.md](../.planning/stm-s3-full-audit-20260829/round38-47-current.md) |
| 新增统计 | 33项：0新增Critical、2 High、5 High/Medium组合、其余Medium/Low组合 |
| 未执行 | O0/Os build、sanitizer/fuzz、NVS/OTA/掉电/reset、UART/BLE/TCP、目标板/车辆验收 |

| 轮次 | 独立主题 | 主要输出 |
|---:|---|---|
| 38 | compiler/UB | fault alias、ROS窄化、C/C++ header、vendor UB、warning/sanitizer profile |
| 39 | reset/re-arm | peer boot epoch、early stop、reset cause、retained record交付 |
| 40 | persistence/OTA | NVS erase、rollback owner、partition schema、session atomicity |
| 41 | control numeric | min-wheel policy改变applied linear speed |
| 42 | sensor/calibration | timestamp/quality保护、cal-result端到端合同、新odometry时间对齐 |
| 43 | resource lifecycle | STM UART/uplink/service init rollback/commit |
| 44 | parser adversarial budget | MotorBoard ACK correlation、Swift parser容量/复杂度 |
| 45 | diagnostic truth | source freshness、ONLINE/READY、counter/snapshot/log loss |
| 46 | supply chain | immutable dependencies、toolchain、license/SBOM、vendor/generator provenance |
| 47 | closeout | 根因去重、停止门、冻结合同和CM4结论 |

第34轮公开Wi-Fi/SoftAP secret Critical没有被本十轮关闭，仍是第一处置项；本十轮没有读取或输出secret值。

### 19.2 带High等级的新发现

| ID | 确定事实 | 影响与证据边界 |
|---|---|---|
| `R39-RESET-REARM-001` | Sync/BOOT_INFO无peer boot epoch；CM7 reset不必断BLE，App非零heartbeat只看BLE connected | CM7重启zero后可由旧App owner自动恢复非零，未经新授权 |
| `R39-BOOT-STOP-002` | CM7 physical zero在clock/GPIO/USART6之后；VOS busy wait与早期HAL失败可永久关中断hang | 外板若保持上次PWM，早期reset/brownout路径无stop上界；未设备注入 |
| `R41-MIN-WHEEL-001` | heading min-wheel把低侧设±80并给两侧同加偏置，保持差速但改变平均值 | 低速requested v可被逐步放大；单轮1000上限仍保护 |
| `R43-STM-UART-001` | RX task创建失败只清initialized，保留UART driver/event queue/mutex | 直接retry再次install同一port，资源恢复不可闭环 |
| `R43-UPLINK-002` | uplink中后段失败不统一unregister/stop/deinit/destroy network/event资源 | partial network owner与后续模块共存，retry可叠加状态；tracked默认off是保护 |
| `R44-MB-ACK-001` | MotorBoard任意文本含OK/ACK/Set即通用成功，sequence只按类型推进 | 旧/无关/flash配置行可提前推进启动步骤；真实板响应语料待采集 |
| `R45-WHEEL-FRESH-001` | 轮速每50 ms重发最后数组，无source age；App以receipt time刷新 | MSPD停流时旧反馈可持续显示为新鲜，掩盖feedback failure |

### 19.3 关键Medium、保护与关闭项

| 类别 | 当前结论 |
|---|---|
| UB/portability | fault record存在strict-aliasing；ROS参数/索引转型缺域检查；SRP头依赖C扩展；vendor lock UB当前memory路径不可达 |
| reset/evidence | 两端无reset cause/boot id；retained record在log accepted前clear；motion BSS不跨reset，风险来自外板保持与App重发 |
| persistence | NVS两类错误整区擦；双OTA分区无OTA owner/rollback；旧16 MiB表不在活动构建；session日志无partial/footer/rename |
| sensor/cal | BMI/LSM timestamp与quality gate存在；内部cal result无Common/CM7/S3发送合同而App 0x25 decoder孤立；新odometry用wheel source dt与当前无timestamp yaw投影 |
| parser | S3 App/SRP C parser固定容量线性，App log parser有上限；Swift control parser无上限且前删近O(n²) |
| diagnostic | App ONLINE可与sync相反；Radar/TCP READY只代表本地阶段；隐藏counter无epoch；radar invalid snapshot与日志loss不可守恒 |
| supply | IDF lock与YDLidar license/version是保护；ROS image/apt、CM7 compiler、CubeMX/Swift、vendor source与LICENSE/SBOM仍未形成immutable release manifest |

明确关闭/保留：当前CM7 UART DMA line alignment/D2 placement/cache barrier和锁定HAL callback READY顺序保持关闭；CM4 D2 alias仍开放。当前无项目OTA入口，不能把双slot称为在线升级能力，也不能把它描述为现有远程攻击入口。

### 19.4 最小建议（本轮未实施）

| 优先级 | 修改位置 | 原因 | 最小内容 | 潜在影响 | 验证 |
|---:|---|---|---|---|---|
| Motion P0 | S3/CM7 sync、App session、boot/reset | reset后旧owner可re-arm，早期zero无上界 | boot nonce/epoch；epoch变化清所有motion并要求App显式re-arm；最早hardware disable/board watchdog | reset后不自动续航；需App re-arm与硬件合同 | CM7/S3 reset、brownout、VOS/PLL/UART fail，未re-arm前物理输出zero |
| Motor P0 | MotorBoard response state、feedback status | 通用ACK推进；旧wheel receipt伪新 | 精确step/echo相关；feedback source timestamp/valid/age，停流显式stale | 需采集板固件响应和versioned telemetry | 旧/乱序/Set/NOT OK；MSPD停流且BLE活着时UI/ROS变stale |
| Control P1 | chassis heading min-speed policy | 防stall平移改变requested v/accel语义 | 优先缩小angular correction保持平均v，或显式限幅/报告applied v | 低速转向/克服摩擦变化 | 正反向v≈80、w 0..2，比较requested/applied与实物停转 |
| Runtime P1 | S3 STM UART/uplink/service init | partial failure不能安全retry | staged owner/逆序cleanup/commit state；失败不打印global READY | cleanup与event callback时序复杂 | 每stage failure/retry，task/driver/mutex/netif/handler数量守恒 |
| Host P1 | ROS params、Swift parser、Common headers | 转换UB、巨量分配、无界/O(n²)、C++扩展 | double域/size/index边界；read-index+max buffer；C/C++ assert macro | 异常配置硬失败，兼容compiler变化 | boundary corpus + ASan/UBSan + C11/C++17 pedantic-errors |
| Evidence P1 | reset/cal/odom/log/diagnostic schema | 状态和结果缺epoch/provenance/time alignment/loss stage | boot/reset/cal epoch、quality/config identity、timestamped attitude、valid snapshot、loss counters、atomic session completion | 增RAM/history/telemetry/fsync与schema版本 | odometry skew回放；每类drop/reset/cal失败唯一归因，截断session显式incomplete |
| Release P0 | toolchain/container/vendor/license governance | 相同commit无法重建相同输入/产物 | immutable digest/version/hash、YDLidar import manifest/patch、LICENSE/NOTICE/SBOM、partition/OTA policy | 依赖升级需显式维护 | clean rebuild provenance一致，artifact-specific SBOM与link graph对账 |

### 19.5 停止门与双核结论

| 门 | 本十轮新增要求 | 未通过时状态 |
|---|---|---|
| S0 | 无变化：先轮换公开secret并扫描所有refs/artifacts/caches | 不分发含secret refs/产物 |
| R0 | immutable toolchain/image/deps/vendor/generator；NVS/partition/OTA/log；LICENSE/NOTICE/SBOM | 只称开发快照，OTA标unsupported，不签release |
| P0 | boot epoch、精确MotorBoard ACK、bounded parsers、strict C/C++ build | 不开放产品motion/自动re-arm |
| M0 | earliest hardware stop、reset re-arm、applied-v合同、feedback source freshness | 禁止车辆运动验收 |
| T0 | O0/Os diagnostics/sanitizers、parser/resource budget、init failure守恒、valid snapshots | 不声明实时/资源/诊断闭环 |
| N0 | ROS参数硬边界、immutable image、source-time freshness | 保持 `transport: unconfigured` 或受控PoC/replay |
| D0 | 既有CM4门外增加boot epoch/reset cause/earliest stop不退化 | 保持CM7-only；CM4仅no-op+heartbeat |
| V0 | reset/brownout/early-fail、feedback stop、低速heading和不自动rearm实测 | 不宣称READY/车辆验收完成 |

双核推荐不变：先关闭CM7-only单核motion/fault/reset链；再做CM4 no-op+heartbeat/boot epoch；最后仅评估低风险日志诊断。传感器、实时UART、MotorBoard和最终安全权继续留在CM7。

### 19.6 终审声明

- 第38-47轮静态审计正文已完成；33项来自当前脏源码/配置/依赖/Git元数据，不代表运行时已触发。
- 最终锚点为2026-08-31 14:29:33 CST / status 161项；35个非secret关键路径manifest为`1de69c579ecfcfb4dd5d1ba2bf16e4fac9d5d5e1cd9de85589742ab9f8203ce6`，完整清单见详细证据第12节。
- 本轮只修改审计Markdown；未修改源码、App、配置、secret、生成物、构建产物，未commit或push。
- 未执行build/test/sanitizer/fuzz/flash/reset/NVS/OTA/capture/target/vehicle；文档完成不能关闭S0/R0/P0/M0/T0/N0/D0/V0。
