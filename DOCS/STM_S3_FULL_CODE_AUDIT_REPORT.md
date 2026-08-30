# STM32H757 与 ESP32-S3 全量代码审计总报告

> 历史快照说明：本文是 SRP v4 硬切换前的只读审计报告，旧协议描述仅保留
> 作为迁移前证据。当前活动 UART2 合同以 `DOCS/SRP_v4_Spec.md`、
> `Common/SRP/` 和现行源代码为准。

> 本文后续出现的 `SCBP`、`HCS`、`FCS`、旧消息 ID 和旧路径，全部属于
> 迁移前审计快照，已废弃且不构成当前构建输入或运行时合同。

> 状态：静态审计终稿，已完成 12 轮主题复核；未执行构建、烧录、链路和车辆验收  
> 日期：2026-08-30  
> 审计方式：当前工作树只读静态审计  
> 代码修改：无  
> 计划：[STM_S3_FULL_CODE_AUDIT_PLAN.md](STM_S3_FULL_CODE_AUDIT_PLAN.md)

## 1. 执行摘要

本报告审计 Smart_Car 当前 STM32H757 与 ESP32-S3 固件的构建边界、任务调度、底层驱动、中间件、应用独立性、跨芯片协议、效率、安全和故障恢复，并给出 STM32H757 双核目标拆分方案。

本报告是基于当前工作树的只读静态终稿。审计期间没有修改固件、协议、配置或生成代码；本轮按用户要求快速收口，不再新增审计轮次。当前已经确认：

- STM32H757 是最终运动与本地安全权威，ESP32-S3 是 BLE/Wi-Fi 网关和雷达端点；本审计不改变该责任边界。
- 当前入口文档表明 CM7 是主实时核；CM4 现有 CMake 目标只声明为构建结构检查，不应把系统描述成已完成双核运行。
- 受跟踪的 STM 树包含大量供应商、生成和二进制材料；全量覆盖以“当前构建中项目自研代码”为主集合，供应商代码只审计集成面。
- 旧架构/状态文档与当前实现之间可能存在漂移；最终结论只接受当前文件、符号和配置反查。
- 本次不构建、不烧录、不做 UART/BLE/Wi-Fi/车辆验证，运行时指标和物理效果会明确列为未验证。

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
