# STM32H757 当前工作树审计证据：第 13-14 轮

> 审计时间：2026-08-31（Asia/Shanghai）  
> 工作树快照：分支 `codex/s3-stm-cn-comments`，`HEAD=f703453727a1`  
> 证据级别：当前工作树源码静态审计；未构建、未烧录、未读 option bytes、未抓取 UART、未执行目标板或车辆测试。  
> 边界：本轮只新增本 Markdown；没有修改 `.c/.h`、CMake、IOC、链接脚本、启动文件、配置或生成物。

## 1. 审计范围与判定口径

第 13 轮重新审计当前 CM7 活动实现：live CMake、SRPv4、启动顺序、FreeRTOS、传感器/标定/DualAHRS、底盘控制、MotorBoard 和安全停机。第 14 轮重新审计 CM4 启动壳、D2 内存、RTOS/IPC 缺口、外设 owner 和双核启用阻断项。

此前第 1-12 轮是 SRPv4 硬切换前的历史基线。本文件只记录当前工作树；发现冲突时以当前源码为事实真值，并把旧产物、历史设计和运行时推断分开。

证据标签：

- `CONFIRMED_SOURCE`：可由当前源码、CMake、链接脚本或 Git 状态直接确认。
- `CONFIRMED_OLD_ARTIFACT`：只由既有 ELF/map 交叉确认，不能证明当前源码已构建或已烧录。
- `INFERRED_RISK`：触发机制可由源码推导，但是否在目标板发生仍需实时证据。
- `UNVERIFIED_RUNTIME`：必须靠匹配镜像、总线捕获、故障注入或车辆台架确认。

## 2. 当前架构事实摘要

| 域 | 当前确定事实 | 关键证据 |
|---|---|---|
| 活动协议 | CM7 直接编译 `Common/SRP/{srp_crc,srp_wire,srp_codec,srp_link}.c`，活动协议为 SRPv4，不再是旧 SCBP | `STM32H757/CM7/CMakeLists.txt:58-63`；`Common/SRP/include/srp_def.h:27-37` |
| CM7 角色 | CM7 是唯一活动实时核，拥有时钟、外设、传感器、RTOS、S3 UART、姿态门和 MotorBoard | `STM32H757/CM7/Core/Src/main.c:56,148-232` |
| CM4 角色 | CM4 仍是 HSEM/WFE 启动壳；唤醒后只有 `HAL_Init()` 和空循环 | `STM32H757/CM4/Core/Src/main.c:35-45,82-129` |
| S3 UART | USART2 PA2/PA3，921600 8N1；DMA1 Stream0 RX，D2 SRAM 512 B DMA buffer | `uart_link.h:21-24`；`uart_link.c:38-39,290-384`；`stm32h7xx_hal_msp.c:335-353` |
| MotorBoard | USART6 PC6/PC7，115200 8N1；轮序固定 `[M1:RR,M2:RF,M3:LR,M4:LF]` | `motor_board_transport_uart.h:19-21`；`motor_board_task.c:62-68` |
| 底盘几何 | `track=193.0 mm`，right=`v+0.5*w*193`，left=`v-0.5*w*193` | `chassis_kinematics.h:16-24`；`chassis_kinematics.c:21-34` |
| 姿态角色 | BMI323 200 Hz 生产者推进 DualAHRS primary；LSM303 accel/mag 构成 redundant，并且两路初始化/更新共同参与放行 | `imu_manager.c:880-932,1373-1397`；`dual_ahrs.c:938-945`；`attitude_startup_coordinator.c:60-95` |
| RTOS | CM7 为 1 kHz preemptive FreeRTOS、32 KiB heap、动态分配，栈溢出/malloc hook 开启 | `STM32H757/Config/FreeRTOSConfig.h:9-35` |

工作树中 `Common/SRP` 和大量 STM 文件已有用户/并行未提交修改；本轮没有回滚或归因。`Common/SmartCarDebug/smartcar_debug_config.h` 当前存在但整个目录仍是未跟踪项，而 CM7 CMake 已把该目录加入 include path（`CM7/CMakeLists.txt:77-100`）。因此当前本地树可解析该头文件，不代表干净检出也能复现。

## 3. 第 13 轮：CM7 / SRPv4 / RTOS / 传感器 / 控制 / 安全

### STM13-CTRL-001 - High - `chassis_task` 实现存在但没有任何启动调用

**确定事实（CONFIRMED_SOURCE）**

- `chassis_task_start()` 会创建名为 `chassis_task` 的 10 ms、优先级 `idle+3` 控制任务，见 `Application/Chassis/chassis_task.c:22-25,289-537,541-552`。
- 当前 `main()` 只启动 `imu_runtime`、`uart_link`、`s3_service` 和 `attitude_startup_coordinator`；正常镜像没有调用 `chassis_task_start()`，见 `CM7/Core/Src/main.c:216-232`。
- 全 STM 源搜索中，`chassis_task_start()` 除声明/定义外没有调用者。
- SRP `CHASSIS_SPEED_CMD` 会调用 `chassis_task_set_velocity()`，该函数能返回成功并保存目标；`CHASSIS_HEADING_CMD` 也能保存目标，见 `s3_service.c:486-548` 与 `chassis_task.c:194-286`。

**推断风险**

- S3 可收到成功 ACK，但 10 ms 运动学/航向控制任务从未运行，目标不会转换为四轮目标，也不会送入 MotorBoard。表现是“同步、ACK 和日志正常，但 chassis/Target Yaw 不动作”。
- `CHASSIS_SPEED_CMD` / `CHASSIS_HEADING_CMD` 与直接 `WHEEL_SPEED_CMD` 的行为因此分叉；这不是协议或物理链已确认故障，而是当前调用图缺口。

**现有保护**

- 缺少任务会使底盘目标不输出，倾向于静止，不会自动产生非零 PWM。
- `chassis_task_set_velocity()` 和 heading admission 仍检查同步、IMU、姿态 freshness 和 MotorBoard 任务存在。

**建议方案（本轮未实施）**

- 修改位置候选：`CM7/Core/Src/main.c` 的任务启动段，或由 `attitude_startup_coordinator` 在安全放行时建立唯一 chassis 任务。
- 修改原因：把已存在的控制生产者接入活动 RTOS 图，同时避免在姿态门前输出。
- 修改内容：只接入一个幂等的 `chassis_task_start()`；不得改 SRP ID、轮序、193 mm 几何、`WHEEL_TRIM` 或既有门控。
- 潜在影响：增加 512 words 栈和 10 ms 高优先级周期；会真正激活当前未运行的控制逻辑，必须按安全变更处理。

**验证方法**

1. 静态调用图确认唯一启动点；任务创建失败保持 zero-PWM。
2. CM7 canonical Debug 构建和任务 HWM/WCET；本轮未执行。
3. 匹配镜像下分别验证 `0x06`、`0x17`：ACK 后 10 ms 内出现正确 `[RR,RF,LR,LF]` 目标。
4. 同步、IMU、attitude freshness 任一撤销时目标清零且恢复后不自动重放。

### STM13-SAFE-001 - High - 直接四轮命令绕过姿态/IMU/MotorBoard READY 门

**确定事实（CONFIRMED_SOURCE）**

- `s3_service_on_frame()` 对 motion 的公共前置检查只有 `HOST_SYNCED`；同步后 `WHEEL_SPEED_CMD` 直接调用 `motor_board_set_target_wheel_speeds()`，见 `s3_service.c:375-400,457-483`。
- `motor_board_set_target_wheel_speeds()` 只验证有限值和 `1000 mm/s` 范围；非零目标会写入数组并把 `s_motion_forced_stop=false`，没有检查 `imu_boot_manager_is_ready()`、`g_attitude_is_ready`、DualAHRS freshness 或 MotorBoard 生命周期 READY，见 `motor_board_task.c:440-463`。
- 相比之下，chassis path 明确要求 sync、IMU ready、`g_attitude_is_ready`、primary fresh 和 `motor_board` task，见 `chassis_task.c:142-149,194-216`。
- 姿态 freshness 丢失时 coordinator 调用 `motor_board_force_stop()`，但不会撤销 SRP 同步，见 `attitude_startup_coordinator.c:113-120`。

**推断风险（INFERRED_RISK）**

- MotorBoard 任务已运行后，若姿态 freshness 丢失，coordinator 先强停；此时一个仍处于 SRP 同步会话的非零 `WHEEL_SPEED_CMD` 可再次清除 `s_motion_forced_stop`。下一条 `$MSPD` 可重新触发非零 PID/PWM，绕过已经关闭的姿态门。
- 这是可由当前调用链推出的安全绕过；实际车辆是否运动还取决于 S3 是否会发该命令、USART6/MotorBoard 是否在线及外部板行为。

**现有保护**

- 未同步的 motion 会被丢弃；BUS_OFF 和 200 ms S3 frame timeout 会撤销 `HOST_SYNCED` 并同时请求 chassis/MotorBoard 强停（`s3_service.c:176-201,876-899`）。
- 零四轮 tuple 会走 `motor_board_force_stop()`；所有单轮值有有限性/范围检查。
- 这些保护不覆盖“SRP 仍同步但本地姿态门已关闭”的场景。

**建议方案（本轮未实施）**

- 修改位置候选：为 `motor_board_set_target_wheel_speeds()` 增加单一 nonzero admission，或使所有 SRP motion 统一进入 `chassis_task`/安全仲裁层。
- 修改原因：最终执行器入口必须拥有一致门控，不能依赖每个上游分支记得检查。
- 修改内容：零目标始终可达；非零目标同时要求 sync、IMU ready、attitude ready/fresh、MotorBoard READY，且门关闭后旧目标不恢复。
- 潜在影响：会改变当前直通 wheel 命令的准入时机；App/S3 主机重试和 ACK 状态需要同步测试，但不应改变 wire ID/布局。

**验证方法**

- 主机状态测试覆盖：正常 READY、未标定、primary stale、LSM offline、S3 timeout、BUS_OFF、MotorBoard 非 READY；只有 READY 的非零命令成功，零命令在所有状态成功。
- 台架在非零运行中注入 BMI/LSM freshness loss，并持续发送 wheel command；USART6 必须只出现 zero-PWM，恢复后需要新鲜的重新准入动作。

### STM13-MB-001 - High - MotorBoard 没有反馈 READY/watchdog，PID 仍使用固定 50 ms

**确定事实（CONFIRMED_SOURCE）**

- 当前状态机只有初始化步骤 `LINK_PROBE -> ... -> UPLOAD -> RUNNING/FAILED`；没有 `LOCKED/WAIT_FEEDBACK/READY` 状态，见 `motor_board_task.c:41-60,672-765`。
- `$upload:0,1,1#` 发送成功并收到任意被认定成功的上传响应后即可推进；没有要求两条有效 `$MSPD`，见 `motor_board_task.c:423-432,752-759`。
- 运行循环只在收到 `MB_FRAME_MSPD` 时调用 `motor_board_update_pid()`；没有保存最后有效 MSPD 时间，也没有运行期 200 ms feedback watchdog，见 `motor_board_task.c:526-570,768-826`。
- PID/Ramp 对每一条 MSPD 固定使用 `MB_PID_DT_SECONDS=0.05f`，协议帧结构没有时间戳，见 `motor_board_task.c:32,552-560` 与 `motor_board_protocol.h:34-41`。
- 当前仓库已有设计文档要求“两条 2-100 ms MSPD 后 READY、丢失 200 ms 强停、旧目标不恢复”，但实现未兑现，见 `DOCS/superpowers/specs/2026-08-28-motor-board-p0-p1-design.md:59-79,105-133`。
- 初始化序列先写 `mtype/mline/mphase/wdiameter`，之后才 `read_flash`；`decode_payload()` 把任意包含 `OK/ACK/Set` 的行视为通用成功，`read_flash:OK!` 会让任务提前推进，见 `motor_board_task.c:685-760`、`motor_board_protocol.c:342-359`、`motor_board_task.c:402-412`。

**推断风险（INFERRED_RISK）**

- MSPD 停止后，CM7 不再更新 PWM，但也不会主动发送 zero-PWM；外部 MotorBoard 是否保持上一 PWM 或自带超时在源码中不可知。
- MSPD 周期抖动、丢帧或恢复时固定 50 ms 会扭曲积分和斜坡；严重程度依赖实际上传周期。
- 每次启动盲写配置和把首个通用 OK 当完整回读，可能掩盖配置不一致并增加 Flash 写入；硬件寿命和板端固件语义未验证。

**现有保护**

- 解析拒绝非有限 MSPD；PWM 输出有 `WHEEL_PID_MAX_OUT` 限幅，目标有 1000 mm/s 限幅。
- 初始化响应超时会重启 link probe；明确 NACK 进入 FAILED 并尝试 zero-PWM。
- S3/姿态故障会调用 `motor_board_force_stop()`，但 MSPD 本身失联不会。

**建议方案（本轮未实施）**

- 修改位置：`motor_board_task.c/.h` 的生命周期、反馈时间和目标准入；`motor_board_protocol.c/.h` 的配置快照/响应关联。
- 修改原因：把已有 P0/P1 设计转成可执行安全状态机。
- 修改内容：先回读并校验配置；两帧正常 MSPD 后 READY；使用实际单调 `dt`，异常间隔重置 PID；200 ms 无反馈即 zero-PWM、清目标、回 LOCKED；非 READY 拒绝非零目标。
- 潜在影响：初始化时间和配置写入顺序改变；必须冻结 `[RR,RF,LR,LF]`、encoder sign、trim、PWM 符号和 193 mm 上游几何。

**验证方法**

- 主机回放：配置字段乱序/缺失/重复/非法；MSPD 首帧、2-100 ms、>100 ms、回绕、200 ms timeout；恢复后旧目标不得恢复。
- 目标板：断开 USART6 RX/TX、制造 TX ring 满、停止 `$MSPD`、MotorBoard 复位；逻辑分析仪确认 200 ms 内 zero-PWM，并记录板端是否自带超时。

### STM13-SAFE-002 - High - 强停是有界队列中的普通尾部消息，可失败或被旧 PWM 排在前面

**确定事实（CONFIRMED_SOURCE）**

- `motor_board_force_stop()` 先清目标/PID并置 `s_motion_forced_stop=true`，但物理停止依赖 `MB_Protocol_SendPwm(0,0,0,0)` 返回成功，失败只记日志，见 `motor_board_task.c:486-505`。
- USART6 TX 是 512 B FIFO；空间不足时 `MB_Transport_Send()` 直接返回 false，不会为急停清队列或抢占，见 `motor_board_transport_uart.h:19-21` 与 `.c:166-193`。
- 新 zero-PWM 追加在 `s_tx_head`，已排队的旧非零 PWM 会先发送；发送 IRQ 按 FIFO 顺序取字节，见 `motor_board_transport_uart.c:182-191,240-252`。

**推断风险（INFERRED_RISK）**

- 队列积压时，软件状态已经 locked，但外部板可能先收到旧非零 PWM，或在队列满时完全收不到 zero-PWM。实际停止延迟取决于串口积压和板端保持策略。

**现有保护**

- `s_motion_forced_stop` 阻止新的 PID PWM 继续入队；所有调用者能看到 bool 并记录 drop。
- 没有硬件 brake/enable GPIO、优先 TX 槽或确认式 stop transaction 的当前证据。

**建议与验证**

- 在 USART6 transport 定义单一高优先级 stop primitive：原子撤销普通 TX、保证完整帧边界、优先发送 zero-PWM，并报告“已进入硬件发送/已完成”的状态；不得在 ISR 做字符串格式化。
- 用满队列、半帧、IRQ 延迟和 MotorBoard 不响应故障注入测量最坏停机时延；在证明前不能把 bool true 等同于车轮已停。

### STM13-SAFE-003 - High - CM7 致命故障/RTOS assert 无独立执行器停机与 watchdog 复位

**确定事实（CONFIRMED_SOURCE）**

- HardFault/MemManage/BusFault/UsageFault 保存 `.noinit` 记录后关中断并永久 WFI，见 `CM7/Core/Src/stm32h7xx_it.c:106-175,231-290`。
- 栈溢出和 malloc failed hook 记录后调用相同停机循环，见 `stm32h7xx_it.c:358-368` 与 `System/Task/rtos_health.c:238-247`。
- `configASSERT` 关中断后永久循环，见 `Config/FreeRTOSConfig.h:65-71`。
- CM7 HAL 配置未启用 IWDG/WWDG；项目源没有 watchdog 初始化、喂狗或外部 MotorBoard enable/brake 控制，见 `CM7/Core/Inc/stm32h7xx_hal_conf.h:47,65` 及项目源搜索结果。

**推断风险（INFERRED_RISK）**

- 若故障发生在非零 PWM 已到达外部 MotorBoard 后，CM7 不再有机会经 USART6 发送 zero-PWM。板端是否自行超时、车辆是否继续运动完全取决于未证明的外部硬件/固件行为。

**现有保护**

- `.noinit` fault/RTOS health 记录可在下一次启动报告；普通任务运行时的 sync/IMU/BUS_OFF 门和强停路径较完整。
- 这些都是“CPU 仍能执行”或“下次已复位”后的保护，不能关闭已挂死 CPU 下的执行器。

**建议与验证**

- 修改位置候选：系统 watchdog/安全层与 MotorBoard 硬件边界；原因是建立独立于任务调度的最终停机保证。
- 内容应包括：外部板明确 command timeout，或独立 enable/brake/继电器；IWDG 只由经过全部关键任务 heartbeat 的监督器喂；复位启动必须先物理 zero-PWM。
- 注入 HardFault、关中断死循环、最高优先级饿死、malloc fail 和 USART6 断线，在非零运行条件下测量车轮停机时间。没有该证据前只能标记 `UNVERIFIED_RUNTIME`。

### STM13-AHRS-001 - High/Medium - `s_dual` 跨任务读写没有一致快照机制

**确定事实（CONFIRMED_SOURCE）**

- DualAHRS 全局上下文 `s_dual` 包含多个 64-bit 时间戳、Euler、quaternion、state 和 flags，见 `dual_ahrs.c:35-74`。
- BMI323 200 Hz 低优先级任务调用 `dual_ahrs_update()`，一次更新会分阶段改写大量字段，见 `imu_manager.c:1123-1225`、`dual_ahrs.c:863-1030`。
- 更高优先级的 coordinator/chassis 可分别调用 `dual_ahrs_is_primary_fresh()`、`dual_ahrs_get_heading_state()`；低优先级 debug/telemetry 同时调用 `dual_ahrs_get_output()`/pack，见 `attitude_startup_coordinator.c:60-66`、`chassis_task.c:142-149,370-371`、`imu_runtime.c:110-119,175-198`。
- `dual_ahrs_get_output()` 直接结构体复制，freshness/heading 直接读取 `s_dual`；源码中没有 mutex、critical section、sequence counter 或双缓冲，见 `dual_ahrs.c:1033-1079`。

**推断风险（INFERRED_RISK）**

- 高优先级任务可在 producer 更新中途抢占，读到混合 sample 的 yaw、gyro、valid、state 或撕裂的 64-bit 时间戳。结果可能是遥测不一致、错误 freshness 判定，或 heading 控制使用不同周期的 yaw/gyro。
- 单个 32-bit float 在 M7 上通常可原子加载，不等于多字段快照一致；64-bit 时间戳尤其不能靠 `volatile` 解决。

**现有保护**

- 输入/输出有 finite、timestamp 单调、50 ms primary freshness、250 ms redundant freshness和状态降级检查。
- 这些校验本身读取同一未同步上下文，不能证明快照一致。

**建议与验证**

- 修改位置：`DualAHRS/dual_ahrs.c/.h` 的 producer/consumer 边界；使用短临界区复制 compact snapshot、单写者 sequence lock 或双缓冲 publish。控制读路径不应长时间等待低优先级 producer mutex。
- 主机并发测试无法完全模拟 M7 抢占；目标板增加高频抢占点/sequence 检查，验证每个读快照前后序号相等、timestamp/flags/state自洽，并测量关中断上限。

### STM13-RTOS-001 - Medium - RTOS 健康名单与活动任务图漂移

**确定事实（CONFIRMED_SOURCE）**

- 健康名单是 `imu_task, uart_link, s3_service, logger, protocol, imu_data_logger, bmi323_task`，见 `rtos_health.c:31-39`。
- 实际 UART task 名是 `srp_uart`，不存在 `uart_link`；当前不存在名为 `protocol` 的任务，见 `uart_link.c:894-904` 及全源 task 创建搜索。
- 安全关键的 `attitude_gate`、`motor_board` 不在名单，未来接入的 `chassis_task` 也不在；对应创建点见 `attitude_startup_coordinator.c:177-179`、`motor_board_task.c:830-838`、`chassis_task.c:541-550`。
- `log_service` 输出的健康摘要只打印 stacks 0..5，没有打印第 7 个 BMI slot，见 `log_service.c:60-86`。

**推断风险**

- 运行日志会把不存在任务的水位长期显示为 0，却漏掉真正决定运动/安全的任务；这降低故障前兆可见性，但栈溢出 hook 仍能捕获已经发生的溢出。

**建议与验证**

- 由单一 task registry 生成创建名和健康名单，至少覆盖实际活动任务；临时 init worker 应与稳态任务分开解释。
- 目标板长期压力下检查每个稳态任务 `sample_count>0`、HWM 有界；故意改小测试栈验证 hook 和 retained record，但不得在车辆可运动状态下注入。

### STM13-IO-001 - Medium - PA9/PA10 的 USART1、TIM1 和 LF_INT1 owner 仍冲突

**确定事实（CONFIRMED_SOURCE）**

- `main()` 先初始化 USART1，再初始化 TIM1，见 `CM7/Core/Src/main.c:185-193`。
- USART1 MSP 把 PA9/PA10 配为 AF7 TX/RX；随后 TIM1 encoder MSP 把 PA8/PA9 配为 AF1，因此 PA9 最终不再是 USART1 TX，见 `stm32h7xx_hal_msp.c:302-329,131-152`。
- 后续 BMI323 初始化会调用 `bsp_gpio_init()`；该函数把 PA10 配回普通输入作为 `LF_INT1`，因此 USART1 RX 也被重写，见 `bmi323_port.c:104-124` 与 `BSP/GPIO/bsp_gpio.c:47-59`。
- `main.c:188` 注释“USART1 owns PA9/PA10”，与最终初始化顺序不一致；TIM1 handle 在当前项目业务源中没有运行消费者。

**推断风险**

- CH340/USART1 调试输出和接收可能失效或电平异常，导致启动/故障诊断被误判为系统未运行。TIM1 encoder 是否真实接线、PA10 LF_INT1 是否需要保留仍是 PCB/IOC 决策，不能在只读审计中替换 owner。

**现有保护与附加观测**

- 主要日志任务当前调用 `bsp_uart_log_write_link_level()`，走 SRP/USART2 而非 USART1；但同步前 SRP log 被门控，且该 BSP 函数忽略实际 send 失败并返回 OK，见 `log_service.c:49-58`、`bsp_uart.c:193-247`。因此冲突也削弱早期启动日志的备用路径。

**建议与验证**

- 修改前先冻结物理 owner：PA9 只能选 USART1_TX 或 TIM1_CH2，PA10 只能选 USART1_RX 或 LF_INT1。当前 MotorBoard 已提供轮速反馈且 TIM1 无消费者，是决策输入，不是本轮授权的引脚变更。
- 用 IOC/原理图/示波器确认 AF 和接线；复位后分阶段读 GPIO AFR/MODER，验证最终 owner，并分别抓 USART1 TX、TIM1 encoder 和 LF_INT1。任何 GPIO/IOC 修改都需单独批准。

### STM13-SRP-001 - Medium - SRPv4 主合同已统一，但 ACK 接收校验和 liveness 语义仍偏宽

**确定事实（CONFIRMED_SOURCE）**

- 线缆帧为 `AA 55 | LEN_LE | HEADER_LE | payload | CRC16-CCITT-FALSE_LE | 0D 0A`，最大 payload 500 B；编解码显式小端且拒绝优先级/保留 flags/长度/CRC/EOF 错误，见 `srp_def.h:27-74`、`srp_codec.c:34-122`。
- 注册表固定 SRPv4 4.0、`CMD_SYNC_REQ=0x08`、`RSP_BOOT_INFO=0x09`，并用 static assert 固定关键 payload 大小，见 `srp_registry.h:20-45,69-103,213-237`。
- CM7 parser callback 在 `s_link_mutex` 内运行；boot-info 使用已持锁的 direct send，先成功发送再置 `HOST_SYNCED`，避免非递归 mutex 死锁，见 `s3_service.c:204-270,273-339,901-914`。
- `srp_link_receive()` 对 ACK/ERROR 只要求 payload 长度“至少 4”，随后按 type+sequence 清 pending；不验证 payload reserved 字节、精确长度、ACK/ERROR flags 或 priority，见 `srp_link.c:181-225`。
- CM7 在已经同步时，对任何 codec-valid frame 都刷新 `s_last_s3_frame_ms`，发生在业务类型/语义校验之前，见 `s3_service.c:557-568`。

**推断风险（INFERRED_RISK）**

- 一个 CRC 正确但语义畸形的 ACK 可提前结束匹配事务；一个持续发送未知合法帧的异常 peer 可维持 200 ms liveness，延迟链路超时强停。当前两端共享实现和可信链路降低概率，但这不是严格协议校验。

**现有保护**

- CRC、header、type+sequence 精确匹配、四个 pending slot、500 ms timeout、最多三次成功重传、REC/TEC/BUS_OFF 都存在。
- S3 heartbeat 主动发送合法 `CMD_SYNC_REQ`，CM7 只有验证完整 4-byte v4.0 payload 后才建立会话。

**建议与验证**

- ACK/ERROR 必须精确 4 B、reserved=0、flags 与 type 相符、priority 合法；liveness 应明确是“任意已验证 frame”还是“受支持 heartbeat/control frame”，并形成两端共同测试向量。
- host fuzz 覆盖超长 ACK、错 flags、错 reserved、未知 type heartbeat、sequence 回绕、pending 满和 transport retry fail；目标板捕获 921600 8N1 的真实 CRC/时延。源码和 host 测试不能证明 UART 电气行为。

### STM13-BUILD-001 - Medium - 当前构建依赖未跟踪的共享调试头

**确定事实（CONFIRMED_SOURCE/GIT）**

- `CM7/CMakeLists.txt:77-100` 加入 `../../Common/SmartCarDebug`；多个当前 STM 源 include `smartcar_debug_config.h`。
- `git status --short -- Common/SmartCarDebug` 显示 `?? Common/SmartCarDebug/`，`git ls-files Common/SmartCarDebug` 为空。
- 当前 `CM7/build/Debug/Smart_Car_H757_CM7.elf` 和 map 是既有产物，本轮没有重建，不能据此证明干净检出可构建。

**风险与验证**

- 本机脏树可见头文件，但同一 commit 的干净 clone/CI 会缺文件。发布前必须把共享配置的版本/跟踪边界定清，并在隔离干净检出中执行 canonical Debug 构建；本轮不授权提交该目录。

## 4. 第 14 轮：CM4 启动、RTOS、D2 内存、IPC 与双核 owner

### STM14-BOOT-001 - High（双核启用阻断）- CM4 等 HSEM0，但 CM7 明确不释放

**确定事实（CONFIRMED_SOURCE）**

- CM4 无条件定义 `DUAL_CORE_BOOT_SYNC_SEQUENCE`，启用 HSEM notification 后进入 D2 STOP/WFE，见 `CM4/Core/Src/main.c:35-45,85-100`。
- CM7 明确注释为 CM7-only，并直接完成时钟/外设/RTOS启动；项目自研 CM7 源没有 `HAL_HSEM_FastTake/Release`、CM4 boot-ready 或 D2CKRDY 同步，见 `CM7/Core/Src/main.c:56,148-232`。
- CM4 即使因其他 event 醒来，也只有 `HAL_Init()` 后空循环，见 `CM4/Core/Src/main.c:102-129`。

**推断风险**

- 若 option bytes 实际启用了 CM4，它会等待永不发生的产品级释放；若虚假唤醒，会以 240 MHz 空转且没有状态报告。是否启用 CM4 无法从源码确认。
- 当前序列也缺少标准双核启动常见的“CM4 已进入 D2 STOP”屏障；若两个核并发碰全局时钟，存在条件性启动风险。

**现有保护**

- 根 `STM32H757/CMakeLists.txt:3-6` 阻止聚合双核构建；CM7 保留所有执行器和安全权威。该保护只覆盖构建意图，不能证明芯片 BCM4/BOOT4_ADD 状态。

**建议与验证**

- 双核第一阶段只实现带超时的 `RESET -> WAIT_CM7_CLOCK -> HSEM_RELEASED -> IPC_READY -> HEARTBEAT` no-op；CM4 缺失/卡死时 CM7 必须继续单核并保持 zero-PWM。
- 读回 option bytes、配对镜像 hash/boot address；抓 D2CKRDY/HSEM 时序，并注入 CM4 未烧录、晚启动、虚假 event、单核复位。

### STM14-MEM-001 - High（条件性）- CM4 `.data/.bss` 与 CM7 UART DMA 使用同一 D2 物理 RAM

**确定事实与旧结论修正**

- CM4 linker 将普通 RAM 定为 `0x10000000/288K`，见 `CM4/stm32h757xx_flash_CM4.ld:38-47`；CM7 `.dma_buffer` 从 `0x30000000/288K` 起，见 `CM7/stm32h757xx_flash_CM7.ld:38-43,214-221`。
- 器件头同时定义 `D2_AXISRAM_BASE=0x10000000` 和 `D2_AHBSRAM_BASE=0x30000000`，是同一 D2 SRAM 的不同总线访问窗口，见 `Drivers/CMSIS/Device/ST/STM32H7xx/Include/stm32h757xx.h:2297-2298`。
- `uart_link.c:38-39` 把 512 B RX buffer 放到 `.dma_buffer`。
- 关键时序修正：CM4 `Reset_Handler` 在进入 `main()`/HSEM 等待之前已经复制 `.data` 并清 `.bss`，见 `CM4/Core/Startup/startup_stm32h757xx_CM4.s:60-102`。因此不是“CM7 释放 CM4 后才首次覆盖”。
- 既有旧 map（`CONFIRMED_OLD_ARTIFACT`，非本轮构建）显示 CM7 `.dma_buffer=0x30000000..0x30000200`；CM4 `.data/.bss` 占 `0x10000000..0x10000030`，`uwTick` 在物理别名偏移 `0x2c`。

**推断风险（INFERRED_RISK）**

- BCM4 启用时，CM4 reset 的初始写可能早于 CM7 DMA arm，是否造成当次错误取决于启动顺序；但 CM4 运行中复位会直接改写活动 DMA 前端。
- CM4 一旦从 WFE 醒来，`HAL_Init()`/SysTick 每毫秒更新 `uwTick`，可持续破坏 CM7 DMA 偏移 `0x2c`，表现可能是 SRP CRC、丢帧、重装和 BUS_OFF。

**现有保护**

- 当前设计意图是 CM7-only；只要实际 BCM4 未启用且 CM4 不复位/唤醒，条件不触发。源码没有 option-byte 证据，不能把该前提标为已确认。

**建议与验证**

- 双核发布前按物理地址归一化两个最终 map，重划 CM4 `.data/.bss/heap/stack`、CM7 DMA 和 shared mailbox；把 overlap 检查做成构建门。
- 读 option bytes 后，用 sentinel/CRC 监视 DMA；注入 CM4 reset/wakeup 并同时压测 SRP，确认无别名写入。

### STM14-IPC-001 - High（双核启用阻断）- 没有共享 ABI、cache、heartbeat 或 reset epoch

**确定事实（CONFIRMED_SOURCE）**

- CM7/CM4 linker 都没有 `.shared`/mailbox section；项目自研源没有 IPCC/OpenAMP/RPMsg、版本化共享结构、跨核 cache clean/invalidate、heartbeat、age timeout 或 CM4 reset generation。
- CM7 开启 I/D cache，见 `CM7/Core/Src/main.c:154-163`；CM4 MPU 只有 privileged default，见 `CM4/Core/Src/main.c:135-145`。
- HSEM0 只出现在未完成的启动等待，不构成 producer/consumer 数据合同。

**推断风险**

- 直接共享结构会有 torn snapshot、cache 陈旧、复位残留、版本不兼容和指针/HAL handle 生命周期问题；CM4 卡死时 CM7没有确定的服务过期判据。

**建议与验证**

- 先建立固定物理区、版本化 SPSC mailbox：magic/version/length/sequence/timestamp/reset_epoch/valid/CRC/commit；单写者、禁止跨核指针/HAL handle。
- 共享区设 non-cacheable/shareable，或严格定义 32-byte cache line clean/invalidate 与 DMB/DSB；no-op heartbeat 先于业务迁移验收。
- cache on/off、乱序、CRC 错、CM4卡死/复位、CM7单核回退压力测试；所有过期数据必须被 CM7丢弃，执行器权威不迁移。

### STM14-RTOS-001 - High（CM4 任务迁移阻断）- IOC 名称不等于可运行的 CM4 FreeRTOS

**确定事实（CONFIRMED_SOURCE）**

- CM4 live source list 只有 Core/startup 和基础 HAL，没有 FreeRTOS、OpenAMP、watchdog或应用任务，见 `CM4/mx-generated.cmake:20-48`。
- CM4 `SVC_Handler`/`PendSV_Handler` 为空，SysTick 只 `HAL_IncTick()`，见 `CM4/Core/Src/stm32h7xx_it.c:144-192`。
- 跟踪树当前只有 CM7 `portable/GCC/ARM_CM7/r0p1`，没有已集成的 CM4F port。
- CM4 HAL conf 的 `USE_RTOS=0`；IOC 虽列 `FREERTOS_M4/OPENAMP_M4/IWDG2`，CMake/IRQ并未兑现。

**推断风险**

- 直接复制 CM7 task/kernel 源到 CM4 会缺正确 port、heap、SVC/PendSV/SysTick、IRQ优先级和 fault hooks；任务可能完全不调度或在异常路径静默自旋。

**建议与验证**

- 明确 CM4 是否用 FreeRTOS；若使用，锁定 vendor CM4F port、独立 config/heap/hook/IRQ，先运行 no-op+heartbeat。若不用，清晰标记 IOC 中能力为 reserved。
- 独立 CM4 Debug build、任务切换/stack HWM/malloc fault测试；不能用 IOC 标签替代运行证明。

### STM14-BUILD-001 - Medium - CM4 启动汇编、产物路径和文档三方漂移

**确定事实（CONFIRMED_SOURCE）**

- `CM4/Core/Startup/startup_stm32h757xx_CM4.s:27-30` 声明 `.cpu cortex-m7`，但 `CM4/CMakeLists.txt:12` 指定 cortex-m4。当前 reset 指令表面上是通用 Thumb-2；错误属性仍允许未来引入 M4 不支持的指令。
- `CM4/README.md:3-4` 称 target 不产出 firmware，但 `CM4/CMakeLists.txt:17-19` 实际 `add_executable`。
- CM4 preset 输出 `CM4/build/Debug`，孤立的根 `mx-generated.cmake:4-17` 仍写 `CM4/build`；根 CMake 又直接 FATAL 要求只配 CM7。
- 工作区存在 2026-08-23 的旧 CM4 ELF/map，仅为历史产物；本轮没有 build，不能证明当前源或 CPU attribute。

**建议与验证**

- 双核实施前统一一个 CM4 build/preset/artifact 路径和成对 firmware manifest；修正/重新生成 startup 后用 `readelf -A`、`objdump`、vector/map检查 M4 属性和指令。
- 发布系统必须拒绝旧 CM4 ELF 与新 CM7 ELF 混配。

### STM14-OWNER-001 - Medium/High - 所有实时外设仍归 CM7，首批迁移不能包含闭环链

**确定事实（CONFIRMED_SOURCE）**

- IOC 的 BMI323 SPI1/CS/INT、LSM303 I2C4、USART2、USART6 和相关 GPIO 均标记 CortexM7；CM4 source list 无 BSP/driver/HAL handle。
- CM7 CMake 纳入传感器、标定、滤波、DualAHRS、S3 UART、安全、底盘和 MotorBoard，见 `CM7/CMakeLists.txt:39-75`。
- 当前安全放行同时依赖 IMU lifecycle、legacy attitude zero、DualAHRS primary freshness、LSM/BMI update_count 和 MotorBoard task，见 `attitude_startup_coordinator.c:60-95,98-159`。

**推断风险**

- 把 SPI/I2C、IMU、DualAHRS、UART DMA 或 MotorBoard 任一段先搬到 CM4，会跨核切开高频闭环并引入 IPC deadline、外设双 owner 和故障回退问题。

**推荐边界（设计建议，不是已实现事实）**

- CM7 保留：全部 sensor bus/DMA owner、IMU/标定/DualAHRS、sync/attitude/BUS_OFF门、USART2最终准入、USART6/MotorBoard和 zero-PWM 权威。
- CM4 首批仅候选：无副作用 heartbeat、低频诊断统计、日志/遥测格式化；只消费版本化只读快照，过期即丢弃。
- 任何迁移收益必须用 CM7 WCET/CPU/IRQ latency 和 IPC 最坏延迟证明；当前没有这些运行数据。

## 5. 优先级与停止条件

| 顺序 | 阻断项 | 原因 |
|---:|---|---|
| P0 | `STM13-SAFE-001` 统一所有非零 motion 最终准入 | 当前存在姿态门关闭后被直通 wheel 命令重新放行的静态路径 |
| P0 | `STM13-MB-001/002` 建立 MotorBoard READY、反馈 watchdog 和优先强停 | 外部执行器停机目前依赖普通 FIFO 和未证明的板端保持语义 |
| P0 | `STM13-SAFE-003` 取得 CPU hang 下的独立停机证据 | fault/assert/watchdog 边界未闭合 |
| P1 | `STM13-CTRL-001` 在安全门内接入 chassis task | 当前 chassis/heading 命令可 ACK 但无执行任务 |
| P1 | `STM13-AHRS-001` 发布一致姿态快照 | 高优先级安全/控制读者可抢占低优先级 producer |
| P1 | `STM13-IO-001` 冻结 PA9/PA10 owner | 当前初始化顺序无法同时满足 USART1、TIM1 和 LF_INT1 |
| 双核前 P0 | `STM14-MEM/BOOT/IPC/RTOS` 全部关闭 | 任一未关闭都不允许烧录/启动 CM4 产品任务 |

在 P0 安全项获得 source/build/bench 三层证据前，不建议进行车辆运动验收；在 CM4 物理内存、option bytes、boot handshake、IPC/reset 和故障注入全部通过前，继续保持 CM7-only。

## 6. 验证矩阵与证据限制

| 验证项 | 静态结论 | 后续必须取得的证据 | 本轮状态 |
|---|---|---|---|
| Live CMake/SRPv4 | 当前 CM7 编译共享 SRP 源 | canonical Debug clean build、host codec tests | 未执行 |
| chassis task | 无启动调用 | task trace/HWM、命令到四轮目标时延 | 未执行 |
| motion gate | 直通 wheel 有绕过路径 | fault/state injection + USART6 capture | 未执行 |
| MotorBoard watchdog | 当前无 200 ms MSPD watchdog | 拔线、停流、板复位、队列满下 zero-PWM 时间 | 未执行 |
| fatal stop | CM7 halt 无独立 stop/watchdog | 非零运行故障注入、硬件 stop 时间 | 未执行 |
| DualAHRS snapshot | 无同步 publish | M7 抢占/sequence一致性和关中断预算 | 未执行 |
| USART1 PA9/PA10 | 最终 AF owner 冲突 | IOC/原理图确认、寄存器和波形 | 未执行 |
| CM4 boot | HSEM 等待且 CM7不释放 | BCM4/BOOT4_ADD、D2CKRDY/HSEM capture | 未执行 |
| D2 alias | 物理区间重叠，旧 map 交叉支持 | 当前双 map归一化、CM4 reset + DMA sentinel | 未执行 |
| CM4 RTOS/IPC | 当前不存在完整实现 | CM4 build、heartbeat/no-op、reset/cache stress | 未执行 |

本文件不能证明：当前 ELF 对应当前全部脏源码、目标板已烧录何种 CM7/CM4 镜像、CM4 option byte 状态、USART2/USART6 电气连通、DMA callback 时序、传感器单位/方向、MotorBoard 自带超时、真实车轮停机、BLE/App 端到端行为或车辆安全。所有这类结论继续标为 `UNVERIFIED_RUNTIME`。

## 7. 本轮文件边界复核

- 新增：`.planning/stm-s3-full-audit-20260829/round13-14-stm-current.md`
- 未修改：任何固件、共享协议、CMake、IOC、linker、startup、配置或生成物。
- 未执行：configure/build/test/flash/reset/serial capture。
- 未回滚、未提交、未格式化用户或并行任务的现有脏改动。
