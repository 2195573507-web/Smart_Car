#ifndef SMARTCAR_DEBUG_CONFIG_H
#define SMARTCAR_DEBUG_CONFIG_H

/*
 * Smart_Car 调试参数统一入口。
 *
 * 创建人：待确认（当前维护人：Zhiqin；各宏的历史首次提交人以 Git 为准）
 * 说明：本文件只保存诊断、抓包和日志节流参数，不保存协议字段、引脚
 *       映射、运动限幅或安全门控。布尔诊断开关默认关闭，数值参数保留
 *       既有默认值。修改后必须重新编译对应固件。
 * 调用方式：由需要调试参数的 S3/STM 源文件直接 #include 本文件。
 * 配置方式：可直接修改本文件，或使用编译器的 -D<宏名>=<值> 预定义。
 *           cmake/idf.py 的 -D 只是 CMake cache 变量；只有工程 CMakeLists.txt
 *           显式映射的选项才会自动转换为编译器宏。
 */

#include <stdint.h>

/*
 * BMI323-only 诊断镜像开关，默认 0：S3/CM7 均运行正常集成生命周期。
 * 设为 1 会跳过 S3 雷达相关业务并改变 CM7 双 IMU、姿态和任务启动路径，属于行为级
 * 诊断模式而非普通日志开关；只允许静止台架镜像，生产及车辆运行镜像必须保持 0。
 */
#ifndef SMARTCAR_BMI323_DEBUG_ONLY
#define SMARTCAR_BMI323_DEBUG_ONLY 0
#endif

/*
 * 启用 CM7 无依赖原始诊断标记。
 * 默认 0；
 * 1 会在部分故障/UART 路径关闭中断并轮询 USART1，增加 IRQ 延迟和控制抖动；
 * 仅限静止台架诊断镜像，生产及车辆运行镜像必须保持 0。
 */
#ifndef SMARTCAR_RAW_DIAGNOSTICS
#define SMARTCAR_RAW_DIAGNOSTICS 0
#endif

/*
 * IMU 运行任务的原始数据格式化开关。
 * 默认 0；
 * 当前 imu_runtime_log() 为 no-op，开启后只增加每 10 ms 的格式化开销而不会输出；
 * 该开关保留用于后续受限日志接入，当前诊断镜像也应保持 0。
 */
#ifndef IMU_RUNTIME_ENABLE_RAW_DATA_LOG
#define IMU_RUNTIME_ENABLE_RAW_DATA_LOG 0U
#endif

/*
 * S3 雷达 UART 原始字节日志开关，默认 0。
 * 设为 1 会分配预览缓冲并在接收任务中做十六进制格式化/日志输出，增加 CPU、日志带宽
 * 和调度抖动；仅限台架抓包，生产镜像保持 0。
 */
#ifndef RADAR_UART_RAW_LOG_ENABLED
#define RADAR_UART_RAW_LOG_ENABLED 0U
#endif

/*
 * S3 雷达原始 UART 字节通过 BLE FFE3 转发的开关。
 * 默认 0；当前实现位于 RADAR_UART_RAW_LOG_ENABLED 代码块内，必须同时打开两个开关。
 * 开启会额外占用 BLE 通知带宽并可能挤压业务日志；仅限台架诊断，生产镜像保持 0。
 */
#ifndef RADAR_BLE_RAW_UART_LOG_ENABLED
#define RADAR_BLE_RAW_UART_LOG_ENABLED 0U
#endif

/*
 * S3 UART 启动阶段最多捕获并打印的字节数，默认 32 字节。
 * 该值决定静态 capture、任务栈 snapshot 和十六进制文本数组大小；增大会消耗 RAM/栈并
 * 延长格式化时间，不改变实际 UART/SRP 帧。生产镜像可保留默认值，增大前须复核栈水位。
 */
#ifndef STM_UART_BOOT_CAPTURE_BYTES
#define STM_UART_BOOT_CAPTURE_BYTES 32U
#endif

/*
 * STM UART 错误日志最小间隔，默认 500000 us；只节流日志，不参与恢复或超时判定。
 * 降低会增加 S3 CPU/console 负载并可能掩盖实时问题；生产镜像允许默认值，禁止设为 0。
 */
#ifndef STM_UART_ERROR_LOG_PERIOD_US
#define STM_UART_ERROR_LOG_PERIOD_US UINT64_C(500000)
#endif

/*
 * S3 服务原生 UART 诊断摘要最小间隔，默认 1000 ms。
 * 降低会增加服务任务格式化和 console 负载；仅影响诊断输出，生产镜像允许默认值。
 */
#ifndef SMARTCAR_SERVICE_UART_DIAG_PERIOD_MS
#define SMARTCAR_SERVICE_UART_DIAG_PERIOD_MS UINT32_C(1000)
#endif

/*
 * S3 服务接收重置原因日志最小间隔，默认 500 ms。
 * 降低会在故障风暴中增加日志负载，不改变 parser reset；生产镜像允许默认值。
 */
#ifndef SMARTCAR_SERVICE_RX_RESET_LOG_PERIOD_MS
#define SMARTCAR_SERVICE_RX_RESET_LOG_PERIOD_MS UINT32_C(500)
#endif

/*
 * S3 服务任务栈水位报告周期，默认 5000 ms。
 * 降低会增加 FreeRTOS 查询和日志开销，不改变栈容量；生产镜像允许默认值。
 */
#ifndef SMARTCAR_SERVICE_STACK_REPORT_PERIOD_MS
#define SMARTCAR_SERVICE_STACK_REPORT_PERIOD_MS UINT32_C(5000)
#endif

/*
 * SRP 错误诊断的原始帧预览上限，默认 64 字节；不会改变接收、解析或实际帧长度。
 * 该值决定 S3 服务任务栈上的十六进制数组大小，增大会提高栈和日志开销；生产镜像
 * 允许默认值，调整后必须复核 smartcar_service 栈水位。
 */
#ifndef SRP_DEBUG_RAW_FRAME_MAX_BYTES
#define SRP_DEBUG_RAW_FRAME_MAX_BYTES UINT16_C(64)
#endif

/*
 * S3 雷达 console 十六进制预览长度，默认 32 字节；只在原始日志开关开启时生效。
 * 增大会增加静态 RAM、格式化时间和日志带宽；生产镜像关闭原始日志并保留默认值。
 */
#ifndef RADAR_UART_HEX_LOG_BYTES
#define RADAR_UART_HEX_LOG_BYTES 32U
#endif

/*
 * S3 雷达经 FFE3 转发的十六进制预览长度，默认 20 字节。
 * 增大会增加 BLE 包长度、格式化负载，并要求同步扩大 RADAR_BLE_LOG_BUFFER_SIZE；
 * 仅台架原始日志使用，生产镜像保留默认值且关闭对应开关。
 */
#ifndef RADAR_BLE_HEX_LOG_BYTES
#define RADAR_BLE_HEX_LOG_BYTES 20U
#endif

/*
 * S3 雷达 BLE 日志文本缓冲容量，默认 96 字节，属于静态 RAM。
 * 必须容纳固定前缀和 3 倍十六进制预览；仅影响诊断文本，不改变雷达帧或 BLE MTU。
 */
#ifndef RADAR_BLE_LOG_BUFFER_SIZE
#define RADAR_BLE_LOG_BUFFER_SIZE 96U
#endif

/*
 * S3 雷达原始 console 日志最小间隔，默认 200 ms。
 * 降低会增加雷达接收任务负载；仅在 RAW_LOG 开启时生效，生产镜像保留默认值。
 */
#ifndef RADAR_UART_HEX_LOG_PERIOD_MS
#define RADAR_UART_HEX_LOG_PERIOD_MS 200U
#endif

/*
 * S3 雷达 FFE3 原始日志最小间隔，默认 200 ms。
 * 降低会增加 BLE 通知和服务竞争；仅台架抓包使用，生产镜像保留默认值。
 */
#ifndef RADAR_BLE_LOG_PERIOD_MS
#define RADAR_BLE_LOG_PERIOD_MS 200U
#endif

/*
 * S3 雷达 parser 统计日志最小间隔，默认 1000 ms。
 * 降低只增加统计格式化/console 负载，不提高解析频率；生产镜像允许默认值。
 */
#ifndef RADAR_PARSER_STATS_LOG_PERIOD_MS
#define RADAR_PARSER_STATS_LOG_PERIOD_MS 1000U
#endif

/*
 * CM7 原始诊断单次 USART1 flag 忙等上限，默认 20000 次循环。
 * 仅 SMARTCAR_RAW_DIAGNOSTICS=1 时生效；增大会延长关 IRQ 时间，减小可能截断输出。
 * 生产镜像因原始诊断必须关闭而不使用该值。
 */
#ifndef CM7_RAW_DIAG_WAIT_SPINS
#define CM7_RAW_DIAG_WAIT_SPINS UINT32_C(20000)
#endif

/*
 * CM7 原始诊断名称/计数槽位数，默认 16，决定诊断镜像静态 RAM 使用量。
 * 槽满后新名称静默丢弃；仅原始诊断镜像使用，生产镜像不应为其增加容量。
 */
#ifndef CM7_RAW_DIAG_SLOT_COUNT
#define CM7_RAW_DIAG_SLOT_COUNT UINT32_C(16)
#endif

/*
 * CM7 IMU 状态、DualAHRS 快照和栈监控周期，默认依次为 5000/1000/5000 ms。
 * 降低会增加传感器快照、格式化和日志队列负载，但不改变 IMU 采样/控制周期；
 * 生产镜像允许默认值，调试提频前须检查任务栈和 UART 带宽。
 */
#ifndef IMU_STATUS_PERIOD_MS
#define IMU_STATUS_PERIOD_MS UINT32_C(5000)
#endif
#ifndef IMU_DUAL_AHRS_LOG_PERIOD_MS
#define IMU_DUAL_AHRS_LOG_PERIOD_MS UINT32_C(1000)
#endif
#ifndef IMU_STACK_MONITOR_PERIOD_MS
#define IMU_STACK_MONITOR_PERIOD_MS UINT32_C(5000)
#endif

/*
 * CM7 S3 服务栈水位日志周期，默认 5000 ms。
 * 降低只增加 FreeRTOS 查询和日志负载，不改变 SRP tick；生产镜像允许默认值。
 */
#ifndef S3_SERVICE_STACK_MONITOR_PERIOD_MS
#define S3_SERVICE_STACK_MONITOR_PERIOD_MS UINT32_C(5000)
#endif

/*
 * CM7 S3 服务 UART/SRP 遥测摘要日志周期，默认 1000 ms。
 * 降低会增加共享日志队列和 USART1 输出负载，不改变实际遥测发送周期；生产允许默认值。
 */
#ifndef S3_SERVICE_TELEMETRY_LOG_PERIOD_MS
#define S3_SERVICE_TELEMETRY_LOG_PERIOD_MS UINT32_C(1000)
#endif

/*
 * STM UART 链路任务栈/硬件统计日志周期，默认 5000 ms。
 * 降低会增加统计临界区、格式化和日志流量，不改变 USART2 DMA；生产允许默认值。
 */
#ifndef UART_LINK_STACK_MONITOR_PERIOD_MS
#define UART_LINK_STACK_MONITOR_PERIOD_MS UINT32_C(5000)
#endif

/*
 * CM7 日志服务采样任务栈/堆水位并输出摘要的周期，默认 5000 ms。
 * 降低会增加任务遍历、校验和与 USART1 输出负载；生产允许默认值，不宜用于高频监控。
 */
#ifndef LOG_SERVICE_HEALTH_PERIOD_MS
#define LOG_SERVICE_HEALTH_PERIOD_MS UINT32_C(5000)
#endif

/*
 * CM7 姿态启动未解锁状态日志周期，默认 1000 ms。
 * 只节流 LOCKED 日志，不改变 20 ms 安全检查或解锁条件；生产允许默认值。
 */
#ifndef ATTITUDE_STARTUP_LOG_PERIOD_MS
#define ATTITUDE_STARTUP_LOG_PERIOD_MS UINT32_C(1000)
#endif

/*
 * CM7 BMI323 端口延时诊断最多记录次数，默认 8。
 * 增大会增加启动阶段 snprintf/USART1 阻塞次数，不改变 SPI 延时本身；生产保留默认值，
 * 需要更多接线证据时只在台架镜像临时提高，0 可用于关闭该类延时跟踪。
 */
#ifndef BMI323_PORT_DELAY_TRACE_LIMIT
#define BMI323_PORT_DELAY_TRACE_LIMIT UINT8_C(8)
#endif

/*
 * BMI323 端口诊断文本单次 USART1 发送超时，默认 100 ms。
 * 增大会扩大启动最坏阻塞时间，减小可能丢日志；不影响 SPI 事务超时，生产保留默认值。
 */
#ifndef BMI323_PORT_LOG_TIMEOUT_MS
#define BMI323_PORT_LOG_TIMEOUT_MS UINT32_C(100)
#endif

/*
 * CM7 航向控制和四轮输出日志最小间隔，默认分别为 500/100 ms。
 * 降低会在 10 ms 控制任务中增加格式化/队列压力和抖动，但不改变控制计算周期；
 * 生产镜像允许默认值，台架提频前须观察任务 deadline 和日志 drop。
 */
#ifndef CHASSIS_HEADING_LOG_PERIOD_MS
#define CHASSIS_HEADING_LOG_PERIOD_MS UINT32_C(500)
#endif
#ifndef CHASSIS_OUTPUT_LOG_PERIOD_MS
#define CHASSIS_OUTPUT_LOG_PERIOD_MS UINT32_C(100)
#endif

/*
 * S3 日志桥 `STM_LOG_RX` 本地标记的最小间隔，默认 50 ms。
 * 只节流 ESP console 标记，不节流实际 FFE3 日志帧转发；降低会增加 console 负载，
 * 生产镜像允许默认值。
 */
#ifndef LOG_BRIDGE_MIN_INTERVAL_MS
#define LOG_BRIDGE_MIN_INTERVAL_MS UINT32_C(50)
#endif

/*
 * 实验性 S3RD 雷达上行统计日志最小间隔，默认 1000 ms。
 * 降低会增加上行任务格式化/console 负载，不改变 TCP 重连或帧发送；功能关闭时无运行影响。
 */
#ifndef RADAR_UPLINK_STATS_INTERVAL_MS
#define RADAR_UPLINK_STATS_INTERVAL_MS UINT32_C(1000)
#endif

/*
 * 实验性 S3RD 遥测可观测摘要的最小输出间隔，默认 2000 ms。
 * 该周期同时限制 TCP 重连退避期间的可观测日志唤醒频率；降低会增加上行任务格式化、
 * console 和 BLE FFE3 负载，但不改变 telemetry 入队、TCP 发送或重同步语义。
 * 生产镜像允许默认值，禁止设为 0。
 */
#ifndef RADAR_UPLINK_TELEMETRY_LOG_INTERVAL_MS
#define RADAR_UPLINK_TELEMETRY_LOG_INTERVAL_MS UINT32_C(2000)
#endif

/* 配置边界：这些宏直接参与数组容量、任务周期或条件编译。 */
#if (SMARTCAR_BMI323_DEBUG_ONLY != 0) && (SMARTCAR_BMI323_DEBUG_ONLY != 1)
#error "SMARTCAR_BMI323_DEBUG_ONLY must be 0 or 1"
#endif
#if (SMARTCAR_RAW_DIAGNOSTICS != 0) && (SMARTCAR_RAW_DIAGNOSTICS != 1)
#error "SMARTCAR_RAW_DIAGNOSTICS must be 0 or 1"
#endif
#if (IMU_RUNTIME_ENABLE_RAW_DATA_LOG != 0) && \
    (IMU_RUNTIME_ENABLE_RAW_DATA_LOG != 1)
#error "IMU_RUNTIME_ENABLE_RAW_DATA_LOG must be 0 or 1"
#endif
#if (RADAR_UART_RAW_LOG_ENABLED != 0) && (RADAR_UART_RAW_LOG_ENABLED != 1)
#error "RADAR_UART_RAW_LOG_ENABLED must be 0 or 1"
#endif
#if (RADAR_BLE_RAW_UART_LOG_ENABLED != 0) && \
    (RADAR_BLE_RAW_UART_LOG_ENABLED != 1)
#error "RADAR_BLE_RAW_UART_LOG_ENABLED must be 0 or 1"
#endif
#if RADAR_BLE_RAW_UART_LOG_ENABLED && !RADAR_UART_RAW_LOG_ENABLED
#error "RADAR_BLE_RAW_UART_LOG_ENABLED requires RADAR_UART_RAW_LOG_ENABLED"
#endif
#if (STM_UART_BOOT_CAPTURE_BYTES == 0) || (SRP_DEBUG_RAW_FRAME_MAX_BYTES == 0)
#error "UART/SRP diagnostic capture length must be greater than zero"
#endif
#if (RADAR_UART_HEX_LOG_BYTES == 0) || (RADAR_BLE_HEX_LOG_BYTES == 0)
#error "Radar diagnostic preview length must be greater than zero"
#endif
#if RADAR_BLE_LOG_BUFFER_SIZE < ((RADAR_BLE_HEX_LOG_BYTES * 3U) + 36U)
#error "RADAR_BLE_LOG_BUFFER_SIZE is too small for the configured preview"
#endif
#if (CM7_RAW_DIAG_WAIT_SPINS == 0) || (CM7_RAW_DIAG_SLOT_COUNT == 0)
#error "CM7 raw diagnostic limits must be greater than zero"
#endif
#if (STM_UART_ERROR_LOG_PERIOD_US == 0) || \
    (SMARTCAR_SERVICE_UART_DIAG_PERIOD_MS == 0) || \
    (SMARTCAR_SERVICE_RX_RESET_LOG_PERIOD_MS == 0) || \
    (SMARTCAR_SERVICE_STACK_REPORT_PERIOD_MS == 0) || \
    (RADAR_UART_HEX_LOG_PERIOD_MS == 0) || \
    (RADAR_BLE_LOG_PERIOD_MS == 0) || \
    (RADAR_PARSER_STATS_LOG_PERIOD_MS == 0) || \
    (IMU_STATUS_PERIOD_MS == 0) || \
    (IMU_DUAL_AHRS_LOG_PERIOD_MS == 0) || \
    (IMU_STACK_MONITOR_PERIOD_MS == 0) || \
    (S3_SERVICE_STACK_MONITOR_PERIOD_MS == 0) || \
    (S3_SERVICE_TELEMETRY_LOG_PERIOD_MS == 0) || \
    (UART_LINK_STACK_MONITOR_PERIOD_MS == 0) || \
    (LOG_SERVICE_HEALTH_PERIOD_MS == 0) || \
    (ATTITUDE_STARTUP_LOG_PERIOD_MS == 0) || \
    (CHASSIS_HEADING_LOG_PERIOD_MS == 0) || \
    (CHASSIS_OUTPUT_LOG_PERIOD_MS == 0) || \
    (LOG_BRIDGE_MIN_INTERVAL_MS == 0) || \
    (RADAR_UPLINK_STATS_INTERVAL_MS == 0) || \
    (RADAR_UPLINK_TELEMETRY_LOG_INTERVAL_MS == 0)
#error "Diagnostic periods must be greater than zero"
#endif

#endif /* SMARTCAR_DEBUG_CONFIG_H */
