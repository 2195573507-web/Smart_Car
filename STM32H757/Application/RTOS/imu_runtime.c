#include "imu_runtime.h"

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "attitude.h"
#include "dual_ahrs.h"
#include "boot_log.h"
#include "imu_calibration.h"
#include "imu_boot_manager.h"
#include "imu_filter.h"
#include "imu_manager.h"
#include "imu_time.h"
#include "mag_filter.h"
#include "log_service.h"
#include "s3_service.h"
#include "srp_registry.h"
#include "smartcar_debug_config.h"

/* CM7 IMU 采样/标定/姿态/遥测任务实现；创建人：待确认（当前维护人：Zhiqin）。 */

#define IMU_DATA_PERIOD_MS         UINT32_C(100)
#define IMU_TELEMETRY_TICK_MS       UINT32_C(10)
#define IMU_TELEMETRY_IMU_PERIOD_MS UINT32_C(100)
#define IMU_TELEMETRY_ATTITUDE_PERIOD_MS UINT32_C(50)
#define IMU_DATA_STACK_WORDS       UINT16_C(512)
#define IMU_DATA_PRIORITY          (tskIDLE_PRIORITY + 1U)
#define IMU_SAMPLE_PRIORITY        (tskIDLE_PRIORITY + 2U)
#define IMU_RUNTIME_PI             3.14159265358979323846f
#define RAD_TO_DEG                 57.295779513f

static volatile uint32_t imu_runtime_log_fail_count;
static TaskHandle_t s_imu_task_handle;
static TaskHandle_t s_imu_debug_task_handle;
static uint8_t s_dual_attitude_payload[DUAL_AHRS_PAYLOAD_LENGTH];

/**
 * @brief 将单精度浮点位模式按小端序写入 4 字节协议字段。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[out] destination 至少 4 字节的可写目标区，不得为 `NULL`。
 * @param[in] value 待序列化的 `float` 值。
 * @return 无返回值；不检查指针、容量或非有限数。
 * 调用方式：构造 LSM303/BMI323 各 30 字节 `IMU_TELEMETRY` 负载时逐字段调用。
 * 线程约束：纯内存写入、不阻塞，可在 ISR 调用；调用者独占目标缓冲区，并依赖当前平台 32 位 IEEE-754 `float` 布局。
 */
static void put_float_le(uint8_t *destination, float value)
{
    uint32_t bits = 0U;

    (void)memcpy(&bits, &value, sizeof(bits));
    destination[0] = (uint8_t)(bits & 0xFFU);
    destination[1] = (uint8_t)((bits >> 8U) & 0xFFU);
    destination[2] = (uint8_t)((bits >> 16U) & 0xFFU);
    destination[3] = (uint8_t)(bits >> 24U);
}

/**
 * @brief 将 32 位无符号数按小端序写入 4 字节协议字段。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[out] destination 至少 4 字节的可写目标区，不得为 `NULL`。
 * @param[in] value 待序列化的 32 位值。
 * @return 无返回值；不检查指针或剩余容量。
 * 调用方式：构造 `IMU_TELEMETRY` 的 32 位时间戳字段时调用。
 * 线程约束：纯内存写入、不阻塞，可在 ISR 调用；目标缓冲区的并发所有权由调用者保证。
 */
static void put_u32_le(uint8_t *destination, uint32_t value)
{
    destination[0] = (uint8_t)(value & 0xFFU);
    destination[1] = (uint8_t)((value >> 8U) & 0xFFU);
    destination[2] = (uint8_t)((value >> 16U) & 0xFFU);
    destination[3] = (uint8_t)(value >> 24U);
}

/**
 * @brief 按独立周期生成双 IMU 原始遥测与双 AHRS 姿态负载并交给 S3 服务。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] now_ms 当前单调毫秒时间，允许 32 位回绕。
 * @param[in,out] last_imu_ms 调用者所有的 IMU 遥测上次尝试时间；`NULL` 时禁用该类遥测，指针不被保留。
 * @param[in,out] last_attitude_ms 调用者所有的姿态遥测上次尝试时间；`NULL` 时禁用该类遥测，指针不被保留。
 * @return 无返回值；快照、打包或发送失败不上报。周期时间在获取/发送前已更新，
 * 因此失败后仍要等待下一个周期。
 * 调用方式：`imu_debug_task()` 每 10 ms 调用，内部对 IMU 使用 100 ms、对姿态使用 50 ms 门限。
 * 线程约束：仅限单一调试任务，不可在 ISR 调用；内部读取多个共享快照并可进入 S3/SRP 传输锁。
 * `s_dual_attitude_payload` 为模块共享缓冲，函数不可并发或重入调用，且默认发送层在返回前已复制负载。
 */
static void imu_send_telemetry(uint32_t now_ms, uint32_t *last_imu_ms,
                               uint32_t *last_attitude_ms)
{
    if (last_imu_ms != NULL &&
        (uint32_t)(now_ms - *last_imu_ms) >= IMU_TELEMETRY_IMU_PERIOD_MS) {
        uint8_t lsm_payload[30] = {0};
        uint8_t bmi_payload[30] = {0};
        imu_raw_data_t snapshot = {0};
        *last_imu_ms = now_ms;

        if (imu_manager_get_snapshot(&snapshot) == BSP_STATUS_OK) {
            lsm_payload[0] = SRP_IMU_SENSOR_LSM303;
            lsm_payload[1] = (uint8_t)(
                (snapshot.lsm_accel_valid != 0U
                     ? SRP_IMU_TELEMETRY_FLAG_ACCEL_VALID
                     : 0U) |
                (snapshot.lsm_mag_valid != 0U
                     ? SRP_IMU_TELEMETRY_FLAG_GYRO_OR_MAG_VALID
                     : 0U) |
                (snapshot.online != 0U ? SRP_IMU_TELEMETRY_FLAG_ONLINE : 0U));
            put_u32_le(&lsm_payload[2], snapshot.lsm_timestamp);
            put_float_le(&lsm_payload[6], snapshot.lsm_ax);
            put_float_le(&lsm_payload[10], snapshot.lsm_ay);
            put_float_le(&lsm_payload[14], snapshot.lsm_az);
            put_float_le(&lsm_payload[18], snapshot.lsm_mx);
            put_float_le(&lsm_payload[22], snapshot.lsm_my);
            put_float_le(&lsm_payload[26], snapshot.lsm_mz);
            s3_service_send_imu_telemetry(lsm_payload, (uint8_t)sizeof(lsm_payload));

            bmi_payload[0] = SRP_IMU_SENSOR_BMI323;
            bmi_payload[1] = (uint8_t)(
                (snapshot.bmi_accel_valid != 0U
                     ? SRP_IMU_TELEMETRY_FLAG_ACCEL_VALID
                     : 0U) |
                (snapshot.bmi_gyro_valid != 0U
                     ? SRP_IMU_TELEMETRY_FLAG_GYRO_OR_MAG_VALID
                     : 0U) |
                (bmi323_is_online() != 0U ? SRP_IMU_TELEMETRY_FLAG_ONLINE : 0U));
            put_u32_le(&bmi_payload[2], snapshot.bmi_timestamp);
            put_float_le(&bmi_payload[6], snapshot.bmi_ax);
            put_float_le(&bmi_payload[10], snapshot.bmi_ay);
            put_float_le(&bmi_payload[14], snapshot.bmi_az);
            put_float_le(&bmi_payload[18], snapshot.bmi_gx);
            put_float_le(&bmi_payload[22], snapshot.bmi_gy);
            put_float_le(&bmi_payload[26], snapshot.bmi_gz);
            s3_service_send_imu_telemetry(bmi_payload, (uint8_t)sizeof(bmi_payload));
        }
    }

    if (last_attitude_ms != NULL &&
        (uint32_t)(now_ms - *last_attitude_ms) >=
            IMU_TELEMETRY_ATTITUDE_PERIOD_MS) {
        *last_attitude_ms = now_ms;
        if (dual_ahrs_pack_payload(s_dual_attitude_payload,
                                   sizeof(s_dual_attitude_payload)) ==
            (int)DUAL_AHRS_PAYLOAD_LENGTH) {
            s3_service_send_dual_attitude(
                s_dual_attitude_payload, (uint8_t)DUAL_AHRS_PAYLOAD_LENGTH);
        }
    }
}

/**
 * @brief 将分类与消息合并为一条有界事件日志，并映射到现有日志级别。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] level 期望日志级别；ERROR/WARN 保留，DEBUG 与未知值当前都按 INFO 输出。
 * @param[in] category 仅在调用期间借用的 NUL 结尾分类文本；`NULL` 时静默返回。
 * @param[in] message 仅在调用期间借用的 NUL 结尾消息文本；`NULL` 时静默返回。
 * @return 无返回值；超长文本被 `snprintf()` 截断，日志后端失败不会上报。
 * 调用方式：初始化、状态快照和任务创建路径同步调用。
 * 线程约束：仅限任务上下文，不可在 ISR 调用；栈上占用 `SMARTCAR_LOG_MAX_PAYLOAD+1` 字节，并可在日志后端阻塞。
 */
static void imu_runtime_log_event(smartcar_log_level_t level,
                                  const char *category,
                                  const char *message)
{
    char event[SMARTCAR_LOG_MAX_PAYLOAD + 1U];

    if (category == NULL || message == NULL) {
        return;
    }
    (void)snprintf(event, sizeof(event), "%s %s", category, message);
    switch (level) {
    case SMARTCAR_LOG_LEVEL_ERROR:
        LOG_ERROR(event);
        break;
    case SMARTCAR_LOG_LEVEL_WARN:
        LOG_WARN(event);
        break;
    case SMARTCAR_LOG_LEVEL_DEBUG:
    case SMARTCAR_LOG_LEVEL_INFO:
    default:
        LOG_INFO(event);
        break;
    }
}

/**
 * @brief 保留原始文本诊断调用点，当前实现明确丢弃文本。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] text 兼容调用传入的文本指针；函数不读取、不复制也不保留该指针。
 * @return 无返回值；当前无论输入为何都不产生输出，也不累加 `imu_runtime_log_fail_count`。
 * 调用方式：旧版文本诊断和可选原始数据格式化路径保留调用；二进制遥测走 S3 服务。
 * 线程约束：空操作、可重入、不阻塞，当前可从任务或 ISR 任意上下文调用。
 */
static void imu_runtime_log(const char *text)
{
    /* Raw/text diagnostics stay disabled; binary telemetry uses the SC frame. */
    (void)text;
}

/**
 * @brief 读取指定 FreeRTOS 任务的历史最小剩余栈深度。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] task 有效且仍存活的 FreeRTOS 任务句柄；`NULL` 表示未创建/不可用。
 * @return `NULL` 时返回 0，否则返回 `uxTaskGetStackHighWaterMark()` 的栈单元数。
 * 调用方式：资源日志周期读取 IMU、调试和 BMI323 任务栈余量时调用。
 * 线程约束：仅限调度器运行后的任务上下文，不可在 ISR 调用；本层不等待模块互斥量，
 * 但 FreeRTOS 查询可进入内核临界区，且调用方需防止句柄与任务删除并发。
 */
static UBaseType_t imu_runtime_stack_high_water(TaskHandle_t task)
{
    return task == NULL ? 0U : uxTaskGetStackHighWaterMark(task);
}

/**
 * @brief 采集 FreeRTOS 剩余堆和三个 IMU 相关任务的栈高水位并输出 INFO 日志。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * 传入参数：无。
 * @return 无返回值；未找到 BMI323 任务时其高水位记为 0，日志失败不上报。
 * 调用方式：`imu_debug_task()` 按 `IMU_STACK_MONITOR_PERIOD_MS` 周期调用。
 * 线程约束：仅限 FreeRTOS 任务上下文，不可在 ISR 调用；内部按名称查找任务并可在日志后端阻塞，具有有界 CPU/栈开销。
 */
static void imu_runtime_log_resources(void)
{
    char line[SMARTCAR_LOG_MAX_PAYLOAD + 1U];
    const TaskHandle_t bmi_task = xTaskGetHandle("bmi323_task");

    (void)snprintf(line, sizeof(line),
                   "IMU_RES heap_free=%lu hwm_imu/dbg/bmi=%lu/%lu/%lu",
                   (unsigned long)xPortGetFreeHeapSize(),
                   (unsigned long)imu_runtime_stack_high_water(
                       s_imu_task_handle),
                   (unsigned long)imu_runtime_stack_high_water(
                       s_imu_debug_task_handle),
                   (unsigned long)imu_runtime_stack_high_water(bmi_task));
    LOG_INFO(line);
}

/**
 * @brief 在双 AHRS 处于 READY/TRACKING 时输出主备姿态及差值的角度日志。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * 传入参数：无。
 * @return 无返回值；非 READY/TRACKING 状态直接返回，格式化截断或日志失败不上报。
 * 调用方式：`imu_debug_task()` 按 `IMU_DUAL_AHRS_LOG_PERIOD_MS` 周期调用。
 * 线程约束：仅限任务上下文；内部读取双 AHRS 共享快照、执行浮点格式化并可在日志后端阻塞，不可在 ISR 调用。
 */
static void imu_runtime_log_dual_ahrs(void)
{
    dual_ahrs_output_t output = {0};
    char line[SMARTCAR_LOG_MAX_PAYLOAD + 1U];

    dual_ahrs_get_output(&output);
    if (output.state != DUAL_AHRS_STATE_READY &&
        output.state != DUAL_AHRS_STATE_TRACKING) {
        return;
    }
    (void)snprintf(
        line, sizeof(line),
        "[DUAL_AHRS] pri_deg=%.2f,%.2f,%.2f red_deg=%.2f,%.2f,%.2f diff_deg=%.2f,%.2f,%.2f",
        (double)(output.primary.roll * RAD_TO_DEG),
        (double)(output.primary.pitch * RAD_TO_DEG),
        (double)(output.primary.yaw * RAD_TO_DEG),
        (double)(output.redundant.roll * RAD_TO_DEG),
        (double)(output.redundant.pitch * RAD_TO_DEG),
        (double)(output.redundant.yaw * RAD_TO_DEG),
        (double)(output.delta_rad.x * RAD_TO_DEG),
        (double)(output.delta_rad.y * RAD_TO_DEG),
        (double)(output.delta_rad.z * RAD_TO_DEG));
    LOG_INFO(line);
}

/** 读取 IMU 日志发送失败累计值。 */
uint32_t imu_runtime_get_log_fail_count(void)
{
    return imu_runtime_log_fail_count;
}

/**
 * @brief 在有界字符缓冲区尾部追加一段 NUL 结尾文本。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in,out] buffer 容量为 `capacity` 的可写字符区；`NULL` 时不写入。
 * @param[in] capacity `buffer` 总容量，包含结尾 NUL 空间。
 * @param[in] offset 本次追加起始位置；大于等于容量时原样返回。
 * @param[in] text 仅在调用期间借用的源文本；`NULL` 时不写入。
 * @return 成功时返回新尾偏移；格式化失败返回原偏移；裁剪时返回 `capacity-1`。
 * 调用方式：调试原始数据和状态块按片段组装文本时链式调用。
 * 线程约束：使用 `snprintf()`、不适用于 ISR；本层不等待 RTOS 锁，但 C 库执行时间依赖格式化实现，
 * 缓冲区所有权与非重叠性由调用者保证。
 */
static size_t imu_append_text(char *buffer, size_t capacity, size_t offset,
                              const char *text)
{
    int written;

    if (buffer == NULL || text == NULL || offset >= capacity) {
        return offset;
    }
    written = snprintf(buffer + offset, capacity - offset, "%s", text);
    if (written <= 0) {
        return offset;
    }
    if ((size_t)written >= capacity - offset) {
        return capacity - 1U;
    }
    return offset + (size_t)written;
}

/**
 * @brief 将 32 位无符号数以可选 `label=` 前缀追加到有界文本缓冲区。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in,out] buffer 容量为 `capacity` 的可写字符区。
 * @param[in] capacity 缓冲区总容量，包含 NUL 空间。
 * @param[in] offset 本次追加起始偏移。
 * @param[in] label 仅在调用期间借用的标签；空字符串表示只输出数值，`NULL` 表示不写入。
 * @param[in] value 待以十进制输出的无符号值。
 * @return 成功时返回新尾偏移；无效输入/格式化失败返回原偏移；裁剪时返回 `capacity-1`。
 * 调用方式：状态文本组装时链式追加进度、样本数、更新计数和时间戳。
 * 线程约束：使用 `snprintf()`、不适用于 ISR；本层不等待 RTOS 锁，但 C 库执行时间依赖格式化实现，
 * 缓冲区所有权由调用者保证。
 */
static size_t imu_append_uint32(char *buffer, size_t capacity, size_t offset,
                                const char *label, uint32_t value)
{
    int written;

    if (buffer == NULL || label == NULL || offset >= capacity) {
        return offset;
    }
    written = label[0] == '\0'
                  ? snprintf(buffer + offset, capacity - offset, "%lu",
                             (unsigned long)value)
                  : snprintf(buffer + offset, capacity - offset, "%s=%lu",
                             label, (unsigned long)value);
    if (written <= 0) {
        return offset;
    }
    if ((size_t)written >= capacity - offset) {
        return capacity - 1U;
    }
    return offset + (size_t)written;
}

#if IMU_RUNTIME_ENABLE_RAW_DATA_LOG
/**
 * @brief 将浮点值按千分位四舍五入为 `label=±integer.mmm` 文本并追加。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in,out] buffer 容量为 `capacity` 的可写字符区。
 * @param[in] capacity 缓冲区总容量，包含 NUL 空间。
 * @param[in] offset 本次追加起始偏移。
 * @param[in] label 仅在调用期间借用的非空标签文本。
 * @param[in] value 待格式化的浮点值；当前实现不检查 NaN/无穷或缩放后超出 `uint32_t` 范围。
 * @return 成功时返回新尾偏移；无效输入/格式化失败返回原偏移；裁剪时返回 `capacity-1`。
 * 调用方式：仅在 `IMU_RUNTIME_ENABLE_RAW_DATA_LOG=1` 时，原始/标定/滤波数据块格式化中调用。
 * 线程约束：使用浮点运算与 `snprintf()`，仅限低频调试任务，不可在 ISR 调用；本层不等待 RTOS 锁。
 */
static size_t imu_append_milli(char *buffer, size_t capacity, size_t offset,
                               const char *label, float value)
{
    int written;
    const uint8_t negative = value < 0.0f ? 1U : 0U;
    const float magnitude = negative != 0U ? -value : value;
    const uint32_t scaled = (uint32_t)((magnitude * 1000.0f) + 0.5f);

    if (buffer == NULL || label == NULL || offset >= capacity) {
        return offset;
    }
    written = snprintf(buffer + offset, capacity - offset,
                       "%s=%s%lu.%03lu", label,
                       negative != 0U ? "-" : "",
                       (unsigned long)(scaled / 1000U),
                       (unsigned long)(scaled % 1000U));
    if (written <= 0) {
        return offset;
    }
    if ((size_t)written >= capacity - offset) {
        return capacity - 1U;
    }
    return offset + (size_t)written;
}
#endif

/**
 * @brief 将弧度角转为度并以两位小数追加到有界文本缓冲区。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in,out] buffer 容量为 `capacity` 的可写字符区。
 * @param[in] capacity 缓冲区总容量，包含 NUL 空间。
 * @param[in] offset 本次追加起始偏移。
 * @param[in] label 仅在调用期间借用的标签；`NULL` 时不写入。
 * @param[in] radians 待转换的弧度值；非有限值由 C 库按 `%f` 规则输出。
 * @return 成功时返回新尾偏移；无效输入/格式化失败返回原偏移；裁剪时返回 `capacity-1`。
 * 调用方式：状态块在 AHRS READY 时追加 roll/pitch/yaw 角度。
 * 线程约束：使用浮点格式化，仅限调试任务上下文，不可在 ISR 调用；本层不等待 RTOS 锁。
 */
static size_t imu_append_degree(char *buffer, size_t capacity, size_t offset,
                                const char *label, float radians)
{
    int written;
    const float degrees = radians * (180.0f / IMU_RUNTIME_PI);

    if (buffer == NULL || label == NULL || offset >= capacity) {
        return offset;
    }
    written = snprintf(buffer + offset, capacity - offset,
                       "%s=%.2f deg", label, (double)degrees);
    if (written <= 0) {
        return offset;
    }
    if ((size_t)written >= capacity - offset) {
        return capacity - 1U;
    }
    return offset + (size_t)written;
}

/**
 * @brief 将兼容 IMU 启动/标定状态映射为诊断名称。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] state 待转换的 `imu_boot_state_t` 值。
 * @return 指向只读静态字符串；未知枚举值返回 `"UNKNOWN"`。
 * 调用方式：标定状态事件和调试文本组装时调用；调用方不得修改或释放返回指针。
 * 线程约束：纯查表、可重入、不阻塞，不访问共享状态，可在 ISR 调用。
 */
static const char *imu_cal_state_name(imu_boot_state_t state)
{
    switch (state) {
    case IMU_BOOT_INIT: return "IMU_BOOT_INIT";
    case WAIT_SYNC: return "WAIT_SYNC";
    case SYNCED: return "SYNCED";
    case STATIC_CAL_WAIT: return "STATIC_CAL_WAIT";
    case STATIC_CAL_SAMPLE: return "STATIC_CAL_SAMPLE";
    case STATIC_CAL_DONE: return "STATIC_CAL_DONE";
    case IMU_READY: return "IMU_READY";
    case IMU_ERROR: return "IMU_ERROR";
    default:
        return "UNKNOWN";
    }
}

#if IMU_RUNTIME_ENABLE_RAW_DATA_LOG
/**
 * @brief 获取 LSM303、磁力计滤波、标定和 IMU 滤波快照，格式化为调试文本块。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * 传入参数：无。
 * @return 无返回值；任一必需快照不可用或 IMU 未 READY 时提前返回。
 * 当前 `imu_runtime_log()` 为空实现，因此即使开启宏也只产生格式化 CPU/栈开销，不实际输出文本。
 * 调用方式：仅在 `IMU_RUNTIME_ENABLE_RAW_DATA_LOG=1` 时由 `imu_debug_task()` 每 10 ms 调用。
 * 线程约束：仅限低优先级调试任务，不可在 ISR 调用；栈上仅文本块即占 768 字节，
 * 内部多次读取共享快照时可因互斥量阻塞，并执行大量浮点/`snprintf()` 格式化，会影响 CPU 时间和任务栈余量。
 */
static void imu_data_print(void)
{
    char block[768];
    lsm_accel_data_t accel = {0};
    lsm_mag_data_t mag = {0};
    mag_filter_data_t mag_filtered = {0};
    imu_sensor_stats_t stats = {0};
    imu_calibrated_data_t calibrated = {0};
    imu_filtered_data_t filtered = {0};
    imu_boot_state_t cal_state;
    size_t offset = 0U;

    if (imu_manager_get_lsm_accel(&accel) != BSP_STATUS_OK ||
        imu_manager_get_lsm_mag(&mag) != BSP_STATUS_OK ||
        imu_get_lsm303_stats(&stats) != BSP_STATUS_OK ||
        imu_is_ready() == 0U || stats.update_count == 0U) {
        return;
    }
    calibrated = imu_calibration_get_data();
    filtered = imu_filter_get_output();
    (void)mag_filter_get(&mag_filtered);
    cal_state = imu_boot_manager_get_state();
    if (cal_state != IMU_READY) {
        return;
    }

    offset = imu_append_text(block, sizeof(block), offset,
                             "[DATA][LSM_ACC] ");
    offset = imu_append_milli(block, sizeof(block), offset, "ax", accel.ax);
    offset = imu_append_text(block, sizeof(block), offset, " ");
    offset = imu_append_milli(block, sizeof(block), offset, "ay", accel.ay);
    offset = imu_append_text(block, sizeof(block), offset, " ");
    offset = imu_append_milli(block, sizeof(block), offset, "az", accel.az);
    offset = imu_append_text(block, sizeof(block), offset,
                             "\r\n[DATA][LSM_MAG] ");
    offset = imu_append_milli(block, sizeof(block), offset, "mx", mag.mx);
    offset = imu_append_text(block, sizeof(block), offset, " ");
    offset = imu_append_milli(block, sizeof(block), offset, "my", mag.my);
    offset = imu_append_text(block, sizeof(block), offset, " ");
    offset = imu_append_milli(block, sizeof(block), offset, "mz", mag.mz);
    offset = imu_append_text(block, sizeof(block), offset,
                             "\r\n[DATA][MAG_FILTER] ");
    offset = imu_append_milli(block, sizeof(block), offset, "mx", mag_filtered.mx);
    offset = imu_append_text(block, sizeof(block), offset, " ");
    offset = imu_append_milli(block, sizeof(block), offset, "my", mag_filtered.my);
    offset = imu_append_text(block, sizeof(block), offset, " ");
    offset = imu_append_milli(block, sizeof(block), offset, "mz", mag_filtered.mz);
    if (calibrated.online == 0U) {
        offset = imu_append_text(block, sizeof(block), offset,
                                 "\r\n[DATA][IMU_CAL] WAIT_CAL\r\n");
        block[sizeof(block) - 1U] = '\0';
        imu_runtime_log(block);
        return;
    }
    offset = imu_append_text(block, sizeof(block), offset,
                             "\r\n[DATA][IMU_CAL] ");
    offset = imu_append_milli(block, sizeof(block), offset, "ax", calibrated.ax);
    offset = imu_append_text(block, sizeof(block), offset, " ");
    offset = imu_append_milli(block, sizeof(block), offset, "ay", calibrated.ay);
    offset = imu_append_text(block, sizeof(block), offset, " ");
    offset = imu_append_milli(block, sizeof(block), offset, "az", calibrated.az);
    offset = imu_append_text(block, sizeof(block), offset, " ");
    offset = imu_append_milli(block, sizeof(block), offset, "mx", calibrated.mx);
    offset = imu_append_text(block, sizeof(block), offset, " ");
    offset = imu_append_milli(block, sizeof(block), offset, "my", calibrated.my);
    offset = imu_append_text(block, sizeof(block), offset, " ");
    offset = imu_append_milli(block, sizeof(block), offset, "mz", calibrated.mz);
    if (filtered.online == 0U) {
        offset = imu_append_text(block, sizeof(block), offset,
                                 "\r\n[DATA][IMU_FILTER] WAIT_CAL\r\n");
        block[sizeof(block) - 1U] = '\0';
        imu_runtime_log(block);
        return;
    }
    offset = imu_append_text(block, sizeof(block), offset,
                             "\r\n[DATA][IMU_FILTER] ");
    offset = imu_append_milli(block, sizeof(block), offset, "ax", filtered.ax);
    offset = imu_append_text(block, sizeof(block), offset, " ");
    offset = imu_append_milli(block, sizeof(block), offset, "ay", filtered.ay);
    offset = imu_append_text(block, sizeof(block), offset, " ");
    offset = imu_append_milli(block, sizeof(block), offset, "az", filtered.az);
    offset = imu_append_text(block, sizeof(block), offset, " ");
    offset = imu_append_milli(block, sizeof(block), offset, "mx", filtered.mx);
    offset = imu_append_text(block, sizeof(block), offset, " ");
    offset = imu_append_milli(block, sizeof(block), offset, "my", filtered.my);
    offset = imu_append_text(block, sizeof(block), offset, " ");
    offset = imu_append_milli(block, sizeof(block), offset, "mz", filtered.mz);
    offset = imu_append_text(block, sizeof(block), offset, "\r\n");
    block[sizeof(block) - 1U] = '\0';
    imu_runtime_log(block);
}
#endif

/**
 * @brief 聚合 LSM303、标定、滤波和 AHRS 状态，输出结构化事件并组装兼容调试块。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * 传入参数：无。
 * @return 无返回值；AHRS 未就绪时输出状态事件后提前返回，文本块经空实现 `imu_runtime_log()` 丢弃。
 * 调用方式：`imu_debug_task()` 按 `IMU_STATUS_PERIOD_MS` 周期调用，用于运行期状态可观测性。
 * 线程约束：仅限调试任务，不可在 ISR 调用；内部获取多个模块快照时可因互斥量阻塞，
 * 随后执行浮点格式化并调用日志后端，
 * 栈上使用 256 字节状态块和一个 `SMARTCAR_LOG_MAX_PAYLOAD+1` 事件缓冲。
 */
static void imu_status_print(void)
{
    char block[256];
    char log_line[SMARTCAR_LOG_MAX_PAYLOAD + 1U];
    attitude_state_t attitude;
    imu_sensor_stats_t stats = {0};
    imu_boot_state_t cal_state;
    imu_boot_status_t boot_status = {0};
    ahrs_state_t ahrs_state;
    uint32_t sample_count;
    uint32_t sample_total;
    uint8_t filter_ready;
    uint8_t ahrs_ready;
    size_t offset = 0U;

    (void)imu_get_lsm303_stats(&stats);
    imu_boot_manager_get_status(&boot_status);
    cal_state = boot_status.state;
    sample_count = boot_status.sample_count;
    sample_total = boot_status.sample_total;
    filter_ready = imu_filter_is_ready();
    ahrs_state = attitude_get_status();
    ahrs_ready = (cal_state == IMU_READY &&
                  filter_ready != 0U &&
                  ahrs_state == AHRS_READY) ? 1U : 0U;
    imu_runtime_log_event(imu_is_ready() != 0U ? SMARTCAR_LOG_LEVEL_INFO
                                                : SMARTCAR_LOG_LEVEL_ERROR,
                          "LSM303 STATUS",
                          imu_is_ready() != 0U ? "ONLINE" : "OFFLINE");
    if (cal_state == IMU_ERROR) {
        (void)snprintf(log_line, sizeof(log_line),
                       "reason=%s state=%s sample=%lu/%lu",
                       boot_status.error_reason != NULL
                           ? boot_status.error_reason
                           : "UNKNOWN",
                       imu_cal_state_name(cal_state),
                       (unsigned long)sample_count,
                       (unsigned long)sample_total);
        imu_runtime_log_event(SMARTCAR_LOG_LEVEL_ERROR,
                              "IMU CALIBRATION ERROR", log_line);
    } else {
        (void)snprintf(log_line, sizeof(log_line),
                       "state=%s progress=%u sample=%lu/%lu",
                       imu_cal_state_name(cal_state),
                       (unsigned)boot_status.progress,
                       (unsigned long)sample_count,
                       (unsigned long)sample_total);
        imu_runtime_log_event(SMARTCAR_LOG_LEVEL_INFO,
                              "IMU CALIBRATION", log_line);
    }
    imu_runtime_log_event(ahrs_ready != 0U ? SMARTCAR_LOG_LEVEL_INFO
                                            : SMARTCAR_LOG_LEVEL_WARN,
                          "ATTITUDE STATUS",
                          ahrs_ready != 0U ? "READY" : "WAIT_CAL");
    offset = imu_append_text(block, sizeof(block), offset, "[IMU_STATUS] ");
    offset = imu_append_text(block, sizeof(block), offset,
                             imu_is_ready() != 0U ? "LSM303=OK " : "LSM303=FAIL ");
    offset = imu_append_text(block, sizeof(block), offset, "cal_state=");
    offset = imu_append_text(block, sizeof(block), offset,
                             imu_cal_state_name(cal_state));
    offset = imu_append_uint32(block, sizeof(block), offset, " progress",
                               boot_status.progress);
    if (cal_state == STATIC_CAL_WAIT || cal_state == STATIC_CAL_SAMPLE) {
        offset = imu_append_text(block, sizeof(block), offset, " sample=");
        offset = imu_append_uint32(block, sizeof(block), offset, "",
                                   sample_count);
        offset = imu_append_text(block, sizeof(block), offset, "/");
        offset = imu_append_uint32(block, sizeof(block), offset, "",
                                   sample_total);
    } else if (cal_state == IMU_READY) {
        offset = imu_append_text(block, sizeof(block), offset,
                                 filter_ready != 0U
                                     ? " filter_state=READY"
                                     : " filter_state=WAIT_CAL");
    } else {
        offset = imu_append_uint32(block, sizeof(block), offset, " raw_count",
                                   stats.update_count);
        offset = imu_append_text(block, sizeof(block), offset, " ");
        offset = imu_append_uint32(block, sizeof(block), offset, "timestamp",
                                   stats.last_update_ms);
        offset = imu_append_text(block, sizeof(block), offset,
                                 filter_ready != 0U
                                     ? " filter_state=READY\r\n"
                                     : " filter_state=WAIT_CAL\r\n");
    }
    if (cal_state == STATIC_CAL_WAIT || cal_state == STATIC_CAL_SAMPLE ||
        cal_state == IMU_READY) {
        offset = imu_append_text(block, sizeof(block), offset, "\r\n");
    }
    offset = imu_append_text(block, sizeof(block), offset, "AHRS_LSM=");
    offset = imu_append_text(block, sizeof(block), offset,
                             ahrs_ready != 0U ? "READY\r\n"
                                              : "WAIT_CAL\r\n");
    if (ahrs_ready == 0U) {
        block[sizeof(block) - 1U] = '\0';
        imu_runtime_log(block);
        return;
    }
    attitude = attitude_get_state();
    offset = imu_append_text(block, sizeof(block), offset,
                             "[AHRS_LSM]\r\n");
    offset = imu_append_degree(block, sizeof(block), offset, "roll",
                               attitude.roll);
    offset = imu_append_text(block, sizeof(block), offset, "\r\n");
    offset = imu_append_degree(block, sizeof(block), offset, "pitch",
                               attitude.pitch);
    offset = imu_append_text(block, sizeof(block), offset, "\r\n");
    offset = imu_append_degree(block, sizeof(block), offset, "yaw",
                               attitude.yaw);
    offset = imu_append_text(block, sizeof(block), offset, "\r\n");
    block[sizeof(block) - 1U] = '\0';
    imu_runtime_log(block);
}

/** 创建 IMU 采样任务和调试任务；失败时由 RTOS 健康路径记录。 */
void imu_runtime_start(void)
{
    char init_status_line[48];
    bsp_status_t init_status;
    BaseType_t task_status;

#if !SMARTCAR_BMI323_DEBUG_ONLY
    attitude_init();
    dual_ahrs_init();
#endif
    imu_runtime_log("[INFO] IMU_INIT_BEGIN\r\n");
#if SMARTCAR_BMI323_DEBUG_ONLY
    imu_runtime_log_event(SMARTCAR_LOG_LEVEL_INFO, "BMI323 DEBUG", "BEGIN");
#else
    imu_runtime_log_event(SMARTCAR_LOG_LEVEL_INFO, "IMU INIT", "BEGIN");
#endif
    init_status = imu_init();
    (void)snprintf(init_status_line, sizeof(init_status_line),
                   "[INFO] IMU_INIT_DONE status=%d\r\n", (int)init_status);
    imu_runtime_log(init_status_line);
    (void)snprintf(init_status_line, sizeof(init_status_line),
                   "status=%d", (int)init_status);
    imu_runtime_log_event(init_status == BSP_STATUS_OK
                              ? SMARTCAR_LOG_LEVEL_INFO
                              : SMARTCAR_LOG_LEVEL_ERROR,
#if SMARTCAR_BMI323_DEBUG_ONLY
                          init_status == BSP_STATUS_OK ? "BMI323 INIT" : "BMI323 ERROR",
#else
                          init_status == BSP_STATUS_OK ? "IMU INIT" : "ERROR",
#endif
                          init_status == BSP_STATUS_OK ? "COMPLETE" : init_status_line);

#if SMARTCAR_BMI323_DEBUG_ONLY
    const bsp_status_t bmi_task_status = imu_manager_start_bmi323_task();
    if (bmi_task_status != BSP_STATUS_OK) {
        boot_log("TASK", "BMI323_TASK CREATE FAIL");
        imu_runtime_log("[INFO] BMI323 TASK CREATE FAIL\r\n");
        imu_runtime_log_event(SMARTCAR_LOG_LEVEL_ERROR, "ERROR",
                              "BMI323 TASK CREATE FAIL");
    } else {
        boot_log("TASK", "BMI323_TASK CREATE OK");
        imu_runtime_log("[INFO] TASK_CREATE_OK task=bmi323_task\r\n");
    }
#else
    /* DUAL_IMU_BOOT starts the independent BMI task only after both INIT
     * workers have completed successfully. */
    imu_runtime_log("[INFO] BMI323_TASK deferred=DUAL_IMU_INIT\r\n");
#endif

    task_status = xTaskCreate(imu_task, "imu_task", IMU_DATA_STACK_WORDS,
                              NULL, IMU_SAMPLE_PRIORITY, &s_imu_task_handle);
    if (task_status != pdPASS) {
        boot_log("TASK", "IMU_TASK CREATE FAIL");
        imu_runtime_log("[INFO] IMU TASK CREATE FAIL status=sample\r\n");
        imu_runtime_log_event(SMARTCAR_LOG_LEVEL_ERROR, "ERROR",
                              "IMU TASK CREATE FAIL");
    } else {
        boot_log("TASK", "IMU_TASK CREATE OK");
        imu_runtime_log("[INFO] TASK_CREATE_OK task=imu_task\r\n");
    }
    task_status = xTaskCreate(imu_debug_task, "imu_data_logger",
                              IMU_DATA_STACK_WORDS, NULL,
                              IMU_DATA_PRIORITY, &s_imu_debug_task_handle);
    if (task_status != pdPASS) {
        boot_log("TASK", "DEBUG_TASK CREATE FAIL");
        imu_runtime_log("[INFO] IMU TASK CREATE FAIL status=data_logger\r\n");
        imu_runtime_log_event(SMARTCAR_LOG_LEVEL_ERROR, "ERROR",
                              "IMU LOGGER TASK CREATE FAIL");
    } else {
        boot_log("TASK", "DEBUG_TASK CREATE OK");
        imu_runtime_log("[INFO] TASK_CREATE_OK task=imu_data_logger\r\n");
    }
}

/** 低频 IMU 调试任务：输出状态快照，不参与控制闭环。 */
void imu_debug_task(void *argument)
{
#if SMARTCAR_BMI323_DEBUG_ONLY
    (void)argument;
    /* BMI323 RAW diagnostics are emitted once during imu_init(). */
    for (;;) {
        vTaskDelay(portMAX_DELAY);
    }
#else
    TickType_t last_wake;
    uint32_t last_status_ms;
    uint32_t last_stack_monitor_ms;
    uint32_t last_imu_telemetry_ms;
    uint32_t last_attitude_telemetry_ms;
    uint32_t last_dual_ahrs_log_ms;

    (void)argument;
    imu_runtime_log("[INFO] SCHEDULER_RUNNING\r\n");
    last_wake = xTaskGetTickCount();
    last_status_ms = imu_time_now_ms();
    last_stack_monitor_ms = last_status_ms;
    last_imu_telemetry_ms = last_status_ms;
    last_attitude_telemetry_ms = last_status_ms;
    last_dual_ahrs_log_ms = last_status_ms;
    for (;;) {
        const uint32_t now_ms = imu_time_now_ms();
#if IMU_RUNTIME_ENABLE_RAW_DATA_LOG
        imu_data_print();
#endif
        imu_send_telemetry(now_ms, &last_imu_telemetry_ms,
                           &last_attitude_telemetry_ms);
        if ((uint32_t)(now_ms - last_status_ms) >= IMU_STATUS_PERIOD_MS) {
            last_status_ms = now_ms;
            imu_status_print();
        }
        if ((uint32_t)(now_ms - last_stack_monitor_ms) >=
            IMU_STACK_MONITOR_PERIOD_MS) {
            last_stack_monitor_ms = now_ms;
            imu_runtime_log_resources();
        }
        if ((uint32_t)(now_ms - last_dual_ahrs_log_ms) >=
            IMU_DUAL_AHRS_LOG_PERIOD_MS) {
            last_dual_ahrs_log_ms = now_ms;
            imu_runtime_log_dual_ahrs();
        }
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(IMU_TELEMETRY_TICK_MS));
    }
#endif
}
