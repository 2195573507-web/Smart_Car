#include "imu_boot_manager.h"

/* 双 IMU 启动/标定状态机实现；创建人：待确认（当前维护人：Zhiqin）。 */

#include <stdio.h>
#include <string.h>

#include "imu_manager.h"
#include "imu_time.h"
#include "log_service.h"
#include "srp_registry.h"

#if defined(IMU_MANAGER_USE_FREERTOS)
#include "FreeRTOS.h"
#include "semphr.h"
#endif

#define DUAL_IMU_INIT_TIMEOUT_MS UINT32_C(5000)
#define DUAL_IMU_SELF_TEST_WINDOW_MS UINT32_C(1000)
#define DUAL_IMU_STATIC_SETTLE_MS UINT32_C(2000)
#define DUAL_IMU_STATIC_PWM_WAIT_MS UINT32_C(30000)
#define DUAL_IMU_STATIC_TIMEOUT_MARGIN_MS UINT32_C(5000)
#define DUAL_IMU_STATIC_MAX_RESTARTS UINT32_C(3)
#define DUAL_IMU_STATUS_PERIOD_MS UINT32_C(200)
#define DUAL_IMU_BOOT_READY_PERIOD_MS UINT32_C(500)
#define DUAL_IMU_PRIMARY_GRAVITY_MIN_MPS2 (9.4f)
#define DUAL_IMU_PRIMARY_GRAVITY_MAX_MPS2 (10.2f)
#define DUAL_IMU_STATIC_PHASE_TIMEOUT_MS \
    (DUAL_IMU_STATIC_PWM_WAIT_MS + DUAL_IMU_STATIC_SETTLE_MS + \
     ((IMU_CAL_STATIC_WINDOW_MS + DUAL_IMU_STATIC_SETTLE_MS) * \
      (DUAL_IMU_STATIC_MAX_RESTARTS + UINT32_C(1))) + \
     DUAL_IMU_STATIC_TIMEOUT_MARGIN_MS)

/* These are the frozen legacy 0x0202 presentation counters. They are not
 * used as calibration quality gates. */
#define IMU_COMPAT_STATIC_SAMPLE_TOTAL UINT32_C(1000)
#define IMU_STAGE_READY SRP_IMU_CAL_STAGE_COMPLETE

typedef struct
{
    dual_imu_manager_t dual;
    imu_boot_state_t state;
    uint8_t lsm_phase_complete;
    uint8_t bmi_phase_complete;
    uint8_t self_test_lsm_seen;
    uint8_t self_test_bmi_seen;
    uint8_t static_zero_ready;
    uint8_t static_window_started;
    uint8_t static_result_ready;
    uint8_t static_restart_count;
    uint32_t static_window_start_ms;
    uint32_t settle_until_ms;
    uint32_t phase_deadline_ms;
    uint32_t next_boot_ready_ms;
    uint32_t last_status_tx_ms;
    uint8_t status_tx_initialized;
    imu_boot_transport_callback_t transport;
} dual_imu_boot_state_t;

static dual_imu_boot_state_t s_boot;

#if defined(IMU_MANAGER_USE_FREERTOS)
static SemaphoreHandle_t s_mutex;
#endif

/**
 * @brief 获取启动状态机互斥量，保护 `s_boot` 的复合读写。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * 传入参数：无。
 * @return 无返回值；非 FreeRTOS 构建或互斥量未创建时直接返回。
 * 调用方式：在访问 `s_boot` 前调用，并与 `unlock_boot()` 成对。
 * 线程约束：仅限任务上下文；FreeRTOS 下可按 `portMAX_DELAY`
 * 永久阻塞，不可在 ISR 调用，也不可在同一任务重入获取非递归互斥量。
 */
static void lock_boot(void)
{
#if defined(IMU_MANAGER_USE_FREERTOS)
    if (s_mutex != NULL) {
        (void)xSemaphoreTake(s_mutex, portMAX_DELAY);
    }
#endif
}

/**
 * @brief 释放由当前任务持有的启动状态机互斥量。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * 传入参数：无。
 * @return 无返回值；非 FreeRTOS 构建或互斥量未创建时直接返回。
 * 调用方式：仅在成功执行 `lock_boot()` 的路径末尾成对调用。
 * 线程约束：仅限当前持锁任务，不可在 ISR 中调用；释放路径不等待、正常情况下不阻塞。
 * 函数不检查所有权，未持锁或跨任务释放属于调用方错误。
 */
static void unlock_boot(void)
{
#if defined(IMU_MANAGER_USE_FREERTOS)
    if (s_mutex != NULL) {
        (void)xSemaphoreGive(s_mutex);
    }
#endif
}

/**
 * @brief 在 FreeRTOS 构建中按需创建启动状态机互斥量。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * 传入参数：无。
 * @return 无返回值；创建失败时 `s_mutex` 保持 `NULL`，本函数不上报错误。
 * 调用方式：由初始化、复位或注册传输回调的公共入口在首次持锁前调用。
 * 线程约束：需在调度器可用的任务上下文串行调用，不可在 ISR 中调用；
 * 本函数不等待其他锁，但可执行 FreeRTOS 堆分配；自身无创建锁，并发首次调用可能重复分配内核对象。
 */
static void ensure_boot_mutex(void)
{
#if defined(IMU_MANAGER_USE_FREERTOS)
    if (s_mutex == NULL) {
        s_mutex = xSemaphoreCreateMutex();
    }
#endif
}

/**
 * @brief 使用有符号差值判断 32 位毫秒时钟是否到达截止点。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] now_ms 当前单调毫秒计数，允许 `uint32_t` 回绕。
 * @param[in] deadline_ms 与 `now_ms` 同一时钟域的截止时间。
 * @return 已到达或越过截止点返回 1，否则返回 0。
 * 调用方式：用于启动、自检和静态标定状态机的周期超时判定。
 * 线程约束：纯计算、不阻塞、不访问共享状态，可在 ISR 调用；时间间隔必须小于 `INT32_MAX` 毫秒。
 */
static uint8_t time_reached(uint32_t now_ms, uint32_t deadline_ms)
{
    return (int32_t)(now_ms - deadline_ms) >= 0 ? 1U : 0U;
}

/**
 * @brief 将已用时间换算为限制在 0..100 的阶段进度。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] now_ms 当前单调毫秒时间。
 * @param[in] start_ms 阶段起始毫秒时间。
 * @param[in] duration_ms 阶段总时长，0 表示直接视为完成。
 * @return 向下取整的进度百分数；时长为 0 或已超时返回 100。
 * 调用方式：由 `update_progress_locked()` 根据当前阶段时间计算进度。
 * 线程约束：纯计算、不阻塞，可在 ISR 调用；调用方保证两个时间来自同一 32 位单调时钟。
 */
static uint8_t elapsed_percent(uint32_t now_ms, uint32_t start_ms,
                               uint32_t duration_ms)
{
    const uint32_t elapsed_ms = now_ms - start_ms;
    uint32_t percent;

    if (duration_ms == 0U || elapsed_ms >= duration_ms) {
        return 100U;
    }
    percent = ((uint64_t)elapsed_ms * UINT32_C(100)) / duration_ms;
    return (uint8_t)(percent > 100U ? 100U : percent);
}

/**
 * @brief 按时间线性折算兼容协议展示用的虚拟样本数。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] now_ms 当前单调毫秒时间。
 * @param[in] start_ms 静态窗口起始毫秒时间。
 * @param[in] duration_ms 窗口总时长，0 表示立即达到总量。
 * @param[in] total 兼容层对外展示的样本总数。
 * @return 限制在 0..`total` 的虚拟样本数，不代表实际采集数。
 * 调用方式：由状态查询在静态窗口中生成冻结的旧协议计数。
 * 线程约束：纯计算、不阻塞，可在 ISR 调用；调用方负责时钟域一致性。
 */
static uint32_t virtual_sample_count(uint32_t now_ms, uint32_t start_ms,
                                     uint32_t duration_ms, uint32_t total)
{
    const uint32_t elapsed_ms = now_ms - start_ms;
    const uint64_t count = duration_ms == 0U || elapsed_ms >= duration_ms
                               ? total
                               : ((uint64_t)elapsed_ms * total) / duration_ms;

    return count > total ? total : (uint32_t)count;
}

/**
 * @brief 将 IMU 启动错误枚举映射为简短诊断字符串。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] error 当前 `imu_error_t` 错误值。
 * @return 指向只读静态字符串；未知值与 `IMU_ERROR_NONE` 均返回 `"NONE"`。
 * 调用方式：由日志路径和状态查询生成人类可读原因；调用方不得修改或释放返回指针。
 * 线程约束：纯查表、可重入、不阻塞，不访问共享状态，可在 ISR 调用。
 */
static const char *imu_error_name(imu_error_t error)
{
    switch (error) {
    case IMU_ERROR_LSM_INIT: return "LSM_INIT";
    case IMU_ERROR_BMI_INIT: return "BMI_INIT";
    case IMU_ERROR_INIT_TIMEOUT: return "INIT_TIMEOUT";
    case IMU_ERROR_LSM_SELF_TEST: return "LSM_SELF_TEST";
    case IMU_ERROR_BMI_SELF_TEST: return "BMI_SELF_TEST";
    case IMU_ERROR_RADAR_SYNC_TIMEOUT: return "RADAR_SYNC_TIMEOUT";
    case IMU_ERROR_STATIC_WINDOW: return "STATIC_WINDOW";
    case IMU_ERROR_TASK_CREATE: return "TASK_CREATE";
    case IMU_ERROR_NONE:
    default: return "NONE";
    }
}

/**
 * @brief 将非空启动诊断文本以 INFO 级别交给现有日志服务。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] text 仅在本次调用期间借用的 NUL 结尾文本；`NULL` 时静默忽略。
 * @return 无返回值；本层不上报日志裁剪、队列满或输出失败。
 * 调用方式：由启动状态转换和质量摘要函数同步调用。
 * 线程约束：仅限任务上下文，不可在 ISR 中调用；实时性和阻塞性取决于日志后端。
 */
static void boot_log(const char *text)
{
    if (text != NULL) {
        LOG_INFO(text);
    }
}

/**
 * @brief 读取静态标定质量快照并格式化为一条 INFO 日志。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * 传入参数：无。
 * @return 无返回值；格式化超长时由 `snprintf()` 截断，日志失败不会上报。
 * 调用方式：静态窗口结束后调用，输出配置频率、实际样本数、最小样本数和质量标志。
 * 线程约束：仅限任务上下文；内部获取标定互斥量并可在日志后端阻塞，不可在 ISR 或持有标定锁时调用。
 */
static void boot_log_static_quality(void)
{
    char line[SMARTCAR_LOG_MAX_PAYLOAD + 1U];
    const imu_calibration_quality_t quality = imu_calibration_get_quality();

    (void)snprintf(line, sizeof(line),
                   "IMU_CAL cfg=%u/%u act=%lu/%lu/%lu min=%lu/%lu/%lu "
                   "q=%u/%u/%u",
                   (unsigned)quality.lsm_accel.configured_rate_hz,
                   (unsigned)quality.bmi_accel.configured_rate_hz,
                   (unsigned long)quality.lsm_accel.actual_sample_count,
                   (unsigned long)quality.bmi_accel.actual_sample_count,
                   (unsigned long)quality.bmi_gyro.actual_sample_count,
                   (unsigned long)quality.lsm_accel.minimum_sample_count,
                   (unsigned long)quality.bmi_accel.minimum_sample_count,
                   (unsigned long)quality.bmi_gyro.minimum_sample_count,
                   (unsigned)quality.lsm_accel.quality_ok,
                   (unsigned)quality.bmi_accel.quality_ok,
                   (unsigned)quality.bmi_gyro.quality_ok);
    boot_log(line);
}

/**
 * @brief 读取 BMI323 与 LSM303 水平标定状态并分别输出诊断日志。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * 传入参数：无。
 * @return 无返回值；格式化或日志输出失败均不向上传递。
 * 调用方式：静态标定结果被冻结后调用，用于记录 `valid`、本地重力、倾角和回退原因。
 * 线程约束：仅限任务上下文；内部读取 IMU 管理器共享快照并可在锁/日志后端阻塞，不可在 ISR 调用。
 */
static void boot_log_leveling(void)
{
    char line[SMARTCAR_LOG_MAX_PAYLOAD + 1U];
    imu_leveling_state_t bmi = {0};
    imu_leveling_state_t lsm = {0};

    imu_manager_get_leveling_states(&bmi, &lsm);
    (void)snprintf(line, sizeof(line),
                   "[LEVELING] BMI valid=%d g=%.2f tilt=%.2f reason=%d",
                   (int)bmi.valid, (double)bmi.g_local_mps2,
                   (double)bmi.tilt_deg, (int)bmi.fallback_reason);
    boot_log(line);
    (void)snprintf(line, sizeof(line),
                   "[LEVELING] LSM valid=%d g=%.2f tilt=%.2f reason=%d",
                   (int)lsm.valid, (double)lsm.g_local_mps2,
                   (double)lsm.tilt_deg, (int)lsm.fallback_reason);
    boot_log(line);
}

/**
 * @brief 检查静态标定后的水平化状态是否允许主姿态链继续。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * 传入参数：无。
 * @return BMI323 水平化有效、无回退且重力范整在 9.4..10.2 m/s^2 时返回 1；
 * 否则返回 0。LSM303 降级只记录日志，当前实现仍返回 1。
 * 调用方式：静态标定完成后，在进入 READY 前作为主传感器门禁。
 * 线程约束：仅限任务上下文；内部读取共享快照，可在锁或 LSM303 降级日志后端阻塞，不可在 ISR 调用。
 */
static uint8_t leveling_states_are_ready(void)
{
    imu_leveling_state_t bmi = {0};
    imu_leveling_state_t lsm = {0};

    imu_manager_get_leveling_states(&bmi, &lsm);
    if (bmi.valid == 0U || bmi.fallback_reason != IMU_LEVELING_FALLBACK_NONE ||
        bmi.g_local_mps2 < DUAL_IMU_PRIMARY_GRAVITY_MIN_MPS2 ||
        bmi.g_local_mps2 > DUAL_IMU_PRIMARY_GRAVITY_MAX_MPS2) {
        return 0U;
    }
    if (lsm.valid == 0U || lsm.fallback_reason != IMU_LEVELING_FALLBACK_NONE) {
        char line[SMARTCAR_LOG_MAX_PAYLOAD + 1U];
        (void)snprintf(line, sizeof(line),
                       "DUAL_IMU_BOOT LSM leveling degraded valid=%u g=%.2f reason=%u",
                       (unsigned)lsm.valid, (double)lsm.g_local_mps2,
                       (unsigned)lsm.fallback_reason);
        boot_log(line);
    }
    return 1U;
}

/**
 * @brief 依据双 IMU 阶段和静态窗口标志刷新旧版兼容状态。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * 传入参数：无。
 * @return 无返回值；未识别阶段按 `IMU_BOOT_INIT` 处理。
 * 调用方式：阶段或静态标定子状态变化后，在持有启动锁时调用。
 * 线程约束：调用者必须已持有 `s_mutex` 或处于非 FreeRTOS 串行环境；
 * 函数不再获取锁、不阻塞，不可独立从 ISR 调用。
 */
static void update_compat_state_locked(void)
{
    switch (s_boot.dual.phase) {
    case IMU_PHASE_STATIC_CALIBRATION:
        if (s_boot.static_result_ready != 0U) {
            s_boot.state = STATIC_CAL_DONE;
        } else if (s_boot.static_window_started != 0U) {
            s_boot.state = STATIC_CAL_SAMPLE;
        } else {
            s_boot.state = s_boot.static_zero_ready != 0U
                               ? STATIC_CAL_WAIT
                               : WAIT_SYNC;
        }
        break;
    case IMU_PHASE_READY:
        s_boot.state = IMU_READY;
        break;
    case IMU_PHASE_FAILED:
        s_boot.state = IMU_ERROR;
        break;
    case IMU_PHASE_IDLE:
    case IMU_PHASE_INIT:
    case IMU_PHASE_SELF_TEST:
    default:
        s_boot.state = IMU_BOOT_INIT;
        break;
    }
}

/**
 * @brief 根据当前生命周期阶段更新总进度、分传感器进度与诊断标志。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] now_ms 与阶段起始时间同域的当前单调毫秒值。
 * @return 无返回值；IDLE/FAILED 不重算总进度，但仍刷新分路进度与标志。
 * 调用方式：由阶段切换、复位和周期步进逻辑在更改状态后调用。
 * 线程约束：调用者必须已持有启动状态锁；函数只做定长计算，不阻塞、不可在 ISR 独立调用。
 */
static void update_progress_locked(uint32_t now_ms)
{
    uint8_t phase_progress = 0U;

    switch (s_boot.dual.phase) {
    case IMU_PHASE_INIT:
        phase_progress = s_boot.lsm_phase_complete != 0U &&
                                 s_boot.bmi_phase_complete != 0U
                             ? 100U
                             : elapsed_percent(now_ms,
                                               s_boot.dual.phase_start_time,
                                               DUAL_IMU_INIT_TIMEOUT_MS);
        s_boot.dual.overall_progress = (uint8_t)((phase_progress * 2U) / 100U);
        break;
    case IMU_PHASE_SELF_TEST:
        phase_progress = elapsed_percent(now_ms, s_boot.dual.phase_start_time,
                                         DUAL_IMU_SELF_TEST_WINDOW_MS);
        s_boot.dual.overall_progress =
            (uint8_t)(2U + ((uint32_t)phase_progress * 3U) / 100U);
        break;
    case IMU_PHASE_STATIC_CALIBRATION:
        if (s_boot.static_result_ready != 0U) {
            phase_progress = 100U;
            s_boot.dual.overall_progress = 100U;
        } else if (s_boot.static_window_started != 0U) {
            phase_progress = elapsed_percent(now_ms, s_boot.static_window_start_ms,
                                             IMU_CAL_STATIC_WINDOW_MS);
            s_boot.dual.overall_progress =
                (uint8_t)(5U + ((uint32_t)phase_progress * 90U) / 100U);
        } else {
            s_boot.dual.overall_progress = 5U;
        }
        break;
    case IMU_PHASE_READY:
        phase_progress = 100U;
        s_boot.dual.overall_progress = 100U;
        break;
    case IMU_PHASE_FAILED:
    case IMU_PHASE_IDLE:
    default:
        break;
    }

    s_boot.dual.lsm_progress = s_boot.lsm_phase_complete != 0U
                                   ? 100U
                                   : phase_progress;
    s_boot.dual.bmi_progress = s_boot.bmi_phase_complete != 0U
                                   ? 100U
                                   : phase_progress;
    /* Keep the dual lifecycle flags available for local diagnostics. */
    s_boot.dual.flags = 0U;
    if (s_boot.lsm_phase_complete != 0U) {
        s_boot.dual.flags |= DUAL_IMU_STATUS_FLAG_LSM_PHASE_COMPLETE;
    }
    if (s_boot.bmi_phase_complete != 0U) {
        s_boot.dual.flags |= DUAL_IMU_STATUS_FLAG_BMI_PHASE_COMPLETE;
    }
    if (s_boot.dual.phase != IMU_PHASE_READY &&
        s_boot.dual.phase != IMU_PHASE_FAILED) {
        s_boot.dual.flags |= DUAL_IMU_STATUS_FLAG_PHASE_ACTIVE;
    }
}

/**
 * @brief 在持锁条件下结束前一阶段并初始化新阶段计时与进度。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] phase 目标生命周期阶段；超出 `IMU_PHASE_COUNT` 时不写入阶段计时数组。
 * @param[in] now_ms 阶段边界的单调毫秒时间。
 * @return 无返回值；同时清除 LSM/BMI 阶段完成标志并刷新兼容状态。
 * 调用方式：状态机需进入 INIT、SELF_TEST、STATIC_CALIBRATION、READY 或 FAILED 时调用。
 * 线程约束：调用者必须已持有启动状态锁；函数不重复加锁、不阻塞，不可从 ISR 调用。
 */
static void enter_phase_locked(imu_phase_t phase, uint32_t now_ms)
{
    const imu_phase_t previous = s_boot.dual.phase;

    if (previous < IMU_PHASE_COUNT &&
        s_boot.dual.phase_timing[previous].end_timestamp == 0U) {
        s_boot.dual.phase_timing[previous].end_timestamp = now_ms;
    }
    s_boot.dual.phase_end_time = now_ms;
    s_boot.dual.phase = phase;
    s_boot.dual.phase_start_time = now_ms;
    s_boot.dual.phase_end_time = 0U;
    if (phase < IMU_PHASE_COUNT) {
        s_boot.dual.phase_timing[phase].start_timestamp = now_ms;
        s_boot.dual.phase_timing[phase].end_timestamp = 0U;
    }
    s_boot.lsm_phase_complete = 0U;
    s_boot.bmi_phase_complete = 0U;
    update_compat_state_locked();
    update_progress_locked(now_ms);
}

/**
 * @brief 在持锁条件下进入不可运动的失败阶段并冻结原进度。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] error 要锁定到 `s_boot.dual.error` 的失败原因。
 * @param[in] now_ms 失败发生时的单调毫秒时间。
 * @return 无返回值；已处于 FAILED 时保留首个错误并直接返回。
 * 调用方式：各启动、自检、静态窗口失败分支在持锁期间调用。
 * 线程约束：调用者必须已持有启动状态锁；函数不执行传输或日志，不阻塞，不可从 ISR 调用。
 */
static void fail_locked(imu_error_t error, uint32_t now_ms)
{
    const uint8_t progress = s_boot.dual.overall_progress;

    if (s_boot.dual.phase == IMU_PHASE_FAILED) {
        return;
    }
    s_boot.dual.error = error;
    enter_phase_locked(IMU_PHASE_FAILED, now_ms);
    s_boot.dual.overall_progress = progress;
}

/**
 * @brief 复制已注册的传输回调，解锁后同步交付一条 SRP 消息。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] message_id SRP v4 消息 ID。
 * @param[in] flags SRP 帧标志，由上层选择流或需 ACK 语义。
 * @param[in] payload 仅在回调执行期间借用的负载；`length` 非 0 时必须指向可读内存。
 * @param[in] length 负载字节数。
 * @return 未注册回调返回 0；已调用回调返回 1。由于回调无返回值，1 不证明实际发送成功。
 * 调用方式：由状态、事件和 BOOT_READY 封装函数在不持有启动锁时调用。
 * 线程约束：仅限任务上下文，不可在 ISR 调用；内部短暂获取启动锁，回调在锁外同步执行，
 * 阻塞性由回调实现决定，回调不得在返回后保留栈上 `payload` 指针。
 */
static uint8_t send_message(uint16_t message_id, uint8_t flags,
                            const uint8_t *payload, uint8_t length)
{
    imu_boot_transport_callback_t transport;

    lock_boot();
    transport = s_boot.transport;
    unlock_boot();
    if (transport == NULL) {
        return 0U;
    }
    transport(message_id, flags, payload, length);
    return 1U;
}

/**
 * @brief 将 32 位无符号数按小端序列化为 4 字节。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[out] destination 至少 4 字节的可写目标区，不得为 `NULL`。
 * @param[in] value 待序列化的 32 位值。
 * @return 无返回值；不检查空指针或容量。
 * 调用方式：构造 11 字节 `IMU_CAL_STATUS` 负载的样本计数字段时调用。
 * 线程约束：纯内存写入、不阻塞，可在 ISR 调用；目标缓冲区的并发所有权由调用者保证。
 */
static void put_u32_le(uint8_t *destination, uint32_t value)
{
    destination[0] = (uint8_t)(value & UINT32_C(0xFF));
    destination[1] = (uint8_t)((value >> 8U) & UINT32_C(0xFF));
    destination[2] = (uint8_t)((value >> 16U) & UINT32_C(0xFF));
    destination[3] = (uint8_t)(value >> 24U);
}

/**
 * @brief 将当前双 IMU 生命周期投影为旧版 0x0202 标定阶段值。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * 传入参数：无。
 * @return `SRP_IMU_CAL_STAGE_*` 阶段值；IDLE/INIT/SELF_TEST 和未知阶段投影为等待雷达就绪。
 * 调用方式：由 `send_cal_status()` 在组装兼容负载时调用。
 * 线程约束：调用者必须已持有启动状态锁；函数不加锁、不阻塞，不可独立从 ISR 调用。
 */
static uint8_t legacy_cal_stage_locked(void)
{
    switch (s_boot.dual.phase) {
    case IMU_PHASE_STATIC_CALIBRATION:
        if (s_boot.static_window_started != 0U) {
            return SRP_IMU_CAL_STAGE_STATIC_SAMPLE;
        }
        return s_boot.static_zero_ready != 0U
                   ? SRP_IMU_CAL_STAGE_STATIC_STABLE_WAIT
                   : SRP_IMU_CAL_STAGE_WAIT_RADAR_READY;
    case IMU_PHASE_READY:
        return IMU_STAGE_READY;
    case IMU_PHASE_FAILED:
        return SRP_IMU_CAL_STAGE_ERROR;
    case IMU_PHASE_IDLE:
    case IMU_PHASE_INIT:
    case IMU_PHASE_SELF_TEST:
    default:
        return SRP_IMU_CAL_STAGE_WAIT_RADAR_READY;
    }
}

/**
 * @brief 构造并尝试发送当前 11 字节 `IMU_CAL_STATUS` 兼容负载。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * 传入参数：无。
 * @return 无返回值；未注册传输回调或底层发送失败均被静默丢弃。
 * 调用方式：由 `imu_boot_manager_step()` 按 200 ms 节流标志在锁外调用。
 * 线程约束：仅限任务上下文；内部多次获取启动锁并同步执行可阻塞的传输回调，
 * 因此不可在 ISR 或已持有启动锁时调用。
 */
static void send_cal_status(void)
{
    imu_boot_status_t status = {0};
    uint8_t payload[11];
    uint8_t stage;

    imu_boot_manager_get_status(&status);
    lock_boot();
    stage = legacy_cal_stage_locked();
    unlock_boot();
    payload[0] = stage;
    payload[1] = 0U;
    put_u32_le(&payload[2], status.sample_count);
    put_u32_le(&payload[6], status.sample_total);
    payload[10] = status.error;
    (void)send_message(SRP_MSG_ID_IMU_CAL_STATUS, SRP_FLAG_STREAM_DATA,
                       payload, (uint8_t)sizeof(payload));
}

/**
 * @brief 将单字节标定事件以需 ACK 的 SRP 消息交给传输回调。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] event_id `SRP_MSG_ID_CAL_EVENT` 的单字节事件 ID。
 * @return 无返回值；传输回调缺失或发送失败不会上报，本函数也不等待 ACK。
 * 调用方式：静态窗口完成后用 `SRP_CAL_EVENT_STATIC_DONE` 调用。
 * 线程约束：仅限任务上下文，不可在 ISR 调用；局部负载仅在同步回调期间有效，阻塞性由传输回调决定。
 */
static void send_cal_event(uint8_t event_id)
{
    const uint8_t payload[1] = {event_id};
    (void)send_message(SRP_MSG_ID_CAL_EVENT, SRP_FLAG_ACK_REQUIRED,
                       payload, (uint8_t)sizeof(payload));
}

/**
 * @brief 尝试发送需 ACK 的 `BOOT_READY`，通知 S3 可进入零 PWM 同步流程。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * 传入参数：无。
 * @return 无返回值；未注册回调或发送失败不会上报，本函数不等待 S3 ACK。
 * 调用方式：初次进入静态标定阶段及等待期间按 500 ms 周期重发。
 * 线程约束：仅限任务上下文；局部 2 字节负载仅在同步回调期间有效，回调可阻塞，不可在 ISR 调用。
 */
static void send_stm_boot_ready(void)
{
    const uint8_t payload[2] = {(uint8_t)WAIT_SYNC, 0U};
    (void)send_message(SRP_MSG_ID_BOOT_READY, SRP_FLAG_ACK_REQUIRED,
                       payload, (uint8_t)sizeof(payload));
}

/**
 * @brief 在持锁条件下初始化静态标定阶段、重试计数和同步超时。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] now_ms 进入静态标定阶段时的单调毫秒值。
 * @return 无返回值；清除窗口/结果标志，并将 BOOT_READY 下次重发时间设为 500 ms 后。
 * 调用方式：SELF_TEST 窗口同时观察到两路有效样本后调用。
 * 线程约束：调用者必须已持有启动状态锁；函数不发送帧、不阻塞，不可从 ISR 调用。
 */
static void enter_static_phase_locked(uint32_t now_ms)
{
    enter_phase_locked(IMU_PHASE_STATIC_CALIBRATION, now_ms);
    s_boot.static_zero_ready = 0U;
    s_boot.static_window_started = 0U;
    s_boot.static_result_ready = 0U;
    s_boot.static_restart_count = 0U;
    s_boot.phase_deadline_ms = now_ms + DUAL_IMU_STATIC_PHASE_TIMEOUT_MS;
    s_boot.next_boot_ready_ms = now_ms + DUAL_IMU_BOOT_READY_PERIOD_MS;
    update_compat_state_locked();
    update_progress_locked(now_ms);
}

/** 复位状态机到初始阶段并清除一次性事件。 */
void imu_boot_manager_reset(void)
{
    imu_boot_transport_callback_t transport;
    const uint32_t now_ms = imu_time_now_ms();

    ensure_boot_mutex();
    lock_boot();
    transport = s_boot.transport;
    (void)memset(&s_boot, 0, sizeof(s_boot));
    s_boot.transport = transport;
    s_boot.dual.phase = IMU_PHASE_IDLE;
    s_boot.dual.error = IMU_ERROR_NONE;
    s_boot.dual.phase_start_time = now_ms;
    s_boot.dual.phase_timing[IMU_PHASE_IDLE].start_timestamp = now_ms;
    s_boot.state = IMU_BOOT_INIT;
    update_progress_locked(now_ms);
    unlock_boot();

    imu_calibration_start();
}

/** 初始化互斥量、状态和阶段计时。 */
void imu_boot_manager_init(void)
{
    imu_boot_manager_reset();
    boot_log("DUAL_IMU_BOOT IDLE");
}

/** 注册跨芯片状态发送回调；回调只应复制/入队。 */
void imu_boot_manager_set_transport(imu_boot_transport_callback_t callback)
{
    ensure_boot_mutex();
    lock_boot();
    s_boot.transport = callback;
    unlock_boot();
}

/** 推进一次启动/标定状态机，并执行有限超时处理。 */
void imu_boot_manager_step(void)
{
    const uint64_t now_us = imu_time_now_us();
    const uint32_t now_ms = (uint32_t)(now_us / UINT64_C(1000));
    imu_dual_init_status_t init_status = {0};
    uint8_t start_init = 0U;
    uint8_t finalize_init = 0U;
    uint8_t start_static = 0U;
    uint8_t finish_static = 0U;
    uint8_t reset_static = 0U;
    uint8_t static_reset_accepted = 0U;
    uint8_t send_boot_ready_now = 0U;
    uint8_t send_static_result = 0U;
    uint8_t send_status = 0U;

    lock_boot();
    if (s_boot.dual.phase == IMU_PHASE_IDLE) {
        enter_phase_locked(IMU_PHASE_INIT, now_ms);
        s_boot.phase_deadline_ms = now_ms + DUAL_IMU_INIT_TIMEOUT_MS;
        start_init = 1U;
    }
    if (s_boot.status_tx_initialized == 0U ||
        (uint32_t)(now_ms - s_boot.last_status_tx_ms) >=
            DUAL_IMU_STATUS_PERIOD_MS) {
        s_boot.status_tx_initialized = 1U;
        s_boot.last_status_tx_ms = now_ms;
        send_status = 1U;
    }
    unlock_boot();

    if (start_init != 0U) {
        if (imu_manager_start_dual_initialization() != BSP_STATUS_OK) {
            lock_boot();
            fail_locked(IMU_ERROR_TASK_CREATE, now_ms);
            unlock_boot();
        } else {
            boot_log("DUAL_IMU_BOOT INIT workers released");
        }
    }

    imu_manager_get_dual_initialization_status(&init_status);
    lock_boot();
    if (s_boot.dual.phase == IMU_PHASE_INIT) {
        s_boot.lsm_phase_complete =
            init_status.lsm_complete != 0U && init_status.lsm_success != 0U;
        s_boot.bmi_phase_complete =
            init_status.bmi_complete != 0U && init_status.bmi_success != 0U;
        if (init_status.lsm_complete != 0U && init_status.lsm_success == 0U) {
            fail_locked(IMU_ERROR_LSM_INIT, now_ms);
        } else if (init_status.bmi_complete != 0U &&
                   init_status.bmi_success == 0U) {
            fail_locked(IMU_ERROR_BMI_INIT, now_ms);
        } else if (s_boot.lsm_phase_complete != 0U &&
                   s_boot.bmi_phase_complete != 0U) {
            finalize_init = 1U;
        } else if (time_reached(now_ms, s_boot.phase_deadline_ms) != 0U) {
            fail_locked(IMU_ERROR_INIT_TIMEOUT, now_ms);
        }
        update_progress_locked(now_ms);
    }
    unlock_boot();

    if (finalize_init != 0U) {
        if (imu_manager_finalize_dual_initialization() == 0U) {
            lock_boot();
            fail_locked(IMU_ERROR_TASK_CREATE, now_ms);
            unlock_boot();
        } else {
            lock_boot();
            if (s_boot.dual.phase == IMU_PHASE_INIT) {
                enter_phase_locked(IMU_PHASE_SELF_TEST, now_ms);
                s_boot.phase_deadline_ms = now_ms + DUAL_IMU_SELF_TEST_WINDOW_MS;
                s_boot.self_test_lsm_seen = 0U;
                s_boot.self_test_bmi_seen = 0U;
            }
            unlock_boot();
            boot_log("DUAL_IMU_BOOT SELF_TEST window opened");
        }
    }

    lock_boot();
    if (s_boot.dual.phase == IMU_PHASE_SELF_TEST &&
        time_reached(now_ms, s_boot.phase_deadline_ms) != 0U) {
        s_boot.lsm_phase_complete = s_boot.self_test_lsm_seen;
        s_boot.bmi_phase_complete = s_boot.self_test_bmi_seen;
        if (s_boot.lsm_phase_complete != 0U &&
            s_boot.bmi_phase_complete != 0U) {
            enter_static_phase_locked(now_ms);
            send_boot_ready_now = 1U;
        } else {
            fail_locked(s_boot.self_test_lsm_seen == 0U
                            ? IMU_ERROR_LSM_SELF_TEST
                            : IMU_ERROR_BMI_SELF_TEST,
                        now_ms);
        }
    }
    if (s_boot.dual.phase == IMU_PHASE_STATIC_CALIBRATION) {
        if (s_boot.static_result_ready == 0U &&
            time_reached(now_ms, s_boot.phase_deadline_ms) != 0U) {
            fail_locked(s_boot.static_zero_ready == 0U
                            ? IMU_ERROR_RADAR_SYNC_TIMEOUT
                            : IMU_ERROR_STATIC_WINDOW,
                        now_ms);
        } else if (s_boot.static_window_started != 0U &&
                   imu_calibration_static_motion_detected() != 0U) {
            reset_static = 1U;
        } else if (s_boot.static_window_started != 0U &&
                   imu_calibration_window_expired(now_us) != 0U) {
            finish_static = 1U;
        } else if (s_boot.static_zero_ready != 0U &&
                   s_boot.static_window_started == 0U &&
                   time_reached(now_ms, s_boot.settle_until_ms) != 0U) {
            s_boot.static_window_started = 1U;
            s_boot.static_window_start_ms = now_ms;
            update_compat_state_locked();
            start_static = 1U;
        } else if (s_boot.static_zero_ready == 0U &&
                   time_reached(now_ms, s_boot.next_boot_ready_ms) != 0U) {
            s_boot.next_boot_ready_ms = now_ms + DUAL_IMU_BOOT_READY_PERIOD_MS;
            send_boot_ready_now = 1U;
        }
    }
    update_compat_state_locked();
    update_progress_locked(now_ms);
    unlock_boot();

    if (send_boot_ready_now != 0U) {
        send_stm_boot_ready();
    }
    if (start_static != 0U) {
        imu_calibration_start();
        imu_calibration_begin_window(
            now_us, (uint16_t)imu_manager_get_bmi323_sample_rate());
        boot_log("DUAL_IMU_BOOT STATIC_CALIBRATION window opened");
    }
    if (reset_static != 0U) {
        boot_log_static_quality();
        boot_log_leveling();
        imu_calibration_start();
        lock_boot();
        if (s_boot.dual.phase == IMU_PHASE_STATIC_CALIBRATION) {
            if (s_boot.static_restart_count >= DUAL_IMU_STATIC_MAX_RESTARTS) {
                fail_locked(IMU_ERROR_STATIC_WINDOW, now_ms);
            } else {
                ++s_boot.static_restart_count;
                s_boot.static_zero_ready = 1U;
                s_boot.static_window_started = 0U;
                s_boot.static_result_ready = 0U;
                s_boot.settle_until_ms = now_ms + DUAL_IMU_STATIC_SETTLE_MS;
                s_boot.next_boot_ready_ms = now_ms +
                                            DUAL_IMU_BOOT_READY_PERIOD_MS;
                update_compat_state_locked();
                update_progress_locked(now_ms);
                static_reset_accepted = 1U;
            }
        }
        unlock_boot();
        if (static_reset_accepted != 0U) {
            boot_log("DUAL_IMU_BOOT static motion; calibration reset");
        } else {
            boot_log("DUAL_IMU_BOOT repeated static motion; calibration failed");
        }
    }
    if (finish_static != 0U) {
        uint8_t lsm_done =
            imu_calibration_finish_window(now_us) != 0U &&
            imu_calibration_is_lsm_complete() != 0U;
        uint8_t bmi_done = imu_calibration_is_bmi_complete();
        const uint8_t static_motion =
            imu_calibration_static_motion_detected();

        if (static_motion != 0U) {
            /* A dynamic sample invalidates the bias and leveling reference.
             * Restart the local window while retaining the already-admitted
             * zero-PWM radar synchronization; no motor task exists yet. */
            boot_log_static_quality();
            boot_log_leveling();
            imu_calibration_start();
            lock_boot();
            if (s_boot.dual.phase == IMU_PHASE_STATIC_CALIBRATION) {
                if (s_boot.static_restart_count >= DUAL_IMU_STATIC_MAX_RESTARTS) {
                    fail_locked(IMU_ERROR_STATIC_WINDOW, now_ms);
                } else {
                    ++s_boot.static_restart_count;
                    s_boot.static_zero_ready = 1U;
                    s_boot.static_window_started = 0U;
                    s_boot.static_result_ready = 0U;
                    s_boot.settle_until_ms = now_ms + DUAL_IMU_STATIC_SETTLE_MS;
                    s_boot.next_boot_ready_ms = now_ms +
                                                DUAL_IMU_BOOT_READY_PERIOD_MS;
                    update_compat_state_locked();
                    update_progress_locked(now_ms);
                    static_reset_accepted = 1U;
                }
            }
            unlock_boot();
            if (static_reset_accepted != 0U) {
                boot_log("DUAL_IMU_BOOT static motion; calibration reset");
            } else {
                boot_log("DUAL_IMU_BOOT repeated static motion; calibration failed");
            }
            finish_static = 0U;
        }

        if (finish_static != 0U && lsm_done != 0U && bmi_done != 0U) {
            imu_manager_commit_leveling();
            if (leveling_states_are_ready() == 0U) {
                lsm_done = 0U;
                bmi_done = 0U;
                boot_log("DUAL_IMU_BOOT leveling rejected");
            }
        }
        if (static_motion == 0U) {
            boot_log_static_quality();
            boot_log_leveling();
        }

        lock_boot();
        if (s_boot.dual.phase == IMU_PHASE_STATIC_CALIBRATION &&
            s_boot.static_window_started != 0U) {
            s_boot.lsm_phase_complete = lsm_done;
            s_boot.bmi_phase_complete = bmi_done;
            s_boot.static_window_started = 0U;
            if (lsm_done != 0U && bmi_done != 0U) {
                s_boot.static_result_ready = 1U;
                enter_phase_locked(IMU_PHASE_READY, now_ms);
                s_boot.lsm_phase_complete = 1U;
                s_boot.bmi_phase_complete = 1U;
                update_progress_locked(now_ms);
                send_static_result = 1U;
            } else {
                fail_locked(IMU_ERROR_STATIC_WINDOW, now_ms);
            }
        }
        unlock_boot();
        if (send_static_result != 0U) {
            send_cal_status();
            send_cal_event(SRP_CAL_EVENT_STATIC_DONE);
            boot_log("DUAL_IMU_BOOT READY");
            boot_log_leveling();
        }
    }
    if (send_status != 0U) {
        send_cal_status();
    }
}

/** 在采样阶段消费最新原始快照。 */
void imu_boot_manager_update(const imu_raw_data_t *raw_data)
{
    const uint32_t now_ms = imu_time_now_ms();

    if (raw_data == NULL) {
        return;
    }
    lock_boot();
    if (s_boot.dual.phase == IMU_PHASE_SELF_TEST) {
        if (raw_data->lsm_accel_valid != 0U && raw_data->lsm_mag_valid != 0U) {
            s_boot.self_test_lsm_seen = 1U;
        }
        if (raw_data->bmi_accel_valid != 0U && raw_data->bmi_gyro_valid != 0U) {
            s_boot.self_test_bmi_seen = 1U;
        }
    }
    if (s_boot.dual.phase == IMU_PHASE_STATIC_CALIBRATION &&
        s_boot.static_window_started != 0U) {
        unlock_boot();
        imu_calibration_update(raw_data);
        return;
    }
    update_progress_locked(now_ms);
    unlock_boot();
}

/** 处理 S3 雷达 PWM READY 事件；无效状态不会跳过生命周期。 */
uint8_t imu_boot_manager_on_radar_pwm_ready(uint8_t speed)
{
    uint8_t accepted = 0U;
    const uint32_t now_ms = imu_time_now_ms();

    lock_boot();
    if (s_boot.dual.phase == IMU_PHASE_STATIC_CALIBRATION && speed == 0U &&
        s_boot.static_result_ready == 0U) {
        if (s_boot.static_zero_ready == 0U) {
            s_boot.static_zero_ready = 1U;
            s_boot.settle_until_ms = now_ms + DUAL_IMU_STATIC_SETTLE_MS;
        }
        accepted = 1U;
    }
    update_compat_state_locked();
    update_progress_locked(now_ms);
    unlock_boot();
    return accepted;
}

imu_boot_state_t imu_boot_manager_get_state(void)
{
    imu_boot_state_t state;

    lock_boot();
    state = s_boot.state;
    unlock_boot();
    return state;
}

/** 复制兼容启动状态摘要。 */
void imu_boot_manager_get_status(imu_boot_status_t *status)
{
    if (status == NULL) {
        return;
    }
    lock_boot();
    status->state = s_boot.state;
    status->phase = s_boot.dual.phase;
    status->progress = s_boot.dual.overall_progress;
    status->lsm_progress = s_boot.dual.lsm_progress;
    status->bmi_progress = s_boot.dual.bmi_progress;
    status->error = (uint8_t)s_boot.dual.error;
    status->error_reason = imu_error_name(s_boot.dual.error);
    if (s_boot.dual.phase == IMU_PHASE_STATIC_CALIBRATION) {
        status->sample_total = IMU_COMPAT_STATIC_SAMPLE_TOTAL;
        status->sample_count = s_boot.static_result_ready != 0U
                                   ? IMU_COMPAT_STATIC_SAMPLE_TOTAL
                                   : (s_boot.static_window_started != 0U
                                          ? virtual_sample_count(
                                                imu_time_now_ms(),
                                                s_boot.static_window_start_ms,
                                                IMU_CAL_STATIC_WINDOW_MS,
                                                IMU_COMPAT_STATIC_SAMPLE_TOTAL)
                                          : 0U);
    } else if (s_boot.dual.phase == IMU_PHASE_READY) {
        status->sample_count = IMU_COMPAT_STATIC_SAMPLE_TOTAL;
        status->sample_total = IMU_COMPAT_STATIC_SAMPLE_TOTAL;
        status->error = (uint8_t)IMU_ERROR_NONE;
        status->error_reason = imu_error_name(IMU_ERROR_NONE);
    } else {
        status->sample_count = 0U;
        status->sample_total = 0U;
    }
    unlock_boot();
}

/** 复制双 IMU 阶段和计时摘要。 */
void imu_boot_manager_get_dual_status(dual_imu_manager_t *status)
{
    if (status == NULL) {
        return;
    }
    lock_boot();
    *status = s_boot.dual;
    unlock_boot();
}

/** 读取指定阶段时间区间。 */
uint8_t imu_boot_manager_get_phase_timing(imu_phase_t phase,
                                          imu_phase_timing_t *timing)
{
    if (timing == NULL || phase >= IMU_PHASE_COUNT) {
        return 0U;
    }
    lock_boot();
    *timing = s_boot.dual.phase_timing[phase];
    unlock_boot();
    return 1U;
}

/** 查询是否到达 IMU_READY。 */
uint8_t imu_boot_manager_is_ready(void)
{
    uint8_t ready;

    lock_boot();
    ready = s_boot.dual.phase == IMU_PHASE_READY ? 1U : 0U;
    unlock_boot();
    return ready;
}

/** 查询是否到达 IMU_ERROR。 */
uint8_t imu_boot_manager_is_error(void)
{
    uint8_t failed;

    lock_boot();
    failed = s_boot.dual.phase == IMU_PHASE_FAILED ? 1U : 0U;
    unlock_boot();
    return failed;
}

/** 读取当前生命周期进度百分数。 */
uint8_t imu_boot_manager_get_progress(void)
{
    uint8_t progress;

    lock_boot();
    progress = s_boot.dual.overall_progress;
    unlock_boot();
    return progress;
}

/** 读取已接受标定样本数。 */
uint32_t imu_boot_manager_get_sample_count(void)
{
    imu_boot_status_t status = {0};

    imu_boot_manager_get_status(&status);
    return status.sample_count;
}

/** 读取目标标定样本总数。 */
uint32_t imu_boot_manager_get_sample_total(void)
{
    imu_boot_status_t status = {0};

    imu_boot_manager_get_status(&status);
    return status.sample_total;
}
