#include "attitude_startup_coordinator.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"

#include "stm32h7xx_hal.h"

#include "attitude.h"
#include "chassis_state_task.h"
#include "dual_ahrs.h"
#include "imu_boot_manager.h"
#include "imu_filter.h"
#include "imu_manager.h"
#include "log_service.h"
#include "motor_board_task.h"
#include "smartcar_debug_config.h"

/* 姿态启动安全协调实现；创建人：待确认（当前维护人：Zhiqin）。 */

#ifndef SMARTCAR_MOTOR_BOARD_ONLY
#define SMARTCAR_MOTOR_BOARD_ONLY 0
#endif

#define ATTITUDE_STARTUP_STACK_WORDS UINT16_C(256)
#define ATTITUDE_STARTUP_PRIORITY (tskIDLE_PRIORITY + 3U)
#define ATTITUDE_STARTUP_PERIOD_MS UINT32_C(20)
#define ATTITUDE_STARTUP_STABLE_CYCLES UINT8_C(5)

volatile uint8_t g_attitude_is_ready;

static TaskHandle_t s_task_handle;

/**
 * @brief 按限频周期输出当前 IMU boot 锁定阶段、进度和错误原因。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] now_ms 当前 HAL tick，单位 ms；使用无符号差值处理回绕。
 * @return 无返回值；距离上次日志不足配置周期时不读取状态；格式化截断或日志队列丢弃
 *         不向调用方报告。
 * 调用方式：仅姿态启动协调任务在保持 MotorBoard 锁定的各失败路径调用。
 * 线程约束：函数内静态限频时间仅由协调任务拥有；会读取 boot 状态、执行 `snprintf()`
 *           并非阻塞式提交日志，不显式获取 mutex，禁止 ISR/并发调用，无指针所有权。
 */
static void startup_log_locked_state(uint32_t now_ms)
{
    static uint32_t last_log_ms;
    imu_boot_status_t status = {0};
    char line[SMARTCAR_LOG_MAX_PAYLOAD + 1U];

    if ((uint32_t)(now_ms - last_log_ms) < ATTITUDE_STARTUP_LOG_PERIOD_MS) {
        return;
    }
    last_log_ms = now_ms;
    imu_boot_manager_get_status(&status);
    (void)snprintf(line, sizeof(line),
                   "[ATTITUDE_STARTUP] LOCKED state=%u phase=%u progress=%u "
                   "error=%u reason=%s",
                   (unsigned)status.state, (unsigned)status.phase,
                   (unsigned)status.progress, (unsigned)status.error,
                   status.error_reason == NULL ? "NONE" : status.error_reason);
    if (status.error != (uint8_t)IMU_ERROR_NONE) {
        LOG_ERROR(line);
    } else {
        LOG_INFO(line);
    }
}

/**
 * @brief 汇总 IMU boot、manager、filter、zero、AHRS 状态和主姿态 freshness 软件门。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * 传入参数：无。
 * @return 本次顺序读取全部通过时返回 true；任一错误、未就绪或主姿态失鲜返回 false；
 *         true 仅表示瞬时软件门满足，仍需外层连续 update_count 稳定周期确认。
 * 调用方式：协调任务每 20 ms 调用；READY 后也持续复查，失败立即触发重新锁定。
 * 线程约束：仅普通任务上下文；部分 getter 可能获取各自 mutex，但本函数没有跨模块总锁，
 *           读取并非原子事务且可能短暂阻塞，禁止 ISR，无输入对象或所有权转移。
 */
static bool attitude_lifecycle_is_ready(void)
{
    if (imu_boot_manager_is_error() != 0U ||
        imu_boot_manager_is_ready() == 0U || imu_is_ready() == 0U ||
        imu_filter_is_ready() == 0U || attitude_zero_is_ready() == 0U ||
        attitude_get_status() != AHRS_READY ||
        dual_ahrs_is_primary_fresh() == 0U) {
        return false;
    }
    return true;
}

/**
 * @brief 检查 LSM303 与 BMI323 update_count 是否都严格前进，并更新调用方比较基线。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in,out] last_lsm_updates 调用方拥有的 LSM 上次计数，不可为 NULL；成功读取统计后更新。
 * @param[in,out] last_bmi_updates 调用方拥有的 BMI 上次计数，不可为 NULL；成功读取统计后更新。
 * @param[in,out] have_baseline 调用方拥有的基线有效标志，不可为 NULL；首次成功时置 true。
 * @return 首次建立基线或两计数均严格前进返回 true；参数无效、getter 失败、停滞或回退
 *         返回 false；停滞/回退时仍把两个计数基线更新为当前值。
 * 调用方式：仅协调任务在 lifecycle 瞬时 ready 后调用；false 会由外层重置稳定周期和基线。
 * 线程约束：统计 getter 可能使用内部锁，本函数不持有跨传感器 mutex，两个快照不保证同刻；
 *           仅任务单 owner 调用，可能短暂阻塞，禁止 ISR，不保存或接管三个指针。
 */
static bool attitude_updates_are_advancing(uint32_t *last_lsm_updates,
                                           uint32_t *last_bmi_updates,
                                           bool *have_baseline)
{
    imu_sensor_stats_t lsm_stats = {0};
    imu_sensor_stats_t bmi_stats = {0};

    if (last_lsm_updates == NULL || last_bmi_updates == NULL ||
        have_baseline == NULL ||
        imu_get_lsm303_stats(&lsm_stats) != BSP_STATUS_OK ||
        imu_get_bmi323_stats(&bmi_stats) != BSP_STATUS_OK) {
        return false;
    }
    if (*have_baseline &&
        (lsm_stats.update_count <= *last_lsm_updates ||
         bmi_stats.update_count <= *last_bmi_updates)) {
        *last_lsm_updates = lsm_stats.update_count;
        *last_bmi_updates = bmi_stats.update_count;
        return false;
    }
    *last_lsm_updates = lsm_stats.update_count;
    *last_bmi_updates = bmi_stats.update_count;
    *have_baseline = true;
    return true;
}

/** 监视 IMU/姿态 freshness，只有稳定通过后才释放 MotorBoard。 */
void attitude_startup_coordinator_task(void *argument)
{
    TickType_t last_wake;
    uint32_t last_lsm_updates = 0U;
    uint32_t last_bmi_updates = 0U;
    uint8_t stable_cycles = 0U;
    bool have_baseline = false;

    (void)argument;
    g_attitude_is_ready = 0U;
    last_wake = xTaskGetTickCount();
    for (;;) {
        const uint32_t now_ms = HAL_GetTick();

        if (g_attitude_is_ready != 0U) {
            if (!attitude_lifecycle_is_ready()) {
                g_attitude_is_ready = 0U;
                stable_cycles = 0U;
                have_baseline = false;
                (void)motor_board_force_stop();
                LOG_WARN("[ATTITUDE_STARTUP] freshness lost; motor locked");
            }
            vTaskDelayUntil(&last_wake,
                            pdMS_TO_TICKS(ATTITUDE_STARTUP_PERIOD_MS));
            continue;
        }

#if SMARTCAR_MOTOR_BOARD_ONLY
        /* This task is not started for the motor-board-only image. Keep the
         * guard here so an accidental call can never unlock that image. */
        (void)motor_board_force_stop();
        startup_log_locked_state(now_ms);
#else
        if (!attitude_lifecycle_is_ready() ||
            !attitude_updates_are_advancing(&last_lsm_updates,
                                            &last_bmi_updates,
                                            &have_baseline)) {
            stable_cycles = 0U;
            have_baseline = false;
            (void)motor_board_force_stop();
            startup_log_locked_state(now_ms);
        } else if (stable_cycles < ATTITUDE_STARTUP_STABLE_CYCLES) {
            ++stable_cycles;
        }

        if (stable_cycles >= ATTITUDE_STARTUP_STABLE_CYCLES) {
            /* The task itself clears its internal stop latch during its
             * initialization. It is deliberately created only here, after
             * the attitude gate, so no command can produce PWM earlier. */
            g_attitude_is_ready = 1U;
            motor_board_task_start();
            if (xTaskGetHandle("motor_board") == NULL) {
                g_attitude_is_ready = 0U;
                stable_cycles = 0U;
                have_baseline = false;
                (void)motor_board_force_stop();
                LOG_ERROR("[ATTITUDE_STARTUP] MOTOR_UNLOCK failed task_missing");
            } else {
                chassis_state_task_start();
                LOG_INFO("[ATTITUDE_STARTUP] READY; MOTOR_UNLOCK task_started");
            }
        }
#endif
        vTaskDelayUntil(&last_wake,
                        pdMS_TO_TICKS(ATTITUDE_STARTUP_PERIOD_MS));
    }
}

/** 创建唯一姿态启动协调任务；创建失败保持电机停止。 */
void attitude_startup_coordinator_start(void)
{
#if SMARTCAR_MOTOR_BOARD_ONLY
    g_attitude_is_ready = 0U;
    return;
#else
    if (s_task_handle != NULL) {
        return;
    }
    g_attitude_is_ready = 0U;
    if (xTaskCreate(attitude_startup_coordinator_task,
                    "attitude_gate", ATTITUDE_STARTUP_STACK_WORDS, NULL,
                    ATTITUDE_STARTUP_PRIORITY, &s_task_handle) != pdPASS) {
        s_task_handle = NULL;
        (void)motor_board_force_stop();
        LOG_ERROR("[ATTITUDE_STARTUP] coordinator task create failed");
    }
#endif
}
