#include "imu_boot_manager.h"

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

static void lock_boot(void)
{
#if defined(IMU_MANAGER_USE_FREERTOS)
    if (s_mutex != NULL) {
        (void)xSemaphoreTake(s_mutex, portMAX_DELAY);
    }
#endif
}

static void unlock_boot(void)
{
#if defined(IMU_MANAGER_USE_FREERTOS)
    if (s_mutex != NULL) {
        (void)xSemaphoreGive(s_mutex);
    }
#endif
}

static void ensure_boot_mutex(void)
{
#if defined(IMU_MANAGER_USE_FREERTOS)
    if (s_mutex == NULL) {
        s_mutex = xSemaphoreCreateMutex();
    }
#endif
}

static uint8_t time_reached(uint32_t now_ms, uint32_t deadline_ms)
{
    return (int32_t)(now_ms - deadline_ms) >= 0 ? 1U : 0U;
}

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

static uint32_t virtual_sample_count(uint32_t now_ms, uint32_t start_ms,
                                     uint32_t duration_ms, uint32_t total)
{
    const uint32_t elapsed_ms = now_ms - start_ms;
    const uint64_t count = duration_ms == 0U || elapsed_ms >= duration_ms
                               ? total
                               : ((uint64_t)elapsed_ms * total) / duration_ms;

    return count > total ? total : (uint32_t)count;
}

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

static void boot_log(const char *text)
{
    if (text != NULL) {
        LOG_INFO(text);
    }
}

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

static void put_u32_le(uint8_t *destination, uint32_t value)
{
    destination[0] = (uint8_t)(value & UINT32_C(0xFF));
    destination[1] = (uint8_t)((value >> 8U) & UINT32_C(0xFF));
    destination[2] = (uint8_t)((value >> 16U) & UINT32_C(0xFF));
    destination[3] = (uint8_t)(value >> 24U);
}

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

static void send_cal_event(uint8_t event_id)
{
    const uint8_t payload[1] = {event_id};
    (void)send_message(SRP_MSG_ID_CAL_EVENT, SRP_FLAG_ACK_REQUIRED,
                       payload, (uint8_t)sizeof(payload));
}

static void send_stm_boot_ready(void)
{
    const uint8_t payload[2] = {(uint8_t)WAIT_SYNC, 0U};
    (void)send_message(SRP_MSG_ID_BOOT_READY, SRP_FLAG_ACK_REQUIRED,
                       payload, (uint8_t)sizeof(payload));
}

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

void imu_boot_manager_init(void)
{
    imu_boot_manager_reset();
    boot_log("DUAL_IMU_BOOT IDLE");
}

void imu_boot_manager_set_transport(imu_boot_transport_callback_t callback)
{
    ensure_boot_mutex();
    lock_boot();
    s_boot.transport = callback;
    unlock_boot();
}

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

void imu_boot_manager_get_dual_status(dual_imu_manager_t *status)
{
    if (status == NULL) {
        return;
    }
    lock_boot();
    *status = s_boot.dual;
    unlock_boot();
}

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

uint8_t imu_boot_manager_is_ready(void)
{
    uint8_t ready;

    lock_boot();
    ready = s_boot.dual.phase == IMU_PHASE_READY ? 1U : 0U;
    unlock_boot();
    return ready;
}

uint8_t imu_boot_manager_is_error(void)
{
    uint8_t failed;

    lock_boot();
    failed = s_boot.dual.phase == IMU_PHASE_FAILED ? 1U : 0U;
    unlock_boot();
    return failed;
}

uint8_t imu_boot_manager_get_progress(void)
{
    uint8_t progress;

    lock_boot();
    progress = s_boot.dual.overall_progress;
    unlock_boot();
    return progress;
}

uint32_t imu_boot_manager_get_sample_count(void)
{
    imu_boot_status_t status = {0};

    imu_boot_manager_get_status(&status);
    return status.sample_count;
}

uint32_t imu_boot_manager_get_sample_total(void)
{
    imu_boot_status_t status = {0};

    imu_boot_manager_get_status(&status);
    return status.sample_total;
}
