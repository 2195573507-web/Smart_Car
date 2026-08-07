#include "imu_boot_manager.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "bsp_timer.h"
#include "imu_filter.h"
#include "log_service.h"
#include "sc_frame.h"

#if defined(IMU_MANAGER_USE_FREERTOS)
#include "FreeRTOS.h"
#include "semphr.h"
#endif

#define IMU_BOOT_SETTLE_TIME_MS UINT32_C(2000)
#define IMU_BOOT_ACK_TIMEOUT_MS UINT32_C(500)
#define IMU_BOOT_MAX_RETRY UINT8_C(3)
#define IMU_BOOT_STATIC_SAMPLE_TIMEOUT_MS UINT32_C(70000)
#define IMU_BOOT_VIBRATION_SAMPLE_TIMEOUT_MS UINT32_C(20000)
#define IMU_BOOT_TOTAL_SAMPLES \
    (IMU_CALIBRATION_ACCEL_SAMPLES + \
     (IMU_VIBRATION_SAMPLES * IMU_VIBRATION_PROFILE_COUNT))

#define RADAR_PWM_ACK_TYPE SC_TYPE_RADAR_PWM_ACK

typedef enum
{
    IMU_BOOT_ERROR_NONE = 0,
    IMU_BOOT_ERROR_LSM303_INIT_FAIL,
    IMU_BOOT_ERROR_RADAR_READY_TIMEOUT,
    IMU_BOOT_ERROR_STATIC_CAL_TIMEOUT,
    IMU_BOOT_ERROR_VIBRATION_TIMEOUT,
    IMU_BOOT_ERROR_CAL_ACK_TIMEOUT,
    IMU_BOOT_ERROR_CAL_ACK_REJECTED
} imu_boot_error_reason_t;

typedef struct
{
    imu_boot_state_t state;
    uint8_t radar_pwm;
    uint8_t vibration_index;
    uint8_t progress;
    uint8_t error;
    imu_boot_error_reason_t error_reason;
    uint8_t static_done_sent;
    uint8_t vibration_started;
    uint8_t complete_sent;
    uint8_t ready_retry_count;
    uint8_t event_retry_count;
    uint8_t ready_waiting;
    uint8_t settle_armed;
    uint8_t event_waiting;
    uint8_t pending_radar_ready;
    uint8_t pending_radar_speed;
    uint32_t settle_until_ms;
    uint32_t ready_deadline_ms;
    uint32_t sample_deadline_ms;
    uint32_t event_deadline_ms;
    uint8_t pending_event_id;
    float rms;
    uint32_t error_sample_count;
    uint32_t error_sample_total;
    imu_boot_transport_callback_t transport;
} imu_boot_manager_state_t;

static imu_boot_manager_state_t s_boot;

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

static void boot_log(const char *text)
{
    if (text != NULL) {
        LOG_INFO(text);
    }
}

static void set_state(imu_boot_state_t state);

static const char *error_reason_name(imu_boot_error_reason_t reason)
{
    switch (reason) {
    case IMU_BOOT_ERROR_LSM303_INIT_FAIL:
        return "LSM303_INIT_FAIL";
    case IMU_BOOT_ERROR_RADAR_READY_TIMEOUT:
        return "RADAR_READY_TIMEOUT";
    case IMU_BOOT_ERROR_STATIC_CAL_TIMEOUT:
        return "STATIC_CAL_TIMEOUT";
    case IMU_BOOT_ERROR_VIBRATION_TIMEOUT:
        return "VIBRATION_TIMEOUT";
    case IMU_BOOT_ERROR_CAL_ACK_TIMEOUT:
        return "CAL_ACK_TIMEOUT";
    case IMU_BOOT_ERROR_CAL_ACK_REJECTED:
        return "CAL_ACK_REJECTED";
    case IMU_BOOT_ERROR_NONE:
    default:
        return "NONE";
    }
}

static const char *boot_state_name(imu_boot_state_t state)
{
    switch (state) {
    case IMU_BOOT_INIT: return "IMU_BOOT_INIT";
    case WAIT_RADAR_ZERO: return "WAIT_RADAR_ZERO";
    case STATIC_CAL_WAIT: return "STATIC_CAL_WAIT";
    case STATIC_CAL_SAMPLE: return "STATIC_CAL_SAMPLE";
    case STATIC_CAL_DONE: return "STATIC_CAL_DONE";
    case WAIT_RADAR_LEVEL: return "WAIT_RADAR_LEVEL";
    case VIBRATION_SAMPLE: return "VIBRATION_SAMPLE";
    case VIBRATION_LEVEL_DONE: return "VIBRATION_LEVEL_DONE";
    case VIBRATION_ALL_DONE: return "VIBRATION_ALL_DONE";
    case FILTER_READY: return "FILTER_READY";
    case IMU_READY: return "IMU_READY";
    case IMU_ERROR: return "IMU_ERROR";
    default: return "UNKNOWN";
    }
}

static void set_error(imu_boot_error_reason_t reason)
{
    const uint8_t progress = s_boot.progress;

    if (s_boot.state <= STATIC_CAL_DONE) {
        s_boot.error_sample_count = imu_calibration_get_sample_count();
        s_boot.error_sample_total = IMU_CALIBRATION_ACCEL_SAMPLES;
    } else {
        s_boot.error_sample_count = imu_vibration_get_sample_count();
        s_boot.error_sample_total = IMU_VIBRATION_SAMPLES;
    }
    s_boot.error = 1U;
    s_boot.error_reason = reason;
    set_state(IMU_ERROR);
    /* Preserve the progress reached before the failure. */
    s_boot.progress = progress;
}

static void log_error_reason(imu_boot_error_reason_t reason)
{
    char line[128];
    imu_boot_state_t state;
    uint32_t sample_count;
    uint32_t sample_total;

    lock_boot();
    state = s_boot.state;
    sample_count = s_boot.error_sample_count;
    sample_total = s_boot.error_sample_total;
    unlock_boot();
    (void)snprintf(line, sizeof(line),
                   "IMU CALIBRATION ERROR reason=%s state=%s sample=%lu/%lu",
                   error_reason_name(reason), boot_state_name(state),
                   (unsigned long)sample_count, (unsigned long)sample_total);
    boot_log(line);
}

static uint8_t send_frame(uint8_t type, const uint8_t *payload, uint16_t length)
{
    imu_boot_transport_callback_t callback;
    lock_boot();
    callback = s_boot.transport;
    unlock_boot();
    if (callback == NULL) {
        boot_log("BOOT TX FAILED transport=NULL");
        return 0U;
    }
    callback(type, payload, length);
    return 1U;
}

static uint8_t send_cal_event(uint8_t event_id)
{
    const uint8_t payload[1] = {event_id};
    char line[48];
    const uint8_t transport_set =
        send_frame(SC_TYPE_CAL_EVENT, payload, (uint16_t)sizeof(payload));

    (void)snprintf(line, sizeof(line), "CAL_EVENT_TX id=%u",
                   (unsigned)event_id);
    boot_log(line);
    return transport_set;
}

static void send_stm_boot_ready(void)
{
    const uint8_t payload[2] = {(uint8_t)WAIT_RADAR_ZERO, 0U};
    const uint8_t transport_set =
        send_frame(SC_TYPE_STM_BOOT_READY, payload, (uint16_t)sizeof(payload));

    boot_log(transport_set != 0U
                 ? "STM_BOOT_READY TX state=WAIT_RADAR_ZERO result=0 transport=SET"
                 : "STM_BOOT_READY TX state=WAIT_RADAR_ZERO result=0 transport=NULL");
}

static uint8_t total_progress(uint32_t current_samples)
{
    uint32_t completed = 0U;
    if (s_boot.state > STATIC_CAL_SAMPLE) {
        completed = IMU_CALIBRATION_ACCEL_SAMPLES;
    }
    if (s_boot.vibration_index != 0U) {
        completed += (uint32_t)s_boot.vibration_index * IMU_VIBRATION_SAMPLES;
    }
    if (s_boot.state == VIBRATION_SAMPLE) {
        completed += current_samples;
    }
    if (s_boot.state == VIBRATION_LEVEL_DONE ||
        s_boot.state == VIBRATION_ALL_DONE ||
        s_boot.state == FILTER_READY || s_boot.state == IMU_READY) {
        completed += IMU_VIBRATION_SAMPLES;
    }
    if (completed >= IMU_BOOT_TOTAL_SAMPLES) {
        return 100U;
    }
    return (uint8_t)((completed * 100U) / IMU_BOOT_TOTAL_SAMPLES);
}

static void set_state(imu_boot_state_t state)
{
    s_boot.state = state;
    s_boot.progress = total_progress(0U);
}

static void arm_ready_wait(uint32_t now_ms)
{
    s_boot.ready_waiting = 1U;
    s_boot.ready_retry_count = 0U;
    s_boot.ready_deadline_ms = now_ms + IMU_BOOT_ACK_TIMEOUT_MS;
}

static void consume_pending_radar_ready(uint32_t now_ms)
{
    const uint8_t expected =
        (uint8_t)(20U + (s_boot.vibration_index * 20U));

    if (s_boot.state != WAIT_RADAR_LEVEL ||
        s_boot.pending_radar_ready == 0U ||
        s_boot.pending_radar_speed != expected) {
        return;
    }

    s_boot.radar_pwm = s_boot.pending_radar_speed;
    s_boot.pending_radar_ready = 0U;
    s_boot.pending_radar_speed = 0U;
    s_boot.ready_waiting = 0U;
    s_boot.settle_until_ms = now_ms + IMU_BOOT_SETTLE_TIME_MS;
    s_boot.settle_armed = 1U;
    set_state(VIBRATION_SAMPLE);
}

static void ensure_boot_mutex(void)
{
#if defined(IMU_MANAGER_USE_FREERTOS)
    if (s_mutex == NULL) {
        s_mutex = xSemaphoreCreateMutex();
    }
#endif
}

void imu_boot_manager_reset(void)
{
    imu_boot_transport_callback_t transport_backup;

    ensure_boot_mutex();
    lock_boot();
    transport_backup = s_boot.transport;
    (void)memset(&s_boot, 0, sizeof(s_boot));
    s_boot.transport = transport_backup;
    s_boot.state = IMU_BOOT_INIT;
    unlock_boot();

    imu_calibration_start();
    imu_vibration_init();
}

void imu_boot_manager_init(bsp_status_t lsm303_status)
{
    imu_boot_manager_reset();

    boot_log("IMU CAL START");
    if (lsm303_status != BSP_STATUS_OK) {
        lock_boot();
        set_error(IMU_BOOT_ERROR_LSM303_INIT_FAIL);
        unlock_boot();
        boot_log("IMU INIT ERROR");
        log_error_reason(IMU_BOOT_ERROR_LSM303_INIT_FAIL);
        return;
    }
    boot_log("IMU INIT OK");
}

void imu_boot_manager_set_transport(imu_boot_transport_callback_t callback)
{
    lock_boot();
    s_boot.transport = callback;
    unlock_boot();
}

void imu_boot_manager_step(void)
{
    const uint32_t now_ms = timer_get_ms();
    uint8_t begin_static = 0U;
    uint8_t begin_vibration = 0U;
    uint8_t apply_filter = 0U;
    uint8_t send_static = 0U;
    uint8_t send_complete = 0U;
    uint8_t send_boot_ready_frame = 0U;
    uint8_t ready_timeout = 0U;
    uint8_t static_timeout = 0U;
    uint8_t vibration_timeout = 0U;
    uint8_t event_timeout = 0U;
    uint8_t resend_event = 0U;
    uint8_t retry_event_id = 0U;

    lock_boot();
    if (s_boot.state == IMU_BOOT_INIT) {
        /* Static calibration starts only after S3 confirms zero radar PWM. */
        s_boot.radar_pwm = 0U;
        set_state(WAIT_RADAR_ZERO);
        arm_ready_wait(now_ms);
        send_boot_ready_frame = 1U;
    }
    if (s_boot.ready_waiting != 0U &&
        (int32_t)(now_ms - s_boot.ready_deadline_ms) >= 0) {
        if (s_boot.ready_retry_count < IMU_BOOT_MAX_RETRY) {
            ++s_boot.ready_retry_count;
            s_boot.ready_deadline_ms = now_ms + IMU_BOOT_ACK_TIMEOUT_MS;
            if (s_boot.state == WAIT_RADAR_ZERO) {
                send_boot_ready_frame = 1U;
            }
        } else {
            s_boot.ready_waiting = 0U;
            set_error(IMU_BOOT_ERROR_RADAR_READY_TIMEOUT);
            ready_timeout = 1U;
        }
    }
    if (s_boot.state == STATIC_CAL_SAMPLE &&
        (int32_t)(now_ms - s_boot.sample_deadline_ms) >= 0) {
        set_error(IMU_BOOT_ERROR_STATIC_CAL_TIMEOUT);
        static_timeout = 1U;
    }
    if (s_boot.state == VIBRATION_SAMPLE &&
        (int32_t)(now_ms - s_boot.sample_deadline_ms) >= 0) {
        set_error(IMU_BOOT_ERROR_VIBRATION_TIMEOUT);
        vibration_timeout = 1U;
    }
    if (s_boot.event_waiting != 0U &&
        (int32_t)(now_ms - s_boot.event_deadline_ms) >= 0) {
        if (s_boot.event_retry_count < IMU_BOOT_MAX_RETRY) {
            ++s_boot.event_retry_count;
            s_boot.event_deadline_ms = now_ms + IMU_BOOT_ACK_TIMEOUT_MS;
            resend_event = 1U;
            retry_event_id = s_boot.pending_event_id;
        } else {
            s_boot.event_waiting = 0U;
            set_error(IMU_BOOT_ERROR_CAL_ACK_TIMEOUT);
            event_timeout = 1U;
        }
    }
    if (s_boot.settle_armed != 0U &&
        (s_boot.state == STATIC_CAL_WAIT || s_boot.state == VIBRATION_SAMPLE) &&
        (int32_t)(now_ms - s_boot.settle_until_ms) >= 0) {
        if (s_boot.state == STATIC_CAL_WAIT) {
            /* Reset the accumulator before publishing the sampling state. */
            imu_calibration_start();
            begin_static = 1U;
            set_state(STATIC_CAL_SAMPLE);
            s_boot.settle_armed = 0U;
            s_boot.sample_deadline_ms = now_ms +
                                        IMU_BOOT_STATIC_SAMPLE_TIMEOUT_MS;
        } else if (s_boot.vibration_started == 0U) {
            /* A new level must never observe the previous level's complete
             * flag or sample_count, even if update call topology changes. */
            imu_vibration_select_profile(s_boot.vibration_index);
            imu_vibration_start(s_boot.radar_pwm);
            begin_vibration = 1U;
            s_boot.vibration_started = 1U;
            s_boot.settle_armed = 0U;
            s_boot.sample_deadline_ms = now_ms +
                                        IMU_BOOT_VIBRATION_SAMPLE_TIMEOUT_MS;
        }
    }
    if (s_boot.state == STATIC_CAL_SAMPLE &&
        imu_calibration_is_complete() != 0U && s_boot.static_done_sent == 0U) {
        s_boot.static_done_sent = 1U;
        set_state(STATIC_CAL_DONE);
        s_boot.pending_event_id = SC_CAL_EVENT_STATIC_CAL_DONE;
        s_boot.event_retry_count = 0U;
        s_boot.event_deadline_ms = now_ms + IMU_BOOT_ACK_TIMEOUT_MS;
        s_boot.event_waiting = 1U;
        send_static = 1U;
    }
    if (s_boot.state == WAIT_RADAR_LEVEL &&
        s_boot.pending_radar_ready != 0U) {
        consume_pending_radar_ready(now_ms);
    }
    if (s_boot.state == VIBRATION_ALL_DONE && s_boot.complete_sent == 0U) {
        s_boot.complete_sent = 1U;
        set_state(FILTER_READY);
        s_boot.pending_event_id = SC_CAL_EVENT_COMPLETE;
        s_boot.event_retry_count = 0U;
        s_boot.event_deadline_ms = now_ms + IMU_BOOT_ACK_TIMEOUT_MS;
        s_boot.event_waiting = 1U;
        send_complete = 1U;
    }
    if (s_boot.state == FILTER_READY && s_boot.event_waiting == 0U) {
        apply_filter = 1U;
    }
    unlock_boot();

    if (send_boot_ready_frame != 0U) {
        send_stm_boot_ready();
    }
    if (ready_timeout != 0U) {
        boot_log("RADAR_PWM_READY TIMEOUT");
        log_error_reason(IMU_BOOT_ERROR_RADAR_READY_TIMEOUT);
    }
    if (static_timeout != 0U) {
        boot_log("STATIC CAL TIMEOUT");
        log_error_reason(IMU_BOOT_ERROR_STATIC_CAL_TIMEOUT);
    }
    if (vibration_timeout != 0U) {
        boot_log("VIBRATION SAMPLE TIMEOUT");
        log_error_reason(IMU_BOOT_ERROR_VIBRATION_TIMEOUT);
    }
    if (event_timeout != 0U) {
        boot_log("CAL EVENT ACK TIMEOUT");
        log_error_reason(IMU_BOOT_ERROR_CAL_ACK_TIMEOUT);
    }
    if (begin_static != 0U) {
        boot_log("IMU CAL: COLLECTING sample=0/5000");
    }
    if (begin_vibration != 0U) {
        boot_log("VIBRATION SAMPLE START");
    }
    if (send_static != 0U) {
        (void)send_cal_event(SC_CAL_EVENT_STATIC_CAL_DONE);
        boot_log("IMU CAL: DONE");
    }
    if (resend_event != 0U) {
        (void)send_cal_event(retry_event_id);
    }
    if (send_complete != 0U) {
        (void)send_cal_event(SC_CAL_EVENT_COMPLETE);
        boot_log("IMU CAL COMPLETE");
    }
    if (apply_filter != 0U) {
        imu_vibration_profile_t profiles[IMU_VIBRATION_PROFILE_COUNT];
        for (uint8_t index = 0U; index < IMU_VIBRATION_PROFILE_COUNT; ++index) {
            (void)imu_vibration_get_profile(index, &profiles[index]);
        }
        filter_set_vibration_profile(profiles, IMU_VIBRATION_PROFILE_COUNT);
        lock_boot();
        if (s_boot.state == FILTER_READY) {
            set_state(IMU_READY);
        }
        unlock_boot();
        boot_log("IMU READY");
    }
}

void imu_boot_manager_update(const imu_raw_data_t *raw_data)
{
    uint8_t send_vibration = 0U;
    imu_vibration_profile_t result;

    if (raw_data == NULL || raw_data->online == 0U) {
        return;
    }

    lock_boot();
    const imu_boot_state_t state = s_boot.state;
    const uint8_t vibration_started = s_boot.vibration_started;
    unlock_boot();

    if (state == STATIC_CAL_SAMPLE) {
        const uint32_t count_before = imu_calibration_get_sample_count();
        imu_calibration_update(raw_data);
        const uint32_t count = imu_calibration_get_sample_count();
        lock_boot();
        s_boot.progress = (uint8_t)((count * 100U) / IMU_BOOT_TOTAL_SAMPLES);
        unlock_boot();
        if (count != count_before && count != 0U && (count % 100U) == 0U) {
            char line[64];
            (void)snprintf(line, sizeof(line),
                           "IMU CAL: COLLECTING sample=%lu/%lu",
                           (unsigned long)count,
                           (unsigned long)IMU_CALIBRATION_ACCEL_SAMPLES);
            boot_log(line);
        }
        return;
    }
    if (state != VIBRATION_SAMPLE || vibration_started == 0U) {
        return;
    }

    {
        const imu_calibrated_data_t calibrated = imu_calibration_apply(raw_data);
        imu_vibration_update(&calibrated);
    }
    const uint32_t count = imu_vibration_get_sample_count();
    if (count >= IMU_VIBRATION_SAMPLES &&
        imu_vibration_is_complete() != 0U) {
        result = imu_vibration_get_result();
        lock_boot();
        if (s_boot.state == VIBRATION_SAMPLE &&
            s_boot.vibration_started != 0U) {
            s_boot.rms = result.total_rms;
            s_boot.vibration_started = 0U;
            s_boot.state = VIBRATION_LEVEL_DONE;
            s_boot.progress = total_progress(IMU_VIBRATION_SAMPLES);
            s_boot.pending_event_id = SC_CAL_EVENT_VIBRATION_STEP_DONE;
            s_boot.event_retry_count = 0U;
            s_boot.event_deadline_ms = timer_get_ms() + IMU_BOOT_ACK_TIMEOUT_MS;
            s_boot.event_waiting = 1U;
            send_vibration = 1U;
        }
        unlock_boot();
    } else {
        lock_boot();
        s_boot.progress = total_progress(count);
        unlock_boot();
        if (count == 500U) {
            boot_log("SAMPLE 500/1000");
        }
    }
    if (send_vibration != 0U) {
        char line[96];
        const uint32_t rms_scaled =
            (uint32_t)((result.total_rms * 10000.0f) + 0.5f);
        (void)send_cal_event(SC_CAL_EVENT_VIBRATION_STEP_DONE);
        (void)snprintf(line, sizeof(line),
                       "RADAR PWM=%u SAMPLE 1000/1000 RMS=%lu.%04lu",
                       (unsigned)result.radar_pwm,
                       (unsigned long)(rms_scaled / 10000U),
                       (unsigned long)(rms_scaled % 10000U));
        boot_log(line);
    }
}

void imu_boot_manager_on_radar_pwm_ready(uint8_t speed)
{
    uint8_t ack[2] = {speed, 0U};
    uint8_t accepted = 0U;
    uint8_t start_settle = 0U;

    lock_boot();
    /* READY may arrive before the matching CAL_ACK. Remember it, but let the
     * CAL_ACK remain the only event that advances the calibration stage. */
    if (s_boot.state == STATIC_CAL_DONE &&
        s_boot.event_waiting != 0U &&
        s_boot.pending_event_id == SC_CAL_EVENT_STATIC_CAL_DONE &&
        speed == 20U) {
        s_boot.pending_radar_ready = 1U;
        s_boot.pending_radar_speed = speed;
        accepted = 1U;
    } else if (s_boot.state == VIBRATION_LEVEL_DONE &&
               s_boot.event_waiting != 0U &&
               s_boot.pending_event_id == SC_CAL_EVENT_VIBRATION_STEP_DONE &&
               s_boot.vibration_index + 1U < IMU_VIBRATION_PROFILE_COUNT &&
               speed == (uint8_t)(s_boot.radar_pwm + 20U)) {
        s_boot.pending_radar_ready = 1U;
        s_boot.pending_radar_speed = speed;
        accepted = 1U;
    }
    if (accepted == 0U) {
        const uint8_t expected =
            (uint8_t)(20U + (s_boot.vibration_index * 20U));
        if ((s_boot.state == IMU_BOOT_INIT || s_boot.state == WAIT_RADAR_ZERO) &&
            speed == 0U) {
            s_boot.radar_pwm = 0U;
            s_boot.settle_armed = 0U;
            set_state(STATIC_CAL_WAIT);
            s_boot.ready_waiting = 0U;
            accepted = 1U;
            start_settle = 1U;
        } else if (s_boot.state == WAIT_RADAR_LEVEL &&
                   s_boot.vibration_index < IMU_VIBRATION_PROFILE_COUNT &&
                   speed == expected) {
            s_boot.radar_pwm = speed;
            s_boot.vibration_started = 0U;
            s_boot.settle_armed = 0U;
            set_state(VIBRATION_SAMPLE);
            s_boot.ready_waiting = 0U;
            accepted = 1U;
            start_settle = 1U;
        } else if (((s_boot.state == STATIC_CAL_DONE) ||
                    (s_boot.state == STATIC_CAL_WAIT) ||
                    (s_boot.state == STATIC_CAL_SAMPLE)) &&
                   speed == 0U && s_boot.radar_pwm == 0U) {
            /* Zero-duty READY is only an acknowledgement; it never starts or
             * restarts a calibration window, including after static completion. */
            accepted = 1U;
        } else if ((s_boot.state == VIBRATION_SAMPLE ||
                    s_boot.state == VIBRATION_LEVEL_DONE) &&
                   speed == s_boot.radar_pwm) {
            /* READY retransmission: acknowledge without restarting the window. */
            accepted = 1U;
        } else {
            ack[1] = 1U;
        }
    }
    unlock_boot();

    send_frame(RADAR_PWM_ACK_TYPE, ack, (uint16_t)sizeof(ack));
    {
        char line[64];
        (void)snprintf(line, sizeof(line),
                       "RADAR_PWM_ACK TX speed=%u result=%u",
                       (unsigned)ack[0], (unsigned)ack[1]);
        boot_log(line);
    }
    if (start_settle != 0U) {
        lock_boot();
        if (s_boot.state == STATIC_CAL_WAIT ||
            s_boot.state == VIBRATION_SAMPLE) {
            s_boot.settle_until_ms = timer_get_ms() + IMU_BOOT_SETTLE_TIME_MS;
            s_boot.settle_armed = 1U;
        }
        unlock_boot();
        if (speed == 0U) {
            boot_log("IMU CAL: WAIT_STABLE sample=0/5000");
        }
    }
    if (accepted != 0U) {
        char line[48];
        (void)snprintf(line, sizeof(line), "RADAR PWM=%u", (unsigned)speed);
        boot_log(line);
    }
}

void imu_boot_manager_on_cal_event_ack(uint8_t event_id, uint8_t result)
{
    uint8_t accepted = 0U;
    uint8_t cal_ack_error = 0U;
    uint8_t state_matches;
    char line[64];

    (void)snprintf(line, sizeof(line), "CAL_ACK_RX id=%d result=%d status=%s",
                   (int)event_id, (int)result,
                   result == CAL_ACK_OK ? "OK" : "FAIL");
    boot_log(line);

    lock_boot();
    state_matches =
        ((event_id == SC_CAL_EVENT_STATIC_CAL_DONE &&
          s_boot.state == STATIC_CAL_DONE) ||
         (event_id == SC_CAL_EVENT_VIBRATION_STEP_DONE &&
          s_boot.state == VIBRATION_LEVEL_DONE) ||
         (event_id == SC_CAL_EVENT_COMPLETE &&
          s_boot.state == FILTER_READY))
            ? 1U
            : 0U;
    if (s_boot.event_waiting != 0U &&
        event_id == s_boot.pending_event_id &&
        state_matches != 0U) {
        if (result != CAL_ACK_OK) {
            s_boot.event_waiting = 0U;
            set_error(IMU_BOOT_ERROR_CAL_ACK_REJECTED);
            cal_ack_error = 1U;
        } else {
            s_boot.event_waiting = 0U;
            s_boot.event_retry_count = 0U;
            if (event_id == SC_CAL_EVENT_STATIC_CAL_DONE &&
                s_boot.state == STATIC_CAL_DONE) {
                set_state(WAIT_RADAR_LEVEL);
                arm_ready_wait(timer_get_ms());
                accepted = 1U;
            } else if (event_id == SC_CAL_EVENT_VIBRATION_STEP_DONE &&
                       s_boot.state == VIBRATION_LEVEL_DONE) {
                if (s_boot.vibration_index + 1U >=
                    IMU_VIBRATION_PROFILE_COUNT) {
                    set_state(VIBRATION_ALL_DONE);
                } else {
                    ++s_boot.vibration_index;
                    set_state(WAIT_RADAR_LEVEL);
                    arm_ready_wait(timer_get_ms());
                }
                accepted = 1U;
            } else if (event_id == SC_CAL_EVENT_COMPLETE &&
                       s_boot.state == FILTER_READY) {
                accepted = 1U;
            }
        }
    }
    unlock_boot();

    if (accepted != 0U) {
        boot_log("CAL EVENT ACK");
    }
    if (cal_ack_error != 0U) {
        log_error_reason(IMU_BOOT_ERROR_CAL_ACK_REJECTED);
    }
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
    status->progress = s_boot.progress;
    status->radar_pwm = s_boot.radar_pwm;
    status->error = s_boot.error;
    status->rms = s_boot.rms;
    status->error_reason = error_reason_name(s_boot.error_reason);
    if (s_boot.state == IMU_ERROR) {
        status->sample_count = s_boot.error_sample_count;
        status->sample_total = s_boot.error_sample_total;
    } else if (s_boot.state <= STATIC_CAL_DONE) {
        status->sample_count = imu_calibration_get_sample_count();
        status->sample_total = IMU_CALIBRATION_ACCEL_SAMPLES;
    } else {
        status->sample_count = imu_vibration_get_sample_count();
        status->sample_total = IMU_VIBRATION_SAMPLES;
    }
    unlock_boot();
}

uint8_t imu_boot_manager_is_ready(void)
{
    return imu_boot_manager_get_state() == IMU_READY ? 1U : 0U;
}

uint8_t imu_boot_manager_is_error(void)
{
    return imu_boot_manager_get_state() == IMU_ERROR ? 1U : 0U;
}

uint8_t imu_boot_manager_get_progress(void)
{
    imu_boot_status_t status;
    imu_boot_manager_get_status(&status);
    return status.progress;
}

uint8_t imu_boot_manager_get_radar_pwm(void)
{
    imu_boot_status_t status;
    imu_boot_manager_get_status(&status);
    return status.radar_pwm;
}

uint32_t imu_boot_manager_get_sample_count(void)
{
    imu_boot_status_t status;
    imu_boot_manager_get_status(&status);
    return status.sample_count;
}

uint32_t imu_boot_manager_get_sample_total(void)
{
    imu_boot_status_t status;
    imu_boot_manager_get_status(&status);
    return status.sample_total;
}

float imu_boot_manager_get_rms(void)
{
    imu_boot_status_t status;
    imu_boot_manager_get_status(&status);
    return status.rms;
}
