#include "imu_manager.h"

#include "imu_time.h"
#include "attitude.h"
#include "dual_ahrs.h"
#include "boot_log.h"
#include "lsm303.h"
#include "BMI323/bmi323.h"
#include "imu_calibration.h"
#include "imu_boot_manager.h"
#include "imu_filter.h"
#include "mag_filter.h"
#include "log_service.h"

#include <stdio.h>
#include <string.h>

#if defined(IMU_MANAGER_USE_FREERTOS)
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#endif

#define IMU_TASK_PERIOD_MS       UINT32_C(10)
#define IMU_RECOVERY_PERIOD_MS   UINT32_C(1000)
#define IMU_LSM303_ONLINE_TIMEOUT_MS UINT32_C(1000)
#define IMU_LSM303_FAIL_LIMIT         UINT8_C(10)
#define IMU_LSM303_HEALTH_PERIOD_MS   UINT32_C(1000)
#if defined(IMU_MANAGER_USE_FREERTOS)
#define BMI323_TASK_STACK_WORDS       UINT16_C(384)
#define BMI323_TASK_PRIORITY          (tskIDLE_PRIORITY + 1U)
#define DUAL_IMU_INIT_TASK_STACK_WORDS UINT16_C(384)
#define DUAL_IMU_INIT_TASK_PRIORITY    (tskIDLE_PRIORITY + 1U)
#endif

#ifndef SMARTCAR_BMI323_DEBUG_ONLY
#define SMARTCAR_BMI323_DEBUG_ONLY 0
#endif

/* BMI323 samples remain manager-local and do not enter the LSM303 AHRS path. */
static bmi323_data_t bmi_data;
static bmi323_ring_buffer_t bmi_ring_buffer;
static bmi323_capture_stat_t bmi_capture_stats;
static volatile uint32_t bmi_capture_contention_drop_count;
static lsm_accel_data_t lsm_accel_data;
static lsm_mag_data_t lsm_mag_data;
static imu_raw_data_t imu_snapshot;
static imu_sensor_stats_t bmi_stats;
static imu_sensor_stats_t lsm_stats;
static volatile uint8_t imu_initialized;
static volatile uint8_t imu_prepared;
static volatile uint8_t bmi323_acquisition_enabled;
static uint8_t lsm303_init_success;
static uint8_t bmi323_init_success;
static imu_dual_init_status_t dual_init_status;
static uint32_t last_accel_success_tick;
static uint32_t last_mag_success_tick;
static uint8_t accel_fail_count;
static uint8_t mag_fail_count;
static uint8_t lsm_accel_valid;
static uint8_t lsm_mag_valid;
static uint8_t dual_ahrs_bias_injected;
static uint8_t legacy_attitude_ready_latched;
static bmi323_diag_status_t bmi323_last_logged_error;
static imu_leveling_state_t g_leveling_bmi;
static imu_leveling_state_t g_leveling_lsm;
#if defined(IMU_MANAGER_USE_FREERTOS)
static TaskHandle_t bmi323_task_handle;
static uint8_t bmi323_task_started;
static TaskHandle_t dual_lsm_init_task_handle;
static TaskHandle_t dual_bmi_init_task_handle;
#endif

typedef struct
{
    uint8_t init;
    uint32_t accel_age;
    uint32_t mag_age;
    uint8_t accel_fail;
    uint8_t mag_fail;
    uint8_t online;
} imu_lsm_health_t;

static void imu_init_log(const char *text)
{
    if (text != NULL) {
        LOG_INFO(text);
    }
}

static void imu_init_log_status(const char *stage, bsp_status_t status)
{
    char line[48];

    if (stage == NULL) {
        return;
    }
    (void)snprintf(line, sizeof(line), "%s ret=%d\r\n", stage, (int)status);
    imu_init_log(line);
}

#if defined(IMU_MANAGER_USE_FREERTOS)
static SemaphoreHandle_t bmi_data_mutex;
static SemaphoreHandle_t bmi_driver_mutex;
static SemaphoreHandle_t lsm_data_mutex;
static SemaphoreHandle_t snapshot_mutex;
static SemaphoreHandle_t dual_init_mutex;
#endif

#if !defined(IMU_MANAGER_USE_FREERTOS)
static void imu_delay_ms(uint32_t delay_ms)
{
    const uint32_t start = imu_time_now_ms();
    while ((uint32_t)(imu_time_now_ms() - start) < delay_ms) {
        /* Non-RTOS fallback for the standalone task entrypoint. */
    }
}
#endif

static bsp_status_t imu_create_data_locks(void)
{
#if defined(IMU_MANAGER_USE_FREERTOS)
    if (bmi_data_mutex == NULL) {
        bmi_data_mutex = xSemaphoreCreateMutex();
        if (bmi_data_mutex == NULL) {
            return BSP_STATUS_ERROR;
        }
    }
    if (bmi_driver_mutex == NULL) {
        bmi_driver_mutex = xSemaphoreCreateMutex();
        if (bmi_driver_mutex == NULL) {
            return BSP_STATUS_ERROR;
        }
    }
    if (lsm_data_mutex == NULL) {
        lsm_data_mutex = xSemaphoreCreateMutex();
        if (lsm_data_mutex == NULL) {
            return BSP_STATUS_ERROR;
        }
    }
    if (snapshot_mutex == NULL) {
        snapshot_mutex = xSemaphoreCreateMutex();
        if (snapshot_mutex == NULL) {
            return BSP_STATUS_ERROR;
        }
    }
    if (dual_init_mutex == NULL) {
        dual_init_mutex = xSemaphoreCreateMutex();
        if (dual_init_mutex == NULL) {
            return BSP_STATUS_ERROR;
        }
    }
#endif
    return BSP_STATUS_OK;
}

static void imu_lock_bmi(void)
{
#if defined(IMU_MANAGER_USE_FREERTOS)
    (void)xSemaphoreTake(bmi_data_mutex, portMAX_DELAY);
#endif
}

static void imu_unlock_bmi(void)
{
#if defined(IMU_MANAGER_USE_FREERTOS)
    (void)xSemaphoreGive(bmi_data_mutex);
#endif
}

static uint8_t imu_try_lock_bmi(void)
{
#if defined(IMU_MANAGER_USE_FREERTOS)
    if (bmi_data_mutex == NULL) {
        return 0U;
    }
    return xSemaphoreTake(bmi_data_mutex, (TickType_t)0) == pdTRUE ? 1U : 0U;
#else
    return 1U;
#endif
}

/*
 * LSM303 sensor frame -> vehicle Body Frame.
 *
 * The selected board mapping is a 180-degree rotation about Body Z.  Keep the
 * map as one proper rotation instead of distributing axis sign changes across
 * the acquisition and attitude paths.  PCB orientation remains a hardware
 * acceptance item; this is the source-level mapping recorded by the project:
 *
 *                  [-1  0  0]
 *     R_lsm_body = [ 0 -1  0], det(R) = +1
 *                  [ 0  0  1]
 */
static const float lsm303_sensor_to_body[3][3] = {
    {-1.0f, 0.0f, 0.0f},
    {0.0f, -1.0f, 0.0f},
    {0.0f, 0.0f, 1.0f},
};

static Vector3f lsm303_to_body(Vector3f sensor)
{
    return (Vector3f){
        (lsm303_sensor_to_body[0][0] * sensor.x) +
            (lsm303_sensor_to_body[0][1] * sensor.y) +
            (lsm303_sensor_to_body[0][2] * sensor.z),
        (lsm303_sensor_to_body[1][0] * sensor.x) +
            (lsm303_sensor_to_body[1][1] * sensor.y) +
            (lsm303_sensor_to_body[1][2] * sensor.z),
        (lsm303_sensor_to_body[2][0] * sensor.x) +
            (lsm303_sensor_to_body[2][1] * sensor.y) +
            (lsm303_sensor_to_body[2][2] * sensor.z),
    };
}

static void imu_lock_bmi_driver(void)
{
#if defined(IMU_MANAGER_USE_FREERTOS)
    (void)xSemaphoreTake(bmi_driver_mutex, portMAX_DELAY);
#endif
}

static void imu_unlock_bmi_driver(void)
{
#if defined(IMU_MANAGER_USE_FREERTOS)
    (void)xSemaphoreGive(bmi_driver_mutex);
#endif
}

static uint8_t imu_try_lock_bmi_driver(void)
{
#if defined(IMU_MANAGER_USE_FREERTOS)
    if (bmi_driver_mutex == NULL) {
        return 0U;
    }
    return xSemaphoreTake(bmi_driver_mutex, (TickType_t)0) == pdTRUE ? 1U : 0U;
#else
    return 1U;
#endif
}

static void imu_lock_lsm(void)
{
#if defined(IMU_MANAGER_USE_FREERTOS)
    (void)xSemaphoreTake(lsm_data_mutex, portMAX_DELAY);
#endif
}

static void imu_unlock_lsm(void)
{
#if defined(IMU_MANAGER_USE_FREERTOS)
    (void)xSemaphoreGive(lsm_data_mutex);
#endif
}

static void imu_lock_snapshot(void)
{
#if defined(IMU_MANAGER_USE_FREERTOS)
    (void)xSemaphoreTake(snapshot_mutex, portMAX_DELAY);
#endif
}

static void imu_unlock_snapshot(void)
{
#if defined(IMU_MANAGER_USE_FREERTOS)
    (void)xSemaphoreGive(snapshot_mutex);
#endif
}

static void imu_lock_dual_init(void)
{
#if defined(IMU_MANAGER_USE_FREERTOS)
    if (dual_init_mutex != NULL) {
        (void)xSemaphoreTake(dual_init_mutex, portMAX_DELAY);
    }
#endif
}

static void imu_unlock_dual_init(void)
{
#if defined(IMU_MANAGER_USE_FREERTOS)
    if (dual_init_mutex != NULL) {
        (void)xSemaphoreGive(dual_init_mutex);
    }
#endif
}

static uint16_t imu_bmi_measured_rate_hz(const bmi323_capture_stat_t *stats)
{
    uint64_t elapsed_us;
    uint64_t rate_hz;

    if (stats == NULL || stats->sample_count < 2U ||
        stats->last_timestamp_us <= stats->first_timestamp_us) {
        return 0U;
    }
    elapsed_us = stats->last_timestamp_us - stats->first_timestamp_us;
    rate_hz = (((uint64_t)(stats->sample_count - 1U) * UINT64_C(1000000)) +
               (elapsed_us / UINT64_C(2))) /
              elapsed_us;
    return rate_hz > UINT16_MAX ? UINT16_MAX : (uint16_t)rate_hz;
}

static void imu_bmi_ring_reset_locked(void)
{
    (void)memset(&bmi_ring_buffer, 0, sizeof(bmi_ring_buffer));
    bmi_capture_contention_drop_count = 0U;
    bmi_capture_stats = (bmi323_capture_stat_t){
        .configured_rate_hz = (uint16_t)bmi323_get_sample_rate()
    };
}

static void imu_bmi_capture_note_contention_drop(void)
{
    const uint32_t drops = bmi_capture_contention_drop_count;

    if (drops != UINT32_MAX) {
        bmi_capture_contention_drop_count = drops + 1U;
    }
}

static void imu_bmi_ring_push(const bmi323_raw_sample_t *sample)
{
    if (sample == NULL || sample->valid == 0U) {
        return;
    }

    /* The producer never waits behind the 10 ms manager. A lock contention
     * drops this observation and is separate from full-ring overflow. */
    if (imu_try_lock_bmi() == 0U) {
        imu_bmi_capture_note_contention_drop();
        return;
    }
    if (bmi_ring_buffer.count >= BMI_RING_BUFFER_SIZE) {
        bmi_ring_buffer.tail =
            (uint16_t)((bmi_ring_buffer.tail + 1U) % BMI_RING_BUFFER_SIZE);
        if (bmi_ring_buffer.overflow_count != UINT32_MAX) {
            ++bmi_ring_buffer.overflow_count;
        }
    } else {
        ++bmi_ring_buffer.count;
    }
    bmi_ring_buffer.buffer[bmi_ring_buffer.head] = *sample;
    bmi_ring_buffer.head =
        (uint16_t)((bmi_ring_buffer.head + 1U) % BMI_RING_BUFFER_SIZE);
    bmi_ring_buffer.last_capture_us = sample->timestamp_us;
    if (bmi_capture_stats.sample_count == 0U) {
        bmi_capture_stats.first_timestamp_us = sample->timestamp_us;
    }
    if (bmi_capture_stats.sample_count != UINT32_MAX) {
        ++bmi_capture_stats.sample_count;
    }
    bmi_capture_stats.overflow_count = bmi_ring_buffer.overflow_count;
    bmi_capture_stats.last_timestamp_us = sample->timestamp_us;
    bmi_capture_stats.last_timestamp =
        (uint32_t)(sample->timestamp_us / UINT64_C(1000));
    bmi_capture_stats.measured_rate_hz =
        imu_bmi_measured_rate_hz(&bmi_capture_stats);
    bmi_capture_stats.pending_count = bmi_ring_buffer.count;
    ++bmi_stats.read_calls;
    ++bmi_stats.update_count;
    bmi_stats.last_update_ms =
        (uint32_t)(sample->timestamp_us / UINT64_C(1000));
    bmi_stats.last_status = BSP_STATUS_OK;
    imu_unlock_bmi();
}

static void imu_bmi_capture_failed(void)
{
    if (imu_try_lock_bmi() == 0U) {
        imu_bmi_capture_note_contention_drop();
        return;
    }
    ++bmi_stats.read_calls;
    bmi_stats.last_status = BSP_STATUS_ERROR;
    ++bmi_data.invalid_count;
    if (bmi_capture_stats.read_fail_count != UINT32_MAX) {
        ++bmi_capture_stats.read_fail_count;
    }
    bmi_capture_stats.pending_count = bmi_ring_buffer.count;
    imu_unlock_bmi();
}

static uint8_t imu_bmi_ring_take_latest(bmi323_raw_sample_t *sample,
                                        uint64_t *capture_us)
{
    uint16_t latest_index;

    if (sample == NULL) {
        return 0U;
    }
    imu_lock_bmi();
    if (bmi_ring_buffer.count == 0U) {
        bmi_capture_stats.pending_count = 0U;
        imu_unlock_bmi();
        return 0U;
    }
    latest_index = (uint16_t)((bmi_ring_buffer.head + BMI_RING_BUFFER_SIZE -
                               1U) % BMI_RING_BUFFER_SIZE);
    *sample = bmi_ring_buffer.buffer[latest_index];
    if (capture_us != NULL) {
        *capture_us = bmi_ring_buffer.last_capture_us;
    }
    /* The manager intentionally publishes only the newest sample. Older
     * high-rate samples are discarded in O(1) time for the next manager tick. */
    bmi_ring_buffer.tail = bmi_ring_buffer.head;
    bmi_ring_buffer.count = 0U;
    bmi_capture_stats.pending_count = 0U;
    imu_unlock_bmi();
    return 1U;
}

static void imu_reset_data(uint8_t reset_count)
{
    legacy_attitude_ready_latched = 0U;
    imu_lock_bmi();
    bmi_data = (bmi323_data_t){0};
    imu_bmi_ring_reset_locked();
    bmi323_init_success = 0U;
    if (reset_count != 0U) {
        bmi_stats = (imu_sensor_stats_t){0};
    }
    bmi_stats.last_status = BSP_STATUS_NOT_READY;
    imu_unlock_bmi();

    imu_lock_lsm();
    lsm_accel_data = (lsm_accel_data_t){0};
    lsm_mag_data = (lsm_mag_data_t){0};
    lsm_accel_valid = 0U;
    lsm_mag_valid = 0U;
    lsm303_init_success = 0U;
    last_accel_success_tick = 0U;
    last_mag_success_tick = 0U;
    accel_fail_count = IMU_LSM303_FAIL_LIMIT;
    mag_fail_count = IMU_LSM303_FAIL_LIMIT;
    if (reset_count != 0U) {
        lsm_stats = (imu_sensor_stats_t){0};
    }
    lsm_stats.last_status = BSP_STATUS_NOT_READY;
    imu_unlock_lsm();

    imu_lock_snapshot();
    imu_snapshot = (imu_raw_data_t){0};
    imu_unlock_snapshot();
}

static void imu_mark_lsm_initialized(bsp_status_t status)
{
    imu_lock_lsm();
    lsm303_init_success = status == BSP_STATUS_OK ? 1U : 0U;
    lsm_accel_valid = 0U;
    lsm_mag_valid = 0U;
    last_accel_success_tick = 0U;
    last_mag_success_tick = 0U;
    accel_fail_count = IMU_LSM303_FAIL_LIMIT;
    mag_fail_count = IMU_LSM303_FAIL_LIMIT;
    /* Keep the aggregate status for diagnostics; online uses independent
     * initialization, freshness, and failure health. */
    lsm_stats.last_status = status;
    imu_unlock_lsm();
}

static void imu_mark_bmi323_initialized(uint8_t initialized)
{
    imu_lock_bmi();
    bmi_data = (bmi323_data_t){0};
    imu_bmi_ring_reset_locked();
    bmi_stats = (imu_sensor_stats_t){0};
    bmi_stats.last_status = initialized != 0U ? BSP_STATUS_OK : BSP_STATUS_ERROR;
    bmi323_init_success = initialized;
    imu_unlock_bmi();
}

static void imu_lsm_health_snapshot(uint32_t now_ms, imu_lsm_health_t *health)
{
    if (health == NULL) {
        return;
    }

    imu_lock_lsm();
    health->init = lsm303_init_success;
    health->accel_fail = accel_fail_count;
    health->mag_fail = mag_fail_count;
    health->accel_age = (accel_fail_count >= IMU_LSM303_FAIL_LIMIT &&
                         last_accel_success_tick == 0U)
                            ? UINT32_MAX
                            : (uint32_t)(now_ms - last_accel_success_tick);
    health->mag_age = (mag_fail_count >= IMU_LSM303_FAIL_LIMIT &&
                       last_mag_success_tick == 0U)
                          ? UINT32_MAX
                          : (uint32_t)(now_ms - last_mag_success_tick);
    health->online = (health->init != 0U &&
                      health->accel_fail < IMU_LSM303_FAIL_LIMIT &&
                      health->mag_fail < IMU_LSM303_FAIL_LIMIT &&
                      health->accel_age < IMU_LSM303_ONLINE_TIMEOUT_MS &&
                      health->mag_age < IMU_LSM303_ONLINE_TIMEOUT_MS)
                         ? 1U
                         : 0U;
    imu_unlock_lsm();
}

static uint8_t imu_lsm_is_online(void)
{
    imu_lsm_health_t health = {0};

    imu_lsm_health_snapshot(imu_time_now_ms(), &health);
    return health.online;
}

static void imu_lsm_health_log(uint32_t now_ms)
{
    char line[160];
    imu_lsm_health_t health = {0};

    imu_lsm_health_snapshot(now_ms, &health);
    (void)snprintf(line, sizeof(line),
                   "[LSM303_HEALTH] init=%u accel_age=%lu mag_age=%lu "
                   "accel_fail=%u mag_fail=%u online=%u\r\n",
                   (unsigned)health.init, (unsigned long)health.accel_age,
                   (unsigned long)health.mag_age, (unsigned)health.accel_fail,
                   (unsigned)health.mag_fail, (unsigned)health.online);
    imu_init_log(line);
}

static void imu_bmi323_error_log(bmi323_diag_status_t status)
{
    char line[64];

    if (status == BMI323_DIAG_STATUS_OK) {
        bmi323_last_logged_error = BMI323_DIAG_STATUS_OK;
        return;
    }
    if (status == bmi323_last_logged_error) {
        return;
    }

    (void)snprintf(line, sizeof(line), "[BMI323][ERROR]\r\nreason=%s\r\n",
                   bmi323_diag_status_name(status));
    LOG_ERROR(line);
    bmi323_last_logged_error = status;
}

static void imu_bmi323_init_log(void)
{
    char line[LOG_SERVICE_TEXT_MAX + 1U];
    bmi323_diagnostics_t diagnostics;

    imu_lock_bmi_driver();
    bmi323_get_diagnostics(&diagnostics);
    imu_unlock_bmi_driver();
    (void)snprintf(line, sizeof(line),
                   "[BMI323][INIT]\r\nPROBE_ID=0x%02X\r\n"
                   "POST_RESET_ID=0x%02X\r\ninit_result=%ld\r\n",
                   (unsigned)diagnostics.who_am_i,
                   (unsigned)diagnostics.post_reset_who_am_i,
                   (long)diagnostics.init_result);
    imu_init_log(line);
    (void)snprintf(line, sizeof(line),
                   "[BMI323][INIT]\r\nCTRL_ACC=0x%04X\r\nCTRL_GYR=0x%04X\r\n"
                   "ACC_CONF=0x%04X\r\nGYR_CONF=0x%04X\r\n",
                   (unsigned)diagnostics.ctrl_acc, (unsigned)diagnostics.ctrl_gyr,
                   (unsigned)diagnostics.acc_conf, (unsigned)diagnostics.gyr_conf);
    imu_init_log(line);
    (void)snprintf(line, sizeof(line), "[BMI323][INIT]\r\nspi_error_count=%lu\r\n",
                   (unsigned long)diagnostics.spi_error_count);
    imu_init_log(line);
    imu_bmi323_error_log(diagnostics.last_status);
}

static void imu_bmi323_debug_log(void)
{
    char line[LOG_SERVICE_TEXT_MAX + 1U];
    bmi323_diagnostics_t diagnostics;
    bmi323_diag_t diag;
    bmi323_capture_stat_t capture = {0};
    bmi323_diag_status_t reported_status;
    static uint32_t last_overflow_count;
    static uint32_t last_driver_read_fail_count;
    static uint32_t last_capture_read_fail_count;

    imu_lock_bmi_driver();
    bmi323_get_diagnostics(&diagnostics);
    bmi323_get_diag(&diag);
    imu_unlock_bmi_driver();
    (void)imu_manager_get_bmi323_capture_stats(&capture);
    reported_status = diagnostics.last_status;
    if (diag.valid == 0U && reported_status == BMI323_DIAG_STATUS_OK) {
        reported_status = BMI323_DIAG_STATUS_DATA_NOT_READY;
    }
    imu_bmi323_error_log(reported_status);
    if (capture.overflow_count != last_overflow_count ||
        diagnostics.read_fail != last_driver_read_fail_count ||
        capture.read_fail_count != last_capture_read_fail_count) {
        if (capture.overflow_count != 0U || diagnostics.read_fail != 0U ||
            capture.read_fail_count != 0U) {
            (void)snprintf(line, sizeof(line),
                           "[BMI323][CAPTURE_WARN] overflow=%lu driver_read_fail=%lu "
                           "capture_read_fail=%lu\r\n",
                           (unsigned long)capture.overflow_count,
                           (unsigned long)diagnostics.read_fail,
                           (unsigned long)capture.read_fail_count);
            LOG_WARN(line);
        }
        last_overflow_count = capture.overflow_count;
        last_driver_read_fail_count = diagnostics.read_fail;
        last_capture_read_fail_count = capture.read_fail_count;
    }
}

static void imu_publish_filter_snapshot(const imu_raw_data_t *data)
{
    imu_calibrated_data_t calibrated_data;

    if (imu_boot_manager_is_ready() == 0U) {
        if (legacy_attitude_ready_latched != 0U) {
            attitude_zero_reset();
            legacy_attitude_ready_latched = 0U;
        }
        /* Never seed or advance the filter before the full boot sequence. */
        return;
    }
    if (data == NULL || data->online == 0U) {
        return;
    }
    calibrated_data = imu_calibration_apply(data);
    {
        float accel_input[3] = {calibrated_data.ax, calibrated_data.ay,
                                calibrated_data.az};
        float accel_output[3];
        float mag_input[3] = {calibrated_data.mx, calibrated_data.my,
                              calibrated_data.mz};
        float mag_output[3];

        /* LSM303 values were mapped to the Body Frame at acquisition. Apply
         * only the frozen leveling matrix here; do not mirror an axis twice. */
        imu_leveling_rotate_vector(&g_leveling_lsm, accel_input, accel_output);
        imu_leveling_rotate_vector(&g_leveling_lsm, mag_input, mag_output);
        calibrated_data.ax = accel_output[0];
        calibrated_data.ay = accel_output[1];
        calibrated_data.az = accel_output[2];
        calibrated_data.lsm_ax = accel_output[0];
        calibrated_data.lsm_ay = accel_output[1];
        calibrated_data.lsm_az = accel_output[2];
        calibrated_data.mx = mag_output[0];
        calibrated_data.my = mag_output[1];
        calibrated_data.mz = mag_output[2];
        calibrated_data.lsm_mx = calibrated_data.mx;
        calibrated_data.lsm_my = calibrated_data.my;
        calibrated_data.lsm_mz = calibrated_data.mz;
    }
    imu_filter_update(&calibrated_data);
    if (imu_filter_is_ready() != 0U) {
        attitude_update();
        if (legacy_attitude_ready_latched == 0U &&
            attitude_zero_capture_current() != 0U) {
            legacy_attitude_ready_latched = 1U;
        }
    }
}

static void imu_publish_unified_snapshot(void)
{
    imu_raw_data_t snapshot = {0};
    bmi323_data_t bmi = {0};
    lsm_accel_data_t accel = {0};
    lsm_mag_data_t mag = {0};
    uint8_t accel_valid;
    uint8_t mag_valid;

    imu_lock_lsm();
    accel = lsm_accel_data;
    mag = lsm_mag_data;
    accel_valid = lsm_accel_valid;
    mag_valid = lsm_mag_valid;
    imu_unlock_lsm();
    imu_lock_bmi();
    bmi = bmi_data;
    imu_unlock_bmi();

    snapshot.ax = accel.ax;
    snapshot.ay = accel.ay;
    snapshot.az = accel.az;
    snapshot.mx = mag.mx;
    snapshot.my = mag.my;
    snapshot.mz = mag.mz;
    snapshot.timestamp = accel_valid != 0U ? accel.timestamp : mag.timestamp;
    snapshot.timestamp_us = accel_valid != 0U ? accel.timestamp_us : mag.timestamp_us;
    snapshot.online = (accel_valid != 0U && mag_valid != 0U) ? 1U : 0U;
    snapshot.lsm_ax = snapshot.ax;
    snapshot.lsm_ay = snapshot.ay;
    snapshot.lsm_az = snapshot.az;
    snapshot.lsm_mx = snapshot.mx;
    snapshot.lsm_my = snapshot.my;
    snapshot.lsm_mz = snapshot.mz;
    snapshot.bmi_ax = bmi.accel_x;
    snapshot.bmi_ay = bmi.accel_y;
    snapshot.bmi_az = bmi.accel_z;
    snapshot.bmi_gx = bmi.gyro_x;
    snapshot.bmi_gy = bmi.gyro_y;
    snapshot.bmi_gz = bmi.gyro_z;
    snapshot.lsm_timestamp = snapshot.timestamp;
    snapshot.lsm_timestamp_us = snapshot.timestamp_us;
    snapshot.bmi_timestamp = bmi.timestamp;
    snapshot.bmi_timestamp_us = bmi.timestamp_us;
    snapshot.lsm_accel_valid = accel_valid;
    snapshot.lsm_mag_valid = mag_valid;
    snapshot.bmi_accel_valid = bmi.accel_valid;
    snapshot.bmi_gyro_valid = bmi.gyro_valid;

    imu_lock_snapshot();
    imu_snapshot = snapshot;
    imu_unlock_snapshot();

    if (snapshot.lsm_accel_valid != 0U || snapshot.lsm_mag_valid != 0U ||
        snapshot.bmi_accel_valid != 0U || snapshot.bmi_gyro_valid != 0U) {
        imu_boot_manager_update(&snapshot);
    } else {
        imu_boot_manager_step();
    }
    /* Only the legacy complete LSM view is allowed to reach the existing
     * calibration/filter/attitude chain. BMI323 remains telemetry-only. */
    if (snapshot.online != 0U) {
        imu_publish_filter_snapshot(&snapshot);
    }
}

static uint8_t imu_dual_ahrs_prepare(void)
{
    if (imu_boot_manager_is_ready() == 0U) {
        if (dual_ahrs_bias_injected != 0U) {
            dual_ahrs_set_bias(NULL);
            dual_ahrs_bias_injected = 0U;
        }
        return 0U;
    }

    if (dual_ahrs_bias_injected == 0U) {
        const imu_calibration_result_t result = imu_calibration_get_result();
        const dual_ahrs_bias_t bias = {
            .bmi_accel = {result.bmi_accel_bias.x,
                          result.bmi_accel_bias.y,
                          result.bmi_accel_bias.z},
            .bmi_gyro = {result.bmi_gyro_bias.x,
                         result.bmi_gyro_bias.y,
                         result.bmi_gyro_bias.z},
            .lsm_accel = {result.lsm_accel_bias.x,
                          result.lsm_accel_bias.y,
                          result.lsm_accel_bias.z},
        };
        dual_ahrs_set_bias(&bias);
        dual_ahrs_bias_injected = 1U;
    }
    return 1U;
}

void imu_manager_reset_leveling(void)
{
    imu_leveling_init(&g_leveling_bmi);
    imu_leveling_init(&g_leveling_lsm);
#if !SMARTCAR_BMI323_DEBUG_ONLY
    dual_ahrs_set_leveling(&g_leveling_bmi, &g_leveling_lsm);
    dual_ahrs_set_local_gravity(g_leveling_bmi.g_local_mps2);
#endif
}

void imu_manager_commit_leveling(void)
{
    const imu_calibration_static_statistics_t statistics =
        imu_calibration_get_static_statistics();

    (void)imu_leveling_compute_with_accel_std_limit(
        &g_leveling_lsm, statistics.lsm.accel_mean,
        statistics.lsm.gyro_rms_radps, statistics.lsm.accel_std_mps2,
        statistics.lsm.valid_ratio, LSM_ACCEL_STD_MAX);
    (void)imu_leveling_compute_with_accel_std_limit(
        &g_leveling_bmi, statistics.bmi.accel_mean,
        statistics.bmi.gyro_rms_radps, statistics.bmi.accel_std_mps2,
        statistics.bmi.valid_ratio, BMI_ACCEL_STD_MAX);
#if !SMARTCAR_BMI323_DEBUG_ONLY
    dual_ahrs_set_leveling(&g_leveling_bmi, &g_leveling_lsm);
    dual_ahrs_set_local_gravity(g_leveling_bmi.g_local_mps2);
#endif
}

void imu_manager_get_leveling_states(imu_leveling_state_t *bmi,
                                     imu_leveling_state_t *lsm)
{
    if (bmi != NULL) {
        *bmi = g_leveling_bmi;
    }
    if (lsm != NULL) {
        *lsm = g_leveling_lsm;
    }
}

/* Feed the independent DualAHRS path from the high-rate BMI producer. The
 * LSM303 values are copied under their existing lock and never enter the
 * legacy attitude/filter path through this hook. */
static void imu_dual_ahrs_feed_bmi(const bmi323_data_t *bmi_sample)
{
    lsm_accel_data_t lsm_accel = {0};
    lsm_mag_data_t lsm_mag = {0};
    dual_ahrs_input_t input = {0};

    if (bmi_sample == NULL) {
        return;
    }
    if (imu_dual_ahrs_prepare() == 0U) {
        return;
    }
    imu_lock_lsm();
    lsm_accel = lsm_accel_data;
    lsm_mag = lsm_mag_data;
    input.lsm_accel_valid = lsm_accel_valid;
    input.lsm_mag_valid = lsm_mag_valid;
    imu_unlock_lsm();
    input.bmi_accel = (dual_ahrs_vector3_t){bmi_sample->accel_x,
                                             bmi_sample->accel_y,
                                             bmi_sample->accel_z};
    /* BMI acceleration and gyro share the same frozen leveling rotation in
     * DualAHRS; no axis-specific sign change is permitted between them. */
    input.gyro = (dual_ahrs_vector3_t){bmi_sample->gyro_x, bmi_sample->gyro_y,
                                      bmi_sample->gyro_z};
    input.lsm_accel = (dual_ahrs_vector3_t){lsm_accel.ax, lsm_accel.ay,
                                            lsm_accel.az};
    input.mag = (dual_ahrs_vector3_t){lsm_mag.mx, lsm_mag.my, lsm_mag.mz};
    input.bmi_timestamp_us = bmi_sample->timestamp_us;
    input.lsm_timestamp_us = lsm_accel.timestamp_us != 0U
                                 ? lsm_accel.timestamp_us
                                 : lsm_mag.timestamp_us;
    input.bmi_accel_valid = bmi_sample->accel_valid;
    input.bmi_gyro_valid = bmi_sample->gyro_valid;
    dual_ahrs_update(&input);
}

static bsp_status_t imu_update_lsm303(void)
{
    lsm_accel_data_t next_accel;
    lsm_mag_data_t next_mag;
    Vector3f acc;
    Vector3f mag;
    bsp_status_t acc_status;
    bsp_status_t mag_status;
    bsp_status_t status;
    uint32_t accel_success_tick = 0U;
    uint32_t mag_success_tick = 0U;
    uint64_t accel_timestamp_us = 0U;
    uint64_t mag_timestamp_us = 0U;
    uint32_t timestamp;

    imu_lock_lsm();
    next_accel = lsm_accel_data;
    next_mag = lsm_mag_data;
    imu_unlock_lsm();

    acc_status = lsm303_read_acc(&acc);
    if (acc_status == BSP_STATUS_OK) {
        accel_timestamp_us = imu_time_now_us();
        accel_success_tick = (uint32_t)(accel_timestamp_us / UINT64_C(1000));
    }
    mag_status = lsm303_read_mag(&mag);
    if (mag_status == BSP_STATUS_OK) {
        mag_timestamp_us = imu_time_now_us();
        mag_success_tick = (uint32_t)(mag_timestamp_us / UINT64_C(1000));
    }
    status = acc_status != BSP_STATUS_OK ? acc_status : mag_status;
    timestamp = imu_time_now_ms();
    if (acc_status == BSP_STATUS_OK) {
        const Vector3f body_acc = lsm303_to_body(acc);

        /* Convert the installed LSM303 sensor frame to the vehicle Body
         * Frame before calibration and leveling. BMI323 remains unchanged. */
        next_accel.ax = body_acc.x;
        next_accel.ay = body_acc.y;
        next_accel.az = body_acc.z;
        next_accel.timestamp = accel_success_tick;
        next_accel.timestamp_us = accel_timestamp_us;
    }
    if (mag_status == BSP_STATUS_OK) {
        const Vector3f body_mag = lsm303_to_body(mag);

        next_mag.mx = body_mag.x;
        next_mag.my = body_mag.y;
        next_mag.mz = body_mag.z;
        next_mag.timestamp = mag_success_tick;
        next_mag.timestamp_us = mag_timestamp_us;
    }

    imu_lock_lsm();
    lsm_accel_data = next_accel;
    lsm_mag_data = next_mag;
    lsm_accel_valid = acc_status == BSP_STATUS_OK ? 1U : 0U;
    lsm_mag_valid = mag_status == BSP_STATUS_OK ? 1U : 0U;
    if (acc_status == BSP_STATUS_OK) {
        last_accel_success_tick = accel_success_tick;
        accel_fail_count = 0U;
    } else if (accel_fail_count < UINT8_MAX) {
        ++accel_fail_count;
    }
    if (mag_status == BSP_STATUS_OK) {
        last_mag_success_tick = mag_success_tick;
        mag_fail_count = 0U;
    } else if (mag_fail_count < UINT8_MAX) {
        ++mag_fail_count;
    }
    ++lsm_stats.read_calls;
    if (status == BSP_STATUS_OK) {
        ++lsm_stats.update_count;
        lsm_stats.last_update_ms = timestamp;
    }
    lsm_stats.last_status = status;
    imu_unlock_lsm();

    if (mag_status == BSP_STATUS_OK) {
        mag_filter_update(&next_mag);
    }

    return status;
}

static void imu_update_bmi323(void)
{
    bmi323_raw_sample_t raw_sample = {0};
    uint64_t capture_timestamp_us = 0U;

    if (imu_bmi_ring_take_latest(&raw_sample, &capture_timestamp_us) == 0U) {
        imu_lock_bmi();
        bmi_data.valid = 0U;
        bmi_data.accel_valid = 0U;
        bmi_data.gyro_valid = 0U;
        if (bmi_data.invalid_count != UINT32_MAX) {
            ++bmi_data.invalid_count;
        }
        if (bmi323_is_online() == 0U) {
            bmi_stats.last_status = BSP_STATUS_NOT_READY;
        }
        imu_unlock_bmi();
        return;
    }

    imu_lock_bmi();
    bmi_data.valid = raw_sample.valid;
    bmi_data.accel_valid = raw_sample.valid;
    bmi_data.gyro_valid = raw_sample.valid;
    bmi_data.accel_x = bmi323_accel_raw_to_mps2(raw_sample.accel[0]);
    bmi_data.accel_y = bmi323_accel_raw_to_mps2(raw_sample.accel[1]);
    bmi_data.accel_z = bmi323_accel_raw_to_mps2(raw_sample.accel[2]);
    bmi_data.gyro_x = bmi323_gyro_raw_to_rads(raw_sample.gyro[0]);
    bmi_data.gyro_y = bmi323_gyro_raw_to_rads(raw_sample.gyro[1]);
    bmi_data.gyro_z = bmi323_gyro_raw_to_rads(raw_sample.gyro[2]);
    bmi_data.timestamp_us = raw_sample.timestamp_us;
    bmi_data.timestamp =
        (uint32_t)(raw_sample.timestamp_us / UINT64_C(1000));
    bmi_data.sample_count = bmi_capture_stats.sample_count;
    bmi_stats.last_status = BSP_STATUS_OK;
    if (capture_timestamp_us != 0U) {
        const uint64_t latency_us =
            imu_time_now_us() - capture_timestamp_us;
        const uint32_t bounded_latency_us = latency_us > UINT32_MAX
                                                ? UINT32_MAX
                                                : (uint32_t)latency_us;
        if (bounded_latency_us > bmi_capture_stats.max_latency_us) {
            bmi_capture_stats.max_latency_us = bounded_latency_us;
        }
    }
    imu_unlock_bmi();
}

#if defined(IMU_MANAGER_USE_FREERTOS)
static TickType_t imu_bmi323_period_ticks(bmi323_sample_rate_t sample_rate,
                                           uint32_t *phase)
{
    const uint32_t rate_hz = (uint32_t)sample_rate;
    TickType_t delay_ticks;

    if (phase == NULL || rate_hz == 0U) {
        return 1U;
    }
    /* The caller seeds phase with rate_hz - 1, so this computes the
     * ceil-rounded cumulative tick boundary. For 800 Hz on a 1 kHz tick,
     * that produces 2/1/1/1 ticks instead of clamping every cycle to 1 tick. */
    *phase += (uint32_t)configTICK_RATE_HZ;
    delay_ticks = (TickType_t)(*phase / rate_hz);
    *phase %= rate_hz;
    return delay_ticks == 0U ? 1U : delay_ticks;
}

static void imu_bmi323_task(void *argument)
{
    bmi323_sample_rate_t previous_rate = bmi323_get_sample_rate();
    uint32_t phase = (uint32_t)previous_rate - 1U;
    TickType_t last_wake = xTaskGetTickCount();

    (void)argument;
    for (;;) {
        bmi323_raw_sample_t sample = {0};
        bool read_ok = false;
        uint64_t capture_timestamp_us;

        if (bmi323_acquisition_enabled == 0U) {
            vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(IMU_TASK_PERIOD_MS));
            continue;
        }

        if (imu_try_lock_bmi_driver() != 0U) {
            read_ok = bmi323_is_online() != 0U &&
                      bmi323_read_raw_sample(sample.accel, sample.gyro);
            imu_unlock_bmi_driver();
            capture_timestamp_us = imu_time_now_us();
            if (read_ok) {
                sample.timestamp_us = capture_timestamp_us;
                sample.valid = 1U;
                if (imu_calibration_bmi_capture_active() != 0U) {
                    imu_calibration_update_bmi323(
                        bmi323_accel_raw_to_mps2(sample.accel[0]),
                        bmi323_accel_raw_to_mps2(sample.accel[1]),
                        bmi323_accel_raw_to_mps2(sample.accel[2]),
                        bmi323_gyro_raw_to_rads(sample.gyro[0]),
                        bmi323_gyro_raw_to_rads(sample.gyro[1]),
                        bmi323_gyro_raw_to_rads(sample.gyro[2]),
                        sample.timestamp_us);
                }
                imu_bmi_ring_push(&sample);
                {
                    const bmi323_data_t dual_sample = {
                        .accel_x = bmi323_accel_raw_to_mps2(sample.accel[0]),
                        .accel_y = bmi323_accel_raw_to_mps2(sample.accel[1]),
                        .accel_z = bmi323_accel_raw_to_mps2(sample.accel[2]),
                        .gyro_x = bmi323_gyro_raw_to_rads(sample.gyro[0]),
                        .gyro_y = bmi323_gyro_raw_to_rads(sample.gyro[1]),
                        .gyro_z = bmi323_gyro_raw_to_rads(sample.gyro[2]),
                        .timestamp = (uint32_t)(capture_timestamp_us /
                                                UINT64_C(1000)),
                        .timestamp_us = capture_timestamp_us,
                        .valid = 1U,
                        .accel_valid = 1U,
                        .gyro_valid = 1U,
                    };
                    imu_dual_ahrs_feed_bmi(&dual_sample);
                }
            } else {
                const bmi323_data_t dual_sample = {
                    .timestamp = (uint32_t)(capture_timestamp_us /
                                            UINT64_C(1000)),
                    .timestamp_us = capture_timestamp_us,
                    .valid = 0U,
                    .accel_valid = 0U,
                    .gyro_valid = 0U,
                };
                imu_bmi_capture_failed();
                imu_dual_ahrs_feed_bmi(&dual_sample);
            }
        } else {
            /* Recovery or ODR reconfiguration owns the driver briefly. Do not
             * delay the high-rate producer; record the skipped interval. */
            imu_bmi_capture_note_contention_drop();
            capture_timestamp_us = imu_time_now_us();
            {
                const bmi323_data_t dual_sample = {
                    .timestamp = (uint32_t)(capture_timestamp_us /
                                            UINT64_C(1000)),
                    .timestamp_us = capture_timestamp_us,
                    .valid = 0U,
                    .accel_valid = 0U,
                    .gyro_valid = 0U,
                };
                imu_dual_ahrs_feed_bmi(&dual_sample);
            }
        }

        const bmi323_sample_rate_t current_rate = bmi323_get_sample_rate();
        if (current_rate != previous_rate) {
            previous_rate = current_rate;
            phase = (uint32_t)current_rate - 1U;
        }
        vTaskDelayUntil(&last_wake,
                        imu_bmi323_period_ticks(current_rate, &phase));
    }
}
#endif

#if defined(IMU_MANAGER_USE_FREERTOS)
static void imu_dual_lsm_init_task(void *argument)
{
    bsp_status_t status;

    (void)argument;
    /* The caller creates both workers before releasing either notification.
     * A task notification remains pending if this task has not run yet, so
     * INIT does not depend on the caller's priority relative to these workers. */
    (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
#if SMARTCAR_BMI323_DEBUG_ONLY
    status = BSP_STATUS_UNSUPPORTED;
#else
    imu_init_log("LSM303_INIT_BEGIN\r\n");
    status = lsm303_init();
    imu_init_log_status("LSM303_INIT_END", status);
#endif
    imu_mark_lsm_initialized(status);

    imu_lock_dual_init();
    dual_init_status.lsm_complete = 1U;
    dual_init_status.lsm_success = status == BSP_STATUS_OK ? 1U : 0U;
    dual_init_status.lsm_end_time = imu_time_now_ms();
    dual_lsm_init_task_handle = NULL;
    imu_unlock_dual_init();
    vTaskDelete(NULL);
}

static void imu_dual_bmi_init_task(void *argument)
{
    bool initialized;

    (void)argument;
    /* See imu_dual_lsm_init_task(): both buses leave the start gate together. */
    (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    imu_lock_bmi_driver();
    initialized = bmi323_init();
    imu_unlock_bmi_driver();
    imu_mark_bmi323_initialized(initialized ? 1U : 0U);
    imu_bmi323_init_log();

    imu_lock_dual_init();
    dual_init_status.bmi_complete = 1U;
    dual_init_status.bmi_success = initialized ? 1U : 0U;
    dual_init_status.bmi_end_time = imu_time_now_ms();
    dual_bmi_init_task_handle = NULL;
    imu_unlock_dual_init();
    vTaskDelete(NULL);
}
#endif

bsp_status_t imu_manager_start_dual_initialization(void)
{
#if defined(IMU_MANAGER_USE_FREERTOS)
    BaseType_t lsm_task_status;
    BaseType_t bmi_task_status;
    const uint32_t now_ms = imu_time_now_ms();

    if (imu_prepared == 0U || imu_create_data_locks() != BSP_STATUS_OK) {
        return BSP_STATUS_NOT_READY;
    }
#if SMARTCAR_BMI323_DEBUG_ONLY
    return BSP_STATUS_UNSUPPORTED;
#else
    imu_lock_dual_init();
    if (dual_init_status.lsm_started != 0U || dual_init_status.bmi_started != 0U) {
        const uint8_t complete = dual_init_status.lsm_complete != 0U &&
                                 dual_init_status.bmi_complete != 0U;
        const uint8_t success = dual_init_status.lsm_success != 0U &&
                                dual_init_status.bmi_success != 0U;
        imu_unlock_dual_init();
        return complete != 0U && success != 0U ? BSP_STATUS_OK : BSP_STATUS_NOT_READY;
    }
    dual_init_status = (imu_dual_init_status_t){
        .lsm_started = 1U,
        .bmi_started = 1U,
        .lsm_start_time = now_ms,
        .bmi_start_time = now_ms,
    };
    imu_unlock_dual_init();

    /* Stop the independent producer before either driver is reset. */
    bmi323_acquisition_enabled = 0U;
    imu_initialized = 0U;
    imu_reset_data(1U);

    lsm_task_status = xTaskCreate(imu_dual_lsm_init_task, "imu_lsm_init",
                                  DUAL_IMU_INIT_TASK_STACK_WORDS, NULL,
                                  DUAL_IMU_INIT_TASK_PRIORITY,
                                  &dual_lsm_init_task_handle);
    if (lsm_task_status != pdPASS) {
        imu_lock_dual_init();
        dual_init_status.lsm_complete = 1U;
        dual_init_status.bmi_complete = 1U;
        dual_init_status.lsm_success = 0U;
        dual_init_status.bmi_success = 0U;
        dual_init_status.lsm_end_time = now_ms;
        dual_init_status.bmi_end_time = now_ms;
        imu_unlock_dual_init();
        return BSP_STATUS_ERROR;
    }
    bmi_task_status = xTaskCreate(imu_dual_bmi_init_task, "imu_bmi_init",
                                  DUAL_IMU_INIT_TASK_STACK_WORDS, NULL,
                                  DUAL_IMU_INIT_TASK_PRIORITY,
                                  &dual_bmi_init_task_handle);
    if (bmi_task_status != pdPASS) {
        vTaskDelete(dual_lsm_init_task_handle);
        dual_lsm_init_task_handle = NULL;
        imu_lock_dual_init();
        dual_init_status.lsm_complete = 1U;
        dual_init_status.bmi_complete = 1U;
        dual_init_status.lsm_success = 0U;
        dual_init_status.bmi_success = 0U;
        dual_init_status.lsm_end_time = now_ms;
        dual_init_status.bmi_end_time = now_ms;
        imu_unlock_dual_init();
        return BSP_STATUS_ERROR;
    }
    /* Release both workers while scheduling is suspended. This is a real
     * common start gate, independent of task priority or creation order. */
    vTaskSuspendAll();
    (void)xTaskNotifyGive(dual_lsm_init_task_handle);
    (void)xTaskNotifyGive(dual_bmi_init_task_handle);
    (void)xTaskResumeAll();
    return BSP_STATUS_OK;
#endif
#else
    return BSP_STATUS_UNSUPPORTED;
#endif
}

void imu_manager_get_dual_initialization_status(imu_dual_init_status_t *status)
{
    if (status == NULL) {
        return;
    }
    imu_lock_dual_init();
    *status = dual_init_status;
    imu_unlock_dual_init();
}

uint8_t imu_manager_finalize_dual_initialization(void)
{
    imu_dual_init_status_t status = {0};

    imu_manager_get_dual_initialization_status(&status);
    if (status.lsm_complete == 0U || status.bmi_complete == 0U ||
        status.lsm_success == 0U || status.bmi_success == 0U) {
        return 0U;
    }
    /* Primary AHRS is designed for the fixed 200 Hz BMI323 input stream.
     * Apply the ODR after hardware init so a previous runtime-rate request
     * cannot silently desynchronize the estimator and its anti-alias filter. */
    if (imu_manager_set_bmi323_sample_rate(BMI323_SAMPLE_RATE_200HZ) !=
        BSP_STATUS_OK) {
        return 0U;
    }
    imu_initialized = 1U;
    bmi323_acquisition_enabled = 1U;
    if (imu_manager_start_bmi323_task() != BSP_STATUS_OK) {
        bmi323_acquisition_enabled = 0U;
        imu_initialized = 0U;
        return 0U;
    }
    return 1U;
}

bsp_status_t imu_manager_set_bmi323_sample_rate(bmi323_sample_rate_t sample_rate)
{
    uint8_t success;

    /* The active Primary AHRS filter is designed for BMI323 ODR=200 Hz.
     * Reject other manager-level rates instead of allowing a silent LPF and
     * decimation mismatch. The low-level driver enum remains available for
     * isolated diagnostics that do not feed DualAHRS. */
    if (sample_rate != BMI323_SAMPLE_RATE_200HZ) {
        return BSP_STATUS_INVALID_ARG;
    }
    if (imu_create_data_locks() != BSP_STATUS_OK) {
        return BSP_STATUS_ERROR;
    }
    imu_lock_bmi_driver();
    success = bmi323_set_sample_rate(sample_rate) ? 1U : 0U;
    imu_unlock_bmi_driver();
    if (success == 0U) {
        return BSP_STATUS_INVALID_ARG;
    }
    imu_lock_bmi();
    imu_bmi_ring_reset_locked();
    imu_unlock_bmi();
    return BSP_STATUS_OK;
}

bmi323_sample_rate_t imu_manager_get_bmi323_sample_rate(void)
{
    return bmi323_get_sample_rate();
}

bsp_status_t imu_manager_get_bmi323_capture_stats(bmi323_capture_stat_t *stats)
{
    if (stats == NULL) {
        return BSP_STATUS_INVALID_ARG;
    }
    if (imu_initialized == 0U) {
        return BSP_STATUS_NOT_READY;
    }
    imu_lock_bmi();
    *stats = bmi_capture_stats;
    stats->overflow_count = bmi_ring_buffer.overflow_count;
    stats->contention_drop_count = bmi_capture_contention_drop_count;
    stats->pending_count = bmi_ring_buffer.count;
    stats->configured_rate_hz = (uint16_t)bmi323_get_sample_rate();
    stats->measured_rate_hz = imu_bmi_measured_rate_hz(stats);
    imu_unlock_bmi();
    return BSP_STATUS_OK;
}

bsp_status_t imu_manager_start_bmi323_task(void)
{
#if defined(IMU_MANAGER_USE_FREERTOS)
    BaseType_t task_status;

    if (imu_initialized == 0U) {
        return BSP_STATUS_NOT_READY;
    }
    if (bmi323_task_started != 0U) {
        return BSP_STATUS_OK;
    }
    task_status = xTaskCreate(imu_bmi323_task, "bmi323_task",
                              BMI323_TASK_STACK_WORDS, NULL,
                              BMI323_TASK_PRIORITY, &bmi323_task_handle);
    if (task_status != pdPASS) {
        return BSP_STATUS_ERROR;
    }
    bmi323_task_started = 1U;
    return BSP_STATUS_OK;
#else
    return BSP_STATUS_UNSUPPORTED;
#endif
}

static bsp_status_t imu_prepare_lifecycle(uint8_t reset_count)
{
    if (imu_create_data_locks() != BSP_STATUS_OK) {
        return BSP_STATUS_ERROR;
    }

    /* Hardware drivers are deliberately not called here. DUAL_IMU_BOOT/INIT
     * owns both worker launches and prevents either sensor from starting a
     * later lifecycle phase by itself. */
    bmi323_acquisition_enabled = 0U;
    imu_initialized = 0U;
    dual_ahrs_bias_injected = 0U;
    imu_manager_reset_leveling();
    imu_reset_data(reset_count);
    imu_lock_dual_init();
    dual_init_status = (imu_dual_init_status_t){0};
    imu_unlock_dual_init();
    imu_prepared = 1U;

#if !SMARTCAR_BMI323_DEBUG_ONLY
    imu_calibration_init();
    imu_filter_init();
    mag_filter_init();
    imu_boot_manager_init();
#endif
    return BSP_STATUS_OK;
}

#if SMARTCAR_BMI323_DEBUG_ONLY
static bsp_status_t imu_init_bmi_debug(void)
{
    bool initialized;

    if (imu_create_data_locks() != BSP_STATUS_OK) {
        return BSP_STATUS_ERROR;
    }
    imu_reset_data(1U);
    imu_manager_reset_leveling();
    imu_lock_bmi_driver();
    initialized = bmi323_init();
    imu_unlock_bmi_driver();
    imu_mark_bmi323_initialized(initialized ? 1U : 0U);
    imu_bmi323_init_log();
    imu_prepared = 1U;
    imu_initialized = initialized ? 1U : 0U;
    bmi323_acquisition_enabled = initialized ? 1U : 0U;
    return initialized ? BSP_STATUS_OK : BSP_STATUS_ERROR;
}
#endif

bsp_status_t imu_init(void)
{
    if (imu_time_init() != BSP_STATUS_OK) {
        return BSP_STATUS_ERROR;
    }
#if SMARTCAR_BMI323_DEBUG_ONLY
    return imu_init_bmi_debug();
#else
    return imu_prepare_lifecycle(1U);
#endif
}

bsp_status_t imu_recover(void)
{
    if (imu_time_init() != BSP_STATUS_OK) {
        return BSP_STATUS_ERROR;
    }
#if SMARTCAR_BMI323_DEBUG_ONLY
    return imu_init_bmi_debug();
#else
    if (imu_prepare_lifecycle(1U) != BSP_STATUS_OK) {
        return BSP_STATUS_ERROR;
    }
    imu_init_log("[IMU_RECOVER] dual lifecycle reset\r\n");
    return BSP_STATUS_OK;
#endif
}

bsp_status_t imu_update(void)
{
#if SMARTCAR_BMI323_DEBUG_ONLY
    if (imu_initialized == 0U) {
        return BSP_STATUS_NOT_READY;
    }
    /* Keep BMI323 acquisition isolated from LSM303 and the AHRS path. */
    imu_update_bmi323();
    imu_publish_unified_snapshot();
    return BSP_STATUS_OK;
#else
    bsp_status_t lsm_status;

    if (imu_prepared == 0U) {
        return BSP_STATUS_NOT_READY;
    }
    if (imu_initialized == 0U) {
        /* INIT workers are running or the lifecycle is terminal. Returning
         * success avoids a legacy recovery loop from starting a sequential
         * initialization path before DUAL_IMU_BOOT decides the outcome. */
        return BSP_STATUS_OK;
    }

    lsm_status = imu_update_lsm303();
    imu_update_bmi323();
    imu_publish_unified_snapshot();
    return lsm_status;
#endif
}

bsp_status_t imu_get_bmi323_data(bmi323_data_t *data)
{
    if (data == NULL) {
        return BSP_STATUS_INVALID_ARG;
    }
    if (imu_initialized == 0U) {
        return BSP_STATUS_NOT_READY;
    }

    imu_lock_bmi();
    *data = bmi_data;
    imu_unlock_bmi();
    return BSP_STATUS_OK;
}

bsp_status_t imu_manager_get_lsm_accel(lsm_accel_data_t *data)
{
    if (data == NULL) {
        return BSP_STATUS_INVALID_ARG;
    }
    if (imu_initialized == 0U) {
        return BSP_STATUS_NOT_READY;
    }

    imu_lock_lsm();
    *data = lsm_accel_data;
    imu_unlock_lsm();
    return BSP_STATUS_OK;
}

bsp_status_t imu_manager_get_lsm_mag(lsm_mag_data_t *data)
{
    if (data == NULL) {
        return BSP_STATUS_INVALID_ARG;
    }
    if (imu_initialized == 0U) {
        return BSP_STATUS_NOT_READY;
    }

    imu_lock_lsm();
    *data = lsm_mag_data;
    imu_unlock_lsm();
    return BSP_STATUS_OK;
}

bsp_status_t imu_manager_get_snapshot(imu_raw_data_t *snapshot)
{
    if (snapshot == NULL) {
        return BSP_STATUS_INVALID_ARG;
    }
    if (imu_initialized == 0U) {
        return BSP_STATUS_NOT_READY;
    }

    imu_lock_snapshot();
    *snapshot = imu_snapshot;
    imu_unlock_snapshot();
    return BSP_STATUS_OK;
}

bsp_status_t imu_get_bmi323_stats(imu_sensor_stats_t *stats)
{
    if (stats == NULL) {
        return BSP_STATUS_INVALID_ARG;
    }
    if (imu_initialized == 0U) {
        return BSP_STATUS_NOT_READY;
    }

    imu_lock_bmi();
    *stats = bmi_stats;
    imu_unlock_bmi();
    return BSP_STATUS_OK;
}

bsp_status_t imu_get_lsm303_stats(imu_sensor_stats_t *stats)
{
    if (stats == NULL) {
        return BSP_STATUS_INVALID_ARG;
    }
    if (imu_initialized == 0U) {
        return BSP_STATUS_NOT_READY;
    }

    imu_lock_lsm();
    *stats = lsm_stats;
    imu_unlock_lsm();
    return BSP_STATUS_OK;
}

bsp_status_t imu_task_step(void)
{
    return imu_update();
}

void imu_task(void *argument)
{
    (void)argument;
    imu_init_log("[INFO] IMU_TASK_RUN\r\n");
#if defined(IMU_MANAGER_USE_FREERTOS)
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(IMU_TASK_PERIOD_MS);
    uint32_t last_recovery_ms = imu_time_now_ms();
    uint32_t last_health_ms = last_recovery_ms;
    for (;;) {
#if !SMARTCAR_BMI323_DEBUG_ONLY
        imu_boot_manager_step();
#endif
        const bsp_status_t status = imu_task_step();
        const uint32_t now_ms = imu_time_now_ms();
        if (status != BSP_STATUS_OK &&
#if !SMARTCAR_BMI323_DEBUG_ONLY
            imu_boot_manager_is_ready() != 0U &&
#endif
            (uint32_t)(now_ms - last_recovery_ms) >= IMU_RECOVERY_PERIOD_MS) {
            last_recovery_ms = now_ms;
            (void)imu_recover();
        }
        if ((uint32_t)(now_ms - last_health_ms) >= IMU_LSM303_HEALTH_PERIOD_MS) {
            last_health_ms = now_ms;
#if !SMARTCAR_BMI323_DEBUG_ONLY
            imu_lsm_health_log(now_ms);
#endif
            imu_bmi323_debug_log();
        }
        vTaskDelayUntil(&last_wake, period);
    }
#else
    uint32_t last_health_ms = imu_time_now_ms();
    for (;;) {
        const uint32_t now_ms = imu_time_now_ms();
        (void)imu_task_step();
        if ((uint32_t)(now_ms - last_health_ms) >= IMU_LSM303_HEALTH_PERIOD_MS) {
            last_health_ms = now_ms;
#if !SMARTCAR_BMI323_DEBUG_ONLY
            imu_lsm_health_log(now_ms);
#endif
            imu_bmi323_debug_log();
        }
        imu_delay_ms(IMU_TASK_PERIOD_MS);
    }
#endif
}

uint8_t imu_is_ready(void)
{
    if (imu_initialized == 0U) {
        return 0U;
    }
#if SMARTCAR_BMI323_DEBUG_ONLY
    return bmi323_init_success != 0U && bmi323_is_online() != 0U ? 1U : 0U;
#else
    return imu_lsm_is_online();
#endif
}

uint8_t imu_manager_get_lsm303_init_success(void)
{
    uint8_t initialized;

    imu_lock_lsm();
    initialized = lsm303_init_success;
    imu_unlock_lsm();
    return initialized;
}

uint8_t imu_manager_get_lsm303_online(void)
{
    return imu_lsm_is_online();
}
