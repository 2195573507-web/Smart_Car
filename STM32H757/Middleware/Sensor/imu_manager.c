#include "imu_manager.h"

#include "bsp_timer.h"
#include "attitude.h"
#include "boot_log.h"
#include "lsm303.h"
#include "imu_calibration.h"
#include "imu_boot_manager.h"
#include "imu_filter.h"
#include "mag_filter.h"
#include "log_service.h"

#include <stdio.h>

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

/* BMI323 state remains exposed for future restoration, but is not sampled. */
static bmi323_data_t bmi_data;
static lsm_accel_data_t lsm_accel_data;
static lsm_mag_data_t lsm_mag_data;
static imu_sensor_stats_t bmi_stats;
static imu_sensor_stats_t lsm_stats;
static volatile uint8_t imu_initialized;
static uint8_t lsm303_init_success;
static uint32_t last_accel_success_tick;
static uint32_t last_mag_success_tick;
static uint8_t accel_fail_count;
static uint8_t mag_fail_count;

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
static SemaphoreHandle_t lsm_data_mutex;
#endif

#if !defined(IMU_MANAGER_USE_FREERTOS)
static void imu_delay_ms(uint32_t delay_ms)
{
    const uint32_t start = timer_get_ms();
    while ((uint32_t)(timer_get_ms() - start) < delay_ms) {
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
    if (lsm_data_mutex == NULL) {
        lsm_data_mutex = xSemaphoreCreateMutex();
        if (lsm_data_mutex == NULL) {
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

static void imu_reset_data(uint8_t reset_count)
{
    imu_lock_bmi();
    bmi_data = (bmi323_data_t){0};
    if (reset_count != 0U) {
        bmi_stats = (imu_sensor_stats_t){0};
    }
    bmi_stats.last_status = BSP_STATUS_NOT_READY;
    imu_unlock_bmi();

    imu_lock_lsm();
    lsm_accel_data = (lsm_accel_data_t){0};
    lsm_mag_data = (lsm_mag_data_t){0};
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
}

static void imu_mark_lsm_initialized(bsp_status_t status)
{
    imu_lock_lsm();
    lsm303_init_success = status == BSP_STATUS_OK ? 1U : 0U;
    last_accel_success_tick = 0U;
    last_mag_success_tick = 0U;
    accel_fail_count = IMU_LSM303_FAIL_LIMIT;
    mag_fail_count = IMU_LSM303_FAIL_LIMIT;
    /* Keep the aggregate status for diagnostics; online uses independent
     * initialization, freshness, and failure health. */
    lsm_stats.last_status = status;
    imu_unlock_lsm();
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

    imu_lsm_health_snapshot(timer_get_ms(), &health);
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

static void imu_publish_filter_snapshot(const imu_raw_data_t *data)
{
    imu_calibrated_data_t calibrated_data;

    if (data == NULL || data->online == 0U) {
        return;
    }

    imu_boot_manager_update(data);
    if (imu_boot_manager_is_ready() == 0U) {
        /* Never seed or advance the filter before the full boot sequence. */
        return;
    }
    calibrated_data = imu_calibration_apply(data);
    imu_filter_update(&calibrated_data);
    if (imu_filter_is_ready() != 0U) {
        attitude_update();
    }
}

static bsp_status_t imu_update_lsm303(void)
{
    lsm_accel_data_t next_accel;
    lsm_mag_data_t next_mag;
    imu_raw_data_t raw_snapshot;
    Vector3f acc;
    Vector3f mag;
    bsp_status_t acc_status;
    bsp_status_t mag_status;
    bsp_status_t status;
    uint32_t accel_success_tick = 0U;
    uint32_t mag_success_tick = 0U;
    uint32_t timestamp;

    imu_lock_lsm();
    next_accel = lsm_accel_data;
    next_mag = lsm_mag_data;
    imu_unlock_lsm();

    acc_status = lsm303_read_acc(&acc);
    if (acc_status == BSP_STATUS_OK) {
        accel_success_tick = timer_get_ms();
    }
    mag_status = lsm303_read_mag(&mag);
    if (mag_status == BSP_STATUS_OK) {
        mag_success_tick = timer_get_ms();
    }
    status = acc_status != BSP_STATUS_OK ? acc_status : mag_status;
    timestamp = timer_get_ms();
    if (acc_status == BSP_STATUS_OK) {
        next_accel.ax = acc.x;
        next_accel.ay = acc.y;
        next_accel.az = acc.z;
    }
    if (mag_status == BSP_STATUS_OK) {
        next_mag.mx = mag.x;
        next_mag.my = mag.y;
        next_mag.mz = mag.z;
    }

    imu_lock_lsm();
    lsm_accel_data = next_accel;
    lsm_mag_data = next_mag;
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

    if (status == BSP_STATUS_OK) {
        raw_snapshot.ax = next_accel.ax;
        raw_snapshot.ay = next_accel.ay;
        raw_snapshot.az = next_accel.az;
        raw_snapshot.mx = next_mag.mx;
        raw_snapshot.my = next_mag.my;
        raw_snapshot.mz = next_mag.mz;
        raw_snapshot.timestamp = timestamp;
        raw_snapshot.online = 1U;
        mag_filter_update(&next_mag);
        /* Publish only a complete raw sample through calibration to the filter. */
        imu_publish_filter_snapshot(&raw_snapshot);
    } else {
        /* Give calibration a chance to convert a sustained sensor outage into
         * its terminal ERROR state instead of silently starving the window. */
        imu_boot_manager_step();
    }

    return status;
}

static const char *imu_boot_state_name(imu_boot_state_t state)
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

static bsp_status_t imu_init_internal(uint8_t reset_count)
{
    bsp_status_t lsm_status;
    bsp_status_t init_status;

    if (reset_count != 0U) {
        boot_log("IMU", "INIT START");
    }
    if (imu_create_data_locks() != BSP_STATUS_OK) {
        return BSP_STATUS_ERROR;
    }

    imu_initialized = 0U;
    imu_lock_lsm();
    lsm303_init_success = 0U;
    last_accel_success_tick = 0U;
    last_mag_success_tick = 0U;
    accel_fail_count = IMU_LSM303_FAIL_LIMIT;
    mag_fail_count = IMU_LSM303_FAIL_LIMIT;
    imu_unlock_lsm();
    if (reset_count != 0U) {
        imu_reset_data(1U);
        imu_calibration_init();
        imu_vibration_init();
        imu_filter_init();
        mag_filter_init();
        boot_log("LSM303", "INIT START");
        boot_log("LSM303", "DEVICE CHECK START");
        imu_init_log("LSM303_INIT_BEGIN\r\n");
        lsm_status = lsm303_init();
        imu_init_log_status("LSM303_INIT_END", lsm_status);
        boot_log("LSM303", lsm_status == BSP_STATUS_OK ? "INIT OK" : "INIT FAIL");
        boot_log("BMI323", "SKIPPED");
    } else {
        imu_init_log("LSM303_INIT_BEGIN\r\n");
        lsm_status = (lsm303_is_ready() != 0U && imu_lsm_is_online() != 0U)
                         ? BSP_STATUS_OK
                         : lsm303_init();
        imu_init_log_status("LSM303_INIT_END", lsm_status);
    }
    imu_mark_lsm_initialized(lsm_status);

    if (reset_count != 0U) {
        imu_boot_manager_init(lsm_status);
    }

    imu_initialized = 1U;
    init_status = lsm_status;
    if (reset_count != 0U) {
        boot_log("IMU", "READY");
    }
    return init_status;
}

bsp_status_t imu_init(void)
{
    (void)timer_init();
    return imu_init_internal(1U);
}

bsp_status_t imu_recover(void)
{
    char line[128];
    const imu_boot_state_t old_state = imu_boot_manager_get_state();

    (void)timer_init();
    const bsp_status_t status = imu_init_internal(0U);
    const imu_boot_state_t new_state = imu_boot_manager_get_state();

    (void)snprintf(line, sizeof(line),
                   "[IMU_RECOVER]\r\nold_state=%s\r\nnew_state=%s\r\n",
                   imu_boot_state_name(old_state), imu_boot_state_name(new_state));
    imu_init_log(line);
    imu_init_log("[BOOT_CAL] state preserved\r\n");
    return status;
}

bsp_status_t imu_update(void)
{
    if (imu_initialized == 0U) {
        return BSP_STATUS_NOT_READY;
    }

    return imu_update_lsm303();
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
    uint32_t last_recovery_ms = timer_get_ms();
    uint32_t last_health_ms = last_recovery_ms;
    for (;;) {
        imu_boot_manager_step();
        const bsp_status_t status = imu_task_step();
        const uint32_t now_ms = timer_get_ms();
        if (status != BSP_STATUS_OK &&
            (uint32_t)(now_ms - last_recovery_ms) >= IMU_RECOVERY_PERIOD_MS) {
            last_recovery_ms = now_ms;
            (void)imu_recover();
        }
        if ((uint32_t)(now_ms - last_health_ms) >= IMU_LSM303_HEALTH_PERIOD_MS) {
            last_health_ms = now_ms;
            imu_lsm_health_log(now_ms);
        }
        vTaskDelayUntil(&last_wake, period);
    }
#else
    uint32_t last_health_ms = timer_get_ms();
    for (;;) {
        const uint32_t now_ms = timer_get_ms();
        (void)imu_task_step();
        if ((uint32_t)(now_ms - last_health_ms) >= IMU_LSM303_HEALTH_PERIOD_MS) {
            last_health_ms = now_ms;
            imu_lsm_health_log(now_ms);
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
    return imu_lsm_is_online();
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
