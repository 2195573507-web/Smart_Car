#include "imu_manager.h"

#include "bsp_uart.h"
#include "bsp_timer.h"
#include "boot_log.h"
#include "lsm303.h"
#include "imu_calibration.h"
#include "imu_filter.h"

#include <stdio.h>

#if defined(IMU_MANAGER_USE_FREERTOS)
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#endif

#define IMU_TASK_PERIOD_MS       UINT32_C(10)
#define IMU_RECOVERY_PERIOD_MS   UINT32_C(1000)
#define IMU_INIT_LOG_TIMEOUT_MS  UINT32_C(100)

/* BMI323 state remains exposed for future restoration, but is not sampled. */
static bmi323_data_t bmi_data;
static lsm303_data_t lsm_data;
static imu_sensor_stats_t bmi_stats;
static imu_sensor_stats_t lsm_stats;
static volatile uint8_t imu_initialized;

static void imu_init_log(const char *text)
{
    if (text != NULL) {
        (void)uart_log_write(text, IMU_INIT_LOG_TIMEOUT_MS);
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
    lsm_data = (lsm303_data_t){0};
    if (reset_count != 0U) {
        lsm_stats = (imu_sensor_stats_t){0};
    }
    lsm_stats.last_status = BSP_STATUS_NOT_READY;
    imu_unlock_lsm();
}

static void imu_set_lsm_online(bsp_status_t status)
{
    imu_lock_lsm();
    lsm_data.online = status == BSP_STATUS_OK ? 1U : 0U;
    lsm_stats.last_status = status;
    imu_unlock_lsm();
}

static uint8_t imu_lsm_is_online(void)
{
    uint8_t online;

    imu_lock_lsm();
    online = lsm_data.online;
    imu_unlock_lsm();
    return online;
}

static void imu_publish_filter_snapshot(const lsm303_data_t *data)
{
    imu_raw_data_t raw_data;
    imu_calibrated_data_t calibrated_data;

    if (data == NULL) {
        return;
    }

    raw_data.ax = data->ax;
    raw_data.ay = data->ay;
    raw_data.az = data->az;
    raw_data.mx = data->mx;
    raw_data.my = data->my;
    raw_data.mz = data->mz;
    raw_data.timestamp = data->timestamp;
    raw_data.online = data->online;
    imu_calibration_update(&raw_data);
    imu_calibration_update_mag();
    calibrated_data = imu_calibration_get_data();
    imu_filter_update(&calibrated_data);
}

static bsp_status_t imu_update_lsm303(void)
{
    lsm303_data_t next;
    Vector3f acc;
    Vector3f mag;
    bsp_status_t acc_status;
    bsp_status_t mag_status;
    bsp_status_t status;

    imu_lock_lsm();
    next = lsm_data;
    imu_unlock_lsm();

    acc_status = lsm303_read_acc(&acc);
    mag_status = lsm303_read_mag(&mag);
    status = acc_status != BSP_STATUS_OK ? acc_status : mag_status;
    next.timestamp = timer_get_ms();
    next.online = status == BSP_STATUS_OK ? 1U : 0U;
    if (acc_status == BSP_STATUS_OK) {
        next.ax = acc.x;
        next.ay = acc.y;
        next.az = acc.z;
    }
    if (mag_status == BSP_STATUS_OK) {
        next.mx = mag.x;
        next.my = mag.y;
        next.mz = mag.z;
    }

    imu_lock_lsm();
    lsm_data = next;
    ++lsm_stats.read_calls;
    if (status == BSP_STATUS_OK) {
        ++lsm_stats.update_count;
        lsm_stats.last_update_ms = next.timestamp;
    }
    lsm_stats.last_status = status;
    imu_unlock_lsm();

    if (status == BSP_STATUS_OK) {
        /* Publish only a complete raw sample through calibration to the filter. */
        imu_publish_filter_snapshot(&next);
    }

    return status;
}

static bsp_status_t imu_init_internal(uint8_t reset_count)
{
    bsp_status_t lsm_status;
    bsp_status_t update_status;
    bsp_status_t init_status;

    if (reset_count != 0U) {
        boot_log("IMU", "INIT START");
    }
    if (imu_create_data_locks() != BSP_STATUS_OK) {
        return BSP_STATUS_ERROR;
    }

    imu_initialized = 0U;
    if (reset_count != 0U) {
        imu_reset_data(1U);
        imu_calibration_init();
        imu_calibration_start_accel_calibration();
        imu_filter_init();
        boot_log("LSM303", "INIT START");
        boot_log("LSM303", "DEVICE CHECK");
        imu_init_log("LSM303_INIT_BEGIN\r\n");
        lsm_status = lsm303_init();
        imu_init_log_status("LSM303_INIT_END", lsm_status);
        boot_log("LSM303", lsm_status == BSP_STATUS_OK ? "INIT OK" : "INIT FAIL");
        boot_log("BMI323", "SKIPPED");
    } else {
        imu_init_log("LSM303_INIT_BEGIN\r\n");
        lsm_status = imu_lsm_is_online() != 0U ? BSP_STATUS_OK : lsm303_init();
        imu_init_log_status("LSM303_INIT_END", lsm_status);
    }
    imu_set_lsm_online(lsm_status);

    imu_initialized = 1U;
    update_status = imu_update();
    if (lsm_status != BSP_STATUS_OK) {
        init_status = lsm_status;
    } else {
        init_status = update_status;
    }
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
    (void)timer_init();
    return imu_init_internal(0U);
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

bsp_status_t imu_get_lsm303_data(lsm303_data_t *data)
{
    if (data == NULL) {
        return BSP_STATUS_INVALID_ARG;
    }
    if (imu_initialized == 0U) {
        return BSP_STATUS_NOT_READY;
    }

    imu_lock_lsm();
    *data = lsm_data;
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
#if defined(IMU_MANAGER_USE_FREERTOS)
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(IMU_TASK_PERIOD_MS);
    uint32_t last_recovery_ms = timer_get_ms();
    for (;;) {
        const bsp_status_t status = imu_task_step();
        const uint32_t now_ms = timer_get_ms();
        if (status != BSP_STATUS_OK &&
            (uint32_t)(now_ms - last_recovery_ms) >= IMU_RECOVERY_PERIOD_MS) {
            last_recovery_ms = now_ms;
            (void)imu_recover();
        }
        vTaskDelayUntil(&last_wake, period);
    }
#else
    for (;;) {
        (void)imu_task_step();
        imu_delay_ms(IMU_TASK_PERIOD_MS);
    }
#endif
}

uint8_t imu_is_ready(void)
{
    lsm303_data_t lsm_snapshot;

    if (imu_get_lsm303_data(&lsm_snapshot) != BSP_STATUS_OK) {
        return 0U;
    }
    return lsm_snapshot.online != 0U ? 1U : 0U;
}
