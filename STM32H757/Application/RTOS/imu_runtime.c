#include "imu_runtime.h"

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"

#include "attitude.h"
#include "bsp_timer.h"
#include "boot_log.h"
#include "imu_calibration.h"
#include "imu_boot_manager.h"
#include "imu_filter.h"
#include "imu_manager.h"
#include "mag_filter.h"
#include "log_service.h"

#define IMU_RUNTIME_LOG_TIMEOUT_MS UINT32_C(100)
#define IMU_DATA_PERIOD_MS         UINT32_C(100)
#define IMU_STATUS_PERIOD_MS        UINT32_C(5000)
#define IMU_STACK_MONITOR_PERIOD_MS UINT32_C(5000)
#define IMU_DATA_STACK_WORDS       UINT16_C(512)
#define IMU_DATA_PRIORITY          (tskIDLE_PRIORITY + 1U)
#define IMU_SAMPLE_PRIORITY        (tskIDLE_PRIORITY + 2U)
#define IMU_RUNTIME_PI             3.14159265358979323846f
#ifndef IMU_RUNTIME_ENABLE_RAW_DATA_LOG
#define IMU_RUNTIME_ENABLE_RAW_DATA_LOG 0U
#endif

static volatile uint32_t imu_runtime_log_fail_count;
static TaskHandle_t s_imu_task_handle;
static TaskHandle_t s_imu_debug_task_handle;

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

static void imu_runtime_log(const char *text)
{
    /* Raw/text diagnostics stay disabled; telemetry remains on 0x10/0x11/0x12. */
    (void)text;
}

static void imu_runtime_log_stack(const char *task_name, TaskHandle_t task)
{
    char line[80];
    const UBaseType_t free_stack = task == NULL
                                       ? 0U
                                       : uxTaskGetStackHighWaterMark(task);

    (void)snprintf(line, sizeof(line),
                   "[TASK_STACK]\r\ntask=%s\r\nfree_stack=%lu\r\n",
                   task_name == NULL ? "UNKNOWN" : task_name,
                   (unsigned long)free_stack);
    LOG_INFO(line);
}

uint32_t imu_runtime_get_log_fail_count(void)
{
    return imu_runtime_log_fail_count;
}

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

static void imu_format_rms(char *buffer, size_t capacity, float value)
{
    const uint32_t scaled = (uint32_t)((value * 10000.0f) + 0.5f);

    if (buffer == NULL || capacity == 0U) {
        return;
    }
    (void)snprintf(buffer, capacity, "%lu.%04lu",
                   (unsigned long)(scaled / 10000U),
                   (unsigned long)(scaled % 10000U));
}

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

static const char *imu_cal_state_name(imu_boot_state_t state)
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
    default:
        return "UNKNOWN";
    }
}

#if IMU_RUNTIME_ENABLE_RAW_DATA_LOG
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
    char rms_text[24];
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
        imu_format_rms(rms_text, sizeof(rms_text), boot_status.rms);
        (void)snprintf(log_line, sizeof(log_line),
                       "state=%s progress=%u radar_pwm=%u rms=%s sample=%lu/%lu",
                       imu_cal_state_name(cal_state),
                       (unsigned)boot_status.progress,
                       (unsigned)boot_status.radar_pwm,
                       rms_text,
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
    offset = imu_append_uint32(block, sizeof(block), offset, " radar_pwm",
                               boot_status.radar_pwm);
    {
        char status_rms[32];
        imu_format_rms(status_rms, sizeof(status_rms), boot_status.rms);
        offset = imu_append_text(block, sizeof(block), offset, " rms=");
        offset = imu_append_text(block, sizeof(block), offset, status_rms);
    }
    if (cal_state == STATIC_CAL_WAIT || cal_state == STATIC_CAL_SAMPLE ||
        cal_state == VIBRATION_SAMPLE) {
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
        cal_state == VIBRATION_SAMPLE || cal_state == IMU_READY) {
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

void imu_runtime_start(void)
{
    char init_status_line[48];
    bsp_status_t init_status;
    BaseType_t task_status;

    attitude_init();
    imu_runtime_log("[INFO] IMU_INIT_BEGIN\r\n");
    imu_runtime_log_event(SMARTCAR_LOG_LEVEL_INFO, "IMU INIT", "BEGIN");
    init_status = imu_init();
    (void)snprintf(init_status_line, sizeof(init_status_line),
                   "[INFO] IMU_INIT_DONE status=%d\r\n", (int)init_status);
    imu_runtime_log(init_status_line);
    (void)snprintf(init_status_line, sizeof(init_status_line),
                   "status=%d", (int)init_status);
    imu_runtime_log_event(init_status == BSP_STATUS_OK
                              ? SMARTCAR_LOG_LEVEL_INFO
                              : SMARTCAR_LOG_LEVEL_ERROR,
                          init_status == BSP_STATUS_OK ? "IMU INIT" : "ERROR",
                          init_status == BSP_STATUS_OK ? "COMPLETE" : init_status_line);

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

void imu_debug_task(void *argument)
{
    TickType_t last_wake;
    uint32_t last_status_ms;
    uint32_t last_stack_monitor_ms;

    (void)argument;
    imu_runtime_log("[INFO] SCHEDULER_RUNNING\r\n");
    last_wake = xTaskGetTickCount();
    last_status_ms = timer_get_ms();
    last_stack_monitor_ms = last_status_ms;
    for (;;) {
#if IMU_RUNTIME_ENABLE_RAW_DATA_LOG
        imu_data_print();
#endif
        if ((uint32_t)(timer_get_ms() - last_status_ms) >= IMU_STATUS_PERIOD_MS) {
            last_status_ms = timer_get_ms();
            imu_status_print();
        }
        if ((uint32_t)(timer_get_ms() - last_stack_monitor_ms) >=
            IMU_STACK_MONITOR_PERIOD_MS) {
            last_stack_monitor_ms = timer_get_ms();
            imu_runtime_log_stack("imu_task", s_imu_task_handle);
            imu_runtime_log_stack("imu_debug_task", s_imu_debug_task_handle);
        }
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(IMU_DATA_PERIOD_MS));
    }
}
