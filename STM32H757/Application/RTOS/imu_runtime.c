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

#define IMU_RUNTIME_LOG_TIMEOUT_MS UINT32_C(100)
#define IMU_DATA_PERIOD_MS         UINT32_C(100)
#define IMU_STATUS_PERIOD_MS        UINT32_C(5000)
#define IMU_TELEMETRY_TICK_MS       UINT32_C(10)
#define IMU_TELEMETRY_IMU_PERIOD_MS UINT32_C(100)
#define IMU_TELEMETRY_ATTITUDE_PERIOD_MS UINT32_C(50)
#define IMU_DUAL_AHRS_LOG_PERIOD_MS  UINT32_C(1000)
#define IMU_STACK_MONITOR_PERIOD_MS UINT32_C(5000)
#define IMU_DATA_STACK_WORDS       UINT16_C(1024)
#define IMU_DATA_PRIORITY          (tskIDLE_PRIORITY + 1U)
#define IMU_SAMPLE_PRIORITY        (tskIDLE_PRIORITY + 2U)
#define IMU_RUNTIME_PI             3.14159265358979323846f
#define RAD_TO_DEG                 57.295779513f
#ifndef SMARTCAR_BMI323_DEBUG_ONLY
#define SMARTCAR_BMI323_DEBUG_ONLY 0
#endif
#ifndef IMU_RUNTIME_ENABLE_RAW_DATA_LOG
#define IMU_RUNTIME_ENABLE_RAW_DATA_LOG 0U
#endif

static volatile uint32_t imu_runtime_log_fail_count;
static TaskHandle_t s_imu_task_handle;
static TaskHandle_t s_imu_debug_task_handle;
static uint8_t s_dual_attitude_payload[DUAL_AHRS_PAYLOAD_LENGTH];

static void put_float_le(uint8_t *destination, float value)
{
    uint32_t bits = 0U;

    (void)memcpy(&bits, &value, sizeof(bits));
    destination[0] = (uint8_t)(bits & 0xFFU);
    destination[1] = (uint8_t)((bits >> 8U) & 0xFFU);
    destination[2] = (uint8_t)((bits >> 16U) & 0xFFU);
    destination[3] = (uint8_t)(bits >> 24U);
}

static void put_u32_le(uint8_t *destination, uint32_t value)
{
    destination[0] = (uint8_t)(value & 0xFFU);
    destination[1] = (uint8_t)((value >> 8U) & 0xFFU);
    destination[2] = (uint8_t)((value >> 16U) & 0xFFU);
    destination[3] = (uint8_t)(value >> 24U);
}

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
    /* Raw/text diagnostics stay disabled; binary telemetry uses the SC frame. */
    (void)text;
}

static UBaseType_t imu_runtime_stack_high_water(TaskHandle_t task)
{
    return task == NULL ? 0U : uxTaskGetStackHighWaterMark(task);
}

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
