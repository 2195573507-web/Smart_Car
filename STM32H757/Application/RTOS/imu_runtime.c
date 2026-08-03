#include "imu_runtime.h"

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"

#include "bsp_uart.h"
#include "boot_log.h"
#include "imu_calibration.h"
#include "imu_filter.h"
#include "imu_manager.h"

#define IMU_RUNTIME_LOG_TIMEOUT_MS UINT32_C(100)
#define IMU_DATA_PERIOD_MS         UINT32_C(100)
#define IMU_DATA_STACK_WORDS       UINT16_C(512)
#define IMU_DATA_PRIORITY          (tskIDLE_PRIORITY + 1U)
#define IMU_SAMPLE_PRIORITY        (tskIDLE_PRIORITY + 2U)

static volatile uint32_t imu_runtime_log_fail_count;

static void imu_runtime_log(const char *text)
{
    bsp_status_t status;

    if (text != NULL) {
        status = uart_log_write(text, IMU_RUNTIME_LOG_TIMEOUT_MS);
        if (status != BSP_STATUS_OK) {
            ++imu_runtime_log_fail_count;
        }
    }
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

static size_t imu_append_milli(char *buffer, size_t capacity, size_t offset,
                               const char *label, float value)
{
    int32_t milli = (int32_t)(value * 1000.0f);
    int32_t absolute = milli < 0 ? -milli : milli;
    int written;

    if (buffer == NULL || label == NULL || offset >= capacity) {
        return offset;
    }
    written = snprintf(buffer + offset, capacity - offset,
                       "%s=%s%ld.%03ld", label,
                       milli < 0 ? "-" : "",
                       (long)(absolute / 1000), (long)(absolute % 1000));
    if (written <= 0) {
        return offset;
    }
    if ((size_t)written >= capacity - offset) {
        return capacity - 1U;
    }
    return offset + (size_t)written;
}

static void imu_data_print(void)
{
    char block[768];
    lsm303_data_t lsm = {0};
    imu_calibrated_data_t calibrated = {0};
    imu_filtered_data_t filtered = {0};
    size_t offset = 0U;

    (void)imu_get_lsm303_data(&lsm);
    calibrated = imu_calibration_get_data();
    filtered = imu_filter_get_output();

    offset = imu_append_text(block, sizeof(block), offset,
                             "[DATA][IMU_RAW] ");
    offset = imu_append_milli(block, sizeof(block), offset, "ax", lsm.ax);
    offset = imu_append_text(block, sizeof(block), offset, " ");
    offset = imu_append_milli(block, sizeof(block), offset, "ay", lsm.ay);
    offset = imu_append_text(block, sizeof(block), offset, " ");
    offset = imu_append_milli(block, sizeof(block), offset, "az", lsm.az);
    offset = imu_append_text(block, sizeof(block), offset, " ");
    offset = imu_append_milli(block, sizeof(block), offset, "mx", lsm.mx);
    offset = imu_append_text(block, sizeof(block), offset, " ");
    offset = imu_append_milli(block, sizeof(block), offset, "my", lsm.my);
    offset = imu_append_text(block, sizeof(block), offset, " ");
    offset = imu_append_milli(block, sizeof(block), offset, "mz", lsm.mz);
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

void imu_runtime_start(void)
{
    char init_status_line[48];
    bsp_status_t init_status;
    BaseType_t task_status;

    imu_runtime_log("[INFO] IMU_INIT_BEGIN\r\n");
    init_status = imu_init();
    (void)snprintf(init_status_line, sizeof(init_status_line),
                   "[INFO] IMU_INIT_DONE status=%d\r\n", (int)init_status);
    imu_runtime_log(init_status_line);

    task_status = xTaskCreate(imu_task, "imu_task", IMU_DATA_STACK_WORDS,
                              NULL, IMU_SAMPLE_PRIORITY, NULL);
    if (task_status != pdPASS) {
        boot_log("TASK", "IMU_TASK CREATE FAIL");
        imu_runtime_log("[INFO] IMU TASK CREATE FAIL status=sample\r\n");
    } else {
        boot_log("TASK", "IMU_TASK CREATE OK");
        imu_runtime_log("[INFO] TASK_CREATE_OK task=imu_task\r\n");
    }
    task_status = xTaskCreate(imu_debug_task, "imu_data_logger",
                              IMU_DATA_STACK_WORDS, NULL,
                              IMU_DATA_PRIORITY, NULL);
    if (task_status != pdPASS) {
        boot_log("TASK", "DEBUG_TASK CREATE FAIL");
        imu_runtime_log("[INFO] IMU TASK CREATE FAIL status=data_logger\r\n");
    } else {
        boot_log("TASK", "DEBUG_TASK CREATE OK");
        imu_runtime_log("[INFO] TASK_CREATE_OK task=imu_data_logger\r\n");
    }
}

void imu_debug_task(void *argument)
{
    TickType_t last_wake;

    (void)argument;
    imu_runtime_log("[INFO] SCHEDULER_RUNNING\r\n");
    last_wake = xTaskGetTickCount();
    for (;;) {
        imu_data_print();
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(IMU_DATA_PERIOD_MS));
    }
}
