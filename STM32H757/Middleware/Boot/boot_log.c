#include "boot_log.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "bsp_uart.h"
#include "main.h"

#define BOOT_LOG_QUEUE_DEPTH    16U
#define BOOT_LOG_MODULE_LENGTH  16U
#define BOOT_LOG_STATUS_LENGTH  64U
#define BOOT_LOG_LINE_LENGTH    128U
#define BOOT_LOG_TIMEOUT_MS     100U

typedef struct
{
    char module[BOOT_LOG_MODULE_LENGTH];
    char status[BOOT_LOG_STATUS_LENGTH];
    uint32_t elapsed_ms;
} boot_log_pending_t;

static boot_log_pending_t pending_logs[BOOT_LOG_QUEUE_DEPTH];
static uint32_t pending_count;
static uint32_t boot_start_ms;
static uint8_t boot_started;
static uint8_t uart_ready;

static void boot_log_copy(char *destination, size_t capacity, const char *source)
{
    if (destination == NULL || capacity == 0U) {
        return;
    }
    if (source == NULL) {
        destination[0] = '\0';
        return;
    }
    (void)snprintf(destination, capacity, "%s", source);
}

static void boot_log_emit(const char *module, const char *status,
                          uint32_t elapsed_ms)
{
    char line[BOOT_LOG_LINE_LENGTH];

    if (module == NULL || status == NULL) {
        return;
    }
    if (strcmp(module, "SYSTEM") == 0 && strcmp(status, "READY") == 0) {
        (void)snprintf(line, sizeof(line), "[BOOT][SYSTEM] READY total=%lums\r\n",
                       (unsigned long)elapsed_ms);
    } else {
        (void)snprintf(line, sizeof(line), "[BOOT][%s] %s t=%lums\r\n",
                       module, status, (unsigned long)elapsed_ms);
    }
    (void)uart_log_write(line, BOOT_LOG_TIMEOUT_MS);
}

void boot_log_start(void)
{
    boot_start_ms = HAL_GetTick();
    pending_count = 0U;
    boot_started = 1U;
    uart_ready = 0U;
}

void boot_log_uart_ready(void)
{
    uint32_t index;

    uart_ready = 1U;
    for (index = 0U; index < pending_count; ++index) {
        boot_log_emit(pending_logs[index].module,
                      pending_logs[index].status,
                      pending_logs[index].elapsed_ms);
    }
    pending_count = 0U;
}

void boot_log(const char *module, const char *status)
{
    uint32_t elapsed_ms;

    if (module == NULL || status == NULL) {
        return;
    }
    if (boot_started == 0U) {
        boot_log_start();
    }
    elapsed_ms = HAL_GetTick() - boot_start_ms;
    if (uart_ready != 0U) {
        boot_log_emit(module, status, elapsed_ms);
        return;
    }
    if (pending_count >= BOOT_LOG_QUEUE_DEPTH) {
        return;
    }
    boot_log_copy(pending_logs[pending_count].module,
                  sizeof(pending_logs[pending_count].module), module);
    boot_log_copy(pending_logs[pending_count].status,
                  sizeof(pending_logs[pending_count].status), status);
    pending_logs[pending_count].elapsed_ms = elapsed_ms;
    ++pending_count;
}
