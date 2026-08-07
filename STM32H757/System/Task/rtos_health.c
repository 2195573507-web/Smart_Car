#include "rtos_health.h"

#include <stddef.h>

#include "FreeRTOS.h"
#include "task.h"
#include "stm32h7xx.h"

#define RTOS_HEALTH_MAGIC UINT32_C(0x5254484C)
#define RTOS_HEALTH_VERSION UINT32_C(1)
#define RTOS_HEALTH_CHECKSUM_SEED UINT32_C(0xA16E4F39)

typedef struct {
    uint32_t magic;
    uint32_t version;
    uint32_t size;
    rtos_health_snapshot_t snapshot;
    uint32_t checksum;
} rtos_health_record_t;

static volatile rtos_health_record_t g_rtos_health_record
    __attribute__((section(".noinit"), used, aligned(8)));

_Static_assert(configMAX_TASK_NAME_LEN == RTOS_HEALTH_TASK_NAME_LENGTH,
               "RTOS health task-name storage must match FreeRTOS");

static const char *const s_monitored_task_names[RTOS_HEALTH_MONITORED_TASK_COUNT] = {
    "imu_task",
    "uart_link",
    "s3_service",
    "logger",
    "protocol",
    "imu_data_logger",
};

static uint32_t rtos_health_checksum(const volatile rtos_health_record_t *record)
{
    const volatile uint32_t *word = (const volatile uint32_t *)record;
    uint32_t checksum = RTOS_HEALTH_CHECKSUM_SEED;
    uint32_t index;

    for (index = 0U;
         index < ((sizeof(rtos_health_record_t) / sizeof(uint32_t)) - 2U);
         ++index) {
        checksum = (checksum << 5U) | (checksum >> 27U);
        checksum ^= word[index + 1U];
    }

    return checksum;
}

static bool rtos_health_record_is_valid(void)
{
    if (g_rtos_health_record.magic != RTOS_HEALTH_MAGIC) {
        return false;
    }
    if ((g_rtos_health_record.version != RTOS_HEALTH_VERSION) ||
        (g_rtos_health_record.size != sizeof(g_rtos_health_record))) {
        return false;
    }

    return g_rtos_health_record.checksum ==
           rtos_health_checksum(&g_rtos_health_record);
}

static void rtos_health_copy_task_name(volatile char *destination,
                                       const char *source)
{
    uint32_t index;

    if (source == NULL) {
        source = "UNKNOWN";
    }

    for (index = 0U; index < (RTOS_HEALTH_TASK_NAME_LENGTH - 1U); ++index) {
        const char character = source[index];

        destination[index] = character;
        if (character == '\0') {
            break;
        }
    }
    for (; index < RTOS_HEALTH_TASK_NAME_LENGTH; ++index) {
        destination[index] = '\0';
    }
}

static void rtos_health_clear_record(void)
{
    volatile uint32_t *word = (volatile uint32_t *)&g_rtos_health_record;
    uint32_t index;

    g_rtos_health_record.magic = 0U;
    __DMB();
    for (index = 1U;
         index < (sizeof(g_rtos_health_record) / sizeof(uint32_t));
         ++index) {
        word[index] = 0U;
    }
    g_rtos_health_record.version = RTOS_HEALTH_VERSION;
    g_rtos_health_record.size = sizeof(g_rtos_health_record);
    g_rtos_health_record.checksum = rtos_health_checksum(&g_rtos_health_record);
    __DMB();
    g_rtos_health_record.magic = RTOS_HEALTH_MAGIC;
}

static void rtos_health_ensure_initialized(void)
{
    if (!rtos_health_record_is_valid()) {
        rtos_health_clear_record();
    }
}

static void rtos_health_commit_event(rtos_health_event_t event,
                                     uintptr_t task_handle,
                                     const char *task_name)
{
    if (!rtos_health_record_is_valid()) {
        rtos_health_clear_record();
    }

    const uint32_t previous_overflow_count =
        g_rtos_health_record.snapshot.stack_overflow_count;
    const uint32_t previous_malloc_failure_count =
        g_rtos_health_record.snapshot.malloc_failed_count;

    g_rtos_health_record.magic = 0U;
    __DMB();
    g_rtos_health_record.version = RTOS_HEALTH_VERSION;
    g_rtos_health_record.size = sizeof(g_rtos_health_record);
    g_rtos_health_record.snapshot.last_event = event;
    g_rtos_health_record.snapshot.last_task_handle = task_handle;
    rtos_health_copy_task_name(g_rtos_health_record.snapshot.last_task_name,
                               task_name);

    if (event == RTOS_HEALTH_EVENT_STACK_OVERFLOW) {
        g_rtos_health_record.snapshot.event_flags |=
            RTOS_HEALTH_FLAG_STACK_OVERFLOW;
        g_rtos_health_record.snapshot.stack_overflow_count =
            previous_overflow_count + 1U;
    } else if (event == RTOS_HEALTH_EVENT_MALLOC_FAILED) {
        g_rtos_health_record.snapshot.event_flags |=
            RTOS_HEALTH_FLAG_MALLOC_FAILED;
        g_rtos_health_record.snapshot.malloc_failed_count =
            previous_malloc_failure_count + 1U;
    }

    g_rtos_health_record.checksum = rtos_health_checksum(&g_rtos_health_record);
    __DMB();
    g_rtos_health_record.magic = RTOS_HEALTH_MAGIC;
}

void rtos_health_sample(void)
{
    uint32_t index;

    if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED) {
        return;
    }

    rtos_health_ensure_initialized();
    g_rtos_health_record.magic = 0U;
    __DMB();
    g_rtos_health_record.snapshot.current_free_heap_bytes =
        (uint32_t)xPortGetFreeHeapSize();
    g_rtos_health_record.snapshot.minimum_free_heap_bytes =
        (uint32_t)xPortGetMinimumEverFreeHeapSize();

    for (index = 0U; index < RTOS_HEALTH_MONITORED_TASK_COUNT; ++index) {
        TaskHandle_t task = xTaskGetHandle(s_monitored_task_names[index]);
        volatile rtos_health_stack_watermark_t *watermark =
            &g_rtos_health_record.snapshot.stacks[index];

        rtos_health_copy_task_name(watermark->task_name,
                                   s_monitored_task_names[index]);
        if (task != NULL) {
            const uint32_t free_stack_words =
                (uint32_t)uxTaskGetStackHighWaterMark(task);

            watermark->current_free_stack_words = free_stack_words;
            if ((watermark->sample_count == 0U) ||
                (free_stack_words < watermark->minimum_free_stack_words)) {
                watermark->minimum_free_stack_words = free_stack_words;
            }
            ++watermark->sample_count;
        }
    }

    g_rtos_health_record.checksum = rtos_health_checksum(&g_rtos_health_record);
    __DMB();
    g_rtos_health_record.magic = RTOS_HEALTH_MAGIC;
}

bool rtos_health_get_snapshot(rtos_health_snapshot_t *snapshot)
{
    if (snapshot == NULL || !rtos_health_record_is_valid()) {
        return false;
    }

    *snapshot = g_rtos_health_record.snapshot;
    return true;
}

bool rtos_health_has_fatal_event(void)
{
    return rtos_health_record_is_valid() &&
           (g_rtos_health_record.snapshot.event_flags != 0U);
}

void rtos_health_clear(void)
{
    rtos_health_clear_record();
}

void rtos_health_record_stack_overflow(uintptr_t task_handle,
                                       const char *task_name)
{
    rtos_health_commit_event(RTOS_HEALTH_EVENT_STACK_OVERFLOW,
                             task_handle, task_name);
}

void rtos_health_record_malloc_failed(void)
{
    rtos_health_commit_event(RTOS_HEALTH_EVENT_MALLOC_FAILED, 0U, NULL);
}

void rtos_health_halt(void)
{
    __asm volatile(
        "cpsid i\n"
        "dsb sy\n"
        "1:\n"
        "wfi\n"
        "b 1b\n");
    __builtin_unreachable();
}
