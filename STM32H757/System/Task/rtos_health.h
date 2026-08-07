#ifndef RTOS_HEALTH_H
#define RTOS_HEALTH_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define RTOS_HEALTH_TASK_NAME_LENGTH UINT32_C(16)
#define RTOS_HEALTH_MONITORED_TASK_COUNT UINT32_C(6)
#define RTOS_HEALTH_FLAG_STACK_OVERFLOW UINT32_C(0x00000001)
#define RTOS_HEALTH_FLAG_MALLOC_FAILED UINT32_C(0x00000002)

typedef enum {
    RTOS_HEALTH_EVENT_NONE = 0U,
    RTOS_HEALTH_EVENT_STACK_OVERFLOW = 1U,
    RTOS_HEALTH_EVENT_MALLOC_FAILED = 2U
} rtos_health_event_t;

typedef struct {
    char task_name[RTOS_HEALTH_TASK_NAME_LENGTH];
    uint32_t current_free_stack_words;
    uint32_t minimum_free_stack_words;
    uint32_t sample_count;
} rtos_health_stack_watermark_t;

typedef struct {
    uint32_t event_flags;
    uint32_t stack_overflow_count;
    uint32_t malloc_failed_count;
    rtos_health_event_t last_event;
    uintptr_t last_task_handle;
    char last_task_name[RTOS_HEALTH_TASK_NAME_LENGTH];
    uint32_t current_free_heap_bytes;
    uint32_t minimum_free_heap_bytes;
    rtos_health_stack_watermark_t stacks[RTOS_HEALTH_MONITORED_TASK_COUNT];
} rtos_health_snapshot_t;

/* Call only from normal task context, never from an ISR or an RTOS hook. */
void rtos_health_sample(void);

/* Copies the retained diagnostic state for a logger or debugger. */
bool rtos_health_get_snapshot(rtos_health_snapshot_t *snapshot);

/* A retained fatal event remains available after a software reset. */
bool rtos_health_has_fatal_event(void);
void rtos_health_clear(void);

/* Hook-only entry points. They use no FreeRTOS, HAL, libc, or heap APIs. */
void rtos_health_record_stack_overflow(uintptr_t task_handle,
                                       const char *task_name);
void rtos_health_record_malloc_failed(void);
void rtos_health_halt(void) __attribute__((noreturn));

#ifdef __cplusplus
}
#endif

#endif /* RTOS_HEALTH_H */
