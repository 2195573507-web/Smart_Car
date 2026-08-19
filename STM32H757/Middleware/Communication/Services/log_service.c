#include "log_service.h"

#include <string.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include "bsp_uart.h"
#include "rtos_health.h"

#define LOG_SERVICE_QUEUE_DEPTH UINT32_C(24)
#define LOG_SERVICE_TASK_STACK_WORDS UINT32_C(384)
#define LOG_SERVICE_TASK_PRIORITY (tskIDLE_PRIORITY + 1U)
#define LOG_SERVICE_HEALTH_PERIOD_MS UINT32_C(5000)

#ifndef SMARTCAR_BMI323_DEBUG_ONLY
#define SMARTCAR_BMI323_DEBUG_ONLY 0
#endif

typedef struct {
    uint8_t level;
    char text[SMARTCAR_LOG_MAX_PAYLOAD + 1U];
} log_item_t;

static QueueHandle_t s_log_queue;
static TaskHandle_t s_log_task;
static volatile uint32_t s_log_drop_count;

static void log_copy_text(char *destination, size_t capacity, const char *source)
{
    size_t length;

    if (destination == NULL || capacity == 0U) {
        return;
    }
    if (source == NULL) {
        destination[0] = '\0';
        return;
    }
    length = strlen(source);
    if (length >= capacity) {
        length = capacity - 1U;
    }
    memcpy(destination, source, length);
    destination[length] = '\0';
}

static void log_service_task(void *argument)
{
    log_item_t item;
    TickType_t last_health_tick = xTaskGetTickCount();

    (void)argument;
    for (;;) {
        if (xQueueReceive(s_log_queue, &item, pdMS_TO_TICKS(250U)) == pdTRUE) {
            (void)bsp_uart_log_write_link_level(item.level, item.text);
        }

        if ((xTaskGetTickCount() - last_health_tick) >=
            pdMS_TO_TICKS(LOG_SERVICE_HEALTH_PERIOD_MS)) {
#if !SMARTCAR_BMI323_DEBUG_ONLY
            rtos_health_snapshot_t snapshot;
            char line[SMARTCAR_LOG_MAX_PAYLOAD + 1U];

            last_health_tick = xTaskGetTickCount();
            rtos_health_sample();
            if (rtos_health_get_snapshot(&snapshot)) {
                (void)snprintf(line, sizeof(line),
                               "RTOS_HEALTH h=%lu m=%lu i=%lu u=%lu s=%lu",
                               (unsigned long)snapshot.current_free_heap_bytes,
                               (unsigned long)snapshot.minimum_free_heap_bytes,
                               (unsigned long)snapshot.stacks[0].current_free_stack_words,
                               (unsigned long)snapshot.stacks[1].current_free_stack_words,
                               (unsigned long)snapshot.stacks[2].current_free_stack_words);
                (void)bsp_uart_log_write_link_level(
                    SMARTCAR_LOG_LEVEL_INFO, line);
                (void)snprintf(line, sizeof(line),
                               "RTOS_HEALTH l=%lu p=%lu d=%lu o=%lu f=%lu",
                               (unsigned long)snapshot.stacks[3].current_free_stack_words,
                               (unsigned long)snapshot.stacks[4].current_free_stack_words,
                               (unsigned long)snapshot.stacks[5].current_free_stack_words,
                               (unsigned long)snapshot.stack_overflow_count,
                               (unsigned long)snapshot.malloc_failed_count);
                (void)bsp_uart_log_write_link_level(
                    SMARTCAR_LOG_LEVEL_INFO, line);
            }
            (void)snprintf(line, sizeof(line), "LOG_STATS drop=%lu",
                           (unsigned long)s_log_drop_count);
            (void)bsp_uart_log_write_link_level(SMARTCAR_LOG_LEVEL_INFO, line);
#else
            last_health_tick = xTaskGetTickCount();
#endif
        }
    }
}

void log_service_write(smartcar_log_level_t level, const char *text)
{
    log_item_t item;

    if (text == NULL || level > SMARTCAR_LOG_LEVEL_ERROR ||
        s_log_queue == NULL) {
        ++s_log_drop_count;
        return;
    }
    item.level = (uint8_t)level;
    log_copy_text(item.text, sizeof(item.text), text);
    if (xQueueSend(s_log_queue, &item, 0U) != pdTRUE) {
        ++s_log_drop_count;
    }
}

void log_service_init(void)
{
    s_log_drop_count = 0U;
    s_log_task = NULL;
    s_log_queue = xQueueCreate(LOG_SERVICE_QUEUE_DEPTH, sizeof(log_item_t));
}

void log_service_start(void)
{
    if (s_log_queue == NULL || s_log_task != NULL) {
        return;
    }
    (void)xTaskCreate(log_service_task, "logger", LOG_SERVICE_TASK_STACK_WORDS,
                      NULL, LOG_SERVICE_TASK_PRIORITY, &s_log_task);
    if (s_log_task == NULL) {
        ++s_log_drop_count;
    }
}

uint32_t log_service_get_drop_count(void)
{
    return s_log_drop_count;
}

void log_service_handle(const sc_frame_view_t *frame)
{
    (void)frame;
}
