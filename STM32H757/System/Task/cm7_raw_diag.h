#ifndef CM7_RAW_DIAG_H
#define CM7_RAW_DIAG_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependency-free USART1 markers for the CM7 debug image. */
void cm7_raw_diag_marker(const char *marker);
void cm7_raw_diag_value(const char *label, uint32_t value);
void cm7_raw_diag_once(const char *marker);
void cm7_raw_diag_task_enter(const char *task_name);
void cm7_raw_diag_task_loop(const char *task_name);
void cm7_raw_diag_tx_phase(const char *phase, uint32_t value);
void cm7_rtos_assert_failed(const char *file, uint32_t line)
    __attribute__((noreturn));

/* Called by the startup assembly Default_Handler path. */
void cm7_raw_diag_default_handler(void) __attribute__((noreturn));

#ifdef __cplusplus
}
#endif

#endif /* CM7_RAW_DIAG_H */
