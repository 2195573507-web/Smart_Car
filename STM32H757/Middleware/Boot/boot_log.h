#ifndef BOOT_LOG_H
#define BOOT_LOG_H

#ifdef __cplusplus
extern "C" {
#endif

/* Start the elapsed-time origin before the generated peripheral sequence. */
void boot_log_start(void);

/* Mark the existing USART1 logger as ready and flush queued early events. */
void boot_log_uart_ready(void);

/* Emit one normalized startup event. */
void boot_log(const char *module, const char *status);

#ifdef __cplusplus
}
#endif

#endif /* BOOT_LOG_H */
