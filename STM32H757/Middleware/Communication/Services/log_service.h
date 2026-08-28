#ifndef LOG_SERVICE_H
#define LOG_SERVICE_H

#include <stdint.h>

#define SMARTCAR_LOG_MAX_PAYLOAD 96U
#define LOG_SERVICE_TEXT_MAX SMARTCAR_LOG_MAX_PAYLOAD

/* CM7 debug builds keep all diagnostic levels available to the logger task. */
#define LOG_LEVEL_DEBUG 0U
#define LOG_LEVEL_INFO  1U
#define LOG_LEVEL_WARN  2U
#define LOG_LEVEL_ERROR 3U

#ifndef LOG_ENABLE
#define LOG_ENABLE 1
#endif

#ifndef LOG_LEVEL
#define LOG_LEVEL LOG_LEVEL_DEBUG
#endif

/* LOG payload: source | level | timestamp LE32 | text length LE16 | UTF-8 text. */
#define LOG_SERVICE_PAYLOAD_HEADER_SIZE UINT16_C(8)

typedef enum {
    SMARTCAR_LOG_LEVEL_DEBUG = LOG_LEVEL_DEBUG,
    SMARTCAR_LOG_LEVEL_INFO = LOG_LEVEL_INFO,
    SMARTCAR_LOG_LEVEL_WARN = LOG_LEVEL_WARN,
    SMARTCAR_LOG_LEVEL_ERROR = LOG_LEVEL_ERROR
} smartcar_log_level_t;

void log_service_write(smartcar_log_level_t level, const char *text);
void log_service_init(void);
void log_service_start(void);
uint32_t log_service_get_drop_count(void);

#if (LOG_ENABLE != 0) && (LOG_LEVEL <= LOG_LEVEL_DEBUG)
#define LOG_DEBUG(text) log_service_write(SMARTCAR_LOG_LEVEL_DEBUG, (text))
#else
#define LOG_DEBUG(text) ((void)0)
#endif

#if (LOG_ENABLE != 0) && (LOG_LEVEL <= LOG_LEVEL_INFO)
#define LOG_INFO(text) log_service_write(SMARTCAR_LOG_LEVEL_INFO, (text))
#else
#define LOG_INFO(text) ((void)0)
#endif

#if (LOG_ENABLE != 0) && (LOG_LEVEL <= LOG_LEVEL_WARN)
#define LOG_WARN(text) log_service_write(SMARTCAR_LOG_LEVEL_WARN, (text))
#else
#define LOG_WARN(text) ((void)0)
#endif

#if (LOG_ENABLE != 0) && (LOG_LEVEL <= LOG_LEVEL_ERROR)
#define LOG_ERROR(text) log_service_write(SMARTCAR_LOG_LEVEL_ERROR, (text))
#else
#define LOG_ERROR(text) ((void)0)
#endif

#endif /* LOG_SERVICE_H */
