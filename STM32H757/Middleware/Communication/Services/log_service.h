#ifndef LOG_SERVICE_H
#define LOG_SERVICE_H

#include <stdint.h>

#include "sc_frame.h"

#define SMARTCAR_LOG_MAX_PAYLOAD 96U
#define LOG_SERVICE_TEXT_MAX SMARTCAR_LOG_MAX_PAYLOAD

/* SC_TYPE_LOG payload: source | level | timestamp LE32 | text length LE16 | UTF-8 text. */
#define LOG_SERVICE_PAYLOAD_HEADER_SIZE UINT16_C(8)

typedef enum {
    SMARTCAR_LOG_LEVEL_DEBUG = 0U,
    SMARTCAR_LOG_LEVEL_INFO = 1U,
    SMARTCAR_LOG_LEVEL_WARN = 2U,
    SMARTCAR_LOG_LEVEL_ERROR = 3U
} smartcar_log_level_t;

void log_service_write(smartcar_log_level_t level, const char *text);
void log_service_init(void);
void log_service_start(void);
uint32_t log_service_get_drop_count(void);
void log_service_handle(const sc_frame_view_t *frame);

#define LOG_DEBUG(text) log_service_write(SMARTCAR_LOG_LEVEL_DEBUG, (text))
#define LOG_INFO(text) log_service_write(SMARTCAR_LOG_LEVEL_INFO, (text))
#define LOG_WARN(text) log_service_write(SMARTCAR_LOG_LEVEL_WARN, (text))
#define LOG_ERROR(text) log_service_write(SMARTCAR_LOG_LEVEL_ERROR, (text))

#endif /* LOG_SERVICE_H */
