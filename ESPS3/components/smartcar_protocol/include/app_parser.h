#ifndef SMARTCAR_APP_PARSER_H
#define SMARTCAR_APP_PARSER_H

#include <stddef.h>
#include <stdint.h>

#define SC_APP_FRAME_HEAD 0xAAU
#define SC_APP_FRAME_TAIL 0x55U
#define SC_APP_FRAME_VERSION 0x01U
#define SC_APP_FRAME_MAX_PAYLOAD 128U
#define SC_APP_FRAME_OVERHEAD 8U
#define SC_APP_FRAME_MAX_SIZE \
    (SC_APP_FRAME_OVERHEAD + SC_APP_FRAME_MAX_PAYLOAD)

#define SC_APP_TYPE_ACK 0x06U
#define SC_APP_TYPE_ATTITUDE 0x11U
#define SC_APP_TYPE_RADAR_SET_SPEED 0x14U
#define SC_APP_ACK_OK 0x00U
#define SC_APP_ACK_REJECTED 0x01U

typedef struct {
    uint8_t version;
    uint8_t type;
    uint16_t length;
    const uint8_t *payload;
} sc_app_frame_view_t;

typedef void (*sc_app_parser_callback_t)(const sc_app_frame_view_t *frame,
                                         void *context);
typedef void (*sc_app_parser_error_callback_t)(int error,
                                               const uint8_t *data,
                                               size_t length,
                                               void *context);

typedef struct {
    uint8_t bytes[SC_APP_FRAME_MAX_SIZE];
    uint16_t length;
    uint16_t expected_length;
    sc_app_parser_callback_t callback;
    sc_app_parser_error_callback_t error_callback;
    void *context;
} sc_app_parser_t;

void sc_app_parser_init(sc_app_parser_t *parser,
                        sc_app_parser_callback_t callback,
                        sc_app_parser_error_callback_t error_callback,
                        void *context);

size_t sc_app_parser_feed(sc_app_parser_t *parser, const uint8_t *data,
                          size_t length);

int sc_app_frame_encode(uint8_t type, const uint8_t *payload, uint16_t length,
                        uint8_t *out, size_t capacity, uint16_t *out_length);

#endif /* SMARTCAR_APP_PARSER_H */
