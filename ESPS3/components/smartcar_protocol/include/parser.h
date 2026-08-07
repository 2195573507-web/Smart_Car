#ifndef SMARTCAR_PARSER_H
#define SMARTCAR_PARSER_H

#include "frame.h"

typedef void (*sc_frame_parser_callback_t)(const sc_frame_view_t *frame, void *context);
typedef void (*sc_frame_parser_error_callback_t)(int error, const uint8_t *data,
                                                 size_t length, void *context);

typedef struct {
    uint8_t bytes[SC_FRAME_MAX_SIZE];
    uint16_t length;
    uint16_t expected_length;
    sc_frame_parser_callback_t callback;
    sc_frame_parser_error_callback_t error_callback;
    void *context;
} sc_frame_parser_t;

void sc_frame_parser_init(sc_frame_parser_t *parser,
                          sc_frame_parser_callback_t callback,
                          sc_frame_parser_error_callback_t error_callback,
                          void *context);
size_t sc_frame_parser_feed(sc_frame_parser_t *parser, const uint8_t *data,
                            size_t length);

#endif /* SMARTCAR_PARSER_H */
