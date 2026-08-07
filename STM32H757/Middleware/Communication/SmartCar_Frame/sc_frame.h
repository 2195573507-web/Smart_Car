#ifndef SC_FRAME_H
#define SC_FRAME_H

#include <stddef.h>
#include <stdint.h>

#define SC_FRAME_HEADER_0 0xAAU
#define SC_FRAME_HEADER_1 0x55U
#define SC_FRAME_VERSION 0x01U
#define SC_FRAME_MAX_PAYLOAD UINT16_C(128)
#define SC_FRAME_OVERHEAD UINT16_C(8)
#define SC_FRAME_MAX_SIZE (SC_FRAME_OVERHEAD + SC_FRAME_MAX_PAYLOAD)

/* Parser diagnostics; values are kept compatible with the existing decode API. */
#define SC_FRAME_ERROR_AA55_FAIL (-2)
#define SC_FRAME_ERROR_VERSION_FAIL (-3)
#define SC_FRAME_ERROR_LEN_FAIL (-4)
#define SC_FRAME_ERROR_CRC_FAIL (-5)

/* AA 55 | VERSION | TYPE | LENGTH LE16 | PAYLOAD | CRC16-MODBUS LE. */
/* CRC covers VERSION through PAYLOAD; there is no control-link log envelope. */
/* CAL_EVENT (0x18): payload event_id. CAL_EVENT_ACK (0x19): event_id/result. */

typedef enum {
    SC_TYPE_PING = 0x01,
    SC_TYPE_PONG = 0x02,
    SC_TYPE_ACK = 0x03,
    SC_TYPE_PWM_READY = 0x10,
    SC_TYPE_RADAR_PWM_READY = 0x16,
    SC_TYPE_RADAR_PWM_ACK = 0x17,
    SC_TYPE_CAL_EVENT = 0x18,
    SC_TYPE_CAL_EVENT_ACK = 0x19,
    SC_TYPE_STM_BOOT_READY = 0x1C,
    SC_TYPE_IMU_STATUS = 0x20,
    SC_TYPE_ATTITUDE = 0x21,
    SC_TYPE_LOG = 0x30,
} sc_frame_type_t;

/* SC_TYPE_CAL_EVENT payload event IDs. */
#define SC_CAL_EVENT_STATIC_CAL_DONE UINT8_C(0x01)
#define SC_CAL_EVENT_VIBRATION_STEP_DONE UINT8_C(0x02)
#define SC_CAL_EVENT_COMPLETE UINT8_C(0x03)

/* SC_TYPE_CAL_EVENT_ACK result values. */
#define CAL_ACK_OK UINT8_C(0x00)
#define CAL_ACK_ERROR UINT8_C(0x01)

typedef struct {
    uint8_t version;
    uint8_t type;
    uint16_t length;
    const uint8_t *payload;
} sc_frame_view_t;

typedef void (*sc_frame_callback_t)(const sc_frame_view_t *frame, void *context);
typedef void (*sc_frame_error_callback_t)(int error, const uint8_t *data,
                                          size_t length, void *context);

typedef struct {
    uint8_t bytes[SC_FRAME_MAX_SIZE];
    uint16_t length;
    uint16_t expected_length;
    uint32_t frame_index;
    sc_frame_callback_t callback;
    sc_frame_error_callback_t error_callback;
    void *context;
} sc_frame_parser_t;

uint16_t sc_frame_crc16(const uint8_t *data, size_t length);
int sc_frame_encode(uint8_t type, const uint8_t *payload, uint16_t length,
                    uint8_t *out, size_t capacity, uint16_t *out_length);
int sc_frame_decode(const uint8_t *frame, size_t length, sc_frame_view_t *view);
void sc_frame_parser_init(sc_frame_parser_t *parser, sc_frame_callback_t callback,
                          sc_frame_error_callback_t error_callback, void *context);
size_t sc_frame_parser_feed(sc_frame_parser_t *parser, const uint8_t *data,
                            size_t length);

#endif /* SC_FRAME_H */
