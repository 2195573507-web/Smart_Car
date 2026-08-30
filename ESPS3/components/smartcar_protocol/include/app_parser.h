#ifndef SMARTCAR_APP_PARSER_H
#define SMARTCAR_APP_PARSER_H

#include <stddef.h>
#include <stdint.h>

#define SC_APP_FRAME_HEAD 0xAAU
#define SC_APP_FRAME_TAIL 0x55U
#define SC_APP_FRAME_VERSION 0x01U
#define SC_APP_FRAME_VERSION_V1 SC_APP_FRAME_VERSION
#define SC_APP_FRAME_VERSION_V2 0x02U
#define SC_APP_FRAME_MAX_PAYLOAD 128U
#define SC_APP_FRAME_OVERHEAD 8U
#define SC_APP_FRAME_MAX_SIZE \
    (SC_APP_FRAME_OVERHEAD + SC_APP_FRAME_MAX_PAYLOAD)

#define SC_APP_TYPE_ACK 0x06U
#define SC_APP_TYPE_STATUS 0x02U
#define SC_APP_TYPE_ATTITUDE 0x11U
#define SC_APP_TYPE_WHEEL_SPEED_CMD 0x15U
#define SC_APP_TYPE_WHEEL_SPEED_STATUS 0x16U
#define SC_APP_TYPE_RADAR_STATUS 0x1AU
#define SC_APP_TYPE_RADAR_SET_SPEED 0x1BU
#define SC_APP_TYPE_POWER_STATUS 0x1CU
#define SC_APP_TYPE_PID_PARAMS_CMD 0x1DU
#define SC_APP_TYPE_CHASSIS_STATE 0x29U
#define SC_APP_TYPE_WHEEL_SPEED_SINGLE_CMD 0x2AU
#define SC_APP_TYPE_MASTER_SPEED_CMD 0x2BU
#define SC_APP_TYPE_WHEEL_CONTROL_STATUS 0x2CU
#define SC_APP_TYPE_CHASSIS_SPEED_CMD 0x2DU
#define SC_APP_TYPE_CHASSIS_HEADING_CMD 0x2EU
#define SC_APP_TYPE_SYS_CONFIG 0x70U
#define SC_APP_ACK_OK 0x00U
#define SC_APP_ACK_REJECTED 0x01U

#define SC_APP_V2_TYPE_HELLO 0x70U
#define SC_APP_V2_TYPE_HELLO_ACK 0x71U
#define SC_APP_V2_TYPE_HEARTBEAT 0x72U
#define SC_APP_V2_TYPE_HEARTBEAT_ACK 0x73U
#define SC_APP_V2_TYPE_COMMAND_ACK 0x74U
#define SC_APP_V2_TYPE_COMMAND 0x75U

#define SC_APP_V2_RESULT_OK 0x00U
#define SC_APP_V2_RESULT_REJECTED 0x01U
#define SC_APP_V2_RESULT_SESSION_INVALID 0x02U
#define SC_APP_V2_RESULT_EXPIRED 0x03U
#define SC_APP_V2_RESULT_STALE_SEQUENCE 0x04U
#define SC_APP_V2_RESULT_BUSY 0x05U

#define SC_APP_V2_STAGE_GATEWAY_ADMITTED 0x00U
#define SC_APP_V2_STAGE_STM32_ACCEPTED 0x01U
#define SC_APP_V2_STAGE_STOP_QUEUED 0x02U

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

int sc_app_frame_encode_version(uint8_t version, uint8_t type,
                                const uint8_t *payload, uint16_t length,
                                uint8_t *out, size_t capacity,
                                uint16_t *out_length);

#endif /* SMARTCAR_APP_PARSER_H */
