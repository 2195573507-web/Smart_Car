#ifndef SC_FRAME_H
#define SC_FRAME_H

#include <stddef.h>
#include <stdint.h>

/* SCBP-V3: AA 55 | VERSION | PRIORITY | SRC | DST | MSG_ID LE16 |
 *          SEQ | FLAGS | LEN LE16 | PAYLOAD | CRC16-MODBUS LE16 */
#define SC_FRAME_HEADER_0 UINT8_C(0xAA)
#define SC_FRAME_HEADER_1 UINT8_C(0x55)
#define SC_FRAME_VERSION UINT8_C(0x01)
#define SC_FRAME_MAX_PAYLOAD UINT16_C(128)
#define SC_FRAME_OVERHEAD UINT16_C(14)
#define SC_FRAME_MAX_SIZE (SC_FRAME_OVERHEAD + SC_FRAME_MAX_PAYLOAD)
#define SCBP_CRC_HEADER_SIZE UINT16_C(10)

#define SCBP_NODE_STM32H757 UINT8_C(0x01)
#define SCBP_NODE_ESP32_S3 UINT8_C(0x02)
#define SCBP_NODE_APP UINT8_C(0x03)
#define SCBP_NODE_ROS2 UINT8_C(0x04)
#define SCBP_NODE_C5 UINT8_C(0x10)
#define SCBP_NODE_BROADCAST UINT8_C(0xFF)

#ifndef SCBP_LOCAL_NODE_ID
#define SCBP_LOCAL_NODE_ID SCBP_NODE_STM32H757
#endif
#ifndef SCBP_DEFAULT_DESTINATION
#define SCBP_DEFAULT_DESTINATION SCBP_NODE_ESP32_S3
#endif

#define SCBP_PRIORITY_EMERGENCY UINT8_C(0x00)
#define SCBP_PRIORITY_REALTIME UINT8_C(0x01)
#define SCBP_PRIORITY_NORMAL UINT8_C(0x02)
#define SCBP_PRIORITY_DEBUG UINT8_C(0x03)

#define SCBP_FLAG_ACK_REQUIRED UINT8_C(0x01)
#define SCBP_FLAG_ACK_FRAME UINT8_C(0x02)
#define SCBP_FLAG_ERROR_FRAME UINT8_C(0x04)
#define SCBP_FLAG_RETRY UINT8_C(0x08)
#define SCBP_FLAG_STREAM_DATA UINT8_C(0x10)
#define SCBP_FLAG_CONFIG UINT8_C(0x20)
#define SCBP_FLAG_RESERVED_MASK UINT8_C(0xC0)

#define SCBP_MSG_ID_PING UINT16_C(0x0001)
#define SCBP_MSG_ID_PONG UINT16_C(0x0002)
#define SCBP_MSG_ID_VERSION UINT16_C(0x0003)
#define SCBP_MSG_ID_RESET UINT16_C(0x0004)
#define SCBP_MSG_ID_ACK UINT16_C(0x0005)
#define SCBP_MSG_ID_ERROR UINT16_C(0x0006)
#define SCBP_MSG_ID_BOOT_READY UINT16_C(0x0007)

#define SCBP_MSG_ID_MOTOR_CONTROL UINT16_C(0x0100)
#define SCBP_MSG_ID_PWM_SET UINT16_C(0x0101)
#define SCBP_MSG_ID_PARAM_SET UINT16_C(0x0102)

#define SCBP_MSG_ID_IMU_STATUS UINT16_C(0x0200)
#define SCBP_MSG_ID_ATTITUDE UINT16_C(0x0201)
#define SCBP_MSG_ID_IMU_CAL_STATUS UINT16_C(0x0202)
#define SCBP_MSG_ID_IMU_BIAS UINT16_C(0x0203)
#define SCBP_MSG_ID_VIBRATION_STATUS UINT16_C(0x0204)
#define SCBP_MSG_ID_IMU_CAL_RESULT UINT16_C(0x0205)
#define SCBP_MSG_ID_IMU_VIBRATION_PROFILE UINT16_C(0x0206)
#define SCBP_MSG_ID_IMU_TELEMETRY UINT16_C(0x0207)
#define SCBP_MSG_ID_DUAL_IMU_STATUS UINT16_C(0x0208)

#define SCBP_MSG_ID_RADAR_CONTROL UINT16_C(0x0300)
#define SCBP_MSG_ID_RADAR_STATUS UINT16_C(0x0301)
#define SCBP_MSG_ID_RADAR_PWM_READY UINT16_C(0x0302)

#define SCBP_MSG_ID_CAL_START UINT16_C(0x0400)
#define SCBP_MSG_ID_CAL_EVENT UINT16_C(0x0401)
#define SCBP_MSG_ID_LOG UINT16_C(0xF000)

#define SCBP_ACK_PAYLOAD_LENGTH UINT16_C(5)
#define SCBP_ERROR_PAYLOAD_LENGTH UINT16_C(5)

#define SCBP_ACK_RESULT_OK UINT8_C(0)
#define SCBP_ACK_RESULT_FAILED UINT8_C(1)

#define SCBP_ERROR_OK UINT8_C(0x00)
#define SCBP_ERROR_UNKNOWN_MSG UINT8_C(0x01)
#define SCBP_ERROR_INVALID_LENGTH UINT8_C(0x02)
#define SCBP_ERROR_CRC UINT8_C(0x03)
#define SCBP_ERROR_BUSY UINT8_C(0x04)
#define SCBP_ERROR_TIMEOUT UINT8_C(0x05)
#define SCBP_ERROR_NOT_READY UINT8_C(0x06)
#define SCBP_ERROR_SENSOR UINT8_C(0x07)
#define SCBP_ERROR_PARAM UINT8_C(0x08)

#define SCBP_SEQUENCE_FIRST UINT8_C(0)
#define SCBP_SEQUENCE_IN_ORDER UINT8_C(1)
#define SCBP_SEQUENCE_GAP UINT8_C(2)
#define SCBP_SEQUENCE_DUPLICATE UINT8_C(3)
#define SCBP_SEQUENCE_OUT_OF_ORDER UINT8_C(4)

#define SC_LEGACY_ATTITUDE_PAYLOAD_LENGTH UINT16_C(26)
#define SC_ATTITUDE_PAYLOAD_LENGTH UINT16_C(30)
#define SC_DUAL_ATTITUDE_PAYLOAD_LENGTH UINT16_C(80)
#define SC_DUAL_ATTITUDE_SCHEMA UINT8_C(2)

/* Parser diagnostics retain the legacy names so transport callers can keep
 * their existing error handling while SCBP-V3 adds field validation. */
#define SC_FRAME_ERROR_AA55_FAIL (-2)
#define SC_FRAME_ERROR_VERSION_FAIL (-3)
#define SC_FRAME_ERROR_LEN_FAIL (-4)
#define SC_FRAME_ERROR_CRC_FAIL (-5)
#define SC_FRAME_ERROR_PRIORITY_FAIL (-6)
#define SC_FRAME_ERROR_FLAGS_FAIL (-7)

/* Legacy values are protocol-adapter inputs only. They never appear in a
 * SCBP-V3 header and keep protected business callbacks source-compatible. */
typedef enum {
    SC_TYPE_PING = 0x01,
    SC_TYPE_PONG = 0x02,
    SC_TYPE_ACK = 0x03,
    SC_TYPE_PWM_READY = 0x10,
    SC_TYPE_IMU_CAL_BIAS = 0x13,
    SC_TYPE_RADAR_PWM_READY = 0x16,
    SC_TYPE_RADAR_PWM_ACK = 0x17,
    SC_TYPE_CAL_EVENT = 0x18,
    SC_TYPE_CAL_EVENT_ACK = 0x19,
    SC_TYPE_STM_BOOT_READY = 0x1C,
    SC_TYPE_IMU_STATUS = 0x20,
    SC_TYPE_ATTITUDE = 0x21,
    SC_TYPE_IMU_CAL_STATUS = 0x22,
    SC_TYPE_STM_IMU_CAL_STATUS = SC_TYPE_IMU_CAL_STATUS,
    SC_TYPE_RADAR_STATUS = 0x23,
    SC_TYPE_STM_RADAR_STATUS = SC_TYPE_RADAR_STATUS,
    SC_TYPE_RADAR_VIBRATION_STATUS = 0x24,
    SC_TYPE_STM_RADAR_VIBRATION_STATUS = SC_TYPE_RADAR_VIBRATION_STATUS,
    SC_TYPE_IMU_CAL_RESULT = 0x25,
    SC_TYPE_IMU_VIBRATION_PROFILE = 0x26,
    SC_TYPE_IMU_TELEMETRY = 0x27,
    SC_TYPE_DUAL_IMU_STATUS = 0x28,
    SC_TYPE_LOG = 0x30,
} sc_frame_type_t;

#define SC_DUAL_IMU_STATUS_PAYLOAD_LENGTH UINT16_C(16)

/* SC_TYPE_IMU_TELEMETRY payload byte 1 retains the original channel-valid
 * bits and adds an explicit device-online bit. */
#define SC_IMU_TELEMETRY_FLAG_ACCEL_VALID UINT8_C(0x01)
#define SC_IMU_TELEMETRY_FLAG_GYRO_OR_MAG_VALID UINT8_C(0x02)
#define SC_IMU_TELEMETRY_FLAG_ONLINE UINT8_C(0x04)

typedef enum
{
    IMU_SENSOR_LSM303 = 0x01,
    IMU_SENSOR_BMI323 = 0x02
} imu_sensor_type_t;

#define SC_IMU_CAL_FLAG_ACCEL UINT8_C(0x01)
#define SC_IMU_CAL_FLAG_GYRO UINT8_C(0x02)

/* Values carried by the legacy calibration payloads, not SCBP-V3 headers. */
#define SC_CAL_STAGE_WAIT_RADAR_READY UINT8_C(0)
#define SC_CAL_STAGE_STATIC_STABLE_WAIT UINT8_C(1)
#define SC_CAL_STAGE_STATIC_SAMPLE UINT8_C(2)
#define SC_CAL_STAGE_VIBRATION_STABLE_WAIT UINT8_C(3)
#define SC_CAL_STAGE_VIBRATION_SAMPLE UINT8_C(4)
#define SC_CAL_STAGE_COMPLETE UINT8_C(5)
#define SC_CAL_STAGE_ERROR UINT8_C(6)

#define SC_CAL_EVENT_STATIC_CAL_DONE UINT8_C(0x01)
#define SC_CAL_EVENT_VIBRATION_STEP_DONE UINT8_C(0x02)
#define SC_CAL_EVENT_COMPLETE UINT8_C(0x03)
#define CAL_ACK_OK UINT8_C(0x00)
#define CAL_ACK_ERROR UINT8_C(0x01)

typedef struct {
    uint8_t version;
    uint8_t priority;
    uint8_t src;
    uint8_t dst;
    uint16_t msg_id;
    uint8_t seq;
    uint8_t flags;
    uint16_t length;
    const uint8_t *payload;
    uint16_t crc;
    uint8_t sequence_status;
} scbp_frame_t;

typedef scbp_frame_t sc_frame_view_t;
typedef void (*sc_frame_callback_t)(const sc_frame_view_t *frame, void *context);
typedef void (*sc_frame_error_callback_t)(int error, const uint8_t *data,
                                          size_t length, void *context);

typedef struct {
    uint8_t bytes[SC_FRAME_MAX_SIZE];
    uint16_t length;
    uint16_t expected_length;
    uint32_t frame_index;
    uint8_t sequence_seen[32];
    uint8_t sequence_last[256];
    sc_frame_callback_t callback;
    sc_frame_error_callback_t error_callback;
    void *context;
} sc_frame_parser_t;

uint16_t scbp_crc16(const uint8_t *data, size_t length);
uint16_t sc_frame_crc16(const uint8_t *data, size_t length);
uint8_t scbp_next_tx_sequence(void);
uint8_t scbp_message_priority(uint16_t msg_id);
uint8_t scbp_message_flags(uint16_t msg_id);
int scbp_legacy_type_to_msg_id(uint8_t legacy_type, uint16_t *msg_id);
void scbp_ack_context_set(uint16_t acknowledged_msg_id, uint8_t acknowledged_seq,
                          uint8_t destination);
void scbp_pending_tx_clear(void);
int scbp_pending_tx_match_ack(const scbp_frame_t *ack,
                              uint8_t *legacy_type,
                              uint8_t *legacy_payload0,
                              uint8_t *result);
int scbp_frame_encode(const scbp_frame_t *frame, uint8_t *out, size_t capacity,
                      uint16_t *out_length);
int scbp_frame_decode(const uint8_t *frame, size_t length, scbp_frame_t *view);

/* Compatibility entry for code outside the authorized protocol/service scope.
 * It converts the legacy type identifiers into a SCBP-V3 wire frame. */
int sc_frame_encode(uint8_t legacy_type, const uint8_t *payload, uint16_t length,
                    uint8_t *out, size_t capacity, uint16_t *out_length);
int sc_frame_decode(const uint8_t *frame, size_t length, sc_frame_view_t *view);
void sc_frame_parser_init(sc_frame_parser_t *parser, sc_frame_callback_t callback,
                          sc_frame_error_callback_t error_callback, void *context);
size_t sc_frame_parser_feed(sc_frame_parser_t *parser, const uint8_t *data,
                            size_t length);

#endif /* SC_FRAME_H */
