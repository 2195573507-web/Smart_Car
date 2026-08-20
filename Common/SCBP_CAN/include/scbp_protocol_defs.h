#ifndef SCBP_PROTOCOL_DEFS_H
#define SCBP_PROTOCOL_DEFS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SCBP_CAN_SOF0 UINT8_C(0x5A)
#define SCBP_CAN_SOF1 UINT8_C(0xA5)
#define SCBP_CAN_EOF0 UINT8_C(0x0D)
#define SCBP_CAN_EOF1 UINT8_C(0x0A)
#define SCBP_CAN_HEADER_SIZE UINT8_C(8)
#define SCBP_CAN_TRAILER_SIZE UINT8_C(4)
#define SCBP_CAN_MAX_PAYLOAD UINT16_C(255)
#define SCBP_CAN_MAX_FRAME_SIZE (SCBP_CAN_HEADER_SIZE + SCBP_CAN_MAX_PAYLOAD + SCBP_CAN_TRAILER_SIZE)

#define SCBP_CAN_PRIORITY_MASK UINT16_C(0xC000)
#define SCBP_CAN_SOURCE_MASK UINT16_C(0x3000)
#define SCBP_CAN_DESTINATION_MASK UINT16_C(0x0C00)
#define SCBP_CAN_MESSAGE_MASK UINT16_C(0x03FF)
#define SCBP_CAN_PRIORITY_SHIFT UINT8_C(14)
#define SCBP_CAN_SOURCE_SHIFT UINT8_C(12)
#define SCBP_CAN_DESTINATION_SHIFT UINT8_C(10)

#define SCBP_CAN_PRIORITY_EMERGENCY UINT8_C(0)
#define SCBP_CAN_PRIORITY_REALTIME UINT8_C(1)
#define SCBP_CAN_PRIORITY_NORMAL UINT8_C(2)
#define SCBP_CAN_PRIORITY_DEBUG UINT8_C(3)

#define SCBP_NODE_STM32H757 UINT8_C(1)
#define SCBP_NODE_ESP32_S3 UINT8_C(2)
#define SCBP_NODE_BROADCAST UINT8_C(3)
#define SCBP_NODE_APP UINT8_C(3)

#define SCBP_CAN_FLAG_ACK_REQUIRED UINT8_C(0x01)
#define SCBP_CAN_FLAG_IS_ACK UINT8_C(0x02)
#define SCBP_CAN_FLAG_IS_ERROR UINT8_C(0x04)
#define SCBP_CAN_FLAG_STREAM_DATA UINT8_C(0x08)
#define SCBP_CAN_FLAG_RESERVED_MASK UINT8_C(0xF0)

#define SCBP_MSG_ID_CAL_EVENT UINT16_C(0x001)
#define SCBP_MSG_ID_ACK UINT16_C(0x005)
#define SCBP_MSG_ID_ERROR UINT16_C(0x006)
#define SCBP_MSG_ID_BOOT_READY UINT16_C(0x007)
#define SCBP_MSG_ID_ATTITUDE UINT16_C(0x201)
#define SCBP_MSG_ID_IMU_CAL_STATUS UINT16_C(0x202)
#define SCBP_MSG_ID_IMU_TELEMETRY UINT16_C(0x207)
#define SCBP_MSG_ID_RADAR_STATUS UINT16_C(0x301)
#define SCBP_MSG_ID_RADAR_PWM_READY UINT16_C(0x302)
#define SCBP_MSG_ID_LOG UINT16_C(0x3F0)

#define SCBP_FAST_RESP_OK UINT8_C(0x00)
#define SCBP_FAST_RESP_HCS_ERROR UINT8_C(0x01)
#define SCBP_FAST_RESP_FCS_ERROR UINT8_C(0x02)
#define SCBP_FAST_RESP_BUSY UINT8_C(0x03)
#define SCBP_FAST_RESP_TIMEOUT UINT8_C(0x04)
#define SCBP_FAST_RESP_INVALID_PARAM UINT8_C(0x05)

#define SCBP_IMU_SENSOR_LSM303 UINT8_C(0x01)
#define SCBP_IMU_SENSOR_BMI323 UINT8_C(0x02)
#define SCBP_IMU_TELEMETRY_FLAG_ACCEL_VALID UINT8_C(0x01)
#define SCBP_IMU_TELEMETRY_FLAG_GYRO_OR_MAG_VALID UINT8_C(0x02)
#define SCBP_IMU_TELEMETRY_FLAG_ONLINE UINT8_C(0x04)
#define SCBP_DUAL_AHRS_SCHEMA UINT8_C(2)
#define SCBP_CAL_EVENT_STATIC_DONE UINT8_C(1)
#define SCBP_CAL_EVENT_VIB_STEP_DONE UINT8_C(2)
#define SCBP_CAL_EVENT_COMPLETE UINT8_C(3)

#define SCBP_IMU_CAL_STAGE_WAIT_RADAR_READY UINT8_C(0)
#define SCBP_IMU_CAL_STAGE_STATIC_STABLE_WAIT UINT8_C(1)
#define SCBP_IMU_CAL_STAGE_STATIC_SAMPLE UINT8_C(2)
#define SCBP_IMU_CAL_STAGE_COMPLETE UINT8_C(3)
#define SCBP_IMU_CAL_STAGE_ERROR UINT8_C(4)

#define SCBP_CAN_ID(priority, source, destination, message) \
    (uint16_t)((((uint16_t)(priority) << SCBP_CAN_PRIORITY_SHIFT) & SCBP_CAN_PRIORITY_MASK) | \
               (((uint16_t)(source) << SCBP_CAN_SOURCE_SHIFT) & SCBP_CAN_SOURCE_MASK) | \
               (((uint16_t)(destination) << SCBP_CAN_DESTINATION_SHIFT) & SCBP_CAN_DESTINATION_MASK) | \
               ((uint16_t)(message) & SCBP_CAN_MESSAGE_MASK))
#define SCBP_CAN_ID_PRIORITY(id) (uint8_t)(((id) & SCBP_CAN_PRIORITY_MASK) >> SCBP_CAN_PRIORITY_SHIFT)
#define SCBP_CAN_ID_SOURCE(id) (uint8_t)(((id) & SCBP_CAN_SOURCE_MASK) >> SCBP_CAN_SOURCE_SHIFT)
#define SCBP_CAN_ID_DESTINATION(id) (uint8_t)(((id) & SCBP_CAN_DESTINATION_MASK) >> SCBP_CAN_DESTINATION_SHIFT)
#define SCBP_CAN_ID_MESSAGE(id) (uint16_t)((id) & SCBP_CAN_MESSAGE_MASK)

#define SCBP_PAYLOAD_BOOT_READY_SIZE UINT16_C(2)
#define SCBP_PAYLOAD_CAL_EVENT_SIZE UINT16_C(1)
#define SCBP_PAYLOAD_RADAR_STATUS_SIZE UINT16_C(2)
#define SCBP_PAYLOAD_RADAR_PWM_READY_SIZE UINT16_C(1)
#define SCBP_PAYLOAD_IMU_CAL_STATUS_SIZE UINT16_C(11)
#define SCBP_PAYLOAD_IMU_TELEMETRY_SIZE UINT16_C(30)
#define SCBP_PAYLOAD_DUAL_AHRS_SIZE UINT16_C(80)

#pragma pack(push, 1)
typedef struct {
    uint16_t ack_can_id;
    uint8_t ack_seq;
    uint8_t status_code;
} scbp_fast_resp_payload_t;

typedef struct {
    uint8_t state;
    uint8_t result;
} scbp_boot_ready_payload_t;

typedef struct {
    uint8_t stage;
    uint8_t radar_pwm;
    uint32_t sample_count;
    uint32_t sample_total;
    uint8_t error_code;
} scbp_imu_cal_status_payload_t;

typedef struct {
    uint8_t sensor_id;
    uint8_t flags;
    uint32_t timestamp;
    float accel[3];
    float vector[3];
} scbp_imu_telemetry_payload_t;

typedef struct {
    uint8_t schema;
    uint8_t flags;
    uint16_t reserved;
    uint32_t timestamp_ms;
    uint32_t sample_sequence;
    float primary_euler[3];
    float primary_quat[4];
    float redundant_euler[3];
    float redundant_quat[4];
    float delta_euler[3];
} scbp_dual_ahrs_payload_t;
#pragma pack(pop)

_Static_assert(sizeof(scbp_fast_resp_payload_t) == 4U, "SCBP fast response size");
_Static_assert(sizeof(scbp_boot_ready_payload_t) == 2U, "SCBP boot payload size");
_Static_assert(sizeof(scbp_imu_cal_status_payload_t) == 11U, "SCBP calibration payload size");
_Static_assert(sizeof(scbp_imu_telemetry_payload_t) == 30U, "SCBP telemetry payload size");
_Static_assert(sizeof(scbp_dual_ahrs_payload_t) == 80U, "SCBP DualAHRS payload size");

#ifdef __cplusplus
}
#endif

#endif /* SCBP_PROTOCOL_DEFS_H */
