#ifndef SRP_REGISTRY_H
#define SRP_REGISTRY_H

#include <stddef.h>
#include <stdint.h>

#include "srp_def.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SRP_MSG_ID_CAL_EVENT UINT8_C(0x01)
#define SRP_MSG_ID_MOTOR_CMD UINT8_C(0x02)
#define SRP_MSG_ID_WHEEL_SPEED_CMD SRP_MSG_ID_MOTOR_CMD
#define SRP_MSG_ID_PID_PARAMS_CMD UINT8_C(0x03)
#define SRP_MSG_ID_WHEEL_SPEED_SINGLE_CMD UINT8_C(0x04)
#define SRP_MSG_ID_MASTER_SPEED_CMD UINT8_C(0x05)
#define SRP_MSG_ID_CHASSIS_SPEED_CMD UINT8_C(0x06)
#define SRP_MSG_ID_CHASSIS_HEADING_CMD UINT8_C(0x17)
#define SRP_MSG_ID_BOOT_READY UINT8_C(0x07)
#define SRP_MSG_ID_CMD_SYNC_REQ UINT8_C(0x08)
#define SRP_MSG_ID_RSP_BOOT_INFO UINT8_C(0x09)
#define SRP_MSG_ID_IMU_TELEMETRY UINT8_C(0x10)
#define SRP_MSG_ID_ATTITUDE UINT8_C(0x11)
#define SRP_MSG_ID_IMU_CAL_STATUS UINT8_C(0x12)
#define SRP_MSG_ID_POWER_STATUS UINT8_C(0x13)
#define SRP_MSG_ID_WHEEL_SPEED_STATUS UINT8_C(0x14)
#define SRP_MSG_ID_CHASSIS_STATE UINT8_C(0x15)
#define SRP_MSG_ID_WHEEL_CONTROL_STATUS UINT8_C(0x16)
#define SRP_MSG_ID_RADAR_STATUS UINT8_C(0x20)
#define SRP_MSG_ID_RADAR_PWM_READY UINT8_C(0x21)
#define SRP_MSG_ID_LOG UINT8_C(0x30)
#define SRP_MSG_ID_SYS_CONFIG UINT8_C(0x70)
#define SRP_MSG_ID_ACK UINT8_C(0x7E)
#define SRP_MSG_ID_ERROR UINT8_C(0x7F)

#define SRP_FAST_RESP_OK UINT8_C(0x00)
#define SRP_FAST_RESP_INVALID_PARAM UINT8_C(0x01)
#define SRP_FAST_RESP_BUSY UINT8_C(0x02)
#define SRP_FAST_RESP_TIMEOUT UINT8_C(0x03)
#define SRP_FAST_RESP_CRC_ERROR UINT8_C(0x04)

#define SRP_IMU_SENSOR_LSM303 UINT8_C(0x01)
#define SRP_IMU_SENSOR_BMI323 UINT8_C(0x02)
#define SRP_IMU_TELEMETRY_FLAG_ACCEL_VALID UINT8_C(0x01)
#define SRP_IMU_TELEMETRY_FLAG_GYRO_OR_MAG_VALID UINT8_C(0x02)
#define SRP_IMU_TELEMETRY_FLAG_ONLINE UINT8_C(0x04)
#define SRP_DUAL_AHRS_SCHEMA UINT8_C(2)
#define SRP_CAL_EVENT_STATIC_DONE UINT8_C(1)
#define SRP_CAL_EVENT_VIB_STEP_DONE UINT8_C(2)
#define SRP_CAL_EVENT_COMPLETE UINT8_C(3)

#define SRP_IMU_CAL_STAGE_WAIT_RADAR_READY UINT8_C(0)
#define SRP_IMU_CAL_STAGE_STATIC_STABLE_WAIT UINT8_C(1)
#define SRP_IMU_CAL_STAGE_STATIC_SAMPLE UINT8_C(2)
#define SRP_IMU_CAL_STAGE_COMPLETE UINT8_C(3)
#define SRP_IMU_CAL_STAGE_ERROR UINT8_C(4)

#define SRP_PAYLOAD_BOOT_READY_SIZE UINT16_C(2)
#define SRP_PAYLOAD_CMD_SYNC_REQ_SIZE UINT16_C(4)
#define SRP_PAYLOAD_RSP_BOOT_INFO_SIZE UINT16_C(8)
#define SRP_PAYLOAD_CAL_EVENT_SIZE UINT16_C(1)
#define SRP_PAYLOAD_MOTOR_CMD_SIZE UINT16_C(16)
#define SRP_PAYLOAD_WHEEL_SPEED_CMD_SIZE SRP_PAYLOAD_MOTOR_CMD_SIZE
#define SRP_PAYLOAD_WHEEL_SPEED_SINGLE_CMD_SIZE UINT16_C(5)
#define SRP_PAYLOAD_MASTER_SPEED_CMD_SIZE UINT16_C(4)
#define SRP_PAYLOAD_CHASSIS_SPEED_CMD_SIZE UINT16_C(16)
#define SRP_PAYLOAD_CHASSIS_HEADING_CMD_SIZE UINT16_C(12)
#define SRP_PAYLOAD_PID_PARAMS_SIZE UINT16_C(16)
#define SRP_PAYLOAD_WHEEL_SPEED_STATUS_SIZE UINT16_C(16)
#define SRP_PAYLOAD_CHASSIS_STATE_SIZE UINT16_C(24)
#define SRP_PAYLOAD_WHEEL_CONTROL_STATUS_SIZE UINT16_C(44)
#define SRP_PAYLOAD_POWER_STATUS_SIZE UINT16_C(4)
#define SRP_PAYLOAD_RADAR_STATUS_SIZE UINT16_C(2)
#define SRP_PAYLOAD_RADAR_PWM_READY_SIZE UINT16_C(1)
#define SRP_PAYLOAD_IMU_CAL_STATUS_SIZE UINT16_C(11)
#define SRP_PAYLOAD_IMU_TELEMETRY_SIZE UINT16_C(30)
#define SRP_PAYLOAD_DUAL_AHRS_SIZE UINT16_C(80)
#define SRP_PAYLOAD_FAST_RESPONSE_SIZE UINT16_C(4)

#define SRP_CHASSIS_STATE_SCHEMA UINT8_C(1)
#define SRP_CHASSIS_STATE_FLAG_ATTITUDE_SAFETY_FUSED UINT8_C(0x01)
#define SRP_CHASSIS_STATE_FLAG_HEADING_LOCKED UINT8_C(0x02)
#define SRP_CHASSIS_STATE_FLAG_ODOMETRY_VALID UINT8_C(0x04)
#define SRP_CHASSIS_STATE_FLAG_ATTITUDE_READY UINT8_C(0x08)
#define SRP_WHEEL_CONTROL_STATUS_SCHEMA UINT8_C(1)
#define SRP_CHASSIS_MODE_DIFF UINT8_C(0)
#define SRP_CHASSIS_MODE_WHEEL_INDEPENDENT UINT8_C(1)
#define SRP_CHASSIS_HEADING_FLAGS_NONE UINT32_C(0)

#define SRP_PROTOCOL_VERSION_MAJOR UINT8_C(4)
#define SRP_PROTOCOL_VERSION_MINOR UINT8_C(0)
#define SRP_SYNC_FLAG_VERSION_OK UINT8_C(0x01)
#define SRP_SYNC_FLAG_TIMEOUT UINT8_C(0x02)

typedef enum {
    SRP_STM_STATE_RESET = 0U,
    SRP_STM_STATE_WAIT_FOR_HOST = 1U,
    SRP_STM_STATE_HOST_SYNCED = 2U
} srp_stm_state_t;

#define SRP_PID_KP_MIN 0.0f
#define SRP_PID_KP_MAX 4.0f
#define SRP_PID_KI_MIN 0.0f
#define SRP_PID_KI_MAX 0.3f
#define SRP_PID_KD_MIN 0.0f
#define SRP_PID_KD_MAX 0.1f
#define SRP_PID_ACCEL_MIN 200.0f
#define SRP_PID_ACCEL_MAX 2000.0f

#define SRP_TLV_TAG_BAUDRATE UINT8_C(0x01)
#define SRP_BAUDRATE_DEFAULT UINT32_C(921600)
#define SRP_BAUDRATE_DEBUG UINT32_C(115200)

#pragma pack(push, 4)
typedef struct SRP_PACKED {
    uint8_t ack_type;
    uint8_t reserved;
    uint8_t ack_sequence;
    uint8_t status_code;
} srp_fast_resp_payload_t;

typedef struct SRP_PACKED {
    uint8_t state;
    uint8_t result;
} srp_boot_ready_payload_t;

typedef struct SRP_PACKED {
    uint8_t version_major;
    uint8_t version_minor;
    uint8_t flags;
    uint8_t reserved;
} srp_cmd_sync_req_payload_t;

typedef struct SRP_PACKED {
    uint8_t version_major;
    uint8_t version_minor;
    uint8_t stm_state;
    uint8_t flags;
    uint8_t request_sequence;
    uint8_t reserved[3];
} srp_rsp_boot_info_payload_t;

typedef struct SRP_PACKED {
    uint8_t stage;
    uint8_t radar_pwm;
    uint8_t reserved[2];
    uint32_t sample_count;
    uint32_t sample_total;
    uint8_t error_code;
    uint8_t reserved_tail[3];
} srp_imu_cal_status_payload_t;

typedef struct SRP_PACKED {
    uint8_t sensor_id;
    uint8_t flags;
    uint16_t reserved;
    uint32_t timestamp;
    float accel[3];
    float vector[3];
} srp_imu_telemetry_payload_t;

typedef struct SRP_PACKED {
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
} srp_dual_ahrs_payload_t;

typedef struct SRP_PACKED {
    uint8_t schema;
    uint8_t flags;
    uint16_t reserved;
    uint32_t timestamp_ms;
    float x_mm;
    float y_mm;
    float yaw_deg;
    float total_dist_m;
} srp_chassis_state_payload_t;

typedef struct SRP_PACKED {
    float target_v_mm_s;
    float target_yaw_deg;
    uint32_t flags;
} srp_chassis_heading_cmd_payload_t;
#pragma pack(pop)

_Static_assert(sizeof(srp_fast_resp_payload_t) == 4U, "SRP response size");
_Static_assert(sizeof(srp_boot_ready_payload_t) == 2U, "SRP boot size");
_Static_assert(sizeof(srp_cmd_sync_req_payload_t) == 4U, "SRP sync request size");
_Static_assert(sizeof(srp_rsp_boot_info_payload_t) == 8U, "SRP boot info size");
_Static_assert(sizeof(srp_imu_cal_status_payload_t) == 16U, "SRP cal host size");
_Static_assert(sizeof(srp_imu_telemetry_payload_t) == 32U, "SRP telemetry host size");
_Static_assert(sizeof(srp_dual_ahrs_payload_t) == 80U, "SRP attitude size");
_Static_assert(sizeof(srp_chassis_state_payload_t) == 24U, "SRP chassis size");
_Static_assert(sizeof(srp_chassis_heading_cmd_payload_t) ==
                   SRP_PAYLOAD_CHASSIS_HEADING_CMD_SIZE,
               "SRP chassis heading command size");
_Static_assert(offsetof(srp_fast_resp_payload_t, status_code) == 3U,
               "SRP response status offset");
_Static_assert(offsetof(srp_cmd_sync_req_payload_t, reserved) == 3U,
               "SRP sync reserved offset");
_Static_assert(offsetof(srp_rsp_boot_info_payload_t, request_sequence) == 4U,
               "SRP boot sequence offset");
_Static_assert(SRP_MSG_ID_CMD_SYNC_REQ == UINT8_C(0x08),
               "SRP sync request ID");
_Static_assert(SRP_MSG_ID_RSP_BOOT_INFO == UINT8_C(0x09),
               "SRP boot info ID");
_Static_assert(SRP_PROTOCOL_VERSION_MAJOR == UINT8_C(4),
               "SRP protocol major version");
_Static_assert(SRP_PROTOCOL_VERSION_MINOR == UINT8_C(0),
               "SRP protocol minor version");

#ifdef __cplusplus
}
#endif

#endif /* SRP_REGISTRY_H */
