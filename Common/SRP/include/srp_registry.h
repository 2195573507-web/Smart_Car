#ifndef SRP_REGISTRY_H
#define SRP_REGISTRY_H

#include <stddef.h>
#include <stdint.h>

#include "srp_def.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * SRP v4 消息注册表。
 * 创建人：待确认（当前维护人：Zhiqin）。
 * STM32 和 S3 必须共同引用本表；新增 ID 时同时更新 payload 长度、状态机、
 * ACK 语义和主机侧解码，禁止在单端私自复用已有编号。
 */

/* 低值为控制/同步，高值为遥测/日志；具体优先级由 SRP header 携带。 */
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

/* 快速响应 payload 的 status_code 值。 */
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
/* 以下为实际 wire payload 字节数，不等于带保留/对齐字段的 host struct sizeof。 */
#define SRP_PAYLOAD_IMU_CAL_STATUS_SIZE UINT16_C(11)
#define SRP_PAYLOAD_IMU_TELEMETRY_SIZE UINT16_C(30)
#define SRP_PAYLOAD_DUAL_AHRS_SIZE UINT16_C(80)
#define SRP_PAYLOAD_FAST_RESPONSE_SIZE UINT16_C(4)

#define SRP_CHASSIS_STATE_SCHEMA UINT8_C(1)
#define SRP_CHASSIS_STATE_FLAG_ATTITUDE_SAFETY_FUSED UINT8_C(0x01)
#define SRP_CHASSIS_STATE_FLAG_HEADING_LOCKED UINT8_C(0x02)
#define SRP_CHASSIS_STATE_FLAG_ODOMETRY_VALID UINT8_C(0x04)
#define SRP_CHASSIS_STATE_FLAG_ATTITUDE_READY UINT8_C(0x08)
#define SRP_CHASSIS_STATE_FLAGS_MASK UINT8_C(0x0F)
#define SRP_WHEEL_CONTROL_STATUS_SCHEMA UINT8_C(1)
#define SRP_CHASSIS_MODE_DIFF UINT8_C(0)
#define SRP_CHASSIS_MODE_WHEEL_INDEPENDENT UINT8_C(1)
#define SRP_CHASSIS_HEADING_FLAGS_NONE UINT32_C(0)

#define SRP_PROTOCOL_VERSION_MAJOR UINT8_C(4)
#define SRP_PROTOCOL_VERSION_MINOR UINT8_C(0)
#define SRP_SYNC_FLAG_VERSION_OK UINT8_C(0x01)
#define SRP_SYNC_FLAG_TIMEOUT UINT8_C(0x02)

/** STM 侧 SRP 会话状态，在线缆启动信息中以 1 byte 发布。 */
typedef enum {
    SRP_STM_STATE_RESET = 0U,         /**< STM 服务尚未进入等待主机同步状态。 */
    SRP_STM_STATE_WAIT_FOR_HOST = 1U, /**< 等待合法 CMD_SYNC_REQ，运动命令不得准入。 */
    SRP_STM_STATE_HOST_SYNCED = 2U    /**< 已接受同步请求并成功提交 RSP_BOOT_INFO。 */
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
/** 快速 ACK/NACK 的固定 4 字节线缆 payload。 */
typedef struct SRP_PACKED {
    uint8_t ack_type; /**< 被确认消息的 8 位 SRP_MSG_ID_* 类型。 */
    uint8_t reserved; /**< 线缆保留字节；发送必须置 0，接收方不得赋予业务语义。 */
    uint8_t ack_sequence; /**< 被确认消息的 8 位序号。 */
    uint8_t status_code; /**< SRP_FAST_RESP_* 成功/拒绝/超时/CRC 状态码。 */
} srp_fast_resp_payload_t;

/** STM 启动就绪状态；state/result 的取值由两端状态机解释。 */
typedef struct SRP_PACKED {
    uint8_t state; /**< 当前 IMU 启动/标定阶段；启动握手当前使用 WAIT_RADAR_READY。 */
    uint8_t result; /**< 阶段结果码；当前 BOOT_READY 成功事件要求为 0。 */
} srp_boot_ready_payload_t;

/** S3 发起的版本同步请求，reserved 字节必须为 0。 */
typedef struct SRP_PACKED {
    uint8_t version_major; /**< 请求方 SRP 主版本；当前必须为 4。 */
    uint8_t version_minor; /**< 请求方 SRP 次版本；当前必须为 0。 */
    uint8_t flags; /**< 请求标志；当前同步准入只接受 0。 */
    uint8_t reserved; /**< 保留字节；发送必须置 0，非零请求被拒绝。 */
} srp_cmd_sync_req_payload_t;

/** STM 返回的启动信息；request_sequence 回显同步请求序号。 */
typedef struct SRP_PACKED {
    uint8_t version_major; /**< STM 支持的 SRP 主版本；当前为 4。 */
    uint8_t version_minor; /**< STM 支持的 SRP 次版本；当前为 0。 */
    uint8_t stm_state; /**< srp_stm_state_t 数值；同步成功响应必须为 HOST_SYNCED。 */
    uint8_t flags; /**< SRP_SYNC_FLAG_* 响应标志；成功响应当前为 VERSION_OK。 */
    uint8_t request_sequence; /**< 原样回显 CMD_SYNC_REQ 的 SRP 序号。 */
    uint8_t reserved[3]; /**< 线缆保留字节；发送必须全 0。 */
} srp_rsp_boot_info_payload_t;

/**
 * IMU 标定状态的 host-side 逻辑容器，sizeof=16。
 * wire payload 仅序列化 11 字节，见 SRP_PAYLOAD_IMU_CAL_STATUS_SIZE；禁止 memcpy 发送。
 */
typedef struct SRP_PACKED {
    uint8_t stage; /**< SRP_IMU_CAL_STAGE_* 标定阶段。 */
    uint8_t radar_pwm; /**< 历史命名的 byte 1 状态；当前 wire 生产者固定写 0。 */
    uint8_t reserved[2]; /**< 仅 host 容器对齐占位，不属于 11 byte wire payload。 */
    uint32_t sample_count; /**< 已采样数量，无单位计数；wire 中从偏移 2 显式编码。 */
    uint32_t sample_total; /**< 目标样本总数，无单位计数；wire 中从偏移 6 显式编码。 */
    uint8_t error_code; /**< 标定/启动错误码；0 表示当前未报告错误。 */
    uint8_t reserved_tail[3]; /**< 仅 host 容器尾部占位，不在线缆上发送。 */
} srp_imu_cal_status_payload_t;

/**
 * IMU 遥测的 host-side 逻辑容器，sizeof=32。
 * wire payload 为 30 字节，浮点值和省略/保留字段必须按注册表显式编码。
 */
typedef struct SRP_PACKED {
    uint8_t sensor_id; /**< SRP_IMU_SENSOR_LSM303 或 SRP_IMU_SENSOR_BMI323。 */
    uint8_t flags; /**< ACCEL_VALID、GYRO_OR_MAG_VALID、ONLINE 状态位。 */
    uint16_t reserved; /**< 仅 host 容器对齐占位，不属于 30 byte wire payload。 */
    uint32_t timestamp; /**< 传感器采样时间戳；当前生产者单位为单调 ms。 */
    float accel[3]; /**< 机体系 X/Y/Z 加速度，单位 m/s^2。 */
    float vector[3]; /**< LSM303 为磁场向量，BMI323 为角速度 rad/s。 */
} srp_imu_telemetry_payload_t;

/** 双 AHRS 姿态快照，schema=2；只读消费者不得修改 payload。 */
typedef struct SRP_PACKED {
    uint8_t schema; /**< 固定为 SRP_DUAL_AHRS_SCHEMA(2)，用于拒绝不兼容布局。 */
    uint8_t flags; /**< DualAHRS 有效性/新鲜度状态位；不能仅凭非零判定全部有效。 */
    uint16_t reserved; /**< 线缆保留字段；发送必须置 0。 */
    uint32_t timestamp_ms; /**< 快照单调时间戳，单位 ms。 */
    uint32_t sample_sequence; /**< 发布样本序号，32 位自然回绕。 */
    float primary_euler[3]; /**< 主 BMI323 姿态 roll/pitch/yaw，单位 rad。 */
    float primary_quat[4]; /**< 主姿态四元数 w/x/y/z，无量纲。 */
    float redundant_euler[3]; /**< 冗余 LSM303 姿态 roll/pitch/yaw，单位 rad。 */
    float redundant_quat[4]; /**< 冗余姿态四元数 w/x/y/z，无量纲。 */
    float delta_euler[3]; /**< 主减冗余的 roll/pitch/yaw 差值，单位 rad。 */
} srp_dual_ahrs_payload_t;

/** 底盘状态快照，单位分别为毫秒、毫米和角度。 */
typedef struct SRP_PACKED {
    uint8_t schema; /**< 固定为 SRP_CHASSIS_STATE_SCHEMA(1)。 */
    uint8_t flags; /**< SRP_CHASSIS_STATE_FLAG_* 状态位，未知高位必须为 0。 */
    uint16_t reserved; /**< 线缆保留字段；发送必须置 0。 */
    uint32_t timestamp_ms; /**< 轮速快照的单调时间戳，单位 ms；无有效轮速时可为 0。 */
    float x_mm; /**< 里程计 X 位置，单位 mm。 */
    float y_mm; /**< 里程计 Y 位置，单位 mm。 */
    float yaw_deg; /**< 底盘航向角，单位 degree。 */
    float total_dist_m; /**< 累计行程，单位 m，合法发布值不小于 0。 */
} srp_chassis_state_payload_t;

/** 目标航向命令：线速度 mm/s、目标航向 deg、保留 flags。 */
typedef struct SRP_PACKED {
    float target_v_mm_s; /**< 目标底盘线速度，单位 mm/s。 */
    float target_yaw_deg; /**< 目标绝对航向，单位 degree。 */
    uint32_t flags; /**< 命令标志；当前仅接受 SRP_CHASSIS_HEADING_FLAGS_NONE(0)。 */
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
_Static_assert(offsetof(srp_chassis_state_payload_t, timestamp_ms) == 4U,
               "SRP chassis timestamp offset");
_Static_assert(offsetof(srp_chassis_state_payload_t, x_mm) == 8U,
               "SRP chassis x offset");
_Static_assert(offsetof(srp_chassis_state_payload_t, total_dist_m) == 20U,
               "SRP chassis distance offset");
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
