#ifndef MOTOR_BOARD_PROTOCOL_H
#define MOTOR_BOARD_PROTOCOL_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MB_PROTOCOL_MAX_FRAME_LEN UINT16_C(128)

typedef enum {
    MB_FRAME_INVALID = 0U,
    MB_FRAME_BATTERY,
    MB_FRAME_MTEP,
    MB_FRAME_MSPD,
    MB_FRAME_MALL,
    MB_FRAME_ACK,
    /* Explicit name for the pure-text acknowledgement, kept value-compatible
     * with the original generic ACK type. */
    MB_FRAME_OK_ACK = MB_FRAME_ACK,
    MB_FRAME_NACK,
    MB_FRAME_UNKNOWN,
    MB_FRAME_FLASH_RAW
} mb_frame_type_t;

/* Protocol naming used by the motor-board response specification. */
#define FRAME_TYPE_OK_ACK MB_FRAME_OK_ACK
#define FRAME_TYPE_FLASH_RAW MB_FRAME_FLASH_RAW

typedef struct {
    mb_frame_type_t type;
    char raw[MB_PROTOCOL_MAX_FRAME_LEN];
    float battery_voltage;
    int32_t pulse[4];
    float speed[4];
    char response_status[MB_PROTOCOL_MAX_FRAME_LEN];
} mb_protocol_frame_t;

typedef struct {
    uint32_t frames;
    uint32_t invalid_frames;
    uint32_t overflow_frames;
} mb_protocol_stats_t;

void MB_Protocol_Init(void);
void MB_Protocol_ResetRx(void);
bool MB_Protocol_Poll(mb_protocol_frame_t *frame);
bool MB_Protocol_SendPwm(int16_t m1, int16_t m2, int16_t m3, int16_t m4);
bool MB_Protocol_SendSpeed(int16_t m1, int16_t m2, int16_t m3, int16_t m4);
bool MB_Protocol_SendMotorType(uint8_t motor_type);
bool MB_Protocol_SendMagneticLine(uint16_t magnetic_line_count);
bool MB_Protocol_SendGearRatio(uint16_t gear_ratio);
bool MB_Protocol_SendWheelDiameter(uint16_t diameter_mm);
bool MB_Protocol_SendReadVoltage(void);
bool MB_Protocol_SendReadFlash(void);
bool MB_Protocol_SendUpload(bool all_encoder, bool ten_ms_encoder, bool speed);
void MB_Protocol_GetStats(mb_protocol_stats_t *stats);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_BOARD_PROTOCOL_H */
