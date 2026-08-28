#include "motor_board_protocol.h"

#include <ctype.h>
#include <errno.h>
#include <limits.h>
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "motor_board_transport_uart.h"

#define MB_PROTOCOL_PAYLOAD_CAPACITY \
    (MB_PROTOCOL_MAX_FRAME_LEN - UINT16_C(3))
#define MB_PROTOCOL_LINE_CAPACITY MB_PROTOCOL_MAX_FRAME_LEN
#define MB_PROTOCOL_TX_TEXT_CAPACITY UINT16_C(64)
#define MB_PROTOCOL_RAW_DIAGNOSTIC_SIZE UINT16_C(24)

/* Motor-board order is M1=RR, M2=RF, M3=LR, M4=LF. MTEP pulse diagnostics
 * retain the motor-board's electrical polarity for reporting only. Encoder
 * speed calibration belongs to the closed-loop task. */
static const int8_t PULSE_DIR_SIGN[4] = { 1, -1, 1, 1 };

typedef enum {
    MB_RX_WAIT_START = 0U,
    MB_RX_IN_FRAME,
    MB_RX_IN_LINE
} mb_rx_state_t;

static mb_rx_state_t s_rx_state;
static char s_rx_payload[MB_PROTOCOL_PAYLOAD_CAPACITY];
static uint16_t s_rx_length;
static uint8_t s_rx_overflow;
static char s_rx_line[MB_PROTOCOL_LINE_CAPACITY];
static uint16_t s_rx_line_length;
static uint8_t s_rx_line_overflow;
static uint8_t s_raw_diagnostic[MB_PROTOCOL_RAW_DIAGNOSTIC_SIZE];
static uint16_t s_raw_diagnostic_length;
static uint8_t s_read_flash_active;
static mb_flash_config_t s_flash_config;
static mb_protocol_stats_t s_stats;

static char ascii_lower(char value)
{
    return (char)tolower((unsigned char)value);
}

static bool text_starts_with_ci(const char *text, const char *prefix)
{
    size_t index;

    if (text == NULL || prefix == NULL) {
        return false;
    }
    for (index = 0U; prefix[index] != '\0'; ++index) {
        if (text[index] == '\0' ||
            ascii_lower(text[index]) != ascii_lower(prefix[index])) {
            return false;
        }
    }
    return true;
}

static bool text_contains_ci(const char *text, const char *needle)
{
    size_t text_length;
    size_t needle_length;

    if (text == NULL || needle == NULL) {
        return false;
    }
    text_length = strlen(text);
    needle_length = strlen(needle);
    if (needle_length == 0U || needle_length > text_length) {
        return false;
    }
    for (size_t start = 0U; start + needle_length <= text_length; ++start) {
        size_t offset;
        for (offset = 0U; offset < needle_length; ++offset) {
            if (ascii_lower(text[start + offset]) !=
                ascii_lower(needle[offset])) {
                break;
            }
        }
        if (offset == needle_length) {
            return true;
        }
    }
    return false;
}

static bool protocol_is_text_byte(uint8_t byte)
{
    return byte == '\t' || (byte >= 0x20U && byte <= 0x7EU);
}

static const char *skip_spaces(const char *text)
{
    while (text != NULL && *text != '\0' &&
           isspace((unsigned char)*text) != 0) {
        ++text;
    }
    return text;
}

static bool parse_int_values(const char *text, int32_t values[4])
{
    const char *cursor = text;

    if (cursor == NULL || values == NULL) {
        return false;
    }
    for (size_t index = 0U; index < 4U; ++index) {
        char *end = NULL;
        long value;

        cursor = skip_spaces(cursor);
        errno = 0;
        value = strtol(cursor, &end, 10);
        if (end == cursor || errno == ERANGE ||
            value < INT32_MIN || value > INT32_MAX) {
            return false;
        }
        values[index] = (int32_t)value;
        cursor = skip_spaces(end);
        if (index != 3U) {
            if (*cursor != ',') {
                return false;
            }
            ++cursor;
        }
    }
    return *skip_spaces(cursor) == '\0';
}

static int32_t apply_pulse_polarity(int32_t value, int8_t sign)
{
    const int64_t corrected = (int64_t)value * (int64_t)sign;

    if (corrected > INT32_MAX) {
        return INT32_MAX;
    }
    if (corrected < INT32_MIN) {
        return INT32_MIN;
    }
    return (int32_t)corrected;
}

static bool parse_float_values(const char *text, float values[4])
{
    const char *cursor = text;

    if (cursor == NULL || values == NULL) {
        return false;
    }
    for (size_t index = 0U; index < 4U; ++index) {
        char *end = NULL;
        float value;

        cursor = skip_spaces(cursor);
        errno = 0;
        value = strtof(cursor, &end);
        if (end == cursor || errno == ERANGE || !isfinite(value)) {
            return false;
        }
        values[index] = value;
        cursor = skip_spaces(end);
        if (index != 3U) {
            if (*cursor != ',') {
                return false;
            }
            ++cursor;
        }
    }
    return *skip_spaces(cursor) == '\0';
}

static bool parse_battery(const char *text, float *voltage)
{
    char *end = NULL;
    float value;
    const char *cursor;

    if (!text_starts_with_ci(text, "Battery:") || voltage == NULL) {
        return false;
    }
    cursor = skip_spaces(text + strlen("Battery:"));
    errno = 0;
    value = strtof(cursor, &end);
    if (end == cursor || errno == ERANGE || !isfinite(value)) {
        return false;
    }
    end = (char *)skip_spaces(end);
    if (*end == 'V' || *end == 'v') {
        end = (char *)skip_spaces(end + 1);
    }
    if (*end != '\0') {
        return false;
    }
    *voltage = value;
    return true;
}

static bool parse_uint16_field(const char *text, uint32_t maximum,
                               uint16_t *value)
{
    const char *cursor = skip_spaces(text);
    char *end = NULL;
    unsigned long parsed;

    if (cursor == NULL || value == NULL || *cursor == '\0') {
        return false;
    }
    errno = 0;
    parsed = strtoul(cursor, &end, 10);
    if (end == cursor || errno == ERANGE || parsed > maximum ||
        *skip_spaces(end) != '\0') {
        return false;
    }
    *value = (uint16_t)parsed;
    return true;
}

static bool parse_float_field(const char *text, float *value)
{
    const char *cursor = skip_spaces(text);
    char *end = NULL;
    float parsed;

    if (cursor == NULL || value == NULL || *cursor == '\0') {
        return false;
    }
    errno = 0;
    parsed = strtof(cursor, &end);
    if (end == cursor || errno == ERANGE || !isfinite(parsed) ||
        *skip_spaces(end) != '\0') {
        return false;
    }
    *value = parsed;
    return true;
}

static bool flash_field_prefix(const char *text, const char *name,
                               const char **value_text)
{
    const size_t name_length = name == NULL ? 0U : strlen(name);

    if (text == NULL || name == NULL || value_text == NULL ||
        name_length == 0U || !text_starts_with_ci(text, name) ||
        text[name_length] != ':') {
        return false;
    }
    *value_text = text + name_length + 1U;
    return true;
}

static bool protocol_parse_flash_line(const char *text)
{
    const char *value_text = NULL;
    uint32_t field;
    uint16_t parsed_uint16;

    if (text == NULL) {
        return false;
    }
    if (flash_field_prefix(text, "Motor_type", &value_text)) {
        field = MB_FLASH_FIELD_MOTOR_TYPE;
        if ((s_flash_config.fields_present & field) != 0U ||
            !parse_uint16_field(value_text, 4U, &parsed_uint16) ||
            parsed_uint16 == 0U) {
            s_flash_config.invalid = true;
        } else {
            s_flash_config.motor_type = (uint8_t)parsed_uint16;
            s_flash_config.fields_present |= field;
        }
        return true;
    }
    if (flash_field_prefix(text, "Dead_Zone", &value_text)) {
        field = MB_FLASH_FIELD_DEAD_ZONE;
        if ((s_flash_config.fields_present & field) != 0U ||
            !parse_uint16_field(value_text, 3600U,
                                &s_flash_config.dead_zone)) {
            s_flash_config.invalid = true;
        } else {
            s_flash_config.fields_present |= field;
        }
        return true;
    }
    if (flash_field_prefix(text, "Pulse_Line", &value_text)) {
        field = MB_FLASH_FIELD_PULSE_LINE;
        if ((s_flash_config.fields_present & field) != 0U ||
            !parse_uint16_field(value_text, UINT16_MAX,
                                &s_flash_config.pulse_line) ||
            s_flash_config.pulse_line == 0U) {
            s_flash_config.invalid = true;
        } else {
            s_flash_config.fields_present |= field;
        }
        return true;
    }
    if (flash_field_prefix(text, "Pulse_Phase", &value_text)) {
        field = MB_FLASH_FIELD_PULSE_PHASE;
        if ((s_flash_config.fields_present & field) != 0U ||
            !parse_uint16_field(value_text, UINT16_MAX,
                                &s_flash_config.pulse_phase) ||
            s_flash_config.pulse_phase == 0U) {
            s_flash_config.invalid = true;
        } else {
            s_flash_config.fields_present |= field;
        }
        return true;
    }
    if (flash_field_prefix(text, "wheel_diameter", &value_text)) {
        field = MB_FLASH_FIELD_WHEEL_DIAMETER;
        if ((s_flash_config.fields_present & field) != 0U ||
            !parse_float_field(value_text, &s_flash_config.wheel_diameter) ||
            s_flash_config.wheel_diameter <= 0.0f ||
            s_flash_config.wheel_diameter > 1000.0f) {
            s_flash_config.invalid = true;
        } else {
            s_flash_config.fields_present |= field;
        }
        return true;
    }
    if (text_starts_with_ci(text, "read_flash:") ||
        text_starts_with_ci(text, "Motor_Version:") ||
        text_starts_with_ci(text, "P:") ||
        text_starts_with_ci(text, "P ")) {
        return true;
    }
    return false;
}

static void protocol_format_raw(char *destination, size_t capacity,
                                const uint8_t *bytes, uint16_t length)
{
    size_t offset = 0U;
    int written;

    if (destination == NULL || capacity == 0U || bytes == NULL) {
        return;
    }
    destination[0] = '\0';
    written = snprintf(destination, capacity, "hex=");
    if (written < 0) {
        return;
    }
    offset = (size_t)written < capacity ? (size_t)written : capacity - 1U;
    for (uint16_t index = 0U; index < length && offset + 3U < capacity;
         ++index) {
        written = snprintf(&destination[offset], capacity - offset,
                           "%02X%s", bytes[index],
                           index + 1U == length ? "" : " ");
        if (written < 0) {
            return;
        }
        offset += (size_t)written;
    }
    if (offset + 7U < capacity) {
        written = snprintf(&destination[offset], capacity - offset,
                           " ascii=");
        if (written < 0) {
            return;
        }
        offset += (size_t)written;
    }
    for (uint16_t index = 0U; index < length && offset + 2U < capacity;
         ++index) {
        const uint8_t byte = bytes[index];
        destination[offset++] = isprint(byte) != 0 ? (char)byte : '.';
        destination[offset] = '\0';
    }
}

static mb_frame_type_t decode_payload(const char *payload,
                                      mb_protocol_frame_t *frame,
                                      bool line_mode);

static bool protocol_emit_raw_diagnostic(mb_protocol_frame_t *frame)
{
    if (frame == NULL || s_raw_diagnostic_length == 0U) {
        return false;
    }
    (void)memset(frame, 0, sizeof(*frame));
    protocol_format_raw(frame->raw, sizeof(frame->raw), s_raw_diagnostic,
                        s_raw_diagnostic_length);
    frame->type = MB_FRAME_INVALID;
    s_raw_diagnostic_length = 0U;
    ++s_stats.frames;
    ++s_stats.invalid_frames;
    return true;
}

static bool protocol_emit_text_line(mb_protocol_frame_t *frame)
{
    mb_frame_type_t type;

    if (frame == NULL ||
        (s_rx_line_length == 0U && s_rx_line_overflow == 0U)) {
        return false;
    }
    (void)memset(frame, 0, sizeof(*frame));
    if (s_rx_line_length != 0U) {
        (void)memcpy(frame->raw, s_rx_line, s_rx_line_length);
    }
    frame->raw[s_rx_line_length] = '\0';
    if (s_rx_line_overflow != 0U) {
        type = MB_FRAME_INVALID;
        ++s_stats.overflow_frames;
    } else {
        type = decode_payload(s_rx_line, frame, true);
    }
    frame->type = type;
    s_rx_line_length = 0U;
    s_rx_line_overflow = 0U;
    (void)memset(s_rx_line, 0, sizeof(s_rx_line));
    ++s_stats.frames;
    if (type == MB_FRAME_INVALID) {
        ++s_stats.invalid_frames;
    }
    return true;
}

static void protocol_set_response_status(mb_protocol_frame_t *frame,
                                         const char *text)
{
    if (frame == NULL || text == NULL) {
        return;
    }
    (void)snprintf(frame->response_status, sizeof(frame->response_status),
                   "%s", text);
}

static mb_frame_type_t decode_payload(const char *payload,
                                      mb_protocol_frame_t *frame,
                                      bool line_mode)
{
    int32_t integer_values[4] = {0};

    if (parse_battery(payload, &frame->battery_voltage)) {
        return MB_FRAME_BATTERY;
    }
    if (text_starts_with_ci(payload, "MTEP:")) {
        if (parse_int_values(payload + strlen("MTEP:"), integer_values)) {
            for (size_t index = 0U; index < 4U; ++index) {
                frame->pulse[index] =
                    apply_pulse_polarity(integer_values[index],
                                         PULSE_DIR_SIGN[index]);
            }
            return MB_FRAME_MTEP;
        }
        return MB_FRAME_INVALID;
    }
    if (text_starts_with_ci(payload, "MAll:")) {
        if (parse_int_values(payload + strlen("MAll:"), integer_values)) {
            for (size_t index = 0U; index < 4U; ++index) {
                frame->pulse[index] = integer_values[index];
            }
            return MB_FRAME_MALL;
        }
        return MB_FRAME_INVALID;
    }
    if (text_starts_with_ci(payload, "MSPD:")) {
        if (parse_float_values(payload + strlen("MSPD:"), frame->speed)) {
            return MB_FRAME_MSPD;
        }
        return MB_FRAME_INVALID;
    }
    if (line_mode && s_read_flash_active != 0U &&
        protocol_parse_flash_line(payload)) {
        return MB_FRAME_FLASH_RAW;
    }
    if (text_contains_ci(payload, "NACK") ||
        text_contains_ci(payload, "NOK") ||
        text_contains_ci(payload, "ERROR") ||
        text_contains_ci(payload, "FAIL")) {
        protocol_set_response_status(frame, payload);
        return MB_FRAME_NACK;
    }
    if (text_contains_ci(payload, "OK") ||
        text_contains_ci(payload, "ACK") ||
        text_contains_ci(payload, "Set ")) {
        protocol_set_response_status(frame, payload);
        return MB_FRAME_OK_ACK;
    }
    if (line_mode && s_read_flash_active != 0U) {
        return MB_FRAME_FLASH_RAW;
    }
    return MB_FRAME_UNKNOWN;
}

static bool MB_Protocol_SendText(const char *text)
{
    const size_t length = text == NULL ? 0U : strlen(text);

    return length != 0U && length < MB_PROTOCOL_TX_TEXT_CAPACITY &&
           MB_Transport_Send((const uint8_t *)text, (uint16_t)length);
}

static bool MB_Protocol_SendFour(const char *command, int16_t m1, int16_t m2,
                                 int16_t m3, int16_t m4)
{
    char text[MB_PROTOCOL_TX_TEXT_CAPACITY];
    int written;

    written = snprintf(text, sizeof(text), "$%s:%d,%d,%d,%d#", command,
                       (int)m1, (int)m2, (int)m3, (int)m4);
    return written > 0 && (size_t)written < sizeof(text) &&
           MB_Protocol_SendText(text);
}

static bool MB_Protocol_SendUnsigned(const char *command, uint16_t value)
{
    char text[MB_PROTOCOL_TX_TEXT_CAPACITY];
    int written;

    if (command == NULL) {
        return false;
    }
    written = snprintf(text, sizeof(text), "$%s:%u#", command,
                       (unsigned int)value);
    return written > 0 && (size_t)written < sizeof(text) &&
           MB_Protocol_SendText(text);
}

void MB_Protocol_Init(void)
{
    MB_Protocol_ResetRx();
    (void)memset(&s_stats, 0, sizeof(s_stats));
}

void MB_Protocol_ResetRx(void)
{
    s_rx_state = MB_RX_WAIT_START;
    s_rx_length = 0U;
    s_rx_overflow = 0U;
    s_rx_line_length = 0U;
    s_rx_line_overflow = 0U;
    s_raw_diagnostic_length = 0U;
    s_read_flash_active = 0U;
    (void)memset(&s_flash_config, 0, sizeof(s_flash_config));
    (void)memset(s_rx_payload, 0, sizeof(s_rx_payload));
    (void)memset(s_rx_line, 0, sizeof(s_rx_line));
    (void)memset(s_raw_diagnostic, 0, sizeof(s_raw_diagnostic));
}

bool MB_Protocol_Poll(mb_protocol_frame_t *frame)
{
    uint8_t byte;

    if (frame == NULL) {
        return false;
    }
    /* Keep the register-level USART6 receiver armed once per poll cycle. */
    (void)MB_Transport_Ensure_Rx_Active();
    while (MB_Transport_ReadByte(&byte)) {
        if (byte == '$') {
            if (s_rx_state == MB_RX_IN_LINE &&
                protocol_emit_text_line(frame)) {
                s_rx_state = MB_RX_IN_FRAME;
                s_rx_length = 0U;
                s_rx_overflow = 0U;
                return true;
            }
            if (s_rx_state == MB_RX_WAIT_START &&
                s_raw_diagnostic_length != 0U) {
                s_rx_state = MB_RX_IN_FRAME;
                s_rx_length = 0U;
                s_rx_overflow = 0U;
                (void)protocol_emit_raw_diagnostic(frame);
                return true;
            }
            s_rx_state = MB_RX_IN_FRAME;
            s_rx_length = 0U;
            s_rx_overflow = 0U;
            continue;
        }
        if (s_rx_state == MB_RX_IN_FRAME) {
            if (byte == '#') {
                mb_frame_type_t type;

                (void)memset(frame, 0, sizeof(*frame));
                frame->raw[0] = '$';
                if (s_rx_length != 0U) {
                    (void)memcpy(&frame->raw[1], s_rx_payload, s_rx_length);
                }
                frame->raw[s_rx_length + 1U] = '#';
                frame->raw[s_rx_length + 2U] = '\0';
                s_rx_state = MB_RX_WAIT_START;
                if (s_rx_overflow != 0U) {
                    type = MB_FRAME_INVALID;
                    ++s_stats.overflow_frames;
                } else {
                    s_rx_payload[s_rx_length] = '\0';
                    type = decode_payload(s_rx_payload, frame, false);
                }
                frame->type = type;
                ++s_stats.frames;
                if (type == MB_FRAME_INVALID) {
                    ++s_stats.invalid_frames;
                }
                return true;
            }
            if (s_rx_overflow == 0U) {
                if (s_rx_length < (uint16_t)(sizeof(s_rx_payload) - 1U)) {
                    s_rx_payload[s_rx_length++] = (char)byte;
                } else {
                    s_rx_overflow = 1U;
                }
            }
            continue;
        }

        if (s_rx_state == MB_RX_IN_LINE) {
            if (byte == '\r' || byte == '\n') {
                if (protocol_emit_text_line(frame)) {
                    s_rx_state = MB_RX_WAIT_START;
                    return true;
                }
                s_rx_state = MB_RX_WAIT_START;
                continue;
            }
            if (s_rx_line_overflow == 0U) {
                if (s_rx_line_length <
                    (uint16_t)(sizeof(s_rx_line) - 1U)) {
                    s_rx_line[s_rx_line_length++] = (char)byte;
                } else {
                    s_rx_line_overflow = 1U;
                }
            }
            continue;
        }

        if (byte == '\r' || byte == '\n') {
            if (protocol_emit_text_line(frame)) {
                s_rx_state = MB_RX_WAIT_START;
                return true;
            }
            continue;
        }
        if (protocol_is_text_byte(byte)) {
            s_rx_state = MB_RX_IN_LINE;
            if (s_rx_line_length < (uint16_t)(sizeof(s_rx_line) - 1U)) {
                s_rx_line[s_rx_line_length++] = (char)byte;
            } else {
                s_rx_line_overflow = 1U;
            }
            continue;
        }
        if (s_raw_diagnostic_length < sizeof(s_raw_diagnostic)) {
            s_raw_diagnostic[s_raw_diagnostic_length++] = byte;
            if (s_raw_diagnostic_length == sizeof(s_raw_diagnostic) &&
                protocol_emit_raw_diagnostic(frame)) {
                return true;
            }
        }
    }
    if (protocol_emit_raw_diagnostic(frame)) {
        return true;
    }
    return false;
}

bool MB_Protocol_SendPwm(int16_t m1, int16_t m2, int16_t m3, int16_t m4)
{
    return MB_Protocol_SendFour("pwm", m1, m2, m3, m4);
}

bool MB_Protocol_SendSpeed(int16_t m1, int16_t m2, int16_t m3, int16_t m4)
{
    return MB_Protocol_SendFour("spd", m1, m2, m3, m4);
}

bool MB_Protocol_SendMotorType(uint8_t motor_type)
{
    if (motor_type == 0U || motor_type > 5U) {
        return false;
    }
    return MB_Protocol_SendUnsigned("mtype", motor_type);
}

bool MB_Protocol_SendDeadzone(uint16_t dead_zone)
{
    return dead_zone <= 3600U && MB_Protocol_SendUnsigned("deadzone", dead_zone);
}

bool MB_Protocol_SendMagneticLine(uint16_t magnetic_line_count)
{
    return magnetic_line_count != 0U &&
           MB_Protocol_SendUnsigned("mline", magnetic_line_count);
}

bool MB_Protocol_SendGearRatio(uint16_t gear_ratio)
{
    return gear_ratio != 0U &&
           MB_Protocol_SendUnsigned("mphase", gear_ratio);
}

bool MB_Protocol_SendWheelDiameter(uint16_t diameter_mm)
{
    return diameter_mm != 0U &&
           MB_Protocol_SendUnsigned("wdiameter", diameter_mm);
}

bool MB_Protocol_SendReadVoltage(void)
{
    return MB_Protocol_SendText("$read_vol#");
}

bool MB_Protocol_SendReadFlash(void)
{
    const bool queued = MB_Protocol_SendText("$read_flash#");

    if (queued) {
        (void)memset(&s_flash_config, 0, sizeof(s_flash_config));
        s_read_flash_active = 1U;
    }
    return queued;
}

void MB_Protocol_EndReadFlash(void)
{
    s_read_flash_active = 0U;
    s_flash_config.complete =
        !s_flash_config.invalid &&
        (s_flash_config.fields_present & MB_FLASH_FIELD_REQUIRED_MASK) ==
            MB_FLASH_FIELD_REQUIRED_MASK;
}

bool MB_Protocol_GetFlashConfig(mb_flash_config_t *config)
{
    if (config == NULL) {
        return false;
    }
    *config = s_flash_config;
    config->complete =
        !config->invalid &&
        (config->fields_present & MB_FLASH_FIELD_REQUIRED_MASK) ==
            MB_FLASH_FIELD_REQUIRED_MASK;
    return config->complete;
}

bool MB_Protocol_SendUpload(bool all_encoder, bool ten_ms_encoder, bool speed)
{
    char text[MB_PROTOCOL_TX_TEXT_CAPACITY];
    int written;

    written = snprintf(text, sizeof(text), "$upload:%u,%u,%u#",
                       all_encoder ? 1U : 0U, ten_ms_encoder ? 1U : 0U,
                       speed ? 1U : 0U);
    return written > 0 && (size_t)written < sizeof(text) &&
           MB_Protocol_SendText(text);
}

void MB_Protocol_GetStats(mb_protocol_stats_t *stats)
{
    if (stats != NULL) {
        *stats = s_stats;
    }
}
