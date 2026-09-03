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
/* MotorBoard 文本协议实现；创建人：待确认（当前维护人：Zhiqin）。 */

#define MB_PROTOCOL_PAYLOAD_CAPACITY \
    (MB_PROTOCOL_MAX_FRAME_LEN - UINT16_C(3))
#define MB_PROTOCOL_LINE_CAPACITY MB_PROTOCOL_MAX_FRAME_LEN
#define MB_PROTOCOL_TX_TEXT_CAPACITY UINT16_C(64)
/* 达到该长度会立即产生 MB_FRAME_FLASH_RAW；它属于解析契约，不是日志容量。 */
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
static mb_protocol_stats_t s_stats;

/**
 * @brief 把一个 ASCII/单字节字符按当前 C locale 转为小写。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param value 输入字符；内部先转 unsigned char，避免 ctype 负值未定义行为。
 * @return tolower() 结果截断为 char。
 * 调用方式：MotorBoard 文本关键字大小写无关比较逐字符调用。
 * 线程约束：不访问模块可变状态；普通任务解析上下文调用，不在 ISR 中运行 libc ctype。
 */
static char ascii_lower(char value)
{
    return (char)tolower((unsigned char)value);
}

/**
 * @brief 判断零结尾文本是否以指定前缀开头，比较时忽略 ASCII 大小写。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param text 只读零结尾文本；允许 NULL，NULL 返回 false。
 * @param prefix 只读零结尾前缀；允许 NULL，空前缀返回 true。
 * @return 完整前缀匹配时 true，否则 false。
 * 调用方式：decode_payload()/parse_battery() 识别 MTEP/MSPD/Battery 等类型前调用。
 * 线程约束：纯只读扫描、可重入；普通任务上下文，不保留输入指针。
 */
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

/**
 * @brief 在文本中查找一个非空、大小写无关的连续子串。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param text 只读零结尾待查文本；允许 NULL。
 * @param needle 只读零结尾关键字；允许 NULL，空串被视为不匹配。
 * @return 找到完整子串时 true；参数无效、关键字过长或未找到时 false。
 * 调用方式：decode_payload() 按优先顺序识别 NACK/ERROR/OK/ACK 文本。
 * 线程约束：纯只读扫描、可重入；普通任务上下文，不保留输入指针。
 */
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

/**
 * @brief 判断字节是否可作为行模式文本的一部分。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param byte 接收字节。
 * @return TAB 或 ASCII 0x20..0x7E 返回 true，其他控制/二进制字节返回 false。
 * 调用方式：MB_Protocol_Poll() 在未进入 `$...#` 帧时选择文本行或 raw diagnostic 路径。
 * 线程约束：纯数值判断、可重入、不阻塞。
 */
static bool protocol_is_text_byte(uint8_t byte)
{
    return byte == '\t' || (byte >= 0x20U && byte <= 0x7EU);
}

/**
 * @brief 跳过零结尾文本开头由 isspace() 识别的空白字符。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param text 借用只读文本；允许 NULL。
 * @return 指向首个非空白/结尾字符的借用指针；输入 NULL 时返回 NULL。
 * 调用方式：数字解析前后调用以允许字段周围空白，不复制输入。
 * 线程约束：纯只读扫描、可重入；返回指针不得超过原文本生命周期。
 */
static const char *skip_spaces(const char *text)
{
    while (text != NULL && *text != '\0' &&
           isspace((unsigned char)*text) != 0) {
        ++text;
    }
    return text;
}

/**
 * @brief 严格解析四个逗号分隔的十进制 int32，允许字段周围空白。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param text 只读零结尾数值列表；不得为 NULL。
 * @param values 至少 4 个元素的可写数组；不得为 NULL。
 * @return 四项均在 int32 范围、分隔符正确且末尾无其他字符时 true；否则 false。
 * 调用方式：解码 MTEP/MAll payload 时调用；false 时 values 可能已部分写入，必须整体丢弃。
 * 线程约束：调用 strtol()/errno，不可重入依赖目标 libc 的 errno 实现；仅协议任务调用，禁止 ISR。
 */
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

/**
 * @brief 用 64 位中间值应用脉冲诊断极性并饱和回 int32。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param value MotorBoard 原始脉冲值。
 * @param sign 极性因子；当前固定表为 [1,-1,1,1]。
 * @return 相乘结果，超出 int32 时饱和到 INT32_MIN/MAX。
 * 调用方式：只对 MTEP 报告应用，不修改闭环速度极性或 PWM 顺序。
 * 线程约束：纯数值计算、可重入、不阻塞。
 */
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

/**
 * @brief 严格解析四个逗号分隔的有限 float，允许字段周围空白。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param text 只读零结尾数值列表；不得为 NULL。
 * @param values 至少 4 个元素的可写数组；不得为 NULL。
 * @return 四项均可解析、未 ERANGE、有限且末尾无其他字符时 true；否则 false。
 * 调用方式：解码 MSPD payload 时调用；false 时 values 可能已部分写入，必须整体丢弃。
 * 线程约束：调用 strtof()/errno；只在 MotorBoard 协议任务上下文使用，禁止 ISR。
 */
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

/**
 * @brief 解析 `Battery:` 前缀后的有限电压值和可选 V/v 单位。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param text 只读零结尾响应文本；允许 NULL。
 * @param voltage 可写输出；不得为 NULL，失败时保持原值。
 * @return 前缀、数值、可选单位和结尾全部合法时 true，否则 false。
 * 调用方式：decode_payload() 最先尝试该格式，成功后返回 MB_FRAME_BATTERY。
 * 线程约束：调用 strtof()/errno；仅协议任务调用，禁止 ISR，不保留 text 指针。
 */
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

/**
 * @brief 把二进制诊断片段格式化为有界 `hex=... ascii=...` 文本。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param destination 可写零结尾输出缓冲；不得为 NULL。
 * @param capacity 输出容量；为 0 时不写入。
 * @param bytes 只读二进制输入；不得为 NULL。
 * @param length 输入字节数；超出输出容量的尾部会被截断。
 * @return 无；格式化失败或容量不足时保留已生成的有界前缀。
 * 调用方式：protocol_emit_raw_diagnostic() 生成 MB_FRAME_INVALID 诊断文本时调用。
 * 线程约束：使用 snprintf()/isprint()，只在协议任务上下文调用，禁止 ISR。
 */
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

/**
 * @brief 把当前 raw diagnostic 缓冲消费为一条 MB_FRAME_INVALID 逻辑帧。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param frame 可写输出；不得为 NULL，失败时不修改。
 * @return 缓冲非空且已产出帧时 true；参数无效或当前无 raw 字节时 false。
 * 调用方式：raw 缓冲达到固定 24 字节、遇到 `$`，或 Poll 暂时排空 RX ring 时调用。
 * 线程约束：消费全局 raw 缓冲并更新统计，无内部锁；仅 parser owner 调用，禁止 ISR。
 */
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

/**
 * @brief 把当前行缓冲解码并消费为一条 MotorBoard 逻辑帧。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param frame 可写输出；不得为 NULL，失败时不修改。
 * @return 行非空或曾溢出且已产出帧时 true；当前没有行数据时 false。
 * 调用方式：Poll 遇 CR/LF、从行切换到 `$` 帧或等待态换行时调用；read_flash 行可产出 FLASH_RAW。
 * 线程约束：消费/清零全局行状态并更新统计，只允许 parser owner 调用。
 */
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

/**
 * @brief 把响应文本有界复制到 frame->response_status。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param frame 可写逻辑帧；允许 NULL。
 * @param text 只读零结尾响应；允许 NULL，过长时由 snprintf 截断。
 * @return 无。
 * 调用方式：decode_payload() 识别 ACK/NACK/ERROR 文本后保存诊断原文。
 * 线程约束：只写调用方 frame；使用 snprintf，仅普通协议任务调用。
 */
static void protocol_set_response_status(mb_protocol_frame_t *frame,
                                         const char *text)
{
    if (frame == NULL || text == NULL) {
        return;
    }
    (void)snprintf(frame->response_status, sizeof(frame->response_status),
                   "%s", text);
}

/**
 * @brief 按 Battery/MTEP/MAll/MSPD/NACK/ACK 顺序解码一条零结尾 payload。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param payload 只读零结尾文本；调用方应保证非 NULL。
 * @param frame 可写逻辑帧；调用方应保证非 NULL，成功类型会填对应字段。
 * @param line_mode true 表示无 `$...#` 包络的文本行，可在 read_flash 模式产出 FLASH_RAW。
 * @return 识别出的 frame type；数值格式错误返回 INVALID，未识别返回 UNKNOWN。
 * 调用方式：完整 `$...#` 帧或文本行组帧后调用；收到 OK/ACK/Set 会清 read_flash_active。
 * 线程约束：修改 frame 和模块 read_flash 状态，无内部锁，仅 parser owner 调用。
 */
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
        s_read_flash_active = 0U;
        return MB_FRAME_OK_ACK;
    }
    if (line_mode && s_read_flash_active != 0U) {
        return MB_FRAME_FLASH_RAW;
    }
    return MB_FRAME_UNKNOWN;
}

/**
 * @brief 校验文本长度并把完整 MotorBoard 命令复制到 USART6 TX ring。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param text 只读零结尾完整命令；允许 NULL，长度必须为 1..63 字节。
 * @return true 仅表示完整文本已进入 TX ring；参数/长度/RX 健康/容量失败返回 false。
 * 调用方式：所有 MotorBoard 命令格式化 helper 的唯一发送出口。
 * 线程约束：调用 transport 任务 API 和短临界区，禁止 ISR；返回不代表物理发送或远端执行。
 */
static bool MB_Protocol_SendText(const char *text)
{
    const size_t length = text == NULL ? 0U : strlen(text);

    return length != 0U && length < MB_PROTOCOL_TX_TEXT_CAPACITY &&
           MB_Transport_Send((const uint8_t *)text, (uint16_t)length);
}

/**
 * @brief 格式化 `$command:m1,m2,m3,m4#` 并提交完整文本。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param command 内部固定命令名，必须为非 NULL 零结尾字符串。
 * @param m1/m2/m3/m4 四路有符号值，顺序由调用者固定为 RR/RF/LR/LF。
 * @return 格式化未截断且文本成功排队时 true，否则 false。
 * 调用方式：PWM 和 speed 公共包装调用；不会校验各值的业务范围。
 * 线程约束：使用栈上 64 字节缓冲和 snprintf，普通任务调用，禁止 ISR。
 */
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

/**
 * @brief 格式化 `$command:value#` 无符号配置命令并提交完整文本。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param command 零结尾命令名；NULL 返回 false。
 * @param value 待格式化 uint16_t；业务非零/范围约束由公共包装先检查。
 * @return 格式化未截断且文本成功排队时 true，否则 false。
 * 调用方式：mtype/mline/mphase/wdiameter 公共包装调用。
 * 线程约束：使用栈上 64 字节缓冲和 snprintf，普通任务调用，禁止 ISR。
 */
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

/** 初始化文本接收状态和协议统计。 */
void MB_Protocol_Init(void)
{
    MB_Protocol_ResetRx();
    (void)memset(&s_stats, 0, sizeof(s_stats));
}

/** 丢弃半帧并重新寻找协议起始符。 */
void MB_Protocol_ResetRx(void)
{
    s_rx_state = MB_RX_WAIT_START;
    s_rx_length = 0U;
    s_rx_overflow = 0U;
    s_rx_line_length = 0U;
    s_rx_line_overflow = 0U;
    s_raw_diagnostic_length = 0U;
    s_read_flash_active = 0U;
    (void)memset(s_rx_payload, 0, sizeof(s_rx_payload));
    (void)memset(s_rx_line, 0, sizeof(s_rx_line));
    (void)memset(s_raw_diagnostic, 0, sizeof(s_raw_diagnostic));
}

/** 轮询字节并解析一条完整 MotorBoard 响应。 */
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
        s_read_flash_active = 1U;
    }
    return queued;
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

/** 复制解析统计快照。 */
void MB_Protocol_GetStats(mb_protocol_stats_t *stats)
{
    if (stats != NULL) {
        *stats = s_stats;
    }
}
