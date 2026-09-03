#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "srp_codec.h"
#include "srp_crc.h"
#include "srp_registry.h"
#include "srp_wire.h"

/* SRP 编解码主机测试；创建人：待确认（当前维护人：Zhiqin）。 */

static uint32_t s_frames;
static uint8_t s_last_type;
static uint8_t s_last_sequence;
static uint32_t s_errors;
static srp_parser_error_t s_last_error;

/**
 * @brief 记录 parser 成功交付帧的次数、类型和序号。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param frame parser 借用的逻辑帧；必须非 NULL，仅在回调期间有效。
 * @param context parser 注册上下文，当前忽略，允许 NULL。
 * @return 返回值：无（void）；frame 为 NULL 时 assert 终止测试进程。
 * 调用方式：由 srp_parser_feed() 在测试主线程中同步调用，用全局快照供随后断言。
 * 线程约束：仅用于单线程主机测试；修改无锁全局计数，不可并发或重入，不保留 frame/payload 指针。
 */
static void on_frame(const srp_frame_t *frame, void *context)
{
    (void)context;
    assert(frame != NULL);
    ++s_frames;
    s_last_type = frame->type;
    s_last_sequence = frame->sequence;
}

/**
 * @brief 记录 parser 最近一次错误类型和累计错误次数。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param error parser 报告的错误枚举。
 * @param data 错误候选字节的借用指针，当前测试不读取，允许 NULL。
 * @param length data 的候选长度，当前测试不使用。
 * @param context parser 注册上下文，当前忽略，允许 NULL。
 * @return 返回值：无（void）。
 * 调用方式：由 srp_parser_feed() 在测试主线程同步调用，供错误类型断言读取全局快照。
 * 线程约束：单线程主机测试专用；无锁修改全局状态，不保留 data/context，也不适用于 ISR/RTOS。
 */
static void on_error(srp_parser_error_t error, const uint8_t *data,
                     size_t length, void *context)
{
    (void)data;
    (void)length;
    (void)context;
    s_last_error = error;
    ++s_errors;
}

/**
 * @brief 验证 CRC16-CCITT-FALSE 标准字符串向量得到 0x29B1。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return 返回值：无（void）；结果不符时 assert 终止测试进程。
 * 调用方式：由 main() 顺序调用，作为 SRP CRC 实现的标准黄金向量检查。
 * 线程约束：单线程纯主机计算，不访问 RTOS、UART 或硬件；静态向量只读且不转移所有权。
 */
static void test_crc_vector(void)
{
    static const uint8_t vector[] = "123456789";

    assert(srp_crc16_ccitt_false(vector, sizeof(vector) - 1U) == UINT16_C(0x29B1));
}

/**
 * @brief 验证一条 MOTOR_CMD 的编码字段、线长、EOF 及解码往返一致性。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return 返回值：无（void）；任一编解码或字段断言失败时终止测试进程。
 * 调用方式：由 main() 顺序调用；解码 payload 仅借用本函数栈上 bytes，断言在其生命周期内完成。
 * 线程约束：单线程主机测试，使用栈缓冲且不访问外设；不可把通过结果解释为 UART 传输成功。
 */
static void test_encode_decode(void)
{
    const uint8_t payload[] = {0x10U, 0x20U, 0x30U, 0x40U};
    uint8_t bytes[SRP_MAX_FRAME_SIZE] = {0};
    uint16_t length = 0U;
    srp_frame_t input = {
        .priority = SRP_PRIORITY_COMMAND,
        .type = SRP_MSG_ID_MOTOR_CMD,
        .sequence = 0xFFU,
        .flags = 0U,
        .length = sizeof(payload),
        .payload = payload,
    };
    srp_frame_t output = {0};

    assert(srp_encode(&input, bytes, sizeof(bytes), &length) == SRP_CODEC_OK);
    assert(length == 16U);
    assert(bytes[0] == 0xAAU && bytes[1] == 0x55U);
    assert(bytes[2] == 0x04U && bytes[3] == 0x00U);
    assert(bytes[4] == 0x00U && bytes[5] == 0xFFU);
    assert(bytes[6] == SRP_MSG_ID_MOTOR_CMD && bytes[7] == 0x01U);
    assert(bytes[14U] == SRP_EOF_BYTE0 && bytes[15U] == SRP_EOF_BYTE1);
    assert(srp_decode(bytes, length, &output) == SRP_CODEC_OK);
    assert(output.priority == input.priority);
    assert(output.type == input.type);
    assert(output.sequence == input.sequence);
    assert(output.length == input.length);
    assert(memcmp(output.payload, payload, sizeof(payload)) == 0);
}

/**
 * @brief 验证 parser 支持逐字节分片和随后整帧拼接输入，并保持回调顺序。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return 返回值：无（void）；消费长度、回调次数、类型或错误计数不符时 assert 终止。
 * 调用方式：由 main() 调用；先逐字节喂首帧，再一次喂第二帧，回调通过全局快照验证。
 * 线程约束：parser 和全局回调状态由测试主线程独占；所有输入缓冲均为栈上副本，不涉及 DMA/ISR。
 */
static void test_parser_fragmented_and_concatenated(void)
{
    const uint8_t first_payload[] = {1U, 2U, 3U};
    const uint8_t second_payload[] = {4U, 5U};
    uint8_t first[SRP_MAX_FRAME_SIZE] = {0};
    uint8_t second[SRP_MAX_FRAME_SIZE] = {0};
    uint8_t combined[SRP_MAX_FRAME_SIZE * 2U] = {0};
    uint16_t first_length = 0U;
    uint16_t second_length = 0U;
    srp_frame_t first_frame = {.priority = SRP_PRIORITY_TELEMETRY,
                               .type = SRP_MSG_ID_IMU_TELEMETRY,
                               .sequence = 0U,
                               .flags = 0U,
                               .length = sizeof(first_payload),
                               .payload = first_payload};
    srp_frame_t second_frame = {.priority = SRP_PRIORITY_LOG,
                                .type = SRP_MSG_ID_LOG,
                                .sequence = 1U,
                                .flags = 0U,
                                .length = sizeof(second_payload),
                                .payload = second_payload};
    srp_parser_t parser;

    assert(srp_encode(&first_frame, first, sizeof(first), &first_length) == 0);
    assert(srp_encode(&second_frame, second, sizeof(second), &second_length) == 0);
    memcpy(combined, first, first_length);
    memcpy(combined + first_length, second, second_length);
    s_frames = 0U;
    s_errors = 0U;
    srp_parser_init(&parser, on_frame, on_error, NULL);
    for (uint16_t index = 0U; index < first_length; ++index) {
        assert(srp_parser_feed(&parser, &combined[index], 1U) == 1U);
    }
    assert(s_frames == 1U && s_last_type == SRP_MSG_ID_IMU_TELEMETRY);
    assert(srp_parser_feed(&parser, combined + first_length, second_length) ==
           second_length);
    assert(s_frames == 2U && s_last_type == SRP_MSG_ID_LOG && s_errors == 0U);
}

/**
 * @brief 校验同步请求与快速 ACK 的固定线缆字节，并验证 CRC 损坏触发 parser 错误。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return 返回值：无（void）；黄金帧、CRC 字节或错误回调断言失败时终止进程。
 * 调用方式：由 main() 调用；编码后与常量逐字节比较，再翻转 CRC 字节喂给 parser。
 * 线程约束：单线程主机测试；常量只读、bytes/parser 为栈所有，不证明跨芯片握手已运行。
 */
static void test_sync_and_ack_golden_frames(void)
{
    static const uint8_t sync_payload[] = {4U, 0U, 0U, 0U};
    static const uint8_t ack_payload[] = {SRP_MSG_ID_CMD_SYNC_REQ, 0U,
                                          0x2AU, SRP_FAST_RESP_OK};
    static const uint8_t expected_sync[] = {
        0xAAU, 0x55U, 0x04U, 0x00U, 0x00U, 0x2AU, 0x08U, 0x01U,
        0x04U, 0x00U, 0x00U, 0x00U, 0x56U, 0xBCU, 0x0DU, 0x0AU
    };
    static const uint8_t expected_ack[] = {
        0xAAU, 0x55U, 0x04U, 0x00U, 0x04U, 0x2BU, 0x7EU, 0x01U,
        0x08U, 0x00U, 0x2AU, 0x00U, 0x38U, 0x65U, 0x0DU, 0x0AU
    };
    uint8_t bytes[SRP_MAX_FRAME_SIZE] = {0};
    uint16_t length = 0U;
    srp_frame_t frame = {.priority = SRP_PRIORITY_COMMAND,
                         .type = SRP_MSG_ID_CMD_SYNC_REQ,
                         .sequence = 0x2AU,
                         .flags = SRP_FLAG_STREAM_DATA,
                         .length = sizeof(sync_payload),
                         .payload = sync_payload};
    srp_parser_t parser;

    assert(srp_encode(&frame, bytes, sizeof(bytes), &length) == SRP_CODEC_OK);
    assert(length == sizeof(expected_sync));
    assert(memcmp(bytes, expected_sync, sizeof(expected_sync)) == 0);
    assert(bytes[12] == 0x56U && bytes[13] == 0xBCU);
    assert(srp_decode(bytes, length, &frame) == SRP_CODEC_OK);

    frame.type = SRP_MSG_ID_ACK;
    frame.sequence = 0x2BU;
    frame.flags = SRP_FLAG_ACK;
    frame.payload = ack_payload;
    assert(srp_encode(&frame, bytes, sizeof(bytes), &length) == SRP_CODEC_OK);
    assert(length == sizeof(expected_ack));
    assert(memcmp(bytes, expected_ack, sizeof(expected_ack)) == 0);
    assert(bytes[12] == 0x38U && bytes[13] == 0x65U);
    assert(srp_decode(bytes, length, &frame) == SRP_CODEC_OK);

    bytes[12] ^= 0x01U;
    s_errors = 0U;
    s_last_error = 0;
    srp_parser_init(&parser, on_frame, on_error, NULL);
    (void)srp_parser_feed(&parser, bytes, sizeof(expected_ack));
    assert(s_errors == 1U && s_last_error == SRP_PARSER_ERROR_CRC);
}

/**
 * @brief 验证启动阶段 sequence=0 的 CMD_SYNC_REQ 固定 SRP v4 黄金帧。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return 返回值：无（void）；编码状态、长度或字节不符时 assert 终止测试进程。
 * 调用方式：由 main() 顺序调用，使用固定版本 payload 和固定序号生成帧。
 * 线程约束：单线程主机纯编码测试，不访问链路状态机、UART 或设备；缓冲由本函数栈拥有。
 */
static void test_startup_sync_golden_frame(void)
{
    static const uint8_t payload[] = {4U, 0U, 0U, 0U};
    static const uint8_t expected[] = {
        0xAAU, 0x55U, 0x04U, 0x00U, 0x00U, 0x00U, 0x08U, 0x01U,
        0x04U, 0x00U, 0x00U, 0x00U, 0xEEU, 0x21U, 0x0DU, 0x0AU
    };
    uint8_t bytes[SRP_MAX_FRAME_SIZE] = {0};
    uint16_t length = 0U;
    const srp_frame_t frame = {
        .priority = SRP_PRIORITY_COMMAND,
        .type = SRP_MSG_ID_CMD_SYNC_REQ,
        .sequence = 0U,
        .flags = SRP_FLAG_STREAM_DATA,
        .length = sizeof(payload),
        .payload = payload,
    };

    assert(srp_encode_frame(&frame, bytes, sizeof(bytes), &length) == SRP_CODEC_OK);
    assert(length == sizeof(expected));
    assert(memcmp(bytes, expected, sizeof(expected)) == 0);
}

/**
 * @brief 验证 CHASSIS_HEADING_CMD 的 12 字节 payload、float 小端值、标志和 ACK_REQUIRED 往返。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return 返回值：无（void）；结构尺寸、编解码或字段断言失败时终止测试进程。
 * 调用方式：由 main() 调用；decoded.payload 借用 bytes，仅在本函数返回前读取。
 * 线程约束：单线程主机测试，验证线缆布局而非 STM 控制准入或车辆航向响应。
 */
static void test_chassis_heading_command_payload(void)
{
    uint8_t payload[SRP_PAYLOAD_CHASSIS_HEADING_CMD_SIZE] = {0};
    uint8_t bytes[SRP_MAX_FRAME_SIZE] = {0};
    uint16_t length = 0U;
    srp_frame_t decoded = {0};
    const srp_frame_t frame = {
        .priority = SRP_PRIORITY_COMMAND,
        .type = SRP_MSG_ID_CHASSIS_HEADING_CMD,
        .sequence = 0x31U,
        .flags = SRP_FLAG_ACK_REQUIRED,
        .length = sizeof(payload),
        .payload = payload,
    };

    srp_wire_write_f32_le(&payload[0], 320.0f);
    srp_wire_write_f32_le(&payload[4], -179.5f);
    srp_wire_write_u32_le(&payload[8], SRP_CHASSIS_HEADING_FLAGS_NONE);
    assert(sizeof(payload) == sizeof(srp_chassis_heading_cmd_payload_t));
    assert(srp_encode(&frame, bytes, sizeof(bytes), &length) == SRP_CODEC_OK);
    assert(srp_decode(bytes, length, &decoded) == SRP_CODEC_OK);
    assert(decoded.type == SRP_MSG_ID_CHASSIS_HEADING_CMD);
    assert(decoded.flags == SRP_FLAG_ACK_REQUIRED);
    assert(decoded.length == SRP_PAYLOAD_CHASSIS_HEADING_CMD_SIZE);
    assert(srp_wire_read_f32_le(&decoded.payload[0]) == 320.0f);
    assert(srp_wire_read_f32_le(&decoded.payload[4]) == -179.5f);
    assert(srp_wire_read_u32_le(&decoded.payload[8]) ==
           SRP_CHASSIS_HEADING_FLAGS_NONE);
}

/**
 * @brief 验证 CHASSIS_STATE schema、有效位、时间戳、位姿/距离字段及完整黄金帧。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return 返回值：无（void）；黄金字节或任一解码字段不符时 assert 终止测试进程。
 * 调用方式：由 main() 调用；先按小端写 payload，再编码、逐字节比较并解码复验。
 * 线程约束：单线程主机测试；局部 payload/bytes 生命周期覆盖 decoded 借用视图，不验证实时里程计来源。
 */
static void test_chassis_state_golden_frame(void)
{
    static const uint8_t expected[] = {
        0xAAU, 0x55U, 0x18U, 0x00U, 0x00U, 0x2AU, 0x15U, 0x02U,
        0x01U, 0x04U, 0x00U, 0x00U, 0xE8U, 0x03U, 0x00U, 0x00U,
        0x00U, 0x00U, 0x7AU, 0x44U, 0x00U, 0x00U, 0xFAU, 0xC3U,
        0x00U, 0x00U, 0x33U, 0x43U, 0x00U, 0x00U, 0x48U, 0x41U,
        0x7FU, 0xC0U, 0x0DU, 0x0AU
    };
    uint8_t payload[SRP_PAYLOAD_CHASSIS_STATE_SIZE] = {0};
    uint8_t bytes[SRP_MAX_FRAME_SIZE] = {0};
    uint16_t length = 0U;
    srp_frame_t decoded = {0};
    const srp_frame_t frame = {
        .priority = SRP_PRIORITY_TELEMETRY,
        .type = SRP_MSG_ID_CHASSIS_STATE,
        .sequence = 0x2AU,
        .flags = SRP_FLAG_STREAM_DATA,
        .length = sizeof(payload),
        .payload = payload,
    };

    payload[0] = SRP_CHASSIS_STATE_SCHEMA;
    payload[1] = SRP_CHASSIS_STATE_FLAG_ODOMETRY_VALID;
    srp_wire_write_u32_le(&payload[4], UINT32_C(1000));
    srp_wire_write_f32_le(&payload[8], 1000.0f);
    srp_wire_write_f32_le(&payload[12], -500.0f);
    srp_wire_write_f32_le(&payload[16], 179.0f);
    srp_wire_write_f32_le(&payload[20], 12.5f);

    assert(srp_encode_frame(&frame, bytes, sizeof(bytes), &length) ==
           SRP_CODEC_OK);
    assert(length == sizeof(expected));
    assert(memcmp(bytes, expected, sizeof(expected)) == 0);
    assert(srp_decode(bytes, length, &decoded) == SRP_CODEC_OK);
    assert(decoded.type == SRP_MSG_ID_CHASSIS_STATE);
    assert(decoded.length == SRP_PAYLOAD_CHASSIS_STATE_SIZE);
    assert(decoded.payload[1] == SRP_CHASSIS_STATE_FLAG_ODOMETRY_VALID);
    assert(srp_wire_read_u32_le(&decoded.payload[4]) == UINT32_C(1000));
    assert(srp_wire_read_f32_le(&decoded.payload[8]) == 1000.0f);
    assert(srp_wire_read_f32_le(&decoded.payload[12]) == -500.0f);
    assert(srp_wire_read_f32_le(&decoded.payload[16]) == 179.0f);
    assert(srp_wire_read_f32_le(&decoded.payload[20]) == 12.5f);
}

/**
 * @brief 验证独立 CRC 损坏和 EOF 损坏均被拒绝，且 parser 对坏 EOF 上报一次错误。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return 返回值：无（void）；错误码或回调次数不符时 assert 终止测试进程。
 * 调用方式：由 main() 调用；每种损坏都从重新编码的有效帧开始，避免前一修改污染下一场景。
 * 线程约束：单线程主机测试，原地修改栈上 bytes；不覆盖串口丢字节、DMA 或并发 feed。
 */
static void test_crc_and_eof_rejection(void)
{
    const uint8_t payload[] = {0xABU};
    uint8_t bytes[SRP_MAX_FRAME_SIZE] = {0};
    uint16_t length = 0U;
    srp_frame_t frame = {.priority = SRP_PRIORITY_COMMAND,
                         .type = SRP_MSG_ID_MOTOR_CMD,
                         .sequence = 0U,
                         .flags = 0U,
                         .length = sizeof(payload),
                         .payload = payload};
    srp_parser_t parser;

    assert(srp_encode(&frame, bytes, sizeof(bytes), &length) == 0);
    bytes[8] ^= 0x01U;
    assert(srp_decode(bytes, length, &frame) == SRP_CODEC_BAD_CRC);
    assert(srp_encode(&frame, bytes, sizeof(bytes), &length) == 0);
    bytes[length - 1U] = 0x00U;
    assert(srp_decode(bytes, length, &frame) == SRP_CODEC_BAD_EOF);
    s_errors = 0U;
    srp_parser_init(&parser, on_frame, on_error, NULL);
    (void)srp_parser_feed(&parser, bytes, length);
    assert(s_errors == 1U);
}

/**
 * @brief 验证保留 flag 被识别为 HEADER 错误，并保留 parser 状态与丢弃字节诊断。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return 返回值：无（void）；错误类型、状态或 last_drop_byte 不符时 assert 终止。
 * 调用方式：由 main() 调用；编码有效帧后仅修改头 flag，再通过 parser 回调和内部诊断字段复验。
 * 线程约束：单线程白盒主机测试；直接读取 parser 结构，不代表现场 UART 错误恢复已验证。
 */
static void test_header_rejection_diagnostics(void)
{
    const uint8_t payload[] = {0x01U};
    uint8_t bytes[SRP_MAX_FRAME_SIZE] = {0};
    uint16_t length = 0U;
    srp_frame_t frame = {.priority = SRP_PRIORITY_COMMAND,
                         .type = SRP_MSG_ID_MOTOR_CMD,
                         .sequence = 0U,
                         .flags = 0U,
                         .length = sizeof(payload),
                         .payload = payload};
    srp_parser_t parser;

    assert(srp_encode(&frame, bytes, sizeof(bytes), &length) == SRP_CODEC_OK);
    bytes[7] |= SRP_FLAG_RESERVED_MASK;
    s_errors = 0U;
    s_last_error = 0;
    srp_parser_init(&parser, on_frame, on_error, NULL);
    (void)srp_parser_feed(&parser, bytes, length);
    assert(s_errors == 1U && s_last_error == SRP_PARSER_ERROR_HEADER);
    assert(parser.last_error_state == SRP_PARSER_READ_HEADER);
    assert(parser.last_drop_byte == bytes[7]);
}

/**
 * @brief 验证接收不连续时 reset 丢弃半帧，后续完整帧仍可正常解析。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return 返回值：无（void）；尾半帧误成帧或完整重发未回调时 assert 终止。
 * 调用方式：由 main() 调用；先喂 4 字节、reset、喂残余，再从头喂完整帧。
 * 线程约束：parser 与全局回调计数由测试主线程独占；只模拟软件断点，不模拟实际 UART/DMA 时序。
 */
static void test_parser_reset_after_discontinuity(void)
{
    const uint8_t payload[] = {0x01U, 0x02U};
    uint8_t bytes[SRP_MAX_FRAME_SIZE] = {0};
    uint16_t length = 0U;
    srp_frame_t frame = {.priority = SRP_PRIORITY_COMMAND,
                         .type = SRP_MSG_ID_RSP_BOOT_INFO,
                         .sequence = 2U,
                         .flags = SRP_FLAG_STREAM_DATA,
                         .length = sizeof(payload),
                         .payload = payload};
    srp_parser_t parser;

    assert(srp_encode(&frame, bytes, sizeof(bytes), &length) == SRP_CODEC_OK);
    s_frames = 0U;
    srp_parser_init(&parser, on_frame, on_error, NULL);
    (void)srp_parser_feed(&parser, bytes, 4U);
    srp_parser_reset(&parser);
    (void)srp_parser_feed(&parser, bytes + 4U, length - 4U);
    assert(s_frames == 0U);
    (void)srp_parser_feed(&parser, bytes, length);
    assert(s_frames == 1U && s_last_type == SRP_MSG_ID_RSP_BOOT_INFO);
}

/**
 * @brief 验证 TLV 迭代器可顺序读取已知波特率项、跳过未知项并在末尾停止。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return 返回值：无（void）；tag、长度、值或终止语义不符时 assert 终止。
 * 调用方式：由 main() 调用；value 是 data 内部借用指针，仅在本函数栈生命周期内读取。
 * 线程约束：单线程纯内存测试，不涉及服务层白名单、波特率切换或 UART 硬件。
 */
static void test_unknown_tlv_is_skipped(void)
{
    const uint8_t data[] = {SRP_TLV_TAG_BAUDRATE, 4U, 0x00U, 0x10U, 0x0EU, 0x00U,
                            0xF0U, 1U, 0xAAU};
    srp_tlv_iter_t iterator;
    uint8_t tag = 0U;
    uint8_t length = 0U;
    const uint8_t *value = NULL;

    srp_tlv_iter_init(&iterator, data, sizeof(data));
    assert(srp_tlv_next(&iterator, &tag, &length, &value));
    assert(tag == SRP_TLV_TAG_BAUDRATE && length == 4U && value[0] == 0x00U);
    assert(srp_tlv_next(&iterator, &tag, &length, &value));
    assert(tag == 0xF0U && length == 1U && value[0] == 0xAAU);
    assert(!srp_tlv_next(&iterator, &tag, &length, &value));
}

/**
 * @brief 验证关键 SRP 结构对齐/偏移约束及序号 0xFF 到 0x00 的宏提取边界。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return 返回值：无（void）；编译期布局在本平台不符或宏结果错误时 assert 终止。
 * 调用方式：由 main() 最后调用，检查当前 host ABI 下的结构布局与纯宏结果。
 * 线程约束：单线程、无可变共享状态；host ABI 通过不自动证明所有交叉编译器布局一致。
 */
static void test_alignment_and_sequence_wrap(void)
{
    assert(_Alignof(srp_wire_header_t) >= 4U);
    assert(offsetof(srp_wire_header_t, header) == 4U);
    assert(_Alignof(srp_parser_t) >= 4U);
    assert(SRP_HDR_SEQ(SRP_HDR_MAKE(0U, 1U, 0xFFU, 0U)) == 0xFFU);
    assert(SRP_HDR_SEQ(SRP_HDR_MAKE(0U, 1U, 0x00U, 0U)) == 0x00U);
}

/**
 * @brief 顺序执行 SRP codec/parser/wire/TLV 主机断言集合。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return 全部断言通过返回 0；任一 assert 失败会提前终止进程而不会返回错误码。
 * 调用方式：编译为独立 host executable 后直接运行，无命令行参数和外部夹具。
 * 线程约束：单进程单线程执行；不创建 RTOS 任务、不访问 UART/BLE/硬件，静态回调状态仅供本进程使用。
 */
int main(void)
{
    test_crc_vector();
    test_encode_decode();
    test_parser_fragmented_and_concatenated();
    test_sync_and_ack_golden_frames();
    test_startup_sync_golden_frame();
    test_chassis_heading_command_payload();
    test_chassis_state_golden_frame();
    test_crc_and_eof_rejection();
    test_header_rejection_diagnostics();
    test_parser_reset_after_discontinuity();
    test_unknown_tlv_is_skipped();
    test_alignment_and_sequence_wrap();
    return 0;
}
