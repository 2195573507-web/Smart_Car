#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "radar_parser.h"
#include "radar_uplink_protocol.h"

/* S3RD 封装主机测试；创建人：待确认（当前维护人：Zhiqin）。 */

/**
 * @brief 将 uint16_t 按小端序写入合成雷达帧字段。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param data 至少含 2 个可写字节的调用方缓冲，不得为 NULL。
 * @param value 待编码的 16 位值。
 * @return 返回值：无（void）。
 * 调用方式：make_frame() 和最大帧构造路径在已知容量内写角度、样本和 checksum。
 * 线程约束：单线程纯内存写入、可重入；不检查边界，同一缓冲不得并发修改。
 */
static void write_le16(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8U);
}

/**
 * @brief 从测试包连续两字节读取小端 uint16_t。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param data 至少含 2 个可读字节的借用指针，不得为 NULL。
 * @return 解码后的 16 位值。
 * 调用方式：合成 checksum 和 S3RD flags 断言在固定字段边界内调用。
 * 线程约束：单线程纯读取、可重入、不保留 data；函数本身不做容量校验。
 */
static uint16_t read_le16(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8U);
}

/**
 * @brief 构造一条含两个 2 字节样本和合法官方 XOR 的合成 YDLIDAR 帧。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param[out] frame 至少可写 14 字节的缓冲，不得为 NULL。
 * @return 合成完整帧的字节数。
 * 调用方式：RAW_FRAME 编解码测试先在最大帧栈缓冲调用，再按返回长度封装。
 * 线程约束：单线程主机缓冲构造；调用方独占 frame，不访问真实雷达或 UART。
 */
static size_t make_frame(uint8_t *frame)
{
    const size_t length = RADAR_X3PRO_HEADER_BYTES + 2U * RADAR_X3PRO_SAMPLE_BYTES;
    memset(frame, 0, length);
    frame[0] = RADAR_X3PRO_HEADER_BYTE_0;
    frame[1] = RADAR_X3PRO_HEADER_BYTE_1;
    frame[2] = 0U;
    frame[3] = 2U;
    write_le16(&frame[4], 0xAE53U);
    write_le16(&frame[6], 0xB553U);
    write_le16(&frame[10], 0x0400U);
    write_le16(&frame[12], 0x0800U);

    uint16_t checksum = 0x55AAU;
    checksum ^= read_le16(&frame[4]);
    checksum ^= read_le16(&frame[10]);
    checksum ^= read_le16(&frame[12]);
    checksum ^= (uint16_t)(((uint16_t)frame[3] << 8U) | frame[2]);
    checksum ^= read_le16(&frame[6]);
    write_le16(&frame[8], checksum);
    return length;
}

/**
 * @brief 验证 RAW_FRAME 专用 S3RD 编解码、元数据和历史 type-1 黄金字节保持一致。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return 返回值：无（void）；状态、元数据、payload 或黄金包断言失败时 assert 终止。
 * 调用方式：由 main() 调用；decoded.payload 借用本函数 packet，比较在缓冲生命周期内完成。
 * 线程约束：单线程 host 编解码测试，不建立 TCP 连接，也不证明服务端/ROS2 已接收。
 */
static void test_round_trip(void)
{
    uint8_t frame[RADAR_PARSER_MAX_FRAME_SIZE];
    uint8_t packet[RADAR_UPLINK_MAX_PACKET_SIZE];
    size_t frame_length = make_frame(frame);
    size_t packet_length = 0U;
    radar_uplink_packet_t decoded = {0};

    assert(radar_uplink_encode_frame(frame,
                                     frame_length,
                                     0x10203040U,
                                     1U,
                                     42U,
                                     123456U,
                                     packet,
                                     sizeof(packet),
                                     &packet_length) == RADAR_UPLINK_OK);
    assert(packet_length == RADAR_UPLINK_HEADER_SIZE + frame_length +
                            RADAR_UPLINK_CRC_SIZE);
    assert(radar_uplink_decode_packet(packet, packet_length, &decoded) ==
           RADAR_UPLINK_OK);
    assert(decoded.version == RADAR_UPLINK_PROTOCOL_VERSION);
    assert(decoded.message_type == RADAR_UPLINK_MESSAGE_RAW_FRAME);
    assert(decoded.flags == 0U);
    assert(decoded.device_id == 0x10203040U);
    assert(decoded.stream_id == 1U);
    assert(decoded.sequence == 42U);
    assert(decoded.timestamp_ms == 123456U);
    assert(decoded.payload_length == frame_length);
    assert(memcmp(decoded.payload, frame, frame_length) == 0);

    /* Golden bytes from the original type-1 encoder must remain unchanged. */
    static const uint8_t expected_packet[] = {
        0x53U, 0x33U, 0x52U, 0x44U, 0x01U, 0x01U, 0x00U, 0x00U,
        0x40U, 0x30U, 0x20U, 0x10U, 0x01U, 0x00U, 0x00U, 0x00U,
        0x2AU, 0x00U, 0x00U, 0x00U, 0x40U, 0xE2U, 0x01U, 0x00U,
        0x0EU, 0x00U, 0xAAU, 0x55U, 0x00U, 0x02U, 0x53U, 0xAEU,
        0x53U, 0xB5U, 0xAAU, 0x40U, 0x00U, 0x04U, 0x00U, 0x08U,
        0xB3U, 0xD6U,
    };
    assert(packet_length == sizeof(expected_packet));
    assert(memcmp(packet, expected_packet, sizeof(expected_packet)) == 0);
}

/**
 * @brief 验证通用 S3RD envelope 保留任意非零类型、flags、ID、序号、时间戳和 payload。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return 返回值：无（void）；通用往返字段不符或 RAW 专用解码未拒绝该类型时 assert 终止。
 * 调用方式：由 main() 调用；成功后 decoded.payload 只借用局部 packet。
 * 线程约束：单线程纯内存测试；不验证实验消息类型的上层业务语义。
 */
static void test_generic_envelope_round_trip(void)
{
    static const uint8_t payload[] = {
        0x00U, 0x01U, 0x7FU, 0x80U, 0xFEU, 0xFFU,
    };
    uint8_t packet[RADAR_UPLINK_MAX_PACKET_SIZE] = {0};
    size_t packet_length = 0U;
    radar_uplink_packet_t decoded = {0};

    assert(radar_uplink_encode_envelope(payload,
                                        sizeof(payload),
                                        UINT8_C(0x7E),
                                        UINT16_C(0xA55A),
                                        UINT32_C(0x10203040),
                                        UINT32_C(0x50607080),
                                        UINT32_C(0x90A0B0C0),
                                        UINT32_C(0xD0E0F000),
                                        packet,
                                        sizeof(packet),
                                        &packet_length) == RADAR_UPLINK_OK);
    assert(packet_length == RADAR_UPLINK_HEADER_SIZE + sizeof(payload) +
                              RADAR_UPLINK_CRC_SIZE);
    assert(radar_uplink_decode_envelope(packet, packet_length, &decoded) ==
           RADAR_UPLINK_OK);
    assert(decoded.version == RADAR_UPLINK_PROTOCOL_VERSION);
    assert(decoded.message_type == UINT8_C(0x7E));
    assert(decoded.flags == UINT16_C(0xA55A));
    assert(decoded.device_id == UINT32_C(0x10203040));
    assert(decoded.stream_id == UINT32_C(0x50607080));
    assert(decoded.sequence == UINT32_C(0x90A0B0C0));
    assert(decoded.timestamp_ms == UINT32_C(0xD0E0F000));
    assert(decoded.payload_length == sizeof(payload));
    assert(memcmp(decoded.payload, payload, sizeof(payload)) == 0);

    /* The generic decoder does not impose a message-specific policy. */
    assert(radar_uplink_decode_packet(packet, packet_length, &decoded) ==
           RADAR_UPLINK_MESSAGE_UNSUPPORTED);
}

/**
 * @brief 验证通用 envelope 支持协议最大 payload 和 NULL+零长度 payload 两个边界。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return 返回值：无（void）；最大包长、内容、零长度视图或元数据不符时 assert 终止。
 * 调用方式：由 main() 调用；先填充最大 payload 往返，再重新编码零 payload 包。
 * 线程约束：单线程主机测试，使用较大栈缓冲；不覆盖网络 MTU、分片或发送背压。
 */
static void test_generic_zero_length_and_maximum_payload(void)
{
    uint8_t packet[RADAR_UPLINK_MAX_PACKET_SIZE] = {0};
    uint8_t payload[RADAR_UPLINK_MAX_PAYLOAD_SIZE];
    size_t packet_length = 0U;
    radar_uplink_packet_t decoded = {0};

    for (size_t index = 0U; index < sizeof(payload); ++index) {
        payload[index] = (uint8_t)(index * 17U + 3U);
    }
    assert(radar_uplink_encode_envelope(payload,
                                        sizeof(payload),
                                        UINT8_C(0xFE),
                                        UINT16_C(0xFFFF),
                                        0U,
                                        UINT32_MAX,
                                        UINT32_MAX,
                                        UINT32_MAX,
                                        packet,
                                        sizeof(packet),
                                        &packet_length) == RADAR_UPLINK_OK);
    assert(packet_length == RADAR_UPLINK_MAX_PACKET_SIZE);
    assert(radar_uplink_decode_envelope(packet, packet_length, &decoded) ==
           RADAR_UPLINK_OK);
    assert(decoded.message_type == UINT8_C(0xFE));
    assert(decoded.flags == UINT16_C(0xFFFF));
    assert(decoded.payload_length == sizeof(payload));
    assert(memcmp(decoded.payload, payload, sizeof(payload)) == 0);

    packet_length = 0U;
    assert(radar_uplink_encode_envelope(NULL,
                                        0U,
                                        UINT8_C(0x02),
                                        UINT16_C(0x1234),
                                        1U,
                                        2U,
                                        3U,
                                        4U,
                                        packet,
                                        sizeof(packet),
                                        &packet_length) == RADAR_UPLINK_OK);
    assert(packet_length == RADAR_UPLINK_MIN_PACKET_SIZE);
    assert(radar_uplink_decode_envelope(packet, packet_length, &decoded) ==
           RADAR_UPLINK_OK);
    assert(decoded.message_type == UINT8_C(0x02));
    assert(decoded.flags == UINT16_C(0x1234));
    assert(decoded.payload == &packet[RADAR_UPLINK_HEADER_SIZE]);
    assert(decoded.payload_length == 0U);
}

/**
 * @brief 验证一个 SRP_MAX_FRAME_SIZE 字节块可作为实验性 telemetry payload 完整封装往返。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return 返回值：无（void）；容量关系、包长或 payload 内容不符时 assert 终止。
 * 调用方式：由 main() 调用；填充确定性模式后使用通用 envelope，不解析内部 SRP 帧。
 * 线程约束：单线程纯内存测试；不验证 telemetry queue、TCP 或 ROS2 消费路径。
 */
static void test_generic_srp_maximum_payload(void)
{
    uint8_t packet[RADAR_UPLINK_MAX_PACKET_SIZE] = {0};
    uint8_t payload[SRP_MAX_FRAME_SIZE];
    size_t packet_length = 0U;
    radar_uplink_packet_t decoded = {0};

    assert(SRP_MAX_FRAME_SIZE <= RADAR_UPLINK_MAX_PAYLOAD_SIZE);
    for (size_t index = 0U; index < sizeof(payload); ++index) {
        payload[index] = (uint8_t)(0xA5U ^ (uint8_t)index);
    }
    assert(radar_uplink_encode_envelope(payload,
                                        sizeof(payload),
                                        UINT8_C(0x03),
                                        UINT16_C(0x0040),
                                        11U,
                                        12U,
                                        13U,
                                        14U,
                                        packet,
                                        sizeof(packet),
                                        &packet_length) == RADAR_UPLINK_OK);
    assert(packet_length == RADAR_UPLINK_HEADER_SIZE +
                              SRP_MAX_FRAME_SIZE +
                              RADAR_UPLINK_CRC_SIZE);
    assert(radar_uplink_decode_envelope(packet, packet_length, &decoded) ==
           RADAR_UPLINK_OK);
    assert(decoded.payload_length == SRP_MAX_FRAME_SIZE);
    assert(memcmp(decoded.payload, payload, sizeof(payload)) == 0);
}

/**
 * @brief 验证通用 envelope 对空输出长度、空 payload、零类型、超长、空输出和短缓冲等参数的拒绝。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return 返回值：无（void）；任一错误码或 output_length 清零语义不符时 assert 终止。
 * 调用方式：由 main() 调用；各断言直接调用 API，失败输出不得继续作为有效包使用。
 * 线程约束：单线程参数边界测试，不访问 socket/RTOS/硬件；局部缓冲由本函数独占。
 */
static void test_generic_rejects_invalid_arguments(void)
{
    static const uint8_t payload[] = {0x42U};
    uint8_t packet[RADAR_UPLINK_MAX_PACKET_SIZE] = {0};
    size_t packet_length = 123U;
    radar_uplink_packet_t decoded = {0};

    assert(radar_uplink_encode_envelope(payload,
                                        sizeof(payload),
                                        1U,
                                        0U,
                                        0U,
                                        0U,
                                        0U,
                                        0U,
                                        packet,
                                        sizeof(packet),
                                        NULL) == RADAR_UPLINK_INVALID_ARG);
    assert(radar_uplink_encode_envelope(NULL,
                                        sizeof(payload),
                                        1U,
                                        0U,
                                        0U,
                                        0U,
                                        0U,
                                        0U,
                                        packet,
                                        sizeof(packet),
                                        &packet_length) == RADAR_UPLINK_INVALID_ARG);
    assert(packet_length == 0U);
    assert(radar_uplink_encode_envelope(payload,
                                        sizeof(payload),
                                        0U,
                                        0U,
                                        0U,
                                        0U,
                                        0U,
                                        0U,
                                        packet,
                                        sizeof(packet),
                                        &packet_length) ==
           RADAR_UPLINK_MESSAGE_UNSUPPORTED);
    assert(radar_uplink_encode_envelope(payload,
                                        RADAR_UPLINK_MAX_PAYLOAD_SIZE + 1U,
                                        1U,
                                        0U,
                                        0U,
                                        0U,
                                        0U,
                                        0U,
                                        packet,
                                        sizeof(packet),
                                        &packet_length) ==
           RADAR_UPLINK_LENGTH_INVALID);
    assert(radar_uplink_encode_envelope(payload,
                                        sizeof(payload),
                                        1U,
                                        0U,
                                        0U,
                                        0U,
                                        0U,
                                        0U,
                                        NULL,
                                        sizeof(packet),
                                        &packet_length) == RADAR_UPLINK_INVALID_ARG);
    assert(radar_uplink_encode_envelope(payload,
                                        sizeof(payload),
                                        1U,
                                        0U,
                                        0U,
                                        0U,
                                        0U,
                                        0U,
                                        packet,
                                        RADAR_UPLINK_MIN_PACKET_SIZE - 1U,
                                        &packet_length) ==
           RADAR_UPLINK_BUFFER_TOO_SMALL);
    assert(radar_uplink_decode_envelope(NULL, 0U, &decoded) ==
           RADAR_UPLINK_INVALID_ARG);
    assert(radar_uplink_decode_envelope(packet, 0U, NULL) ==
           RADAR_UPLINK_INVALID_ARG);
}

/**
 * @brief 验证通用 decoder 拒绝 CRC、长度、零类型、magic 和版本损坏。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return 返回值：无（void）；损坏场景未返回预期状态时 assert 终止测试进程。
 * 调用方式：由 main() 调用；必要时重新编码干净包，避免前一处原地修改污染下一用例。
 * 线程约束：单线程白盒字节变异测试；只证明 decoder 拒绝逻辑，不模拟 TCP 截包。
 */
static void test_generic_rejects_zero_type_and_bad_wire_data(void)
{
    static const uint8_t payload[] = {0x10U, 0x20U, 0x30U};
    uint8_t packet[RADAR_UPLINK_MAX_PACKET_SIZE] = {0};
    size_t packet_length = 0U;
    radar_uplink_packet_t decoded = {0};

    assert(radar_uplink_encode_envelope(payload,
                                        sizeof(payload),
                                        UINT8_C(0x09),
                                        UINT16_C(0xC001),
                                        1U,
                                        2U,
                                        3U,
                                        4U,
                                        packet,
                                        sizeof(packet),
                                        &packet_length) == RADAR_UPLINK_OK);
    packet[packet_length - 1U] ^= 0x01U;
    assert(radar_uplink_decode_envelope(packet, packet_length, &decoded) ==
           RADAR_UPLINK_CRC_MISMATCH);
    packet[packet_length - 1U] ^= 0x01U;
    packet[24] = (uint8_t)(sizeof(payload) + 1U);
    packet[25] = 0U;
    assert(radar_uplink_decode_envelope(packet, packet_length, &decoded) ==
           RADAR_UPLINK_LENGTH_INVALID);

    assert(radar_uplink_encode_envelope(payload,
                                        sizeof(payload),
                                        UINT8_C(0x09),
                                        UINT16_C(0xC001),
                                        1U,
                                        2U,
                                        3U,
                                        4U,
                                        packet,
                                        sizeof(packet),
                                        &packet_length) == RADAR_UPLINK_OK);
    packet[5] = 0U;
    assert(radar_uplink_decode_envelope(packet, packet_length, &decoded) ==
           RADAR_UPLINK_MESSAGE_UNSUPPORTED);

    assert(radar_uplink_encode_envelope(payload,
                                        sizeof(payload),
                                        UINT8_C(0x09),
                                        UINT16_C(0xC001),
                                        1U,
                                        2U,
                                        3U,
                                        4U,
                                        packet,
                                        sizeof(packet),
                                        &packet_length) == RADAR_UPLINK_OK);
    packet[0] ^= 0x01U;
    assert(radar_uplink_decode_envelope(packet, packet_length, &decoded) ==
           RADAR_UPLINK_LENGTH_INVALID);
    packet[0] ^= 0x01U;
    packet[4] = (uint8_t)(RADAR_UPLINK_PROTOCOL_VERSION + 1U);
    assert(radar_uplink_decode_envelope(packet, packet_length, &decoded) ==
           RADAR_UPLINK_VERSION_UNSUPPORTED);
}

/**
 * @brief 验证 RAW_PACKET decoder 拒绝 CRC 损坏、截断和未知 flag，并验证编码短缓冲错误。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return 返回值：无（void）；任一专用状态码不符时 assert 终止。
 * 调用方式：由 main() 调用；从有效合成帧封装后原地变异 S3RD 包头/尾。
 * 线程约束：单线程主机协议测试，不覆盖非阻塞 send、重试或连接重建。
 */
static void test_rejects_corruption_and_truncation(void)
{
    uint8_t frame[RADAR_PARSER_MAX_FRAME_SIZE];
    uint8_t packet[RADAR_UPLINK_MAX_PACKET_SIZE];
    size_t frame_length = make_frame(frame);
    size_t packet_length = 0U;
    radar_uplink_packet_t decoded = {0};

    assert(radar_uplink_encode_frame(frame,
                                     frame_length,
                                     1U,
                                     1U,
                                     1U,
                                     2U,
                                     packet,
                                     sizeof(packet),
                                     &packet_length) == RADAR_UPLINK_OK);
    packet[packet_length - 1U] ^= 0x01U;
    assert(radar_uplink_decode_packet(packet, packet_length, &decoded) ==
           RADAR_UPLINK_CRC_MISMATCH);

    packet[packet_length - 1U] ^= 0x01U;
    assert(radar_uplink_decode_packet(packet, packet_length - 1U, &decoded) ==
           RADAR_UPLINK_LENGTH_INVALID);
    assert(radar_uplink_encode_frame(frame,
                                     frame_length,
                                     1U,
                                     1U,
                                     1U,
                                     2U,
                                     packet,
                                     sizeof(packet),
                                     &packet_length) == RADAR_UPLINK_OK);
    packet[6] = 0x02U;
    assert(radar_uplink_decode_packet(packet, packet_length, &decoded) ==
           RADAR_UPLINK_LENGTH_INVALID);
    packet[6] = 0U;
    assert(radar_uplink_decode_packet(packet, packet_length, &decoded) ==
           RADAR_UPLINK_OK);
    assert(radar_uplink_encode_frame(frame,
                                     frame_length,
                                     1U,
                                     1U,
                                     1U,
                                     2U,
                                     packet,
                                     RADAR_UPLINK_HEADER_SIZE,
                                     &packet_length) ==
           RADAR_UPLINK_BUFFER_TOO_SMALL);
}

/**
 * @brief 验证 RAW_FRAME encoder 在外层封装前拒绝校验和损坏的雷达帧。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return 返回值：无（void）；损坏帧未返回 RADAR_UPLINK_FRAME_INVALID 时 assert 终止。
 * 调用方式：由 main() 调用；翻转合成雷达帧 checksum 字节后尝试编码。
 * 线程约束：单线程内存测试；不验证真实雷达数据质量或 parser 重同步。
 */
static void test_rejects_invalid_lidar_frame(void)
{
    uint8_t frame[RADAR_PARSER_MAX_FRAME_SIZE];
    uint8_t packet[RADAR_UPLINK_MAX_PACKET_SIZE];
    size_t frame_length = make_frame(frame);
    size_t packet_length = 0U;

    frame[8] ^= 0x10U;
    assert(radar_uplink_encode_frame(frame,
                                     frame_length,
                                     1U,
                                     1U,
                                     1U,
                                     2U,
                                     packet,
                                     sizeof(packet),
                                     &packet_length) == RADAR_UPLINK_FRAME_INVALID);
}

/**
 * @brief 验证 CT 零包位映射为 S3RD ZERO_PACKET flag，并检查版本/消息类型拒绝顺序。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return 返回值：无（void）；flag 往返或头字段错误码不符时 assert 终止。
 * 调用方式：由 main() 调用；修改 CT 后同步修正其参与的 XOR，再编码并变异外层头。
 * 线程约束：单线程白盒协议测试，不代表扫描零包在 TCP 断线后的实机重同步已验证。
 */
static void test_zero_packet_flag_and_header_validation(void)
{
    uint8_t frame[RADAR_PARSER_MAX_FRAME_SIZE];
    uint8_t packet[RADAR_UPLINK_MAX_PACKET_SIZE];
    size_t frame_length = make_frame(frame);
    size_t packet_length = 0U;
    radar_uplink_packet_t decoded = {0};

    frame[2] = 0x01U;
    frame[8] ^= 0x01U; /* CT is part of the official LSN/CT XOR word. */
    assert(radar_uplink_encode_frame(frame,
                                     frame_length,
                                     1U,
                                     2U,
                                     3U,
                                     4U,
                                     packet,
                                     sizeof(packet),
                                     &packet_length) == RADAR_UPLINK_OK);
    assert(read_le16(&packet[6]) == RADAR_UPLINK_FLAG_ZERO_PACKET);
    assert(radar_uplink_decode_packet(packet, packet_length, &decoded) ==
           RADAR_UPLINK_OK);
    assert(decoded.flags == RADAR_UPLINK_FLAG_ZERO_PACKET);

    packet[4] = (uint8_t)(RADAR_UPLINK_PROTOCOL_VERSION + 1U);
    assert(radar_uplink_decode_packet(packet, packet_length, &decoded) ==
           RADAR_UPLINK_VERSION_UNSUPPORTED);
    packet[4] = RADAR_UPLINK_PROTOCOL_VERSION;
    packet[5] = (uint8_t)(RADAR_UPLINK_MESSAGE_RAW_FRAME + 1U);
    assert(radar_uplink_decode_packet(packet, packet_length, &decoded) ==
           RADAR_UPLINK_MESSAGE_UNSUPPORTED);
}

/**
 * @brief 构造最大 3 字节样本雷达帧并验证最大 S3RD 包的完整往返和 32 位元数据。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return 返回值：无（void）；最大包长、解码状态、payload 长度或元数据不符时 assert 终止。
 * 调用方式：由 main() 调用；在栈缓冲构造全部最大样本及官方 XOR 后封装。
 * 线程约束：单线程 host 大缓冲测试；不测任务栈余量、PSRAM、网络分片或服务端吞吐。
 */
static void test_maximum_frame_round_trip(void)
{
    uint8_t frame[RADAR_PARSER_MAX_FRAME_SIZE];
    uint8_t packet[RADAR_UPLINK_MAX_PACKET_SIZE];
    size_t packet_length = 0U;
    radar_uplink_packet_t decoded = {0};
    memset(frame, 0, sizeof(frame));

    /* Build a maximum-size intensity frame and calculate its official XOR. */
    frame[0] = RADAR_X3PRO_HEADER_BYTE_0;
    frame[1] = RADAR_X3PRO_HEADER_BYTE_1;
    frame[2] = 0x00U;
    frame[3] = RADAR_X3PRO_MAX_SAMPLES;
    write_le16(&frame[4], 0xAE53U);
    write_le16(&frame[6], 0xAE53U);
    for (size_t index = 0U; index < RADAR_X3PRO_MAX_SAMPLES; ++index) {
        const size_t offset = RADAR_X3PRO_HEADER_BYTES +
                              index * RADAR_X3PRO_MAX_SAMPLE_BYTES;
        frame[offset] = (uint8_t)index;
        write_le16(&frame[offset + 1U], (uint16_t)(0x0400U + index));
    }
    uint16_t checksum = 0x55AAU;
    checksum ^= read_le16(&frame[4]);
    for (size_t index = 0U; index < RADAR_X3PRO_MAX_SAMPLES; ++index) {
        const size_t offset = RADAR_X3PRO_HEADER_BYTES +
                              index * RADAR_X3PRO_MAX_SAMPLE_BYTES;
        checksum ^= frame[offset];
        checksum ^= read_le16(&frame[offset + 1U]);
    }
    checksum ^= (uint16_t)(((uint16_t)frame[3] << 8U) | frame[2]);
    checksum ^= read_le16(&frame[6]);
    write_le16(&frame[8], checksum);

    assert(radar_uplink_encode_frame(frame,
                                     sizeof(frame),
                                     UINT32_C(0xAABBCCDD),
                                     UINT32_C(0x11223344),
                                     UINT32_C(0x55667788),
                                     UINT32_C(0x99AABBCC),
                                     packet,
                                     sizeof(packet),
                                     &packet_length) == RADAR_UPLINK_OK);
    assert(packet_length == RADAR_UPLINK_MAX_PACKET_SIZE);
    assert(radar_uplink_decode_packet(packet, packet_length, &decoded) ==
           RADAR_UPLINK_OK);
    assert(decoded.payload_length == RADAR_PARSER_MAX_FRAME_SIZE);
    assert(decoded.device_id == UINT32_C(0xAABBCCDD));
    assert(decoded.stream_id == UINT32_C(0x11223344));
    assert(decoded.sequence == UINT32_C(0x55667788));
    assert(decoded.timestamp_ms == UINT32_C(0x99AABBCC));
}

/**
 * @brief 顺序执行 S3RD RAW_FRAME 与通用 envelope 的主机协议断言集合。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return 全部断言通过返回 0；任一 assert 失败会终止测试进程。
 * 调用方式：由 radar/tests/run_host_tests.sh 编译并直接运行。
 * 线程约束：单进程单线程，不建立 Wi-Fi/TCP/ROS2 链路，也不读取 UART1 雷达。
 */
int main(void)
{
    test_round_trip();
    test_generic_envelope_round_trip();
    test_generic_zero_length_and_maximum_payload();
    test_generic_srp_maximum_payload();
    test_generic_rejects_invalid_arguments();
    test_generic_rejects_zero_type_and_bad_wire_data();
    test_rejects_corruption_and_truncation();
    test_rejects_invalid_lidar_frame();
    test_zero_packet_flag_and_header_validation();
    test_maximum_frame_round_trip();
    return 0;
}
