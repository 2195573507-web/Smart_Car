#include "radar_uplink_protocol.h"

/* S3RD 外层封装实现；创建人：待确认（当前维护人：Zhiqin）。 */

#include <string.h>

/**
 * @brief  计算指定字节序列的 CRC16-Modbus 校验值。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（函数契约补充）。
 * @param  data length 大于 0 时必须指向至少 length 字节的只读数据。
 * @param  length 参与计算的字节数，可为 0。
 * @return 以 0xFFFF 为初值、0xA001 为反射多项式计算出的 16 位校验值。
 * 调用方式：S3RD 编码/解码在已验证缓冲边界后调用；本函数不检查 data 是否为空。
 * 线程约束：无静态可变状态、可重入、不阻塞；计算量与 length 成正比，大包不适合 ISR。
 */
static uint16_t crc16_modbus(const uint8_t *data, size_t length)
{
    uint16_t crc = UINT16_C(0xFFFF);
    for (size_t index = 0U; index < length; ++index) {
        crc ^= data[index];
        for (uint8_t bit = 0U; bit < 8U; ++bit) {
            crc = (crc & 1U) != 0U
                      ? (uint16_t)((crc >> 1U) ^ UINT16_C(0xA001))
                      : (uint16_t)(crc >> 1U);
        }
    }
    return crc;
}

/**
 * @brief  从连续两字节读取一个小端 16 位整数。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（函数契约补充）。
 * @param  data 必须指向至少 2 个可读字节，函数不做空指针或边界检查。
 * @return 解码后的 uint16_t 值。
 * 调用方式：仅在 S3RD 包长度完成校验后读取固定字段或 CRC。
 * 线程约束：纯只读计算、可重入、不阻塞，可在普通任务中调用。
 */
static uint16_t read_le16(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8U);
}

/**
 * @brief  从连续四字节读取一个小端 32 位整数。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（函数契约补充）。
 * @param  data 必须指向至少 4 个可读字节，函数不做空指针或边界检查。
 * @return 解码后的 uint32_t 值。
 * 调用方式：仅在 S3RD 包长度完成校验后读取设备、流、序号和时间戳字段。
 * 线程约束：纯只读计算、可重入、不阻塞，可在普通任务中调用。
 */
static uint32_t read_le32(const uint8_t *data)
{
    return (uint32_t)data[0] |
           ((uint32_t)data[1] << 8U) |
           ((uint32_t)data[2] << 16U) |
           ((uint32_t)data[3] << 24U);
}

/**
 * @brief  将 16 位整数按小端序写入连续两字节。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（函数契约补充）。
 * @param[out] data 必须指向至少 2 个可写字节，函数不做空指针或容量检查。
 * @param  value 待编码的 16 位值。
 * @return 无。
 * 调用方式：仅在编码器确认 output 容量足够后写固定字段或 CRC。
 * 线程约束：纯内存写入、可重入、不阻塞；同一输出缓冲不得被并发修改。
 */
static void write_le16(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8U);
}

/**
 * @brief  将 32 位整数按小端序写入连续四字节。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（函数契约补充）。
 * @param[out] data 必须指向至少 4 个可写字节，函数不做空指针或容量检查。
 * @param  value 待编码的 32 位值。
 * @return 无。
 * 调用方式：仅在编码器确认 output 容量足够后写设备、流、序号和时间戳字段。
 * 线程约束：纯内存写入、可重入、不阻塞；同一输出缓冲不得被并发修改。
 */
static void write_le32(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8U);
    data[2] = (uint8_t)(value >> 16U);
    data[3] = (uint8_t)(value >> 24U);
}

/**
 * @brief  检查缓冲起始四字节是否为 S3RD 魔数。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（函数契约补充）。
 * @param  packet 必须指向至少 4 个可读字节；调用方先完成最小包长检查。
 * @return 四个魔数字节全部匹配时为 true，否则为 false。
 * 调用方式：由 S3RD 解码入口在访问其他头字段前调用；不独立检查空指针或长度。
 * 线程约束：纯只读计算、可重入、不阻塞。
 */
static bool has_magic(const uint8_t *packet)
{
    return packet[0] == RADAR_UPLINK_MAGIC_0 &&
           packet[1] == RADAR_UPLINK_MAGIC_1 &&
           packet[2] == RADAR_UPLINK_MAGIC_2 &&
           packet[3] == RADAR_UPLINK_MAGIC_3;
}

/**
 * @brief  编码通用 S3RD 外层包并追加 CRC16-Modbus。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（函数契约补充）。
 * @param  payload 消息体；payload_length 为 0 时可为 NULL，否则必须非 NULL。
 * @param  payload_length 消息体字节数，不超过 RADAR_UPLINK_MAX_PAYLOAD_SIZE 和 UINT16_MAX。
 * @param  message_type 非 0 的实验性消息类型；本入口不校验类型专属 payload。
 * @param  flags 写入包头的 16 位标志。
 * @param  device_id 设备 ID，按小端序编码。
 * @param  stream_id 数据流 ID，按小端序编码。
 * @param  sequence 上行包序号，按小端序编码。
 * @param  timestamp_ms S3 时间戳，单位 ms，按小端序编码。
 * @param[out] output 非 NULL、至少可写 output_capacity 字节，且不得与 payload 重叠。
 * @param  output_capacity output 的字节容量。
 * @param[out] output_length 必须非 NULL；进入函数即清零，成功时写完整包长。
 * @return RADAR_UPLINK_OK，或参数、类型、长度、输出容量对应的错误状态。
 * 调用方式：上行任务或主机测试构造完整 S3RD 包；只在返回 OK 后发送 output。
 * 线程约束：无静态可变状态、可重入、不阻塞；大包复制和 CRC 扫描不适合 ISR。
 */
radar_uplink_status_t radar_uplink_encode_envelope(
    const uint8_t *payload,
    size_t payload_length,
    uint8_t message_type,
    uint16_t flags,
    uint32_t device_id,
    uint32_t stream_id,
    uint32_t sequence,
    uint32_t timestamp_ms,
    uint8_t *output,
    size_t output_capacity,
    size_t *output_length)
{
    size_t packet_length;
    uint16_t crc;

    if (output_length == NULL) {
        return RADAR_UPLINK_INVALID_ARG;
    }
    *output_length = 0U;
    if (output == NULL || (payload == NULL && payload_length != 0U)) {
        return RADAR_UPLINK_INVALID_ARG;
    }
    if (message_type == 0U) {
        return RADAR_UPLINK_MESSAGE_UNSUPPORTED;
    }
    if (payload_length > RADAR_UPLINK_MAX_PAYLOAD_SIZE ||
        payload_length > UINT16_MAX) {
        return RADAR_UPLINK_LENGTH_INVALID;
    }
    if (payload_length > SIZE_MAX - RADAR_UPLINK_MIN_PACKET_SIZE) {
        return RADAR_UPLINK_LENGTH_INVALID;
    }
    packet_length = RADAR_UPLINK_MIN_PACKET_SIZE + payload_length;
    if (output_capacity < packet_length) {
        return RADAR_UPLINK_BUFFER_TOO_SMALL;
    }

    output[0] = RADAR_UPLINK_MAGIC_0;
    output[1] = RADAR_UPLINK_MAGIC_1;
    output[2] = RADAR_UPLINK_MAGIC_2;
    output[3] = RADAR_UPLINK_MAGIC_3;
    output[4] = RADAR_UPLINK_PROTOCOL_VERSION;
    output[5] = message_type;
    write_le16(&output[6], flags);
    write_le32(&output[8], device_id);
    write_le32(&output[12], stream_id);
    write_le32(&output[16], sequence);
    write_le32(&output[20], timestamp_ms);
    write_le16(&output[24], (uint16_t)payload_length);
    if (payload_length != 0U) {
        memcpy(&output[RADAR_UPLINK_HEADER_SIZE], payload, payload_length);
    }
    crc = crc16_modbus(&output[4], RADAR_UPLINK_HEADER_SIZE - 4U + payload_length);
    write_le16(&output[RADAR_UPLINK_HEADER_SIZE + payload_length], crc);
    *output_length = packet_length;
    return RADAR_UPLINK_OK;
}

/**
 * @brief  校验并解析通用 S3RD 外层包，返回指向输入包内部的 payload 视图。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（函数契约补充）。
 * @param  packet 非 NULL 的完整 S3RD 包缓冲。
 * @param  packet_length 必须精确等于固定头、payload 和 CRC 的总长度。
 * @param[out] decoded 非 NULL；仅返回 RADAR_UPLINK_OK 时各字段有效。
 * @return RADAR_UPLINK_OK，或参数、长度、版本、类型、CRC 对应的错误状态。
 * 调用方式：接收解析任务调用；decoded->payload 借用 packet 内存，packet 复用前必须消费或复制。
 * 线程约束：只读、可重入、不阻塞；失败时 decoded 可能保留旧值，禁止使用其内容；
 *           大包 CRC 不适合 ISR。
 */
radar_uplink_status_t radar_uplink_decode_envelope(
    const uint8_t *packet,
    size_t packet_length,
    radar_uplink_packet_t *decoded)
{
    size_t payload_length;
    size_t expected_length;
    uint16_t received_crc;
    uint16_t calculated_crc;

    if (packet == NULL || decoded == NULL) {
        return RADAR_UPLINK_INVALID_ARG;
    }
    if (packet_length < RADAR_UPLINK_MIN_PACKET_SIZE ||
        !has_magic(packet)) {
        return RADAR_UPLINK_LENGTH_INVALID;
    }
    if (packet[4] != RADAR_UPLINK_PROTOCOL_VERSION) {
        return RADAR_UPLINK_VERSION_UNSUPPORTED;
    }
    if (packet[5] == 0U) {
        return RADAR_UPLINK_MESSAGE_UNSUPPORTED;
    }
    payload_length = read_le16(&packet[24]);
    if (payload_length > SIZE_MAX - RADAR_UPLINK_MIN_PACKET_SIZE) {
        return RADAR_UPLINK_LENGTH_INVALID;
    }
    expected_length = RADAR_UPLINK_MIN_PACKET_SIZE + payload_length;
    if (payload_length > RADAR_UPLINK_MAX_PAYLOAD_SIZE ||
        packet_length != expected_length) {
        return RADAR_UPLINK_LENGTH_INVALID;
    }

    received_crc = read_le16(&packet[RADAR_UPLINK_HEADER_SIZE + payload_length]);
    calculated_crc = crc16_modbus(&packet[4],
                                  RADAR_UPLINK_HEADER_SIZE - 4U + payload_length);
    if (received_crc != calculated_crc) {
        return RADAR_UPLINK_CRC_MISMATCH;
    }

    decoded->version = packet[4];
    decoded->message_type = packet[5];
    decoded->flags = read_le16(&packet[6]);
    decoded->device_id = read_le32(&packet[8]);
    decoded->stream_id = read_le32(&packet[12]);
    decoded->sequence = read_le32(&packet[16]);
    decoded->timestamp_ms = read_le32(&packet[20]);
    decoded->payload = &packet[RADAR_UPLINK_HEADER_SIZE];
    decoded->payload_length = payload_length;
    return RADAR_UPLINK_OK;
}

/**
 * @brief  复验完整 YDLIDAR 帧并封装为 S3RD RAW_FRAME 消息。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（函数契约补充）。
 * @param  frame 非 NULL、候选的完整雷达原始帧。
 * @param  frame_length 范围为 1..RADAR_PARSER_MAX_FRAME_SIZE，且须与 LSN/校验和一致。
 * @param  device_id 设备 ID。
 * @param  stream_id 雷达流 ID。
 * @param  sequence S3RD 包序号，不要求等同 UART 帧序号。
 * @param  timestamp_ms UART 接收时间，单位 ms。
 * @param[out] output 非 NULL 的输出包缓冲，且不得与 frame 重叠。
 * @param  output_capacity output 的字节容量。
 * @param[out] output_length 必须非 NULL；只保证在通用编码入口执行后被清零/写入，
 *                           帧预校验失败时可能保持调用前值。
 * @return 成功为 RADAR_UPLINK_OK；参数、帧校验或通用编码失败时返回对应状态。
 * 调用方式：上行任务从 FIFO 取出完整帧后调用；调用方应预先清零 output_length 且只在 OK 后读取。
 * 线程约束：无静态可变状态、可重入；完整帧校验、复制和 CRC 不适合 ISR。
 */
radar_uplink_status_t radar_uplink_encode_frame(
    const uint8_t *frame,
    size_t frame_length,
    uint32_t device_id,
    uint32_t stream_id,
    uint32_t sequence,
    uint32_t timestamp_ms,
    uint8_t *output,
    size_t output_capacity,
    size_t *output_length)
{
    if (output_length == NULL || output == NULL ||
        (frame == NULL && frame_length != 0U)) {
        return RADAR_UPLINK_INVALID_ARG;
    }
    if (frame_length == 0U || frame_length > RADAR_PARSER_MAX_FRAME_SIZE) {
        return RADAR_UPLINK_FRAME_INVALID;
    }
    if (!radar_parser_validate_frame(frame, frame_length, NULL)) {
        return RADAR_UPLINK_FRAME_INVALID;
    }

    const uint16_t flags =
        (frame[2] & 0x01U) != 0U ? RADAR_UPLINK_FLAG_ZERO_PACKET : 0U;
    return radar_uplink_encode_envelope(frame,
                                        frame_length,
                                        RADAR_UPLINK_MESSAGE_RAW_FRAME,
                                        flags,
                                        device_id,
                                        stream_id,
                                        sequence,
                                        timestamp_ms,
                                        output,
                                        output_capacity,
                                        output_length);
}

/**
 * @brief  兼容旧名称的 RAW_FRAME 专用 S3RD 解码和严格复验入口。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（函数契约补充）。
 * @param  packet 非 NULL 的完整 S3RD 包。
 * @param  packet_length 完整包字节数。
 * @param[out] decoded 非 NULL；仅返回 OK 时有效，payload 借用 packet 生命周期。
 * @return 除通用解码错误外，还会拒绝非 RAW_FRAME 类型、未知 flag、无效雷达帧以及
 *         ZERO_PACKET 标志与 CT 位不一致的包。
 * 调用方式：主机兼容解析和协议测试调用；多类型分发应优先使用 decode_envelope()。
 * 线程约束：只读、可重入、不阻塞；失败时 decoded 内容不得使用，大包复验不适合 ISR。
 */
radar_uplink_status_t radar_uplink_decode_packet(
    const uint8_t *packet,
    size_t packet_length,
    radar_uplink_packet_t *decoded)
{
    /* Preserve the legacy decoder's header rejection order and status codes. */
    if (packet == NULL || decoded == NULL) {
        return RADAR_UPLINK_INVALID_ARG;
    }
    if (packet_length < RADAR_UPLINK_MIN_PACKET_SIZE ||
        !has_magic(packet)) {
        return RADAR_UPLINK_LENGTH_INVALID;
    }
    if (packet[4] != RADAR_UPLINK_PROTOCOL_VERSION) {
        return RADAR_UPLINK_VERSION_UNSUPPORTED;
    }
    if (packet[5] != RADAR_UPLINK_MESSAGE_RAW_FRAME) {
        return RADAR_UPLINK_MESSAGE_UNSUPPORTED;
    }
    const uint16_t header_flags = read_le16(&packet[6]);
    if ((header_flags & (uint16_t)~RADAR_UPLINK_FLAG_ZERO_PACKET) != 0U) {
        return RADAR_UPLINK_LENGTH_INVALID;
    }

    radar_uplink_status_t status =
        radar_uplink_decode_envelope(packet, packet_length, decoded);
    if (status != RADAR_UPLINK_OK) {
        return status;
    }
    if (decoded->message_type != RADAR_UPLINK_MESSAGE_RAW_FRAME) {
        return RADAR_UPLINK_MESSAGE_UNSUPPORTED;
    }
    if (decoded->payload_length == 0U ||
        decoded->payload_length > RADAR_PARSER_MAX_FRAME_SIZE) {
        return RADAR_UPLINK_LENGTH_INVALID;
    }
    if (!radar_parser_validate_frame(decoded->payload,
                                     decoded->payload_length,
                                     NULL)) {
        return RADAR_UPLINK_FRAME_INVALID;
    }
    const bool zero_packet = (decoded->payload[2] & 0x01U) != 0U;
    if (((decoded->flags & RADAR_UPLINK_FLAG_ZERO_PACKET) != 0U) !=
        zero_packet) {
        return RADAR_UPLINK_FRAME_INVALID;
    }
    return RADAR_UPLINK_OK;
}
