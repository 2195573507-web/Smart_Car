#ifndef S3_RADAR_UPLINK_PROTOCOL_H
#define S3_RADAR_UPLINK_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "radar_parser.h"
#include "srp_registry.h"

/*
 * 实验性 S3 -> 主机雷达封装（S3RD）。
 * 创建人：待确认（当前维护人：Zhiqin）。
 * 说明：该封装仍需 S3/Windows 联合冻结；本头文件不代表已完成端到端验收。
 */
#define RADAR_UPLINK_MAGIC_0 0x53U /* 'S' */
#define RADAR_UPLINK_MAGIC_1 0x33U /* '3' */
#define RADAR_UPLINK_MAGIC_2 0x52U /* 'R' */
#define RADAR_UPLINK_MAGIC_3 0x44U /* 'D' */
#define RADAR_UPLINK_PROTOCOL_VERSION 1U
/* 保留原公共名称以兼容既有源码。 */
#define RADAR_UPLINK_MESSAGE_RAW_FRAME 1U
#define RADAR_UPLINK_MESSAGE_TYPE_RAW_FRAME RADAR_UPLINK_MESSAGE_RAW_FRAME
/* 候选值：SRPv4 遥测类型必须经过 S3/Windows 联合评审后才能冻结。 */
#define RADAR_UPLINK_MESSAGE_SRP_TELEMETRY_EXPERIMENTAL 2U
#define RADAR_UPLINK_FLAG_ZERO_PACKET 0x0001U
#define RADAR_UPLINK_HEADER_SIZE 26U
#define RADAR_UPLINK_CRC_SIZE 2U
#define RADAR_UPLINK_MIN_PACKET_SIZE \
    (RADAR_UPLINK_HEADER_SIZE + RADAR_UPLINK_CRC_SIZE)
/*
 * 外层容量同时覆盖当前 YDLIDAR 最大帧和完整 SRPv4 帧；SRPv4 上限以共享协议
 * 定义为唯一依据，不在本模块复制数值。
 */
#define RADAR_UPLINK_MAX_PAYLOAD_SIZE \
    ((RADAR_PARSER_MAX_FRAME_SIZE > SRP_MAX_FRAME_SIZE) \
         ? RADAR_PARSER_MAX_FRAME_SIZE \
         : SRP_MAX_FRAME_SIZE)
#define RADAR_UPLINK_MAX_PACKET_SIZE \
    (RADAR_UPLINK_HEADER_SIZE + RADAR_UPLINK_MAX_PAYLOAD_SIZE + \
     RADAR_UPLINK_CRC_SIZE)

/** 实验性 S3RD 外层编解码结果；非 OK 时输出包或 decoded 视图不得使用。 */
typedef enum {
    RADAR_UPLINK_OK = 0,             /**< 外层包编码或解码成功。 */
    RADAR_UPLINK_INVALID_ARG,        /**< 必需指针或 payload 指针/长度组合非法。 */
    RADAR_UPLINK_FRAME_INVALID,      /**< RAW_FRAME 内层 YDLIDAR 帧或 ZERO_PACKET 语义无效。 */
    RADAR_UPLINK_BUFFER_TOO_SMALL,   /**< 编码输出容量不足。 */
    RADAR_UPLINK_LENGTH_INVALID,     /**< 外层/内层长度、魔数或严格 flags 边界无效。 */
    RADAR_UPLINK_VERSION_UNSUPPORTED,/**< 包内版本不是当前 S3RD v1。 */
    RADAR_UPLINK_MESSAGE_UNSUPPORTED,/**< 消息类型为保留值 0 或不符合严格 RAW_FRAME 入口。 */
    RADAR_UPLINK_CRC_MISMATCH        /**< 外层 CRC-16/MODBUS 校验不一致。 */
} radar_uplink_status_t;

/** 已解码 S3RD 逻辑包视图；payload 借用输入 packet，结构体不拥有存储。 */
typedef struct {
    uint16_t flags; /**< 16 位外层标志；通用入口不解释未知位，RAW_FRAME 只允许 ZERO_PACKET。 */
    uint32_t device_id; /**< 设备标识，具体分配规则尚待 S3/Windows 联合冻结。 */
    uint32_t stream_id; /**< 同一设备内的数据流标识，具体分配规则尚待冻结。 */
    uint32_t sequence; /**< S3RD 上行包序号，按 32 位自然回绕。 */
    uint32_t timestamp_ms; /**< S3 接收/封装时间，单位单调 ms。 */
    const uint8_t *payload; /**< 借用输入包内部消息体；packet 复用前必须消费或复制。 */
    size_t payload_length; /**< payload 字节数，不超过 RADAR_UPLINK_MAX_PAYLOAD_SIZE。 */
    /* 追加在末尾以保持既有成员的顺序兼容。 */
    uint8_t version; /**< 外层协议版本；当前成功解码值为 1。 */
    uint8_t message_type; /**< 非零 S3RD 消息类型；RAW_FRAME=1，SRP telemetry=实验性 2。 */
} radar_uplink_packet_t;

/* 通用入口复制 payload；允许任意非零消息类型和 16 位 flags。零长度 payload
 * 可配 NULL 指针，消息类型 0 保留。 */
/**
 * @brief 编码一条通用 S3RD 外层包。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（接口契约补充）。
 * @param  payload 消息体；payload_length 为 0 时可为 NULL，否则必须非 NULL。
 * @param  payload_length 消息体字节数，不超过 RADAR_UPLINK_MAX_PAYLOAD_SIZE/UINT16_MAX。
 * @param  message_type 非 0 的实验性消息类型；函数不做类型专属 payload 校验。
 * @param  flags 16 位标志，通用入口接受任意位组合。
 * @param  device_id 设备 ID，按小端序编码。
 * @param  stream_id 数据流 ID，按小端序编码。
 * @param  sequence 上行包序号，按小端序编码。
 * @param  timestamp_ms S3 侧时间戳，单位 ms，按小端序编码。
 * @param[out] output 非 NULL、至少 output_capacity 字节的输出缓冲，不得与 payload 重叠。
 * @param  output_capacity output 容量。
 * @param[out] output_length 必须非 NULL；进入函数即清零，成功时写实际包长。
 * @return OK，或 INVALID_ARG、MESSAGE_UNSUPPORTED、LENGTH_INVALID、BUFFER_TOO_SMALL。
 * 调用方式：普通任务/主机测试构造完整 S3RD 包；函数复制 payload 并追加 CRC16-Modbus。
 * 线程约束：无静态可变状态、可重入、不阻塞；大包复制/CRC 不适合 ISR。
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
    size_t *output_length);

/* 通用解码只验证 S3RD 外层，刻意接受未知非零类型和任意 flags；调用方负责
 * 类型专属 payload 校验，decoded->payload 指向输入 packet 内部。 */
/**
 * @brief 校验并解析通用 S3RD 外层包。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（接口契约补充）。
 * @param  packet 非 NULL 的完整 S3RD 包缓冲。
 * @param  packet_length 必须精确等于 header + payload + CRC 长度。
 * @param[out] decoded 非 NULL；仅返回 OK 时字段有效，失败时内容不得使用。
 * @return OK，或 INVALID_ARG、LENGTH_INVALID、VERSION_UNSUPPORTED、
 *         MESSAGE_UNSUPPORTED、CRC_MISMATCH。
 * 调用方式：接收解析任务调用；接受任意非 0 message_type/flags，类型专属语义由上层校验。
 *           decoded->payload 只是 packet 内部借用视图，packet 失效/复用前必须完成消费或复制。
 * 线程约束：只读计算、可重入、不阻塞；CRC 大包扫描不适合 ISR。
 */
radar_uplink_status_t radar_uplink_decode_envelope(
    const uint8_t *packet,
    size_t packet_length,
    radar_uplink_packet_t *decoded);

/**
 * @brief 将已校验雷达原始帧封装为 S3RD RAW_FRAME 消息。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（接口契约补充）。
 * @param  frame 非 NULL、通过默认 X3/X3PRO 校验的完整 YDLIDAR 帧。
 * @param  frame_length 范围 1..RADAR_PARSER_MAX_FRAME_SIZE，且须与 LSN/校验和一致。
 * @param  device_id 设备 ID。
 * @param  stream_id 雷达流 ID。
 * @param  sequence S3RD 包序号；不强制等同 UART 帧序号。
 * @param  timestamp_ms UART 接收侧时间戳，单位 ms。
 * @param[out] output 非 NULL 的包缓冲，不得与 frame 重叠。
 * @param  output_capacity output 容量。
 * @param[out] output_length 必须非 NULL；成功时写包长。帧预校验阶段失败时当前实现可能保留
 *                           调用前值，调用方应预先清零并且只在返回 OK 后读取。
 * @return 通用编码状态，或帧为空/超长/校验失败时 RADAR_UPLINK_FRAME_INVALID。
 * 调用方式：雷达上行任务从 FIFO 取出完整帧后调用；只封装原始帧，不解码测量点。
 * 线程约束：无静态可变状态、可重入；大帧校验/复制/CRC 不适合 ISR。
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
    size_t *output_length);

/**
 * @brief 兼容旧接口的 RAW_FRAME 专用 S3RD 解码与严格复验入口。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（接口契约补充）。
 * @param  packet 非 NULL 的完整 S3RD 包。
 * @param  packet_length 完整包长度。
 * @param[out] decoded 非 NULL；仅返回 OK 时有效，payload 借用 packet 生命周期。
 * @return 除通用 envelope 错误外，还会拒绝非 RAW_FRAME 类型、未知 flag、空/超长 payload、
 *         YDLIDAR 校验失败和 ZERO_PACKET flag 与原始帧 CT 不一致。
 * 调用方式：主机端兼容解析和协议测试使用；通用多类型分发先调用 decode_envelope()，再做类型校验。
 * 线程约束：只读计算、可重入、不阻塞；大包复验不适合 ISR。
 */
radar_uplink_status_t radar_uplink_decode_packet(
    const uint8_t *packet,
    size_t packet_length,
    radar_uplink_packet_t *decoded);

#endif
