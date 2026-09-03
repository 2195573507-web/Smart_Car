#ifndef SRP_DEF_H
#define SRP_DEF_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * SRP v4 线缆基础定义。
 * 创建人：待确认（当前维护人：Zhiqin）。
 * 本文件是 STM32H757 与 ESP32-S3 的共享协议真值；修改任何字节常量前，
 * 必须同步两端实现、测试向量和协议文档。
 */

#if defined(__GNUC__) || defined(__clang__)
#define SRP_PACKED __attribute__((packed))
#define SRP_PACKED_ALIGNED(alignment) \
    __attribute__((packed, aligned(alignment)))
#else
#define SRP_PACKED
#define SRP_PACKED_ALIGNED(alignment)
#endif

/* 帧头/帧尾均按线缆字节序显式写入，不能直接发送 C 结构体。 */
#define SRP_MAGIC UINT16_C(0x55AA)
#define SRP_MAGIC_BYTE0 UINT8_C(0xAA)
#define SRP_MAGIC_BYTE1 UINT8_C(0x55)
#define SRP_EOF UINT16_C(0x0A0D)
#define SRP_EOF_BYTE0 UINT8_C(0x0D)
#define SRP_EOF_BYTE1 UINT8_C(0x0A)
#define SRP_HEADER_SIZE UINT16_C(8)
#define SRP_TRAILER_SIZE UINT16_C(4)
#define SRP_MAX_PAYLOAD UINT16_C(500)
#define SRP_MAX_FRAME_SIZE (SRP_HEADER_SIZE + SRP_MAX_PAYLOAD + SRP_TRAILER_SIZE)

#define SRP_HDR_PRI_MASK UINT32_C(0xFF000000)
#define SRP_HDR_TYPE_MASK UINT32_C(0x00FF0000)
#define SRP_HDR_SEQ_MASK UINT32_C(0x0000FF00)
#define SRP_HDR_FLAGS_MASK UINT32_C(0x000000FF)
#define SRP_HDR_PRI_SHIFT UINT8_C(24)
#define SRP_HDR_TYPE_SHIFT UINT8_C(16)
#define SRP_HDR_SEQ_SHIFT UINT8_C(8)

/* 将优先级、类型、序号和标志组合为逻辑 header；参数会被转换为无符号值。 */
#define SRP_HDR_MAKE(priority, type, sequence, flags) \
    ((((uint32_t)(priority) << SRP_HDR_PRI_SHIFT) & SRP_HDR_PRI_MASK) | \
     (((uint32_t)(type) << SRP_HDR_TYPE_SHIFT) & SRP_HDR_TYPE_MASK) | \
     (((uint32_t)(sequence) << SRP_HDR_SEQ_SHIFT) & SRP_HDR_SEQ_MASK) | \
     ((uint32_t)(flags) & SRP_HDR_FLAGS_MASK))
#define SRP_HDR_PRI(header) \
    ((uint8_t)(((uint32_t)(header) & SRP_HDR_PRI_MASK) >> SRP_HDR_PRI_SHIFT))
#define SRP_HDR_TYPE(header) \
    ((uint8_t)(((uint32_t)(header) & SRP_HDR_TYPE_MASK) >> SRP_HDR_TYPE_SHIFT))
#define SRP_HDR_SEQ(header) \
    ((uint8_t)(((uint32_t)(header) & SRP_HDR_SEQ_MASK) >> SRP_HDR_SEQ_SHIFT))
#define SRP_HDR_FLAGS(header) ((uint8_t)((uint32_t)(header) & SRP_HDR_FLAGS_MASK))

/** SRP header 的 8 位逻辑优先级；数值越小表示业务紧急程度越高。 */
typedef enum {
    SRP_PRIORITY_EMERGENCY = 0, /**< 急停/故障类最高优先级，不表示可绕过安全门。 */
    SRP_PRIORITY_COMMAND = 1,   /**< 同步、控制和事务响应优先级。 */
    SRP_PRIORITY_TELEMETRY = 2, /**< 周期状态与传感器遥测优先级。 */
    SRP_PRIORITY_LOG = 3        /**< 可丢弃诊断日志最低优先级。 */
} srp_priority_t;

enum {
    SRP_FLAG_TLV = UINT8_C(0x01),
    SRP_FLAG_ACK_REQUIRED = UINT8_C(0x02),
    SRP_FLAG_ACK = UINT8_C(0x04),
    SRP_FLAG_ERROR = UINT8_C(0x08),
    SRP_FLAG_RESERVED_MASK = UINT8_C(0xF0)
};

#define SRP_FLAG_STREAM_DATA UINT8_C(0)

enum {
    SRP_NODE_STM32H757 = UINT8_C(1),
    SRP_NODE_ESP32_S3 = UINT8_C(2),
    SRP_NODE_BROADCAST = UINT8_C(3)
};

#pragma pack(push, 4)
/** 线缆固定头布局说明；仅描述字段，不得直接 memcpy 到 UART。 */
typedef struct SRP_PACKED_ALIGNED(4) {
    uint16_t magic; /**< 小端魔数 SRP_MAGIC，在线缆上占 2 byte。 */
    uint16_t length; /**< payload 字节数，不含头、CRC 和 EOF。 */
    uint32_t header; /**< priority/type/sequence/flags 组合值，按小端显式编码。 */
} srp_wire_header_t;

/** 线缆固定尾布局说明；CRC 与 EOF 均需按小端显式序列化。 */
typedef struct SRP_PACKED_ALIGNED(2) {
    uint16_t crc16; /**< 从 length 字段到 payload 末尾的 CRC-16/CCITT-FALSE。 */
    uint16_t eof; /**< 固定帧尾 SRP_EOF，在线缆上占 2 byte。 */
} srp_wire_trailer_t;

/* This is a logical view containing a pointer, not a serialized wire block.
 * Keep natural field alignment so parser/link callbacks cannot fault on an
 * unaligned pointer access. */
/**
 * 逻辑帧视图，不是可直接发送的线缆块。payload 指针只借用输入缓冲区。
 */
typedef struct {
    uint8_t priority; /**< srp_priority_t 数值；解码后由输入 header 提供。 */
    uint8_t type; /**< 8 位 SRP_MSG_ID_* 消息类型。 */
    uint8_t sequence; /**< 发送序号，模 256 自然回绕；用于 ACK 匹配。 */
    uint8_t flags; /**< SRP_FLAG_* 位集合；高四位必须为 0。 */
    uint16_t length; /**< payload 长度，单位 byte，范围 0..SRP_MAX_PAYLOAD。 */
    const uint8_t *payload; /**< 借用的 payload 视图；不由 srp_frame_t 释放或复制。 */
} srp_frame_t;
#pragma pack(pop)

_Static_assert(sizeof(srp_wire_header_t) == 8U, "SRP header size");
_Static_assert(sizeof(srp_wire_trailer_t) == 4U, "SRP trailer size");
_Static_assert(offsetof(srp_wire_header_t, magic) == 0U,
               "SRP magic offset");
_Static_assert(offsetof(srp_wire_header_t, length) == 2U,
               "SRP length offset");
_Static_assert(offsetof(srp_wire_header_t, header) == 4U,
               "SRP header offset");
_Static_assert(offsetof(srp_wire_trailer_t, crc16) == 0U,
               "SRP CRC offset");
_Static_assert(offsetof(srp_wire_trailer_t, eof) == 2U,
               "SRP EOF offset");
_Static_assert(_Alignof(srp_wire_header_t) >= 4U, "SRP header alignment");
_Static_assert(_Alignof(srp_wire_trailer_t) >= 2U, "SRP trailer alignment");

#ifdef __cplusplus
}
#endif

#endif /* SRP_DEF_H */
