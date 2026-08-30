#ifndef SRP_DEF_H
#define SRP_DEF_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#if defined(__GNUC__) || defined(__clang__)
#define SRP_PACKED __attribute__((packed))
#define SRP_PACKED_ALIGNED(alignment) \
    __attribute__((packed, aligned(alignment)))
#else
#define SRP_PACKED
#define SRP_PACKED_ALIGNED(alignment)
#endif

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

typedef enum {
    SRP_PRIORITY_EMERGENCY = 0,
    SRP_PRIORITY_COMMAND = 1,
    SRP_PRIORITY_TELEMETRY = 2,
    SRP_PRIORITY_LOG = 3
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
typedef struct SRP_PACKED_ALIGNED(4) {
    uint16_t magic;
    uint16_t length;
    uint32_t header;
} srp_wire_header_t;

typedef struct SRP_PACKED_ALIGNED(2) {
    uint16_t crc16;
    uint16_t eof;
} srp_wire_trailer_t;

/* This is a logical view containing a pointer, not a serialized wire block.
 * Keep natural field alignment so parser/link callbacks cannot fault on an
 * unaligned pointer access. */
typedef struct {
    uint8_t priority;
    uint8_t type;
    uint8_t sequence;
    uint8_t flags;
    uint16_t length;
    const uint8_t *payload;
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
