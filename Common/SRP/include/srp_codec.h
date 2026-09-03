#ifndef SRP_CODEC_H
#define SRP_CODEC_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "srp_def.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * SRP 编解码器只处理内存中的完整帧和增量字节流，不拥有 UART、锁或队列。
 * 创建人：待确认（当前维护人：Zhiqin）。
 * 所有 payload 指针均为借用引用；跨任务保存前必须复制数据。
 */

/** SRP 完整帧编解码结果；非零值均表示本次输出或解码视图不可使用。 */
typedef enum {
    SRP_CODEC_OK = 0,                 /**< 编解码成功，全部输出字段有效。 */
    SRP_CODEC_INVALID_ARGUMENT = 1,   /**< 必需指针、payload 组合或逻辑帧参数非法。 */
    SRP_CODEC_INVALID_LENGTH,         /**< payload 或完整线缆帧长度不符合 SRP v4 边界。 */
    SRP_CODEC_INVALID_HEADER,         /**< 优先级或 header 保留标志不合法。 */
    SRP_CODEC_BAD_MAGIC,              /**< 线缆帧起始魔数不是 SRP_MAGIC。 */
    SRP_CODEC_BAD_CRC,                /**< 接收 CRC 与按 CCITT-FALSE 计算值不一致。 */
    SRP_CODEC_BAD_EOF,                /**< 帧尾字节不是 SRP_EOF。 */
    SRP_CODEC_OVERFLOW                /**< 调用方输出缓冲容量不足，未得到完整帧。 */
} srp_codec_status_t;

/** SRP 增量解析器当前所处的字节流状态。 */
typedef enum {
    SRP_PARSER_WAIT_MAGIC0 = 0, /**< 等待魔数首字节 0xAA。 */
    SRP_PARSER_WAIT_MAGIC1,     /**< 已见 0xAA，等待魔数第二字节 0x55。 */
    SRP_PARSER_READ_HEADER,     /**< 收集固定 8 字节头并推导完整帧长。 */
    SRP_PARSER_READ_BODY        /**< 收集 payload、CRC 和 EOF，完成后同步校验。 */
} srp_parser_state_t;

/** 增量解析失败分类；回调收到后只能把 data 当作调用期间的诊断片段。 */
typedef enum {
    SRP_PARSER_ERROR_MAGIC = 1, /**< 魔数第二字节不匹配，解析器重新同步。 */
    SRP_PARSER_ERROR_LENGTH,    /**< 帧内 payload 长度超限或完整长度不合法。 */
    SRP_PARSER_ERROR_HEADER,    /**< 优先级或 header 保留位非法。 */
    SRP_PARSER_ERROR_CRC,       /**< 完整候选帧 CRC 校验失败。 */
    SRP_PARSER_ERROR_EOF,       /**< 完整候选帧 EOF 校验失败。 */
    SRP_PARSER_ERROR_OVERFLOW   /**< 内部候选帧存储不足，当前半帧被丢弃。 */
} srp_parser_error_t;

#pragma pack(push, 4)
/** 单 owner 的 SRP 增量解析状态；内部拥有候选帧字节，回调和 context 仅借用。 */
typedef struct {
    srp_parser_state_t state; /**< 当前字节流状态，不是跨芯片会话状态。 */
    uint16_t index; /**< bytes 中已缓存的候选帧字节数，也是下一写入偏移。 */
    uint16_t expected_length; /**< 从头部推导的完整线缆帧长度，单位 byte；未知时为 0。 */
    uint32_t frame_count; /**< 已完整校验的累计帧数；即使未注册回调也增加，自然回绕。 */
    uint32_t crc_error_count; /**< CRC 错误累计次数；自然回绕，仅供诊断。 */
    uint32_t eof_error_count; /**< EOF 错误累计次数；自然回绕，仅供诊断。 */
    uint32_t length_error_count; /**< 长度或内部溢出累计次数；自然回绕。 */
    /* Last byte/state associated with a header or length rejection. These
     * fields are diagnostic only and are not part of the SRP wire format. */
    srp_parser_state_t last_error_state; /**< 最近一次魔数/header/长度拒绝发生时的状态快照。 */
    uint8_t last_drop_byte; /**< 最近一次魔数/header/长度拒绝的触发字节，仅供定位。 */
    _Alignas(4) uint8_t bytes[SRP_MAX_FRAME_SIZE]; /**< 解析器拥有的候选帧存储；回调返回后可被覆盖。 */
    void (*frame_callback)(const srp_frame_t *frame, void *context); /**< 借用的同步完整帧回调，可为 NULL。 */
    void (*error_callback)(srp_parser_error_t error, const uint8_t *data,
                           size_t length, void *context); /**< 借用的同步错误回调，可为 NULL。 */
    void *context; /**< 原样传给两个回调的借用上下文；解析器不释放。 */
} srp_parser_t;

/** 只读 TLV 游标；data 由调用方拥有，迭代器只推进字节偏移。 */
typedef struct {
    const uint8_t *data; /**< 借用的 TLV payload 起始地址，迭代期间必须有效且不变。 */
    size_t length; /**< data 的总字节数。 */
    size_t offset; /**< 下一 TLV tag 的字节偏移；到达 length 表示迭代结束。 */
} srp_tlv_iter_t;
#pragma pack(pop)

/**
 * @brief 将逻辑帧编码为 SRP v4 线缆帧。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * @param frame 待编码逻辑帧；payload 由调用方拥有，不能为野指针。
 * @param out 输出缓冲区；容量至少为 SRP_MAX_FRAME_SIZE。
 * @param capacity out 的实际容量，防止写越界。
 * @param out_length 成功时写入实际帧长；失败时可保持原值。
 * @return `SRP_CODEC_OK`（0）或 `srp_codec_status_t` 错误码。
 * 调用方式：普通任务/测试上下文；不访问硬件、不阻塞。输出帧可直接交给传输层。
 * 线程约束：纯内存操作；不同输出/对象可并发，同一 out 缓冲不得并发写。
 */
int srp_encode(const srp_frame_t *frame, uint8_t *out, size_t capacity,
               uint16_t *out_length);

/**
 * @brief 统一的 SRP 帧编码入口，语义与 srp_encode 相同。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * 传入参数：frame/out/capacity/out_length 的所有权、容量和失败语义同 srp_encode()。
 * @return SRP_CODEC_OK 或与 srp_encode() 相同的 srp_codec_status_t 错误码。
 * 调用方式：新代码优先使用本函数，以避免不同传输层出现编码分叉。
 * 线程约束：纯内存操作；不同缓冲可并发，同一 out 不得并发写。
 */
int srp_encode_frame(const srp_frame_t *frame, uint8_t *out, size_t capacity,
                     uint16_t *out_length);

/**
 * @brief 校验并解析一条完整 SRP 线缆帧。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * @param data 完整帧起始地址，包含头、payload、CRC 和 EOF。
 * @param length 实际字节数，必须与头部长度一致。
 * @param frame 输出逻辑视图；payload 指向 data 内部，不能脱离 data 生命周期。
 * @return `SRP_CODEC_OK` 或具体格式/CRC/长度错误。
 * 调用方式：任务上下文或测试上下文；调用方应在错误时触发丢帧/恢复策略。
 * 线程约束：纯内存操作；frame->payload 借用 data，data 在消费完成前不得修改。
 */
int srp_decode(const uint8_t *data, size_t length, srp_frame_t *frame);

/**
 * @brief 初始化增量解析器。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * @param parser 调用方提供的长期存储，不可为 NULL。
 * @param frame_callback 完整帧回调；可为 NULL，但将不会上报帧。
 * @param error_callback 错误回调；可为 NULL。
 * @param context 原样传给回调的上下文指针，不由解析器释放。
 * @return 无；parser=NULL 时不写入。
 * 调用方式：在创建 UART/网络接收任务前调用一次；回调通常只做复制或入队。
 * 线程约束：同一 parser 只能由一个接收 owner 初始化/喂入；禁止与 feed 并发。
 */
void srp_parser_init(srp_parser_t *parser,
                     void (*frame_callback)(const srp_frame_t *frame,
                                             void *context),
                     void (*error_callback)(srp_parser_error_t error,
                                            const uint8_t *data, size_t length,
                                            void *context),
                     void *context);
/**
 * @brief 丢弃当前半帧并回到等待魔数状态。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * 传入参数：parser 为已初始化对象；NULL 时安全返回。
 * 返回值：无；保留回调、上下文和累计诊断计数。
 * 调用方式：接收流发生明确不连续、重同步或链路恢复时由 parser owner 调用。
 * 线程约束：与 srp_parser_feed() 串行调用。
 */
void srp_parser_reset(srp_parser_t *parser);

/**
 * @brief 向解析器喂入一段连续字节。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * @param parser 已初始化的解析器。
 * @param data 输入字节；length 为 0 时可为 NULL。
 * @param length 本次输入长度。
 * @return 实际消费的字节数；错误帧也会被消费并通过回调报告。
 * 调用方式：可在 UART 任务中重复调用；不要在 ISR 中执行回调所属的业务逻辑。
 * 线程约束：单 owner、不可重入；回调在本函数调用栈内同步执行。
 */
size_t srp_parser_feed(srp_parser_t *parser, const uint8_t *data, size_t length);

/**
 * @brief 初始化 TLV 只读迭代器。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * 传入参数：iterator 为可写状态；data/length 为借用 payload。
 * 返回值：无；iterator=NULL 时安全返回。
 * 调用方式：开始解析一条已验证的 TLV payload 前调用，随后循环 srp_tlv_next()。
 * 线程约束：迭代期间 data 必须保持有效且不变。
 */
void srp_tlv_iter_init(srp_tlv_iter_t *iterator, const uint8_t *data,
                       size_t length);

/**
 * @brief 读取下一个 TLV 项。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * @param iterator 已初始化的迭代器。
 * @param tag/value_length/value 输出字段；value 指向原输入，不会复制。
 * @return true 表示成功读取一项；false 表示结束或格式错误。
 * 调用方式：循环调用直至 false；不要保存 value 超过原始 payload 生命周期。
 * 线程约束：修改 iterator->offset，同一 iterator 不可并发使用。
 */
bool srp_tlv_next(srp_tlv_iter_t *iterator, uint8_t *tag, uint8_t *value_length,
                  const uint8_t **value);

#ifdef __cplusplus
}
#endif

#endif /* SRP_CODEC_H */
