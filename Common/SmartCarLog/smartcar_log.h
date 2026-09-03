#ifndef SMARTCAR_LOG_H
#define SMARTCAR_LOG_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 独立日志封装，不等同于 SRP 控制帧。
 * 创建人：待确认（当前维护人：Zhiqin）。
 * 日志接口只复制有限长度文本，不应在 ISR 中调用，也不能替代安全状态上报。
 */

/* 此日志包络与 App 控制帧和 STM32-S3 SRP v4 帧相互独立。 */
#define SMARTCAR_LOG_HEAD_0 UINT8_C(0xAA)
#define SMARTCAR_LOG_HEAD_1 UINT8_C(0x55)
#define SMARTCAR_LOG_VERSION UINT8_C(0x01)
#define SMARTCAR_LOG_MAX_PAYLOAD UINT8_C(96)
#define SMARTCAR_LOG_HEADER_SIZE UINT8_C(10)
#define SMARTCAR_LOG_CRC_SIZE UINT8_C(2)
#define SMARTCAR_LOG_FRAME_OVERHEAD \
    (SMARTCAR_LOG_HEADER_SIZE + SMARTCAR_LOG_CRC_SIZE)
#define SMARTCAR_LOG_MAX_FRAME_SIZE \
    (SMARTCAR_LOG_FRAME_OVERHEAD + SMARTCAR_LOG_MAX_PAYLOAD)

/** 独立日志包络的来源标识；只表示记录生产端，不代表链路方向。 */
typedef enum {
    SMARTCAR_LOG_SOURCE_STM32 = 0U, /**< 日志由 STM32H757 侧产生。 */
    SMARTCAR_LOG_SOURCE_S3 = 1U     /**< 日志由 ESP32-S3 侧产生。 */
} smartcar_log_source_t;

/** 日志严重级别；数值越大表示越需要保留和处理。 */
typedef enum {
    SMARTCAR_LOG_LEVEL_DEBUG = 0U, /**< 高频调试信息，资源紧张时可优先丢弃。 */
    SMARTCAR_LOG_LEVEL_INFO = 1U,  /**< 正常状态迁移或低频运行信息。 */
    SMARTCAR_LOG_LEVEL_WARN = 2U,  /**< 可恢复异常或降级告警。 */
    SMARTCAR_LOG_LEVEL_ERROR = 3U  /**< 失败/故障信息；仍不能替代安全状态通道。 */
} smartcar_log_level_t;

/** 日志编解码状态；非 OK 时输出长度或记录视图不得作为新结果使用。 */
typedef enum {
    SMARTCAR_LOG_OK = 0,            /**< 编码或解码成功。 */
    SMARTCAR_LOG_INVALID_ARG,       /**< 必需指针、来源、级别或 payload 指针组合非法。 */
    SMARTCAR_LOG_BUFFER_TOO_SMALL,  /**< 编码输出容量小于完整包络所需字节数。 */
    SMARTCAR_LOG_PAYLOAD_TOO_LARGE, /**< payload 超过 SMARTCAR_LOG_MAX_PAYLOAD。 */
    SMARTCAR_LOG_INVALID_FRAME,     /**< 魔数、版本、来源、级别或完整长度非法。 */
    SMARTCAR_LOG_CRC_MISMATCH       /**< 接收 CRC-16/MODBUS 与计算值不一致。 */
} smartcar_log_status_t;

/** 已解码日志逻辑视图；payload 借用输入 frame，不拥有或释放其存储。 */
typedef struct {
    smartcar_log_source_t source; /**< 日志产生端。 */
    smartcar_log_level_t level; /**< 日志严重级别。 */
    uint32_t timestamp_ms; /**< 产生端单调时间戳，单位 ms；跨芯片不保证同一时基。 */
    const uint8_t *payload; /**< 借用的日志文本/字节内容，指向输入帧内部。 */
    uint8_t payload_length; /**< payload 有效长度，单位 byte，最大 96。 */
} smartcar_log_record_t;

/**
 * @brief 计算日志帧使用的 CRC-16/MODBUS。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * @param data 输入字节；length=0 时可为 NULL。
 * @param length 输入长度。
 * @return 初值 0xFFFF、多项式 0xA001 的 CRC；data=NULL 且 length>0 时返回 0。
 * 调用方式：日志编码/解码或主机测试调用；不能替代 SRP v4 的 CCITT-FALSE CRC。
 * 线程约束：纯函数、可重入、不阻塞、不分配内存；ISR 中避免处理无界长输入。
 */
uint16_t smartcar_log_crc16_modbus(const uint8_t *data, size_t length);

/**
 * @brief 编码一条日志记录。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * @param source 日志来源（STM32 或 S3）。
 * @param level 日志级别。
 * @param timestamp_ms 来源单调毫秒时间戳。
 * @param payload 文本字节；length=0 时允许 NULL，不得与 output 缓冲重叠。
 * @param payload_length 文本长度，不能超过 SMARTCAR_LOG_MAX_PAYLOAD。
 * @param output 可写完整帧缓冲，容量至少为固定开销加 payload_length。
 * @param output_capacity output 的实际字节容量。
 * @param output_length 成功时写入完整帧长度；失败时保持调用前内容，不得读取为新值。
 * @return SMARTCAR_LOG_OK，或 INVALID_ARG/PAYLOAD_TOO_LARGE/BUFFER_TOO_SMALL。
 * 调用方式：普通任务或主机测试调用；CRC 覆盖 version 至 payload，不含 AA55 和 CRC 字段。
 * 线程约束：不使用全局可变状态，可重入且不分配内存；禁止在 ISR 中编码日志。
 */
smartcar_log_status_t smartcar_log_encode(
    smartcar_log_source_t source,
    smartcar_log_level_t level,
    uint32_t timestamp_ms,
    const uint8_t *payload,
    uint8_t payload_length,
    uint8_t *output,
    size_t output_capacity,
    size_t *output_length);

/**
 * @brief 校验并解析一条完整日志帧。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * @param frame 只读完整帧；成功后 record->payload 借用其内部空间。
 * @param frame_length 必须精确等于包络固定开销加帧内 payload_length。
 * @param record 可写逻辑记录；失败时保持调用前内容，不拥有 payload 存储。
 * @return SMARTCAR_LOG_OK，或 INVALID_ARG/INVALID_FRAME/CRC_MISMATCH。
 * 调用方式：解析任务在取得完整候选帧后调用；record 的 payload 仅在 frame 有效期内可读。
 * 线程约束：纯解析、可重入、不分配内存；禁止从 ISR 调用日志处理链。
 */
smartcar_log_status_t smartcar_log_decode(
    const uint8_t *frame,
    size_t frame_length,
    smartcar_log_record_t *record);

#ifdef __cplusplus
}
#endif

#endif /* SMARTCAR_LOG_H */
