#ifndef S3_RADAR_PARSER_H
#define S3_RADAR_PARSER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*
 * YDLIDAR X3/X3PRO 原始流解析器。
 * 创建人：待确认（当前维护人：Zhiqin）。
 * 解析器只负责定界、校验和原始帧回调；字段未确认时不会伪造测量点。
 */

/*
 * X3PRO 扫描包采用小端序并以 AA 55 开始（协议值 0x55AA）。包头包含 CT、
 * LSN、FSA、LSA 和 CS；LSN 表示十字节包头之后的采样数量。
 */
#define RADAR_X3PRO_HEADER_BYTE_0 0xAAU
#define RADAR_X3PRO_HEADER_BYTE_1 0x55U
#define RADAR_X3PRO_LENGTH_OFFSET 3U
#define RADAR_X3PRO_HEADER_BYTES 10U
#define RADAR_X3PRO_SAMPLE_BYTES_AUTO 0U
#define RADAR_X3PRO_SAMPLE_BYTES 2U
#define RADAR_X3PRO_MAX_SAMPLE_BYTES 3U
#define RADAR_X3PRO_MAX_SAMPLES 255U

#define RADAR_PARSER_RING_BUFFER_SIZE 4096U
#define RADAR_PARSER_MAX_FRAME_SIZE \
    (RADAR_X3PRO_HEADER_BYTES + \
     (RADAR_X3PRO_MAX_SAMPLES * RADAR_X3PRO_MAX_SAMPLE_BYTES))

/**
 * @brief  增量解析器发现一条校验通过的完整原始帧时的同步回调。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（接口契约补充）。
 * @param  data 解析器栈上临时帧缓冲，只在本次回调返回前有效，接收方必须复制。
 * @param  length 完整帧字节数，范围不超过 RADAR_PARSER_MAX_FRAME_SIZE。
 * @param  context radar_parser_feed() 传入的原样上下文，可为 NULL。
 * @return 无。
 * 调用方式：由 radar_parser_feed() 同步触发；回调只做有界复制/计数，不得保存 data 指针。
 * 线程约束：运行在 feed 调用者上下文，不是 UART ISR；不得递归 feed 同一个 parser 或无限阻塞。
 */
typedef void (*radar_frame_callback_t)(const uint8_t *data,
                                       size_t length,
                                       void *context);

/*
 * 默认校验器实现 YDLIDAR 文档中的 X3/X3PRO XOR 校验。调用方可以安装变体
 * 校验器而不改变环形缓冲和定界逻辑；传入 NULL 恢复默认校验器。
 */
/**
 * @brief  为设备变体提供完整候选帧校验函数。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（接口契约补充）。
 * @param  frame 候选帧的临时只读缓冲，仅在回调期间有效。
 * @param  length 候选帧长度。
 * @param  context set_checksum_validator() 注册的借用上下文。
 * @return true 表示候选帧校验通过；false 触发继续等待三字节格式或重新同步。
 * 调用方式：由 feed 同步调用；校验器负责变体完整性，不得修改 frame 或 parser。
 * 线程约束：与 feed 相同，无阻塞、不可递归；context 生命周期覆盖注册使用期。
 */
typedef bool (*radar_parser_checksum_validator_t)(const uint8_t *frame,
                                                   size_t length,
                                                   void *context);

/** YDLIDAR 字节流解析累计统计；计数为 32 位自然回绕且不代表物理测量已解码。 */
typedef struct {
    uint32_t valid_frame_count; /**< 通过定界、结构和校验并交付回调的完整帧累计数。 */
    uint32_t valid_distance_frame_count; /**< 其中每样本 2 byte 距离格式的累计帧数。 */
    uint32_t valid_intensity_frame_count; /**< 其中每样本 3 byte 强度格式的累计帧数。 */
    uint32_t checksum_error_count; /**< 完整候选长度已具备但校验失败的累计次数。 */
    uint32_t invalid_frame_count; /**< LSN、检查位或固定格式长度非法的累计候选数。 */
    uint32_t header_resync_count; /**< 丢噪声或候选失败后重新寻找 AA55 的累计动作数。 */
    uint32_t overflow_count; /**< 环形缓冲满时被覆盖丢弃的最旧字节累计数。 */
    uint8_t last_sample_bytes; /**< 最近有效帧的每样本字节数 2/3；尚无有效帧时为 0。 */
} radar_parser_stats_t;

/** 为未来测量点解码保留的结果；当前实现只清零并保持 valid=false。 */
typedef struct {
    bool valid; /**< 字段是否由已冻结测量协议成功解码；当前始终为 false。 */
    uint16_t angle_cdeg; /**< 预留角度，单位 0.01 degree；valid=false 时不得使用。 */
    uint16_t distance_mm; /**< 预留距离，单位 mm；valid=false 时不得使用。 */
    uint16_t quality; /**< 预留质量值，量纲尚未冻结；valid=false 时不得使用。 */
} radar_measurement_t;

/** 单 owner 的雷达增量解析器；拥有环形字节与统计，校验回调/context 仅借用。 */
typedef struct {
    uint8_t buffer[RADAR_PARSER_RING_BUFFER_SIZE]; /**< 解析器拥有的原始 UART 环形缓冲。 */
    size_t head; /**< 下一输入字节写入索引。 */
    size_t tail; /**< 当前最旧未处理字节索引。 */
    size_t size; /**< 环形缓冲当前未处理字节数。 */
    size_t sample_bytes; /**< 配置的每样本字节数：0 自动、2 距离、3 强度。 */
    radar_parser_checksum_validator_t checksum_validator; /**< 借用的候选帧校验回调；init 默认使用文档 XOR。 */
    void *checksum_context; /**< 原样传给自定义校验器的借用上下文，可为 NULL。 */
    radar_parser_stats_t stats; /**< 解析器拥有的累计统计；reset_stream 不清零。 */
} radar_parser_t;

/**
 * @brief  清零解析器环形流/统计，并选择自动采样格式和默认 XOR 校验器。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（接口契约补充）。
 * @param[out] parser 可为 NULL；非 NULL 时整体重置，既有统计和自定义校验器会丢失。
 * @return 无。
 * 调用方式：UART 接收任务启动前调用；parser 必须由调用方长期持有且不能移动其内部缓冲。
 * 线程约束：纯内存操作、无锁；初始化时不得与 feed/get_stats 并发。
 */
void radar_parser_init(radar_parser_t *parser);

/**
 * @brief  丢弃当前未完成字节流并重新等待 AA 55 帧头。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（接口契约补充）。
 * @param  parser 可为 NULL；非 NULL 时只清 head/tail/size。
 * @return 无；累计统计、采样格式和自定义校验器保持不变。
 * 调用方式：UART FIFO 溢出/驱动缓冲丢失后调用，防止跨缺口拼接伪帧。
 * 线程约束：无锁；必须与 feed 串行执行，禁止从 UART ISR 直接调用同一实例。
 */
void radar_parser_reset_stream(radar_parser_t *parser);

/**
 * @brief  安装设备变体校验器，或恢复默认 X3/X3PRO XOR 校验。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（接口契约补充）。
 * @param  parser 解析器实例；NULL 时不动作。
 * @param  validator 自定义回调；NULL 表示恢复默认实现。
 * @param  context 原样传给自定义回调；不转移所有权，默认校验器会忽略它。
 * @return 无。
 * 调用方式：初始化后、喂入字节前设置；变更时先 reset_stream，避免同一半帧跨校验策略。
 * 线程约束：无锁；不得与 feed 并发，context 生命周期必须覆盖所有后续 feed。
 */
void radar_parser_set_checksum_validator(
    radar_parser_t *parser,
    radar_parser_checksum_validator_t validator,
    void *context);

/* 自动模式会结合校验测试文档规定的 2/3 字节采样格式；设备抓包确认后可锁定。 */
/**
 * @brief  选择自动、2 字节距离或 3 字节强度采样布局。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（接口契约补充）。
 * @param  parser 非 NULL 的解析器实例。
 * @param  sample_bytes 仅允许 AUTO(0)、2 或 3。
 * @return true 表示配置已更新；实例为空或值不支持返回 false，原配置保持不变。
 * 调用方式：设备抓包确认前保持 AUTO；运行期切换前先 reset_stream，函数自身不会丢弃半帧。
 * 线程约束：无锁；不得与 feed 并发。
 */
bool radar_parser_set_sample_bytes(radar_parser_t *parser, size_t sample_bytes);

/**
 * @brief  复制有效帧、校验错误、重同步和环形溢出统计。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（接口契约补充）。
 * @param  parser 可为 NULL；NULL 时输出全零。
 * @param[out] stats 可为 NULL；非 NULL 时先清零再复制统计。
 * @return 无。
 * 调用方式：健康日志低频读取；统计只证明本地解析结果，不证明物理量解码或主机接收。
 * 线程约束：无内部锁；与 feed 并发读取需由调用方串行化以获得一致快照。
 */
void radar_parser_get_stats(const radar_parser_t *parser,
                            radar_parser_stats_t *stats);

/* 独立于流状态校验完整原始包，并报告文档规定的 2/3 字节采样布局。 */
/**
 * @brief  使用默认 X3/X3PRO 规则独立校验一条完整原始帧。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（接口契约补充）。
 * @param  frame 非 NULL 的完整帧缓冲；函数不保留指针。
 * @param  length 必须精确匹配 LSN 推导的 2/3 字节采样帧长度。
 * @param[out] sample_bytes 可为 NULL；成功时写入检测到的 2 或 3。
 * @return true 表示帧头、LSN、检查位、长度和默认 XOR 均通过；否则 false。
 * 调用方式：协议封装/主机测试复验完整帧；不使用 parser 实例安装的自定义校验器。
 * 线程约束：纯只读计算、可重入、无阻塞；大帧校验不应放入 ISR。
 */
bool radar_parser_validate_frame(const uint8_t *frame,
                                 size_t length,
                                 size_t *sample_bytes);

/**
 * @brief  将连续 UART 字节追加到环形缓冲并同步提取所有可用完整帧。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（接口契约补充）。
 * @param  parser 非 NULL、已初始化的解析器实例。
 * @param  data 输入字节；length 大于 0 时必须非 NULL，函数返回前完成读取。
 * @param  length 输入字节数；可为 0。
 * @param  callback 可为 NULL；非 NULL 时每条有效帧同步调用一次。
 * @param  context 原样传给 callback，不由解析器拥有。
 * @return 无；环形满时丢最旧字节并增加 overflow_count，接口不返回逐帧错误。
 * 调用方式：由单一雷达 UART 任务按接收块调用；回调必须在返回前复制帧数据。
 * 线程约束：会修改 parser 且使用较大的栈上帧缓冲；不可重入、不可并发，禁止 ISR 调用。
 */
void radar_parser_feed(radar_parser_t *parser,
                       const uint8_t *data,
                       size_t length,
                       radar_frame_callback_t callback,
                       void *context);

/* 在点字段变体确认前保留；绝不伪造角度、距离或质量值。 */
/**
 * @brief  预留的测量点解码入口，当前不会输出角度/距离/质量。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（接口契约补充）。
 * @param  frame 候选完整帧，可为 NULL。
 * @param  length frame 长度。
 * @param[out] measurement 输出对象；非 NULL 时首先整体清零。
 * @return 当前实现固定返回 false，包括帧基本格式有效时；不得据输出构造 LaserScan。
 * 调用方式：仅保留接口兼容，待真实设备字段和主机协议联合冻结后再实现。
 * 线程约束：纯内存操作、可重入；不应在 ISR 中调用无意义的预留解码。
 */
bool radar_parser_parse_measurement(const uint8_t *frame,
                                    size_t length,
                                    radar_measurement_t *measurement);

/**
 * @brief  将字节序列格式化为以空格分隔的大写十六进制诊断字符串。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（接口契约补充）。
 * @param  data 输入字节；NULL 时产生空字符串。
 * @param  length 希望格式化的字节数。
 * @param[out] output 输出字符缓冲；NULL 时不动作。
 * @param  output_size 缓冲容量；大于 0 时保证 NUL 结尾，空间不足时安全截断。
 * @return 无；格式化失败时输出空字符串，接口不报告截断位置。
 * 调用方式：仅用于有节流的诊断路径；生产高速流不要对整帧频繁格式化。
 * 线程约束：只使用调用方缓冲、可重入；snprintf 成本不适合 ISR/实时回调。
 */
void radar_parser_format_hex(const uint8_t *data,
                             size_t length,
                             char *output,
                             size_t output_size);

#endif
