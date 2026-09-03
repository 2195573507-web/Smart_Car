#ifndef S3_BLE_LOG_TX_H
#define S3_BLE_LOG_TX_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "smartcar_log.h"

#ifdef __cplusplus
extern "C" {
#endif

#define S3_BLE_LOG_TX_NORMAL_CAPACITY 40U
#define S3_BLE_LOG_TX_CRITICAL_CAPACITY 8U
#define S3_BLE_LOG_TX_CRITICAL_BURST_MAX 4U
#define S3_BLE_LOG_TX_MAX_FRAME_SIZE SMARTCAR_LOG_MAX_FRAME_SIZE

/** BLE 日志发送队列优先级；仅影响本地调度，不改变日志帧内 level。 */
typedef enum {
    S3_BLE_LOG_TX_PRIORITY_NORMAL = 0, /**< 普通日志队列；在连续关键帧达到上限后获得发送机会。 */
    S3_BLE_LOG_TX_PRIORITY_CRITICAL    /**< 关键日志队列；优先发送但受四帧 burst 公平限制。 */
} s3_ble_log_tx_priority_t;

/** 准备下一 BLE notification 分片的结果；只有 READY 时 chunk 输出有效。 */
typedef enum {
    S3_BLE_LOG_TX_PREPARE_READY = 0,  /**< 已复制一个分片并登记非零 in-flight token。 */
    S3_BLE_LOG_TX_PREPARE_EMPTY,      /**< active 与两个等待队列均无可发送帧。 */
    S3_BLE_LOG_TX_PREPARE_PAUSED,     /**< 未连接、CCC 未开启或当前拥塞，队列内容保留。 */
    S3_BLE_LOG_TX_PREPARE_BUSY,       /**< 已有一个分片等待 complete，禁止并行准备。 */
    S3_BLE_LOG_TX_PREPARE_INVALID_ARG /**< tx/chunk 为空或最大分片长度为 0。 */
} s3_ble_log_tx_prepare_result_t;

/** 完成一个已准备分片后的状态；STALE 不消费当前有效 in-flight 分片。 */
typedef enum {
    S3_BLE_LOG_TX_COMPLETE_MORE = 0,          /**< 分片发送成功，但 active 帧仍有后续字节。 */
    S3_BLE_LOG_TX_COMPLETE_FRAME_DONE,        /**< 最后分片成功，active 帧已计入 sent_frames。 */
    S3_BLE_LOG_TX_COMPLETE_FRAME_DROPPED,     /**< 分片发送失败，当前 active 帧被整体放弃。 */
    S3_BLE_LOG_TX_COMPLETE_ABORTED,           /**< 准备后因断连/关闭 CCC 使 active 帧失效。 */
    S3_BLE_LOG_TX_COMPLETE_STALE              /**< 无在途分片或 token 为 0/不匹配，状态未推进。 */
} s3_ble_log_tx_complete_result_t;

/** BLE 日志调度累计统计；所有计数使用饱和加法，不代表手机已持久化日志。 */
typedef struct {
    uint32_t queued; /**< 成功接收入队的累计帧数；队满覆盖后接收的新帧也计入。 */
    uint32_t sent_frames; /**< 最后分片完成成功的累计帧数。 */
    uint32_t sent_chunks; /**< complete 回报成功的累计 notification 分片数。 */
    uint32_t drop_normal; /**< 普通队列满时被覆盖丢弃的最旧帧累计数。 */
    uint32_t drop_critical; /**< 关键队列满时被覆盖丢弃的最旧帧累计数。 */
    uint32_t send_fail; /**< complete 回报发送失败的累计分片数。 */
    uint32_t congest_events; /**< set_congested() 调用累计次数；包含解除拥塞和重复赋值。 */
    uint32_t partial_drop; /**< 断连/关闭 CCC 时已有在途分片或成功前缀，或发送失败时已有成功前缀的 active 帧累计数。 */
    uint32_t current_depth; /**< 两个等待队列当前帧数之和，不包含 active 帧。 */
    uint32_t high_watermark; /**< 初始化以来 current_depth 的历史最大值。 */
} s3_ble_log_tx_stats_t;

/** 一次待提交 BLE notification 的独立字节副本；由调用方在 complete 前持有。 */
typedef struct {
    uint32_t token; /**< 非零在途标识；32 位回绕时跳过 0，用于拒绝迟到完成事件。 */
    uint16_t frame_offset; /**< 本分片在 active 日志帧中的起始偏移，单位 byte。 */
    uint16_t length; /**< data 有效字节数，不超过 max_chunk_size 和剩余帧长。 */
    bool frame_end; /**< true 表示本分片覆盖 active 帧最后一个字节。 */
    uint8_t data[S3_BLE_LOG_TX_MAX_FRAME_SIZE]; /**< 分片自有副本，仅前 length 字节有效。 */
} s3_ble_log_tx_chunk_t;

/** 队列内部的一条完整 SmartCarLog 帧；结构体自身拥有 data 副本。 */
typedef struct {
    uint16_t length; /**< data 有效完整帧长度，单位 byte，范围 1..最大日志帧。 */
    uint8_t data[S3_BLE_LOG_TX_MAX_FRAME_SIZE]; /**< 完整日志包络存储，仅前 length 字节有效。 */
} s3_ble_log_tx_frame_t;

/**
 * BLE 日志双优先级队列与单在途分片状态；对象拥有全部帧存储但不含锁。
 * 固件 owner 必须用同一短 portMUX 临界区包住每次操作，host test 只可串行调用。
 */
typedef struct {
    s3_ble_log_tx_frame_t normal[S3_BLE_LOG_TX_NORMAL_CAPACITY]; /**< 对象拥有的普通帧环形队列。 */
    s3_ble_log_tx_frame_t critical[S3_BLE_LOG_TX_CRITICAL_CAPACITY]; /**< 对象拥有的关键帧环形队列。 */
    s3_ble_log_tx_frame_t active; /**< 已从等待队列取出的当前帧副本；由 active_valid 判定有效。 */
    uint8_t normal_head; /**< 下一普通帧写入索引。 */
    uint8_t normal_tail; /**< 最旧待发普通帧索引。 */
    uint8_t normal_count; /**< 普通等待队列当前帧数，范围 0..40。 */
    uint8_t critical_head; /**< 下一关键帧写入索引。 */
    uint8_t critical_tail; /**< 最旧待发关键帧索引。 */
    uint8_t critical_count; /**< 关键等待队列当前帧数，范围 0..8。 */
    uint8_t critical_burst_count; /**< 自上次普通帧以来连续选择的关键帧数，最大保持 4。 */
    bool connected; /**< 外层登记的 BLE 连接状态；false 时不能 prepare。 */
    bool ccc_enabled; /**< 手机是否开启日志 characteristic notification。 */
    bool congested; /**< GATT 拥塞门；true 时暂停 prepare 但保留等待队列。 */
    bool active_valid; /**< active 是否包含正在分片的有效完整帧。 */
    bool chunk_in_flight; /**< 是否已有一个 token 等待 complete，最多只能有一个。 */
    uint16_t active_offset; /**< active 已成功完成的前缀长度，单位 byte。 */
    uint16_t in_flight_length; /**< 当前 token 对应分片长度，单位 byte。 */
    uint32_t in_flight_token; /**< 当前在途分片的非零 token；无在途时为 0。 */
    uint32_t next_token; /**< 最近发出的 token；自增回绕时跳过 0。 */
    s3_ble_log_tx_stats_t stats; /**< 对象拥有的饱和累计统计与等待队列深度。 */
} s3_ble_log_tx_t;

/**
 * @brief 将 BLE 日志发送状态、双队列和全部累计统计清零。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * @param tx 待初始化的可写对象；允许 NULL，NULL 时不执行操作；对象存储由调用方持有。
 * @return 无。
 * 调用方式：在创建 FFE3 TX worker 和首次入队前调用一次；重复调用会丢弃对象内全部待发状态和统计。
 * 线程约束：函数内部不加锁；初始化时须独占对象，禁止与入队、发送或 GATT 状态更新并发，禁止 ISR。
 */
void s3_ble_log_tx_init(s3_ble_log_tx_t *tx);

/**
 * @brief 将一条完整日志帧复制进指定优先级的固定环形队列。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * @param tx 已初始化的可写状态对象，不得为 NULL。
 * @param priority 普通或关键队列优先级；其他枚举值无效。
 * @param frame 调用方拥有的只读完整帧；本函数返回前有效即可，成功时已复制，不转移所有权。
 * @param length frame 有效字节数，范围为 1..S3_BLE_LOG_TX_MAX_FRAME_SIZE。
 * @return 参数有效并完成复制返回 true；参数非法返回 false。队满时覆盖同级最旧帧并计数，仍返回 true。
 * 调用方式：日志生产者编码完整 SmartCarLog 帧后调用；随后唤醒唯一 FFE3 TX worker。
 * 线程约束：内部不加锁且不等待；固件多生产者必须由同一短临界区保护，host test 仅可串行调用，禁止 ISR。
 */
bool s3_ble_log_tx_enqueue(s3_ble_log_tx_t *tx,
                           s3_ble_log_tx_priority_t priority,
                           const uint8_t *frame,
                           uint16_t length);

/**
 * @brief 更新 BLE 连接门并在断连时中止当前 active 帧。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * @param tx 已初始化的可写状态对象；允许 NULL，NULL 时不执行操作。
 * @param connected true 表示连接建立；false 表示断连，同时关闭 CCC、清除拥塞并中止 active 帧。
 * @return 无。
 * 调用方式：GATT connect/disconnect 事件更新外层连接状态时调用；等待队列保留，迟到 complete 由 token 语义处理。
 * 线程约束：内部不加锁；须与 enqueue/prepare/complete 使用同一外部临界区，禁止 ISR。
 */
void s3_ble_log_tx_set_connected(s3_ble_log_tx_t *tx, bool connected);

/**
 * @brief 更新 FFE3 CCC 通知开关，并在关闭时中止当前 active 帧。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * @param tx 已初始化的可写状态对象；允许 NULL，NULL 时不执行操作。
 * @param enabled true 允许 prepare；false 暂停发送并中止 active 帧，但保留两个等待队列。
 * @return 无。
 * 调用方式：解析 FFE3 CCC 写事件后调用，再唤醒 worker 重新评估发送门。
 * 线程约束：内部不加锁；须由状态对象的统一外部临界区保护，禁止 ISR。
 */
void s3_ble_log_tx_set_ccc_enabled(s3_ble_log_tx_t *tx, bool enabled);

/**
 * @brief 更新 GATT 拥塞门并累计一次状态更新事件。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * @param tx 已初始化的可写状态对象；允许 NULL，NULL 时不执行操作也不计数。
 * @param congested true 暂停后续 prepare，false 恢复；不会清空等待队列或中止 active 帧。
 * @return 无。
 * 调用方式：每次匹配当前连接的 GATT CONGEST 事件调用；解除拥塞后唤醒 worker。
 * 线程约束：内部不加锁；须由状态对象的统一外部临界区保护，禁止 ISR。
 */
void s3_ble_log_tx_set_congested(s3_ble_log_tx_t *tx, bool congested);

/**
 * @brief 按连接门、优先级和分片上限准备唯一一个待提交 notification 副本。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * @param tx 已初始化的可写状态对象，不得为 NULL。
 * @param max_chunk_size 本次允许的最大分片字节数；0 无效，通常取 ATT_MTU-3。
 * @param chunk 调用方拥有的可写输出；仅返回 S3_BLE_LOG_TX_PREPARE_READY 时字段和 data 有效。
 * @return 返回 READY、EMPTY、PAUSED、BUSY 或 INVALID_ARG；READY 时登记非零 token，必须配对 complete。
 * 调用方式：唯一 FFE3 TX worker 在 GATT 提交前调用；复制出的 chunk 可在退出临界区后使用。
 * 线程约束：内部不加锁；须在统一外部临界区内调用，任一时刻最多一个在途分片，禁止 ISR。
 */
s3_ble_log_tx_prepare_result_t s3_ble_log_tx_prepare_chunk(
    s3_ble_log_tx_t *tx,
    uint16_t max_chunk_size,
    s3_ble_log_tx_chunk_t *chunk);

/**
 * @brief 以 token 回报一个已准备分片的 GATT 提交结果并推进或放弃 active 帧。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * @param tx 已初始化的可写状态对象，不得为 NULL。
 * @param token prepare 返回的非零在途标识；0、迟到或不匹配 token 不推进状态。
 * @param send_succeeded true 计入 sent_chunks，且 active 仍有效时推进偏移；false 计入 send_fail 并放弃有效 active 帧，
 *                       仅此前已有成功前缀时增加 partial_drop。
 * @return 返回 MORE、FRAME_DONE、FRAME_DROPPED、ABORTED 或 STALE；具体状态见枚举说明。
 * 调用方式：每次 READY 后由唯一 TX worker 在一次 GATT 提交返回后调用一次；ESP_OK 提交不等于手机收到。
 * 线程约束：内部不加锁；须与 prepare 和连接门更新使用同一外部临界区，禁止 ISR。
 */
s3_ble_log_tx_complete_result_t s3_ble_log_tx_complete_chunk(
    s3_ble_log_tx_t *tx,
    uint32_t token,
    bool send_succeeded);

/**
 * @brief 复制日志调度统计快照，读取不清零且不含 active 帧深度。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * @param tx 已初始化的只读状态对象；允许 NULL，NULL 时不写输出。
 * @param stats 调用方拥有的可写输出；允许 NULL，NULL 时不执行复制。
 * @return 无；任一参数为 NULL 时保持输出不变。
 * 调用方式：低频健康诊断读取；统计只描述本地队列与 GATT 提交，不代表手机接收或持久化。
 * 线程约束：内部不加锁；固件读取须用统一外部临界区取得一致快照，host test 可串行调用，禁止 ISR。
 */
void s3_ble_log_tx_get_stats(const s3_ble_log_tx_t *tx,
                             s3_ble_log_tx_stats_t *stats);

#ifdef __cplusplus
}
#endif

#endif /* S3_BLE_LOG_TX_H */
