#ifndef SRP_LINK_H
#define SRP_LINK_H

#include <stdint.h>

#include "srp_codec.h"
#include "srp_registry.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * SRP 链路层负责 ACK、有限重试和 REC/TEC 状态，不负责具体 UART 实现。
 * 创建人：待确认（当前维护人：Zhiqin）。
 * 调用方必须串行化同一个 srp_link_t 的可变状态访问，除非在外层加锁。
 */
#define SRP_LINK_PENDING_SLOTS UINT8_C(4)
#define SRP_LINK_ACK_TIMEOUT_MS UINT32_C(500)
#define SRP_LINK_MAX_RETRIES UINT8_C(3)

/** 由 max(TEC, REC) 阈值推导的链路健康状态，不等同于 UART 物理连通。 */
typedef enum {
    SRP_LINK_ACTIVE = 0, /**< 错误分数小于 32，可按上层会话门正常使用。 */
    SRP_LINK_WARNING,    /**< 错误分数为 32..127，需告警但链路层仍可发送。 */
    SRP_LINK_PASSIVE,    /**< 错误分数为 128..255，需由上层执行降级策略。 */
    SRP_LINK_BUS_OFF     /**< 错误分数至少 256；状态持续期间 tick 电平式报告故障。 */
} srp_link_state_t;

/** ACK_REQUIRED 事务最终结果；由同步完成回调消费。 */
typedef enum {
    SRP_LINK_TX_OK = 0,             /**< 收到匹配 ACK 且远端 status_code 为 OK。 */
    SRP_LINK_TX_TRANSPORT_FAILURE,  /**< 底层传输未接受帧；当前完成路径保留该分类。 */
    SRP_LINK_TX_TIMEOUT,            /**< 达到最大重试数后仍未收到匹配响应。 */
    SRP_LINK_TX_REMOTE_ERROR,       /**< 收到匹配 ERROR 或非 OK 快速响应状态。 */
    SRP_LINK_TX_BUS_OFF             /**< BUS_OFF 完成结果分类；当前链路实现尚未主动回调该值。 */
} srp_link_tx_result_t;

/**
 * @brief SRP 链路调用的底层发送回调类型。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * @param data 只读完整线缆帧，只在回调期间有效，回调必须同步消费或复制。
 * @param length data 的实际字节数。
 * @param context srp_link_config_t 登记的上下文，允许 NULL。
 * @return 0 表示传输层接受；非零表示本次发送失败，链路会更新 TEC/结果。
 * 调用方式：由 send/tick 的调用栈同步触发，不得回调同一 link 的可变 API。
 * 线程约束：继承链路 owner 上下文；不得长期阻塞或保存 data 指针。
 */
typedef int (*srp_link_transport_send_t)(const uint8_t *data, uint16_t length,
                                         void *context);
/**
 * @brief ACK 成功、远端错误、超时或 BUS_OFF 的同步完成回调类型。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * @param result 最终发送结果。
 * @param status_code 快速响应状态码；非远端响应结果时按调用路径解释。
 * @param context srp_link_send() 登记的回调上下文，允许 NULL。
 * @return 无。
 * 调用方式：由 receive/tick/recover 路径同步触发；只做有界状态更新或排队。
 * 线程约束：不得递归修改同一 link，耗时工作应移交任务队列。
 */
typedef void (*srp_link_tx_callback_t)(srp_link_tx_result_t result,
                                       uint8_t status_code, void *context);
/**
 * @brief 非 ACK 业务帧的同步交付回调类型。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * @param frame 已校验逻辑帧；payload 只在回调期间借用。
 * @param context link 配置上下文，允许 NULL。
 * @return 无。
 * 调用方式：由 srp_link_receive() 同步触发；跨任务保存前复制 payload。
 * 线程约束：不得保留 frame/payload，且不得递归操作同一 link 的可变状态。
 */
typedef void (*srp_link_frame_callback_t)(const srp_frame_t *frame,
                                          void *context);
/**
 * @brief 链路保持 BUS_OFF 时的电平式故障回调类型。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * @param context link 配置上下文，允许 NULL。
 * @return 无。
 * srp_link_tick() 在链路保持 BUS_OFF 的每次调用中都可能触发，直到上层 recover；
 * 回调必须幂等且不得假定只执行一次。
 * 调用方式：立即置故障/停机标志，恢复由外层状态机另行调度。
 * 线程约束：在 tick owner 上下文同步执行，不得递归调用同一 link。
 */
typedef void (*srp_link_bus_off_callback_t)(void *context);

#pragma pack(push, 4)
/** SRP 链路初始化配置；初始化时按值复制，回调及 context 指针仍由调用方维护。 */
typedef struct {
    uint8_t local_node; /**< 本端 SRP_NODE_* 标识；当前 v4 header 不序列化该字段。 */
    uint32_t ack_timeout_ms; /**< ACK 等待时间，单位 ms；0 在 init 时替换为 500 ms 默认值。 */
    uint8_t max_retries; /**< 首次发送后的最大重发次数；0 在 init 时替换为默认 3。 */
    srp_link_transport_send_t transport_send; /**< 必需的借用底层发送回调，link 不拥有。 */
    srp_link_frame_callback_t on_frame; /**< 可选借用业务帧回调；同步执行。 */
    srp_link_bus_off_callback_t on_bus_off; /**< 可选借用 BUS_OFF 电平回调；tick 可重复调用。 */
    void *context; /**< 三个链路回调共享的借用上下文；link 不释放。 */
} srp_link_config_t;

/** 单条 ACK_REQUIRED 帧的内部重试槽；frame_bytes 由链路对象完整拥有。 */
typedef struct {
    uint8_t in_use; /**< 0 表示空闲，非 0 表示正在等待 ACK。 */
    uint8_t retry_count; /**< 已被 transport 接受的重发次数，不含首次发送。 */
    uint8_t type; /**< 被确认消息的 8 位类型，用于匹配快速响应。 */
    uint8_t sequence; /**< 被确认消息的 8 位序号，用于匹配快速响应。 */
    uint16_t frame_length; /**< frame_bytes 中有效完整线缆帧长度，单位 byte。 */
    uint32_t last_tx_ms; /**< 首次或最近一次成功重发时间，单位单调 ms。 */
    srp_link_tx_callback_t callback; /**< 借用完成回调，可为 NULL；槽位不拥有代码或上下文。 */
    void *callback_context; /**< 原样传给 callback 的借用上下文，可为 NULL。 */
    uint8_t frame_bytes[SRP_MAX_FRAME_SIZE]; /**< 供超时重发使用的内部完整帧副本。 */
} srp_link_pending_t;

/** 一个 SRP 会话方向的可变链路状态；必须由单 owner 或外部锁串行访问。 */
typedef struct {
    uint16_t tec; /**< 发送错误计数，按错误权重增减并在 UINT16_MAX 饱和。 */
    uint16_t rec; /**< 接收错误计数，按错误权重增减并在 UINT16_MAX 饱和。 */
    uint8_t next_sequence; /**< 下一发送序号，使用后自增并按 8 位自然回绕。 */
    srp_link_state_t state; /**< 由 TEC/REC 推导的当前健康状态。 */
    srp_link_config_t config; /**< init 时复制的配置；其中指针仍为借用。 */
    srp_link_pending_t pending[SRP_LINK_PENDING_SLOTS]; /**< 链路拥有的四个待 ACK/重试槽。 */
    /* Callers serialize srp_link_send() for this mutable, non-ACK TX buffer. */
    uint8_t tx_scratch[SRP_MAX_FRAME_SIZE]; /**< 非 ACK 帧共享编码缓冲；发送回调返回后可复用。 */
} srp_link_t;
#pragma pack(pop)

/**
 * @brief 初始化链路状态和待确认槽位。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * @param link 调用方持有的链路对象。
 * @param config 传输回调、节点号、ACK 超时和重试上限；初始化时复制。
 * @return 无；link 或 config 为 NULL 时不写入。
 * 调用方式：在接收/发送任务启动前调用；config 中的回调不得在初始化后失效。
 * 线程约束：非线程安全；初始化期间禁止并发 send/receive/tick。
 */
void srp_link_init(srp_link_t *link, const srp_link_config_t *config);

/**
 * @brief 编码并发送一条 SRP 消息，可选地登记 ACK 等待。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * @param priority 优先级；急停/普通命令/遥测/日志须使用注册表约定。
 * @param destination 保留兼容参数；当前 SRP v4 header 不编码节点地址，实现会忽略该值。
 * @param type 消息类型 ID。
 * @param flags SRP 标志位；ACK_REQUIRED 时必须提供有效回调/上下文策略。
 * @param payload payload 指针；函数在调用期间读取，不接管所有权。
 * @param length payload 长度，不能超过 SRP_MAX_PAYLOAD。
 * @param now_ms 单调毫秒时间，用于 ACK 超时计算。
 * @param callback ACK/失败完成回调，可为 NULL。
 * @param callback_context 回调上下文。
 * @return 0 表示已交给 transport_send；负值表示参数、槽位或链路错误。
 * 调用方式：普通任务上下文；同一 link 的调用必须串行化，回调不得递归发送同一 link。
 * 线程约束：函数会修改序号、pending 槽和 tx_scratch；同一 link 不可并发调用。
 */
int srp_link_send(srp_link_t *link, uint8_t priority, uint8_t destination,
                  uint16_t type, uint8_t flags, const uint8_t *payload,
                  uint16_t length, uint32_t now_ms,
                  srp_link_tx_callback_t callback, void *callback_context);
/**
 * @brief 发送不进入重试队列的快速 ACK/NACK 响应。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * @param link 已初始化链路对象。
 * @param priority 响应优先级。
 * @param destination 保留兼容参数，当前实现忽略。
 * @param is_error 非零表示设置 SRP_FLAG_ERROR。
 * @param ack_type/ack_sequence 被确认的消息类型和序号。
 * @param status_code `srp_registry.h` 中的快速响应码。
 * @param now_ms 当前单调毫秒时间；当前快速路径不建立 pending，但传给统一发送入口。
 * @return 0 表示 transport 接受；负值语义同 srp_link_send()。
 * 调用方式：解析回调或服务任务均可调用；若回调持有外层锁，transport_send 不得反向取锁。
 * 线程约束：复用 link->tx_scratch，同一 link 必须由外层串行化。
 */
int srp_link_send_fast_response(srp_link_t *link, uint8_t priority,
                                uint8_t destination, uint8_t is_error,
                                uint16_t ack_type, uint8_t ack_sequence,
                                uint8_t status_code, uint32_t now_ms);
/**
 * @brief 取消指定类型的所有待 ACK 消息。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * @param link 链路对象；NULL 时安全返回。
 * @param type 消息类型，当前 pending 槽按低 8 位比较。
 * 返回值：无。调用方式：停止/重同步时由服务 owner 调用。
 * 线程约束：修改 pending 槽，与 send/receive/tick 串行化。
 */
void srp_link_cancel_message(srp_link_t *link, uint16_t type);

/**
 * @brief 将完整逻辑帧交给 ACK 匹配或业务回调。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * @param link 已初始化链路对象。
 * @param frame 已通过 codec 校验的借用视图，仅在调用期间有效。
 * 返回值：无；未知 ACK 被忽略，非法响应会增加 REC。
 * 调用方式：parser 完整帧回调在同一服务 owner 中调用，返回前完成 ACK 匹配/业务分发。
 * 线程约束：可能同步调用完成/业务回调；同一 link 必须外层串行化。
 */
void srp_link_receive(srp_link_t *link, const srp_frame_t *frame);

/**
 * @brief 报告解析器错误并按错误类型增加 REC。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * @param link 链路对象；NULL 时安全返回。
 * @param error parser 错误类型；CRC/EOF 权重高于普通格式错误。
 * 返回值：无；可能推动 WARNING/PASSIVE/BUS_OFF 状态转换。
 * 调用方式：仅由与该 link 配对的 parser 错误回调调用。
 * 线程约束：修改链路计数，与其他链路 API 串行化。
 */
void srp_link_report_parser_error(srp_link_t *link, srp_parser_error_t error);

/**
 * @brief 推进 ACK 超时、有限重试和 BUS_OFF 电平回调。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * @param link 链路对象。
 * @param now_ms 单调毫秒时间，使用无符号差值支持回绕。
 * 返回值：无；可能同步调用 transport、完成回调和 on_bus_off。
 * 调用方式：链路 owner 按固定周期调用；BUS_OFF 回调为电平语义，可能每次 tick 重复。
 * 线程约束：由固定周期单 owner 调用，与 send/receive/recover 串行化。
 */
void srp_link_tick(srp_link_t *link, uint32_t now_ms);

/**
 * @brief 清空 pending、TEC/REC 和降级状态，恢复为 ACTIVE。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * @param link 链路对象；NULL 时安全返回。返回值：无。
 * 调用方式：底层 UART 已清理且上层即将重新同步时调用；不会自动重发旧消息。
 * 线程约束：与全部链路 API 串行化。
 */
void srp_link_recover(srp_link_t *link);

/**
 * @brief 读取发送错误计数 TEC。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * @param link 只读链路对象；允许 NULL。
 * 返回值：link=NULL 时返回 0，否则返回当前 TEC；只读、不清零。
 * 调用方式：服务 owner 用于低频诊断和降级状态展示。
 * 线程约束：无锁快照；与写侧并发时不提供跨字段一致性保证。
 */
uint16_t srp_link_get_tec(const srp_link_t *link);

/**
 * @brief 读取接收错误计数 REC。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * @param link 只读链路对象；允许 NULL。
 * 返回值：link=NULL 时返回 0，否则返回当前 REC；只读、不清零。
 * 调用方式：服务 owner 用于低频诊断和降级状态展示。
 * 线程约束：无锁快照；与写侧并发时不提供跨字段一致性保证。
 */
uint16_t srp_link_get_rec(const srp_link_t *link);

/**
 * @brief 读取当前 ACTIVE/WARNING/PASSIVE/BUS_OFF 状态。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * @param link 只读链路对象；允许 NULL。
 * 返回值：link=NULL 时保守返回 BUS_OFF，否则返回由 TEC/REC 推导的状态。
 * 调用方式：服务 owner 用于门控/诊断；仍须同时检查会话同步和本地安全门。
 * 线程约束：只读快照；调用方不应据此省略同步和本地安全门。
 */
srp_link_state_t srp_link_get_state(const srp_link_t *link);

#ifdef __cplusplus
}
#endif

#endif /* SRP_LINK_H */
