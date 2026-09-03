#include "srp_link.h"

#include <string.h>

/* SRP ACK/重试与错误状态实现；创建人：待确认（当前维护人：Zhiqin）。 */

/**
 * @brief 以 TEC/REC 较大值重新计算 ACTIVE/WARNING/PASSIVE/BUS_OFF 状态。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param link 非 NULL 的可写链路对象。
 * @return 无；阈值依次为 32、128、256。
 * 调用方式：错误计数增加或减少后立即调用，保证 state 与当前计数一致。
 * 线程约束：无内部锁；同一 link 的所有调用由外层单 owner 或 mutex 串行化。
 */
static void update_state(srp_link_t *link)
{
    const uint16_t score = link->tec > link->rec ? link->tec : link->rec;

    link->state = score >= 256U ? SRP_LINK_BUS_OFF
                                : score >= 128U ? SRP_LINK_PASSIVE
                                : score >= 32U ? SRP_LINK_WARNING
                                               : SRP_LINK_ACTIVE;
}

/**
 * @brief 饱和增加接收错误计数 REC 并刷新链路状态。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param link 非 NULL 的可写链路对象。
 * @param value 本次错误权重；加法在 UINT16_MAX 饱和。
 * @return 无。
 * 调用方式：无效 ACK 或 parser 错误路径调用。
 * 线程约束：无锁修改 link；必须与 send/receive/tick/recover 串行化。
 */
static void add_rec(srp_link_t *link, uint16_t value)
{
    link->rec = link->rec > UINT16_MAX - value ? UINT16_MAX
                                               : (uint16_t)(link->rec + value);
    update_state(link);
}

/**
 * @brief 饱和增加发送错误计数 TEC 并刷新链路状态。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param link 非 NULL 的可写链路对象。
 * @param value 本次错误权重；加法在 UINT16_MAX 饱和。
 * @return 无。
 * 调用方式：首次 transport 失败、重试失败或重试耗尽路径调用。
 * 线程约束：无锁修改 link；必须与 send/receive/tick/recover 串行化。
 */
static void add_tec(srp_link_t *link, uint16_t value)
{
    link->tec = link->tec > UINT16_MAX - value ? UINT16_MAX
                                               : (uint16_t)(link->tec + value);
    update_state(link);
}

/**
 * @brief 在收到一条逻辑帧时把非零 REC 减一并刷新状态。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param link 非 NULL 的可写链路对象。
 * @return 无；REC 已为 0 时保持全部状态不变。
 * 调用方式：srp_link_receive() 接受非 NULL 帧后、处理 ACK/业务前调用。
 * 线程约束：无锁修改 link；只允许链路 owner 调用。
 */
static void reduce_rec(srp_link_t *link)
{
    if (link->rec != 0U) {
        --link->rec;
        update_state(link);
    }
}

/**
 * @brief 在收到成功 ACK 时把非零 TEC 减一并刷新状态。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param link 非 NULL 的可写链路对象。
 * @return 无；TEC 已为 0 时保持全部状态不变。
 * 调用方式：匹配 ACK 且 status_code 为 OK 后调用。
 * 线程约束：无锁修改 link；只允许链路 owner 调用。
 */
static void reduce_tec(srp_link_t *link)
{
    if (link->tec != 0U) {
        --link->tec;
        update_state(link);
    }
}

/**
 * @brief 按消息类型和序号查找正在等待 ACK 的槽位。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param link 非 NULL 链路对象。
 * @param type 被确认消息的 8 位类型。
 * @param sequence 被确认消息的序号。
 * @return 匹配的内部槽指针；没有匹配时返回 NULL，调用方不得长期保存该指针。
 * 调用方式：srp_link_receive() 解析快速响应 payload 后调用。
 * 线程约束：无锁扫描；同一 link 必须由外层串行化。
 */
static srp_link_pending_t *find_pending(srp_link_t *link, uint8_t type,
                                        uint8_t sequence)
{
    for (size_t index = 0U; index < SRP_LINK_PENDING_SLOTS; ++index) {
        srp_link_pending_t *pending = &link->pending[index];
        if (pending->in_use != 0U && pending->type == type &&
            pending->sequence == sequence) {
            return pending;
        }
    }
    return NULL;
}

/**
 * @brief 查找第一个空闲 pending 槽，但不立即标记占用。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param link 非 NULL 链路对象。
 * @return 空闲内部槽指针；四个槽均占用时返回 NULL。
 * 调用方式：srp_link_send() 对 ACK_REQUIRED 消息编码前调用，编码成功后才填充并置 in_use。
 * 线程约束：返回到置位之间没有内部锁；同一 link 必须由单 owner/外层 mutex 串行化。
 */
static srp_link_pending_t *reserve_pending(srp_link_t *link)
{
    for (size_t index = 0U; index < SRP_LINK_PENDING_SLOTS; ++index) {
        if (link->pending[index].in_use == 0U) {
            return &link->pending[index];
        }
    }
    return NULL;
}

/** 初始化链路对象并填充缺省 ACK/重试参数。 */
void srp_link_init(srp_link_t *link, const srp_link_config_t *config)
{
    if (link == NULL) {
        return;
    }
    (void)memset(link, 0, sizeof(*link));
    if (config != NULL) {
        link->config = *config;
    }
    if (link->config.ack_timeout_ms == 0U) {
        link->config.ack_timeout_ms = SRP_LINK_ACK_TIMEOUT_MS;
    }
    if (link->config.max_retries == 0U) {
        link->config.max_retries = SRP_LINK_MAX_RETRIES;
    }
    link->state = SRP_LINK_ACTIVE;
}

/** 编码、发送并在需要时登记一条待 ACK 消息。 */
int srp_link_send(srp_link_t *link, uint8_t priority, uint8_t destination,
                  uint16_t type, uint8_t flags, const uint8_t *payload,
                  uint16_t length, uint32_t now_ms,
                  srp_link_tx_callback_t callback, void *callback_context)
{
    srp_frame_t frame;
    srp_link_pending_t *pending = NULL;
    uint8_t *frame_bytes;
    uint16_t encoded_length = 0U;
    uint8_t sequence;
    int result;

    (void)destination;
    if (link == NULL || link->config.transport_send == NULL || type > UINT8_MAX ||
        length > SRP_MAX_PAYLOAD || (flags & SRP_FLAG_RESERVED_MASK) != 0U) {
        return -1;
    }
    if ((flags & SRP_FLAG_ACK_REQUIRED) != 0U) {
        pending = reserve_pending(link);
        if (pending == NULL) {
            return -2;
        }
    }
    sequence = link->next_sequence++;
    frame.priority = priority;
    frame.type = (uint8_t)type;
    frame.sequence = sequence;
    frame.flags = flags;
    frame.length = length;
    frame.payload = payload;
    frame_bytes = pending != NULL ? pending->frame_bytes : link->tx_scratch;
    result = srp_encode_frame(&frame, frame_bytes, SRP_MAX_FRAME_SIZE,
                              &encoded_length);
    if (result != SRP_CODEC_OK) {
        return -3;
    }
    if (pending != NULL) {
        pending->in_use = 1U;
        pending->retry_count = 0U;
        pending->type = (uint8_t)type;
        pending->sequence = sequence;
        pending->frame_length = encoded_length;
        pending->last_tx_ms = now_ms;
        pending->callback = callback;
        pending->callback_context = callback_context;
        result = link->config.transport_send(frame_bytes, encoded_length,
                                              link->config.context);
    } else {
        result = link->config.transport_send(frame_bytes, encoded_length,
                                             link->config.context);
    }
    if (result != 0) {
        if (pending != NULL) {
            pending->in_use = 0U;
        }
        add_tec(link, 8U);
        return -4;
    }
    return 0;
}

/** 发送不占用待 ACK 槽位的快速响应。 */
int srp_link_send_fast_response(srp_link_t *link, uint8_t priority,
                                uint8_t destination, uint8_t is_error,
                                uint16_t ack_type, uint8_t ack_sequence,
                                uint8_t status_code, uint32_t now_ms)
{
    uint8_t payload[SRP_PAYLOAD_FAST_RESPONSE_SIZE] = {
        (uint8_t)ack_type, 0U, ack_sequence, status_code};
    const uint8_t type = is_error != 0U ? SRP_MSG_ID_ERROR : SRP_MSG_ID_ACK;

    (void)now_ms;
    return srp_link_send(link, priority, destination, type,
                         (uint8_t)(is_error != 0U ? SRP_FLAG_ERROR : SRP_FLAG_ACK),
                         payload, sizeof(payload), now_ms, NULL, NULL);
}

/** 清理指定类型的所有待 ACK 项。 */
void srp_link_cancel_message(srp_link_t *link, uint16_t type)
{
    if (link == NULL) {
        return;
    }
    for (size_t index = 0U; index < SRP_LINK_PENDING_SLOTS; ++index) {
        if (link->pending[index].in_use != 0U &&
            link->pending[index].type == (uint8_t)type) {
            link->pending[index].in_use = 0U;
        }
    }
}

/** 消费一条接收帧，处理 ACK 或转交业务回调。 */
void srp_link_receive(srp_link_t *link, const srp_frame_t *frame)
{
    srp_frame_t local;
    srp_link_pending_t *pending;
    uint8_t ack_type;
    uint8_t ack_sequence;
    uint8_t status_code;

    if (link == NULL || frame == NULL) {
        return;
    }
    reduce_rec(link);
    if (frame->type == SRP_MSG_ID_ACK || frame->type == SRP_MSG_ID_ERROR) {
        if (frame->length < SRP_PAYLOAD_FAST_RESPONSE_SIZE || frame->payload == NULL) {
            add_rec(link, 8U);
            return;
        }
        ack_type = frame->payload[0];
        ack_sequence = frame->payload[2];
        status_code = frame->payload[3];
        pending = find_pending(link, ack_type, ack_sequence);
        if (pending == NULL) {
            return;
        }
        pending->in_use = 0U;
        if (frame->type == SRP_MSG_ID_ACK && status_code == SRP_FAST_RESP_OK) {
            reduce_tec(link);
            if (pending->callback != NULL) {
                pending->callback(SRP_LINK_TX_OK, status_code,
                                  pending->callback_context);
            }
        } else {
            if (pending->callback != NULL) {
                pending->callback(SRP_LINK_TX_REMOTE_ERROR, status_code,
                                  pending->callback_context);
            }
        }
        return;
    }
    local = *frame;
    if (link->config.on_frame != NULL) {
        link->config.on_frame(&local, link->config.context);
    }
}

/** 将解析错误折算为 REC 增量并刷新链路状态。 */
void srp_link_report_parser_error(srp_link_t *link, srp_parser_error_t error)
{
    if (link == NULL) {
        return;
    }
    if (error == SRP_PARSER_ERROR_CRC || error == SRP_PARSER_ERROR_EOF) {
        add_rec(link, 8U);
    } else {
        add_rec(link, 1U);
    }
}

/** 推进 ACK 超时、有限重试和 BUS_OFF 回调。 */
void srp_link_tick(srp_link_t *link, uint32_t now_ms)
{
    if (link == NULL || link->config.transport_send == NULL) {
        return;
    }
    for (size_t index = 0U; index < SRP_LINK_PENDING_SLOTS; ++index) {
        srp_link_pending_t *pending = &link->pending[index];
        if (pending->in_use == 0U ||
            (uint32_t)(now_ms - pending->last_tx_ms) < link->config.ack_timeout_ms) {
            continue;
        }
        if (pending->retry_count < link->config.max_retries) {
            if (link->config.transport_send(pending->frame_bytes,
                                             pending->frame_length,
                                             link->config.context) == 0) {
                ++pending->retry_count;
                pending->last_tx_ms = now_ms;
            } else {
                add_tec(link, 8U);
            }
        } else {
            pending->in_use = 0U;
            add_tec(link, 8U);
            if (pending->callback != NULL) {
                pending->callback(SRP_LINK_TX_TIMEOUT, SRP_FAST_RESP_TIMEOUT,
                                  pending->callback_context);
            }
        }
    }
    if (link->state == SRP_LINK_BUS_OFF && link->config.on_bus_off != NULL) {
        link->config.on_bus_off(link->config.context);
    }
}

/** 清除待 ACK 和错误状态，供上层重新同步。 */
void srp_link_recover(srp_link_t *link)
{
    if (link == NULL) {
        return;
    }
    (void)memset(link->pending, 0, sizeof(link->pending));
    link->tec = 0U;
    link->rec = 0U;
    link->state = SRP_LINK_ACTIVE;
}

uint16_t srp_link_get_tec(const srp_link_t *link)
{
    return link == NULL ? 0U : link->tec;
}

uint16_t srp_link_get_rec(const srp_link_t *link)
{
    return link == NULL ? 0U : link->rec;
}

srp_link_state_t srp_link_get_state(const srp_link_t *link)
{
    return link == NULL ? SRP_LINK_BUS_OFF : link->state;
}
