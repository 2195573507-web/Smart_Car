#include "app_parser.h"

#include <string.h>

#include "srp_crc.h"

/* App BLE 外层解析实现；创建人：待确认（当前维护人：Zhiqin）。 */

/**
 * @brief 清除当前半帧长度状态，但保留解析回调及其上下文。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param parser 已初始化且非 NULL 的解析器状态，由调用方独占。
 * @return 返回值：无（void）。
 * 调用方式：仅由本文件的增量解析流程在丢弃半帧或完成一帧后调用。
 * 线程约束：不加锁、不分配内存；必须与 sc_app_parser_feed() 同任务串行，禁止 ISR、并发或递归调用。
 */
static void app_parser_reset(sc_app_parser_t *parser)
{
    parser->length = 0U;
    parser->expected_length = 0U;
}

/**
 * @brief 在保持当前候选帧状态的前提下同步上报解析错误。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param parser 已初始化且非 NULL 的解析器；错误回调和内部字节缓冲均从该对象读取。
 * @param error 传给回调的负错误码，含义由 app_parser.h 定义。
 * @return 返回值：无（void）。
 * 调用方式：由 sc_app_parser_feed() 在长度、尾字节、版本或 CRC 校验失败时调用。
 * 线程约束：错误回调在 feed 调用栈内同步执行；bytes 指针仅在回调期间借用，回调不得阻塞、保留指针或递归 feed 同一解析器。
 */
static void app_parser_report_error(sc_app_parser_t *parser, int error)
{
    if (parser->error_callback != NULL) {
        parser->error_callback(error, parser->bytes, parser->length,
                               parser->context);
    }
}

/**
 * @brief 初始化 App BLE 增量解析器并清除既有半帧状态。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param parser 调用方长期持有的可写状态；NULL 时直接返回。
 * @param callback 完整帧同步回调，允许 NULL。
 * @param error_callback 格式错误同步回调，允许 NULL。
 * @param context 回调上下文，仅保存指针、不接管所有权，允许 NULL。
 * @return 返回值：无（void）。
 * 调用方式：在 BLE RX 队列消费者开始 feed 前调用；重新调用会清除当前半帧。
 * 线程约束：初始化必须与 feed 串行；无锁、不分配内存，运行中不得从 ISR 或其他任务并发重置同一 parser。
 */
void sc_app_parser_init(sc_app_parser_t *parser,
                        sc_app_parser_callback_t callback,
                        sc_app_parser_error_callback_t error_callback,
                        void *context)
{
    if (parser == NULL) {
        return;
    }
    memset(parser, 0, sizeof(*parser));
    parser->callback = callback;
    parser->error_callback = error_callback;
    parser->context = context;
}

/**
 * @brief 喂入一段 App BLE 字节，并在完整合法帧到达时同步触发回调。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param parser 已初始化的可写解析状态；NULL 时返回 0。
 * @param data 本次连续输入字节，仅在调用期间读取；length 为 0 时允许 NULL。
 * @param length 输入字节数；有效参数下会逐字节扫描并合并到内部半帧状态。
 * @return 本次完成且通过版本、尾字节和 CRC 校验的帧数，不是消费字节数；错误帧不计数。
 * 调用方式：由 smartcar_service 的 BLE RX 队列消费者持续调用，支持拆包和多帧合包。
 * 线程约束：单 parser、单任务 owner；完整帧/错误回调均在本函数栈内同步执行，禁止递归、并发 feed 或在回调后保留 payload 指针。
 */
size_t sc_app_parser_feed(sc_app_parser_t *parser, const uint8_t *data,
                          size_t length)
{
    if (parser == NULL || (data == NULL && length != 0U)) {
        return 0U;
    }

    size_t frames = 0U;
    for (size_t index = 0U; index < length; ++index) {
        const uint8_t byte = data[index];

        if (parser->length == 0U) {
            if (byte == SC_APP_FRAME_HEAD) {
                parser->bytes[parser->length++] = byte;
            }
            continue;
        }
        if (parser->length == 1U) {
            /* Accept only the two explicitly supported App-BLE versions. */
            if (byte == SC_APP_FRAME_VERSION ||
                byte == SC_APP_FRAME_VERSION_V2) {
                parser->bytes[parser->length++] = byte;
            } else if (byte == SC_APP_FRAME_HEAD) {
                parser->bytes[0] = byte;
            } else {
                app_parser_reset(parser);
            }
            continue;
        }
        parser->bytes[parser->length++] = byte;

        if (parser->length == 5U) {
            const uint16_t payload_length = (uint16_t)parser->bytes[3] |
                                            ((uint16_t)parser->bytes[4] << 8U);
            if (payload_length > SC_APP_FRAME_MAX_PAYLOAD) {
                app_parser_report_error(parser, -4);
                app_parser_reset(parser);
                if (byte == SC_APP_FRAME_HEAD) {
                    parser->bytes[parser->length++] = byte;
                }
                continue;
            }
            parser->expected_length =
                (uint16_t)(SC_APP_FRAME_OVERHEAD + payload_length);
        }

        if (parser->expected_length != 0U &&
            parser->length == parser->expected_length) {
            const uint16_t payload_length =
                (uint16_t)parser->bytes[3] |
                ((uint16_t)parser->bytes[4] << 8U);
            const uint16_t received_crc =
                (uint16_t)parser->bytes[5U + payload_length] |
                ((uint16_t)parser->bytes[6U + payload_length] << 8U);
            const uint16_t calculated_crc =
                srp_crc16_modbus(&parser->bytes[1], 4U + payload_length);

            if (parser->bytes[1] != SC_APP_FRAME_VERSION &&
                parser->bytes[1] != SC_APP_FRAME_VERSION_V2) {
                app_parser_report_error(parser, -3);
            } else if (parser->bytes[7U + payload_length] !=
                       SC_APP_FRAME_TAIL) {
                app_parser_report_error(parser, -2);
            } else if (received_crc != calculated_crc) {
                app_parser_report_error(parser, -5);
            } else {
                sc_app_frame_view_t view = {
                    .version = parser->bytes[1],
                    .type = parser->bytes[2],
                    .length = payload_length,
                    .payload = &parser->bytes[5],
                };
                if (parser->callback != NULL) {
                    parser->callback(&view, parser->context);
                }
                ++frames;
            }
            app_parser_reset(parser);
        }
    }
    return frames;
}

/**
 * @brief 使用当前默认 App 版本编码一条 BLE 外层帧，不接触 STM-S3 SRP 状态。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param type App V1 消息类型，本函数不校验类型是否已注册。
 * @param payload 只读 payload；length 为 0 时允许 NULL，不得与 out 重叠。
 * @param length payload 字节数，不得超过 SC_APP_FRAME_MAX_PAYLOAD。
 * @param out 可写完整帧缓冲，成功时写入头、版本、类型、长度、payload、CRC 和尾字节。
 * @param capacity out 的实际容量，至少为 SC_APP_FRAME_OVERHEAD + length。
 * @param out_length 成功时写入完整帧长度；失败时保持调用前内容。
 * @return 0 表示成功；-1 表示指针组合非法；-2 表示长度或容量非法。
 * 调用方式：任务上下文编码后交给 s3_ble_notify_send()；输出不能直接当作 SRP 帧使用。
 * 线程约束：纯编码、可重入、不分配内存；调用期间独占 out/out_length，禁止从 ISR 直接驱动后续 BLE 发送。
 */
int sc_app_frame_encode(uint8_t type, const uint8_t *payload, uint16_t length,
                        uint8_t *out, size_t capacity, uint16_t *out_length)
{
    return sc_app_frame_encode_version(SC_APP_FRAME_VERSION, type, payload,
                                       length, out, capacity, out_length);
}

/**
 * @brief 使用显式 App 协议版本编码帧，供 V1/V2 兼容路径和回放测试使用。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param version 只能为 SC_APP_FRAME_VERSION 或 SC_APP_FRAME_VERSION_V2。
 * @param type App 消息类型，本函数不查询类型注册表。
 * @param payload 只读 payload；length 为 0 时允许 NULL，不得与 out 重叠。
 * @param length payload 字节数，不得超过 SC_APP_FRAME_MAX_PAYLOAD。
 * @param out 可写完整帧缓冲。
 * @param capacity out 的实际容量，至少为固定开销加 length。
 * @param out_length 成功时写入完整帧长度；失败时保持调用前内容。
 * @return 0 表示成功；-1 表示指针组合非法；-2 表示版本、长度或容量非法。
 * 调用方式：只有明确知道接收端 App 版本时调用；不得用本函数伪造 STM-S3 SRP 版本。
 * 线程约束：纯编码、可重入、不分配内存；调用期间独占输出缓冲，禁止从 ISR 调用后续 BLE 栈路径。
 */
int sc_app_frame_encode_version(uint8_t version, uint8_t type,
                                const uint8_t *payload, uint16_t length,
                                uint8_t *out, size_t capacity,
                                uint16_t *out_length)
{
    if (out == NULL || out_length == NULL ||
        (payload == NULL && length != 0U)) {
        return -1;
    }
    if ((version != SC_APP_FRAME_VERSION &&
         version != SC_APP_FRAME_VERSION_V2) ||
        length > SC_APP_FRAME_MAX_PAYLOAD ||
        capacity < SC_APP_FRAME_OVERHEAD + length) {
        return -2;
    }

    out[0] = SC_APP_FRAME_HEAD;
    out[1] = version;
    out[2] = type;
    out[3] = (uint8_t)(length & 0xFFU);
    out[4] = (uint8_t)(length >> 8U);
    if (length != 0U) {
        memcpy(&out[5], payload, length);
    }
    const uint16_t crc = srp_crc16_modbus(&out[1], 4U + length);
    out[5U + length] = (uint8_t)(crc & 0xFFU);
    out[6U + length] = (uint8_t)(crc >> 8U);
    out[7U + length] = SC_APP_FRAME_TAIL;
    *out_length = (uint16_t)(SC_APP_FRAME_OVERHEAD + length);
    return 0;
}
