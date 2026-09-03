#ifndef SMARTCAR_APP_PARSER_H
#define SMARTCAR_APP_PARSER_H

#include <stddef.h>
#include <stdint.h>

/*
 * App BLE 外层帧解析器。它与 STM32-S3 的 SRP v4 线缆帧是两个独立协议层。
 * 创建人：待确认（当前维护人：Zhiqin）。
 */

#define SC_APP_FRAME_HEAD 0xAAU
#define SC_APP_FRAME_TAIL 0x55U
#define SC_APP_FRAME_VERSION 0x01U
#define SC_APP_FRAME_VERSION_V1 SC_APP_FRAME_VERSION
#define SC_APP_FRAME_VERSION_V2 0x02U
#define SC_APP_FRAME_MAX_PAYLOAD 128U
#define SC_APP_FRAME_OVERHEAD 8U
#define SC_APP_FRAME_MAX_SIZE \
    (SC_APP_FRAME_OVERHEAD + SC_APP_FRAME_MAX_PAYLOAD)

#define SC_APP_TYPE_ACK 0x06U
#define SC_APP_TYPE_STATUS 0x02U
#define SC_APP_TYPE_ATTITUDE 0x11U
#define SC_APP_TYPE_WHEEL_SPEED_CMD 0x15U
#define SC_APP_TYPE_WHEEL_SPEED_STATUS 0x16U
#define SC_APP_TYPE_RADAR_STATUS 0x1AU
#define SC_APP_TYPE_RADAR_SET_SPEED 0x1BU
#define SC_APP_TYPE_POWER_STATUS 0x1CU
#define SC_APP_TYPE_PID_PARAMS_CMD 0x1DU
#define SC_APP_TYPE_CHASSIS_STATE 0x29U
#define SC_APP_TYPE_WHEEL_SPEED_SINGLE_CMD 0x2AU
#define SC_APP_TYPE_MASTER_SPEED_CMD 0x2BU
#define SC_APP_TYPE_WHEEL_CONTROL_STATUS 0x2CU
#define SC_APP_TYPE_CHASSIS_SPEED_CMD 0x2DU
#define SC_APP_TYPE_CHASSIS_HEADING_CMD 0x2EU
#define SC_APP_TYPE_SYS_CONFIG 0x70U
#define SC_APP_ACK_OK 0x00U
#define SC_APP_ACK_REJECTED 0x01U

#define SC_APP_V2_TYPE_HELLO 0x70U
#define SC_APP_V2_TYPE_HELLO_ACK 0x71U
#define SC_APP_V2_TYPE_HEARTBEAT 0x72U
#define SC_APP_V2_TYPE_HEARTBEAT_ACK 0x73U
#define SC_APP_V2_TYPE_COMMAND_ACK 0x74U
#define SC_APP_V2_TYPE_COMMAND 0x75U

#define SC_APP_V2_RESULT_OK 0x00U
#define SC_APP_V2_RESULT_REJECTED 0x01U
#define SC_APP_V2_RESULT_SESSION_INVALID 0x02U
#define SC_APP_V2_RESULT_EXPIRED 0x03U
#define SC_APP_V2_RESULT_STALE_SEQUENCE 0x04U
#define SC_APP_V2_RESULT_BUSY 0x05U

#define SC_APP_V2_STAGE_GATEWAY_ADMITTED 0x00U
#define SC_APP_V2_STAGE_STM32_ACCEPTED 0x01U
#define SC_APP_V2_STAGE_STOP_QUEUED 0x02U

/** 已校验 App BLE 帧的只读逻辑视图；payload 借用 parser 内部缓冲。 */
typedef struct {
    uint8_t version; /**< SC_APP_FRAME_VERSION_V1/V2 协议版本。 */
    uint8_t type; /**< SC_APP_TYPE_* 或 SC_APP_V2_TYPE_* 消息类型。 */
    uint16_t length; /**< payload 字节数，范围 0..SC_APP_FRAME_MAX_PAYLOAD。 */
    const uint8_t *payload; /**< 借用的 payload；同步回调返回后即可能被覆盖。 */
} sc_app_frame_view_t;

/**
 * @brief App 完整帧同步回调类型。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * @param frame 非 NULL 的栈上逻辑视图；payload 借用 parser 内部缓冲，只在回调期间有效。
 * @param context 初始化解析器时登记的原样上下文，允许 NULL。
 * @return 无。
 * 调用方式：由 sc_app_parser_feed() 在调用者任务中同步触发；回调只复制/排队规范化命令。
 * 线程约束：不得阻塞、递归 feed 同一 parser、保留 frame/payload，或直接操作执行器。
 */
typedef void (*sc_app_parser_callback_t)(const sc_app_frame_view_t *frame,
                                         void *context);
/**
 * @brief App 长度、尾字节或 CRC 错误的同步回调类型。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * @param error 负错误码：-2 尾字节、-3 版本、-4 payload 过长、-5 CRC。
 * @param data parser 内部候选帧缓冲，只在回调期间有效。
 * @param length 当前候选帧已缓存字节数。
 * @param context 初始化时登记的原样上下文，允许 NULL。
 * @return 无。
 * 调用方式：由 feed() 同步触发；用于有界计数/日志，不得在回调中递归使用 parser。
 * 线程约束：不得阻塞或保留 data 指针。
 */
typedef void (*sc_app_parser_error_callback_t)(int error,
                                               const uint8_t *data,
                                               size_t length,
                                               void *context);

/** 单消费者 App BLE 增量解析状态；拥有半帧字节，回调和 context 仅借用。 */
typedef struct {
    uint8_t bytes[SC_APP_FRAME_MAX_SIZE]; /**< 解析器拥有的候选帧缓冲；回调返回后可复用。 */
    uint16_t length; /**< 当前已缓存候选帧字节数，也是下一写入偏移。 */
    uint16_t expected_length; /**< 从 payload length 推导的完整帧字节数；尚未取得长度时为 0。 */
    sc_app_parser_callback_t callback; /**< 借用的同步完整帧回调，可为 NULL。 */
    sc_app_parser_error_callback_t error_callback; /**< 借用的同步格式错误回调，可为 NULL。 */
    void *context; /**< 原样传给回调的借用上下文；parser 不释放。 */
} sc_app_parser_t;

/**
 * @brief 初始化 App 帧增量解析器。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * @param parser 调用方长期持有的可写状态；NULL 时直接返回。
 * @param callback 完整帧回调，允许 NULL。
 * @param error_callback 格式错误回调，允许 NULL。
 * @param context 回调上下文，仅保存指针、不接管所有权，允许 NULL。
 * @return 无；会清零 parser 的既有半帧和回调状态。
 * 调用方式：在 BLE RX 队列消费者创建前调用；回调只应复制或提交规范化命令。
 * 线程约束：初始化必须与 feed 串行，运行中不得并发重置同一 parser。
 */
void sc_app_parser_init(sc_app_parser_t *parser,
                        sc_app_parser_callback_t callback,
                        sc_app_parser_error_callback_t error_callback,
                        void *context);

/**
 * @brief 喂入一段 App BLE 字节并按帧边界触发回调。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * @param parser 已初始化的可写解析状态；NULL 返回 0。
 * @param data 本次连续字节，只在调用期间读取；length=0 时允许 NULL。
 * @param length 输入字节数；有效参数下所有字节都会被扫描并合并到半帧状态。
 * @return 本次完成且通过尾字节/CRC 校验的帧数，不是消费字节数；错误帧不计数。
 * 调用方式：仅由 smartcar_service 的 BLE RX 队列消费者调用；支持拆包和多帧合包。
 * 线程约束：单 parser、单任务 owner；回调在本函数栈内同步执行，禁止递归或并发 feed。
 */
size_t sc_app_parser_feed(sc_app_parser_t *parser, const uint8_t *data,
                          size_t length);

/**
 * @brief 按当前默认版本编码一条 App BLE 帧。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * @param type App V1 消息类型，本函数不校验类型是否已注册。
 * @param payload 只读 payload；length=0 时允许 NULL，不得与 out 重叠。
 * @param length payload 字节数，不得超过 SC_APP_FRAME_MAX_PAYLOAD。
 * @param out 可写完整帧缓冲。
 * @param capacity out 的实际容量，至少为 SC_APP_FRAME_OVERHEAD + length。
 * @param out_length 成功时写入帧长；失败时保持调用前内容。
 * @return 0 成功；-1 表示指针组合非法；-2 表示长度或容量非法。
 * 调用方式：任务上下文编码后交给 s3_ble_notify_send()；输出不能直接当作 SRP 帧。
 * 线程约束：纯编码、可重入、不分配内存；禁止从 GATT/ISR 回调直接发送控制响应。
 */
int sc_app_frame_encode(uint8_t type, const uint8_t *payload, uint16_t length,
                        uint8_t *out, size_t capacity, uint16_t *out_length);

/**
 * @brief 指定 App 协议版本编码帧，用于 V1/V2 兼容或回放测试。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * @param version 只能为 SC_APP_FRAME_VERSION 或 SC_APP_FRAME_VERSION_V2。
 * @param type App 消息类型，不在本函数中查注册表。
 * @param payload 只读 payload；length=0 时允许 NULL，不得与 out 重叠。
 * @param length payload 字节数，不得超过 SC_APP_FRAME_MAX_PAYLOAD。
 * @param out 可写完整帧缓冲。
 * @param capacity out 的实际容量，至少为固定开销加 length。
 * @param out_length 成功时写入帧长；失败时保持调用前内容。
 * @return 0 成功；-1 表示指针组合非法；-2 表示版本、长度或容量非法。
 * 调用方式：只有明确知道接收端版本时使用；不得用来伪造 STM-S3 SRP 版本。
 * 线程约束：纯编码、可重入、不分配内存；禁止从 ISR 调用。
 */
int sc_app_frame_encode_version(uint8_t version, uint8_t type,
                                const uint8_t *payload, uint16_t length,
                                uint8_t *out, size_t capacity,
                                uint16_t *out_length);

#endif /* SMARTCAR_APP_PARSER_H */
