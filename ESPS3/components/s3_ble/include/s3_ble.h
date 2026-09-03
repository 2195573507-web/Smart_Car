#ifndef S3_BLE_H
#define S3_BLE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "smartcar_log.h"

/*
 * SmartCar S3 BLE GATT 传输层。
 * 创建人：待确认（当前维护人：Zhiqin）。
 * 传输层只负责连接、特征和有限通知；命令解析/仲裁由 smartcar_service 完成。
 */

/**
 * @brief 初始化 Bluedroid GATT 服务并开始广播。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * 传入参数：无。
 * 返回值：ESP_OK 表示初始化和异步应用注册已提交，或此前已成功初始化；否则返回首个 ESP-IDF 错误。
 * 调用方式：由 app_main 启动路径串行调用；失败可能保留部分协议栈资源，不得宣称 BLE 控制链路可用。
 * 线程约束：内部没有并发初始化锁；成功后的重复调用直接返回 ESP_OK，仅启动任务调用，禁止 ISR。
 */
esp_err_t s3_ble_init(void);

/**
 * @brief 向 FFE2 发送一条已编码的状态/控制响应通知。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * @param data 只读完整 App 帧；每次底层提交函数返回前必须保持有效，不转移所有权。
 * @param len App 帧有效字节数，范围为 1..S3 BLE 属性容量（当前实现为 1032 字节）。
 * @return 全部分片提交成功返回 ESP_OK；参数非法返回 ESP_ERR_INVALID_ARG，未初始化或 FFE2 未 ready
 *         返回 ESP_ERR_INVALID_STATE，底层提交失败返回对应错误；本接口不单独维护 FFE2 拥塞门。
 * 调用方式：服务任务编码 App 帧后调用；ESP_OK 只表示无确认 notification 已提交，不表示客户端收到或重组完成。
 * 线程约束：无锁读取连接/MTU 状态；禁止硬件 ISR 和多个任务并发分片发送，断开可与发送竞争。
 */
esp_err_t s3_ble_notify_send(const uint8_t *data, uint16_t len);

/**
 * @brief 仅通过 FFE3 发送一条完整独立日志帧。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * @param data smartcar_log_encode() 生成的只读完整帧；本函数返回前有效即可，不转移所有权。
 * @param len 完整帧长度，范围为 1..SMARTCAR_LOG_MAX_FRAME_SIZE，并须通过包络、长度和 CRC 校验。
 * @return ESP_OK 仅表示完整帧已复制入固定队列，不表示客户端收到；帧非法返回 ESP_ERR_INVALID_ARG。
 * 调用方式：多生产者任务调用，实际分片只由低优先级 FFE3 TX worker 执行。
 * 线程约束：零等待入队、禁止从硬件 ISR 调用；队满时按优先级队列丢最旧帧并计数。
 */
esp_err_t s3_ble_log_notify_send(const uint8_t *data, uint16_t len);

/**
 * @brief 构造一条 S3 来源日志并复制进 FFE3 固定队列。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * @param level SmartCarLog 日志级别；非法值由本函数或公共编码器拒绝。
 * @param text NUL 结尾只读文本字节串；NULL 或空串无效，不校验 UTF-8，超长时仅编码协议上限内前缀。
 * @return 参数非法返回 ESP_ERR_INVALID_ARG，编码失败返回 ESP_FAIL，入队成功返回 ESP_OK；队满覆盖同级最旧帧。
 * 调用方式：任务上下文调用；编码为完整帧后复制进 FFE3 固定队列。
 * 线程约束：不等待 BLE 发送，禁止从硬件 ISR 调用；不得持有控制路径锁调用。
 */
esp_err_t s3_ble_log_emit(smartcar_log_level_t level, const char *text);

/**
 * @brief 将 INFO 级 S3 诊断文本编码并复制进 FFE3 固定队列。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * 传入参数：text 语义同 s3_ble_log_emit()，NULL 或空串返回参数错误。
 * 返回值：编码/入队结果；ESP_OK 不表示 worker 已发送或手机已收到。
 * 调用方式：任务上下文记录 INFO 诊断，队列语义与 s3_ble_log_emit() 相同。
 * 线程约束：任务上下文；可能访问有界待发送日志队列，禁止从 ISR 调用。
 */
esp_err_t s3_log_info(const char *text);
/**
 * @brief 将 WARN 级 S3 诊断文本编码并复制进 FFE3 关键队列。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * 传入参数、返回值和线程约束同 s3_log_info()。
 * 调用方式：任务上下文记录 WARN 诊断；无论 BLE 是否 ready，成功都只表示完成本地入队。
 */
esp_err_t s3_log_warn(const char *text);
/**
 * @brief 将 ERROR 级 S3 诊断文本编码并复制进 FFE3 关键队列。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * 传入参数、返回值和线程约束同 s3_log_info()。
 * 调用方式：任务上下文发送 ERROR 诊断；不能代替故障状态或急停上报。
 */
esp_err_t s3_log_error(const char *text);

/**
 * @brief 查询 FFE2 状态/控制响应通知是否 ready。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * 传入参数：无。
 * 返回值：true 仅表示连接存在且客户端已打开 FFE2 CCC；不证明对端实际收到通知。
 * 调用方式：服务任务发送前或低频诊断读取；不能作为控制会话唯一准入条件。
 * 线程约束：只读状态查询，不阻塞；禁止据此绕过服务层准入。
 */
bool s3_ble_is_ready(void);
/**
 * @brief 查询 FFE3 日志通知是否 ready。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * 传入参数：无。
 * 返回值：true 仅表示模块已初始化、连接存在且 FFE3 CCC 已开启；不包含拥塞、队列或 worker 状态。
 * 调用方式：低频诊断读取；生产者无论 ready 与否都只入固定队列。
 * 线程约束：只读、不阻塞。
 */
bool s3_ble_is_log_ready(void);

/** FFE3 固定队列和唯一 TX worker 的只读累计统计。 */
typedef struct {
    uint32_t queued; /**< FFE3 成功接收入队的饱和累计帧数；不表示手机已收到。 */
    uint32_t sent_frames; /**< 最后分片提交成功的饱和累计完整帧数。 */
    uint32_t sent_chunks; /**< GATT 分片提交成功的饱和累计次数。 */
    uint32_t drop_normal; /**< 普通队列满时被覆盖的最旧帧饱和累计数。 */
    uint32_t drop_critical; /**< 关键队列满时被覆盖的最旧帧饱和累计数。 */
    uint32_t send_fail; /**< 底层 GATT 分片提交失败的饱和累计次数。 */
    uint32_t congest_events; /**< 拥塞状态更新调用的饱和累计次数，包含解除/重复赋值。 */
    uint32_t partial_drop; /**< 断连/关闭 CCC 时已有在途分片或成功前缀，或发送失败时已有成功前缀的 active 帧饱和累计数。 */
    uint32_t current_depth; /**< 两个完整帧等待队列的当前深度，不包含 active 帧。 */
    uint32_t high_watermark; /**< 自初始化以来 current_depth 的历史最大值。 */
} s3_ble_log_notify_stats_t;

/** 最近一次 GATT disconnect reason 和自启动以来的饱和累计次数。 */
typedef struct {
    bool valid; /**< true 表示至少记录过一次 GATT 断开，reason/count 可读。 */
    uint8_t reason; /**< 最近一次 ESP GATT 断开原始原因码，不是 SmartCar 业务错误码。 */
    uint32_t count; /**< 自启动以来 GATT 断开事件的饱和累计次数。 */
} s3_ble_disconnect_info_t;

/**
 * @brief 复制 FFE3 队列/TX 统计快照，读取不清零。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * @param stats 非 NULL 输出；current_depth 只含尚未开始的完整帧，不含 active 帧。
 * @return ESP_OK，或 stats=NULL 时返回 ESP_ERR_INVALID_ARG。
 * 调用方式：由任务低频读取链路健康度；ESP_OK 只表示完成本地快照复制，不证明手机已收到日志。
 * 线程约束：短临界区只读复制，任务上下文调用，禁止 ISR。
 */
esp_err_t s3_ble_get_log_notify_stats(s3_ble_log_notify_stats_t *stats);

/**
 * @brief 复制最近断开原因和累计次数，读取不清零。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * @param info 非 NULL 输出；尚未发生断开时 valid=false。
 * @return ESP_OK，或 info=NULL 时返回 ESP_ERR_INVALID_ARG。
 * 调用方式：由任务低频读取 GATT 断开诊断；必须先检查 valid，再解释 reason 和 count。
 * 线程约束：短临界区只读复制，任务上下文调用，禁止 ISR。
 */
esp_err_t s3_ble_get_disconnect_info(s3_ble_disconnect_info_t *info);

/**
 * @brief 读取 BLE 通知失败累计计数。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * 传入参数：无。
 * 返回值：自启动以来 FFE2/FFE3 底层 GATT 提交失败饱和累计，读取不清零。
 * 调用方式：保持旧语义兼容；队列 drop、未连接和 CCC 未开启不计入。
 * 线程约束：短临界区诊断快照，不作为单次发送确认。
 */
uint32_t s3_ble_get_notify_fail_count(void);

/**
 * @brief BLE FFE2 就绪边沿回调类型。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * @param context 注册时保存的调用方上下文，允许 NULL。
 * @return 无。
 * 调用方式：由 FFE2 状态从未就绪切换为就绪时触发；注册时已就绪则同步触发一次。
 * 调用上下文：状态从未就绪变为就绪时运行于 GATT 事件上下文；注册时若连接已经
 *             就绪，s3_ble_set_ready_callback() 会在注册调用者上下文同步调用一次。
 * 线程约束：实现必须快速返回，只允许置位、复制或通知业务任务。
 */
typedef void (*s3_ble_ready_callback_t)(void *context);

/**
 * @brief 注册“连接且 Notify 已开启”边沿回调。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * 传入参数：callback/context；callback=NULL 用于清除回调，非 NULL context 由调用方
 *           持有至回调清除或 BLE 服务结束。
 * 返回值：当前实现固定返回 ESP_OK。
 * 调用方式：优先在初始化前注册；若注册时已经 ready，会在本函数返回前同步回调。
 * 线程约束：回调不得执行阻塞 I/O，不得假定只运行于 GATT 线程。
 */
esp_err_t s3_ble_set_ready_callback(s3_ble_ready_callback_t callback,
                                    void *context);

/**
 * @brief BLE 连接断开同步回调类型。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * @param context 注册时保存的调用方上下文，允许 NULL。
 * @return 无。
 * 调用方式：由 Bluedroid GATT 断开事件同步触发。
 * 线程约束：运行于 GATT 事件上下文；
 * 回调不得阻塞或直接驱动硬件，停机/撤销会话由服务任务完成。
 */
typedef void (*s3_ble_disconnect_callback_t)(void *context);

/**
 * @brief 注册 GATT 断开回调。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * 传入参数：callback/context；callback=NULL 清除回调，context 不转移所有权。
 * 返回值：当前实现固定返回 ESP_OK。
 * 调用方式：回调必须快速返回，控制权撤销和停机由服务任务完成。
 * 线程约束：注册状态无锁更新，应在初始化阶段串行调用；回调运行于 GATT 事件上下文。
 */
esp_err_t s3_ble_set_disconnect_callback(s3_ble_disconnect_callback_t callback,
                                         void *context);

/**
 * @brief 兼容旧调用方的 FFE2 发送包装。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * 传入参数/返回值/线程约束同 s3_ble_notify_send()；新代码不得继续扩散本别名。
 * 调用方式：仅供历史调用点迁移，等价调用 s3_ble_notify_send()。
 */
esp_err_t s3_ble_send(const uint8_t *data, uint16_t len);

/**
 * @brief FFE1 写入数据的同步交接回调类型。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * @param data GATT 拥有的只读写入缓冲，只在回调期间有效。
 * @param len 本次写入字节数。
 * @param context 注册时保存的调用方上下文，允许 NULL。
 * @return 无。
 * 调用方式：由 Bluedroid GATT 写事件同步触发；接收方必须在返回前复制/入队。
 * 线程约束：运行于 GATT 回调上下文；
 * data 只在 Bluedroid GATT 回调期间有效；接收方必须复制后交给任务/队列，
 * 不得解析完整业务帧、阻塞或直接控制硬件。
 */
typedef void (*s3_ble_rx_callback_t)(const uint8_t *data, size_t len, void *context);

/**
 * @brief 注册唯一的 FFE1 写入消费者。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * @param callback FFE1 消费者；NULL 清除当前消费者。
 * @param context 回调上下文，只保存指针、不转移所有权，允许 NULL。
 * 注册只选择传输交接目标；非 NULL 接收方必须在 BLE 服务整个生命周期内有效，
 * 且不得在回调上下文解析帧或控制硬件。
 * 返回值：当前实现固定返回 ESP_OK。
 * 调用方式：smartcar_service 创建 BLE RX 队列后注册；停止服务时可传 NULL 清除。
 * 线程约束：应在初始化阶段调用，不与 GATT 写回调并发更换消费者。
 */
esp_err_t s3_ble_register_rx_callback(s3_ble_rx_callback_t callback,
                                      void *context);

/**
 * @brief 兼容旧名称的 RX 回调注册包装。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * 传入参数：callback/context 同 s3_ble_register_rx_callback()，callback=NULL 可清除。
 * 返回值：当前实现固定返回 ESP_OK。
 * 调用方式：仅供旧调用点兼容，新代码使用 s3_ble_register_rx_callback()。
 * 线程约束：同注册主接口，不与 GATT 写事件并发更换。
 */
esp_err_t s3_ble_set_rx_callback(s3_ble_rx_callback_t callback, void *context);

#endif /* S3_BLE_H */
