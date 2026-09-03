#ifndef SMARTCAR_SERVICE_H
#define SMARTCAR_SERVICE_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

/*
 * S3 网关服务公共接口。
 * 创建人：待确认（当前维护人：Zhiqin）。
 * 说明：服务任务串行处理 App BLE、SRP、STM UART 和安全恢复；调用者不能
 *       直接绕过服务向 STM32 发送运动帧。
 */

/**
 * @brief S3 收到 STM 遥测后的同步下游 sink 类型。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * @param message_id 已验证的 SRP 消息类型。
 * @param encoded_frame 只读完整 SRP v4 wire 帧，只在回调期间有效。
 * @param encoded_length encoded_frame 的实际字节数。
 * @param ingress_timestamp_ms S3 收到该帧的单调毫秒时间戳。
 * @param context 注册时保存的下游上下文，允许 NULL。
 * @return true 表示下游已复制/入队；false 表示拒绝或丢弃。
 * 调用方式：由 smartcar_service 解析任务同步调用，跨任务使用前复制整帧。
 * 线程约束：禁止阻塞网络/BLE/文件 I/O、保留输入指针或递归进入服务发送接口。
 */
typedef bool (*smartcar_service_telemetry_sink_t)(
    uint16_t message_id,
    const uint8_t *encoded_frame,
    uint16_t encoded_length,
    uint32_t ingress_timestamp_ms,
    void *context);

/**
 * @brief 注册遥测下游接收器。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * 传入参数：sink 接收完整 SRPv4 帧的回调；context 为调用方上下文。
 *          两者必须同时为 NULL 或同时非 NULL。
 * 返回值：ESP_OK 表示已注册；服务已初始化或参数不成对时返回错误。
 * 调用方式：必须在 smartcar_service_init() 前调用；回调只允许复制/入队，禁止阻塞 I/O。
 * 线程约束：回调运行于服务任务上下文，不得递归调用服务发送接口。
 */
esp_err_t smartcar_service_set_telemetry_sink(
    smartcar_service_telemetry_sink_t sink,
    void *context);

/**
 * @brief 初始化 S3 网关服务及其 SRP/App 解析、队列和任务。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * 传入参数：无。
 * 返回值：ESP_OK 表示所有必要资源创建成功；失败时服务不具备控制能力。
 * 调用方式：app_main 在 UART/BLE 基础驱动就绪后调用一次。
 * 安全约束：初始化失败或链路未同步时不得把状态当作可运动状态。
 * 线程约束：非幂等；仅启动任务调用，禁止与 BLE 注册/服务任务并发调用。
 */
esp_err_t smartcar_service_init(void);

#endif /* SMARTCAR_SERVICE_H */
