#ifndef SMARTCAR_LOG_BRIDGE_H
#define SMARTCAR_LOG_BRIDGE_H

#include "srp_codec.h"
#include "srp_registry.h"

/**
 * @brief 处理一条 STM32->S3 的 SRP 日志帧并转发到 BLE/日志 sink。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * @param frame 已通过 SRP 解码的只读逻辑帧；payload 只在调用期间借用，允许传 NULL。
 * @return 无；类型/长度/字段非法时丢弃并记录警告；BLE FFE3 未就绪或提交失败时
 *         当前实现忽略发送返回值，可能静默丢弃，调用方不能据此确认 App 已收到。
 * 调用方式：只由 smartcar_service 在分发 SRP_MSG_ID_LOG 时调用；重编码为独立日志包络。
 * 线程约束：使用静态帧缓冲和无锁限频状态，只允许服务任务单 owner 调用；
 *           会调用 ESP 日志和 BLE 栈，禁止从 ISR、GATT 回调或其他任务并发调用。
 */
void log_bridge_handle(const srp_frame_t *frame);

#endif /* SMARTCAR_LOG_BRIDGE_H */
