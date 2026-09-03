#ifndef S3_SERVICE_H
#define S3_SERVICE_H

#include <stdint.h>

#include "srp_registry.h"

/*
 * STM32 CM7 S3 服务层。
 * 创建人：待确认（当前维护人：Zhiqin）。
 * 该层是 SRP 会话、同步、遥测和安全失联的单一服务入口；最终运动安全仍由
 * chassis_task/MotorBoard 本地门控，调用者不得绕过服务直接写 UART。
 */

/**
 * @brief 初始化 SRP parser/link、mutex、同步状态和诊断计数。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * 传入参数：无。返回值：无；mutex 创建失败通过未初始化状态和日志体现。
 * 调用方式：UART link 初始化后、服务任务启动前调用；重复调用在已初始化时直接返回。
 * 线程约束：仅启动任务调用，禁止与 step/send/recover 并发，禁止从 ISR 调用。
 */
void s3_service_init(void);
/**
 * @brief 推进一次接收解析、ACK/重试、失联停机、BUS_OFF 恢复和波特率切换。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * 传入参数：无。返回值：无。
 * 调用方式：正常运行仅由 s3_service_task() 每 1 ms 调用；测试可在初始化后单步调用。
 * 线程约束：可能获取链路 mutex、调用阻塞 UART/HAL 和强制停机，禁止从 ISR 调用。
 */
void s3_service_step(void);
/**
 * @brief S3 SRP FreeRTOS 服务任务入口。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * 传入参数：argument 预留，当前不解引用。返回值：不返回。
 * 调用方式：仅由 s3_service_start() 创建；周期内调用 step、遥测和栈诊断。
 * 线程约束：单实例任务，不得手动从其他任务或 ISR 直接调用。
 */
void s3_service_task(void *argument);
/**
 * @brief 初始化并创建唯一 S3 服务任务。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * 传入参数：无。返回值：无；创建失败通过日志和空 task handle 体现。
 * 调用方式：CM7 启动路径调用一次；重复调用不会创建第二个任务。
 * 线程约束：仅启动任务调用，禁止从 ISR 调用。
 */
void s3_service_start(void);
/**
 * @brief 查询是否已验证 CMD_SYNC_REQ 并打开 STM-S3 控制会话。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * 传入参数：无。
 * 返回值：1 表示 HOST_SYNCED，0 表示未初始化/待同步/已超时；不证明 UART 物理质量。
 * 调用方式：底盘/安全/诊断任务读取会话门状态；不能替代姿态和本地故障门。
 * 线程约束：只读状态查询，不阻塞；运动路径仍须检查姿态和本地安全门。
 */
uint8_t s3_service_is_synced(void);

/**
 * @brief 发送一条 SRP 消息。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * 传入参数：priority/message_id/flags 为协议字段；payload/length 为只读 payload。
 * 返回值：0 表示已交给链路层；负值表示未同步、参数、BUS_OFF 或队列错误。
 * 调用方式：任务上下文调用；运动命令必须先经过本地安全准入。
 * 线程约束：最多等待链路 mutex 20 ms，底层 UART 发送还可能阻塞；禁止从 ISR 调用。
 */
int s3_service_send_message(uint8_t priority, uint16_t message_id, uint8_t flags,
                            const uint8_t *payload, uint8_t length);
/**
 * @brief 以 COMMAND 优先级发送启动/标定生命周期消息。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * 传入参数：message_id/flags/payload/length 必须符合 SRP 注册表。
 * 返回值：无；内部发送失败被丢弃，调用方须依靠状态机重试/诊断而非直接写 UART。
 * 调用方式：作为 imu_boot_manager transport 回调，仅任务上下文调用。
 * 线程约束：内部进入服务发送路径并可能等待 mutex/阻塞 UART；禁止从 ISR 调用。
 */
void s3_service_send_boot_message(uint16_t message_id, uint8_t flags,
                                  const uint8_t *payload, uint8_t length);
/**
 * @brief 发送固定 30 字节 IMU wire payload。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * 传入参数：payload 为只读 wire 字节，length 必须等于 SRP_PAYLOAD_IMU_TELEMETRY_SIZE。
 * 返回值：无；参数或发送失败时不产生帧。
 * 调用方式：IMU 遥测任务完成显式小端编码后调用；不得传 host struct sizeof。
 * 线程约束：任务上下文，payload 只需保持到函数返回。
 */
void s3_service_send_imu_telemetry(const uint8_t *payload, uint8_t length);
/**
 * @brief 校验并发送 DualAHRS schema=2 姿态 wire payload。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * 传入参数：payload/length 必须匹配注册表，schema=2 且 reserved 字节为 0。
 * 返回值：无；校验/发送失败时静默丢弃并由链路统计观察。
 * 调用方式：DualAHRS 快照成功打包为 80 字节 wire payload 后调用。
 * 线程约束：任务上下文，禁止从 ISR 调用。
 */
void s3_service_send_dual_attitude(const uint8_t *payload, uint8_t length);
/**
 * @brief 校验并发送底盘状态 wire payload，不改变控制目标。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * 传入参数：payload/length 必须匹配 chassis schema 和 reserved 约束。
 * 返回值：无；失败时不产生帧。线程约束：仅任务上下文。
 * 调用方式：底盘状态任务显式编码固定 24 字节 wire payload 后调用。
 */
void s3_service_send_chassis_state(const uint8_t *payload, uint8_t length);
/**
 * @brief 校验并发送四轮闭环状态 wire payload。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * 传入参数：payload/length 必须匹配 schema、模式和 reserved 约束。
 * 返回值：无；失败时不产生帧。线程约束：仅任务上下文。
 * 调用方式：MotorBoard 控制任务生成固定 44 字节闭环状态 payload 后调用。
 */
void s3_service_send_wheel_control_status(const uint8_t *payload, uint8_t length);
/**
 * @brief 发送一条已构造的 SRP LOG payload。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * 传入参数：payload/length 为只读日志 payload，长度受 SRP_MAX_PAYLOAD 限制。
 * 返回值：0 表示链路接受；负值表示状态、锁、编码或 UART 发送失败。
 * 调用方式：log_service 构造来源/级别/时间戳/文本包络后调用；失败时只计数不阻塞控制。
 * 线程约束：任务上下文；日志失败不得阻塞控制或无限重试。
 */
int s3_service_send_log(const uint8_t *payload, uint8_t length);

#endif /* S3_SERVICE_H */
