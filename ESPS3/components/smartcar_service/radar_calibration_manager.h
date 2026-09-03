#ifndef RADAR_CALIBRATION_MANAGER_H
#define RADAR_CALIBRATION_MANAGER_H

#include <stdbool.h>
#include <stdint.h>

#include "srp_link.h"

/*
 * S3 雷达/STM 标定握手状态机。
 * 创建人：待确认（当前维护人：Zhiqin）。
 * 只协调 PWM READY、静态标定事件和有限重试，不直接决定电机安全输出。
 */

/** S3 与 STM 之间的雷达零 PWM/静态标定握手状态；由服务任务单 owner 推进。 */
typedef enum {
    RADAR_WAIT_SYNC = 0,      /**< 等待合法 BOOT_READY，失败路径也回退到此状态。 */
    RADAR_SET_PWM,            /**< 已接受启动事件，下一 step 应用零 PWM 并提交事务。 */
    RADAR_WAIT_ACK,           /**< RADAR_PWM_READY 已提交，等待匹配 ACK/拒绝/超时。 */
    RADAR_WAIT_STATIC_EVENT,  /**< 已收到成功 ACK，等待 STATIC_CAL_DONE 事件。 */
    RADAR_CAL_DONE,           /**< 完成事件已接受且本地 radar_control 已进入 RUNNING。 */
    RADAR_CAL_ERROR           /**< 历史保留错误状态；当前失败路径直接回到 WAIT_SYNC。 */
} radar_calibration_state_t;

/**
 * @brief  提交 RADAR_PWM_READY SRP 事务的回调类型。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（接口契约补充）。
 * @param  speed_percent STM 请求的标定 PWM 百分数；当前状态机只提交 0。
 * @param  context 注册时保存的调用方上下文，可为 NULL。
 * @return 0 表示事务已被 SRP 链路接受；非 0 表示本次未提交，状态机将回到 WAIT_SYNC。
 * 调用方式：由 radar_calibration_manager_step() 在 S3 服务任务中同步调用。
 * 线程约束：不得无限阻塞或递归调用 manager；完成/超时结果另由 on_ready_response() 回送。
 */
typedef int (*radar_calibration_send_ready_t)(uint8_t speed_percent, void *context);

/**
 * @brief  清空标定跟踪状态并进入 RADAR_WAIT_SYNC。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（接口契约补充）。
 * 传入参数：无。
 * @return 无。
 * 调用方式：radar_control_init() 后、服务循环和所有 manager 事件前调用；重复调用会重置进度，
 *           但不会清除已注册的 transport，也不会主动改变当前 PWM。
 * 线程约束：仅 S3 服务任务/启动上下文调用；模块使用无锁静态状态，不可并发或从 ISR/GATT 回调调用。
 */
void radar_calibration_manager_init(void);
/**
 * @brief  注册 RADAR_PWM_READY 事务提交回调及其上下文。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（接口契约补充）。
 * @param  send_ready 回调；可传 NULL 取消 transport，此时下一次提交将失败并回到同步等待。
 * @param  context 原样传给 send_ready，不转移所有权；生命周期必须覆盖注册使用期。
 * @return 无。
 * 调用方式：初始化后、收到 BOOT_READY 前注册；不要在事务等待期间替换回调。
 * 线程约束：仅服务任务调用，无锁且不可与 step/事件处理并发。
 */
void radar_calibration_manager_set_transport(radar_calibration_send_ready_t send_ready,
                                             void *context);
/**
 * @brief  推进一次 PWM READY 提交或静态标定超时处理。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（接口契约补充）。
 * 传入参数：无。
 * @return 无；未初始化、未收到 BOOT_READY 或已完成时不动作。
 * 调用方式：由 smartcar_service 主循环周期调用；可能同步调用 transport 和 radar_control 接口。
 * 线程约束：仅服务任务；可能等待 radar_control mutex，禁止 ISR/GATT 回调和并发调用。
 */
void radar_calibration_manager_step(void);
/**
 * @brief  校验并消费 STM 的 BOOT_READY 标定门事件。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（接口契约补充）。
 * @param  payload 非 NULL、长度为 SRP_PAYLOAD_BOOT_READY_SIZE 的借用 payload。
 * @param  length payload 字节数；状态必须为 WAIT_RADAR_ZERO 且 reserved 字节为 0。
 * @return true 表示首次事件或合法重传被接受；状态/长度/内容不符返回 false。
 * 调用方式：仅在 SRP 帧 CRC、类型和会话已由 command_bridge 校验后调用；函数不保留 payload。
 * 线程约束：服务任务上下文；会调用 radar_control，可能等待 mutex，禁止 ISR 和并发调用。
 */
bool radar_calibration_manager_on_boot_ready(const uint8_t *payload, uint8_t length);
/**
 * @brief  消费 STM 的 STATIC_CAL_DONE 事件并尝试释放雷达运行门。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（接口契约补充）。
 * @param  payload 非 NULL、长度为 SRP_PAYLOAD_CAL_EVENT_SIZE 的借用 payload。
 * @param  length payload 字节数；事件 ID 必须为 SRP_CAL_EVENT_STATIC_DONE。
 * @return true 表示完成事件被接受或是完成后的合法重传；乱序、内容错误或控制门失败返回 false。
 * 调用方式：仅在已校验 SRP 事件路径调用；函数不保留 payload，成功后状态进入 RADAR_CAL_DONE。
 * 线程约束：服务任务上下文；会操作 radar_control mutex/LEDC，禁止 ISR 和并发调用。
 */
bool radar_calibration_manager_on_cal_event(const uint8_t *payload, uint8_t length);
/**
 * @brief  处理 RADAR_PWM_READY 事务的 ACK、拒绝或超时结果。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（接口契约补充）。
 * @param  result SRP 链路事务结果。
 * @param  status_code 快速响应状态；仅 TX_OK 加 SRP_FAST_RESP_OK 进入 WAIT_STATIC_EVENT。
 * @return 无；非 WAIT_ACK 状态的迟到/重复回调被忽略，失败会回到 WAIT_SYNC 并请求 PWM 安全复位。
 * 调用方式：作为对应 SRP 事务完成回调，由 command_bridge 转发；不要手工伪造 ACK。
 * 线程约束：必须与 manager 主状态机在同一服务任务串行执行；禁止 ISR 和并发调用。
 */
void radar_calibration_manager_on_ready_response(srp_link_tx_result_t result,
                                                 uint8_t status_code);
/**
 * @brief  返回当前雷达标定握手状态。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（接口契约补充）。
 * 传入参数：无。
 * @return radar_calibration_state_t 当前值；不会附带一致性版本号。
 * 调用方式：服务任务诊断/状态上报读取；不能单独据此绕过 radar_control 安全门。
 * 线程约束：无锁快照；只在 manager 所属服务任务内读取，跨任务读取需由调用方同步。
 */
radar_calibration_state_t radar_calibration_manager_get_state(void);
/**
 * @brief  返回最近一次 manager 请求的标定 PWM 百分数。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（接口契约补充）。
 * 传入参数：无。
 * @return 当前请求值；现有流程为 0，不代表 LEDC 写入或 STM ACK 已成功。
 * 调用方式：只用于诊断/状态显示，实际 PWM 由 radar_control 状态机确认。
 * 线程约束：无锁快照；仅服务任务读取。
 */
uint8_t radar_calibration_manager_get_pwm(void);
/**
 * @brief  查询本次 S3/STM 静态标定握手是否完成。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（接口契约补充）。
 * 传入参数：无。
 * @return true 仅表示 manager 已接受 STATIC_CAL_DONE 且 radar_control 已进入 RUNNING。
 * 调用方式：服务层状态上报读取；不等同于 UART/BLE/传感器或车辆验收结果。
 * 线程约束：无锁快照；仅服务任务读取。
 */
bool radar_calibration_manager_is_done(void);

#endif /* RADAR_CALIBRATION_MANAGER_H */
