#ifndef MOTOR_BOARD_PROTOCOL_H
#define MOTOR_BOARD_PROTOCOL_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* MotorBoard 文本协议封装；创建人：待确认（当前维护人：Zhiqin）。 */

#define MB_PROTOCOL_MAX_FRAME_LEN UINT16_C(128)

/** MotorBoard 文本/诊断解析后产生的逻辑帧类型。 */
typedef enum {
    MB_FRAME_INVALID = 0U, /**< 结构、数值、溢出或 raw diagnostic 帧。 */
    MB_FRAME_BATTERY, /**< `Battery:` 电压响应。 */
    MB_FRAME_MTEP, /**< 四路编码器脉冲响应，已应用诊断极性。 */
    MB_FRAME_MSPD, /**< 四路实际速度响应。 */
    MB_FRAME_MALL, /**< 四路累计脉冲响应。 */
    MB_FRAME_ACK, /**< 通用文本 ACK 类型的兼容值。 */
    /* Explicit name for the pure-text acknowledgement, kept value-compatible
     * with the original generic ACK type. */
    MB_FRAME_OK_ACK = MB_FRAME_ACK, /**< OK/ACK/Set 成功文本，与 ACK 同值。 */
    MB_FRAME_NACK, /**< NACK/NOK/ERROR/FAIL 文本。 */
    MB_FRAME_UNKNOWN, /**< 语法完整但未识别的普通响应。 */
    MB_FRAME_FLASH_RAW /**< read_flash 期间的未分类文本行。 */
} mb_frame_type_t;

/* Protocol naming used by the motor-board response specification. */
#define FRAME_TYPE_OK_ACK MB_FRAME_OK_ACK
#define FRAME_TYPE_FLASH_RAW MB_FRAME_FLASH_RAW

/** 一条已解析 MotorBoard 响应的按值逻辑容器。 */
typedef struct {
    mb_frame_type_t type; /**< 解析后的帧类型。 */
    char raw[MB_PROTOCOL_MAX_FRAME_LEN]; /**< 原始响应或格式化 raw 诊断，零结尾。 */
    float battery_voltage; /**< BATTERY 类型电压，单位 V。 */
    int32_t pulse[4]; /**< MTEP/MAll 四路脉冲，顺序 [RR,RF,LR,LF]。 */
    float speed[4]; /**< MSPD 四路速度，MotorBoard 原始单位/极性。 */
    char response_status[MB_PROTOCOL_MAX_FRAME_LEN]; /**< ACK/NACK 成功或错误文本副本。 */
} mb_protocol_frame_t;

/** MotorBoard parser 自初始化以来的累计统计。 */
typedef struct {
    uint32_t frames; /**< 产出的完整逻辑帧总数。 */
    uint32_t invalid_frames; /**< 类型为 INVALID 的帧数。 */
    uint32_t overflow_frames; /**< `$...#` 或文本行超容量次数。 */
} mb_protocol_stats_t;

/**
 * @brief 初始化 MotorBoard 文本解析状态并清零协议统计。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * 传入参数：无。返回值：无；不初始化 USART6 transport。
 * 调用方式：MB_Transport_Init() 后、MotorBoard 任务启动前调用一次。
 * 线程约束：单 parser owner；运行中不得与 Poll/Reset 并发调用。
 */
void MB_Protocol_Init(void);
/**
 * @brief 丢弃文本/二进制半帧并回到等待帧头状态。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * 传入参数：无。
 * 返回值：无；不清累计统计和底层 USART6 RX ring。
 * 调用方式：协议失步时先按需要清 transport RX，再调用本函数。
 * 线程约束：与 MB_Protocol_Poll() 串行化，禁止从 ISR 调用。
 */
void MB_Protocol_ResetRx(void);
/**
 * @brief 从 USART6 RX ring 轮询并解析至多一条完整响应。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * @param frame 可写输出；NULL 返回 false。成功时覆盖 type/raw 及对应业务字段。
 * @return true 表示产生一条完整文本/原始诊断帧，false 表示当前数据不足或参数无效。
 * 调用方式：MotorBoard 单一任务循环调用，true 时可继续调用以排空积压。
 * 线程约束：会修改 parser 状态和统计，不可重入，禁止从 ISR 调用。
 */
bool MB_Protocol_Poll(mb_protocol_frame_t *frame);
/**
 * @brief 格式化并排队 `$pwm:m1,m2,m3,m4#`。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * @param m1/m2/m3/m4 MotorBoard 原始有符号 PWM，顺序固定为 RR/RF/LR/LF。
 * @return true 仅表示完整文本已进入 TX ring；不代表物理发送或电机板执行成功。
 * 调用方式：由 MotorBoard 控制/安全任务生成四路命令后调用，失败时保持故障/停机状态。
 * 线程约束：任务上下文；底层用短临界区排队，禁止从 ISR 调用。
 */
bool MB_Protocol_SendPwm(int16_t m1, int16_t m2, int16_t m3, int16_t m4);
/**
 * @brief 格式化并排队 `$spd:m1,m2,m3,m4#`。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * @param m1/m2/m3/m4 电机板协议有符号速度值，顺序固定为 RR/RF/LR/LF。
 * @return true 表示已排队；false 表示格式或 TX ring 失败。
 * 调用方式：仅在明确使用 MotorBoard `$spd` 模式的任务路径调用。
 * 线程约束：任务上下文，禁止从 ISR 调用。
 */
bool MB_Protocol_SendSpeed(int16_t m1, int16_t m2, int16_t m3, int16_t m4);
/**
 * @brief 排队电机类型配置 `$mtype:value#`。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * @param motor_type 有效范围 1..5。
 * @return true 表示命令已排队，false 表示越界或 TX 失败。
 * 调用方式：MotorBoard 启动配置状态机在对应步骤调用并等待响应。
 * 线程约束：任务上下文；禁止从 ISR 调用。
 */
bool MB_Protocol_SendMotorType(uint8_t motor_type);
/**
 * @brief 排队磁导航线数量配置 `$mline:value#`。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * @param magnetic_line_count 必须大于 0。
 * @return true 表示已排队；false 表示参数或 TX 失败。
 * 调用方式：MotorBoard 启动配置状态机调用，成功排队后等待响应。
 * 线程约束：任务上下文；禁止从 ISR 调用。
 */
bool MB_Protocol_SendMagneticLine(uint16_t magnetic_line_count);
/**
 * @brief 排队减速/齿轮比配置 `$mphase:value#`。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * @param gear_ratio 必须大于 0，单位/缩放遵循电机板协议。
 * @return true 表示已排队；false 表示参数或 TX 失败。
 * 调用方式：MotorBoard 启动配置状态机调用，成功排队后等待响应。
 * 线程约束：任务上下文；禁止从 ISR 调用。
 */
bool MB_Protocol_SendGearRatio(uint16_t gear_ratio);
/**
 * @brief 排队轮径配置 `$wdiameter:value#`。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * @param diameter_mm 轮径毫米值，必须大于 0。
 * @return true 表示已排队；false 表示参数或 TX 失败。
 * 调用方式：MotorBoard 启动配置状态机调用，成功排队后等待响应。
 * 线程约束：任务上下文；禁止从 ISR 调用。
 */
bool MB_Protocol_SendWheelDiameter(uint16_t diameter_mm);
/**
 * @brief 排队电池电压查询 `$read_vol#`。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * 传入参数：无。
 * @return true 表示查询已排队；应由上层状态机等待 BATTERY 响应和超时。
 * 调用方式：MotorBoard 启动/周期状态机调用；返回 true 不表示已有新电压。
 * 线程约束：任务上下文；禁止从 ISR 调用。
 */
bool MB_Protocol_SendReadVoltage(void);
/**
 * @brief 排队 Flash 参数查询并启用后续原始/文本解析模式。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * 传入参数：无。
 * @return true 表示 `$read_flash#` 已排队；false 时不进入 read-flash 模式。
 * 调用方式：仅由 MotorBoard 启动配置状态机调用并等待相应响应/超时。
 * 线程约束：与 Poll/Reset 串行化，禁止从 ISR 调用。
 */
bool MB_Protocol_SendReadFlash(void);
/**
 * @brief 排队 `$upload:all_encoder,ten_ms_encoder,speed#` 上传配置。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * @param all_encoder/ten_ms_encoder/speed 分别映射为 0/1 标志。
 * @return true 表示命令已排队；上层仍须等待 ACK/数据流确认。
 * 调用方式：启动配置状态机设置反馈上传组合时调用。
 * 线程约束：任务上下文；禁止从 ISR 调用。
 */
bool MB_Protocol_SendUpload(bool all_encoder, bool ten_ms_encoder, bool speed);
/**
 * @brief 复制完整帧、无效帧和溢出帧累计统计。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * @param stats 可写输出；NULL 时直接返回。
 * 返回值：无，读取不清零。
 * 调用方式：MotorBoard owner 低频生成诊断快照时调用。
 * 线程约束：无锁快照；应由 parser owner 或暂停解析后调用。
 */
void MB_Protocol_GetStats(mb_protocol_stats_t *stats);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_BOARD_PROTOCOL_H */
