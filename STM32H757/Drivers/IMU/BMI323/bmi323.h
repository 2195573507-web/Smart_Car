#ifndef BMI323_H
#define BMI323_H

#include <stdint.h>

#include "bsp_status.h"

/* BMI323 设备驱动公共接口；创建人：待确认（当前维护人：Zhiqin）。
 * 加速度单位 m/s^2，角速度单位 rad/s，温度单位 degC；不负责标定/融合。 */

#ifdef __cplusplus
extern "C" {
#endif

#ifndef IMU_VECTOR3F_DEFINED
#define IMU_VECTOR3F_DEFINED
/**
 * 三轴浮点值容器；对象存储由调用方所有，具体物理单位由读写该对象的接口约定。
 */
typedef struct
{
    float x; /**< X 轴分量；单位见具体传感器读取接口。 */
    float y; /**< Y 轴分量；单位见具体传感器读取接口。 */
    float z; /**< Z 轴分量；单位见具体传感器读取接口。 */
} Vector3f;
#endif

#define BMI323_CHIP_ID_VALUE UINT16_C(0x0043)

/**
 * @brief 初始化兼容 BMI323 驱动并校验 CHIP_ID、复位及量程配置。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * 传入参数：无。
 * @return `BSP_STATUS_OK` 表示设备已就绪；其余 `bsp_status_t` 表示 SPI、
 *         身份校验或寄存器配置失败，失败后 `bmi323_is_ready()` 保持 0。
 * @note 调用方式与线程约束：仅在启动/恢复任务中、BSP GPIO/SPI 可初始化时调用；内部执行
 *       阻塞式 SPI、复位延时和有限 UART 日志，禁止从 ISR 或实时控制周期调用。
 */
bsp_status_t bmi323_init(void);
/**
 * @brief 以诊断模式初始化兼容 BMI323，并额外输出一次硬件探针信息。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * 传入参数：无。
 * @return 返回值和失效条件与 `bmi323_init()` 相同。
 * @note 调用方式与线程约束：仅用于受控启动诊断；会增加阻塞式 SPI、延时和 USART1 日志，
 *       不得与正常初始化并发，也不得从 ISR 调用。
 */
bsp_status_t bmi323_init_diag(void);
/**
 * @brief 复制初始化阶段已经校验并缓存的 BMI323 CHIP_ID 低字节。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * @param[out] chip_id 调用方拥有的 1 字节输出地址，不允许为 NULL。
 * @return `BSP_STATUS_OK` 表示输出有效；NULL 返回 `BSP_STATUS_INVALID_ARG`，
 *         驱动未就绪返回 `BSP_STATUS_NOT_READY`。
 * @note 调用方式与线程约束：初始化成功后在任务上下文读取；本函数不访问总线、不阻塞，
 *       输出存储由调用方管理。
 */
bsp_status_t bmi323_get_chip_id(uint8_t *chip_id);
/**
 * @brief 阻塞读取并换算 BMI323 三轴加速度。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * @param[out] acc 调用方拥有的三轴输出，单位为 m/s^2，不允许为 NULL。
 * @return `BSP_STATUS_OK` 时 `acc` 有效；参数、未就绪或 SPI 错误返回对应状态，
 *         失败时调用方不得使用本次输出。
 * @note 调用方式与线程约束：初始化成功后由单一传感器任务调用；内部占用 SPI/CS 并阻塞，
 *       禁止从 ISR 调用，且不得与同一 SPI 设备事务并发。
 */
bsp_status_t bmi323_read_acc(Vector3f *acc);
/**
 * @brief 阻塞读取并换算 BMI323 三轴角速度。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * @param[out] gyro 调用方拥有的三轴输出，单位为 rad/s，不允许为 NULL。
 * @return `BSP_STATUS_OK` 时 `gyro` 有效；参数、未就绪或 SPI 错误返回对应状态，
 *         失败时调用方不得使用本次输出。
 * @note 调用方式与线程约束：初始化成功后由单一传感器任务调用；函数阻塞访问 SPI，
 *       禁止从 ISR 调用，且不得与同一 SPI 设备事务并发。
 */
bsp_status_t bmi323_read_gyro(Vector3f *gyro);
/**
 * @brief 阻塞读取 BMI323 温度寄存器并换算为摄氏度。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * @param[out] temperature 调用方拥有的温度输出，单位为 degC，不允许为 NULL。
 * @return `BSP_STATUS_OK` 时输出有效；参数、未就绪或 SPI 错误返回对应状态，
 *         失败时调用方不得使用本次输出。
 * @note 调用方式与线程约束：初始化成功后在任务上下文按需调用；函数阻塞访问 SPI，
 *       禁止从 ISR 调用。
 */
bsp_status_t bmi323_read_temperature(float *temperature);
/**
 * @brief 查询兼容 BMI323 驱动是否完成身份校验和寄存器配置。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * 传入参数：无。
 * @return 1 表示初始化成功，0 表示未初始化或最近初始化失败；不代表新样本可用。
 * @note 调用方式与线程约束：任务上下文只读查询；不访问硬件、不阻塞，不能替代采样返回值。
 */
uint8_t bmi323_is_ready(void);

#ifdef __cplusplus
}
#endif

#endif /* BMI323_H */
