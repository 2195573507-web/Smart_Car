#ifndef LSM303_H
#define LSM303_H

#include <stdint.h>

#include "bsp_status.h"

/* LSM303 设备驱动公共接口；创建人：待确认（当前维护人：Zhiqin）。
 * 加速度输出单位 m/s^2，磁场输出单位 uT；本驱动不负责标定和融合。 */

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

/* LSM303DLHC 兼容模块的 7 位 I2C 地址。 */
#define LSM303_ACCEL_ADDRESS_DEFAULT UINT8_C(0x19)
#define LSM303_MAG_ADDRESS_DEFAULT   UINT8_C(0x1E)

/**
 * @brief 初始化 LSM303 加速度计和磁力计并校验两路设备身份。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * 传入参数：无。
 * @return `BSP_STATUS_OK` 表示 I2C4、地址探测和寄存器配置成功；其余状态
 *         表示总线、身份或配置失败，失败后 `lsm303_is_ready()` 保持 0。
 * @note 调用方式与线程约束：仅在启动/恢复任务调用；内部执行阻塞式 I2C、配置写入和
 *       有限 USART1 日志，禁止从 ISR 或实时控制周期调用。
 */
bsp_status_t lsm303_init(void);
/**
 * @brief 以诊断模式初始化 LSM303，并执行一次地址扫描和身份日志输出。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * 传入参数：无。
 * @return 返回值和失效条件与 `lsm303_init()` 相同。
 * @note 调用方式与线程约束：仅用于受控启动诊断；地址扫描会长时间占用 I2C4，禁止与
 *       采样任务并发或从 ISR 调用。
 */
bsp_status_t lsm303_init_diag(void);
/**
 * @brief 复制初始化阶段已经校验并缓存的加速度计 ID。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * @param[out] id 调用方拥有的 1 字节输出地址，不允许为 NULL。
 * @return `BSP_STATUS_OK` 表示输出有效；NULL 返回 `BSP_STATUS_INVALID_ARG`，
 *         驱动未就绪返回 `BSP_STATUS_NOT_READY`。
 * @note 调用方式与线程约束：初始化成功后在任务上下文读取；本函数不访问 I2C、不阻塞。
 */
bsp_status_t lsm303_get_accel_id(uint8_t *id);
/**
 * @brief 复制初始化阶段已经校验并缓存的磁力计三字节 ID。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * @param[out] id 调用方提供的至少 3 字节数组，不允许为 NULL。
 * @return `BSP_STATUS_OK` 表示三个输出字节有效；NULL 或未就绪返回对应错误。
 * @note 调用方式与线程约束：初始化成功后在任务上下文读取；本函数不访问 I2C、不阻塞，
 *       数组所有权始终属于调用方。
 */
bsp_status_t lsm303_get_mag_id(uint8_t id[3]);
/**
 * @brief 检查数据就绪位并阻塞读取 LSM303 三轴加速度。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * @param[out] acc 调用方拥有的三轴输出，单位为 m/s^2，不允许为 NULL。
 * @return `BSP_STATUS_OK` 时输出有效；无新样本或未初始化返回
 *         `BSP_STATUS_NOT_READY`，参数/I2C 失败返回对应状态。
 * @note 调用方式与线程约束：由 IMU 采样任务调用；内部阻塞占用 I2C4，禁止从 ISR 调用；
 *       非 OK 返回时不得把旧输出当作本次新样本。
 */
bsp_status_t lsm303_read_acc(Vector3f *acc);
/**
 * @brief 检查数据就绪位并阻塞读取 LSM303 三轴磁场。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * @param[out] mag 调用方拥有的三轴输出，单位为 uT，不允许为 NULL。
 * @return `BSP_STATUS_OK` 时输出有效；无新样本或未初始化返回
 *         `BSP_STATUS_NOT_READY`，参数/I2C 失败返回对应状态。
 * @note 调用方式与线程约束：由 IMU 采样任务调用；状态和数据使用两次阻塞 I2C 事务，
 *       禁止从 ISR 调用；非 OK 返回时不得使用本次输出。
 */
bsp_status_t lsm303_read_mag(Vector3f *mag);
/**
 * @brief 查询 LSM303 两路设备是否完成身份校验和寄存器配置。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * 传入参数：无。
 * @return 1 表示初始化成功，0 表示未初始化或失败；不代表当前有新数据。
 * @note 调用方式与线程约束：任务上下文只读查询；不访问 I2C、不阻塞，不能替代 read 返回值。
 */
uint8_t lsm303_is_ready(void);

#ifdef __cplusplus
}
#endif

#endif /* LSM303_H */
