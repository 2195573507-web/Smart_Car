#ifndef ATTITUDE_H
#define ATTITUDE_H

#include <stdint.h>

/* 兼容旧姿态接口；创建人：待确认（当前维护人：Zhiqin）。 */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 兼容姿态接口保留的四元数字段类型。
 *
 * 分量定义沿用 w/x/y/z、均无量纲；当前 `attitude.c` 不计算该四元数，公开状态中的
 * 四项保持全零，不能把它当作有效单位四元数使用。
 */
typedef struct
{
    float w; /**< 四元数标量分量；当前兼容实现保持为 0。 */
    float x; /**< 四元数 X 向量分量；当前兼容实现保持为 0。 */
    float y; /**< 四元数 Y 向量分量；当前兼容实现保持为 0。 */
    float z; /**< 四元数 Z 向量分量；当前兼容实现保持为 0。 */
} attitude_quaternion_t;

/**
 * @brief 旧 LSM303 加速度/磁场姿态路径的无锁值快照。
 *
 * getter 按值返回该结构，不转移任何资源；由于没有内部锁，与单 writer 更新并发时不保证
 * 跨字段事务一致性。结构内没有有效位，使用前必须另查 `attitude_get_status()`。
 */
typedef struct
{
    float roll;  /**< 相对 500 样本零参考并低通后的横滚角，单位 rad。 */
    float pitch; /**< 相对 500 样本零参考并低通后的俯仰角，单位 rad。 */
    float yaw;   /**< 相对圆周均值航向零参考并折返/低通后的航向角，单位 rad。 */
    attitude_quaternion_t quaternion; /**< 接口兼容占位；当前实现全零且不表示有效姿态。 */
} attitude_state_t;

/**
 * @brief 兼容姿态零位生命周期状态。
 *
 * 该状态仅反映 500 个有效样本的零位窗口是否完成，不包含 DualAHRS 主分支 freshness、
 * 传感器在线状态或底盘运动许可。
 */
typedef enum
{
    AHRS_WAIT_CAL = 0, /**< 初始化、复位或零位累计未完成，兼容姿态尚不可用。 */
    AHRS_READY /**< 零位累计已经完成；仍须由上层另行检查传感器与 DualAHRS 安全门。 */
} ahrs_state_t;

#define ATTITUDE_ZERO_SAMPLE_COUNT 500U

/**
 * @brief 初始化兼容姿态、零位累计器和 AHRS_WAIT_CAL 状态。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * 传入参数：无。
 * @return 无返回值。
 * @note 调用方式与线程约束：IMU runtime 启动阶段、任何 update/getter 前调用一次；会清空
 *       全部兼容输出，非线程安全，仅 IMU 生命周期 owner 调用，禁止从 ISR 调用。
 */
void attitude_init(void);
/**
 * @brief 清除零位累计、偏移和 READY 状态。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * 传入参数：无。
 * @return 无返回值。
 * @note 调用方式与线程约束：标定重启、输入失效或显式生命周期 reset 时调用；纯内存操作，
 *       但必须与 `attitude_update()`/getter 串行，禁止从 ISR 调用。
 */
void attitude_zero_reset(void);
/**
 * @brief 开始 500 个有效样本的零位累计窗口。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * 传入参数：无。
 * @return 无返回值；已在累计或已经 ready 时保持现状。
 * @note 调用方式与线程约束：标定结果和滤波输入就绪后由生命周期 owner 调用；完成前保持
 *       WAIT_CAL，必须与 `attitude_update()` 串行，禁止从 ISR 调用。
 */
void attitude_zero_init(void);
/**
 * @brief 兼容旧 API，查询生产零位窗口是否已经完成。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * 传入参数：无。
 * @return 1 表示生产零位窗口已经完成，0 表示未完成；函数不会捕获单样本或改偏移。
 * @note 调用方式与线程约束：旧调用方只可把它当作查询，新代码使用
 *       `attitude_zero_is_ready()`；只读、不阻塞，结果会随生命周期 reset 失效。
 */
uint8_t attitude_zero_capture_current(void);
/**
 * @brief 查询零位累计是否完成。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * 传入参数：无。
 * @return 1 表示软件零位已完成，0 表示未完成；不证明传感器 freshness/物理健康。
 * @note 调用方式与线程约束：启动协调任务只读查询；不阻塞，生命周期 reset 后返回值失效。
 */
uint8_t attitude_zero_is_ready(void);
/**
 * @brief 消费最新滤波加速度/磁场并推进零位累计和兼容姿态输出。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * 传入参数：无；内部读取 `imu_filter_get_output()` 快照，单位为
 *        m/s^2 和 uT。
 * @return 无返回值；标定未完成、滤波离线或非有限输入时不更新。
 * @note 调用方式与线程约束：仅 IMU 采样任务在有效滤波数据到达后周期调用；单 writer，
 *       包含浮点运算但不访问总线，禁止并发调用和 ISR 调用。
 */
void attitude_update(void);
/**
 * @brief 按值复制当前兼容欧拉角和四元数状态。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * 传入参数：无。
 * @return 当前状态副本，欧拉角单位 rad；调用方必须另行检查
 *         `attitude_get_status()`。
 * @note 调用方式与线程约束：兼容诊断/遥测读取；不阻塞但为无锁结构体复制，可能与 writer
 *       交错，安全控制优先使用 DualAHRS freshness API。
 */
attitude_state_t attitude_get_state(void);
/**
 * @brief 查询兼容姿态状态机。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * 传入参数：无。
 * @return `AHRS_WAIT_CAL` 或 `AHRS_READY`；READY 不替代 DualAHRS freshness 检查。
 * @note 调用方式与线程约束：启动协调或兼容状态消费者只读查询；不阻塞，reset 后结果失效。
 */
ahrs_state_t attitude_get_status(void);

#ifdef __cplusplus
}
#endif

#endif /* ATTITUDE_H */
