#ifndef CHASSIS_KINEMATICS_H
#define CHASSIS_KINEMATICS_H

#include <stdbool.h>

/*
 * 四轮差速运动学接口。
 * 创建人：待确认（当前维护人：Zhiqin）。
 * 契约补充日期：2026-08-31。
 * 轮序固定为 RR、RF、LR、LF；本文件只计算目标轮速，不直接写电机。
 */

#ifdef __cplusplus
extern "C" {
#endif

/* 左右轮中心轨距，单位 mm；属于车体标定常量。 */
#define CHASSIS_TRACK_WIDTH_MM 193.0f
/* MotorBoard 四轮数量，与下方枚举的索引顺序一致。 */
#define CHASSIS_WHEEL_COUNT 4U
/* 运动学输出的单轮速度绝对值上限，单位 mm/s。 */
#define CHASSIS_WHEEL_SPEED_LIMIT_MM_S 1000.0f

/** 四轮数组固定索引，必须与 MotorBoard 的 M1..M4 物理顺序一致。 */
typedef enum {
    CHASSIS_WHEEL_RR = 0U, /**< M1：右后轮。 */
    CHASSIS_WHEEL_RF = 1U, /**< M2：右前轮。 */
    CHASSIS_WHEEL_LR = 2U, /**< M3：左后轮。 */
    CHASSIS_WHEEL_LF = 3U  /**< M4：左前轮。 */
} chassis_wheel_index_t;

/**
 * @brief 根据底盘线速度和角速度计算四个轮子的目标速度。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * @param linear_mm_s 车体前向线速度，单位 mm/s，必须为有限值。
 * @param angular_rad_s 绕车体 Z 轴的角速度，单位 rad/s；正值表示左转，
 *        对应右轮加速、左轮减速。
 * @param wheel_speed 至少包含 CHASSIS_WHEEL_COUNT 个 float 的可写数组；
 *        成功后按 RR、RF、LR、LF 顺序写入，单位 mm/s。
 * @return true 表示四轮结果有限且未超限；false 表示指针、输入或结果无效，
 *         此时不修改 wheel_speed 内容。
 * 调用方式：chassis_task 在接受速度命令和每次输出前调用；主机测试
 * 可直接调用。
 * 线程约束：函数无全局状态、可重入、不阻塞；中断不应直接执行车辆控制决策。
 */
bool chassis_kinematics_compute(float linear_mm_s, float angular_rad_s,
                                float wheel_speed[CHASSIS_WHEEL_COUNT]);

#ifdef __cplusplus
}
#endif

#endif /* CHASSIS_KINEMATICS_H */
