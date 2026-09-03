#ifndef DUAL_AHRS_H
#define DUAL_AHRS_H

#include <stddef.h>
#include <stdint.h>

#include "imu_leveling.h"

/*
 * 双 IMU AHRS 融合接口。
 * 创建人：待确认（当前维护人：Zhiqin）。
 * 输入时间戳以微秒为主，输出姿态角为弧度；只有在标定和 freshness 条件满足时
 * 才能被底盘控制当作有效姿态。结构体是逻辑对象，不可直接当线缆帧发送。
 */

#ifdef __cplusplus
extern "C" {
#endif

#define DUAL_AHRS_SCHEMA UINT8_C(2)
#define DUAL_AHRS_PAYLOAD_LENGTH UINT16_C(80)
#define DUAL_AHRS_PI 3.14159265358979323846f
#define DUAL_AHRS_TWO_PI (2.0f * DUAL_AHRS_PI)

/**
 * @brief 通用三轴数值容器；具体坐标系和单位由外层字段定义。
 *
 * 该类型只保存三个浮点值，不持有缓冲区或硬件资源。用于传感器向量时分别表示
 * X/Y/Z 轴；用于 `delta_rad` 时依次表示 roll/pitch/yaw 差值。
 */
typedef struct
{
    float x; /**< 第一分量；传感器向量为 X 轴，姿态差值为 roll，单位随外层字段。 */
    float y; /**< 第二分量；传感器向量为 Y 轴，姿态差值为 pitch，单位随外层字段。 */
    float z; /**< 第三分量；传感器向量为 Z 轴，姿态差值为 yaw，单位随外层字段。 */
} dual_ahrs_vector3_t;

/**
 * @brief 一次冻结静态标定生成的双 IMU 偏置快照。
 *
 * 调用方持有原对象，`dual_ahrs_set_bias()` 会按值复制；三组偏置均在传感器坐标系中，
 * 由 DualAHRS 在水平旋转前扣除。
 */
typedef struct
{
    dual_ahrs_vector3_t bmi_accel; /**< BMI323 三轴加速度偏置，单位 m/s^2。 */
    dual_ahrs_vector3_t bmi_gyro;  /**< BMI323 三轴角速度偏置，单位 rad/s。 */
    dual_ahrs_vector3_t lsm_accel; /**< LSM303 三轴加速度偏置，单位 m/s^2。 */
} dual_ahrs_bias_t;

/**
 * @brief 与 DualAHRS 最终欧拉角处于同一参考系的姿态四元数。
 *
 * 四个分量均无量纲；有效姿态由实现归一化，并在航向零参考调整后与 roll/pitch/yaw
 * 重新同步。该值类型不表达指针所有权。
 */
typedef struct
{
    float w; /**< 四元数标量分量，无量纲。 */
    float x; /**< 四元数 X 向量分量，无量纲。 */
    float y; /**< 四元数 Y 向量分量，无量纲。 */
    float z; /**< 四元数 Z 向量分量，无量纲。 */
} dual_ahrs_quaternion_t;

/**
 * @brief 单个主或冗余估计器发布的姿态值快照。
 *
 * 只有 `valid` 非零时其余字段才可使用；无效不会保证数值字段已清零。通过
 * `dual_ahrs_get_output()` 取得的副本还会按读取时刻重新应用 freshness 门限。
 */
typedef struct
{
    float roll;  /**< 水平坐标系横滚角，单位 rad。 */
    float pitch; /**< 水平坐标系俯仰角，单位 rad。 */
    float yaw;   /**< 相对该分支启动航向零参考的航向角，单位 rad。 */
    dual_ahrs_quaternion_t quaternion; /**< 与上述最终欧拉角同步的无量纲四元数。 */
    uint8_t valid; /**< 非零表示该分支姿态当前可用；零时必须忽略其他姿态字段。 */
} dual_ahrs_attitude_t;

/**
 * @brief 由 IMU 单一生产者提交给一次 DualAHRS 更新的输入快照。
 *
 * 所有向量和时间戳均由调用方持有，`dual_ahrs_update()` 只在调用期间读取并按值复制，
 * 不保存任何外部地址。有效位只说明对应样本可参与本次处理，freshness 仍由时间戳判定。
 */
typedef struct
{
    dual_ahrs_vector3_t bmi_accel; /**< BMI323 三轴加速度样本，单位 m/s^2。 */
    dual_ahrs_vector3_t gyro;      /**< BMI323 三轴角速度样本，单位 rad/s。 */
    dual_ahrs_vector3_t lsm_accel; /**< LSM303 三轴加速度样本，单位 m/s^2。 */
    dual_ahrs_vector3_t mag;       /**< LSM303 三轴磁场样本，单位 uT。 */
    uint64_t bmi_timestamp_us; /**< BMI 加速度/陀螺共同采样时间，单调时钟微秒；0 表示无时间。 */
    uint64_t lsm_timestamp_us; /**< 兼容旧调用方的 LSM 共享时间；仅在对应独立时间为 0 时回退使用。 */
    uint64_t lsm_accel_timestamp_us; /**< LSM 加速度独立采样时间，单调时钟微秒；0 表示未提供。 */
    uint64_t lsm_mag_timestamp_us; /**< LSM 磁场独立采样时间，单调时钟微秒；0 表示未提供。 */
    uint8_t bmi_accel_valid; /**< 非零表示 `bmi_accel` 可用；零会使主分支无效。 */
    uint8_t bmi_gyro_valid;  /**< 非零表示 `gyro` 可用；零会使主分支无效。 */
    uint8_t lsm_accel_valid; /**< 非零表示 `lsm_accel` 可用；须与磁场配对才可更新冗余分支。 */
    uint8_t lsm_mag_valid;   /**< 非零表示 `mag` 可用；须与加速度配对才可更新冗余分支。 */
} dual_ahrs_input_t;

/**
 * @brief DualAHRS 标定、建链、跟踪和故障生命周期状态。
 *
 * `RESET`/`RUNNING` 是源代码兼容别名，数值上无法与其对应的主状态区分；`READY`
 * 仅表示偏置已接收且正等待有效样本，不代表底盘已经满足运动门禁。
 */
typedef enum
{
    DUAL_AHRS_STATE_WAIT_CAL = 0U, /**< 尚无有效偏置；融合更新被标定门禁拒绝。 */
    DUAL_AHRS_STATE_RESET = DUAL_AHRS_STATE_WAIT_CAL, /**< WAIT_CAL 的兼容别名，不是独立状态。 */
    DUAL_AHRS_STATE_WARMUP = 1U, /**< 已见传感器输入，但当前主、冗余姿态均尚未有效。 */
    DUAL_AHRS_STATE_TRACKING = 2U, /**< 主、冗余姿态均有效且处于各自 freshness 窗口。 */
    DUAL_AHRS_STATE_RUNNING = DUAL_AHRS_STATE_TRACKING, /**< TRACKING 的兼容别名。 */
    DUAL_AHRS_STATE_DEGRADED = 3U, /**< 至少一路不可用或过期，未满足双路新鲜跟踪条件。 */
    DUAL_AHRS_STATE_FAULT = 4U, /**< 两路姿态均无效且本轮没有可用于预热的加速度输入。 */
    DUAL_AHRS_STATE_READY = 5U /**< 偏置已接受、历史已复位，等待首批有效姿态样本。 */
} dual_ahrs_state_t;

/**
 * @brief DualAHRS 最新逻辑输出快照，不等同于 80 字节 SRP 线缆布局。
 *
 * 模块内部由单 writer 更新；getter 将其按值复制给调用方并在副本上修正 freshness，
 * 但当前无锁结构体复制不提供跨字段事务一致性。`valid`、`flags` 和 `state` 必须联合判断。
 */
typedef struct
{
    uint8_t schema; /**< 逻辑数据版本，当前固定为 `DUAL_AHRS_SCHEMA`（2）。 */
    uint8_t flags; /**< 位图：bit0 主有效、bit1 冗余有效、bit2 磁场样本有效、bit3 保留；
                    * bit4 重力置信度低于 0.5、bit5 磁场置信度低于 0.25、bit6 任一路过期、
                    * bit7 FAULT。getter 可按读取时刻清 bit0/bit1/bit2 并置 bit6。 */
    uint32_t timestamp_ms; /**< 本轮最新输入时间戳由 us 截断为 ms 后的低 32 位。 */
    uint32_t sample_sequence; /**< 通过标定门禁的更新序号；偏置提交/清除会归零，允许自然回绕。 */
    dual_ahrs_attitude_t primary; /**< BMI323 加速度+陀螺主估计器的姿态快照。 */
    dual_ahrs_attitude_t redundant; /**< LSM303 加速度+磁场冗余估计器的姿态快照。 */
    dual_ahrs_vector3_t delta_rad; /**< 主减冗余的 roll/pitch/yaw 最短角差，单位 rad；任一路无效时清零。 */
    float gravity_confidence; /**< BMI 加速度模长可信度，范围 0..1；当前不写入 80 字节 payload。 */
    float magnetic_confidence; /**< LSM 磁场模长可信度，范围 0..1；当前不写入 80 字节 payload。 */
    dual_ahrs_state_t state; /**< 本轮生命周期状态；getter 可能仅在返回副本中降级为 DEGRADED。 */
} dual_ahrs_output_t;

/* 角度差辅助函数为纯计算；两项 confidence 函数会读取当前 DualAHRS 参考值。 */
/**
 * @brief 根据加速度模长与当前本地重力参考计算 0..1 可信度。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * @param[in] accel 三轴加速度，单位 m/s^2；NULL 返回 0。
 * @return 0..1 的有限可信度，模长越接近当前重力参考越高。
 * @note 调用方式与线程约束：DualAHRS 更新或主机回放使用；不阻塞、不保存指针，但读取模块
 *       全局重力参考，不能与 setter 并发并视为确定性纯函数。
 */
float gravity_confidence(const dual_ahrs_vector3_t *accel);
/**
 * @brief 根据磁场模长与运行期参考模长计算 0..1 可信度。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * @param[in] mag 三轴磁场向量，单位沿用 LSM303 输入 uT；NULL/无效模长返回 0。
 * @return 0..1 的有限可信度；参考尚未建立且输入有效时返回 1。
 * @note 调用方式与线程约束：DualAHRS 更新或回放使用；不阻塞、不保存指针，但读取运行期
 *       全局磁场参考，须与单 writer 更新上下文保持串行。
 */
float mag_confidence(const dual_ahrs_vector3_t *mag);
/**
 * @brief 将弧度角包装到实现使用的 [-pi, pi] 区间。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * @param[in] angle 待包装角度，单位 rad。
 * @return 包装后的角度；输入非有限时返回 0。
 * @note 调用方式与线程约束：任务或主机测试均可调用；纯计算、不阻塞且可重入。
 */
float dual_ahrs_wrap_pi(float angle);
/**
 * @brief 计算主/冗余横滚角的最短有符号差。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * @param[in] primary 主横滚角，单位 rad。
 * @param[in] redundant 冗余横滚角，单位 rad。
 * @return `primary - redundant` 的包装结果；非有限输入经包装函数返回 0。
 * @note 调用方式与线程约束：融合更新或主机测试调用；纯计算、不阻塞且可重入。
 */
float delta_roll(float primary, float redundant);
/**
 * @brief 计算主/冗余俯仰角的最短有符号差。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * @param[in] primary 主俯仰角，单位 rad。
 * @param[in] redundant 冗余俯仰角，单位 rad。
 * @return `primary - redundant` 的包装结果；非有限输入经包装函数返回 0。
 * @note 调用方式与线程约束：融合更新或主机测试调用；纯计算、不阻塞且可重入。
 */
float delta_pitch(float primary, float redundant);
/**
 * @brief 计算主/冗余航向角的最短有符号差。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * @param[in] primary 主航向角，单位 rad。
 * @param[in] redundant 冗余航向角，单位 rad。
 * @return `primary - redundant` 的包装结果；非有限输入经包装函数返回 0。
 * @note 调用方式与线程约束：融合更新或主机测试调用；纯计算、不阻塞且可重入。
 */
float delta_yaw(float primary, float redundant);

/**
 * @brief 初始化 DualAHRS 滤波器、水平矩阵和运行状态。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * 传入参数：无。
 * @return 无返回值；完成后状态为 `DUAL_AHRS_STATE_WAIT_CAL`。
 * @note 调用方式与线程约束：IMU runtime 启动时、任何 setter/update/getter 前调用一次；清空
 *       全部历史和输出，非线程安全，禁止与融合/控制读取并发或从 ISR 调用。
 */
void dual_ahrs_init(void);
/* Pass NULL to hold the estimator at WAIT_CAL and clear runtime history. */
/**
 * @brief 提交双 IMU 偏置并重置融合历史，或撤销偏置回到 WAIT_CAL。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * @param[in] bias BMI 加速度/陀螺和 LSM 加速度偏置，单位分别为 m/s^2、rad/s、
 *                 m/s^2；调用期间复制。NULL 或任一非有限分量会清历史并保持 WAIT_CAL。
 * @return 无返回值。
 * @note 调用方式与线程约束：由 IMU manager 在静态标定结果冻结或失效时调用；会重置输出，
 *       必须与 `dual_ahrs_update()` 和所有读取串行，禁止从 ISR 调用。
 */
void dual_ahrs_set_bias(const dual_ahrs_bias_t *bias);
/**
 * @brief 复制 BMI323 与 LSM303 的冻结水平校准状态。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * @param[in] bmi BMI323 水平矩阵；NULL 时恢复单位矩阵/未计算状态。
 * @param[in] lsm LSM303 水平矩阵；NULL 时恢复单位矩阵/未计算状态。
 * @return 无返回值。
 * @note 调用方式与线程约束：由 IMU manager 在静态窗口提交或重置时调用；函数按值复制，
 *       不保存指针、不阻塞，但必须与融合更新串行。
 */
void dual_ahrs_set_leveling(const imu_leveling_state_t *bmi,
                            const imu_leveling_state_t *lsm);
/**
 * @brief 设置重力可信度使用的本地重力参考。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * @param[in] gravity_mps2 本地重力模长，单位 m/s^2；非有限或超出允许范围时
 *                         自动回退到默认值。
 * @return 无返回值。
 * @note 调用方式与线程约束：水平校准提交后、融合更新前调用；不阻塞，但必须与 confidence
 *       计算和融合更新串行。
 */
void dual_ahrs_set_local_gravity(float gravity_mps2);
/**
 * @brief 消费一组带独立时间戳/有效位的双 IMU 输入并推进融合状态机。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * @param[in] input BMI 加速度 m/s^2、角速度 rad/s、LSM 加速度 m/s^2、磁场 uT，
 *                  时间戳单位 us；指针仅在调用期间读取。NULL 或 WAIT_CAL 时不更新。
 * @return 无返回值；输出通过 getter 读取，无效/陈旧输入会降低有效位或状态。
 * @note 调用方式与线程约束：仅 BMI323 单一采样 owner 在新样本到达后调用；执行有限浮点计算，
 *       不访问总线、不阻塞，但无内部锁，禁止并发更新或从 ISR 调用。
 */
void dual_ahrs_update(const dual_ahrs_input_t *input);
/**
 * @brief 复制最新融合输出，并按当前单调时间重新应用主/冗余 freshness 标志。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * @param[out] output 调用方拥有的输出对象；NULL 时不执行复制。
 * @return 无返回值；过期分支的 `valid` 会清零，delta 也会在任一分支无效时清零。
 * @note 调用方式与线程约束：遥测/诊断任务读取；不保存指针、不阻塞，但当前实现无锁复制，
 *       与 writer 并发时不保证结构体事务级一致性，运动控制应使用 heading/fresh API。
 */
void dual_ahrs_get_output(dual_ahrs_output_t *output);

/* Returns the latest primary yaw and transformed/filtered body Z rate. */
/**
 * @brief 获取可用于底盘航向控制的主 AHRS 航向和机体系 Z 轴角速度。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * @param[out] yaw_rad 最新零参考航向角，单位 rad，不允许为 NULL。
 * @param[out] gyro_z_rad_s 滤波后的机体系 Z 轴角速度，单位 rad/s，不允许为 NULL。
 * @return 1 表示主分支有效、未超 freshness 窗口且两个输出有限；0 时输出不写入。
 * @note 调用方式与线程约束：底盘任务每个控制周期调用；不阻塞、不访问总线，当前实现无锁，
 *       调用方必须在返回 0 时立即走既有停机/门控路径。
 */
uint8_t dual_ahrs_get_heading_state(float *yaw_rad, float *gyro_z_rad_s);

/**
 * @brief 查询主 AHRS 最近样本是否仍处于底盘运动 freshness 窗口。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * 传入参数：无。
 * @return 1 表示主输出有效且当前时间与最近 BMI 时间戳差小于实现门限；0 表示
 *         无效、过期、时间倒退或尚无样本。
 * @note 调用方式与线程约束：姿态启动协调和底盘安全门周期调用；不阻塞、不访问总线，
 *       返回值是瞬时状态，不能跨控制周期缓存。
 */
uint8_t dual_ahrs_is_primary_fresh(void);

/* Serializes the schema=2 SRPv4 DualAHRS payload. */
/**
 * @brief 将经过当前 freshness 修正的 DualAHRS 快照序列化为 SRP schema=2。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31
 * @param[out] payload 调用方拥有的输出缓冲区，不允许为 NULL。
 * @param[in] capacity 输出容量，单位字节，至少为 `DUAL_AHRS_PAYLOAD_LENGTH`。
 * @return 成功返回固定 payload 字节数 80；参数或容量不足返回 -1。
 * @note 调用方式与线程约束：IMU 遥测任务调用；显式小端写入且不保存缓冲指针，不阻塞。
 *       成功只表示序列化完成，接收方仍必须检查 flags/schema/freshness。
 */
int dual_ahrs_pack_payload(uint8_t *payload, size_t capacity);

#ifdef __cplusplus
}
#endif

#endif /* DUAL_AHRS_H */
