#ifndef SRP_WIRE_H
#define SRP_WIRE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* SRP 显式线缆序列化接口；创建人：待确认（当前维护人：Zhiqin）。 */

/**
 * @brief 将无符号 32 位整数按小端序写入 4 字节线缆缓冲。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * @param out 至少 4 字节的可写缓冲；NULL 时静默返回。
 * @param value 待序列化的主机整数。
 * @return 无。
 * 调用方式：编码 SRP payload 时显式写字段，禁止用结构体 memcpy 替代。
 * 线程约束：纯转换、可重入、不阻塞，不保留 out 指针。
 */
void srp_wire_write_u32_le(uint8_t out[4], uint32_t value);

/**
 * @brief 从 4 个小端字节读取无符号 32 位整数。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * @param in 至少 4 字节的只读缓冲；NULL 时返回 0。
 * @return 解码值；返回 0 不能区分合法零值与 NULL 输入，调用方应先验证指针/长度。
 * 调用方式：完成消息 ID 和 payload 长度校验后读取固定字段。
 * 线程约束：纯转换、可重入、不阻塞，不保留 in 指针。
 */
uint32_t srp_wire_read_u32_le(const uint8_t in[4]);

/**
 * @brief 按 IEEE-754 binary32 位模式将 float 写入 4 个小端字节。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * @param out 至少 4 字节的可写缓冲；NULL 时静默返回。
 * @param value 原样序列化的单精度值；函数不拒绝 NaN 或 Inf。
 * @return 无。
 * 调用方式：业务层先完成有限性/范围校验，再编码 SRP float 字段。
 * 线程约束：使用局部变量和 memcpy，可重入、不阻塞，不依赖结构体对齐。
 */
void srp_wire_write_f32_le(uint8_t out[4], float value);

/**
 * @brief 从 4 个小端字节恢复 IEEE-754 binary32 单精度值。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * @param in 至少 4 字节的只读缓冲；NULL 被解释为全零位模式。
 * @return 解码 float；不会拒绝 NaN/Inf，NULL 返回 0.0f。
 * 调用方式：完成 payload 长度校验后读取，并由业务层继续执行 isfinite/范围校验。
 * 线程约束：纯转换、可重入、不阻塞，不保留输入指针。
 */
float srp_wire_read_f32_le(const uint8_t in[4]);

/**
 * @brief 连续写入 count 个小端单精度值。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * @param out 输出缓冲区，容量至少为 count * sizeof(float)；不得与 values 重叠。
 * @param values 至少 count 个元素的只读源数组，调用期间保持有效。
 * @param count 元素数量。
 * @return 无；任一指针为 NULL 时不写入，函数无法报告输出容量不足。
 * 调用方式：调用方用注册表固定长度分配缓冲后编码；业务层先检查每个值的有限性。
 * 线程约束：纯转换、可重入、不阻塞、不分配内存，也不保留输入输出指针。
 */
void srp_wire_write_f32_array_le(uint8_t *out, const float *values, size_t count);

/**
 * @brief 读取恰好 count 个小端 float，并逐项检查为有限值。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * @param in 只读输入，长度必须精确等于 count * sizeof(float)，不得为 NULL。
 * @param length in 的实际字节数；多一个或少一个字节都返回 false。
 * @param values 至少 count 个元素的可写数组，不得为 NULL 或与 in 重叠。
 * @param count 期望解码的 float 数量。
 * @return true 表示长度匹配且所有值有限；false 时 values 可能已被部分写入，必须整体丢弃。
 * 调用方式：消息 ID/固定 payload 长度校验后调用，成功后再做业务范围检查。
 * 线程约束：纯转换、可重入、不阻塞，不保留输入输出指针。
 */
bool srp_wire_read_f32_array_le(const uint8_t *in, size_t length, float *values,
                                size_t count);

#ifdef __cplusplus
}
#endif

#endif /* SRP_WIRE_H */
