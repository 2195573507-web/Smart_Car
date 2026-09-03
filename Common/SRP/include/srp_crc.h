#ifndef SRP_CRC_H
#define SRP_CRC_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 计算 CRC-16/CCITT-FALSE。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * @param data 参与计算的字节数组；允许在 length 为 0 时为 NULL。
 * @param length 字节数，不包含协议头或尾之外的隐式数据。
 * @return 初值 0xFFFF、多项式 0x1021 的 16 位 CRC；data=NULL 且 length>0 时返回 0。
 * 调用方式：普通任务或主机测试均可调用；函数不访问硬件、全局状态或堆。
 * 线程约束：纯函数、可重入且不阻塞；调用方负责按 SRP 规定的小端序列化，
 *           ISR 中应避免对无界长缓冲计算。
 */
uint16_t srp_crc16_ccitt_false(const uint8_t *data, size_t length);

/**
 * @brief 计算 CRC-16/MODBUS。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（接口契约补充）。
 * @param data 参与计算的字节数组；允许在 length 为 0 时为 NULL。
 * @param length 字节数。
 * @return 初值 0xFFFF、多项式 0xA001 的 16 位 CRC；data=NULL 且 length>0 时返回 0。
 * 调用方式：只用于明确要求 MODBUS CRC 的 App/日志封装或主机测试，不替代 SRP 主 CRC。
 * 线程约束：纯函数、可重入且不阻塞；ISR 中应避免对无界长缓冲计算。
 */
uint16_t srp_crc16_modbus(const uint8_t *data, size_t length);

#ifdef __cplusplus
}
#endif

#endif /* SRP_CRC_H */
