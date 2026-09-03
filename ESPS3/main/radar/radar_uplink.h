#ifndef S3_RADAR_UPLINK_H
#define S3_RADAR_UPLINK_H

#include <stdbool.h>

#include "esp_err.h"

/* S3 雷达 Wi-Fi 上行服务；创建人：待确认（当前维护人：Zhiqin）。
 * 当前 TCP/S3RD 路径仍属实验性网关，任务就绪或 TCP CONNECTED 日志都不能替代
 * Windows 接收、解码、ROS2 LaserScan 和真实网络抓包验收。 */
/**
 * @brief 启动可选 Wi-Fi STA 和低优先级雷达上行任务。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（接口契约补充）。
 * 传入参数：无。
 * @return 功能在 Kconfig 中关闭时返回 ESP_OK 但不创建任何任务；启用时，ESP_OK 仅表示
 *         Wi-Fi/队列/telemetry sink/任务启动完成，配置、内存或 ESP-IDF 初始化失败返回对应错误。
 * 调用方式：app_main 在 radar_uart_init() 后调用一次；启用时必须配置非空 Wi-Fi 凭据、主机和端口。
 *           失败不得绕过本地雷达解析或影响 STM/车辆安全停机；部分初始化失败后不保证可直接重试。
 * 线程约束：只允许系统启动任务调用；会分配 PSRAM/内部 RAM、注册事件回调、启动 Wi-Fi 和 FreeRTOS
 *           任务，禁止 ISR/并发调用。
 */
esp_err_t radar_uplink_init(void);
/**
 * @brief  查询实验性上行功能是否完成本地初始化。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（接口契约补充）。
 * 传入参数：无。
 * @return Kconfig 关闭时固定 false；启用时仅当 initialized、Wi-Fi started 和任务句柄均有效才为 true。
 * 调用方式：状态/诊断读取；不代表已获得 IP、TCP 已连接、对端收包或 ROS2 已发布 /scan。
 * 线程约束：无锁快照、不阻塞；仅在初始化完成后读取。
 */
bool radar_uplink_is_running(void);

#endif
