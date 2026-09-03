#include <assert.h>
#include <stdint.h>

#include "radar_telemetry_age.h"

/* Telemetry age 纯函数主机测试；创建人：待确认（当前维护人：Zhiqin）。 */

/**
 * @brief 验证 1000 ms 门限为包含边界，只有严格大于门限才判 stale。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31。
 * 传入参数：无。
 * @return 返回值：无（void）；任一门限断言失败时终止测试进程。
 * 调用方式：由 main() 在 host test 中调用，不读取设备时钟。
 * 线程约束：单线程纯计算、无全局状态、无阻塞，不代表 ESP32-S3 实机计时已验证。
 */
static void test_stale_threshold_is_strictly_greater(void)
{
    const uint32_t ingress_ms = UINT32_C(10000);
    const uint32_t limit_ms = UINT32_C(1000);

    assert(!radar_telemetry_age_is_stale(ingress_ms, ingress_ms, limit_ms));
    assert(!radar_telemetry_age_is_stale(ingress_ms + UINT32_C(999),
                                         ingress_ms,
                                         limit_ms));
    assert(!radar_telemetry_age_is_stale(ingress_ms + UINT32_C(1000),
                                         ingress_ms,
                                         limit_ms));
    assert(radar_telemetry_age_is_stale(ingress_ms + UINT32_C(1001),
                                        ingress_ms,
                                        limit_ms));
}

/**
 * @brief 验证 uint32_t 单调毫秒计数跨回绕时仍保持 1000/1001 ms 门限语义。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31。
 * 传入参数：无。
 * @return 返回值：无（void）；模减法或 stale 判定不符时终止测试进程。
 * 调用方式：由 main() 使用靠近 UINT32_MAX 的合成时间戳调用。
 * 线程约束：单线程纯计算、无硬件访问；只验证小于一个完整回绕周期的 age 语义。
 */
static void test_uint32_wrap_preserves_age_boundary(void)
{
    const uint32_t ingress_ms = UINT32_MAX - UINT32_C(500);
    const uint32_t limit_ms = UINT32_C(1000);

    assert(!radar_telemetry_age_is_stale(UINT32_C(499),
                                         ingress_ms,
                                         limit_ms));
    assert(radar_telemetry_age_is_stale(UINT32_C(500),
                                        ingress_ms,
                                        limit_ms));
}

/**
 * @brief 顺序执行 telemetry age 门限与回绕主机断言。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31。
 * 传入参数：无。
 * @return 全部断言通过返回 0；任一 assert 失败会终止进程。
 * 调用方式：由 `radar/tests/run_host_tests.sh` 编译并直接运行。
 * 线程约束：单进程单线程，不创建 FreeRTOS 任务、TCP socket 或真实 telemetry 队列。
 */
int main(void)
{
    test_stale_threshold_is_strictly_greater();
    test_uint32_wrap_preserves_age_boundary();
    return 0;
}
