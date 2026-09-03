#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "radar_frame_fifo.h"

/* 雷达 FIFO 主机测试；创建人：待确认（当前维护人：Zhiqin）。 */

/**
 * @brief 验证 FIFO 按入队顺序复制帧，并保留序号、时间戳及深度统计。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return 返回值：无（void）；任一顺序、元数据、内容或统计断言失败时终止测试进程。
 * 调用方式：由 main() 顺序调用；修改第二次 input 后确认第一项已被 FIFO 独立复制。
 * 线程约束：单线程主机测试，fifo/缓冲由本函数独占；不覆盖外层 FreeRTOS mutex 或生产者并发。
 */
static void test_fifo_preserves_order_and_metadata(void)
{
    radar_frame_fifo_t fifo;
    static radar_frame_fifo_entry_t entries[4];
    uint8_t input[3] = {0xAAU, 0x55U, 0x01U};
    uint8_t output[RADAR_PARSER_MAX_FRAME_SIZE] = {0};
    size_t length = 0U;
    uint32_t sequence = 0U;
    uint32_t timestamp_ms = 0U;
    radar_frame_fifo_stats_t stats;

    assert(radar_frame_fifo_init(&fifo, entries, 4U));
    assert(radar_frame_fifo_push(&fifo, input, sizeof(input), 10U, 100U));
    input[2] = 0x02U;
    assert(radar_frame_fifo_push(&fifo, input, sizeof(input), 11U, 200U));

    assert(radar_frame_fifo_pop(&fifo, output, sizeof(output), &length,
                                &sequence, &timestamp_ms));
    assert(length == sizeof(input));
    assert(sequence == 10U);
    assert(timestamp_ms == 100U);
    assert(memcmp(output, (uint8_t[]){0xAAU, 0x55U, 0x01U}, length) == 0);

    assert(radar_frame_fifo_pop(&fifo, output, sizeof(output), &length,
                                &sequence, &timestamp_ms));
    assert(sequence == 11U);
    assert(timestamp_ms == 200U);
    radar_frame_fifo_get_stats(&fifo, &stats);
    assert(stats.capacity == 4U);
    assert(stats.count == 0U);
    assert(stats.high_watermark == 2U);
    assert(stats.dropped_oldest_count == 0U);
}

/**
 * @brief 验证 FIFO 满时仅丢弃最旧项，新项入队且水位/丢弃计数正确。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return 返回值：无（void）；容量、计数、首个弹出序号或清空语义不符时 assert 终止。
 * 调用方式：由 main() 调用；填满固定容量后再推入第九项，再逐项弹出至空。
 * 线程约束：单线程主机测试；静态 entries 由本测试独占，不模拟锁竞争或实时上行消费。
 */
static void test_full_fifo_drops_oldest_only(void)
{
    radar_frame_fifo_t fifo;
    static radar_frame_fifo_entry_t entries[RADAR_FRAME_FIFO_DEPTH];
    uint8_t input[1] = {0U};
    uint8_t output[RADAR_PARSER_MAX_FRAME_SIZE] = {0};
    size_t length = 0U;
    uint32_t sequence = 0U;
    radar_frame_fifo_stats_t stats;

    assert(radar_frame_fifo_init(&fifo, entries, RADAR_FRAME_FIFO_DEPTH));
    for (uint32_t value = 1U; value <= RADAR_FRAME_FIFO_DEPTH; ++value) {
        input[0] = (uint8_t)value;
        assert(radar_frame_fifo_push(&fifo, input, sizeof(input), value, value));
    }
    input[0] = 9U;
    assert(radar_frame_fifo_push(&fifo, input, sizeof(input), 9U, 9U));

    radar_frame_fifo_get_stats(&fifo, &stats);
    assert(stats.capacity == RADAR_FRAME_FIFO_DEPTH);
    assert(stats.count == RADAR_FRAME_FIFO_DEPTH);
    assert(stats.high_watermark == RADAR_FRAME_FIFO_DEPTH);
    assert(stats.dropped_oldest_count == 1U);
    assert(radar_frame_fifo_pop(&fifo, output, sizeof(output), &length,
                                &sequence, NULL));
    assert(sequence == 2U);
    assert(output[0] == 2U);
    while (radar_frame_fifo_pop(&fifo, output, sizeof(output), &length,
                                &sequence, NULL)) {
    }
    assert(length == 0U);
}

/**
 * @brief 验证输出缓冲过短时报告所需长度且不消费队首帧。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return 返回值：无（void）；短缓冲返回、长度或随后成功弹出语义不符时 assert 终止。
 * 调用方式：由 main() 调用；先用 2 字节输出尝试弹出 4 字节帧，再以足够容量重试。
 * 线程约束：单线程主机测试，所有数组在本函数返回前有效；不涉及 DMA 或跨任务所有权。
 */
static void test_short_output_does_not_consume(void)
{
    radar_frame_fifo_t fifo;
    static radar_frame_fifo_entry_t entries[1];
    uint8_t input[4] = {1U, 2U, 3U, 4U};
    uint8_t output[2] = {0};
    size_t length = 0U;
    uint32_t sequence = 0U;

    assert(radar_frame_fifo_init(&fifo, entries, 1U));
    assert(radar_frame_fifo_push(&fifo, input, sizeof(input), 7U, 70U));
    assert(!radar_frame_fifo_pop(&fifo, output, sizeof(output), &length,
                                 &sequence, NULL));
    assert(length == sizeof(input));
    assert(radar_frame_fifo_pop(&fifo, input, sizeof(input), &length,
                                &sequence, NULL));
    assert(length == sizeof(input));
    assert(sequence == 7U);
}

/**
 * @brief 验证外部 entries 为空或容量为零时 FIFO 初始化被拒绝。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return 返回值：无（void）；任一非法存储被接受时 assert 终止测试进程。
 * 调用方式：由 main() 调用，分别传 NULL 存储和零容量；不读取失败后的 fifo 内容。
 * 线程约束：单线程纯内存测试，无 RTOS/ISR/硬件访问。
 */
static void test_invalid_storage_is_rejected(void)
{
    radar_frame_fifo_t fifo;
    radar_frame_fifo_entry_t entries[1];

    assert(!radar_frame_fifo_init(&fifo, NULL, 1U));
    assert(!radar_frame_fifo_init(&fifo, entries, 0U));
}

/**
 * @brief 顺序运行雷达完整帧 FIFO 的主机断言集合。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return 全部断言通过返回 0；assert 失败会终止进程而不返回非零状态。
 * 调用方式：由 run_host_tests.sh 编译为独立 executable 后直接运行。
 * 线程约束：单进程单线程，不创建 FreeRTOS 任务；通过结果不证明 UART 生产者与 TCP 消费者并发正确。
 */
int main(void)
{
    test_fifo_preserves_order_and_metadata();
    test_full_fifo_drops_oldest_only();
    test_short_output_does_not_consume();
    test_invalid_storage_is_rejected();
    return 0;
}
