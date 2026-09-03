#include "s3_ble_log_tx.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/**
 * @brief 检查 host 测试条件，失败时打印表达式与源码位置并终止测试进程。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param condition 仅求值一次的布尔表达式。
 * @return 宏本身无独立返回值；condition 为假时调用 exit(EXIT_FAILURE)，后续测试不再执行。
 * 调用方式：只在本文件 helper/test 中作为语句使用；失败信息写入 stderr。
 * 线程约束：单线程主机测试；不具备跨线程日志原子性，也不可在固件 ISR/任务错误路径中复用。
 */
#define CHECK(condition)                                                       \
    do {                                                                       \
        if (!(condition)) {                                                    \
            (void)fprintf(stderr, "CHECK failed at %s:%d: %s\n",             \
                          __FILE__, __LINE__, #condition);                     \
            exit(EXIT_FAILURE);                                                \
        }                                                                      \
    } while (0)

/**
 * @brief 构造以 id 递增填充的完整日志帧，并按指定优先级压入待测发送队列。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param[in,out] tx 已初始化且由调用方独占的 BLE 日志发送状态，不得为 NULL。
 * @param priority 待入队帧的普通或关键优先级。
 * @param id 帧首字节及后续递增测试模式的起始值。
 * @param length 帧长，必须位于 1..S3_BLE_LOG_TX_MAX_FRAME_SIZE。
 * @return 返回值：无（void）；长度非法或 enqueue 返回 false 时 CHECK 终止进程。
 * 调用方式：各场景在 prepare/complete 前调用；队列会复制局部 frame，函数返回后不保留栈指针。
 * 线程约束：单线程 host helper；tx 不含内部锁，不得与其他线程并发访问同一对象。
 */
static void enqueue_id(s3_ble_log_tx_t *tx,
                       s3_ble_log_tx_priority_t priority,
                       uint8_t id,
                       uint16_t length)
{
    uint8_t frame[S3_BLE_LOG_TX_MAX_FRAME_SIZE];

    CHECK(length > 0U && length <= sizeof(frame));
    for (uint16_t index = 0U; index < length; ++index) {
        frame[index] = (uint8_t)(id + index);
    }
    CHECK(s3_ble_log_tx_enqueue(tx, priority, frame, length));
}

/**
 * @brief 将队首完整帧按最大分片长度一次 prepare/complete，并返回其首字节标识。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param[in,out] tx 已连接、已开启 CCC 且至少含一条不超过最大帧长的待发队列状态。
 * @return 成功完成整帧后返回该帧首字节；任何前置状态、分片或完成结果不符时 CHECK 终止进程。
 * 调用方式：测试先调用 make_ready()，再用返回首字节断言双队列调度顺序。
 * 线程约束：单线程同步 host helper；prepare 与 complete 之间独占 tx，不模拟 GATT 异步回调。
 */
static uint8_t send_one_frame(s3_ble_log_tx_t *tx)
{
    s3_ble_log_tx_chunk_t chunk;

    CHECK(s3_ble_log_tx_prepare_chunk(tx, S3_BLE_LOG_TX_MAX_FRAME_SIZE,
                                      &chunk) == S3_BLE_LOG_TX_PREPARE_READY);
    CHECK(chunk.frame_offset == 0U);
    CHECK(chunk.frame_end);
    CHECK(s3_ble_log_tx_complete_chunk(tx, chunk.token, true) ==
          S3_BLE_LOG_TX_COMPLETE_FRAME_DONE);
    return chunk.data[0];
}

/**
 * @brief 将待测发送状态设置为 BLE 已连接且日志 CCC 已开启。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param[in,out] tx 已初始化且由调用方独占的 BLE 日志发送状态，不得为 NULL。
 * @return 返回值：无（void）；仅修改本地门控状态，不验证真实连接或手机订阅。
 * 调用方式：在需要 prepare 分片的测试阶段调用；不会自动解除 congested 状态。
 * 线程约束：单线程 host helper；tx 无内部锁，不得与连接事件或发送回调并发操作。
 */
static void make_ready(s3_ble_log_tx_t *tx)
{
    s3_ble_log_tx_set_connected(tx, true);
    s3_ble_log_tx_set_ccc_enabled(tx, true);
}

/**
 * @brief 验证普通队列按 FIFO 发送三帧，并检查入队、发送、深度和高水位统计。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return 返回值：无（void）；任一顺序、空队列状态或统计断言失败时 CHECK 终止进程。
 * 调用方式：由 main() 调用；初始化局部 tx，入队 1/2/3 后开启本地发送门控。
 * 线程约束：单线程确定性 host 测试；不覆盖 FreeRTOS 临界区、BLE 拥塞或手机接收。
 */
static void test_fifo(void)
{
    s3_ble_log_tx_t tx;
    s3_ble_log_tx_stats_t stats;

    s3_ble_log_tx_init(&tx);
    enqueue_id(&tx, S3_BLE_LOG_TX_PRIORITY_NORMAL, 1U, 1U);
    enqueue_id(&tx, S3_BLE_LOG_TX_PRIORITY_NORMAL, 2U, 1U);
    enqueue_id(&tx, S3_BLE_LOG_TX_PRIORITY_NORMAL, 3U, 1U);
    make_ready(&tx);

    CHECK(send_one_frame(&tx) == 1U);
    CHECK(send_one_frame(&tx) == 2U);
    CHECK(send_one_frame(&tx) == 3U);
    CHECK(s3_ble_log_tx_prepare_chunk(&tx, 20U, &(s3_ble_log_tx_chunk_t){0}) ==
          S3_BLE_LOG_TX_PREPARE_EMPTY);

    s3_ble_log_tx_get_stats(&tx, &stats);
    CHECK(stats.queued == 3U);
    CHECK(stats.sent_frames == 3U);
    CHECK(stats.current_depth == 0U);
    CHECK(stats.high_watermark == 3U);
}

/**
 * @brief 验证关键队列优先于普通队列，同时两个优先级内部各自保持 FIFO。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return 返回值：无（void）；实际发送顺序不是 20/21/10/11 时 CHECK 终止进程。
 * 调用方式：由 main() 调用；交错压入普通和关键帧，再通过 send_one_frame() 逐帧消费。
 * 线程约束：单线程 host 调度测试；不模拟生产任务与 GATT 完成回调并发。
 */
static void test_two_level_priority(void)
{
    s3_ble_log_tx_t tx;

    s3_ble_log_tx_init(&tx);
    enqueue_id(&tx, S3_BLE_LOG_TX_PRIORITY_NORMAL, 10U, 1U);
    enqueue_id(&tx, S3_BLE_LOG_TX_PRIORITY_CRITICAL, 20U, 1U);
    enqueue_id(&tx, S3_BLE_LOG_TX_PRIORITY_NORMAL, 11U, 1U);
    enqueue_id(&tx, S3_BLE_LOG_TX_PRIORITY_CRITICAL, 21U, 1U);
    make_ready(&tx);

    CHECK(send_one_frame(&tx) == 20U);
    CHECK(send_one_frame(&tx) == 21U);
    CHECK(send_one_frame(&tx) == 10U);
    CHECK(send_one_frame(&tx) == 11U);
}

/**
 * @brief 验证普通与关键队列满载后均覆盖各自最旧帧，并正确累计 drop 与当前深度。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return 返回值：无（void）；覆盖统计、容量或覆盖后首帧不符时 CHECK 终止进程。
 * 调用方式：由 main() 调用；分别重置 tx、填满一个优先级、追加 id=99，再确认 id=1 被丢弃。
 * 线程约束：单线程容量边界测试；不覆盖锁竞争期间的并发入队或真实 RAM/PSRAM 压力。
 */
static void test_drop_oldest(void)
{
    s3_ble_log_tx_t tx;
    s3_ble_log_tx_stats_t stats;

    s3_ble_log_tx_init(&tx);
    for (uint8_t id = 1U; id <= S3_BLE_LOG_TX_NORMAL_CAPACITY; ++id) {
        enqueue_id(&tx, S3_BLE_LOG_TX_PRIORITY_NORMAL, id, 1U);
    }
    enqueue_id(&tx, S3_BLE_LOG_TX_PRIORITY_NORMAL, 99U, 1U);
    s3_ble_log_tx_get_stats(&tx, &stats);
    CHECK(stats.drop_normal == 1U);
    CHECK(stats.current_depth == S3_BLE_LOG_TX_NORMAL_CAPACITY);
    make_ready(&tx);
    CHECK(send_one_frame(&tx) == 2U);

    s3_ble_log_tx_init(&tx);
    for (uint8_t id = 1U; id <= S3_BLE_LOG_TX_CRITICAL_CAPACITY; ++id) {
        enqueue_id(&tx, S3_BLE_LOG_TX_PRIORITY_CRITICAL, id, 1U);
    }
    enqueue_id(&tx, S3_BLE_LOG_TX_PRIORITY_CRITICAL, 99U, 1U);
    s3_ble_log_tx_get_stats(&tx, &stats);
    CHECK(stats.drop_critical == 1U);
    CHECK(stats.current_depth == S3_BLE_LOG_TX_CRITICAL_CAPACITY);
    make_ready(&tx);
    CHECK(send_one_frame(&tx) == 2U);
}

/**
 * @brief 验证连续发送四条关键帧后让出一次普通帧，随后继续发送剩余关键帧。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return 返回值：无（void）；发送顺序偏离 1/2/3/4/90/5/6 时 CHECK 终止进程。
 * 调用方式：由 main() 调用；先入队一条普通帧和六条关键帧，再逐帧完成发送。
 * 线程约束：单线程公平性测试；不证明持续多生产者负载下的调度时延上界。
 */
static void test_critical_burst_fairness(void)
{
    s3_ble_log_tx_t tx;

    s3_ble_log_tx_init(&tx);
    enqueue_id(&tx, S3_BLE_LOG_TX_PRIORITY_NORMAL, 90U, 1U);
    for (uint8_t id = 1U; id <= 6U; ++id) {
        enqueue_id(&tx, S3_BLE_LOG_TX_PRIORITY_CRITICAL, id, 1U);
    }
    make_ready(&tx);

    CHECK(send_one_frame(&tx) == 1U);
    CHECK(send_one_frame(&tx) == 2U);
    CHECK(send_one_frame(&tx) == 3U);
    CHECK(send_one_frame(&tx) == 4U);
    CHECK(send_one_frame(&tx) == 90U);
    CHECK(send_one_frame(&tx) == 5U);
    CHECK(send_one_frame(&tx) == 6U);
}

/**
 * @brief 验证 45 字节帧按 20/20/5 分片完整发送后才切换到下一条 7 字节帧。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return 返回值：无（void）；分片偏移、长度、尾标记、数据或发送统计不符时 CHECK 终止进程。
 * 调用方式：由 main() 调用；每个 prepare 得到的 token 均立即以成功结果 complete。
 * 线程约束：单线程同步分片测试；不覆盖 BLE MTU 动态变化、异步完成乱序或链路重传。
 */
static void test_fragmentation_does_not_interleave(void)
{
    s3_ble_log_tx_t tx;
    s3_ble_log_tx_chunk_t chunk;
    s3_ble_log_tx_stats_t stats;

    s3_ble_log_tx_init(&tx);
    enqueue_id(&tx, S3_BLE_LOG_TX_PRIORITY_NORMAL, 0x10U, 45U);
    enqueue_id(&tx, S3_BLE_LOG_TX_PRIORITY_NORMAL, 0x80U, 7U);
    make_ready(&tx);

    CHECK(s3_ble_log_tx_prepare_chunk(&tx, 20U, &chunk) ==
          S3_BLE_LOG_TX_PREPARE_READY);
    CHECK(chunk.frame_offset == 0U && chunk.length == 20U && !chunk.frame_end);
    CHECK(chunk.data[0] == 0x10U);
    CHECK(s3_ble_log_tx_complete_chunk(&tx, chunk.token, true) ==
          S3_BLE_LOG_TX_COMPLETE_MORE);

    CHECK(s3_ble_log_tx_prepare_chunk(&tx, 20U, &chunk) ==
          S3_BLE_LOG_TX_PREPARE_READY);
    CHECK(chunk.frame_offset == 20U && chunk.length == 20U && !chunk.frame_end);
    CHECK(chunk.data[0] == (uint8_t)(0x10U + 20U));
    CHECK(s3_ble_log_tx_complete_chunk(&tx, chunk.token, true) ==
          S3_BLE_LOG_TX_COMPLETE_MORE);

    CHECK(s3_ble_log_tx_prepare_chunk(&tx, 20U, &chunk) ==
          S3_BLE_LOG_TX_PREPARE_READY);
    CHECK(chunk.frame_offset == 40U && chunk.length == 5U && chunk.frame_end);
    CHECK(chunk.data[0] == (uint8_t)(0x10U + 40U));
    CHECK(s3_ble_log_tx_complete_chunk(&tx, chunk.token, true) ==
          S3_BLE_LOG_TX_COMPLETE_FRAME_DONE);

    CHECK(s3_ble_log_tx_prepare_chunk(&tx, 20U, &chunk) ==
          S3_BLE_LOG_TX_PREPARE_READY);
    CHECK(chunk.frame_offset == 0U && chunk.length == 7U && chunk.frame_end);
    CHECK(chunk.data[0] == 0x80U);
    CHECK(s3_ble_log_tx_complete_chunk(&tx, chunk.token, true) ==
          S3_BLE_LOG_TX_COMPLETE_FRAME_DONE);

    s3_ble_log_tx_get_stats(&tx, &stats);
    CHECK(stats.sent_frames == 2U);
    CHECK(stats.sent_chunks == 4U);
}

/**
 * @brief 验证分片发送中断连只丢弃当前部分发送帧，并保留其后的完整等待帧。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return 返回值：无（void）；partial_drop、等待深度或重连后帧标识不符时 CHECK 终止进程。
 * 调用方式：由 main() 调用；完成 45 字节帧首个分片后置断连，再恢复门控并发送 id=0x90 帧。
 * 线程约束：单线程状态转换测试；不模拟 GAP/GATT 回调时序或真实重连与 CCC 恢复过程。
 */
static void test_disconnect_drops_only_active_partial(void)
{
    s3_ble_log_tx_t tx;
    s3_ble_log_tx_chunk_t chunk;
    s3_ble_log_tx_stats_t stats;

    s3_ble_log_tx_init(&tx);
    enqueue_id(&tx, S3_BLE_LOG_TX_PRIORITY_NORMAL, 0x20U, 45U);
    enqueue_id(&tx, S3_BLE_LOG_TX_PRIORITY_NORMAL, 0x90U, 5U);
    make_ready(&tx);

    CHECK(s3_ble_log_tx_prepare_chunk(&tx, 20U, &chunk) ==
          S3_BLE_LOG_TX_PREPARE_READY);
    CHECK(s3_ble_log_tx_complete_chunk(&tx, chunk.token, true) ==
          S3_BLE_LOG_TX_COMPLETE_MORE);
    s3_ble_log_tx_set_connected(&tx, false);

    s3_ble_log_tx_get_stats(&tx, &stats);
    CHECK(stats.partial_drop == 1U);
    CHECK(stats.current_depth == 1U);

    make_ready(&tx);
    CHECK(send_one_frame(&tx) == 0x90U);
}

/**
 * @brief 验证拥塞门置位时 prepare 暂停，解除后保留帧可继续发送，并统计两次状态设置事件。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return 返回值：无（void）；暂停/恢复结果、完成状态或 congest_events 不符时 CHECK 终止进程。
 * 调用方式：由 main() 调用；同一 8 字节帧依次经历 set_congested(true/false) 后完成发送。
 * 线程约束：单线程 host 门控测试；不覆盖 ESP GATT 拥塞回调与发送任务并发唤醒。
 */
static void test_congestion_pause_resume(void)
{
    s3_ble_log_tx_t tx;
    s3_ble_log_tx_chunk_t chunk;
    s3_ble_log_tx_stats_t stats;

    s3_ble_log_tx_init(&tx);
    enqueue_id(&tx, S3_BLE_LOG_TX_PRIORITY_NORMAL, 0x44U, 8U);
    make_ready(&tx);
    s3_ble_log_tx_set_congested(&tx, true);
    CHECK(s3_ble_log_tx_prepare_chunk(&tx, 20U, &chunk) ==
          S3_BLE_LOG_TX_PREPARE_PAUSED);
    s3_ble_log_tx_set_congested(&tx, false);
    CHECK(s3_ble_log_tx_prepare_chunk(&tx, 20U, &chunk) ==
          S3_BLE_LOG_TX_PREPARE_READY);
    CHECK(s3_ble_log_tx_complete_chunk(&tx, chunk.token, true) ==
          S3_BLE_LOG_TX_COMPLETE_FRAME_DONE);

    s3_ble_log_tx_get_stats(&tx, &stats);
    CHECK(stats.congest_events == 2U);
}

/**
 * @brief 验证累计计数保持 UINT32_MAX 饱和、token 回绕跳过 0，并覆盖断连后的完成中止状态。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return 返回值：无（void）；token、完成状态或任一饱和计数发生回绕时 CHECK 终止进程。
 * 调用方式：由 main() 调用；白盒预置 tx 统计和 next_token 为 UINT32_MAX，再触发入队、拥塞、发送和断连路径。
 * 线程约束：单线程白盒 host 测试；直接访问内部字段，不可作为固件并发调用范例。
 */
static void test_counter_saturation_and_token_wrap(void)
{
    s3_ble_log_tx_t tx;
    s3_ble_log_tx_chunk_t chunk;
    s3_ble_log_tx_stats_t stats;

    s3_ble_log_tx_init(&tx);
    tx.stats.queued = UINT32_MAX;
    tx.stats.drop_normal = UINT32_MAX;
    tx.stats.sent_chunks = UINT32_MAX;
    tx.stats.sent_frames = UINT32_MAX;
    tx.stats.send_fail = UINT32_MAX;
    tx.stats.congest_events = UINT32_MAX;
    tx.stats.partial_drop = UINT32_MAX;
    tx.next_token = UINT32_MAX;

    for (uint8_t id = 1U; id <= S3_BLE_LOG_TX_NORMAL_CAPACITY; ++id) {
        enqueue_id(&tx, S3_BLE_LOG_TX_PRIORITY_NORMAL, id, 1U);
    }
    enqueue_id(&tx, S3_BLE_LOG_TX_PRIORITY_NORMAL, 99U, 1U);
    make_ready(&tx);
    s3_ble_log_tx_set_congested(&tx, true);
    s3_ble_log_tx_set_congested(&tx, false);
    CHECK(s3_ble_log_tx_prepare_chunk(&tx, 20U, &chunk) ==
          S3_BLE_LOG_TX_PREPARE_READY);
    CHECK(chunk.token == 1U);
    CHECK(s3_ble_log_tx_complete_chunk(&tx, chunk.token, true) ==
          S3_BLE_LOG_TX_COMPLETE_FRAME_DONE);
    CHECK(s3_ble_log_tx_prepare_chunk(&tx, 20U, &chunk) ==
          S3_BLE_LOG_TX_PREPARE_READY);
    s3_ble_log_tx_set_connected(&tx, false);
    CHECK(s3_ble_log_tx_complete_chunk(&tx, chunk.token, false) ==
          S3_BLE_LOG_TX_COMPLETE_ABORTED);

    s3_ble_log_tx_get_stats(&tx, &stats);
    CHECK(stats.queued == UINT32_MAX);
    CHECK(stats.drop_normal == UINT32_MAX);
    CHECK(stats.sent_chunks == UINT32_MAX);
    CHECK(stats.sent_frames == UINT32_MAX);
    CHECK(stats.send_fail == UINT32_MAX);
    CHECK(stats.congest_events == UINT32_MAX);
    CHECK(stats.partial_drop == UINT32_MAX);
}

/**
 * @brief 顺序执行 BLE 日志双优先级队列、分片、门控、饱和计数和 token 回绕主机测试。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return 全部检查通过返回 EXIT_SUCCESS；任一 CHECK 失败会先打印位置再以 EXIT_FAILURE 终止。
 * 调用方式：由 components/s3_ble/tests/run_host_tests.sh 编译并运行普通版与 sanitizer 版；无命令行参数。
 * 线程约束：单进程单线程 host 测试，不创建 FreeRTOS 任务，也不证明 BLE 手机端已收到日志。
 */
int main(void)
{
    test_fifo();
    test_two_level_priority();
    test_drop_oldest();
    test_critical_burst_fairness();
    test_fragmentation_does_not_interleave();
    test_disconnect_drops_only_active_partial();
    test_congestion_pause_resume();
    test_counter_saturation_and_token_wrap();

    (void)printf("s3_ble_log_tx host tests: PASS (state=%zu bytes)\n",
                 sizeof(s3_ble_log_tx_t));
    return EXIT_SUCCESS;
}
