#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "radar_parser.h"

/* 雷达解析器主机测试；创建人：待确认（当前维护人：Zhiqin）。 */

typedef struct {
    uint32_t count;
    size_t last_length;
    uint8_t last_frame[RADAR_PARSER_MAX_FRAME_SIZE];
} frame_capture_t;

/**
 * @brief 从测试帧的连续两字节读取小端 uint16_t。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param data 至少含 2 个可读字节的借用指针，不得为 NULL。
 * @return 解码后的 16 位值。
 * 调用方式：构造测试校验和时在已知固定字段/样本边界内调用。
 * 线程约束：单线程主机纯读取、可重入、不保留 data；函数本身不做边界检查。
 */
static uint16_t read_le16(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8U);
}

/**
 * @brief 将 uint16_t 按小端序写入测试帧的连续两字节。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param data 至少含 2 个可写字节的调用方缓冲，不得为 NULL。
 * @param value 待编码的 16 位值。
 * @return 返回值：无（void）。
 * 调用方式：make_frame() 和相关黄金数据构造路径在容量已计算后调用。
 * 线程约束：单线程纯内存写入、可重入；不检查容量，同一 data 不得并发写。
 */
static void write_le16(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8U);
}

/**
 * @brief 按 X3/X3PRO 官方异或顺序计算合成帧的校验字段。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param frame 已填入头、角度和样本的只读测试帧，不得为 NULL。
 * @param sample_count 样本数量，必须与 frame[3] 及缓冲内容一致。
 * @param sample_bytes 每样本字节数；当前调用只使用 2 或 3。
 * @return 计算得到的 16 位 XOR 校验和。
 * 调用方式：仅由 make_frame() 在完整填充样本后调用，函数不验证输入布局或容量。
 * 线程约束：单线程主机纯计算、可重入，不保留 frame；错误参数可能导致越界读取。
 */
static uint16_t make_checksum(const uint8_t *frame,
                              size_t sample_count,
                              size_t sample_bytes)
{
    uint16_t checksum = 0x55AAU;
    checksum ^= read_le16(&frame[4]);
    for (size_t index = 0U; index < sample_count; ++index) {
        const size_t offset = RADAR_X3PRO_HEADER_BYTES + index * sample_bytes;
        if (sample_bytes == 2U) {
            checksum ^= read_le16(&frame[offset]);
        } else {
            checksum ^= frame[offset];
            checksum ^= read_le16(&frame[offset + 1U]);
        }
    }
    checksum ^= (uint16_t)(((uint16_t)frame[3] << 8U) | frame[2]);
    checksum ^= read_le16(&frame[6]);
    return checksum;
}

/**
 * @brief 构造一条具有合法头、角度检查位、样本和校验和的合成雷达帧。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param[out] frame 至少可写 `HEADER + sample_count * sample_bytes` 字节的缓冲。
 * @param ct 写入帧的 CT 字节。
 * @param sample_count 合成样本数，须在协议/缓冲容量范围内。
 * @param sample_bytes 每样本 2 字节距离或 3 字节强度+距离布局。
 * @return 构造后的完整帧字节数。
 * 调用方式：各 parser 测试在栈上最大帧缓冲中调用，随后按返回长度 feed。
 * 线程约束：单线程主机缓冲构造，不访问真实 UART；调用方独占 frame，函数不做参数防御。
 */
static size_t make_frame(uint8_t *frame,
                         uint8_t ct,
                         uint8_t sample_count,
                         size_t sample_bytes)
{
    const size_t length = RADAR_X3PRO_HEADER_BYTES +
                          (size_t)sample_count * sample_bytes;
    memset(frame, 0, length);
    frame[0] = RADAR_X3PRO_HEADER_BYTE_0;
    frame[1] = RADAR_X3PRO_HEADER_BYTE_1;
    frame[2] = ct;
    frame[3] = sample_count;
    write_le16(&frame[4], 0xAE53U);
    write_le16(&frame[6], 0xAE53U);
    for (size_t index = 0U; index < sample_count; ++index) {
        const size_t offset = RADAR_X3PRO_HEADER_BYTES + index * sample_bytes;
        if (sample_bytes == 2U) {
            write_le16(&frame[offset], (uint16_t)(0x0400U + index * 4U));
        } else {
            frame[offset] = (uint8_t)(0x10U + index);
            write_le16(&frame[offset + 1U],
                       (uint16_t)(0x0400U + index * 4U));
        }
    }
    write_le16(&frame[8], make_checksum(frame, sample_count, sample_bytes));
    return length;
}

/**
 * @brief 将 parser 回调给出的临时完整帧复制到测试捕获结构并累计次数。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param data parser 借用帧，仅在回调期间有效；成功路径不得为 NULL。
 * @param length data 字节数，不得超过 capture->last_frame 容量。
 * @param context 指向调用方持有的 frame_capture_t，不得为 NULL。
 * @return 返回值：无（void）；上下文或长度断言失败时终止测试进程。
 * 调用方式：作为 radar_parser_feed() 的同步回调注册，返回前完成帧复制。
 * 线程约束：单线程测试专用；修改 context 对象，不保留 data，禁止并发复用同一 capture。
 */
static void capture_frame(const uint8_t *data, size_t length, void *context)
{
    frame_capture_t *capture = context;
    assert(capture != NULL);
    assert(length <= sizeof(capture->last_frame));
    ++capture->count;
    capture->last_length = length;
    memcpy(capture->last_frame, data, length);
}

/**
 * @brief 验证距离帧可跨输入块拼接、连续解析两帧并正确累计 parser 统计。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return 返回值：无（void）；回调内容、帧计数或距离/强度统计不符时 assert 终止。
 * 调用方式：由 main() 调用；先按 1 字节+余量拆帧，再整帧输入第二次。
 * 线程约束：parser/capture/帧缓冲均由 host 主线程独占；不覆盖 UART 任务并发和 FIFO 锁。
 */
static void test_split_and_combined_distance_frames(void)
{
    uint8_t frame[RADAR_PARSER_MAX_FRAME_SIZE];
    const size_t length = make_frame(frame, 0x01U, 2U, 2U);
    radar_parser_t parser;
    frame_capture_t capture = {0};
    radar_parser_stats_t stats;

    radar_parser_init(&parser);
    radar_parser_feed(&parser, frame, 1U, capture_frame, &capture);
    radar_parser_feed(&parser, &frame[1], length - 1U, capture_frame, &capture);
    radar_parser_feed(&parser, frame, length, capture_frame, &capture);

    radar_parser_get_stats(&parser, &stats);
    assert(capture.count == 2U);
    assert(capture.last_length == length);
    assert(memcmp(capture.last_frame, frame, length) == 0);
    assert(stats.valid_frame_count == 2U);
    assert(stats.valid_distance_frame_count == 2U);
    assert(stats.valid_intensity_frame_count == 0U);
    assert(stats.last_sample_bytes == RADAR_X3PRO_SAMPLE_BYTES);
    assert(stats.checksum_error_count == 0U);
}

/**
 * @brief 验证一条官方协议示例帧可被默认 parser 原样接受。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return 返回值：无（void）；回调次数、长度或字节内容不符时 assert 终止。
 * 调用方式：由 main() 调用，将只读示例数组一次性喂给新初始化 parser。
 * 线程约束：单线程纯解析测试；示例帧通过不证明具体雷达型号或实时串口数据已验证。
 */
static void test_official_protocol_example(void)
{
    const uint8_t frame[] = {
        0xAAU, 0x55U, 0x01U, 0x01U, 0x53U, 0xAEU,
        0x53U, 0xAEU, 0xABU, 0x54U, 0x00U, 0x00U,
    };
    radar_parser_t parser;
    frame_capture_t capture = {0};

    radar_parser_init(&parser);
    radar_parser_feed(&parser, frame, sizeof(frame), capture_frame, &capture);

    assert(capture.count == 1U);
    assert(capture.last_length == sizeof(frame));
    assert(memcmp(capture.last_frame, frame, sizeof(frame)) == 0);
}

/**
 * @brief 验证自动模式会等待完整 3 字节样本候选，而不会把前缀误报为坏距离帧。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return 返回值：无（void）；提前回调、错误计数或最终强度帧统计不符时 assert 终止。
 * 调用方式：由 main() 调用；先只喂可能的 2 字节布局长度，再补齐强度帧尾部。
 * 线程约束：单线程 host parser 状态测试，不涉及串口超时或任务调度。
 */
static void test_auto_detects_intensity_frame(void)
{
    uint8_t frame[RADAR_PARSER_MAX_FRAME_SIZE];
    const size_t length = make_frame(frame, 0x00U, 2U, 3U);
    radar_parser_t parser;
    frame_capture_t capture = {0};
    radar_parser_stats_t stats;

    radar_parser_init(&parser);
    const size_t distance_candidate_length =
        RADAR_X3PRO_HEADER_BYTES + (2U * RADAR_X3PRO_SAMPLE_BYTES);
    radar_parser_feed(&parser, frame, distance_candidate_length,
                      capture_frame, &capture);
    radar_parser_get_stats(&parser, &stats);
    assert(capture.count == 0U);
    assert(stats.checksum_error_count == 0U);

    radar_parser_feed(&parser, &frame[distance_candidate_length],
                      length - distance_candidate_length,
                      capture_frame, &capture);

    radar_parser_get_stats(&parser, &stats);
    assert(capture.count == 1U);
    assert(capture.last_length == length);
    assert(stats.valid_frame_count == 1U);
    assert(stats.valid_distance_frame_count == 0U);
    assert(stats.valid_intensity_frame_count == 1U);
    assert(stats.last_sample_bytes == RADAR_X3PRO_MAX_SAMPLE_BYTES);
}

/**
 * @brief 验证显式 3 字节样本模式、恢复 AUTO 以及非法样本宽度拒绝。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return 返回值：无（void）；配置返回值、回调或强度帧统计不符时 assert 终止。
 * 调用方式：由 main() 调用，在 feed 前设置固定样本宽度并在帧后检查配置接口。
 * 线程约束：单线程主机测试；不验证运行期与 UART feed 并发切换的调用方同步。
 */
static void test_explicit_sample_mode(void)
{
    uint8_t frame[RADAR_PARSER_MAX_FRAME_SIZE];
    const size_t length = make_frame(frame, 0x00U, 1U, 3U);
    radar_parser_t parser;
    frame_capture_t capture = {0};
    radar_parser_stats_t stats;

    radar_parser_init(&parser);
    assert(radar_parser_set_sample_bytes(&parser, RADAR_X3PRO_MAX_SAMPLE_BYTES));
    radar_parser_feed(&parser, frame, length, capture_frame, &capture);
    radar_parser_get_stats(&parser, &stats);

    assert(capture.count == 1U);
    assert(stats.valid_intensity_frame_count == 1U);
    assert(radar_parser_set_sample_bytes(&parser,
                                         RADAR_X3PRO_SAMPLE_BYTES_AUTO));
    assert(!radar_parser_set_sample_bytes(&parser, 1U));
}

/**
 * @brief 验证坏校验帧被计数并重同步，紧随其后的有效帧仍能交付。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return 返回值：无（void）；有效帧数、校验错误或重同步统计不符时 assert 终止。
 * 调用方式：由 main() 调用；翻转合成帧校验字节后先 feed 坏帧，再 feed 有效帧。
 * 线程约束：单线程内存流测试，不模拟 UART 丢字节、DMA 溢出或多生产者。
 */
static void test_bad_checksum_and_resync(void)
{
    uint8_t bad_frame[RADAR_PARSER_MAX_FRAME_SIZE];
    uint8_t valid_frame[RADAR_PARSER_MAX_FRAME_SIZE];
    const size_t bad_length = make_frame(bad_frame, 0x00U, 1U, 2U);
    const size_t valid_length = make_frame(valid_frame, 0x01U, 1U, 2U);
    radar_parser_t parser;
    frame_capture_t capture = {0};
    radar_parser_stats_t stats;

    bad_frame[8] ^= 0x01U;
    radar_parser_init(&parser);
    radar_parser_feed(&parser, bad_frame, bad_length, capture_frame, &capture);
    radar_parser_feed(&parser, valid_frame, valid_length, capture_frame, &capture);

    radar_parser_get_stats(&parser, &stats);
    assert(capture.count == 1U);
    assert(stats.valid_frame_count == 1U);
    assert(stats.checksum_error_count == 1U);
    assert(stats.header_resync_count >= 1U);
}

/**
 * @brief 验证噪声末尾 AA 与下一块 55 可拼接帧头，并拒绝零样本长度后继续解析。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return 返回值：无（void）；有效帧、非法帧或重同步统计不符时 assert 终止。
 * 调用方式：由 main() 调用，依次输入噪声、从 55 开始的帧余量、非法头和完整有效帧。
 * 线程约束：单线程 host 流分块测试；输入数组均在调用期间有效且不与 parser 并发修改。
 */
static void test_noise_invalid_length_and_split_header(void)
{
    const uint8_t noise[] = {0x10U, 0x20U, 0xAAU};
    const uint8_t invalid_length[] = {0xAAU, 0x55U, 0x00U, 0x00U};
    uint8_t valid_frame[RADAR_PARSER_MAX_FRAME_SIZE];
    const size_t valid_length = make_frame(valid_frame, 0x00U, 1U, 2U);
    radar_parser_t parser;
    frame_capture_t capture = {0};
    radar_parser_stats_t stats;

    radar_parser_init(&parser);
    radar_parser_feed(&parser, noise, sizeof(noise), capture_frame, &capture);
    radar_parser_feed(&parser, &valid_frame[1], valid_length - 1U,
                      capture_frame, &capture);
    radar_parser_feed(&parser, invalid_length, sizeof(invalid_length),
                      capture_frame, &capture);
    radar_parser_feed(&parser, valid_frame, valid_length, capture_frame, &capture);

    radar_parser_get_stats(&parser, &stats);
    assert(capture.count == 2U);
    assert(stats.invalid_frame_count == 1U);
    assert(stats.header_resync_count >= 1U);
}

/**
 * @brief 验证 reset_stream 丢弃半帧但保留此前有效帧累计统计。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return 返回值：无（void）；重置后回调总数或 valid_frame_count 不为 2 时 assert 终止。
 * 调用方式：由 main() 调用；完整帧、4 字节半帧、reset、完整帧依次执行。
 * 线程约束：单线程主机白盒测试；不验证 reset 与真实 UART task 并发时序。
 */
static void test_stream_reset_preserves_stats(void)
{
    uint8_t frame[RADAR_PARSER_MAX_FRAME_SIZE];
    const size_t length = make_frame(frame, 0x00U, 1U, 2U);
    radar_parser_t parser;
    frame_capture_t capture = {0};
    radar_parser_stats_t stats;

    radar_parser_init(&parser);
    radar_parser_feed(&parser, frame, length, capture_frame, &capture);
    radar_parser_feed(&parser, frame, 4U, capture_frame, &capture);
    radar_parser_reset_stream(&parser);
    radar_parser_feed(&parser, frame, length, capture_frame, &capture);

    radar_parser_get_stats(&parser, &stats);
    assert(capture.count == 2U);
    assert(stats.valid_frame_count == 2U);
}

/**
 * @brief 顺序执行雷达 parser 分片、布局、自恢复和流重置主机断言集合。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return 全部断言通过返回 0；任一 assert 失败会终止进程。
 * 调用方式：由 radar/tests/run_host_tests.sh 编译并直接运行。
 * 线程约束：单进程单线程，不访问 UART1/GPIO44 或真实雷达；仅证明当前 host 解析逻辑。
 */
int main(void)
{
    test_split_and_combined_distance_frames();
    test_official_protocol_example();
    test_auto_detects_intensity_frame();
    test_explicit_sample_mode();
    test_bad_checksum_and_resync();
    test_noise_invalid_length_and_split_header();
    test_stream_reset_preserves_stats();
    return 0;
}
