#include "s3_ydlidar_bridge/framing.hpp"
#include "s3_ydlidar_bridge/official_decoder.hpp"
#include "s3_ydlidar_bridge/scan_mapper.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace {
void appendU16(std::vector<uint8_t> &bytes, uint16_t value) {
  bytes.push_back(static_cast<uint8_t>(value & 0xFFU));
  bytes.push_back(static_cast<uint8_t>(value >> 8U));
}

void appendU32(std::vector<uint8_t> &bytes, uint32_t value) {
  for (unsigned shift = 0U; shift < 32U; shift += 8U) {
    bytes.push_back(static_cast<uint8_t>(value >> shift));
  }
}

uint16_t crc16Modbus(const std::vector<uint8_t> &bytes, size_t begin,
                     size_t end) {
  uint16_t crc = 0xFFFFU;
  for (size_t index = begin; index < end; ++index) {
    crc ^= bytes[index];
    for (unsigned bit = 0U; bit < 8U; ++bit) {
      crc = (crc & 1U) != 0U ? static_cast<uint16_t>((crc >> 1U) ^ 0xA001U)
                              : static_cast<uint16_t>(crc >> 1U);
    }
  }
  return crc;
}

std::vector<uint8_t> makeS3Frame(const std::vector<uint8_t> &payload,
                                 uint32_t sequence = 1U,
                                 uint8_t version = 1U,
                                 uint8_t message_type = 1U,
                                 uint16_t flags = 0U,
                                 uint32_t device_id = 1U,
                                 uint32_t stream_id = 1U) {
  std::vector<uint8_t> frame{'S', '3', 'R', 'D', version, message_type};
  appendU16(frame, flags);
  appendU32(frame, device_id);
  appendU32(frame, stream_id);
  appendU32(frame, sequence);
  appendU32(frame, 1234U);
  appendU16(frame, static_cast<uint16_t>(payload.size()));
  frame.insert(frame.end(), payload.begin(), payload.end());
  appendU16(frame, crc16Modbus(frame, 4U, frame.size()));
  return frame;
}

std::vector<uint8_t> minimalYdlidarPayload(bool intensity, uint8_t ct = 0U) {
  if (intensity) {
    return {0xAAU, 0x55U, ct, 0x02U, 0x01U,
            0x00U, 0x81U, 0x00U, 0x00U, 0x00U,
            0x18U, 0xA0U, 0x0FU, 0x24U, 0x40U, 0x1FU};
  }
  return {0xAAU, 0x55U, ct, 0x02U, 0x01U,
          0x00U, 0x81U, 0x00U, 0x00U, 0x00U,
          0xA0U, 0x0FU, 0x40U, 0x1FU};
}

std::vector<uint8_t> trianglePayload(uint8_t ct, uint16_t start_degrees,
                                     uint16_t end_degrees,
                                     const std::vector<uint16_t> &samples) {
  const uint16_t fsa = static_cast<uint16_t>(
      (start_degrees * 64U) << 1U | 1U);
  const uint16_t lsa = static_cast<uint16_t>(
      (end_degrees * 64U) << 1U | 1U);
  uint16_t checksum = 0x55AAU;
  checksum ^= static_cast<uint16_t>(ct) |
              static_cast<uint16_t>(samples.size() << 8U);
  checksum ^= fsa;
  checksum ^= lsa;
  for (uint16_t sample : samples) {
    checksum ^= sample;
  }

  std::vector<uint8_t> payload{0xAAU, 0x55U, ct,
                               static_cast<uint8_t>(samples.size())};
  appendU16(payload, fsa);
  appendU16(payload, lsa);
  appendU16(payload, checksum);
  for (uint16_t sample : samples) {
    appendU16(payload, sample);
  }
  return payload;
}
}  // namespace

TEST(S3Protocol, AcceptsCompleteNoIntensityAndIntensityPayloads) {
  s3_ydlidar_bridge::S3FrameExtractor extractor;
  for (const bool intensity : {false, true}) {
    const auto frame = makeS3Frame(minimalYdlidarPayload(intensity),
                                   intensity ? 8U : 7U);
    size_t consumed = 0U;
    s3_ydlidar_bridge::ReceivedFrame decoded;
    ASSERT_EQ(extractor.extract(frame, consumed, decoded),
              s3_ydlidar_bridge::ExtractStatus::kFrameReady);
    EXPECT_EQ(consumed, frame.size());
    EXPECT_EQ(decoded.sequence, intensity ? 8U : 7U);
    EXPECT_EQ(decoded.payload, minimalYdlidarPayload(intensity));
  }
  EXPECT_EQ(extractor.counters().accepted_frames, 2U);
}

TEST(S3Protocol, AcceptsNormalAndZeroPositionFlagsWhenCtMatches) {
  s3_ydlidar_bridge::S3FrameExtractor extractor;
  const auto normal = makeS3Frame(minimalYdlidarPayload(false, 0U), 10U, 1U,
                                  1U, 0U);
  const auto zero = makeS3Frame(minimalYdlidarPayload(false, 1U), 11U, 1U,
                                1U, 1U);

  size_t consumed = 0U;
  s3_ydlidar_bridge::ReceivedFrame decoded;
  ASSERT_EQ(extractor.extract(normal, consumed, decoded),
            s3_ydlidar_bridge::ExtractStatus::kFrameReady);
  EXPECT_EQ(consumed, normal.size());
  EXPECT_FALSE(decoded.zero_packet);

  consumed = 0U;
  ASSERT_EQ(extractor.extract(zero, consumed, decoded),
            s3_ydlidar_bridge::ExtractStatus::kFrameReady);
  EXPECT_EQ(consumed, zero.size());
  EXPECT_TRUE(decoded.zero_packet);

  EXPECT_EQ(extractor.counters().accepted_frames, 2U);
  EXPECT_EQ(extractor.counters().flags_errors, 0U);
}

TEST(S3Protocol, RejectsFlagsWhenCtDoesNotMatch) {
  s3_ydlidar_bridge::S3FrameExtractor extractor;
  const auto flags_without_ct = makeS3Frame(
      minimalYdlidarPayload(false, 0U), 12U, 1U, 1U, 1U);
  const auto ct_without_flags = makeS3Frame(
      minimalYdlidarPayload(false, 1U), 13U, 1U, 1U, 0U);

  for (const auto *candidate : {&flags_without_ct, &ct_without_flags}) {
    size_t consumed = 0U;
    s3_ydlidar_bridge::ReceivedFrame ignored;
    EXPECT_EQ(extractor.extract(*candidate, consumed, ignored),
              s3_ydlidar_bridge::ExtractStatus::kInvalid);
  }
  EXPECT_EQ(extractor.counters().flags_errors, 2U);
}

TEST(S3Protocol, RejectsUnknownFlagsEvenWithValidCt) {
  s3_ydlidar_bridge::S3FrameExtractor extractor;
  const auto unknown = makeS3Frame(minimalYdlidarPayload(false, 0U), 14U, 1U,
                                   1U, 0x0002U);
  size_t consumed = 0U;
  s3_ydlidar_bridge::ReceivedFrame ignored;
  EXPECT_EQ(extractor.extract(unknown, consumed, ignored),
            s3_ydlidar_bridge::ExtractStatus::kInvalid);
  EXPECT_EQ(extractor.counters().flags_errors, 1U);
}

TEST(S3Protocol, RejectsPayloadShorterThanYdlidarHeader) {
  s3_ydlidar_bridge::S3ProtocolConfig config;
  config.min_payload_bytes = 0U;
  s3_ydlidar_bridge::S3FrameExtractor extractor(config);
  const auto short_payload = makeS3Frame({0xAAU, 0x55U}, 15U);

  size_t consumed = 0U;
  s3_ydlidar_bridge::ReceivedFrame ignored;
  EXPECT_EQ(extractor.extract(short_payload, consumed, ignored),
            s3_ydlidar_bridge::ExtractStatus::kInvalid);
  EXPECT_EQ(consumed, 1U);
  EXPECT_EQ(extractor.counters().length_errors, 1U);
}

TEST(S3Protocol, HandlesSplitAndStickyTcpChunksWithoutDroppingFrames) {
  auto extractor = std::make_shared<s3_ydlidar_bridge::S3FrameExtractor>();
  s3_ydlidar_bridge::TcpChunkAssembler assembler(extractor, 4096U, 8U);
  const auto first = makeS3Frame(minimalYdlidarPayload(false), 3U);
  const auto second = makeS3Frame(minimalYdlidarPayload(true), 4U);

  ASSERT_FALSE(assembler.feed(first.data(), 9U));
  std::vector<uint8_t> sticky(first.begin() + 9, first.end());
  sticky.insert(sticky.end(), second.begin(), second.end());
  ASSERT_TRUE(assembler.feed(sticky.data(), sticky.size()));
  const auto frames = assembler.takeAll();
  ASSERT_EQ(frames.size(), 2U);
  EXPECT_EQ(frames[0].sequence, 3U);
  EXPECT_EQ(frames[1].sequence, 4U);
  EXPECT_EQ(assembler.bufferedBytes(), 0U);
}

TEST(S3Protocol, EnforcesReadyQueueCapacityByDroppingOldestFrames) {
  auto extractor = std::make_shared<s3_ydlidar_bridge::S3FrameExtractor>();
  s3_ydlidar_bridge::TcpChunkAssembler assembler(extractor, 4096U, 1U);
  const auto first = makeS3Frame(minimalYdlidarPayload(false), 21U);
  const auto second = makeS3Frame(minimalYdlidarPayload(false), 22U);
  const auto third = makeS3Frame(minimalYdlidarPayload(false), 23U);
  std::vector<uint8_t> sticky = first;
  sticky.insert(sticky.end(), second.begin(), second.end());
  sticky.insert(sticky.end(), third.begin(), third.end());

  ASSERT_TRUE(assembler.feed(sticky.data(), sticky.size()));
  const auto queued_stats = assembler.readyStats();
  EXPECT_EQ(queued_stats.depth, 1U);
  EXPECT_EQ(queued_stats.capacity, 1U);
  EXPECT_EQ(queued_stats.dropped_ready, 2U);
  EXPECT_EQ(queued_stats.overflow, 2U);
  EXPECT_EQ(queued_stats.dropped_ready_frames, 2U);
  EXPECT_EQ(queued_stats.ready_queue_overflows, 2U);
  const auto frames = assembler.takeAll();
  ASSERT_EQ(frames.size(), 1U);
  EXPECT_EQ(frames[0].sequence, 23U);
  EXPECT_EQ(assembler.readyFrames(), 0U);
  EXPECT_EQ(assembler.droppedReady(), 2U);
  EXPECT_EQ(assembler.readyOverflow(), 2U);
}

TEST(S3Protocol, ZeroReadyCapacityDropsParsedFramesExplicitly) {
  auto extractor = std::make_shared<s3_ydlidar_bridge::S3FrameExtractor>();
  s3_ydlidar_bridge::TcpChunkAssembler assembler(extractor, 4096U, 0U);
  const auto candidate = makeS3Frame(minimalYdlidarPayload(false), 24U);

  ASSERT_TRUE(assembler.feed(candidate.data(), candidate.size()));
  EXPECT_EQ(assembler.readyFrames(), 0U);
  EXPECT_EQ(assembler.droppedReady(), 1U);
  EXPECT_EQ(assembler.readyOverflow(), 1U);
  const auto stats = assembler.readyStats();
  EXPECT_EQ(stats.capacity, 0U);
  EXPECT_EQ(stats.depth, 0U);
  EXPECT_EQ(stats.dropped_ready, 1U);
  EXPECT_EQ(stats.overflow, 1U);
}

TEST(S3Protocol, GoldenS3rdReplayPublishesOnlyAtTheNextZeroPacket) {
  auto extractor = std::make_shared<s3_ydlidar_bridge::S3FrameExtractor>();
  s3_ydlidar_bridge::TcpChunkAssembler assembler(extractor, 4096U, 8U);
  const auto zero_start = makeS3Frame(
      trianglePayload(0x01U, 5U, 5U, {4000U}), 100U, 1U, 1U, 0x0001U);
  const auto normal = makeS3Frame(
      trianglePayload(0x00U, 90U, 180U, {8000U, 0U}), 102U);
  const auto zero_end = makeS3Frame(
      trianglePayload(0x01U, 5U, 5U, {4000U}), 103U, 1U, 1U, 0x0001U);

  ASSERT_FALSE(assembler.feed(zero_start.data(), 7U));
  std::vector<uint8_t> sticky(zero_start.begin() + 7U, zero_start.end());
  sticky.insert(sticky.end(), normal.begin(), normal.end());
  sticky.insert(sticky.end(), zero_end.begin(), zero_end.end());
  ASSERT_TRUE(assembler.feed(sticky.data(), sticky.size()));
  auto frames = assembler.takeAll();
  ASSERT_EQ(frames.size(), 3U);
  for (size_t index = 0U; index < frames.size(); ++index) {
    frames[index].received_steady_ns = (index + 1U) * 100000000U;
  }

  s3_ydlidar_bridge::OfficialDecoder decoder;
  s3_ydlidar_bridge::ScanMapper mapper(
      s3_ydlidar_bridge::ScanMapperConfig{});
  for (size_t index = 0U; index < frames.size(); ++index) {
    std::vector<::node_info> nodes;
    std::string error;
    ASSERT_TRUE(decoder.decode(frames[index].payload.data(),
                               frames[index].payload.size(), nodes, error))
        << error;
    const auto result = mapper.accumulate(
        nodes, frames[index].zero_packet, index == 1U,
        frames[index].received_steady_ns,
        rclcpp::Time(static_cast<int64_t>(index + 1U) * 100));
    if (index < 2U) {
      EXPECT_TRUE(result.completed_scans.empty());
      continue;
    }
    ASSERT_EQ(result.completed_scans.size(), 1U);
    ASSERT_TRUE(result.revolution_diagnostics.has_value());
    EXPECT_EQ(result.revolution_diagnostics->frame_count, 2U);
    EXPECT_EQ(result.revolution_diagnostics->sequence_gaps, 1U);
    EXPECT_FLOAT_EQ(result.completed_scans.front().scan_time, 0.2F);
    EXPECT_EQ(result.completed_scans.front().header.frame_id, "laser_frame");
    EXPECT_TRUE(std::isinf(result.completed_scans.front().ranges[0U]));
  }
}

TEST(S3Protocol, CountsCrcVersionLengthAndIdentityErrors) {
  s3_ydlidar_bridge::S3FrameExtractor extractor;
  const auto valid = makeS3Frame(minimalYdlidarPayload(false));

  auto bad_crc = valid;
  bad_crc.back() ^= 0x01U;
  auto bad_version = makeS3Frame(minimalYdlidarPayload(false), 2U, 2U);
  auto bad_identity = makeS3Frame(minimalYdlidarPayload(false), 3U, 1U, 1U,
                                   0U, 9U, 8U);
  auto bad_length = valid;
  bad_length[24] = 1U;
  bad_length[25] = 0U;

  for (const auto *candidate : {&bad_crc, &bad_version, &bad_identity,
                                &bad_length}) {
    size_t consumed = 0U;
    s3_ydlidar_bridge::ReceivedFrame ignored;
    EXPECT_EQ(extractor.extract(*candidate, consumed, ignored),
              s3_ydlidar_bridge::ExtractStatus::kInvalid);
    EXPECT_GT(consumed, 0U);
  }
  const auto counters = extractor.counters();
  EXPECT_EQ(counters.crc_errors, 1U);
  EXPECT_EQ(counters.version_errors, 1U);
  EXPECT_EQ(counters.identity_errors, 1U);
  EXPECT_EQ(counters.length_errors, 1U);
}

TEST(S3Protocol, RejectsWrongMessageTypeAndFlags) {
  s3_ydlidar_bridge::S3FrameExtractor extractor;
  const auto bad_type = makeS3Frame(minimalYdlidarPayload(false), 1U, 1U, 2U);
  const auto bad_flags = makeS3Frame(minimalYdlidarPayload(false), 2U, 1U,
                                     1U, 4U);
  for (const auto *candidate : {&bad_type, &bad_flags}) {
    size_t consumed = 0U;
    s3_ydlidar_bridge::ReceivedFrame ignored;
    EXPECT_EQ(extractor.extract(*candidate, consumed, ignored),
              s3_ydlidar_bridge::ExtractStatus::kInvalid);
  }
  EXPECT_EQ(extractor.counters().type_errors, 1U);
  EXPECT_EQ(extractor.counters().flags_errors, 1U);
}

TEST(S3Protocol, AcceptsOpaqueMessageTypeWithoutYdlidarPayloadValidation) {
  s3_ydlidar_bridge::S3ProtocolConfig config;
  config.message_type_rules = {
      {1U, s3_ydlidar_bridge::S3MessageTypePolicy::kRawYdlidar},
      {7U, s3_ydlidar_bridge::S3MessageTypePolicy::kOpaque},
  };
  config.expected_stream_id = 9U;
  config.opaque_min_payload_bytes = 0U;
  s3_ydlidar_bridge::S3FrameExtractor extractor(config);

  // A one-byte payload and the zero-position flag are intentionally invalid
  // for a raw YDLIDAR packet, but valid for an opaque, dispatcher-owned type.
  const auto opaque = makeS3Frame({0x42U}, 44U, 1U, 7U, 0x0001U, 1U, 9U);
  size_t consumed = 0U;
  s3_ydlidar_bridge::ReceivedFrame decoded;
  ASSERT_EQ(extractor.extract(opaque, consumed, decoded),
            s3_ydlidar_bridge::ExtractStatus::kFrameReady);
  EXPECT_EQ(consumed, opaque.size());
  EXPECT_TRUE(decoded.isOpaque());
  EXPECT_FALSE(decoded.isRawYdlidar());
  EXPECT_FALSE(decoded.zero_packet);
  EXPECT_EQ(decoded.payload, std::vector<uint8_t>({0x42U}));
  EXPECT_EQ(decoded.payload_length, 1U);
  EXPECT_EQ(decoded.message_type, 7U);
  EXPECT_EQ(decoded.flags, 0x0001U);
  EXPECT_EQ(decoded.device_id, 1U);
  EXPECT_EQ(decoded.stream_id, 9U);
  EXPECT_EQ(decoded.sequence, 44U);
  EXPECT_EQ(decoded.timestamp_ms, 1234U);
  EXPECT_EQ(decoded.metadata.message_type, 7U);
  EXPECT_EQ(decoded.metadata.payload_kind,
            s3_ydlidar_bridge::S3FramePayloadKind::kOpaque);
  EXPECT_EQ(decoded.metadata.payload_length, 1U);
  EXPECT_EQ(extractor.counters().accepted_frames, 1U);
  EXPECT_EQ(extractor.counters().raw_frames, 0U);
  EXPECT_EQ(extractor.counters().opaque_frames, 1U);
}

TEST(S3Protocol, RoutesTypeTwoSrpChassisFrameAsOpaqueOnly) {
  const std::vector<uint8_t> srp_chassis{
      0xAAU, 0x55U, 0x18U, 0x00U, 0x00U, 0x2AU, 0x15U, 0x02U, 0x01U,
      0x04U, 0x00U, 0x00U, 0xE8U, 0x03U, 0x00U, 0x00U, 0x00U, 0x00U,
      0x7AU, 0x44U, 0x00U, 0x00U, 0xFAU, 0xC3U, 0x00U, 0x00U, 0x33U,
      0x43U, 0x00U, 0x00U, 0x48U, 0x41U, 0x7FU, 0xC0U, 0x0DU, 0x0AU,
  };
  s3_ydlidar_bridge::S3ProtocolConfig config;
  config.opaque_message_types = {2U};
  s3_ydlidar_bridge::S3FrameExtractor extractor(config);
  const auto outer = makeS3Frame(srp_chassis, 46U, 1U, 2U);

  size_t consumed = 0U;
  s3_ydlidar_bridge::ReceivedFrame decoded;
  ASSERT_EQ(extractor.extract(outer, consumed, decoded),
            s3_ydlidar_bridge::ExtractStatus::kFrameReady);
  EXPECT_EQ(consumed, outer.size());
  EXPECT_EQ(decoded.message_type, 2U);
  EXPECT_TRUE(decoded.isOpaque());
  EXPECT_FALSE(decoded.isRawYdlidar());
  EXPECT_FALSE(decoded.zero_packet);
  EXPECT_EQ(decoded.payload, srp_chassis);
  EXPECT_EQ(extractor.counters().raw_frames, 0U);
  EXPECT_EQ(extractor.counters().opaque_frames, 1U);
}

TEST(S3Protocol, OpaqueConvenienceAllowListAcceptsShortPayload) {
  s3_ydlidar_bridge::S3ProtocolConfig config;
  config.opaque_message_types = {7U};
  s3_ydlidar_bridge::S3FrameExtractor extractor(config);
  const auto opaque = makeS3Frame({}, 45U, 1U, 7U);

  size_t consumed = 0U;
  s3_ydlidar_bridge::ReceivedFrame decoded;
  ASSERT_EQ(extractor.extract(opaque, consumed, decoded),
            s3_ydlidar_bridge::ExtractStatus::kFrameReady);
  EXPECT_EQ(consumed, opaque.size());
  EXPECT_TRUE(decoded.isOpaque());
  EXPECT_TRUE(decoded.payload.empty());
  EXPECT_EQ(extractor.counters().opaque_frames, 1U);
}
