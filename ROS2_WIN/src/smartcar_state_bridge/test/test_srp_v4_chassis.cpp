#include "chassis_test_utils.hpp"

#include "smartcar_state_bridge/srp_v4_chassis.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <utility>

namespace {

using smartcar_state_bridge::SrpV4ChassisDecoder;
using smartcar_state_bridge::SrpV4Decoder;
using smartcar_state_bridge::SrpV4DecodeStatus;
using smartcar_state_bridge::SrpV4FrameDecodeStatus;
using smartcar_state_bridge::test::goldenFrame;
using smartcar_state_bridge::test::makeSrpFrame;
using smartcar_state_bridge::test::repairCrc;
using smartcar_state_bridge::test::writeF32Le;
using smartcar_state_bridge::test::writeU32Le;

constexpr double kPi = 3.14159265358979323846;

TEST(SrpV4Common, ValidatesAndExtractsInterleavedMessageHeaders) {
  SrpV4Decoder decoder;
  struct Shape {
    uint8_t message_id;
    uint8_t sequence;
    uint16_t payload_length;
  };
  for (const Shape shape : {Shape{0x10U, 9U, 30U},
                            Shape{0x14U, 10U, 16U},
                            Shape{0x15U, 11U, 24U}}) {
    const auto frame =
        makeSrpFrame(shape.message_id, shape.sequence, shape.payload_length);
    const auto decoded = decoder.decode(frame);
    ASSERT_EQ(decoded.status, SrpV4FrameDecodeStatus::kAccepted)
        << decoded.reason;
    ASSERT_TRUE(decoded.header_available);
    EXPECT_EQ(decoded.header.message_id, shape.message_id);
    EXPECT_EQ(decoded.header.sequence, shape.sequence);
    EXPECT_EQ(decoded.header.payload_length, shape.payload_length);
    EXPECT_EQ(decoded.header.priority, 2U);
    EXPECT_EQ(decoded.header.flags, 0U);
    EXPECT_EQ(decoded.expected_crc, decoded.received_crc);
  }
}

TEST(SrpV4Common, RejectsDeclaredLengthCrcAndEofBeforeTypeDispatch) {
  SrpV4Decoder decoder;
  auto frame = makeSrpFrame(0x10U, 1U, 30U);
  frame.pop_back();
  auto decoded = decoder.decode(frame);
  EXPECT_EQ(decoded.status, SrpV4FrameDecodeStatus::kLengthError);
  ASSERT_TRUE(decoded.header_available);
  EXPECT_EQ(decoded.header.message_id, 0x10U);

  frame = makeSrpFrame(0x14U, 2U, 16U);
  frame[8U] ^= 0x01U;
  EXPECT_EQ(decoder.decode(frame).status,
            SrpV4FrameDecodeStatus::kCrcError);

  frame = makeSrpFrame(0x15U, 3U, 24U);
  frame.back() = 0U;
  EXPECT_EQ(decoder.decode(frame).status,
            SrpV4FrameDecodeStatus::kEofError);
}

TEST(SrpV4Chassis, AcceptsGolden36ByteFrame) {
  const auto frame = goldenFrame();
  SrpV4ChassisDecoder decoder;
  const auto decoded = decoder.decode(frame);

  ASSERT_EQ(frame.size(), 36U);
  ASSERT_EQ(decoded.status, SrpV4DecodeStatus::kAccepted)
      << decoded.reason;
  EXPECT_EQ(decoded.expected_crc, 0xC07FU);
  EXPECT_EQ(decoded.received_crc, 0xC07FU);
  EXPECT_EQ(decoded.sample.schema, 1U);
  EXPECT_EQ(decoded.sample.status_flags, 0x04U);
  EXPECT_EQ(decoded.sample.sequence, 0x2AU);
  EXPECT_EQ(decoded.sample.timestamp_ms, 1000U);
  EXPECT_DOUBLE_EQ(decoded.sample.x_m, 1.0);
  EXPECT_DOUBLE_EQ(decoded.sample.y_m, -0.5);
  EXPECT_NEAR(decoded.sample.yaw_rad, 179.0 * kPi / 180.0, 1.0e-12);
  EXPECT_DOUBLE_EQ(decoded.sample.total_dist_m, 12.5);
}

TEST(SrpV4Chassis, AcceptsFlags0CWithSameGoldenFields) {
  auto frame = goldenFrame();
  frame[9U] = 0x0CU;
  repairCrc(frame);

  ASSERT_EQ(frame[32U], 0x44U);
  ASSERT_EQ(frame[33U], 0xD8U);
  SrpV4ChassisDecoder decoder;
  const auto decoded = decoder.decode(frame);

  ASSERT_EQ(decoded.status, SrpV4DecodeStatus::kAccepted)
      << decoded.reason;
  EXPECT_EQ(decoded.expected_crc, 0xD844U);
  EXPECT_EQ(decoded.sample.status_flags, 0x0CU);
  EXPECT_EQ(decoded.sample.sequence, 0x2AU);
  EXPECT_EQ(decoded.sample.timestamp_ms, 1000U);
  EXPECT_DOUBLE_EQ(decoded.sample.x_m, 1.0);
  EXPECT_DOUBLE_EQ(decoded.sample.y_m, -0.5);
  EXPECT_NEAR(decoded.sample.yaw_rad, 179.0 * kPi / 180.0, 1.0e-12);
  EXPECT_DOUBLE_EQ(decoded.sample.total_dist_m, 12.5);
}

TEST(SrpV4Chassis, EnforcesLowNibbleMaskAndOdometryValidSeparately) {
  SrpV4ChassisDecoder decoder;
  for (uint8_t flags = 0x00U; flags <= 0x0FU; ++flags) {
    auto frame = goldenFrame();
    frame[9U] = flags;
    repairCrc(frame);
    const auto decoded = decoder.decode(frame);
    const auto expected = (flags & 0x04U) != 0U
                              ? SrpV4DecodeStatus::kAccepted
                              : SrpV4DecodeStatus::kOdometryInvalid;
    EXPECT_EQ(decoded.status, expected)
        << "flags=" << static_cast<unsigned>(flags);
  }

  for (unsigned high_nibble = 0x10U; high_nibble <= 0xF0U;
       high_nibble += 0x10U) {
    auto frame = goldenFrame();
    frame[9U] = static_cast<uint8_t>(high_nibble | 0x04U);
    repairCrc(frame);
    EXPECT_EQ(decoder.decode(frame).status,
              SrpV4DecodeStatus::kPayloadFlagsError)
        << "flags=" << static_cast<unsigned>(frame[9U]);
  }
}

TEST(SrpV4Chassis, AcceptsProducerOwnedSequenceAndTimestamp) {
  SrpV4ChassisDecoder decoder;
  for (const auto &values :
       {std::pair<uint8_t, uint32_t>{0x00U, 0U},
        std::pair<uint8_t, uint32_t>{0xA7U, 424242U},
        std::pair<uint8_t, uint32_t>{0xFFU, 0xFFFFFFFFU}}) {
    auto frame = goldenFrame();
    frame[5U] = values.first;
    writeU32Le(frame, 12U, values.second);
    repairCrc(frame);
    const auto decoded = decoder.decode(frame);
    ASSERT_EQ(decoded.status, SrpV4DecodeStatus::kAccepted)
        << decoded.reason;
    EXPECT_EQ(decoded.sample.sequence, values.first);
    EXPECT_EQ(decoded.sample.timestamp_ms, values.second);
  }
}

TEST(SrpV4Chassis, RejectsSizeMagicLengthHeaderCrcAndEofErrors) {
  SrpV4ChassisDecoder decoder;

  auto frame = goldenFrame();
  frame.pop_back();
  EXPECT_EQ(decoder.decode(frame).status, SrpV4DecodeStatus::kSizeError);

  frame = goldenFrame();
  frame[0U] = 0xABU;
  EXPECT_EQ(decoder.decode(frame).status, SrpV4DecodeStatus::kMagicError);

  frame = goldenFrame();
  frame[2U] = 23U;
  EXPECT_EQ(decoder.decode(frame).status, SrpV4DecodeStatus::kLengthError);

  frame = goldenFrame();
  frame[7U] = 3U;
  repairCrc(frame);
  EXPECT_EQ(decoder.decode(frame).status, SrpV4DecodeStatus::kPriorityError);

  frame = goldenFrame();
  frame[6U] = 0x16U;
  repairCrc(frame);
  EXPECT_EQ(decoder.decode(frame).status, SrpV4DecodeStatus::kTypeError);

  frame = goldenFrame();
  frame[4U] = 1U;
  repairCrc(frame);
  EXPECT_EQ(decoder.decode(frame).status,
            SrpV4DecodeStatus::kHeaderFlagsError);

  frame = goldenFrame();
  frame[16U] ^= 0x01U;
  EXPECT_EQ(decoder.decode(frame).status, SrpV4DecodeStatus::kCrcError);

  frame = goldenFrame();
  frame[34U] = 0x00U;
  EXPECT_EQ(decoder.decode(frame).status, SrpV4DecodeStatus::kEofError);
}

TEST(SrpV4Chassis, RejectsSchemaReservedFlagsAndInvalidOdometry) {
  SrpV4ChassisDecoder decoder;

  auto frame = goldenFrame();
  frame[8U] = 2U;
  repairCrc(frame);
  EXPECT_EQ(decoder.decode(frame).status, SrpV4DecodeStatus::kSchemaError);

  frame = goldenFrame();
  frame[10U] = 1U;
  repairCrc(frame);
  EXPECT_EQ(decoder.decode(frame).status, SrpV4DecodeStatus::kReservedError);

  frame = goldenFrame();
  frame[9U] = 0x84U;
  repairCrc(frame);
  EXPECT_EQ(decoder.decode(frame).status,
            SrpV4DecodeStatus::kPayloadFlagsError);

  frame = goldenFrame();
  frame[9U] = 0x03U;
  repairCrc(frame);
  EXPECT_EQ(decoder.decode(frame).status,
            SrpV4DecodeStatus::kOdometryInvalid);
}

TEST(SrpV4Chassis, RejectsEveryNonFiniteFloatField) {
  SrpV4ChassisDecoder decoder;
  for (const std::size_t offset : {16U, 20U, 24U, 28U}) {
    auto frame = goldenFrame();
    writeF32Le(frame, offset, std::numeric_limits<float>::quiet_NaN());
    repairCrc(frame);
    EXPECT_EQ(decoder.decode(frame).status, SrpV4DecodeStatus::kNonFinite)
        << "offset=" << offset;

    frame = goldenFrame();
    writeF32Le(frame, offset, std::numeric_limits<float>::infinity());
    repairCrc(frame);
    EXPECT_EQ(decoder.decode(frame).status, SrpV4DecodeStatus::kNonFinite)
        << "offset=" << offset;
  }
}

}  // namespace
