#include "s3_ydlidar_bridge/framing.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <initializer_list>
#include <memory>
#include <vector>

namespace {
class SyntheticLengthExtractor final
    : public s3_ydlidar_bridge::FrameExtractor {
 public:
  s3_ydlidar_bridge::ExtractStatus extract(
      const std::vector<uint8_t> &buffer, size_t &consumed,
      s3_ydlidar_bridge::ReceivedFrame &frame) override {
    consumed = 0;
    frame = s3_ydlidar_bridge::ReceivedFrame{};
    if (buffer.size() < 4U) {
      return s3_ydlidar_bridge::ExtractStatus::kNeedMore;
    }
    if (buffer[0] != 0xA5U || buffer[1] != 0x5AU) {
      return s3_ydlidar_bridge::ExtractStatus::kInvalid;
    }
    const size_t length = static_cast<size_t>(buffer[2]) |
                          (static_cast<size_t>(buffer[3]) << 8U);
    if (length == 0U || length > 64U) {
      return s3_ydlidar_bridge::ExtractStatus::kInvalid;
    }
    if (buffer.size() < 4U + length) {
      return s3_ydlidar_bridge::ExtractStatus::kNeedMore;
    }
    frame.payload.assign(buffer.begin() + 4,
                         buffer.begin() + static_cast<ptrdiff_t>(4U + length));
    frame.sequence = static_cast<uint64_t>(frame.payload.front());
    consumed = 4U + length;
    return s3_ydlidar_bridge::ExtractStatus::kFrameReady;
  }
};

std::vector<uint8_t> envelope(std::initializer_list<uint8_t> bytes) {
  std::vector<uint8_t> result{0xA5U, 0x5AU,
                              static_cast<uint8_t>(bytes.size()), 0U};
  result.insert(result.end(), bytes.begin(), bytes.end());
  return result;
}
}  // namespace

TEST(Framing, PreservesAllSplitAndStickyFrames) {
  auto extractor = std::make_shared<SyntheticLengthExtractor>();
  s3_ydlidar_bridge::TcpChunkAssembler assembler(extractor, 128U);
  const auto first = envelope({1U, 2U, 3U});
  const auto second = envelope({9U, 8U});

  ASSERT_FALSE(assembler.feed(first.data(), 2U));
  ASSERT_TRUE(assembler.feed(first.data() + 2U, first.size() - 2U));
  std::vector<uint8_t> sticky = first;
  sticky.insert(sticky.end(), second.begin(), second.end());
  ASSERT_TRUE(assembler.feed(sticky.data(), sticky.size()));

  const auto frames = assembler.takeAll();
  ASSERT_EQ(frames.size(), 3U);
  EXPECT_EQ(frames[0].payload, (std::vector<uint8_t>{1U, 2U, 3U}));
  EXPECT_EQ(frames[1].payload, (std::vector<uint8_t>{1U, 2U, 3U}));
  EXPECT_EQ(frames[2].payload, (std::vector<uint8_t>{9U, 8U}));
  EXPECT_EQ(assembler.bufferedBytes(), 0U);
}

TEST(Framing, RejectsIllegalLengthAndBoundsBuffer) {
  auto extractor = std::make_shared<SyntheticLengthExtractor>();
  s3_ydlidar_bridge::TcpChunkAssembler assembler(extractor, 8U);
  const std::vector<uint8_t> invalid{0xA5U, 0x5AU, 0xFFU, 0x7FU};
  EXPECT_FALSE(assembler.feed(invalid.data(), invalid.size()));
  EXPECT_GT(assembler.invalidFrames(), 0U);
  const std::vector<uint8_t> noise(32U, 0x11U);
  assembler.feed(noise.data(), noise.size());
  EXPECT_LE(assembler.bufferedBytes(), 8U);
  EXPECT_GT(assembler.droppedBytes(), 0U);
}

TEST(Framing, DetectsDuplicateAndSequenceJump) {
  s3_ydlidar_bridge::SequenceTracker tracker;
  tracker.beginConnection(1U);
  EXPECT_EQ(tracker.observe(4U), s3_ydlidar_bridge::SequenceStatus::kFirst);
  EXPECT_EQ(tracker.observe(4U), s3_ydlidar_bridge::SequenceStatus::kDuplicate);
  EXPECT_EQ(tracker.observe(6U), s3_ydlidar_bridge::SequenceStatus::kJump);
  EXPECT_EQ(tracker.observe(7U), s3_ydlidar_bridge::SequenceStatus::kInOrder);
}

TEST(Framing, TracksTypeOneAndTypeTwoInOneConnectionSequence) {
  s3_ydlidar_bridge::SequenceTracker tracker;
  tracker.beginConnection(9U);
  std::vector<s3_ydlidar_bridge::ReceivedFrame> frames(6U);
  const std::vector<uint8_t> message_types{1U, 2U, 2U, 1U, 2U, 1U};
  for (std::size_t index = 0U; index < frames.size(); ++index) {
    frames[index].message_type = message_types[index];
    frames[index].sequence = 500U + index;
  }

  EXPECT_EQ(tracker.observe(*frames.front().sequence),
            s3_ydlidar_bridge::SequenceStatus::kFirst);
  for (std::size_t index = 1U; index < frames.size(); ++index) {
    EXPECT_EQ(tracker.observe(*frames[index].sequence),
              s3_ydlidar_bridge::SequenceStatus::kInOrder)
        << "message_type=" << static_cast<unsigned>(frames[index].message_type);
  }
  const auto snapshot = tracker.snapshot();
  EXPECT_EQ(snapshot.connection_epoch, 9U);
  ASSERT_TRUE(snapshot.first_sequence.has_value());
  ASSERT_TRUE(snapshot.last_sequence.has_value());
  EXPECT_EQ(*snapshot.first_sequence, 500U);
  EXPECT_EQ(*snapshot.last_sequence, 505U);
}

TEST(Framing, DetectsSequenceWrap) {
  s3_ydlidar_bridge::SequenceTracker tracker;
  tracker.beginConnection(1U);
  EXPECT_EQ(tracker.observe(0xFFFFFFFFU),
            s3_ydlidar_bridge::SequenceStatus::kFirst);
  EXPECT_EQ(tracker.observe(0U), s3_ydlidar_bridge::SequenceStatus::kWrap);
}

TEST(Framing, AcceptsForwardGapAcrossSequenceWrap) {
  s3_ydlidar_bridge::SequenceTracker tracker;
  tracker.beginConnection(1U);
  EXPECT_EQ(tracker.observe(0xFFFFFFFEU),
            s3_ydlidar_bridge::SequenceStatus::kFirst);
  EXPECT_EQ(tracker.observe(0U), s3_ydlidar_bridge::SequenceStatus::kWrap);
  EXPECT_EQ(tracker.observe(1U), s3_ydlidar_bridge::SequenceStatus::kInOrder);

  const auto snapshot = tracker.snapshot();
  ASSERT_TRUE(snapshot.last_sequence.has_value());
  EXPECT_EQ(*snapshot.last_sequence, 1U);
}

TEST(Framing, DetectsOutOfOrderSequence) {
  s3_ydlidar_bridge::SequenceTracker tracker;
  tracker.beginConnection(1U);
  EXPECT_EQ(tracker.observe(8U), s3_ydlidar_bridge::SequenceStatus::kFirst);
  EXPECT_EQ(tracker.observe(7U),
            s3_ydlidar_bridge::SequenceStatus::kOutOfOrder);
}

TEST(Framing, ResetsSequenceStateForEachConnectionEpoch) {
  s3_ydlidar_bridge::SequenceTracker tracker;

  tracker.beginConnection(1U);
  EXPECT_EQ(tracker.observe(100U, 0U),
            s3_ydlidar_bridge::SequenceStatus::kFirst);
  EXPECT_EQ(tracker.observe(103U, 1U),
            s3_ydlidar_bridge::SequenceStatus::kJump);
  EXPECT_EQ(tracker.observe(103U, 0U),
            s3_ydlidar_bridge::SequenceStatus::kDuplicate);
  EXPECT_EQ(tracker.observe(102U, 0U),
            s3_ydlidar_bridge::SequenceStatus::kOutOfOrder);

  auto snapshot = tracker.snapshot();
  EXPECT_EQ(snapshot.connection_epoch, 1U);
  ASSERT_TRUE(snapshot.first_sequence.has_value());
  EXPECT_EQ(*snapshot.first_sequence, 100U);
  ASSERT_TRUE(snapshot.last_sequence.has_value());
  EXPECT_EQ(*snapshot.last_sequence, 103U);
  ASSERT_TRUE(snapshot.last_flags.has_value());
  EXPECT_EQ(*snapshot.last_flags, 1U);

  tracker.endConnection(1U);
  snapshot = tracker.snapshot();
  EXPECT_EQ(snapshot.connection_epoch, 1U);
  EXPECT_FALSE(snapshot.first_sequence.has_value());
  EXPECT_FALSE(snapshot.last_sequence.has_value());
  EXPECT_FALSE(snapshot.last_flags.has_value());

  tracker.beginConnection(2U);
  snapshot = tracker.snapshot();
  EXPECT_EQ(snapshot.connection_epoch, 2U);
  EXPECT_FALSE(snapshot.first_sequence.has_value());
  EXPECT_FALSE(snapshot.last_sequence.has_value());
  EXPECT_FALSE(snapshot.last_flags.has_value());
  EXPECT_EQ(tracker.observe(1U, 0U),
            s3_ydlidar_bridge::SequenceStatus::kFirst);

  snapshot = tracker.snapshot();
  ASSERT_TRUE(snapshot.first_sequence.has_value());
  EXPECT_EQ(*snapshot.first_sequence, 1U);
  ASSERT_TRUE(snapshot.last_sequence.has_value());
  EXPECT_EQ(*snapshot.last_sequence, 1U);
  ASSERT_TRUE(snapshot.last_flags.has_value());
  EXPECT_EQ(*snapshot.last_flags, 0U);
}

TEST(Framing, StopsOnInvalidResultForEmptyBuffer) {
  class EmptyInvalidExtractor final
      : public s3_ydlidar_bridge::FrameExtractor {
   public:
    s3_ydlidar_bridge::ExtractStatus extract(
        const std::vector<uint8_t> &, size_t &consumed,
        s3_ydlidar_bridge::ReceivedFrame &frame) override {
      consumed = 0;
      frame = s3_ydlidar_bridge::ReceivedFrame{};
      return s3_ydlidar_bridge::ExtractStatus::kInvalid;
    }
  };

  auto extractor = std::make_shared<EmptyInvalidExtractor>();
  s3_ydlidar_bridge::TcpChunkAssembler assembler(extractor, 8U);
  const uint8_t byte = 0x42U;
  EXPECT_FALSE(assembler.feed(&byte, 1U));
  EXPECT_EQ(assembler.bufferedBytes(), 0U);
  EXPECT_GE(assembler.invalidFrames(), 1U);
}
