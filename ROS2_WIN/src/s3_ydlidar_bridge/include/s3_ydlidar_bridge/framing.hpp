#pragma once

#include <cstddef>
#include <cstdint>
#include <atomic>
#include <deque>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace s3_ydlidar_bridge {

// The outer S3RD envelope is shared by radar and future telemetry streams.
// Only the raw-radar policy interprets the payload as a YDLIDAR packet.
enum class S3MessageTypePolicy : uint8_t {
  kRawYdlidar = 0,
  kOpaque = 1,
};

// Keep the payload-oriented name available to callers that do not need to
// know that the policy is selected by the outer message type.
using S3PayloadPolicy = S3MessageTypePolicy;

struct S3MessageTypeRule {
  uint8_t message_type{1};
  S3MessageTypePolicy policy{S3MessageTypePolicy::kRawYdlidar};
};

enum class S3FramePayloadKind : uint8_t {
  kRawYdlidar = 0,
  kOpaque = 1,
};

// Metadata needed by a dispatcher.  It deliberately contains no schema for
// an opaque payload; downstream code must validate/interpret it separately.
struct S3FrameMetadata {
  uint8_t version{0};
  uint8_t message_type{0};
  uint16_t flags{0};
  uint32_t device_id{0};
  uint32_t stream_id{0};
  uint32_t sequence{0};
  uint32_t timestamp_ms{0};
  uint16_t payload_length{0};
  S3FramePayloadKind payload_kind{S3FramePayloadKind::kRawYdlidar};
};

struct ReceivedFrame {
  S3FrameMetadata metadata{};
  std::vector<uint8_t> payload;
  std::optional<uint64_t> sequence;
  uint64_t connection_epoch{0};
  bool zero_packet{false};
  uint8_t version{0};
  uint8_t message_type{0};
  uint16_t flags{0};
  uint32_t device_id{0};
  uint32_t stream_id{0};
  uint32_t timestamp_ms{0};
  uint64_t received_steady_ns{0};
  uint16_t payload_length{0};
  S3FramePayloadKind payload_kind{S3FramePayloadKind::kRawYdlidar};

  bool isRawYdlidar() const noexcept {
    return payload_kind == S3FramePayloadKind::kRawYdlidar;
  }

  bool isOpaque() const noexcept {
    return payload_kind == S3FramePayloadKind::kOpaque;
  }
};

enum class ExtractStatus {
  kNeedMore,
  kFrameReady,
  kInvalid,
  kNotConfigured,
};

class FrameExtractor {
 public:
  virtual ~FrameExtractor() = default;
  virtual ExtractStatus extract(const std::vector<uint8_t> &buffer,
                                size_t &consumed,
                                ReceivedFrame &frame) = 0;
};

class UnconfiguredFrameExtractor final : public FrameExtractor {
 public:
  ExtractStatus extract(const std::vector<uint8_t> &, size_t &consumed,
                        ReceivedFrame &frame) override {
    consumed = 0;
    frame = ReceivedFrame{};
    return ExtractStatus::kNotConfigured;
  }
};

struct S3ProtocolConfig {
  uint8_t expected_version{1};
  uint8_t expected_message_type{1};
  uint16_t allowed_flags_mask{0x0001U};
  uint32_t expected_device_id{1};
  uint32_t expected_stream_id{1};
  size_t min_payload_bytes{10};
  size_t max_payload_bytes{65535};
  // Legacy/default behavior remains one exact raw YDLIDAR type.  When
  // message_type_rules is non-empty it becomes an explicit allow-list and
  // can map one type to raw and other types to opaque payloads.
  S3MessageTypePolicy message_type_policy{S3MessageTypePolicy::kRawYdlidar};
  std::vector<S3MessageTypeRule> message_type_rules;
  // Convenience allow-list for opaque types when the explicit rule table is
  // not needed.  The expected_message_type keeps its configured policy.
  std::vector<uint8_t> opaque_message_types;
  // Opaque frames have no YDLIDAR header requirement.  A caller may still set
  // an independent lower bound when its approved envelope contract requires
  // one; the common max bound always applies.
  size_t opaque_min_payload_bytes{0};
};

struct S3ProtocolCounters {
  uint64_t accepted_frames{0};
  uint64_t magic_errors{0};
  uint64_t version_errors{0};
  uint64_t type_errors{0};
  uint64_t flags_errors{0};
  uint64_t identity_errors{0};
  uint64_t length_errors{0};
  uint64_t crc_errors{0};
  uint64_t raw_frames{0};
  uint64_t opaque_frames{0};
};

class S3FrameExtractor final : public FrameExtractor {
 public:
  explicit S3FrameExtractor(S3ProtocolConfig config = {})
      : config_(config) {}

  ExtractStatus extract(const std::vector<uint8_t> &buffer,
                        size_t &consumed,
                        ReceivedFrame &frame) override;

  S3ProtocolCounters counters() const noexcept;

  // Returns the configured policy for a type, or nullopt when the type is not
  // admitted by the allow-list.  This is useful to a dispatcher and does not
  // inspect the payload.
  std::optional<S3MessageTypePolicy> policyForMessageType(
      uint8_t message_type) const noexcept;

 private:
  S3ProtocolConfig config_;
  std::atomic<uint64_t> accepted_frames_{0};
  std::atomic<uint64_t> magic_errors_{0};
  std::atomic<uint64_t> version_errors_{0};
  std::atomic<uint64_t> type_errors_{0};
  std::atomic<uint64_t> flags_errors_{0};
  std::atomic<uint64_t> identity_errors_{0};
  std::atomic<uint64_t> length_errors_{0};
  std::atomic<uint64_t> crc_errors_{0};
  std::atomic<uint64_t> raw_frames_{0};
  std::atomic<uint64_t> opaque_frames_{0};
};

struct ReadyQueueStats {
  size_t depth{0};
  size_t capacity{0};
  uint64_t dropped_ready{0};
  uint64_t overflow{0};

  // Descriptive aliases keep the statistic self-documenting at call sites.
  uint64_t dropped_ready_frames{0};
  uint64_t ready_queue_overflows{0};
};

class TcpChunkAssembler {
 public:
  // The legacy ready-frame argument is retained for configuration/API
  // compatibility; valid frames are drained in FIFO order by the transport.
  explicit TcpChunkAssembler(std::shared_ptr<FrameExtractor> extractor,
                             size_t max_buffer_bytes = 256U * 1024U,
                             size_t = 64U);

  bool feed(const uint8_t *data, size_t size);
  std::vector<ReceivedFrame> takeAll();

  size_t bufferedBytes() const noexcept { return buffer_.size(); }
  size_t droppedBytes() const noexcept { return dropped_bytes_; }
  size_t invalidFrames() const noexcept { return invalid_frames_; }
  size_t maxReadyFrames() const noexcept { return max_ready_frames_; }
  size_t readyFrames() const noexcept { return ready_.size(); }
  uint64_t droppedReady() const noexcept { return dropped_ready_; }
  uint64_t readyOverflow() const noexcept { return ready_overflow_; }
  uint64_t droppedReadyFrames() const noexcept { return dropped_ready_; }
  uint64_t readyQueueOverflows() const noexcept { return ready_overflow_; }
  ReadyQueueStats readyStats() const noexcept;
  bool protocolConfigured() const noexcept { return protocol_configured_; }

 private:
  std::shared_ptr<FrameExtractor> extractor_;
  std::vector<uint8_t> buffer_;
  std::deque<ReceivedFrame> ready_;
  size_t max_buffer_bytes_;
  size_t max_ready_frames_;
  size_t dropped_bytes_{0};
  size_t invalid_frames_{0};
  uint64_t dropped_ready_{0};
  uint64_t ready_overflow_{0};
  bool protocol_configured_{true};
};

enum class SequenceStatus {
  kFirst,
  kInOrder,
  kDuplicate,
  kOutOfOrder,
  kJump,
  kWrap,
};

struct SequenceSnapshot {
  uint64_t connection_epoch{0};
  std::optional<uint64_t> first_sequence;
  std::optional<uint64_t> last_sequence;
  std::optional<uint16_t> last_flags;
};

class SequenceTracker {
 public:
  void beginConnection(uint64_t connection_epoch) noexcept;
  void endConnection(uint64_t connection_epoch) noexcept;
  SequenceStatus observe(uint64_t sequence, uint16_t flags = 0U) noexcept;
  void reset() noexcept;
  SequenceSnapshot snapshot() const noexcept;

 private:
  bool have_last_{false};
  uint64_t last_{0};
  uint64_t connection_epoch_{0};
  std::optional<uint64_t> first_sequence_;
  std::optional<uint16_t> last_flags_;
};

}  // namespace s3_ydlidar_bridge
