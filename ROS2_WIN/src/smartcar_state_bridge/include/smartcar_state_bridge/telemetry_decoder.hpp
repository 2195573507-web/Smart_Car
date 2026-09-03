#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace smartcar_state_bridge {

// These identifiers are protocol names, not a locally invented wire format.
// The byte-level SCBP parser is intentionally injected by the gateway once its
// contract is frozen.
inline constexpr uint16_t kWheelStatusMessageType = 0x0210U;
inline constexpr uint16_t kDualAhrsMessageType = 0x0201U;
inline constexpr uint16_t kImuMessageType = 0x0207U;
inline constexpr std::size_t kScbpMaximumFrameBytes = 267U;

enum class TelemetryOrigin {
  kLiveGateway,
  kOfflineFixture,
};

// Classification produced by the gateway's outer per-stream sequence
// tracker.  The state adapter consumes this metadata but deliberately does
// not depend on the gateway package, which keeps the library reusable in
// offline tests and future composable deployments.
enum class TelemetryOuterSequenceStatus : uint8_t {
  kUnknown,
  kFirst,
  kInOrder,
  kDuplicate,
  kOutOfOrder,
  kJump,
  kWrap,
};

// The order is part of the P1 contract and must not be changed to the more
// common FL/FR/RL/RR order: RR, RF, LR, LF.
struct WheelStatusSample {
  std::array<double, 4U> speed_mm_s{};
  std::optional<uint32_t> sample_tick;
  std::optional<uint32_t> sample_seq;
  std::optional<double> source_time_s;
  std::optional<uint32_t> source_age_ms;
  std::optional<uint32_t> source_epoch;
  bool valid{false};

  // Metadata copied from the outer envelope by TelemetryDecoder.  It is kept
  // separate from sample_seq/tick because the two sequence domains have
  // different owners and semantics.
  uint64_t outer_sequence{0U};
  uint8_t inner_sequence{0U};
  uint64_t connection_epoch{0U};
  uint64_t host_received_steady_ns{0U};
};

struct TelemetryEnvelope {
  TelemetryOrigin origin{TelemetryOrigin::kOfflineFixture};
  // Inner protocol type (for example the reviewed SCBP 0x0210 wheel type).
  // This is intentionally separate from the uint8 outer S3RD type below.
  uint16_t message_type{0U};
  std::optional<uint8_t> outer_message_type;
  std::optional<uint16_t> source_id;
  std::optional<uint16_t> destination_id;
  uint16_t flags{0U};
  uint8_t inner_sequence{0U};
  uint64_t outer_sequence{0U};
  TelemetryOuterSequenceStatus outer_sequence_status{
      TelemetryOuterSequenceStatus::kUnknown};
  uint64_t outer_sequence_gap{0U};
  uint64_t connection_epoch{0U};
  uint64_t host_received_steady_ns{0U};
  std::vector<uint8_t> payload;

  // This field is populated only by an approved SCBP decoder or a deterministic
  // offline fixture.  TelemetryDecoder never guesses a layout from payload.
  std::optional<WheelStatusSample> wheel_status;
};

struct TelemetryDecoderConfig {
  // Live input is a deliberate opt-in.  The default remains disabled until a
  // reviewed S3/SCBP contract and captures exist.
  bool allow_live{false};
  bool allow_offline_fixtures{true};
  bool require_source_freshness{true};
  std::size_t max_payload_bytes{kScbpMaximumFrameBytes};
  std::optional<uint16_t> expected_source_id;
  std::optional<uint16_t> expected_destination_id;
  // Retained for parameter compatibility.  TelemetryDecoder accepts only
  // kWheelStatusMessageType, so another value is an invalid configuration.
  uint16_t wheel_message_type{kWheelStatusMessageType};
};

enum class TelemetryDecodeStatus {
  kAccepted,
  kDisabled,
  kUnsupportedType,
  kNotConfigured,
  kMalformed,
  kIdentityMismatch,
  kInvalidSample,
  kSequenceError,
};

struct TelemetryDecoderCounters {
  uint64_t accepted{0U};
  uint64_t disabled{0U};
  uint64_t unsupported_type{0U};
  uint64_t not_configured{0U};
  uint64_t malformed{0U};
  uint64_t identity_errors{0U};
  uint64_t invalid_samples{0U};
  uint64_t sequence_errors{0U};
};

// Decoder for the structured result of the gateway's SCBP parser.  Keeping
// this boundary explicit lets the host package compile and test without
// copying or inventing Common/SCBP_CAN definitions.
class TelemetryDecoder final {
 public:
  explicit TelemetryDecoder(TelemetryDecoderConfig config = {})
      : config_(std::move(config)) {}

  TelemetryDecodeStatus decodeWheelStatus(const TelemetryEnvelope &envelope,
                                          WheelStatusSample &sample,
                                          std::string &error) noexcept;

  TelemetryDecoderCounters counters() const noexcept { return counters_; }
  const TelemetryDecoderConfig &config() const noexcept { return config_; }

 private:
  static bool finiteSample(const WheelStatusSample &sample) noexcept;
  TelemetryDecodeStatus reject(TelemetryDecodeStatus status,
                               uint64_t &counter,
                               const char *message,
                               std::string &error) noexcept;

  TelemetryDecoderConfig config_;
  TelemetryDecoderCounters counters_{};
};

// Convenience factory for tests and replay tools.  It deliberately labels the
// resulting envelope as an offline fixture; callers cannot accidentally turn
// it into a live network frame by reusing the structure.
TelemetryEnvelope makeOfflineWheelEnvelope(const WheelStatusSample &sample,
                                            uint64_t outer_sequence = 0U,
                                            uint64_t connection_epoch = 0U);

}  // namespace smartcar_state_bridge
