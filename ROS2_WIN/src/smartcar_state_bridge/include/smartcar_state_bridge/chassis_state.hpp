#pragma once

#include "smartcar_state_bridge/srp_v4_chassis.hpp"
#include "smartcar_state_bridge/telemetry_decoder.hpp"

#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace smartcar_state_bridge {

struct ChassisOdomConfig {
  double min_dt_s{1.0e-6};
  double max_dt_s{1.0};
  uint32_t stale_timeout_ms{500U};
};

enum class ChassisOdomStatus {
  kNoSample,
  kAnchored,
  kAccepted,
  kSessionClosed,
  kInvalidConfig,
  kInvalidSample,
  kEpochChanged,
  kSequenceDuplicate,
  kSequenceRollback,
  kTimestampDuplicate,
  kTimestampRollback,
  kDtInvalid,
  kHostTimeBackward,
  kStale,
};

struct ChassisOdomState {
  double x_m{0.0};
  double y_m{0.0};
  double yaw_rad{0.0};
  double total_dist_m{0.0};
  double linear_x_mps{0.0};
  double linear_y_mps{0.0};
  double angular_z_rps{0.0};
  uint64_t accepted_updates{0U};
  bool session_active{false};
  bool baseline_ready{false};
  bool valid{false};
  std::optional<uint64_t> connection_epoch;
  std::optional<uint8_t> last_inner_sequence;
  std::optional<uint32_t> last_timestamp_ms;
  uint64_t last_host_received_steady_ns{0U};
};

struct ChassisOdomUpdate {
  ChassisOdomStatus status{ChassisOdomStatus::kNoSample};
  ChassisOdomState state{};
  double dt_s{0.0};
  std::string reason;
};

class ChassisOdomTracker final {
 public:
  explicit ChassisOdomTracker(ChassisOdomConfig config = {});

  void beginSession(uint64_t connection_epoch) noexcept;
  void endSession() noexcept;

  ChassisOdomUpdate update(const SrpV4ChassisSample &sample,
                           uint64_t connection_epoch,
                           uint64_t host_received_steady_ns) noexcept;
  ChassisOdomUpdate invalidate(ChassisOdomStatus status,
                               const char *reason) noexcept;
  ChassisOdomUpdate checkStale(uint64_t host_now_steady_ns) noexcept;

  const ChassisOdomConfig &config() const noexcept { return config_; }
  const ChassisOdomState &state() const noexcept { return state_; }
  ChassisOdomStatus lastStatus() const noexcept { return last_status_; }

 private:
  void clearBaseline() noexcept;
  void anchor(const SrpV4ChassisSample &sample,
              uint64_t host_received_steady_ns) noexcept;
  ChassisOdomUpdate result(ChassisOdomStatus status,
                           const char *reason,
                           double dt_s = 0.0) noexcept;

  ChassisOdomConfig config_{};
  ChassisOdomState state_{};
  std::optional<SrpV4ChassisSample> previous_;
  bool config_valid_{true};
  ChassisOdomStatus last_status_{ChassisOdomStatus::kNoSample};
};

struct ChassisTelemetryFrame {
  TelemetryOrigin origin{TelemetryOrigin::kOfflineFixture};
  std::vector<uint8_t> payload;
  TelemetryOuterSequenceStatus outer_sequence_status{
      TelemetryOuterSequenceStatus::kUnknown};
  uint64_t outer_sequence_gap{0U};
  uint64_t connection_epoch{0U};
  uint64_t host_received_steady_ns{0U};
};

struct ChassisStateAdapterConfig {
  bool allow_live_telemetry{false};
  bool enable_live_odom{false};
  bool allow_offline_fixtures{true};
  bool require_outer_sequence{true};
  ChassisOdomConfig odom{};
};

enum class ChassisSubmitStatus {
  kDisabled,
  kDecodeRejected,
  kSequenceRejected,
  kOdomRejected,
  kAnchored,
  kAccepted,
};

struct ChassisStateAdapterCounters {
  uint64_t disabled{0U};
  uint64_t decode_rejected{0U};
  uint64_t sequence_rejected{0U};
  uint64_t odom_rejected{0U};
  uint64_t anchored{0U};
  uint64_t accepted{0U};
};

struct ChassisSubmitResult {
  ChassisSubmitStatus status{ChassisSubmitStatus::kDisabled};
  SrpV4DecodeStatus decode_status{SrpV4DecodeStatus::kNotAttempted};
  ChassisOdomUpdate odom_update{};
  std::string error;
};

class ChassisStateAdapter final {
 public:
  explicit ChassisStateAdapter(ChassisStateAdapterConfig config = {})
      : config_(std::move(config)), odom_(config_.odom) {}

  void beginSession(uint64_t connection_epoch) noexcept;
  void endSession() noexcept;
  ChassisSubmitResult submit(const ChassisTelemetryFrame &frame);
  ChassisSubmitResult submitDecoded(
      const ChassisTelemetryFrame &frame,
      const SrpV4FrameDecodeResult &decoded_frame);
  ChassisOdomUpdate invalidate(ChassisOdomStatus status,
                               const char *reason) noexcept;
  ChassisOdomUpdate checkStale(uint64_t host_now_steady_ns) noexcept;

  const ChassisStateAdapterConfig &config() const noexcept { return config_; }
  const ChassisOdomTracker &odom() const noexcept { return odom_; }
  const ChassisStateAdapterCounters &counters() const noexcept {
    return counters_;
  }
  SrpV4DecodeStatus lastDecodeStatus() const noexcept {
    return last_decode_status_;
  }

 private:
  ChassisSubmitResult rejectSequence(const char *reason);

  ChassisStateAdapterConfig config_{};
  SrpV4ChassisDecoder decoder_{};
  ChassisOdomTracker odom_{};
  ChassisStateAdapterCounters counters_{};
  SrpV4DecodeStatus last_decode_status_{SrpV4DecodeStatus::kNotAttempted};
};

enum class SrpV4TelemetrySubmitStatus {
  kFrameDecodeRejected,
  kOuterSequenceRejected,
  kIgnored,
  kChassis,
};

struct SrpV4TelemetryCounters {
  uint64_t received{0U};
  uint64_t frame_decode_rejected{0U};
  uint64_t outer_sequence_rejected{0U};
  uint64_t imu_frames{0U};
  uint64_t wheel_frames{0U};
  uint64_t chassis_frames{0U};
  uint64_t other_frames{0U};
};

struct SrpV4TelemetrySubmitResult {
  SrpV4TelemetrySubmitStatus status{
      SrpV4TelemetrySubmitStatus::kFrameDecodeRejected};
  SrpV4FrameDecodeStatus frame_decode_status{
      SrpV4FrameDecodeStatus::kNotAttempted};
  std::optional<uint8_t> message_id;
  ChassisSubmitResult chassis_result{};
  std::string error;
};

// Validates every S3RD type-2 payload with the common SRP decoder, then routes
// only message 0x15 into the chassis-specific decoder and odometry admission.
class SrpV4TelemetryAdapter final {
 public:
  explicit SrpV4TelemetryAdapter(ChassisStateAdapterConfig config = {})
      : config_(std::move(config)), chassis_(config_) {}

  void beginSession(uint64_t connection_epoch) noexcept;
  void endSession() noexcept;
  SrpV4TelemetrySubmitResult submit(const ChassisTelemetryFrame &frame);
  ChassisOdomUpdate invalidateChassis(ChassisOdomStatus status,
                                      const char *reason) noexcept;
  ChassisOdomUpdate checkStale(uint64_t host_now_steady_ns) noexcept;

  const ChassisStateAdapterConfig &config() const noexcept { return config_; }
  const ChassisStateAdapter &chassis() const noexcept { return chassis_; }
  const SrpV4TelemetryCounters &counters() const noexcept { return counters_; }
  SrpV4FrameDecodeStatus lastFrameDecodeStatus() const noexcept {
    return last_frame_decode_status_;
  }

 private:
  bool rejectOuterSequence(const ChassisTelemetryFrame &frame,
                           std::string &reason) const;

  ChassisStateAdapterConfig config_{};
  SrpV4Decoder decoder_{};
  ChassisStateAdapter chassis_{};
  SrpV4TelemetryCounters counters_{};
  SrpV4FrameDecodeStatus last_frame_decode_status_{
      SrpV4FrameDecodeStatus::kNotAttempted};
};

const char *toString(ChassisOdomStatus status) noexcept;
const char *toString(ChassisSubmitStatus status) noexcept;

}  // namespace smartcar_state_bridge
