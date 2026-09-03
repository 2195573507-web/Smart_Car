#pragma once

#include "smartcar_state_bridge/telemetry_decoder.hpp"
#include "smartcar_state_bridge/wheel_odom.hpp"

#include <cstdint>
#include <string>
#include <utility>

namespace smartcar_state_bridge {

struct StateAdapterConfig {
  TelemetryDecoderConfig decoder{};
  WheelOdomConfig odom{};
  // This is an independent safety gate from decoder.allow_live.  Both must
  // be enabled before a live sample can advance odometry.
  bool enable_live_odom{false};
  bool require_outer_sequence{true};
};

struct StateSubmitResult {
  TelemetryDecodeStatus decode_status{TelemetryDecodeStatus::kNotConfigured};
  OdomUpdate odom_update{};
  std::string error;
};

struct StateAdapterCounters {
  uint64_t submitted_samples{0U};
  uint64_t accepted_samples{0U};
  uint64_t rejected_samples{0U};
  uint64_t outer_sequence_gaps{0U};
  uint64_t outer_sequence_duplicates{0U};
  uint64_t outer_sequence_out_of_order{0U};
};

// Transport-independent state adapter.  The gateway owns the socket and
// passes only a parsed TelemetryEnvelope here.  Keeping this class free of ROS
// publishers lets the same gate be embedded in the single gateway or tested
// deterministically with offline fixtures.
class StateAdapter final {
 public:
  explicit StateAdapter(StateAdapterConfig config = {})
      : config_(std::move(config)), decoder_(config_.decoder), odom_(config_.odom) {}

  StateSubmitResult submitTelemetry(const TelemetryEnvelope &envelope);

  void beginSession(uint64_t connection_epoch = 0U) noexcept;
  void endSession() noexcept;
  // Starts a fresh adapter session and clears accumulated counters.  This is
  // an explicit administrative action, not an automatic recovery path.
  void reset() noexcept;

  const StateAdapterConfig &config() const noexcept { return config_; }
  const TelemetryDecoder &decoder() const noexcept { return decoder_; }
  const WheelOdom &odom() const noexcept { return odom_; }
  WheelOdom &odom() noexcept { return odom_; }
  const StateAdapterCounters &counters() const noexcept { return counters_; }
  TelemetryDecodeStatus lastDecodeStatus() const noexcept {
    return last_decode_status_;
  }

 private:
  StateAdapterConfig config_{};
  TelemetryDecoder decoder_{};
  WheelOdom odom_{};
  StateAdapterCounters counters_{};
  TelemetryDecodeStatus last_decode_status_{TelemetryDecodeStatus::kNotConfigured};
};

const char *telemetryStatusString(TelemetryDecodeStatus status) noexcept;

}  // namespace smartcar_state_bridge
