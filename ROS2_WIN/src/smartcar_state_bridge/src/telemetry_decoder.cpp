#include "smartcar_state_bridge/telemetry_decoder.hpp"

#include <cmath>
#include <limits>
#include <utility>

namespace smartcar_state_bridge {

namespace {
constexpr double kMaximumWheelSpeedMmPerSecond = 100000.0;
}

bool TelemetryDecoder::finiteSample(const WheelStatusSample &sample) noexcept {
  for (const double speed : sample.speed_mm_s) {
    if (!std::isfinite(speed) ||
        std::abs(speed) > kMaximumWheelSpeedMmPerSecond) {
      return false;
    }
  }
  if (sample.source_time_s.has_value() &&
      (!std::isfinite(*sample.source_time_s) || *sample.source_time_s < 0.0)) {
    return false;
  }
  return true;
}

TelemetryDecodeStatus TelemetryDecoder::reject(TelemetryDecodeStatus status,
                                                uint64_t &counter,
                                                const char *message,
                                                std::string &error) noexcept {
  ++counter;
  error = message;
  return status;
}

TelemetryDecodeStatus TelemetryDecoder::decodeWheelStatus(
    const TelemetryEnvelope &envelope, WheelStatusSample &sample,
    std::string &error) noexcept {
  // Callers commonly reuse this output object in a callback.  Clear it before
  // every gate so a rejected envelope can never leave a previous valid wheel
  // sample visible to its caller.
  sample = WheelStatusSample{};
  error.clear();

  if (envelope.origin == TelemetryOrigin::kLiveGateway && !config_.allow_live) {
    return reject(TelemetryDecodeStatus::kDisabled, counters_.disabled,
                  "live telemetry is disabled until the wire contract is frozen",
                  error);
  }
  if (envelope.origin == TelemetryOrigin::kOfflineFixture &&
      !config_.allow_offline_fixtures) {
    return reject(TelemetryDecodeStatus::kDisabled, counters_.disabled,
                  "offline telemetry fixtures are disabled", error);
  }
  if (config_.wheel_message_type != kWheelStatusMessageType) {
    // The decoder has exactly one reviewed wheel schema.  Do not make a
    // parameter into implicit authorization for another telemetry layout.
    return reject(TelemetryDecodeStatus::kNotConfigured,
                  counters_.not_configured,
                  "wheel decoder only supports reviewed message type 0x0210",
                  error);
  }
  if (envelope.origin == TelemetryOrigin::kLiveGateway &&
      (!config_.expected_source_id.has_value() ||
       !config_.expected_destination_id.has_value())) {
    return reject(
        TelemetryDecodeStatus::kNotConfigured, counters_.not_configured,
        "live telemetry requires configured source and destination identities",
        error);
  }
  if (envelope.origin == TelemetryOrigin::kLiveGateway &&
      envelope.connection_epoch == 0U) {
    return reject(TelemetryDecodeStatus::kNotConfigured,
                  counters_.not_configured,
                  "live telemetry requires a non-zero connection epoch",
                  error);
  }
  if (envelope.payload.size() > config_.max_payload_bytes) {
    return reject(TelemetryDecodeStatus::kMalformed, counters_.malformed,
                  "telemetry payload exceeds configured maximum", error);
  }
  if (config_.expected_source_id.has_value() &&
      (!envelope.source_id.has_value() ||
       *envelope.source_id != *config_.expected_source_id)) {
    return reject(TelemetryDecodeStatus::kIdentityMismatch,
                  counters_.identity_errors, "telemetry source identity mismatch",
                  error);
  }
  if (config_.expected_destination_id.has_value() &&
      (!envelope.destination_id.has_value() ||
       *envelope.destination_id != *config_.expected_destination_id)) {
    return reject(TelemetryDecodeStatus::kIdentityMismatch,
                  counters_.identity_errors,
                  "telemetry destination identity mismatch", error);
  }
  if (envelope.message_type != kWheelStatusMessageType) {
    return reject(TelemetryDecodeStatus::kUnsupportedType,
                  counters_.unsupported_type,
                  "telemetry message type is not wheel status", error);
  }
  if (!envelope.wheel_status.has_value()) {
    // A raw payload without a reviewed SCBP parser must never be interpreted
    // as four floats merely because its length happens to match.
    return reject(TelemetryDecodeStatus::kNotConfigured,
                  counters_.not_configured,
                  "wheel status byte layout is not configured", error);
  }

  WheelStatusSample candidate = *envelope.wheel_status;
  if (config_.require_source_freshness &&
      (!candidate.sample_tick.has_value() ||
       !candidate.sample_seq.has_value())) {
    return reject(TelemetryDecodeStatus::kNotConfigured,
                  counters_.not_configured,
                  "wheel status lacks sample_tick/sample_seq freshness fields",
                  error);
  }
  if (!candidate.valid) {
    return reject(TelemetryDecodeStatus::kInvalidSample,
                  counters_.invalid_samples,
                  "wheel status source marked sample invalid", error);
  }
  if (!finiteSample(candidate)) {
    return reject(TelemetryDecodeStatus::kInvalidSample,
                  counters_.invalid_samples,
                  "wheel status contains a non-finite or excessive speed",
                  error);
  }

  // Outer metadata belongs to the gateway stream and is never substituted for
  // the source sample sequence/tick.
  candidate.outer_sequence = envelope.outer_sequence;
  candidate.inner_sequence = envelope.inner_sequence;
  candidate.connection_epoch = envelope.connection_epoch;
  candidate.host_received_steady_ns = envelope.host_received_steady_ns;
  sample = std::move(candidate);
  ++counters_.accepted;
  return TelemetryDecodeStatus::kAccepted;
}

TelemetryEnvelope makeOfflineWheelEnvelope(const WheelStatusSample &sample,
                                            uint64_t outer_sequence,
                                            uint64_t connection_epoch) {
  TelemetryEnvelope envelope;
  envelope.origin = TelemetryOrigin::kOfflineFixture;
  envelope.message_type = kWheelStatusMessageType;
  envelope.outer_sequence = outer_sequence;
  envelope.connection_epoch = connection_epoch;
  envelope.wheel_status = sample;
  return envelope;
}

}  // namespace smartcar_state_bridge
