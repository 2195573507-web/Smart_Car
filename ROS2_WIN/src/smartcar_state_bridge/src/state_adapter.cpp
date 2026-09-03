#include "smartcar_state_bridge/state_adapter.hpp"

#include <chrono>

namespace smartcar_state_bridge {

namespace {
uint64_t steadyNowNs() noexcept {
  const auto now = std::chrono::steady_clock::now().time_since_epoch();
  return static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(now).count());
}
}  // namespace

const char *telemetryStatusString(TelemetryDecodeStatus status) noexcept {
  switch (status) {
    case TelemetryDecodeStatus::kAccepted:
      return "accepted";
    case TelemetryDecodeStatus::kDisabled:
      return "disabled";
    case TelemetryDecodeStatus::kUnsupportedType:
      return "unsupported_type";
    case TelemetryDecodeStatus::kNotConfigured:
      return "not_configured";
    case TelemetryDecodeStatus::kMalformed:
      return "malformed";
    case TelemetryDecodeStatus::kIdentityMismatch:
      return "identity_mismatch";
    case TelemetryDecodeStatus::kInvalidSample:
      return "invalid_sample";
    case TelemetryDecodeStatus::kSequenceError:
      return "sequence_error";
  }
  return "unknown";
}

StateSubmitResult StateAdapter::submitTelemetry(
    const TelemetryEnvelope &envelope) {
  StateSubmitResult result;
  WheelStatusSample sample;
  result.decode_status = decoder_.decodeWheelStatus(envelope, sample, result.error);
  last_decode_status_ = result.decode_status;
  if (result.decode_status != TelemetryDecodeStatus::kAccepted) {
    ++counters_.rejected_samples;
    // A live stream that was explicitly authorized for odometry must stop on
    // any decoder fault.  Otherwise a malformed/missing-freshness frame could
    // be dropped while an old pose remains publishable.  Offline fixtures are
    // kept recoverable so negative tests can continue in one process.
    if (envelope.origin == TelemetryOrigin::kLiveGateway &&
        config_.enable_live_odom) {
      result.odom_update = odom_.latchExternalInvalid(
          OdomUpdateStatus::kInvalidSample,
          result.error.empty() ? "live wheel telemetry decode failed"
                                : result.error.c_str());
    } else {
      result.odom_update.status = OdomUpdateStatus::kInvalidSample;
      result.odom_update.state = odom_.state();
      result.odom_update.reason = result.error;
    }
    return result;
  }

  if (envelope.origin == TelemetryOrigin::kLiveGateway) {
    if (!config_.enable_live_odom) {
      result.decode_status = TelemetryDecodeStatus::kDisabled;
      result.error = "live odom publication is disabled by parameter";
      ++counters_.rejected_samples;
      result.odom_update.status = OdomUpdateStatus::kSessionClosed;
      result.odom_update.state = odom_.state();
      result.odom_update.reason = result.error;
      return result;
    }
    if (config_.require_outer_sequence &&
        envelope.outer_sequence_status ==
            TelemetryOuterSequenceStatus::kUnknown) {
      ++counters_.rejected_samples;
      result.decode_status = TelemetryDecodeStatus::kSequenceError;
      result.error = "live wheel sample has no outer sequence classification";
      result.odom_update = odom_.latchExternalInvalid(
          OdomUpdateStatus::kSequenceGap, result.error.c_str());
      return result;
    }
  }

  // A gap, duplicate, or reordering at the outer stream boundary means that
  // an unknown amount of wheel travel is missing.  Never integrate around it.
  OdomUpdateStatus sequence_fault = OdomUpdateStatus::kAccepted;
  const char *sequence_reason = nullptr;
  switch (envelope.outer_sequence_status) {
    case TelemetryOuterSequenceStatus::kDuplicate:
      ++counters_.outer_sequence_duplicates;
      sequence_fault = OdomUpdateStatus::kSequenceDuplicate;
      sequence_reason = "outer wheel stream sequence duplicated";
      break;
    case TelemetryOuterSequenceStatus::kOutOfOrder:
      ++counters_.outer_sequence_out_of_order;
      sequence_fault = OdomUpdateStatus::kSequenceOutOfOrder;
      sequence_reason = "outer wheel stream sequence moved backwards";
      break;
    case TelemetryOuterSequenceStatus::kJump:
      ++counters_.outer_sequence_gaps;
      sequence_fault = OdomUpdateStatus::kSequenceGap;
      sequence_reason = "outer wheel stream sequence contains a gap";
      break;
    default:
      if (envelope.outer_sequence_gap != 0U) {
        ++counters_.outer_sequence_gaps;
        sequence_fault = OdomUpdateStatus::kSequenceGap;
        sequence_reason = "outer wheel stream reports missing frames";
      }
      break;
  }
  if (sequence_reason != nullptr) {
    ++counters_.rejected_samples;
    result.decode_status = TelemetryDecodeStatus::kSequenceError;
    result.error = sequence_reason;
    result.odom_update =
        odom_.latchExternalInvalid(sequence_fault, sequence_reason);
    return result;
  }

  if (sample.host_received_steady_ns == 0U) {
    sample.host_received_steady_ns = envelope.host_received_steady_ns;
  }
  if (sample.host_received_steady_ns == 0U) {
    sample.host_received_steady_ns = steadyNowNs();
  }
  ++counters_.submitted_samples;

  // Samples are always admitted through the dedicated FIFO.  Processing one
  // item immediately keeps latency low in the normal case while preserving
  // explicit overflow/back-pressure semantics for bursty callbacks.
  if (odom_.enqueue(sample) != WheelFifoPushStatus::kAccepted) {
    ++counters_.rejected_samples;
    result.odom_update = odom_.latchExternalInvalid(
        OdomUpdateStatus::kLatchedInvalid,
        "wheel FIFO overflowed or rejected an invalid sample");
    result.error = result.odom_update.reason;
    return result;
  }
  const auto update = odom_.processNext();
  if (!update.has_value()) {
    ++counters_.rejected_samples;
    result.odom_update = odom_.latchExternalInvalid(
        OdomUpdateStatus::kLatchedInvalid,
        "wheel FIFO lost an enqueued sample");
    result.error = result.odom_update.reason;
    return result;
  }
  result.odom_update = *update;
  if (result.odom_update.status == OdomUpdateStatus::kAccepted ||
      result.odom_update.status == OdomUpdateStatus::kAnchored) {
    ++counters_.accepted_samples;
  } else {
    ++counters_.rejected_samples;
    result.error = result.odom_update.reason;
  }
  return result;
}

void StateAdapter::beginSession(uint64_t connection_epoch) noexcept {
  odom_.beginSession(connection_epoch);
}

void StateAdapter::endSession() noexcept { odom_.endSession(); }

void StateAdapter::reset() noexcept {
  // Reset is an explicit administrative request for a new local session;
  // WheelOdom itself exposes recovery only through beginSession().
  odom_.beginSession();
  counters_ = StateAdapterCounters{};
  last_decode_status_ = TelemetryDecodeStatus::kNotConfigured;
}

}  // namespace smartcar_state_bridge
