#include "smartcar_state_bridge/chassis_state.hpp"

#include <cmath>

namespace smartcar_state_bridge {

namespace {

constexpr double kTwoPi = 6.28318530717958647692;

bool finiteSample(const SrpV4ChassisSample &sample) noexcept {
  return std::isfinite(sample.x_m) && std::isfinite(sample.y_m) &&
         std::isfinite(sample.yaw_rad) &&
         std::isfinite(sample.total_dist_m);
}

}  // namespace

ChassisOdomTracker::ChassisOdomTracker(ChassisOdomConfig config)
    : config_(std::move(config)) {
  config_valid_ = std::isfinite(config_.min_dt_s) &&
                  std::isfinite(config_.max_dt_s) &&
                  config_.min_dt_s > 0.0 &&
                  config_.max_dt_s >= config_.min_dt_s &&
                  config_.stale_timeout_ms > 0U;
}

void ChassisOdomTracker::clearBaseline() noexcept {
  previous_.reset();
  state_.baseline_ready = false;
  state_.valid = false;
  state_.linear_x_mps = 0.0;
  state_.linear_y_mps = 0.0;
  state_.angular_z_rps = 0.0;
  state_.last_inner_sequence.reset();
  state_.last_timestamp_ms.reset();
  state_.last_host_received_steady_ns = 0U;
}

void ChassisOdomTracker::beginSession(uint64_t connection_epoch) noexcept {
  state_ = ChassisOdomState{};
  state_.session_active = true;
  state_.connection_epoch = connection_epoch;
  previous_.reset();
  last_status_ = ChassisOdomStatus::kNoSample;
}

void ChassisOdomTracker::endSession() noexcept {
  clearBaseline();
  state_.session_active = false;
  last_status_ = ChassisOdomStatus::kSessionClosed;
}

void ChassisOdomTracker::anchor(const SrpV4ChassisSample &sample,
                                uint64_t host_received_steady_ns) noexcept {
  previous_ = sample;
  state_.x_m = sample.x_m;
  state_.y_m = sample.y_m;
  state_.yaw_rad = sample.yaw_rad;
  state_.total_dist_m = sample.total_dist_m;
  state_.linear_x_mps = 0.0;
  state_.linear_y_mps = 0.0;
  state_.angular_z_rps = 0.0;
  state_.baseline_ready = true;
  state_.valid = true;
  state_.last_inner_sequence = sample.sequence;
  state_.last_timestamp_ms = sample.timestamp_ms;
  state_.last_host_received_steady_ns = host_received_steady_ns;
}

ChassisOdomUpdate ChassisOdomTracker::result(ChassisOdomStatus status,
                                             const char *reason,
                                             double dt_s) noexcept {
  last_status_ = status;
  ChassisOdomUpdate update;
  update.status = status;
  update.state = state_;
  update.dt_s = dt_s;
  if (reason != nullptr) {
    update.reason = reason;
  }
  return update;
}

ChassisOdomUpdate ChassisOdomTracker::invalidate(
    ChassisOdomStatus status, const char *reason) noexcept {
  clearBaseline();
  return result(status, reason);
}

ChassisOdomUpdate ChassisOdomTracker::update(
    const SrpV4ChassisSample &sample, uint64_t connection_epoch,
    uint64_t host_received_steady_ns) noexcept {
  if (!config_valid_) {
    return invalidate(ChassisOdomStatus::kInvalidConfig,
                      "chassis odom timing configuration is invalid");
  }
  if (!state_.session_active || !state_.connection_epoch.has_value()) {
    return result(ChassisOdomStatus::kSessionClosed,
                  "chassis odom session is closed");
  }
  if (*state_.connection_epoch != connection_epoch) {
    return invalidate(ChassisOdomStatus::kEpochChanged,
                      "chassis connection epoch changed without beginSession");
  }
  if (!finiteSample(sample)) {
    return invalidate(ChassisOdomStatus::kInvalidSample,
                      "chassis pose contains NaN or infinity");
  }
  if (state_.last_host_received_steady_ns != 0U &&
      host_received_steady_ns < state_.last_host_received_steady_ns) {
    return invalidate(ChassisOdomStatus::kHostTimeBackward,
                      "host receive time moved backwards");
  }
  if (state_.last_inner_sequence.has_value()) {
    const uint8_t forward_delta = static_cast<uint8_t>(
        sample.sequence - *state_.last_inner_sequence);
    if (forward_delta == 0U) {
      return invalidate(ChassisOdomStatus::kSequenceDuplicate,
                        "chassis inner sequence is duplicated");
    }
    if (forward_delta >= 0x80U) {
      return invalidate(ChassisOdomStatus::kSequenceRollback,
                        "chassis inner sequence moved backwards");
    }
  }
  if (state_.last_timestamp_ms.has_value()) {
    if (sample.timestamp_ms == *state_.last_timestamp_ms) {
      return invalidate(ChassisOdomStatus::kTimestampDuplicate,
                        "chassis timestamp is duplicated");
    }
    if (sample.timestamp_ms < *state_.last_timestamp_ms) {
      return invalidate(ChassisOdomStatus::kTimestampRollback,
                        "chassis timestamp moved backwards");
    }
  }
  if (previous_.has_value() && state_.last_host_received_steady_ns != 0U &&
      host_received_steady_ns - state_.last_host_received_steady_ns >
          static_cast<uint64_t>(config_.stale_timeout_ms) * 1000000U) {
    return invalidate(ChassisOdomStatus::kStale,
                      "chassis host receive interval exceeded stale timeout");
  }
  if (!previous_.has_value()) {
    anchor(sample, host_received_steady_ns);
    return result(ChassisOdomStatus::kAnchored,
                  "first valid chassis pose establishes the baseline");
  }

  const uint32_t previous_timestamp = previous_->timestamp_ms;
  const uint32_t delta_ms = sample.timestamp_ms - previous_timestamp;
  const double dt_s = static_cast<double>(delta_ms) / 1000.0;
  if (delta_ms > config_.stale_timeout_ms) {
    return invalidate(ChassisOdomStatus::kStale,
                      "chassis source timestamp exceeded stale timeout");
  }
  if (!std::isfinite(dt_s) || dt_s < config_.min_dt_s ||
      dt_s > config_.max_dt_s) {
    return invalidate(ChassisOdomStatus::kDtInvalid,
                      "chassis source timestamp produced unreasonable dt");
  }

  const double dx_odom = sample.x_m - previous_->x_m;
  const double dy_odom = sample.y_m - previous_->y_m;
  const double previous_yaw = previous_->yaw_rad;
  const double cos_yaw = std::cos(previous_yaw);
  const double sin_yaw = std::sin(previous_yaw);
  const double dx_body = cos_yaw * dx_odom + sin_yaw * dy_odom;
  const double dy_body = -sin_yaw * dx_odom + cos_yaw * dy_odom;
  const double yaw_delta =
      std::remainder(sample.yaw_rad - previous_yaw, kTwoPi);
  const double linear_x = dx_body / dt_s;
  const double linear_y = dy_body / dt_s;
  const double angular_z = yaw_delta / dt_s;
  if (!std::isfinite(linear_x) || !std::isfinite(linear_y) ||
      !std::isfinite(angular_z)) {
    return invalidate(ChassisOdomStatus::kInvalidSample,
                      "derived chassis twist is not finite");
  }

  anchor(sample, host_received_steady_ns);
  state_.linear_x_mps = linear_x;
  state_.linear_y_mps = linear_y;
  state_.angular_z_rps = angular_z;
  ++state_.accepted_updates;
  return result(ChassisOdomStatus::kAccepted, nullptr, dt_s);
}

ChassisOdomUpdate ChassisOdomTracker::checkStale(
    uint64_t host_now_steady_ns) noexcept {
  if (!state_.session_active || !previous_.has_value() ||
      state_.last_host_received_steady_ns == 0U) {
    if (!state_.session_active) {
      return result(ChassisOdomStatus::kSessionClosed,
                    "chassis odom session is closed");
    }
    const ChassisOdomStatus status =
        last_status_ == ChassisOdomStatus::kNoSample
            ? ChassisOdomStatus::kNoSample
            : last_status_;
    return result(status, "no chassis baseline");
  }
  if (host_now_steady_ns < state_.last_host_received_steady_ns) {
    return invalidate(ChassisOdomStatus::kHostTimeBackward,
                      "host freshness clock moved backwards");
  }
  const uint64_t timeout_ns =
      static_cast<uint64_t>(config_.stale_timeout_ms) * 1000000U;
  if (host_now_steady_ns - state_.last_host_received_steady_ns > timeout_ns) {
    return invalidate(ChassisOdomStatus::kStale,
                      "chassis telemetry receive stream is stale");
  }
  return result(last_status_, nullptr);
}

void ChassisStateAdapter::beginSession(uint64_t connection_epoch) noexcept {
  odom_.beginSession(connection_epoch);
}

void ChassisStateAdapter::endSession() noexcept {
  odom_.endSession();
}

ChassisSubmitResult ChassisStateAdapter::rejectSequence(const char *reason) {
  ++counters_.sequence_rejected;
  ChassisSubmitResult result;
  result.status = ChassisSubmitStatus::kSequenceRejected;
  result.decode_status = SrpV4DecodeStatus::kNotAttempted;
  result.odom_update =
      odom_.invalidate(ChassisOdomStatus::kInvalidSample, reason);
  result.error = reason;
  return result;
}

ChassisSubmitResult ChassisStateAdapter::submit(
    const ChassisTelemetryFrame &frame) {
  const SrpV4FrameDecodeResult decoded_frame =
      SrpV4Decoder{}.decode(frame.payload);
  return submitDecoded(frame, decoded_frame);
}

ChassisSubmitResult ChassisStateAdapter::submitDecoded(
    const ChassisTelemetryFrame &frame,
    const SrpV4FrameDecodeResult &decoded_frame) {
  ChassisSubmitResult result;
  const SrpV4DecodeResult decoded = decoder_.decode(
      decoded_frame, frame.payload.data(), frame.payload.size());
  last_decode_status_ = decoded.status;
  result.decode_status = decoded.status;
  if (!decoded.accepted()) {
    ++counters_.decode_rejected;
    result.status = ChassisSubmitStatus::kDecodeRejected;
    result.error = decoded.reason;
    result.odom_update = odom_.invalidate(
        ChassisOdomStatus::kInvalidSample, result.error.c_str());
    return result;
  }

  if ((frame.origin == TelemetryOrigin::kLiveGateway &&
       (!config_.allow_live_telemetry || !config_.enable_live_odom)) ||
      (frame.origin == TelemetryOrigin::kOfflineFixture &&
       !config_.allow_offline_fixtures)) {
    ++counters_.disabled;
    result.status = ChassisSubmitStatus::kDisabled;
    result.error = frame.origin == TelemetryOrigin::kLiveGateway
                       ? "live chassis telemetry or odom is disabled"
                       : "offline chassis fixtures are disabled";
    result.odom_update.status = ChassisOdomStatus::kSessionClosed;
    result.odom_update.state = odom_.state();
    result.odom_update.reason = result.error;
    return result;
  }
  if (frame.origin == TelemetryOrigin::kLiveGateway &&
      frame.connection_epoch == 0U) {
    return rejectSequence("live chassis telemetry requires a connection epoch");
  }
  if (frame.origin == TelemetryOrigin::kLiveGateway &&
      config_.require_outer_sequence) {
    switch (frame.outer_sequence_status) {
      case TelemetryOuterSequenceStatus::kFirst:
      case TelemetryOuterSequenceStatus::kInOrder:
      case TelemetryOuterSequenceStatus::kWrap:
        break;
      case TelemetryOuterSequenceStatus::kUnknown:
        return rejectSequence("chassis frame lacks outer sequence state");
      case TelemetryOuterSequenceStatus::kDuplicate:
        return rejectSequence("chassis outer sequence duplicated");
      case TelemetryOuterSequenceStatus::kOutOfOrder:
        return rejectSequence("chassis outer sequence moved backwards");
      case TelemetryOuterSequenceStatus::kJump:
        return rejectSequence("chassis outer sequence contains a gap");
    }
    if (frame.outer_sequence_gap != 0U) {
      return rejectSequence("chassis outer sequence reports missing frames");
    }
  }

  result.odom_update = odom_.update(decoded.sample, frame.connection_epoch,
                                    frame.host_received_steady_ns);
  if (result.odom_update.status == ChassisOdomStatus::kAnchored) {
    ++counters_.anchored;
    result.status = ChassisSubmitStatus::kAnchored;
    return result;
  }
  if (result.odom_update.status == ChassisOdomStatus::kAccepted) {
    ++counters_.accepted;
    result.status = ChassisSubmitStatus::kAccepted;
    return result;
  }

  if (result.odom_update.status == ChassisOdomStatus::kSequenceDuplicate ||
      result.odom_update.status == ChassisOdomStatus::kSequenceRollback) {
    ++counters_.sequence_rejected;
    result.status = ChassisSubmitStatus::kSequenceRejected;
  } else {
    ++counters_.odom_rejected;
    result.status = ChassisSubmitStatus::kOdomRejected;
  }
  result.error = result.odom_update.reason;
  return result;
}

ChassisOdomUpdate ChassisStateAdapter::checkStale(
    uint64_t host_now_steady_ns) noexcept {
  return odom_.checkStale(host_now_steady_ns);
}

ChassisOdomUpdate ChassisStateAdapter::invalidate(
    ChassisOdomStatus status, const char *reason) noexcept {
  ++counters_.odom_rejected;
  return odom_.invalidate(status, reason);
}

void SrpV4TelemetryAdapter::beginSession(uint64_t connection_epoch) noexcept {
  chassis_.beginSession(connection_epoch);
}

void SrpV4TelemetryAdapter::endSession() noexcept {
  chassis_.endSession();
}

bool SrpV4TelemetryAdapter::rejectOuterSequence(
    const ChassisTelemetryFrame &frame, std::string &reason) const {
  if (frame.origin != TelemetryOrigin::kLiveGateway ||
      !config_.require_outer_sequence) {
    return false;
  }
  switch (frame.outer_sequence_status) {
    case TelemetryOuterSequenceStatus::kFirst:
    case TelemetryOuterSequenceStatus::kInOrder:
    case TelemetryOuterSequenceStatus::kWrap:
      if (frame.outer_sequence_gap == 0U) {
        return false;
      }
      reason = "SRP v4 outer sequence reports missing frames";
      return true;
    case TelemetryOuterSequenceStatus::kUnknown:
      reason = "SRP v4 frame lacks outer sequence state";
      return true;
    case TelemetryOuterSequenceStatus::kDuplicate:
      reason = "SRP v4 outer sequence duplicated";
      return true;
    case TelemetryOuterSequenceStatus::kOutOfOrder:
      reason = "SRP v4 outer sequence moved backwards";
      return true;
    case TelemetryOuterSequenceStatus::kJump:
      reason = "SRP v4 outer sequence contains a gap";
      return true;
  }
  return false;
}

SrpV4TelemetrySubmitResult SrpV4TelemetryAdapter::submit(
    const ChassisTelemetryFrame &frame) {
  SrpV4TelemetrySubmitResult result;
  ++counters_.received;
  const SrpV4FrameDecodeResult decoded = decoder_.decode(frame.payload);
  last_frame_decode_status_ = decoded.status;
  result.frame_decode_status = decoded.status;
  if (decoded.header_available) {
    result.message_id = decoded.header.message_id;
  }
  if (!decoded.accepted()) {
    ++counters_.frame_decode_rejected;
    result.status = SrpV4TelemetrySubmitStatus::kFrameDecodeRejected;
    result.error = decoded.reason;
    if (decoded.header_available &&
        decoded.header.message_id == kSrpV4ChassisMessageId) {
      ++counters_.chassis_frames;
      result.chassis_result = chassis_.submitDecoded(frame, decoded);
    } else {
      result.chassis_result.odom_update = chassis_.invalidate(
          ChassisOdomStatus::kInvalidSample,
          decoded.reason.empty() ? "SRP v4 common decode rejected"
                                 : decoded.reason.c_str());
    }
    return result;
  }

  if (decoded.header.message_id == kSrpV4ChassisMessageId) {
    ++counters_.chassis_frames;
    result.status = SrpV4TelemetrySubmitStatus::kChassis;
    result.chassis_result = chassis_.submitDecoded(frame, decoded);
    result.error = result.chassis_result.error;
    return result;
  }

  if (decoded.header.message_id == kSrpV4ImuMessageId) {
    ++counters_.imu_frames;
  } else if (decoded.header.message_id == kSrpV4WheelMessageId) {
    ++counters_.wheel_frames;
  } else {
    ++counters_.other_frames;
  }

  if (rejectOuterSequence(frame, result.error)) {
    ++counters_.outer_sequence_rejected;
    result.status = SrpV4TelemetrySubmitStatus::kOuterSequenceRejected;
    // A rejected outer frame only invalidates chassis admission when the
    // validated inner SRP message is actually chassis state. Other legal
    // telemetry types share the global outer sequence but must not disturb
    // the chassis baseline.
    if (decoded.header.message_id == kSrpV4ChassisMessageId) {
      result.chassis_result.odom_update = chassis_.invalidate(
          ChassisOdomStatus::kInvalidSample, result.error.c_str());
    }
    return result;
  }

  result.status = SrpV4TelemetrySubmitStatus::kIgnored;
  return result;
}

ChassisOdomUpdate SrpV4TelemetryAdapter::invalidateChassis(
    ChassisOdomStatus status, const char *reason) noexcept {
  return chassis_.invalidate(status, reason);
}

ChassisOdomUpdate SrpV4TelemetryAdapter::checkStale(
    uint64_t host_now_steady_ns) noexcept {
  return chassis_.checkStale(host_now_steady_ns);
}

const char *toString(ChassisOdomStatus status) noexcept {
  switch (status) {
    case ChassisOdomStatus::kNoSample:
      return "no_sample";
    case ChassisOdomStatus::kAnchored:
      return "anchored";
    case ChassisOdomStatus::kAccepted:
      return "accepted";
    case ChassisOdomStatus::kSessionClosed:
      return "session_closed";
    case ChassisOdomStatus::kInvalidConfig:
      return "invalid_config";
    case ChassisOdomStatus::kInvalidSample:
      return "invalid_sample";
    case ChassisOdomStatus::kEpochChanged:
      return "epoch_changed";
    case ChassisOdomStatus::kSequenceDuplicate:
      return "sequence_duplicate";
    case ChassisOdomStatus::kSequenceRollback:
      return "sequence_rollback";
    case ChassisOdomStatus::kTimestampDuplicate:
      return "timestamp_duplicate";
    case ChassisOdomStatus::kTimestampRollback:
      return "timestamp_rollback";
    case ChassisOdomStatus::kDtInvalid:
      return "dt_invalid";
    case ChassisOdomStatus::kHostTimeBackward:
      return "host_time_backward";
    case ChassisOdomStatus::kStale:
      return "stale";
  }
  return "unknown";
}

const char *toString(ChassisSubmitStatus status) noexcept {
  switch (status) {
    case ChassisSubmitStatus::kDisabled:
      return "disabled";
    case ChassisSubmitStatus::kDecodeRejected:
      return "decode_rejected";
    case ChassisSubmitStatus::kSequenceRejected:
      return "sequence_rejected";
    case ChassisSubmitStatus::kOdomRejected:
      return "odom_rejected";
    case ChassisSubmitStatus::kAnchored:
      return "anchored";
    case ChassisSubmitStatus::kAccepted:
      return "accepted";
  }
  return "unknown";
}

}  // namespace smartcar_state_bridge
