#include "smartcar_state_bridge/wheel_odom.hpp"

#include <cmath>
#include <limits>

namespace smartcar_state_bridge {

namespace {
constexpr double kPi = 3.1415926535897932384626433832795;
constexpr double kTwoPi = 2.0 * kPi;
// diff_drive_controller rejects intervals below this threshold internally.
// Validate it before calling the official API so a failed update can be
// latched deterministically instead of leaving wrapper and library state out
// of sync.
constexpr double kOfficialMinimumDtS = 1.0e-4;

bool finite(double value) noexcept { return std::isfinite(value); }

double normalizeAngle(double angle) noexcept {
  if (!finite(angle)) {
    return 0.0;
  }
  return std::remainder(angle, kTwoPi);
}

bool validFreshness(const WheelStatusSample &sample) noexcept {
  return sample.sample_tick.has_value() && sample.sample_seq.has_value();
}
}  // namespace

KinematicsStatus WheelKinematics::compute(const WheelStatusSample &sample,
                                           WheelKinematicsResult &result,
                                           std::string &error) const noexcept {
  result = WheelKinematicsResult{};
  error.clear();
  if (!finite(config_.track_width_m) || config_.track_width_m <= 0.0 ||
      !finite(config_.wheel_diameter_m) || config_.wheel_diameter_m <= 0.0 ||
      !finite(config_.max_abs_speed_mm_s) ||
      config_.max_abs_speed_mm_s <= 0.0) {
    error =
        "track_width_m, wheel_diameter_m, and max_abs_speed_mm_s must be finite and positive";
    return KinematicsStatus::kInvalidConfig;
  }
  for (const double sign : config_.wheel_speed_sign) {
    if (!finite(sign) || std::abs(std::abs(sign) - 1.0) > 1.0e-12) {
      error = "wheel_speed_sign entries must be +1 or -1";
      return KinematicsStatus::kInvalidConfig;
    }
  }
  if (!sample.valid) {
    error = "wheel sample is marked invalid";
    return KinematicsStatus::kInvalidSample;
  }

  for (std::size_t index = 0U; index < kWheelCount; ++index) {
    const double raw = sample.speed_mm_s[index];
    const double corrected = raw * config_.wheel_speed_sign[index];
    if (!finite(raw) || !finite(corrected) ||
        std::abs(raw) > config_.max_abs_speed_mm_s ||
        std::abs(corrected) > config_.max_abs_speed_mm_s) {
      error = "wheel speed is non-finite or outside configured bounds";
      return KinematicsStatus::kInvalidSample;
    }
    result.corrected_speed_mm_s[index] = corrected;
  }

  // RR/RF are the right side; LR/LF are the left side.  Do not reorder these
  // indices to FL/FR/RL/RR here.
  result.right_mps =
      0.5 * (result.corrected_speed_mm_s[static_cast<std::size_t>(WheelIndex::kRR)] +
             result.corrected_speed_mm_s[static_cast<std::size_t>(WheelIndex::kRF)]) /
      1000.0;
  result.left_mps =
      0.5 * (result.corrected_speed_mm_s[static_cast<std::size_t>(WheelIndex::kLR)] +
             result.corrected_speed_mm_s[static_cast<std::size_t>(WheelIndex::kLF)]) /
      1000.0;
  result.linear_mps = 0.5 * (result.left_mps + result.right_mps);
  result.angular_rps = (result.right_mps - result.left_mps) /
                      config_.track_width_m;
  if (!finite(result.linear_mps) || !finite(result.angular_rps)) {
    error = "computed wheel kinematics are non-finite";
    return KinematicsStatus::kInvalidSample;
  }
  return KinematicsStatus::kAccepted;
}

bool WheelKinematics::wheelAngularVelocityRadPerSec(
    const WheelStatusSample &sample, std::array<double, kWheelCount> &result,
    std::string &error) const noexcept {
  WheelKinematicsResult corrected;
  if (compute(sample, corrected, error) != KinematicsStatus::kAccepted) {
    result = {};
    return false;
  }
  const double radius = config_.wheel_diameter_m * 0.5;
  if (!finite(radius) || radius <= 0.0) {
    result = {};
    error = "wheel_diameter_m produces an invalid radius";
    return false;
  }
  for (std::size_t index = 0U; index < kWheelCount; ++index) {
    result[index] = (corrected.corrected_speed_mm_s[index] / 1000.0) / radius;
  }
  return true;
}

WheelFifoPushStatus WheelSampleFifo::push(
    const WheelStatusSample &sample) noexcept {
  if (!sample.valid) {
    ++invalid_count_;
    return WheelFifoPushStatus::kInvalid;
  }
  for (const double speed : sample.speed_mm_s) {
    if (!finite(speed)) {
      ++invalid_count_;
      return WheelFifoPushStatus::kInvalid;
    }
  }
  if (depth_ == 0U || queue_.size() >= depth_) {
    ++overflow_count_;
    overflowed_ = true;
    return WheelFifoPushStatus::kFull;
  }
  queue_.push_back(sample);
  return WheelFifoPushStatus::kAccepted;
}

bool WheelSampleFifo::pop(WheelStatusSample &sample) noexcept {
  if (queue_.empty()) {
    return false;
  }
  sample = std::move(queue_.front());
  queue_.pop_front();
  return true;
}

void WheelSampleFifo::clear() noexcept {
  queue_.clear();
  overflowed_ = false;
}

WheelOdom::WheelOdom(WheelOdomConfig config)
    : config_(std::move(config)), kinematics_(config_), fifo_(config_.fifo_depth) {
  config_valid_ = finite(config_.sample_tick_period_s) &&
                  config_.sample_tick_period_s > 0.0 &&
                  finite(config_.min_dt_s) && config_.min_dt_s >= 0.0 &&
                  finite(config_.max_dt_s) && config_.max_dt_s > config_.min_dt_s &&
                  finite(config_.wheel_diameter_m) &&
                  config_.wheel_diameter_m > 0.0 &&
                  config_.stale_timeout_ms > 0U;
  official_odom_.setWheelParams(
      config_.track_width_m, config_.wheel_diameter_m * 0.5,
      config_.wheel_diameter_m * 0.5);
  official_odom_.init(rclcpp::Time(static_cast<int64_t>(0), RCL_SYSTEM_TIME));
}

OdomUpdate WheelOdom::invalid(OdomUpdateStatus status,
                              const char *message) noexcept {
  OdomUpdate result;
  result.status = status;
  result.state = state_;
  result.reason = message == nullptr ? std::string{} : std::string(message);
  last_status_ = status;
  return result;
}

void WheelOdom::latch(OdomUpdateStatus status) noexcept {
  state_.invalid_latched = true;
  state_.valid = false;
  last_status_ = status;
}

bool WheelOdom::checkForwardCounter(uint32_t previous, uint32_t current,
                                    bool allow_wrap, uint32_t &delta,
                                    bool &wrapped) const noexcept {
  delta = current - previous;
  wrapped = current < previous;
  if (delta == 0U || delta >= 0x80000000U) {
    return false;
  }
  if (wrapped && !allow_wrap) {
    return false;
  }
  return true;
}

OdomUpdate WheelOdom::integrate(const WheelStatusSample &sample,
                                uint64_t host_received_steady_ns) noexcept {
  if (!session_active_) {
    return invalid(OdomUpdateStatus::kSessionClosed,
                   "odom session is closed; call beginSession");
  }
  if (state_.invalid_latched) {
    return invalid(OdomUpdateStatus::kLatchedInvalid,
                   "odom is latched invalid; begin a new session");
  }
  if (!config_valid_) {
    latch(OdomUpdateStatus::kInvalidConfig);
    return invalid(OdomUpdateStatus::kInvalidConfig,
                   "wheel odom configuration is invalid");
  }
  if (connection_epoch_.has_value()) {
    if (sample.connection_epoch == 0U) {
      latch(OdomUpdateStatus::kEpochChanged);
      return invalid(OdomUpdateStatus::kEpochChanged,
                     "sample is missing the active connection epoch");
    }
    if (*connection_epoch_ != sample.connection_epoch) {
      latch(OdomUpdateStatus::kEpochChanged);
      return invalid(OdomUpdateStatus::kEpochChanged,
                     "connection epoch changed; begin a new odom session");
    }
  } else if (sample.connection_epoch != 0U) {
    connection_epoch_ = sample.connection_epoch;
    state_.session_epoch = connection_epoch_;
  }
  if (sample.source_epoch.has_value()) {
    if (!source_epoch_.has_value()) {
      source_epoch_ = *sample.source_epoch;
      state_.source_epoch = source_epoch_;
    } else if (*source_epoch_ != *sample.source_epoch) {
      latch(OdomUpdateStatus::kSourceEpochChanged);
      return invalid(OdomUpdateStatus::kSourceEpochChanged,
                     "source wheel epoch changed; begin a new odom session");
    }
  } else if (source_epoch_.has_value()) {
    latch(OdomUpdateStatus::kSourceEpochChanged);
    return invalid(OdomUpdateStatus::kSourceEpochChanged,
                   "source wheel epoch disappeared after anchoring");
  }
  if (!sample.valid) {
    latch(OdomUpdateStatus::kInvalidSample);
    return invalid(OdomUpdateStatus::kInvalidSample,
                   "wheel source marked sample invalid");
  }
  if (config_.require_source_freshness && !validFreshness(sample)) {
    latch(OdomUpdateStatus::kMissingFreshness);
    return invalid(OdomUpdateStatus::kMissingFreshness,
                   "sample_tick and sample_seq are required");
  }
  if (sample.source_age_ms.has_value() &&
      *sample.source_age_ms > config_.stale_timeout_ms) {
    latch(OdomUpdateStatus::kStale);
    return invalid(OdomUpdateStatus::kStale,
                   "source wheel sample exceeds stale timeout");
  }

  WheelKinematicsResult kin;
  std::string kinematics_error;
  const KinematicsStatus kin_status =
      kinematics_.compute(sample, kin, kinematics_error);
  if (kin_status == KinematicsStatus::kInvalidConfig) {
    latch(OdomUpdateStatus::kInvalidConfig);
    return invalid(OdomUpdateStatus::kInvalidConfig, kinematics_error.c_str());
  }
  if (kin_status != KinematicsStatus::kAccepted) {
    latch(OdomUpdateStatus::kInvalidSample);
    return invalid(OdomUpdateStatus::kInvalidSample, kinematics_error.c_str());
  }

  const uint64_t received_ns = host_received_steady_ns != 0U
                                   ? host_received_steady_ns
                                   : sample.host_received_steady_ns;
  if (last_host_received_steady_ns_ != 0U && received_ns != 0U) {
    if (received_ns < last_host_received_steady_ns_) {
      latch(OdomUpdateStatus::kHostTimeBackward);
      return invalid(OdomUpdateStatus::kHostTimeBackward,
                     "host receive time moved backwards");
    }
    const uint64_t timeout_ns =
        static_cast<uint64_t>(config_.stale_timeout_ms) * 1000000ULL;
    if (received_ns - last_host_received_steady_ns_ > timeout_ns) {
      latch(OdomUpdateStatus::kStale);
      return invalid(OdomUpdateStatus::kStale,
                     "no fresh wheel sample within stale timeout");
    }
  }

  // The first valid sample anchors sequence/time; it does not fabricate a
  // displacement because no interval has been observed yet.
  if (!last_tick_.has_value() || !last_seq_.has_value()) {
    last_tick_ = sample.sample_tick;
    last_seq_ = sample.sample_seq;
    last_source_time_s_ = sample.source_time_s;
    last_host_received_steady_ns_ = received_ns;
    state_.anchored = true;
    state_.valid = true;
    state_.last_sample_tick = sample.sample_tick;
    state_.last_sample_seq = sample.sample_seq;
    state_.last_host_received_steady_ns = received_ns;
    state_.last_source_time_s = sample.source_time_s;
    state_.last_source_age_ms = sample.source_age_ms;
    if (connection_epoch_.has_value()) {
      state_.session_epoch = connection_epoch_;
    }
    // Start the official integrator at a logical zero timestamp.  The source
    // tick is retained for freshness/sequence checks; it is not used as a ROS
    // wall-clock value and may legally wrap.
    official_odom_.resetOdometry();
    official_odom_.setWheelParams(
        config_.track_width_m, config_.wheel_diameter_m * 0.5,
        config_.wheel_diameter_m * 0.5);
    official_odom_.init(
        rclcpp::Time(static_cast<int64_t>(0), RCL_SYSTEM_TIME));
    integration_time_s_ = 0.0;
    integration_initialized_ = true;
    OdomUpdate result;
    result.status = OdomUpdateStatus::kAnchored;
    result.state = state_;
    result.kinematics = kin;
    last_status_ = result.status;
    return result;
  }

  if (!sample.sample_seq.has_value() || !sample.sample_tick.has_value()) {
    // This can only happen when freshness is optional and the initial sample
    // was manually anchored; no interval may be integrated without both IDs.
    latch(OdomUpdateStatus::kMissingFreshness);
    return invalid(OdomUpdateStatus::kMissingFreshness,
                   "sample freshness disappeared after anchoring");
  }

  uint32_t sequence_delta = 0U;
  bool sequence_wrapped = false;
  if (!checkForwardCounter(*last_seq_, *sample.sample_seq,
                           config_.allow_sequence_wrap, sequence_delta,
                           sequence_wrapped)) {
    if (*sample.sample_seq == *last_seq_) {
      latch(OdomUpdateStatus::kSequenceDuplicate);
      return invalid(OdomUpdateStatus::kSequenceDuplicate,
                     "duplicate wheel sample sequence");
    }
    latch(OdomUpdateStatus::kSequenceOutOfOrder);
    return invalid(OdomUpdateStatus::kSequenceOutOfOrder,
                   "wheel sample sequence moved backwards");
  }
  if (sequence_delta != 1U) {
    latch(OdomUpdateStatus::kSequenceGap);
    return invalid(OdomUpdateStatus::kSequenceGap,
                   "wheel sample sequence contains a gap");
  }
  (void)sequence_wrapped;

  uint32_t tick_delta = 0U;
  bool tick_wrapped = false;
  if (!checkForwardCounter(*last_tick_, *sample.sample_tick,
                           config_.allow_tick_wrap, tick_delta, tick_wrapped)) {
    if (*sample.sample_tick == *last_tick_) {
      latch(OdomUpdateStatus::kTickDuplicate);
      return invalid(OdomUpdateStatus::kTickDuplicate,
                     "wheel sample tick did not advance");
    }
    latch(OdomUpdateStatus::kTickBackward);
    return invalid(OdomUpdateStatus::kTickBackward,
                   "wheel sample tick moved backwards");
  }
  (void)tick_wrapped;

  const double dt = static_cast<double>(tick_delta) *
                    config_.sample_tick_period_s;
  if (!finite(dt) || dt <= config_.min_dt_s || dt > config_.max_dt_s) {
    latch(OdomUpdateStatus::kTimeInvalid);
    return invalid(OdomUpdateStatus::kTimeInvalid,
                   "wheel sample interval is outside configured bounds");
  }
  if (last_source_time_s_.has_value()) {
    if (!sample.source_time_s.has_value()) {
      latch(OdomUpdateStatus::kTimeInvalid);
      return invalid(OdomUpdateStatus::kTimeInvalid,
                     "source sample time disappeared after anchoring");
    }
    const double source_dt = *sample.source_time_s - *last_source_time_s_;
    if (!finite(source_dt) || source_dt <= 0.0) {
      latch(OdomUpdateStatus::kTimeInvalid);
      return invalid(OdomUpdateStatus::kTimeInvalid,
                     "source sample time is not strictly increasing");
    }
  }

  if (!integration_initialized_ || dt < kOfficialMinimumDtS) {
    latch(OdomUpdateStatus::kTimeInvalid);
    return invalid(OdomUpdateStatus::kTimeInvalid,
                   "wheel sample interval is below diff-drive integration minimum");
  }

  integration_time_s_ += dt;
  if (!finite(integration_time_s_) ||
      integration_time_s_ >
          static_cast<double>(std::numeric_limits<int64_t>::max()) / 1.0e9) {
    latch(OdomUpdateStatus::kTimeInvalid);
    return invalid(OdomUpdateStatus::kTimeInvalid,
                   "logical odom integration time is non-finite");
  }
  const auto integration_ns = static_cast<int64_t>(
      std::llround(integration_time_s_ * 1.0e9));
  const rclcpp::Time integration_time(integration_ns, RCL_SYSTEM_TIME);
  // Humble 2.53's updateFromVelocity takes per-cycle wheel displacement, not
  // m/s.  The wrapper has already validated dt and source freshness.
  const bool integrated = official_odom_.updateFromVelocity(
      kin.left_mps * dt, kin.right_mps * dt, integration_time);
  if (!integrated || !finite(official_odom_.getX()) ||
      !finite(official_odom_.getY()) ||
      !finite(official_odom_.getHeading())) {
    latch(OdomUpdateStatus::kTimeInvalid);
    return invalid(OdomUpdateStatus::kTimeInvalid,
                   "official diff-drive odometry rejected the interval");
  }

  state_.x_m = official_odom_.getX();
  state_.y_m = official_odom_.getY();
  state_.heading_rad = normalizeAngle(official_odom_.getHeading());
  state_.linear_mps = official_odom_.getLinear();
  state_.angular_rps = official_odom_.getAngular();
  if (!finite(state_.linear_mps) || !finite(state_.angular_rps)) {
    latch(OdomUpdateStatus::kTimeInvalid);
    return invalid(OdomUpdateStatus::kTimeInvalid,
                   "official diff-drive velocity is non-finite");
  }
  ++state_.accepted_samples;
  state_.valid = true;
  state_.last_sample_tick = sample.sample_tick;
  state_.last_sample_seq = sample.sample_seq;
  state_.last_host_received_steady_ns = received_ns;
  state_.last_source_time_s = sample.source_time_s;
  state_.last_source_age_ms = sample.source_age_ms;
  last_tick_ = sample.sample_tick;
  last_seq_ = sample.sample_seq;
  last_source_time_s_ = sample.source_time_s;
  last_host_received_steady_ns_ = received_ns;

  OdomUpdate result;
  result.status = OdomUpdateStatus::kAccepted;
  result.state = state_;
  result.kinematics = kin;
  result.dt_s = dt;
  last_status_ = result.status;
  return result;
}

OdomUpdate WheelOdom::update(const WheelStatusSample &sample,
                             uint64_t host_received_steady_ns) noexcept {
  return integrate(sample, host_received_steady_ns);
}

OdomUpdate WheelOdom::latchExternalInvalid(OdomUpdateStatus status,
                                            const char *message) noexcept {
  latch(status);
  return invalid(status, message);
}

OdomUpdate WheelOdom::checkStale(uint64_t host_now_steady_ns) noexcept {
  if (!session_active_) {
    return invalid(OdomUpdateStatus::kSessionClosed,
                   "odom session is closed; call beginSession");
  }
  if (state_.invalid_latched) {
    return invalid(OdomUpdateStatus::kLatchedInvalid,
                   "odom is latched invalid; begin a new session");
  }
  if (!state_.anchored || last_host_received_steady_ns_ == 0U ||
      host_now_steady_ns == 0U) {
    OdomUpdate result;
    result.status = OdomUpdateStatus::kNoSample;
    result.state = state_;
    result.reason = "no anchored wheel sample to age";
    last_status_ = result.status;
    return result;
  }
  if (host_now_steady_ns < last_host_received_steady_ns_) {
    latch(OdomUpdateStatus::kHostTimeBackward);
    return invalid(OdomUpdateStatus::kHostTimeBackward,
                   "host watchdog time moved backwards");
  }
  const uint64_t timeout_ns =
      static_cast<uint64_t>(config_.stale_timeout_ms) * 1000000ULL;
  if (host_now_steady_ns - last_host_received_steady_ns_ > timeout_ns) {
    latch(OdomUpdateStatus::kStale);
    return invalid(OdomUpdateStatus::kStale,
                   "no fresh wheel sample within stale timeout");
  }
  OdomUpdate result;
  result.status = OdomUpdateStatus::kNoSample;
  result.state = state_;
  last_status_ = result.status;
  return result;
}

WheelFifoPushStatus WheelOdom::enqueue(
    const WheelStatusSample &sample) noexcept {
  const WheelFifoPushStatus result = fifo_.push(sample);
  if (result == WheelFifoPushStatus::kFull) {
    latch(OdomUpdateStatus::kLatchedInvalid);
    last_status_ = OdomUpdateStatus::kLatchedInvalid;
  } else if (result == WheelFifoPushStatus::kInvalid) {
    latch(OdomUpdateStatus::kInvalidSample);
    last_status_ = OdomUpdateStatus::kInvalidSample;
  }
  return result;
}

std::optional<OdomUpdate> WheelOdom::processNext() noexcept {
  WheelStatusSample sample;
  if (!fifo_.pop(sample)) {
    return std::nullopt;
  }
  return update(sample, sample.host_received_steady_ns);
}

std::vector<OdomUpdate> WheelOdom::processAll() noexcept {
  std::vector<OdomUpdate> results;
  while (fifo_.size() > 0U) {
    auto result = processNext();
    if (!result.has_value()) {
      break;
    }
    results.push_back(*result);
    if (state_.invalid_latched) {
      break;
    }
  }
  return results;
}

void WheelOdom::beginSession(uint64_t connection_epoch) noexcept {
  clearForSession();
  session_active_ = true;
  if (connection_epoch != 0U) {
    connection_epoch_ = connection_epoch;
    state_.session_epoch = connection_epoch_;
  }
}

void WheelOdom::endSession() noexcept {
  session_active_ = false;
  state_.valid = false;
  state_.invalid_latched = true;
  fifo_.clear();
  last_status_ = OdomUpdateStatus::kSessionClosed;
}

void WheelOdom::clearForSession() noexcept {
  fifo_.clear();
  state_ = OdomState{};
  last_status_ = OdomUpdateStatus::kNoSample;
  connection_epoch_.reset();
  last_tick_.reset();
  last_seq_.reset();
  last_source_time_s_.reset();
  source_epoch_.reset();
  last_host_received_steady_ns_ = 0U;
  official_odom_.resetOdometry();
  official_odom_.setWheelParams(
      config_.track_width_m, config_.wheel_diameter_m * 0.5,
      config_.wheel_diameter_m * 0.5);
  official_odom_.init(rclcpp::Time(static_cast<int64_t>(0), RCL_SYSTEM_TIME));
  integration_time_s_ = 0.0;
  integration_initialized_ = false;
  session_active_ = true;
}

const char *toString(OdomUpdateStatus status) noexcept {
  switch (status) {
    case OdomUpdateStatus::kAccepted:
      return "accepted";
    case OdomUpdateStatus::kAnchored:
      return "anchored";
    case OdomUpdateStatus::kNoSample:
      return "no_sample";
    case OdomUpdateStatus::kSessionClosed:
      return "session_closed";
    case OdomUpdateStatus::kLatchedInvalid:
      return "latched_invalid";
    case OdomUpdateStatus::kInvalidConfig:
      return "invalid_config";
    case OdomUpdateStatus::kInvalidSample:
      return "invalid_sample";
    case OdomUpdateStatus::kMissingFreshness:
      return "missing_freshness";
    case OdomUpdateStatus::kEpochChanged:
      return "epoch_changed";
    case OdomUpdateStatus::kSourceEpochChanged:
      return "source_epoch_changed";
    case OdomUpdateStatus::kSequenceDuplicate:
      return "sequence_duplicate";
    case OdomUpdateStatus::kSequenceOutOfOrder:
      return "sequence_out_of_order";
    case OdomUpdateStatus::kSequenceGap:
      return "sequence_gap";
    case OdomUpdateStatus::kTickBackward:
      return "tick_backward";
    case OdomUpdateStatus::kTickDuplicate:
      return "tick_duplicate";
    case OdomUpdateStatus::kTimeInvalid:
      return "time_invalid";
    case OdomUpdateStatus::kHostTimeBackward:
      return "host_time_backward";
    case OdomUpdateStatus::kStale:
      return "stale";
  }
  return "unknown";
}

}  // namespace smartcar_state_bridge
