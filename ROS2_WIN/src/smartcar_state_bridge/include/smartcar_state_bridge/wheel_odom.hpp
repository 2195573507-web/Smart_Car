#pragma once

#include "diff_drive_controller/odometry.hpp"
#include "smartcar_state_bridge/telemetry_decoder.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace smartcar_state_bridge {

inline constexpr std::size_t kWheelCount = 4U;

// Keep this explicit in every calculation and test.  It is the firmware wire
// order, not a display/UI order.
enum class WheelIndex : std::size_t {
  kRR = 0U,
  kRF = 1U,
  kLR = 2U,
  kLF = 3U,
};

struct WheelKinematicsConfig {
  // 193 mm is only a provisional starting value from the project plan.  A
  // measured/calibrated value must be supplied before live acceptance.
  double track_width_m{0.193};
  std::array<double, kWheelCount> wheel_speed_sign{{1.0, 1.0, 1.0, 1.0}};
  double wheel_diameter_m{0.065};
  double max_abs_speed_mm_s{100000.0};
};

struct WheelKinematicsResult {
  double right_mps{0.0};
  double left_mps{0.0};
  double linear_mps{0.0};
  double angular_rps{0.0};
  std::array<double, kWheelCount> corrected_speed_mm_s{};
};

enum class KinematicsStatus {
  kAccepted,
  kInvalidConfig,
  kInvalidSample,
};

class WheelKinematics final {
 public:
  explicit WheelKinematics(WheelKinematicsConfig config = {})
      : config_(std::move(config)) {}

  KinematicsStatus compute(const WheelStatusSample &sample,
                           WheelKinematicsResult &result,
                           std::string &error) const noexcept;

  // The optional JointState observation uses the measured wheel diameter.  It
  // is intentionally separate from linear odometry, whose wire unit is mm/s.
  bool wheelAngularVelocityRadPerSec(const WheelStatusSample &sample,
                                     std::array<double, kWheelCount> &result,
                                     std::string &error) const noexcept;

  const WheelKinematicsConfig &config() const noexcept { return config_; }

 private:
  WheelKinematicsConfig config_;
};

enum class WheelFifoPushStatus {
  kAccepted,
  kFull,
  kInvalid,
};

class WheelSampleFifo final {
 public:
  explicit WheelSampleFifo(std::size_t depth = 32U) : depth_(depth) {}

  WheelFifoPushStatus push(const WheelStatusSample &sample) noexcept;
  bool pop(WheelStatusSample &sample) noexcept;
  void clear() noexcept;

  std::size_t size() const noexcept { return queue_.size(); }
  std::size_t depth() const noexcept { return depth_; }
  std::size_t overflowCount() const noexcept { return overflow_count_; }
  std::size_t invalidCount() const noexcept { return invalid_count_; }
  bool overflowed() const noexcept { return overflowed_; }

 private:
  std::deque<WheelStatusSample> queue_;
  std::size_t depth_{32U};
  std::size_t overflow_count_{0U};
  std::size_t invalid_count_{0U};
  bool overflowed_{false};
};

struct WheelOdomConfig : public WheelKinematicsConfig {
  // Source sample_tick units.  This is a configured contract value, not an
  // inference from ROS receive time.
  double sample_tick_period_s{0.001};
  double min_dt_s{1.0e-6};
  double max_dt_s{1.0};
  uint32_t stale_timeout_ms{500U};
  std::size_t fifo_depth{32U};
  bool require_source_freshness{true};
  bool allow_sequence_wrap{true};
  bool allow_tick_wrap{true};
};

enum class OdomUpdateStatus {
  kAccepted,
  kAnchored,
  kNoSample,
  kSessionClosed,
  kLatchedInvalid,
  kInvalidConfig,
  kInvalidSample,
  kMissingFreshness,
  kEpochChanged,
  kSourceEpochChanged,
  kSequenceDuplicate,
  kSequenceOutOfOrder,
  kSequenceGap,
  kTickBackward,
  kTickDuplicate,
  kTimeInvalid,
  kHostTimeBackward,
  kStale,
};

struct OdomState {
  double x_m{0.0};
  double y_m{0.0};
  double heading_rad{0.0};
  double linear_mps{0.0};
  double angular_rps{0.0};
  uint64_t accepted_samples{0U};
  bool anchored{false};
  bool valid{false};
  bool invalid_latched{false};
  std::optional<uint32_t> last_sample_tick;
  std::optional<uint32_t> last_sample_seq;
  std::optional<uint64_t> session_epoch;
  std::optional<uint32_t> source_epoch;
  // Diagnostics-only timing metadata.  The monotonic host timestamp is never
  // used as a substitute for the source sample tick during integration.
  uint64_t last_host_received_steady_ns{0U};
  std::optional<double> last_source_time_s;
  std::optional<uint32_t> last_source_age_ms;
};

struct OdomUpdate {
  OdomUpdateStatus status{OdomUpdateStatus::kNoSample};
  OdomState state{};
  WheelKinematicsResult kinematics{};
  double dt_s{0.0};
  std::string reason;
};

// A deterministic, read-only differential-drive integrator.  It mirrors the
// exact arc integration used by mature ROS controllers while keeping the
// freshness/sequence gate in this project-owned wrapper.
class WheelOdom final {
 public:
  explicit WheelOdom(WheelOdomConfig config = {});

  OdomUpdate update(const WheelStatusSample &sample,
                    uint64_t host_received_steady_ns = 0U) noexcept;

  WheelFifoPushStatus enqueue(const WheelStatusSample &sample) noexcept;
  std::optional<OdomUpdate> processNext() noexcept;
  std::vector<OdomUpdate> processAll() noexcept;

  // Allows an owning gateway to latch a transport-level fault (for example an
  // outer sequence gap) without fabricating a wheel sample.  Recovery still
  // requires beginSession().
  OdomUpdate latchExternalInvalid(OdomUpdateStatus status,
                                  const char *message) noexcept;

  // A new connection/session is the only recovery from a latched invalid
  // state.  This prevents stale feedback from silently resuming integration.
  void beginSession(uint64_t connection_epoch = 0U) noexcept;
  void endSession() noexcept;

  // Check source silence using the monotonic host receive clock.  This is
  // intended for an owning node's watchdog/diagnostics timer; it never
  // fabricates an odometry sample.
  OdomUpdate checkStale(uint64_t host_now_steady_ns) noexcept;

  const WheelOdomConfig &config() const noexcept { return config_; }
  const OdomState &state() const noexcept { return state_; }
  OdomUpdateStatus lastStatus() const noexcept { return last_status_; }
  const WheelSampleFifo &fifo() const noexcept { return fifo_; }
  WheelSampleFifo &fifo() noexcept { return fifo_; }

 private:
  OdomUpdate invalid(OdomUpdateStatus status, const char *message) noexcept;
  OdomUpdate integrate(const WheelStatusSample &sample,
                       uint64_t host_received_steady_ns) noexcept;
  bool checkForwardCounter(uint32_t previous, uint32_t current,
                           bool allow_wrap, uint32_t &delta,
                           bool &wrapped) const noexcept;
  void clearForSession() noexcept;
  void latch(OdomUpdateStatus status) noexcept;

  WheelOdomConfig config_;
  WheelKinematics kinematics_;
  WheelSampleFifo fifo_;
  OdomState state_{};
  OdomUpdateStatus last_status_{OdomUpdateStatus::kNoSample};
  std::optional<uint64_t> connection_epoch_;
  std::optional<uint32_t> source_epoch_;
  std::optional<uint32_t> last_tick_;
  std::optional<uint32_t> last_seq_;
  std::optional<double> last_source_time_s_;
  uint64_t last_host_received_steady_ns_{0U};
  // Humble's public updateFromVelocity API consumes per-cycle wheel
  // displacements (metres), despite its historical parameter names.  Keep a
  // logical monotonic clock so source tick wrap does not move the ROS time
  // backwards while the wrapper continues to enforce source freshness.
  diff_drive_controller::Odometry official_odom_{1U};
  double integration_time_s_{0.0};
  bool integration_initialized_{false};
  bool session_active_{true};
  bool config_valid_{true};
};

const char *toString(OdomUpdateStatus status) noexcept;

}  // namespace smartcar_state_bridge
