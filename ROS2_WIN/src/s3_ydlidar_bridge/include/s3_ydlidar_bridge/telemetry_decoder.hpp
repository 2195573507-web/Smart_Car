#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace s3_ydlidar_bridge {

enum class TelemetryStatus {
  kInvalid,
  kUnsupported,
  kWheelSpeed,
  kAttitude,
  kImu,
};

struct WheelSpeedTelemetry {
  std::array<float, 4> mm_per_s{};
  uint8_t sequence{0};
  // SRP wheel-speed status carries no source sample tick/valid bit.
  bool freshness_available{false};
};

struct AttitudeTelemetry {
  uint8_t flags{0};
  uint32_t timestamp_ms{0};
  uint32_t sample_sequence{0};
  std::array<float, 3> primary_euler{};
  std::array<float, 4> primary_quat{};
  std::array<float, 3> redundant_euler{};
  std::array<float, 4> redundant_quat{};
  std::array<float, 3> delta_euler{};
};

struct ImuTelemetry {
  uint8_t sensor_id{0};
  uint8_t flags{0};
  uint32_t timestamp{0};
  std::array<float, 3> accel{};
  std::array<float, 3> vector{};
};

struct TelemetryCounters {
  uint64_t accepted{0};
  uint64_t invalid{0};
  uint64_t unsupported{0};
  uint64_t wheel_speed{0};
  uint64_t attitude{0};
  uint64_t imu{0};
  uint64_t nonfinite{0};
  uint64_t freshness_unavailable{0};
};

class TelemetryDecoder final {
 public:
  TelemetryStatus decode(const std::vector<uint8_t> &encoded_frame,
                         WheelSpeedTelemetry &wheel,
                         AttitudeTelemetry &attitude,
                         ImuTelemetry &imu,
                         std::string &error);

  TelemetryCounters counters() const noexcept { return counters_; }

 private:
  TelemetryCounters counters_;
};

}  // namespace s3_ydlidar_bridge
