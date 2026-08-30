#include "s3_ydlidar_bridge/telemetry_decoder.hpp"

#include "srp_codec.h"
#include "srp_registry.h"
#include "srp_wire.h"

#include <cmath>
#include <cstddef>

namespace s3_ydlidar_bridge {

namespace {
bool finiteFloats(const uint8_t *payload, size_t offset, size_t count) {
  if (payload == nullptr) {
    return false;
  }
  for (size_t index = 0; index < count; ++index) {
    if (!std::isfinite(srp_wire_read_f32_le(
            payload + offset + index * sizeof(float)))) {
      return false;
    }
  }
  return true;
}

void copyFloats(const uint8_t *payload, size_t offset, float *destination,
                size_t count) {
  for (size_t index = 0; index < count; ++index) {
    destination[index] = srp_wire_read_f32_le(
        payload + offset + index * sizeof(float));
  }
}
}  // namespace

TelemetryStatus TelemetryDecoder::decode(
    const std::vector<uint8_t> &encoded_frame, WheelSpeedTelemetry &wheel,
    AttitudeTelemetry &attitude, ImuTelemetry &imu, std::string &error) {
  error.clear();
  srp_frame_t frame{};
  if (encoded_frame.empty() ||
      srp_decode(encoded_frame.data(), encoded_frame.size(), &frame) != 0) {
    ++counters_.invalid;
    error = "invalid SRP frame";
    return TelemetryStatus::kInvalid;
  }

  const uint16_t message_id = frame.type;
  const bool valid_priority =
      message_id == SRP_MSG_ID_ATTITUDE
          ? (frame.priority == SRP_PRIORITY_COMMAND ||
             frame.priority == SRP_PRIORITY_TELEMETRY)
          : frame.priority == SRP_PRIORITY_TELEMETRY;
  if (frame.flags != SRP_FLAG_STREAM_DATA || !valid_priority) {
    ++counters_.invalid;
    error = "SRP telemetry priority or flags mismatch";
    return TelemetryStatus::kInvalid;
  }

  switch (message_id) {
    case SRP_MSG_ID_WHEEL_SPEED_STATUS:
      if (frame.length != SRP_PAYLOAD_WHEEL_SPEED_STATUS_SIZE) {
        ++counters_.invalid;
        error = "invalid SRP WHEEL_SPEED_STATUS payload (type 0x14)";
        return TelemetryStatus::kInvalid;
      }
      if (!finiteFloats(frame.payload, 0U, 4U)) {
        ++counters_.invalid;
        ++counters_.nonfinite;
        error = "non-finite SRP WHEEL_SPEED_STATUS payload (type 0x14)";
        return TelemetryStatus::kInvalid;
      }
      copyFloats(frame.payload, 0U, wheel.mm_per_s.data(), 4U);
      wheel.sequence = frame.sequence;
      wheel.freshness_available = false;
      ++counters_.accepted;
      ++counters_.wheel_speed;
      ++counters_.freshness_unavailable;
      return TelemetryStatus::kWheelSpeed;

    case SRP_MSG_ID_ATTITUDE:
      if (frame.length != SRP_PAYLOAD_DUAL_AHRS_SIZE) {
        ++counters_.invalid;
        error = "invalid SRP ATTITUDE payload (type 0x11)";
        return TelemetryStatus::kInvalid;
      }
      if (frame.payload[0] != SRP_DUAL_AHRS_SCHEMA ||
          frame.payload[2] != 0U || frame.payload[3] != 0U) {
        ++counters_.invalid;
        error = "invalid SRP ATTITUDE schema (type 0x11)";
        return TelemetryStatus::kInvalid;
      }
      if (!finiteFloats(frame.payload, 12U, 17U)) {
        ++counters_.invalid;
        ++counters_.nonfinite;
        error = "non-finite SRP ATTITUDE payload (type 0x11)";
        return TelemetryStatus::kInvalid;
      }
      attitude.flags = frame.payload[1];
      attitude.timestamp_ms = srp_wire_read_u32_le(frame.payload + 4U);
      attitude.sample_sequence = srp_wire_read_u32_le(frame.payload + 8U);
      copyFloats(frame.payload, 12U, attitude.primary_euler.data(), 3U);
      copyFloats(frame.payload, 24U, attitude.primary_quat.data(), 4U);
      copyFloats(frame.payload, 40U, attitude.redundant_euler.data(), 3U);
      copyFloats(frame.payload, 52U, attitude.redundant_quat.data(), 4U);
      copyFloats(frame.payload, 68U, attitude.delta_euler.data(), 3U);
      ++counters_.accepted;
      ++counters_.attitude;
      return TelemetryStatus::kAttitude;

    case SRP_MSG_ID_IMU_TELEMETRY:
      if (frame.length != SRP_PAYLOAD_IMU_TELEMETRY_SIZE) {
        ++counters_.invalid;
        error = "invalid SRP IMU_TELEMETRY payload (type 0x10)";
        return TelemetryStatus::kInvalid;
      }
      if (frame.payload[0] != SRP_IMU_SENSOR_LSM303 &&
          frame.payload[0] != SRP_IMU_SENSOR_BMI323) {
        ++counters_.invalid;
        error = "invalid SRP IMU_TELEMETRY sensor id (type 0x10)";
        return TelemetryStatus::kInvalid;
      }
      if (!finiteFloats(frame.payload, 6U, 6U)) {
        ++counters_.invalid;
        ++counters_.nonfinite;
        error = "non-finite SRP IMU_TELEMETRY payload (type 0x10)";
        return TelemetryStatus::kInvalid;
      }
      imu.sensor_id = frame.payload[0];
      imu.flags = frame.payload[1];
      imu.timestamp = srp_wire_read_u32_le(frame.payload + 2U);
      copyFloats(frame.payload, 6U, imu.accel.data(), 3U);
      copyFloats(frame.payload, 18U, imu.vector.data(), 3U);
      ++counters_.accepted;
      ++counters_.imu;
      return TelemetryStatus::kImu;

    default:
      ++counters_.unsupported;
      error = "unsupported SRP telemetry message id";
      return TelemetryStatus::kUnsupported;
  }
}

}  // namespace s3_ydlidar_bridge
