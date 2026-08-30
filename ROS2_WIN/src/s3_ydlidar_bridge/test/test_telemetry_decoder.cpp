#include "s3_ydlidar_bridge/telemetry_decoder.hpp"

#include "srp_codec.h"
#include "srp_registry.h"
#include "srp_wire.h"

#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace {
std::vector<uint8_t> encodeFrame(uint16_t message_id,
                                 const std::vector<uint8_t> &payload,
                                 uint8_t sequence = 7U,
                                 uint8_t priority = SRP_PRIORITY_TELEMETRY,
                                 uint8_t flags = SRP_FLAG_STREAM_DATA) {
  srp_frame_t frame{};
  frame.priority = priority;
  frame.type = static_cast<uint8_t>(message_id);
  frame.sequence = sequence;
  frame.flags = flags;
  frame.length = static_cast<uint16_t>(payload.size());
  frame.payload = payload.data();
  std::array<uint8_t, SRP_MAX_FRAME_SIZE> encoded{};
  uint16_t encoded_length = 0U;
  EXPECT_EQ(srp_encode(&frame, encoded.data(), encoded.size(),
                            &encoded_length),
            0);
  return std::vector<uint8_t>(encoded.begin(),
                              encoded.begin() + encoded_length);
}

std::vector<uint8_t> wheelPayload(const std::array<float, 4> &values) {
  std::vector<uint8_t> payload(SRP_PAYLOAD_WHEEL_SPEED_STATUS_SIZE);
  srp_wire_write_f32_array_le(payload.data(), values.data(), values.size());
  return payload;
}

std::vector<uint8_t> attitudePayload() {
  std::vector<uint8_t> payload(SRP_PAYLOAD_DUAL_AHRS_SIZE, 0U);
  payload[0] = SRP_DUAL_AHRS_SCHEMA;
  payload[1] = 0x03U;
  srp_wire_write_u32_le(payload.data() + 4U, 123456U);
  srp_wire_write_u32_le(payload.data() + 8U, 42U);
  for (size_t index = 0U; index < 17U; ++index) {
    srp_wire_write_f32_le(payload.data() + 12U + index * sizeof(float),
                           static_cast<float>(index) * 0.1F);
  }
  return payload;
}

std::vector<uint8_t> imuPayload(uint8_t sensor_id) {
  std::vector<uint8_t> payload(SRP_PAYLOAD_IMU_TELEMETRY_SIZE, 0U);
  payload[0] = sensor_id;
  payload[1] = SRP_IMU_TELEMETRY_FLAG_ONLINE;
  srp_wire_write_u32_le(payload.data() + 2U, 9876U);
  for (size_t index = 0U; index < 6U; ++index) {
    srp_wire_write_f32_le(payload.data() + 6U + index * sizeof(float),
                           static_cast<float>(index + 1U));
  }
  return payload;
}
}  // namespace

TEST(TelemetryDecoder, DecodesSrpWheelSpeedAndReportsMissingFreshness) {
  s3_ydlidar_bridge::TelemetryDecoder decoder;
  s3_ydlidar_bridge::WheelSpeedTelemetry wheel;
  s3_ydlidar_bridge::AttitudeTelemetry attitude;
  s3_ydlidar_bridge::ImuTelemetry imu;
  std::string error;
  const auto encoded = encodeFrame(
      SRP_MSG_ID_WHEEL_SPEED_STATUS,
      wheelPayload(std::array<float, 4>{120.0F, 121.0F, -80.0F, -81.0F}));

  EXPECT_EQ(decoder.decode(encoded, wheel, attitude, imu, error),
            s3_ydlidar_bridge::TelemetryStatus::kWheelSpeed);
  EXPECT_TRUE(error.empty());
  EXPECT_FLOAT_EQ(wheel.mm_per_s[0], 120.0F);
  EXPECT_FLOAT_EQ(wheel.mm_per_s[3], -81.0F);
  EXPECT_EQ(wheel.sequence, 7U);
  EXPECT_FALSE(wheel.freshness_available);
  EXPECT_EQ(decoder.counters().freshness_unavailable, 1U);
}

TEST(TelemetryDecoder, DecodesAttitudeAndBothImuSensors) {
  s3_ydlidar_bridge::TelemetryDecoder decoder;
  s3_ydlidar_bridge::WheelSpeedTelemetry wheel;
  s3_ydlidar_bridge::AttitudeTelemetry attitude;
  s3_ydlidar_bridge::ImuTelemetry imu;
  std::string error;

  auto encoded = encodeFrame(SRP_MSG_ID_ATTITUDE, attitudePayload(), 8U);
  EXPECT_EQ(decoder.decode(encoded, wheel, attitude, imu, error),
            s3_ydlidar_bridge::TelemetryStatus::kAttitude);
  EXPECT_EQ(attitude.timestamp_ms, 123456U);
  EXPECT_EQ(attitude.sample_sequence, 42U);
  EXPECT_FLOAT_EQ(attitude.primary_quat[2], 0.3F);

  for (const uint8_t sensor : {SRP_IMU_SENSOR_LSM303,
                               SRP_IMU_SENSOR_BMI323}) {
    encoded = encodeFrame(SRP_MSG_ID_IMU_TELEMETRY, imuPayload(sensor),
                          static_cast<uint8_t>(9U + sensor));
    EXPECT_EQ(decoder.decode(encoded, wheel, attitude, imu, error),
              s3_ydlidar_bridge::TelemetryStatus::kImu);
    EXPECT_EQ(imu.sensor_id, sensor);
    EXPECT_EQ(imu.timestamp, 9876U);
    EXPECT_FLOAT_EQ(imu.vector[2], 6.0F);
  }
  EXPECT_EQ(decoder.counters().attitude, 1U);
  EXPECT_EQ(decoder.counters().imu, 2U);
}

TEST(TelemetryDecoder, RejectsBadPriorityFlagsAndNonFinitePayload) {
  s3_ydlidar_bridge::TelemetryDecoder decoder;
  s3_ydlidar_bridge::WheelSpeedTelemetry wheel;
  s3_ydlidar_bridge::AttitudeTelemetry attitude;
  s3_ydlidar_bridge::ImuTelemetry imu;
  std::string error;
  const auto payload = wheelPayload(std::array<float, 4>{1.0F, 2.0F, 3.0F, 4.0F});

  auto encoded = encodeFrame(SRP_MSG_ID_WHEEL_SPEED_STATUS, payload, 1U,
                             SRP_PRIORITY_EMERGENCY);
  EXPECT_EQ(decoder.decode(encoded, wheel, attitude, imu, error),
            s3_ydlidar_bridge::TelemetryStatus::kInvalid);

  encoded = encodeFrame(SRP_MSG_ID_WHEEL_SPEED_STATUS, payload, 2U,
                        SRP_PRIORITY_TELEMETRY,
                        SRP_FLAG_ACK_REQUIRED);
  EXPECT_EQ(decoder.decode(encoded, wheel, attitude, imu, error),
            s3_ydlidar_bridge::TelemetryStatus::kInvalid);

  auto nonfinite = payload;
  srp_wire_write_f32_le(nonfinite.data() + 4U,
                         std::numeric_limits<float>::quiet_NaN());
  encoded = encodeFrame(SRP_MSG_ID_WHEEL_SPEED_STATUS, nonfinite, 3U);
  EXPECT_EQ(decoder.decode(encoded, wheel, attitude, imu, error),
            s3_ydlidar_bridge::TelemetryStatus::kInvalid);
  EXPECT_EQ(decoder.counters().invalid, 3U);
  EXPECT_EQ(decoder.counters().nonfinite, 1U);
}

TEST(TelemetryDecoder, ReportsUnsupportedMessageWithoutTreatingItAsWheelData) {
  s3_ydlidar_bridge::TelemetryDecoder decoder;
  s3_ydlidar_bridge::WheelSpeedTelemetry wheel;
  s3_ydlidar_bridge::AttitudeTelemetry attitude;
  s3_ydlidar_bridge::ImuTelemetry imu;
  std::string error;
  std::vector<uint8_t> payload{0x01U};
  const auto encoded = encodeFrame(SRP_MSG_ID_POWER_STATUS, payload);

  EXPECT_EQ(decoder.decode(encoded, wheel, attitude, imu, error),
            s3_ydlidar_bridge::TelemetryStatus::kUnsupported);
  EXPECT_EQ(decoder.counters().unsupported, 1U);
}
