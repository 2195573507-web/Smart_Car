#include "smartcar_state_bridge/telemetry_decoder.hpp"

#include <cmath>
#include <limits>

#include <gtest/gtest.h>

namespace smartcar_state_bridge {
namespace {

WheelStatusSample validSample() {
  WheelStatusSample sample;
  sample.speed_mm_s = {100.0, 200.0, 300.0, 400.0};
  sample.sample_tick = 100U;
  sample.sample_seq = 7U;
  sample.source_time_s = 1.25;
  sample.source_age_ms = 4U;
  sample.valid = true;
  return sample;
}

TEST(TelemetryDecoder, AcceptsReviewedOfflineWheelEnvelope) {
  const auto source = validSample();
  const auto envelope = makeOfflineWheelEnvelope(source, 42U, 9U);

  TelemetryDecoder decoder;
  WheelStatusSample decoded;
  std::string error;
  ASSERT_EQ(decoder.decodeWheelStatus(envelope, decoded, error),
            TelemetryDecodeStatus::kAccepted)
      << error;
  EXPECT_TRUE(error.empty());
  EXPECT_EQ(decoded.speed_mm_s, source.speed_mm_s);
  EXPECT_EQ(decoded.sample_tick, source.sample_tick);
  EXPECT_EQ(decoded.sample_seq, source.sample_seq);
  EXPECT_EQ(decoded.outer_sequence, 42U);
  EXPECT_EQ(decoded.connection_epoch, 9U);
  EXPECT_EQ(decoder.counters().accepted, 1U);
}

TEST(TelemetryDecoder, DoesNotInferLayoutFromRawPayload) {
  TelemetryEnvelope envelope;
  envelope.message_type = kWheelStatusMessageType;
  envelope.payload.resize(16U, 0U);

  TelemetryDecoder decoder;
  WheelStatusSample decoded;
  std::string error;
  EXPECT_EQ(decoder.decodeWheelStatus(envelope, decoded, error),
            TelemetryDecodeStatus::kNotConfigured);
  EXPECT_NE(error.find("layout"), std::string::npos);
  EXPECT_EQ(decoder.counters().not_configured, 1U);
}

TEST(TelemetryDecoder, RejectsLiveInputUnlessExplicitlyEnabled) {
  auto envelope = makeOfflineWheelEnvelope(validSample());
  envelope.origin = TelemetryOrigin::kLiveGateway;

  TelemetryDecoder decoder;
  WheelStatusSample decoded;
  std::string error;
  EXPECT_EQ(decoder.decodeWheelStatus(envelope, decoded, error),
            TelemetryDecodeStatus::kDisabled);
  EXPECT_EQ(decoder.counters().disabled, 1U);
}

TEST(TelemetryDecoder, RequiresConfiguredAndPresentLiveIdentities) {
  auto envelope = makeOfflineWheelEnvelope(validSample());
  envelope.origin = TelemetryOrigin::kLiveGateway;
  envelope.connection_epoch = 1U;

  TelemetryDecoderConfig missing_config;
  missing_config.allow_live = true;
  TelemetryDecoder decoder_without_identities(missing_config);
  WheelStatusSample decoded = validSample();
  std::string error;
  EXPECT_EQ(decoder_without_identities.decodeWheelStatus(envelope, decoded,
                                                         error),
            TelemetryDecodeStatus::kNotConfigured);
  EXPECT_FALSE(decoded.valid);

  TelemetryDecoderConfig config;
  config.allow_live = true;
  config.expected_source_id = 3U;
  config.expected_destination_id = 8U;
  TelemetryDecoder decoder(config);

  decoded = validSample();
  EXPECT_EQ(decoder.decodeWheelStatus(envelope, decoded, error),
            TelemetryDecodeStatus::kIdentityMismatch);
  EXPECT_FALSE(decoded.valid);

  envelope.source_id = 3U;
  decoded = validSample();
  EXPECT_EQ(decoder.decodeWheelStatus(envelope, decoded, error),
            TelemetryDecodeStatus::kIdentityMismatch);
  EXPECT_FALSE(decoded.valid);

  envelope.destination_id = 8U;
  EXPECT_EQ(decoder.decodeWheelStatus(envelope, decoded, error),
            TelemetryDecodeStatus::kAccepted)
      << error;
  EXPECT_TRUE(decoded.valid);
}

TEST(TelemetryDecoder, RequiresLiveConnectionEpoch) {
  TelemetryDecoderConfig config;
  config.allow_live = true;
  config.expected_source_id = 3U;
  config.expected_destination_id = 8U;
  TelemetryDecoder decoder(config);
  auto envelope = makeOfflineWheelEnvelope(validSample());
  envelope.origin = TelemetryOrigin::kLiveGateway;
  envelope.source_id = 3U;
  envelope.destination_id = 8U;

  WheelStatusSample decoded;
  std::string error;
  EXPECT_EQ(decoder.decodeWheelStatus(envelope, decoded, error),
            TelemetryDecodeStatus::kNotConfigured);
  EXPECT_FALSE(decoded.valid);

  envelope.connection_epoch = 1U;
  EXPECT_EQ(decoder.decodeWheelStatus(envelope, decoded, error),
            TelemetryDecodeStatus::kAccepted)
      << error;
}

TEST(TelemetryDecoder, EnforcesFreshnessIdentityAndPayloadBounds) {
  TelemetryDecoderConfig config;
  config.expected_source_id = 3U;
  config.expected_destination_id = 8U;
  config.max_payload_bytes = 4U;
  TelemetryDecoder decoder(config);
  WheelStatusSample decoded;
  std::string error;

  auto envelope = makeOfflineWheelEnvelope(validSample());
  envelope.source_id = 3U;
  envelope.destination_id = 8U;
  envelope.payload.resize(5U, 0U);
  EXPECT_EQ(decoder.decodeWheelStatus(envelope, decoded, error),
            TelemetryDecodeStatus::kMalformed);

  envelope.payload.clear();
  envelope.source_id = 4U;
  EXPECT_EQ(decoder.decodeWheelStatus(envelope, decoded, error),
            TelemetryDecodeStatus::kIdentityMismatch);

  envelope.source_id = 3U;
  auto missing = validSample();
  missing.sample_seq.reset();
  envelope.wheel_status = missing;
  EXPECT_EQ(decoder.decodeWheelStatus(envelope, decoded, error),
            TelemetryDecodeStatus::kNotConfigured);

  const auto counters = decoder.counters();
  EXPECT_EQ(counters.malformed, 1U);
  EXPECT_EQ(counters.identity_errors, 1U);
  EXPECT_EQ(counters.not_configured, 1U);
}

TEST(TelemetryDecoder, RejectsUnsupportedAndInvalidSamples) {
  TelemetryDecoder decoder;
  WheelStatusSample decoded;
  std::string error;
  auto envelope = makeOfflineWheelEnvelope(validSample());

  envelope.message_type = kImuMessageType;
  EXPECT_EQ(decoder.decodeWheelStatus(envelope, decoded, error),
            TelemetryDecodeStatus::kUnsupportedType);

  envelope.message_type = kWheelStatusMessageType;
  auto invalid = validSample();
  invalid.valid = false;
  envelope.wheel_status = invalid;
  EXPECT_EQ(decoder.decodeWheelStatus(envelope, decoded, error),
            TelemetryDecodeStatus::kInvalidSample);

  invalid.valid = true;
  invalid.speed_mm_s[0] = std::numeric_limits<double>::quiet_NaN();
  envelope.wheel_status = invalid;
  EXPECT_EQ(decoder.decodeWheelStatus(envelope, decoded, error),
            TelemetryDecodeStatus::kInvalidSample);

  const auto counters = decoder.counters();
  EXPECT_EQ(counters.unsupported_type, 1U);
  EXPECT_EQ(counters.invalid_samples, 2U);
}

TEST(TelemetryDecoder, AcceptsOnlyReviewedWheelMessageType) {
  TelemetryDecoderConfig config;
  config.wheel_message_type = kImuMessageType;
  TelemetryDecoder decoder(config);
  WheelStatusSample decoded = validSample();
  std::string error;
  const auto envelope = makeOfflineWheelEnvelope(validSample());

  EXPECT_EQ(decoder.decodeWheelStatus(envelope, decoded, error),
            TelemetryDecodeStatus::kNotConfigured);
  EXPECT_FALSE(decoded.valid);
  EXPECT_EQ(decoder.counters().not_configured, 1U);
}

TEST(TelemetryDecoder, ClearsOutputWhenMessageTypeIsRejected) {
  TelemetryDecoder decoder;
  auto envelope = makeOfflineWheelEnvelope(validSample());
  envelope.message_type = kImuMessageType;

  WheelStatusSample decoded = validSample();
  decoded.outer_sequence = 99U;
  decoded.connection_epoch = 12U;
  std::string error;
  EXPECT_EQ(decoder.decodeWheelStatus(envelope, decoded, error),
            TelemetryDecodeStatus::kUnsupportedType);
  EXPECT_EQ(decoded.speed_mm_s, (std::array<double, 4U>{0.0, 0.0, 0.0, 0.0}));
  EXPECT_FALSE(decoded.sample_tick.has_value());
  EXPECT_FALSE(decoded.sample_seq.has_value());
  EXPECT_FALSE(decoded.source_time_s.has_value());
  EXPECT_FALSE(decoded.source_age_ms.has_value());
  EXPECT_FALSE(decoded.source_epoch.has_value());
  EXPECT_FALSE(decoded.valid);
  EXPECT_EQ(decoded.outer_sequence, 0U);
  EXPECT_EQ(decoded.connection_epoch, 0U);
}

TEST(TelemetryDecoder, LiveCanBeEnabledWithoutChangingOfflinePolicy) {
  TelemetryDecoderConfig config;
  config.allow_live = true;
  config.expected_source_id = 3U;
  config.expected_destination_id = 8U;
  TelemetryDecoder decoder(config);
  auto envelope = makeOfflineWheelEnvelope(validSample(), 12U, 4U);
  envelope.origin = TelemetryOrigin::kLiveGateway;
  envelope.source_id = 3U;
  envelope.destination_id = 8U;
  envelope.connection_epoch = 4U;

  WheelStatusSample decoded;
  std::string error;
  EXPECT_EQ(decoder.decodeWheelStatus(envelope, decoded, error),
            TelemetryDecodeStatus::kAccepted);
  EXPECT_EQ(decoded.outer_sequence, 12U);
}

}  // namespace
}  // namespace smartcar_state_bridge
