#include "smartcar_state_bridge/state_adapter.hpp"

#include <gtest/gtest.h>

namespace smartcar_state_bridge {
namespace {

WheelStatusSample sample(uint32_t tick, uint32_t sequence,
                         double speed_mm_s = 1000.0) {
  WheelStatusSample result;
  result.speed_mm_s = {speed_mm_s, speed_mm_s, speed_mm_s, speed_mm_s};
  result.sample_tick = tick;
  result.sample_seq = sequence;
  result.valid = true;
  return result;
}

TelemetryEnvelope offline(uint32_t tick, uint32_t sequence,
                          uint64_t outer_sequence = 0U) {
  return makeOfflineWheelEnvelope(sample(tick, sequence), outer_sequence, 1U);
}

StateAdapterConfig liveConfig() {
  StateAdapterConfig config;
  config.decoder.allow_live = true;
  config.decoder.expected_source_id = 3U;
  config.decoder.expected_destination_id = 8U;
  config.enable_live_odom = true;
  return config;
}

void markLive(TelemetryEnvelope &envelope, uint64_t connection_epoch,
              TelemetryOuterSequenceStatus sequence_status) {
  envelope.origin = TelemetryOrigin::kLiveGateway;
  envelope.connection_epoch = connection_epoch;
  envelope.source_id = 3U;
  envelope.destination_id = 8U;
  envelope.outer_sequence_status = sequence_status;
}

}  // namespace

TEST(StateAdapter, UsesDedicatedFifoAndAcceptsOfflineFixture) {
  StateAdapter adapter;
  auto first = adapter.submitTelemetry(offline(0U, 0U));
  ASSERT_EQ(first.decode_status, TelemetryDecodeStatus::kAccepted);
  EXPECT_EQ(first.odom_update.status, OdomUpdateStatus::kAnchored);

  auto second = adapter.submitTelemetry(offline(100U, 1U));
  EXPECT_EQ(second.odom_update.status, OdomUpdateStatus::kAccepted);
  EXPECT_NEAR(second.odom_update.state.x_m, 0.1, 1e-9);
  EXPECT_EQ(adapter.odom().fifo().size(), 0U);
}

TEST(StateAdapter, LiveRequiresGatewaySequenceClassification) {
  StateAdapterConfig config = liveConfig();
  StateAdapter adapter(config);

  TelemetryEnvelope envelope = makeOfflineWheelEnvelope(sample(0U, 0U));
  markLive(envelope, 1U, TelemetryOuterSequenceStatus::kUnknown);
  const auto result = adapter.submitTelemetry(envelope);
  EXPECT_EQ(result.decode_status, TelemetryDecodeStatus::kSequenceError);
  EXPECT_TRUE(adapter.odom().state().invalid_latched);
}

TEST(StateAdapter, OuterGapStopsIntegrationUntilSessionRestart) {
  StateAdapterConfig config = liveConfig();
  StateAdapter adapter(config);

  TelemetryEnvelope first = makeOfflineWheelEnvelope(sample(0U, 0U));
  markLive(first, 2U, TelemetryOuterSequenceStatus::kFirst);
  EXPECT_EQ(adapter.submitTelemetry(first).odom_update.status,
            OdomUpdateStatus::kAnchored);

  TelemetryEnvelope gap = makeOfflineWheelEnvelope(sample(100U, 1U));
  markLive(gap, 2U, TelemetryOuterSequenceStatus::kJump);
  gap.outer_sequence_gap = 2U;
  const auto rejected = adapter.submitTelemetry(gap);
  EXPECT_EQ(rejected.decode_status, TelemetryDecodeStatus::kSequenceError);
  EXPECT_EQ(rejected.odom_update.status, OdomUpdateStatus::kSequenceGap);

  adapter.beginSession(3U);
  markLive(first, 3U, TelemetryOuterSequenceStatus::kFirst);
  EXPECT_EQ(adapter.submitTelemetry(first).odom_update.status,
            OdomUpdateStatus::kAnchored);
}

TEST(StateAdapter, LiveDecoderFaultLatchesOdomUntilSessionRestart) {
  StateAdapterConfig config = liveConfig();
  StateAdapter adapter(config);

  TelemetryEnvelope first = makeOfflineWheelEnvelope(sample(0U, 0U));
  markLive(first, 11U, TelemetryOuterSequenceStatus::kFirst);
  ASSERT_EQ(adapter.submitTelemetry(first).odom_update.status,
            OdomUpdateStatus::kAnchored);

  TelemetryEnvelope malformed = first;
  malformed.outer_sequence_status = TelemetryOuterSequenceStatus::kInOrder;
  malformed.wheel_status->sample_seq.reset();
  const auto rejected = adapter.submitTelemetry(malformed);
  EXPECT_EQ(rejected.decode_status, TelemetryDecodeStatus::kNotConfigured);
  EXPECT_EQ(rejected.odom_update.status, OdomUpdateStatus::kInvalidSample);
  EXPECT_TRUE(adapter.odom().state().invalid_latched);

  TelemetryEnvelope good = first;
  good.wheel_status->sample_tick = 100U;
  good.wheel_status->sample_seq = 1U;
  EXPECT_EQ(adapter.submitTelemetry(good).odom_update.status,
            OdomUpdateStatus::kLatchedInvalid);

  // Decoder faults are terminal for the current live connection.  Recovery
  // requires an explicit new session and a new connection epoch.
  adapter.beginSession(12U);
  markLive(good, 12U, TelemetryOuterSequenceStatus::kFirst);
  EXPECT_EQ(adapter.submitTelemetry(good).odom_update.status,
            OdomUpdateStatus::kAnchored);
}

}  // namespace smartcar_state_bridge
