#include "chassis_test_utils.hpp"

#include "smartcar_state_bridge/chassis_state.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace {

using namespace smartcar_state_bridge;

constexpr double kPi = 3.14159265358979323846;

SrpV4ChassisSample sample(uint32_t timestamp_ms, double x_m, double y_m,
                          double yaw_deg, uint8_t sequence) {
  SrpV4ChassisSample result;
  result.schema = 1U;
  result.status_flags = kSrpV4OdometryValid;
  result.sequence = sequence;
  result.timestamp_ms = timestamp_ms;
  result.x_m = x_m;
  result.y_m = y_m;
  result.yaw_rad = yaw_deg * kPi / 180.0;
  result.total_dist_m = std::hypot(x_m, y_m);
  return result;
}

ChassisTelemetryFrame liveFrame(std::vector<uint8_t> payload,
                                uint64_t epoch, uint64_t receive_ns,
                                TelemetryOuterSequenceStatus sequence_status) {
  ChassisTelemetryFrame frame;
  frame.origin = TelemetryOrigin::kLiveGateway;
  frame.payload = std::move(payload);
  frame.connection_epoch = epoch;
  frame.host_received_steady_ns = receive_ns;
  frame.outer_sequence_status = sequence_status;
  return frame;
}

TEST(ChassisOdom, FirstFrameAnchorsWithoutVelocitySpike) {
  ChassisOdomTracker tracker;
  tracker.beginSession(7U);
  const auto update = tracker.update(sample(1000U, 4.0, -2.0, 90.0, 10U), 7U,
                                     1000000000U);

  ASSERT_EQ(update.status, ChassisOdomStatus::kAnchored);
  EXPECT_DOUBLE_EQ(update.state.x_m, 4.0);
  EXPECT_DOUBLE_EQ(update.state.y_m, -2.0);
  EXPECT_DOUBLE_EQ(update.state.linear_x_mps, 0.0);
  EXPECT_DOUBLE_EQ(update.state.linear_y_mps, 0.0);
  EXPECT_DOUBLE_EQ(update.state.angular_z_rps, 0.0);
  EXPECT_EQ(update.state.accepted_updates, 0U);
}

TEST(ChassisOdom, DerivesBodyFrameTwistFromAuthoritativePose) {
  ChassisOdomTracker tracker;
  tracker.beginSession(8U);
  ASSERT_EQ(tracker.update(sample(1000U, 5.0, 2.0, 90.0, 10U), 8U,
                           1000000000U)
                .status,
            ChassisOdomStatus::kAnchored);
  const auto update = tracker.update(sample(1100U, 5.0, 2.1, 90.0, 20U), 8U,
                                     1100000000U);

  ASSERT_EQ(update.status, ChassisOdomStatus::kAccepted);
  EXPECT_NEAR(update.state.linear_x_mps, 1.0, 1.0e-12);
  EXPECT_NEAR(update.state.linear_y_mps, 0.0, 1.0e-12);
  EXPECT_NEAR(update.state.angular_z_rps, 0.0, 1.0e-12);
  EXPECT_DOUBLE_EQ(update.state.x_m, 5.0);
  EXPECT_DOUBLE_EQ(update.state.y_m, 2.1);
}

TEST(ChassisOdom, UsesShortestYawDeltaAcrossPlusMinus180) {
  ChassisOdomTracker tracker;
  tracker.beginSession(9U);
  ASSERT_EQ(tracker.update(sample(2000U, 0.0, 0.0, 179.0, 10U), 9U,
                           2000000000U)
                .status,
            ChassisOdomStatus::kAnchored);
  const auto update = tracker.update(sample(2100U, 0.0, 0.0, -179.0, 20U), 9U,
                                     2100000000U);

  ASSERT_EQ(update.status, ChassisOdomStatus::kAccepted);
  EXPECT_NEAR(update.state.angular_z_rps, (2.0 * kPi / 180.0) / 0.1,
              1.0e-12);
}

TEST(ChassisOdom, RejectsDuplicateRollbackUnreasonableDtAndStaleTimestamp) {
  ChassisOdomTracker tracker;
  tracker.beginSession(1U);
  ASSERT_EQ(tracker.update(sample(100U, 0.0, 0.0, 0.0, 10U), 1U, 100U).status,
            ChassisOdomStatus::kAnchored);
  EXPECT_EQ(tracker.update(sample(100U, 0.0, 0.0, 0.0, 11U), 1U, 101U).status,
            ChassisOdomStatus::kTimestampDuplicate);
  EXPECT_EQ(tracker.update(sample(90U, 0.0, 0.0, 0.0, 12U), 1U, 102U).status,
            ChassisOdomStatus::kAnchored);
  EXPECT_EQ(tracker.update(sample(190U, 0.0, 0.0, 0.0, 13U), 1U, 103U).status,
            ChassisOdomStatus::kAccepted);
  EXPECT_EQ(tracker.update(sample(180U, 0.0, 0.0, 0.0, 14U), 1U, 104U).status,
            ChassisOdomStatus::kTimestampRollback);
  EXPECT_EQ(tracker.update(sample(200U, 0.0, 0.0, 0.0, 15U), 1U, 105U).status,
            ChassisOdomStatus::kAnchored);

  ChassisOdomConfig small_dt_config;
  small_dt_config.min_dt_s = 0.010;
  ChassisOdomTracker small_dt(small_dt_config);
  small_dt.beginSession(2U);
  ASSERT_EQ(small_dt.update(sample(100U, 0.0, 0.0, 0.0, 10U), 2U, 100U).status,
            ChassisOdomStatus::kAnchored);
  EXPECT_EQ(small_dt.update(sample(101U, 0.0, 0.0, 0.0, 11U), 2U, 101U).status,
            ChassisOdomStatus::kDtInvalid);

  ChassisOdomTracker stale;
  stale.beginSession(3U);
  ASSERT_EQ(stale.update(sample(100U, 0.0, 0.0, 0.0, 10U), 3U, 100U).status,
            ChassisOdomStatus::kAnchored);
  EXPECT_EQ(stale.update(sample(601U, 0.0, 0.0, 0.0, 11U), 3U, 101U).status,
            ChassisOdomStatus::kStale);
}

TEST(ChassisOdom, AcceptsForwardNonContiguousInnerSequence) {
  ChassisOdomTracker tracker;
  tracker.beginSession(4U);
  EXPECT_EQ(tracker.update(sample(1000U, 0.0, 0.0, 0.0, 10U), 4U,
                           1000000000U)
                .status,
            ChassisOdomStatus::kAnchored);
  EXPECT_EQ(tracker.update(sample(1100U, 0.1, 0.0, 0.0, 25U), 4U,
                           1100000000U)
                .status,
            ChassisOdomStatus::kAccepted);
}

TEST(ChassisOdom, RejectsInnerSequenceDuplicateAndRollback) {
  ChassisOdomTracker tracker;
  tracker.beginSession(5U);
  ASSERT_EQ(tracker.update(sample(1000U, 0.0, 0.0, 0.0, 40U), 5U,
                           1000000000U)
                .status,
            ChassisOdomStatus::kAnchored);
  EXPECT_EQ(tracker.update(sample(1100U, 0.1, 0.0, 0.0, 40U), 5U,
                           1100000000U)
                .status,
            ChassisOdomStatus::kSequenceDuplicate);
  EXPECT_FALSE(tracker.state().baseline_ready);
  ASSERT_EQ(tracker.update(sample(1200U, 0.2, 0.0, 0.0, 41U), 5U,
                           1200000000U)
                .status,
            ChassisOdomStatus::kAnchored);
  ASSERT_EQ(tracker.update(sample(1300U, 0.3, 0.0, 0.0, 42U), 5U,
                           1300000000U)
                .status,
            ChassisOdomStatus::kAccepted);
  EXPECT_EQ(tracker.update(sample(1400U, 0.4, 0.0, 0.0, 40U), 5U,
                           1400000000U)
                .status,
            ChassisOdomStatus::kSequenceRollback);
  EXPECT_FALSE(tracker.state().baseline_ready);
}

TEST(ChassisOdom, ResetsBaselineForEpochDisconnectAndHostStale) {
  ChassisOdomTracker tracker;
  tracker.beginSession(1U);
  ASSERT_EQ(tracker.update(sample(1000U, 0.0, 0.0, 0.0, 10U), 1U,
                           1000000000U)
                .status,
            ChassisOdomStatus::kAnchored);
  ASSERT_EQ(tracker.update(sample(1100U, 0.1, 0.0, 0.0, 20U), 1U,
                           1100000000U)
                .status,
            ChassisOdomStatus::kAccepted);

  tracker.beginSession(2U);
  EXPECT_EQ(tracker.update(sample(10U, 9.0, 9.0, 45.0, 1U), 2U,
                           1200000000U)
                .status,
            ChassisOdomStatus::kAnchored);
  tracker.endSession();
  EXPECT_EQ(tracker.update(sample(20U, 10.0, 10.0, 45.0, 2U), 2U,
                           1300000000U)
                .status,
            ChassisOdomStatus::kSessionClosed);

  tracker.beginSession(3U);
  ASSERT_EQ(tracker.update(sample(100U, 0.0, 0.0, 0.0, 10U), 3U,
                           2000000000U)
                .status,
            ChassisOdomStatus::kAnchored);
  EXPECT_EQ(tracker.checkStale(2500000001U).status,
            ChassisOdomStatus::kStale);
  EXPECT_EQ(tracker.update(sample(700U, 1.0, 0.0, 0.0, 20U), 3U,
                           2500000002U)
                .status,
            ChassisOdomStatus::kAnchored);

  tracker.beginSession(4U);
  ASSERT_EQ(tracker.update(sample(100U, 0.0, 0.0, 0.0, 10U), 4U,
                           3000000000U)
                .status,
            ChassisOdomStatus::kAnchored);
  EXPECT_EQ(tracker.update(sample(200U, 0.1, 0.0, 0.0, 20U), 4U,
                           3500000001U)
                .status,
            ChassisOdomStatus::kStale);
  EXPECT_EQ(tracker.update(sample(300U, 0.2, 0.0, 0.0, 30U), 4U,
                           3500000002U)
                .status,
            ChassisOdomStatus::kAnchored);
}

TEST(ChassisStateAdapter, DefaultsRejectLiveTelemetryAndOdom) {
  ChassisStateAdapter adapter;
  adapter.beginSession(1U);
  const auto result = adapter.submit(liveFrame(
      test::goldenFrame(), 1U, 1000U, TelemetryOuterSequenceStatus::kFirst));
  EXPECT_EQ(result.status, ChassisSubmitStatus::kDisabled);
  EXPECT_EQ(result.decode_status, SrpV4DecodeStatus::kAccepted);
  EXPECT_FALSE(adapter.config().allow_live_telemetry);
  EXPECT_FALSE(adapter.config().enable_live_odom);
}

TEST(ChassisStateAdapter, InvalidFrameAndOdomValidClearRequireNewBaseline) {
  ChassisStateAdapterConfig config;
  config.allow_live_telemetry = true;
  config.enable_live_odom = true;
  ChassisStateAdapter adapter(config);
  adapter.beginSession(4U);

  auto first = liveFrame(test::makeFrame(1000U, 0.0F, 0.0F, 0.0F, 0.0F,
                                         0x04U, 10U),
                         4U, 1000000000U,
                         TelemetryOuterSequenceStatus::kFirst);
  EXPECT_EQ(adapter.submit(first).status, ChassisSubmitStatus::kAnchored);

  auto invalid = liveFrame(
      test::makeFrame(1100U, 100.0F, 0.0F, 0.0F, 0.1F, 0x00U, 11U), 4U,
      1100000000U, TelemetryOuterSequenceStatus::kInOrder);
  EXPECT_EQ(adapter.submit(invalid).status,
            ChassisSubmitStatus::kDecodeRejected);

  auto recovered = liveFrame(
      test::makeFrame(1200U, 200.0F, 0.0F, 0.0F, 0.2F, 0x04U, 12U), 4U,
      1200000000U, TelemetryOuterSequenceStatus::kInOrder);
  EXPECT_EQ(adapter.submit(recovered).status, ChassisSubmitStatus::kAnchored);

  auto bad_crc =
      test::makeFrame(1300U, 300.0F, 0.0F, 0.0F, 0.3F, 0x04U, 13U);
  bad_crc[32U] ^= 0x01U;
  EXPECT_EQ(adapter.submit(liveFrame(std::move(bad_crc), 4U, 1300000000U,
                                    TelemetryOuterSequenceStatus::kInOrder))
                .status,
            ChassisSubmitStatus::kDecodeRejected);
  EXPECT_EQ(adapter.submit(liveFrame(
                test::makeFrame(1400U, 400.0F, 0.0F, 0.0F, 0.4F, 0x04U,
                                14U), 4U,
                1400000000U, TelemetryOuterSequenceStatus::kInOrder))
                .status,
            ChassisSubmitStatus::kAnchored);
}

TEST(ChassisStateAdapter, DisconnectStopsUpdatesAndNewEpochReanchors) {
  ChassisStateAdapterConfig config;
  config.allow_live_telemetry = true;
  config.enable_live_odom = true;
  ChassisStateAdapter adapter(config);
  adapter.beginSession(10U);
  EXPECT_EQ(adapter.submit(liveFrame(
                test::makeFrame(100U, 0.0F, 0.0F, 0.0F, 0.0F, 0x04U,
                                10U), 10U,
                1000000000U, TelemetryOuterSequenceStatus::kFirst))
                .status,
            ChassisSubmitStatus::kAnchored);
  EXPECT_EQ(adapter.submit(liveFrame(
                test::makeFrame(200U, 100.0F, 0.0F, 0.0F, 0.1F, 0x04U,
                                20U), 10U,
                1100000000U, TelemetryOuterSequenceStatus::kInOrder))
                .status,
            ChassisSubmitStatus::kAccepted);

  adapter.endSession();
  EXPECT_EQ(adapter.submit(liveFrame(
                test::makeFrame(300U, 200.0F, 0.0F, 0.0F, 0.2F, 0x04U,
                                30U), 10U,
                1200000000U, TelemetryOuterSequenceStatus::kInOrder))
                .status,
            ChassisSubmitStatus::kOdomRejected);

  adapter.beginSession(11U);
  EXPECT_EQ(adapter.submit(liveFrame(
                test::makeFrame(1U, 9.0F, 9.0F, 5.0F, 1.0F, 0x04U, 1U), 11U,
                1300000000U, TelemetryOuterSequenceStatus::kFirst))
                .status,
            ChassisSubmitStatus::kAnchored);
}

TEST(SrpV4TelemetryAdapter, RoutesInterleavedTypesWithoutResettingChassis) {
  ChassisStateAdapterConfig config;
  config.allow_live_telemetry = true;
  config.enable_live_odom = true;
  SrpV4TelemetryAdapter adapter(config);
  adapter.beginSession(20U);

  EXPECT_EQ(adapter.submit(liveFrame(
                test::makeSrpFrame(kSrpV4ImuMessageId, 10U, 30U), 20U,
                1000000000U, TelemetryOuterSequenceStatus::kFirst))
                .status,
            SrpV4TelemetrySubmitStatus::kIgnored);
  EXPECT_EQ(adapter.submit(liveFrame(
                test::makeSrpFrame(kSrpV4WheelMessageId, 11U, 16U), 20U,
                1010000000U, TelemetryOuterSequenceStatus::kInOrder))
                .status,
            SrpV4TelemetrySubmitStatus::kIgnored);

  auto first_chassis = adapter.submit(liveFrame(
      test::makeFrame(1000U, 0.0F, 0.0F, 0.0F, 0.0F, 0x04U, 12U), 20U,
      1020000000U, TelemetryOuterSequenceStatus::kInOrder));
  ASSERT_EQ(first_chassis.status, SrpV4TelemetrySubmitStatus::kChassis);
  EXPECT_EQ(first_chassis.chassis_result.status,
            ChassisSubmitStatus::kAnchored);

  const auto reject_snapshot = adapter.chassis().counters();
  EXPECT_EQ(adapter.submit(liveFrame(
                test::makeSrpFrame(kSrpV4ImuMessageId, 13U, 30U), 20U,
                1030000000U, TelemetryOuterSequenceStatus::kInOrder))
                .status,
            SrpV4TelemetrySubmitStatus::kIgnored);
  EXPECT_TRUE(adapter.chassis().odom().state().baseline_ready);
  EXPECT_EQ(adapter.chassis().counters().decode_rejected,
            reject_snapshot.decode_rejected);
  EXPECT_EQ(adapter.chassis().counters().sequence_rejected,
            reject_snapshot.sequence_rejected);
  EXPECT_EQ(adapter.chassis().counters().odom_rejected,
            reject_snapshot.odom_rejected);

  auto second_chassis = adapter.submit(liveFrame(
      test::makeFrame(1100U, 100.0F, 0.0F, 0.0F, 0.1F, 0x04U, 20U), 20U,
      1120000000U, TelemetryOuterSequenceStatus::kInOrder));
  ASSERT_EQ(second_chassis.status, SrpV4TelemetrySubmitStatus::kChassis);
  EXPECT_EQ(second_chassis.chassis_result.status,
            ChassisSubmitStatus::kAccepted);

  const auto counters = adapter.counters();
  EXPECT_EQ(counters.received, 5U);
  EXPECT_EQ(counters.imu_frames, 2U);
  EXPECT_EQ(counters.wheel_frames, 1U);
  EXPECT_EQ(counters.chassis_frames, 2U);
  EXPECT_EQ(counters.frame_decode_rejected, 0U);
}

TEST(SrpV4TelemetryAdapter,
     NonChassisOuterSequenceRejectDoesNotClearChassisBaseline) {
  ChassisStateAdapterConfig config;
  config.allow_live_telemetry = true;
  config.enable_live_odom = true;
  SrpV4TelemetryAdapter adapter(config);
  adapter.beginSession(23U);

  ASSERT_EQ(adapter.submit(liveFrame(
                test::makeFrame(1000U, 0.0F, 0.0F, 0.0F, 0.0F, 0x04U,
                                10U),
                23U, 1000000000U, TelemetryOuterSequenceStatus::kFirst))
                .chassis_result.status,
            ChassisSubmitStatus::kAnchored);

  const auto before = adapter.chassis().counters();
  const auto rejected = adapter.submit(liveFrame(
      test::makeSrpFrame(kSrpV4ImuMessageId, 11U, 30U), 23U, 1010000000U,
      TelemetryOuterSequenceStatus::kDuplicate));
  EXPECT_EQ(rejected.status,
            SrpV4TelemetrySubmitStatus::kOuterSequenceRejected);
  EXPECT_TRUE(adapter.chassis().odom().state().baseline_ready);
  EXPECT_EQ(adapter.chassis().counters().decode_rejected,
            before.decode_rejected);
  EXPECT_EQ(adapter.chassis().counters().sequence_rejected,
            before.sequence_rejected);
  EXPECT_EQ(adapter.chassis().counters().odom_rejected,
            before.odom_rejected);

  EXPECT_EQ(adapter.submit(liveFrame(
                test::makeFrame(1100U, 0.1F, 0.0F, 0.0F, 0.1F, 0x04U,
                                20U),
                23U, 1020000000U,
                TelemetryOuterSequenceStatus::kInOrder))
                .chassis_result.status,
            ChassisSubmitStatus::kAccepted);
}

TEST(SrpV4TelemetryAdapter, WrongSizeChassisRejectsAndClearsBaseline) {
  ChassisStateAdapterConfig config;
  config.allow_live_telemetry = true;
  config.enable_live_odom = true;
  SrpV4TelemetryAdapter adapter(config);
  adapter.beginSession(21U);

  ASSERT_EQ(adapter.submit(liveFrame(
                test::makeFrame(1000U, 0.0F, 0.0F, 0.0F, 0.0F, 0x04U,
                                10U), 21U, 1000000000U,
                TelemetryOuterSequenceStatus::kFirst))
                .chassis_result.status,
            ChassisSubmitStatus::kAnchored);
  auto wrong_size =
      test::makeFrame(1100U, 100.0F, 0.0F, 0.0F, 0.1F, 0x04U, 11U);
  wrong_size.pop_back();
  const auto rejected = adapter.submit(liveFrame(
      std::move(wrong_size), 21U, 1100000000U,
      TelemetryOuterSequenceStatus::kInOrder));

  EXPECT_EQ(rejected.status,
            SrpV4TelemetrySubmitStatus::kFrameDecodeRejected);
  ASSERT_TRUE(rejected.message_id.has_value());
  EXPECT_EQ(*rejected.message_id, kSrpV4ChassisMessageId);
  EXPECT_EQ(rejected.chassis_result.status,
            ChassisSubmitStatus::kDecodeRejected);
  EXPECT_EQ(rejected.chassis_result.decode_status,
            SrpV4DecodeStatus::kSizeError);
  EXPECT_FALSE(adapter.chassis().odom().state().baseline_ready);
  EXPECT_EQ(adapter.chassis().counters().decode_rejected, 1U);
}

TEST(ChassisStateAdapter, CountsInnerDuplicateAndRollbackAsSequenceRejects) {
  ChassisStateAdapterConfig config;
  config.allow_live_telemetry = true;
  config.enable_live_odom = true;
  ChassisStateAdapter adapter(config);
  adapter.beginSession(22U);

  ASSERT_EQ(adapter.submit(liveFrame(
                test::makeFrame(1000U, 0.0F, 0.0F, 0.0F, 0.0F, 0x04U,
                                50U), 22U, 1000000000U,
                TelemetryOuterSequenceStatus::kFirst))
                .status,
            ChassisSubmitStatus::kAnchored);
  EXPECT_EQ(adapter.submit(liveFrame(
                test::makeFrame(1100U, 100.0F, 0.0F, 0.0F, 0.1F, 0x04U,
                                50U), 22U, 1100000000U,
                TelemetryOuterSequenceStatus::kInOrder))
                .status,
            ChassisSubmitStatus::kSequenceRejected);
  EXPECT_FALSE(adapter.odom().state().baseline_ready);

  ASSERT_EQ(adapter.submit(liveFrame(
                test::makeFrame(1200U, 200.0F, 0.0F, 0.0F, 0.2F, 0x04U,
                                51U), 22U, 1200000000U,
                TelemetryOuterSequenceStatus::kInOrder))
                .status,
            ChassisSubmitStatus::kAnchored);
  EXPECT_EQ(adapter.submit(liveFrame(
                test::makeFrame(1300U, 300.0F, 0.0F, 0.0F, 0.3F, 0x04U,
                                50U), 22U, 1300000000U,
                TelemetryOuterSequenceStatus::kInOrder))
                .status,
            ChassisSubmitStatus::kSequenceRejected);
  EXPECT_FALSE(adapter.odom().state().baseline_ready);
  EXPECT_EQ(adapter.counters().sequence_rejected, 2U);
  EXPECT_EQ(adapter.counters().decode_rejected, 0U);
}

}  // namespace
