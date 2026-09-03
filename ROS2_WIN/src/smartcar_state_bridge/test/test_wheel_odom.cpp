#include "smartcar_state_bridge/wheel_odom.hpp"

#include <cmath>
#include <cstdint>
#include <limits>

#include <gtest/gtest.h>

namespace smartcar_state_bridge {
namespace {

WheelStatusSample sample(uint32_t tick, uint32_t sequence,
                         double right_mm_s = 1000.0,
                         double left_mm_s = 1000.0,
                         uint64_t connection_epoch = 0U) {
  WheelStatusSample result;
  result.speed_mm_s = {right_mm_s, right_mm_s, left_mm_s, left_mm_s};
  result.sample_tick = tick;
  result.sample_seq = sequence;
  result.connection_epoch = connection_epoch;
  result.valid = true;
  return result;
}

struct RosYawMotionCase {
  const char *name;
  double right_mps;
  double left_mps;
  double linear_mps;
  double angular_rps;
};

class RosYawMotion : public ::testing::TestWithParam<RosYawMotionCase> {};

TEST(WheelKinematics, UsesRrRfLrLfOrderAndMillimetreUnits) {
  WheelKinematicsConfig config;
  config.track_width_m = 0.2;
  WheelKinematics kinematics(config);
  auto input = sample(1U, 1U, 1000.0, 500.0);
  WheelKinematicsResult result;
  std::string error;

  ASSERT_EQ(kinematics.compute(input, result, error),
            KinematicsStatus::kAccepted)
      << error;
  EXPECT_DOUBLE_EQ(result.right_mps, 1.0);
  EXPECT_DOUBLE_EQ(result.left_mps, 0.5);
  EXPECT_DOUBLE_EQ(result.linear_mps, 0.75);
  EXPECT_DOUBLE_EQ(result.angular_rps, 2.5);
}

TEST_P(RosYawMotion, PreservesRosYawSignForPureAndCombinedMotion) {
  // The wheel samples use the frozen RR, RF, LR, LF wire order.  These are
  // expected physical side velocities for a 193 mm differential-drive track.
  WheelKinematicsConfig config;
  config.track_width_m = 0.193;
  WheelKinematics kinematics(config);
  const RosYawMotionCase &motion = GetParam();
  WheelKinematicsResult result;
  std::string error;
  ASSERT_EQ(kinematics.compute(
                sample(1U, 1U, motion.right_mps * 1000.0,
                       motion.left_mps * 1000.0),
                result, error),
            KinematicsStatus::kAccepted)
      << error;
  EXPECT_NEAR(result.right_mps, motion.right_mps, 1.0e-12);
  EXPECT_NEAR(result.left_mps, motion.left_mps, 1.0e-12);
  EXPECT_NEAR(result.linear_mps, motion.linear_mps, 1.0e-12);
  EXPECT_NEAR(result.angular_rps, motion.angular_rps, 1.0e-12);
}

INSTANTIATE_TEST_SUITE_P(
    RosYawSign, RosYawMotion,
    ::testing::Values(
        RosYawMotionCase{"stationary_positive_yaw_left", 0.0965, -0.0965,
                         0.0, 1.0},
        RosYawMotionCase{"stationary_negative_yaw_right", -0.0965, 0.0965,
                         0.0, -1.0},
        RosYawMotionCase{"forward_straight", 0.3, 0.3, 0.3, 0.0},
        RosYawMotionCase{"forward_left", 0.3772, 0.2228, 0.3, 0.8},
        RosYawMotionCase{"forward_right", 0.2228, 0.3772, 0.3, -0.8},
        RosYawMotionCase{"reverse_left", -0.2228, -0.3772, -0.3, 0.8},
        RosYawMotionCase{"reverse_right", -0.3772, -0.2228, -0.3, -0.8}),
    [](const ::testing::TestParamInfo<RosYawMotionCase> &info) {
      return info.param.name;
    });

TEST(WheelKinematics, AppliesOnlyConfiguredWireSigns) {
  WheelKinematicsConfig config;
  // Signs are per wheel in the fixed RR, RF, LR, LF order.
  config.wheel_speed_sign = {-1.0, -1.0, 1.0, 1.0};
  WheelKinematics kinematics(config);
  auto input = sample(1U, 1U, 1000.0, 1000.0);
  WheelKinematicsResult result;
  std::string error;

  ASSERT_EQ(kinematics.compute(input, result, error),
            KinematicsStatus::kAccepted);
  EXPECT_DOUBLE_EQ(result.corrected_speed_mm_s[0], -1000.0);
  EXPECT_DOUBLE_EQ(result.corrected_speed_mm_s[1], -1000.0);
  EXPECT_DOUBLE_EQ(result.right_mps, -1.0);
  EXPECT_DOUBLE_EQ(result.left_mps, 1.0);
}

TEST(WheelKinematics, RejectsInvalidConfigurationAndSamples) {
  WheelKinematicsConfig bad_config;
  bad_config.track_width_m = 0.0;
  WheelKinematics bad_kinematics(bad_config);
  WheelKinematicsResult result;
  std::string error;
  EXPECT_EQ(bad_kinematics.compute(sample(1U, 1U), result, error),
            KinematicsStatus::kInvalidConfig);

  auto invalid = sample(1U, 1U);
  invalid.valid = false;
  WheelKinematics kinematics;
  EXPECT_EQ(kinematics.compute(invalid, result, error),
            KinematicsStatus::kInvalidSample);

  invalid = sample(1U, 1U);
  invalid.speed_mm_s[2] = std::numeric_limits<double>::infinity();
  EXPECT_EQ(kinematics.compute(invalid, result, error),
            KinematicsStatus::kInvalidSample);
}

TEST(WheelKinematics, RejectsNonPositiveOrNonFiniteWheelDiameter) {
  const std::array<double, 4U> invalid_diameters{
      0.0, -0.01, std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity()};
  for (const double diameter : invalid_diameters) {
    WheelKinematicsConfig config;
    config.wheel_diameter_m = diameter;
    WheelKinematics kinematics(config);
    WheelKinematicsResult result;
    std::string error;
    EXPECT_EQ(kinematics.compute(sample(1U, 1U), result, error),
              KinematicsStatus::kInvalidConfig)
        << "diameter=" << diameter;
  }
}

TEST(WheelOdom, RejectsInvalidWheelDiameterConfiguration) {
  const std::array<double, 4U> invalid_diameters{
      0.0, -0.01, std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity()};
  for (const double diameter : invalid_diameters) {
    WheelOdomConfig config;
    config.wheel_diameter_m = diameter;
    WheelOdom odom(config);
    EXPECT_EQ(odom.update(sample(1U, 1U)).status,
              OdomUpdateStatus::kInvalidConfig)
        << "diameter=" << diameter;
  }
}

TEST(WheelKinematics, ConvertsToAngularWheelVelocityUsingDiameter) {
  WheelKinematicsConfig config;
  config.wheel_diameter_m = 0.1;
  WheelKinematics kinematics(config);
  std::array<double, kWheelCount> result{};
  std::string error;

  ASSERT_TRUE(kinematics.wheelAngularVelocityRadPerSec(
      sample(1U, 1U), result, error))
      << error;
  EXPECT_NEAR(result[0], 20.0, 1.0e-12);
}

TEST(WheelSampleFifo, IsBoundedAndCountsInvalidInput) {
  WheelSampleFifo fifo(1U);
  EXPECT_EQ(fifo.push(sample(1U, 1U)), WheelFifoPushStatus::kAccepted);
  EXPECT_EQ(fifo.push(sample(2U, 2U)), WheelFifoPushStatus::kFull);
  EXPECT_TRUE(fifo.overflowed());
  EXPECT_EQ(fifo.overflowCount(), 1U);

  auto invalid = sample(3U, 3U);
  invalid.valid = false;
  EXPECT_EQ(fifo.push(invalid), WheelFifoPushStatus::kInvalid);
  EXPECT_EQ(fifo.invalidCount(), 1U);

  WheelStatusSample popped;
  ASSERT_TRUE(fifo.pop(popped));
  EXPECT_EQ(popped.sample_seq, 1U);
  EXPECT_FALSE(fifo.pop(popped));
}

TEST(WheelOdom, AnchorsThenIntegratesForwardMotion) {
  WheelOdomConfig config;
  config.track_width_m = 0.2;
  config.sample_tick_period_s = 0.001;
  WheelOdom odom(config);

  auto first = odom.update(sample(100U, 10U), 1'000'000'000ULL);
  EXPECT_EQ(first.status, OdomUpdateStatus::kAnchored);
  EXPECT_TRUE(first.state.anchored);
  EXPECT_EQ(first.state.accepted_samples, 0U);

  auto second = odom.update(sample(110U, 11U), 1'010'000'000ULL);
  EXPECT_EQ(second.status, OdomUpdateStatus::kAccepted);
  EXPECT_NEAR(second.dt_s, 0.01, 1.0e-12);
  EXPECT_NEAR(second.state.x_m, 0.01, 1.0e-12);
  EXPECT_NEAR(second.state.y_m, 0.0, 1.0e-12);
  EXPECT_EQ(second.state.accepted_samples, 1U);
}

TEST(WheelOdom, DifferentialArcUsesRightMinusLeftConvention) {
  WheelOdomConfig config;
  config.track_width_m = 0.2;
  config.sample_tick_period_s = 0.1;
  config.stale_timeout_ms = 2000U;
  WheelOdom odom(config);

  ASSERT_EQ(odom.update(sample(0U, 0U, 2000.0, 1000.0)).status,
            OdomUpdateStatus::kAnchored);
  const auto result = odom.update(sample(1U, 1U, 2000.0, 1000.0));
  ASSERT_EQ(result.status, OdomUpdateStatus::kAccepted);
  EXPECT_NEAR(result.state.heading_rad, 0.5, 1.0e-12);
  EXPECT_GT(result.state.x_m, 0.0);
  EXPECT_GT(result.state.y_m, 0.0);
}

TEST(WheelOdom, LinksHumbleDiffDriveOdometryWithPerCycleDisplacements) {
  // Keep this small ABI probe close to the wrapper tests.  Humble's public
  // API names these arguments "velocity", but the implementation consumes
  // left/right metres per update and divides by dt for the reported speed.
  diff_drive_controller::Odometry official(1U);
  official.setWheelParams(0.2, 0.0325, 0.0325);
  official.init(rclcpp::Time(static_cast<int64_t>(0), RCL_SYSTEM_TIME));
  ASSERT_TRUE(official.updateFromVelocity(
      0.01, 0.01,
      rclcpp::Time(static_cast<int64_t>(10'000'000), RCL_SYSTEM_TIME)));
  EXPECT_NEAR(official.getX(), 0.01, 1.0e-12);
  EXPECT_NEAR(official.getY(), 0.0, 1.0e-12);
  EXPECT_NEAR(official.getHeading(), 0.0, 1.0e-12);
  EXPECT_NEAR(official.getLinear(), 1.0, 1.0e-12);
}

TEST(WheelOdom, RejectsDuplicateGapAndOutOfOrderSequences) {
  WheelOdom odom;
  ASSERT_EQ(odom.update(sample(10U, 1U)).status, OdomUpdateStatus::kAnchored);

  auto duplicate = odom.update(sample(11U, 1U));
  EXPECT_EQ(duplicate.status, OdomUpdateStatus::kSequenceDuplicate);
  EXPECT_TRUE(odom.state().invalid_latched);

  odom.beginSession(2U);
  ASSERT_EQ(odom.update(sample(10U, 1U, 1000.0, 1000.0, 2U)).status,
            OdomUpdateStatus::kAnchored);
  auto gap = odom.update(sample(12U, 3U, 1000.0, 1000.0, 2U));
  EXPECT_EQ(gap.status, OdomUpdateStatus::kSequenceGap);

  odom.beginSession(3U);
  ASSERT_EQ(odom.update(sample(10U, 3U, 1000.0, 1000.0, 3U)).status,
            OdomUpdateStatus::kAnchored);
  auto backwards = odom.update(sample(11U, 2U, 1000.0, 1000.0, 3U));
  EXPECT_EQ(backwards.status, OdomUpdateStatus::kSequenceOutOfOrder);
}

TEST(WheelOdom, RejectsStaleAndHostClockRegressionUntilNewSession) {
  WheelOdomConfig config;
  config.stale_timeout_ms = 20U;
  WheelOdom odom(config);
  ASSERT_EQ(odom.update(sample(10U, 1U), 1'000'000'000ULL).status,
            OdomUpdateStatus::kAnchored);

  auto stale = sample(11U, 2U);
  stale.source_age_ms = 21U;
  EXPECT_EQ(odom.update(stale, 1'001'000'000ULL).status,
            OdomUpdateStatus::kStale);
  EXPECT_TRUE(odom.state().invalid_latched);

  auto still_invalid = odom.update(sample(12U, 3U), 1'002'000'000ULL);
  EXPECT_EQ(still_invalid.status, OdomUpdateStatus::kLatchedInvalid);

  odom.beginSession(9U);
  ASSERT_EQ(odom.update(sample(10U, 1U, 1000.0, 1000.0, 9U),
                        2'000'000'000ULL).status,
            OdomUpdateStatus::kAnchored);
  EXPECT_EQ(odom.update(sample(11U, 2U, 1000.0, 1000.0, 9U),
                        1'999'000'000ULL).status,
            OdomUpdateStatus::kHostTimeBackward);
}

TEST(WheelOdom, WatchdogLatchesAfterSourceSilence) {
  WheelOdomConfig config;
  config.stale_timeout_ms = 20U;
  WheelOdom odom(config);
  ASSERT_EQ(odom.update(sample(10U, 1U), 1'000'000'000ULL).status,
            OdomUpdateStatus::kAnchored);

  const auto healthy = odom.checkStale(1'019'000'000ULL);
  EXPECT_EQ(healthy.status, OdomUpdateStatus::kNoSample);
  EXPECT_EQ(odom.lastStatus(), OdomUpdateStatus::kNoSample);
  EXPECT_FALSE(odom.state().invalid_latched);

  const auto stale = odom.checkStale(1'021'000'001ULL);
  EXPECT_EQ(stale.status, OdomUpdateStatus::kStale);
  EXPECT_TRUE(odom.state().invalid_latched);
  EXPECT_EQ(odom.update(sample(11U, 2U), 1'022'000'000ULL).status,
            OdomUpdateStatus::kLatchedInvalid);
}

TEST(WheelOdom, EnforcesFreshnessAndConnectionEpoch) {
  WheelOdom odom;
  auto missing = sample(10U, 1U);
  missing.sample_tick.reset();
  EXPECT_EQ(odom.update(missing).status, OdomUpdateStatus::kMissingFreshness);

  odom.beginSession(4U);
  auto first = sample(10U, 1U);
  first.connection_epoch = 4U;
  ASSERT_EQ(odom.update(first).status, OdomUpdateStatus::kAnchored);
  auto missing_epoch = sample(11U, 2U);
  EXPECT_EQ(odom.update(missing_epoch).status, OdomUpdateStatus::kEpochChanged);

  odom.beginSession(4U);
  first.connection_epoch = 4U;
  ASSERT_EQ(odom.update(first).status, OdomUpdateStatus::kAnchored);
  auto changed = sample(11U, 2U);
  changed.connection_epoch = 5U;
  EXPECT_EQ(odom.update(changed).status, OdomUpdateStatus::kEpochChanged);
}

TEST(WheelOdom, KeepsFullWidthConnectionEpochs) {
  constexpr uint64_t epoch = 0x100000001ULL;
  WheelOdom odom;
  odom.beginSession(epoch);
  auto first = sample(10U, 1U);
  first.connection_epoch = epoch;
  ASSERT_EQ(odom.update(first).status, OdomUpdateStatus::kAnchored);
  EXPECT_EQ(odom.state().session_epoch, epoch);

  auto colliding_low_word = sample(11U, 2U);
  colliding_low_word.connection_epoch = 1U;
  EXPECT_EQ(odom.update(colliding_low_word).status,
            OdomUpdateStatus::kEpochChanged);
}

TEST(WheelOdom, RejectsSourceEpochChangeUntilNewSession) {
  WheelOdom odom;
  auto first = sample(10U, 1U);
  first.source_epoch = 7U;
  ASSERT_EQ(odom.update(first).status, OdomUpdateStatus::kAnchored);

  auto changed = sample(11U, 2U);
  changed.source_epoch = 8U;
  EXPECT_EQ(odom.update(changed).status,
            OdomUpdateStatus::kSourceEpochChanged);
  EXPECT_TRUE(odom.state().invalid_latched);
}

TEST(WheelOdom, RejectsSourceTimeDisappearanceAfterAnchoring) {
  WheelOdom odom;
  auto first = sample(10U, 1U);
  first.source_time_s = 1.0;
  ASSERT_EQ(odom.update(first).status, OdomUpdateStatus::kAnchored);

  auto missing_time = sample(11U, 2U);
  EXPECT_EQ(odom.update(missing_time).status, OdomUpdateStatus::kTimeInvalid);
  EXPECT_TRUE(odom.state().invalid_latched);
}

TEST(WheelOdom, SupportsDefinedCounterWrap) {
  WheelOdom odom;
  ASSERT_EQ(odom.update(sample(std::numeric_limits<uint32_t>::max(),
                               std::numeric_limits<uint32_t>::max()))
                .status,
            OdomUpdateStatus::kAnchored);
  auto wrapped = odom.update(sample(0U, 0U));
  EXPECT_EQ(wrapped.status, OdomUpdateStatus::kAccepted);
  EXPECT_NEAR(wrapped.dt_s, 0.001, 1.0e-12);
}

TEST(WheelOdom, LatchesFifoOverflowAndRequiresSessionRestart) {
  WheelOdomConfig config;
  config.fifo_depth = 1U;
  WheelOdom odom(config);
  EXPECT_EQ(odom.enqueue(sample(1U, 1U)), WheelFifoPushStatus::kAccepted);
  EXPECT_EQ(odom.enqueue(sample(2U, 2U)), WheelFifoPushStatus::kFull);
  EXPECT_TRUE(odom.state().invalid_latched);
  EXPECT_EQ(odom.processNext()->status, OdomUpdateStatus::kLatchedInvalid);

  odom.beginSession(8U);
  EXPECT_FALSE(odom.state().invalid_latched);
  EXPECT_EQ(odom.enqueue(sample(1U, 1U, 1000.0, 1000.0, 8U)),
            WheelFifoPushStatus::kAccepted);
  ASSERT_TRUE(odom.processNext().has_value());
  EXPECT_EQ(odom.lastStatus(), OdomUpdateStatus::kAnchored);
}

}  // namespace
}  // namespace smartcar_state_bridge
