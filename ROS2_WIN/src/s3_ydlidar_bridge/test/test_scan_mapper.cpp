#include "s3_ydlidar_bridge/scan_mapper.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace {
::node_info nodeAt(uint16_t degrees, uint16_t distance_q2 = 4000U,
                   uint8_t quality = 0U) {
  ::node_info node{};
  node.angle_q6_checkbit =
      static_cast<uint16_t>((degrees * 64U) << 1U | 1U);
  node.distance_q2 = distance_q2;
  node.sync_quality = quality;
  return node;
}

size_t scanIndexForDegrees(float degrees) {
  constexpr float kPi = 3.14159265358979323846F;
  float angle = degrees * kPi / 180.0F;
  while (angle > kPi) {
    angle -= 2.0F * kPi;
  }
  return static_cast<size_t>(std::ceil(
      (angle + kPi) / ((2.0F * kPi) / 359.0F)));
}
}  // namespace

TEST(ScanMapper, MapsOfficialNodeUnitsAndInvalidDistance) {
  s3_ydlidar_bridge::ScanMapperConfig config;
  config.samples = 360U;
  config.range_min = 0.1F;
  config.range_max = 8.0F;
  config.invalid_range_is_inf = true;
  s3_ydlidar_bridge::ScanMapper mapper(config);

  ::node_info valid{};
  valid.angle_q6_checkbit = static_cast<uint16_t>((90U * 64U) << 1U | 1U);
  valid.distance_q2 = 4000U;
  valid.sync_quality = 22U;
  ::node_info invalid{};
  invalid.angle_q6_checkbit = static_cast<uint16_t>((91U * 64U) << 1U | 1U);
  invalid.distance_q2 = 0U;
  ::node_info invalid_angle{};
  invalid_angle.angle_q6_checkbit = static_cast<uint16_t>(92U * 64U << 1U);
  invalid_angle.distance_q2 = 4000U;
  ::node_info out_of_range_angle{};
  out_of_range_angle.angle_q6_checkbit =
      static_cast<uint16_t>((361U * 64U) << 1U | 1U);
  out_of_range_angle.distance_q2 = 4000U;

  const auto message =
      mapper.map({valid, invalid, invalid_angle, out_of_range_angle},
                 rclcpp::Time(123456789));
  EXPECT_EQ(message.header.frame_id, "laser_frame");
  EXPECT_FLOAT_EQ(message.range_min, 0.1F);
  EXPECT_FLOAT_EQ(message.range_max, 8.0F);
  ASSERT_EQ(message.ranges.size(), 360U);
  EXPECT_TRUE(std::any_of(message.ranges.begin(), message.ranges.end(),
                          [](float value) { return value == 1.0F; }));
  EXPECT_TRUE(std::any_of(message.ranges.begin(), message.ranges.end(),
                          [](float value) {
                            return std::isinf(value) && value > 0.0F;
                          }));
  EXPECT_EQ(std::count_if(message.ranges.begin(), message.ranges.end(),
            [](float value) { return std::isfinite(value); }),
            1);
}

TEST(ScanMapper, RejectsInvalidAngleSpan) {
  s3_ydlidar_bridge::ScanMapperConfig config;
  config.angle_min = 1.0F;
  config.angle_max = 1.0F;
  config.samples = 360U;
  s3_ydlidar_bridge::ScanMapper mapper(config);

  ::node_info node{};
  node.angle_q6_checkbit = static_cast<uint16_t>((10U * 64U) << 1U | 1U);
  node.distance_q2 = 4000U;
  const auto message = mapper.map({node}, rclcpp::Time(0));

  EXPECT_FLOAT_EQ(message.angle_increment, 0.0F);
  EXPECT_TRUE(std::all_of(message.ranges.begin(), message.ranges.end(),
                          [](float value) {
                            return std::isinf(value) && value > 0.0F;
                          }));
}

TEST(ScanMapper, ZeroPacketsStartAndCloseACompleteRevolution) {
  s3_ydlidar_bridge::ScanMapper mapper(
      s3_ydlidar_bridge::ScanMapperConfig{});

  EXPECT_TRUE(mapper.accumulate({nodeAt(350U)}, false, false, 100U,
                                rclcpp::Time(100)).completed_scans.empty());
  EXPECT_FALSE(mapper.accumulationSnapshot().has_partial_revolution);

  EXPECT_TRUE(mapper.accumulate({nodeAt(5U), nodeAt(20U, 6000U)}, true,
                                false, 1000000000U, rclcpp::Time(100)).
                  completed_scans.empty());
  EXPECT_TRUE(mapper.accumulate({nodeAt(100U, 8000U), nodeAt(180U, 0U)},
                                false, true, 1500000000U, rclcpp::Time(200)).
                  completed_scans.empty());
  const auto completed = mapper.accumulate({nodeAt(5U, 16000U)}, true, false,
                                           2000000000U, rclcpp::Time(300));
  ASSERT_EQ(completed.completed_scans.size(), 1U);
  ASSERT_TRUE(completed.revolution_diagnostics.has_value());
  const auto &scan = completed.completed_scans.front();
  EXPECT_EQ(scan.header.frame_id, "laser_frame");
  EXPECT_EQ(scan.header.stamp.sec, 0);
  EXPECT_EQ(scan.header.stamp.nanosec, 100U);
  EXPECT_FLOAT_EQ(scan.scan_time, 1.0F);
  EXPECT_EQ(std::count_if(scan.ranges.begin(), scan.ranges.end(),
                          [](float value) { return std::isfinite(value); }),
            3);
  EXPECT_FLOAT_EQ(scan.ranges[scanIndexForDegrees(5.0F)], 1.0F);
  EXPECT_FLOAT_EQ(scan.ranges[scanIndexForDegrees(100.0F)], 2.0F);
  EXPECT_TRUE(std::isinf(scan.ranges[scanIndexForDegrees(180.0F)]));
  EXPECT_TRUE(std::isinf(scan.ranges[scanIndexForDegrees(300.0F)]));
  EXPECT_EQ(completed.revolution_diagnostics->frame_count, 2U);
  EXPECT_EQ(completed.revolution_diagnostics->sequence_gaps, 1U);
  EXPECT_GT(completed.revolution_diagnostics->coverage_ratio, 0.20F);
  EXPECT_FALSE(completed.revolution_diagnostics->incomplete);
  EXPECT_FALSE(completed.revolution_diagnostics->zero_timeout);

  const auto snapshot = mapper.accumulationSnapshot();
  EXPECT_TRUE(snapshot.has_partial_revolution);
  EXPECT_EQ(snapshot.coverage, 1U);
}

TEST(ScanMapper, DropsIncompleteRevolutionWhenZeroPacketTimesOut) {
  s3_ydlidar_bridge::ScanMapper mapper(
      s3_ydlidar_bridge::ScanMapperConfig{});

  EXPECT_TRUE(mapper.accumulate({nodeAt(5U), nodeAt(90U, 12000U)}, true,
                                false, 1000000000U, rclcpp::Time(100)).
                  completed_scans.empty());
  EXPECT_TRUE(mapper.accumulate({nodeAt(180U, 8000U)}, false, false,
                                1500000000U, rclcpp::Time(200)).
                  completed_scans.empty());
  const auto timed_out = mapper.expire(2000000001U, 1000000000U);
  EXPECT_TRUE(timed_out.completed_scans.empty());
  EXPECT_TRUE(timed_out.partial_revolution_dropped);
  EXPECT_TRUE(timed_out.zero_timeout);
  ASSERT_TRUE(timed_out.revolution_diagnostics.has_value());
  EXPECT_TRUE(timed_out.revolution_diagnostics->incomplete);
  EXPECT_TRUE(timed_out.revolution_diagnostics->zero_timeout);
  EXPECT_FALSE(mapper.accumulationSnapshot().has_partial_revolution);
  EXPECT_EQ(mapper.accumulationSnapshot().coverage, 0U);

  EXPECT_TRUE(mapper.accumulate({nodeAt(270U)}, false, false, 2100000000U,
                                rclcpp::Time(300)).completed_scans.empty());
  EXPECT_FALSE(mapper.accumulationSnapshot().has_partial_revolution);
  EXPECT_TRUE(mapper.accumulate({nodeAt(5U)}, true, false, 2200000000U,
                                rclcpp::Time(400)).completed_scans.empty());
  EXPECT_TRUE(mapper.accumulationSnapshot().has_partial_revolution);
}

TEST(ScanMapper, ResetDropsPartialTurnAndPreventsConnectionEpochMixing) {
  s3_ydlidar_bridge::ScanMapper mapper(
      s3_ydlidar_bridge::ScanMapperConfig{});

  EXPECT_TRUE(mapper.accumulate({nodeAt(5U), nodeAt(90U, 12000U)}, true,
                                false, 100U, rclcpp::Time(100)).
                  completed_scans.empty());
  const auto reset = mapper.resetAccumulation();
  EXPECT_TRUE(reset.partial_revolution_dropped);
  EXPECT_FALSE(mapper.accumulationSnapshot().has_partial_revolution);
  EXPECT_TRUE(mapper.accumulate({nodeAt(5U), nodeAt(90U, 6000U)}, true,
                                false, 200U, rclcpp::Time(200)).
                  completed_scans.empty());
  const auto completed = mapper.accumulate({nodeAt(5U)}, true, false, 300U,
                                           rclcpp::Time(300));
  ASSERT_EQ(completed.completed_scans.size(), 1U);
  EXPECT_FLOAT_EQ(completed.completed_scans.front().ranges[
                      scanIndexForDegrees(90.0F)],
                  1.5F);
}
