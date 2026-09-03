#include "smartcar_state_bridge/odom_message.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace {

constexpr double kPi = 3.14159265358979323846;

TEST(OdomMessage, ConvertsYawToUnitQuaternionAndCopiesBodyTwist) {
  smartcar_state_bridge::PlanarOdomData data;
  data.x_m = 1.25;
  data.y_m = -0.5;
  data.yaw_rad = 179.0 * kPi / 180.0;
  data.linear_x_mps = 0.4;
  data.linear_y_mps = -0.1;
  data.angular_z_rps = 0.2;
  const rclcpp::Time stamp(1234567890LL, RCL_ROS_TIME);

  const auto odom = smartcar_state_bridge::makeOdometryMessage(data, stamp);
  EXPECT_EQ(odom.header.stamp.sec, 1);
  EXPECT_EQ(odom.header.stamp.nanosec, 234567890U);
  EXPECT_EQ(odom.header.frame_id, "odom");
  EXPECT_EQ(odom.child_frame_id, "base_link");
  EXPECT_DOUBLE_EQ(odom.pose.pose.position.x, 1.25);
  EXPECT_DOUBLE_EQ(odom.pose.pose.position.y, -0.5);
  EXPECT_NEAR(odom.pose.pose.orientation.z,
              std::sin(data.yaw_rad * 0.5), 1.0e-12);
  EXPECT_NEAR(odom.pose.pose.orientation.w,
              std::cos(data.yaw_rad * 0.5), 1.0e-12);
  EXPECT_NEAR(std::hypot(odom.pose.pose.orientation.z,
                         odom.pose.pose.orientation.w),
              1.0, 1.0e-12);
  EXPECT_DOUBLE_EQ(odom.twist.twist.linear.x, 0.4);
  EXPECT_DOUBLE_EQ(odom.twist.twist.linear.y, -0.1);
  EXPECT_DOUBLE_EQ(odom.twist.twist.angular.z, 0.2);
}

TEST(OdomMessage, OdomAndTfShareTimestampPoseAndNonZeroCovariance) {
  smartcar_state_bridge::PlanarOdomData data;
  data.x_m = 2.0;
  data.y_m = 3.0;
  data.yaw_rad = 0.5;
  const rclcpp::Time stamp(9876543210LL, RCL_ROS_TIME);
  const auto odom = smartcar_state_bridge::makeOdometryMessage(data, stamp);
  const auto transform = smartcar_state_bridge::makeOdomTransform(odom);

  EXPECT_EQ(transform.header.stamp, odom.header.stamp);
  EXPECT_EQ(transform.header.frame_id, odom.header.frame_id);
  EXPECT_EQ(transform.child_frame_id, odom.child_frame_id);
  EXPECT_DOUBLE_EQ(transform.transform.translation.x,
                   odom.pose.pose.position.x);
  EXPECT_DOUBLE_EQ(transform.transform.translation.y,
                   odom.pose.pose.position.y);
  EXPECT_EQ(transform.transform.rotation, odom.pose.pose.orientation);

  for (std::size_t axis = 0U; axis < 6U; ++axis) {
    EXPECT_GT(odom.pose.covariance[axis * 7U], 0.0);
    EXPECT_GT(odom.twist.covariance[axis * 7U], 0.0);
  }
}

}  // namespace
