#pragma once

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/time.hpp"

#include <array>
#include <string>

namespace smartcar_state_bridge {

struct PlanarOdomData {
  double x_m{0.0};
  double y_m{0.0};
  double yaw_rad{0.0};
  double linear_x_mps{0.0};
  double linear_y_mps{0.0};
  double angular_z_rps{0.0};
};

struct OdomMessageConfig {
  std::string frame_id{"odom"};
  std::string child_frame_id{"base_link"};
  // These diagonals preserve the existing conservative, non-zero covariance
  // values. Unobserved axes stay explicitly high-variance.
  std::array<double, 6U> pose_covariance_diagonal{
      {1.0, 1.0, 1.0e6, 1.0e6, 1.0e6, 1.0}};
  std::array<double, 6U> twist_covariance_diagonal{
      {0.25, 1.0e6, 1.0e6, 1.0e6, 1.0e6, 0.25}};
};

nav_msgs::msg::Odometry makeOdometryMessage(
    const PlanarOdomData &data, const rclcpp::Time &receive_time,
    const OdomMessageConfig &config = {});

// Build TF directly from the already-stamped odometry message so both outputs
// necessarily carry the same ROS receive timestamp and pose.
geometry_msgs::msg::TransformStamped makeOdomTransform(
    const nav_msgs::msg::Odometry &odom);

}  // namespace smartcar_state_bridge
