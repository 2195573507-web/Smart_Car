#include "smartcar_state_bridge/odom_message.hpp"

#include <cmath>

namespace smartcar_state_bridge {

nav_msgs::msg::Odometry makeOdometryMessage(
    const PlanarOdomData &data, const rclcpp::Time &receive_time,
    const OdomMessageConfig &config) {
  nav_msgs::msg::Odometry message;
  message.header.stamp = receive_time;
  message.header.frame_id = config.frame_id;
  message.child_frame_id = config.child_frame_id;
  message.pose.pose.position.x = data.x_m;
  message.pose.pose.position.y = data.y_m;
  message.pose.pose.position.z = 0.0;
  message.pose.pose.orientation.z = std::sin(data.yaw_rad * 0.5);
  message.pose.pose.orientation.w = std::cos(data.yaw_rad * 0.5);
  message.twist.twist.linear.x = data.linear_x_mps;
  message.twist.twist.linear.y = data.linear_y_mps;
  message.twist.twist.angular.z = data.angular_z_rps;

  message.pose.covariance.fill(0.0);
  message.twist.covariance.fill(0.0);
  for (std::size_t axis = 0U; axis < 6U; ++axis) {
    const std::size_t diagonal = axis * 7U;
    message.pose.covariance[diagonal] =
        config.pose_covariance_diagonal[axis];
    message.twist.covariance[diagonal] =
        config.twist_covariance_diagonal[axis];
  }
  return message;
}

geometry_msgs::msg::TransformStamped makeOdomTransform(
    const nav_msgs::msg::Odometry &odom) {
  geometry_msgs::msg::TransformStamped transform;
  transform.header = odom.header;
  transform.child_frame_id = odom.child_frame_id;
  transform.transform.translation.x = odom.pose.pose.position.x;
  transform.transform.translation.y = odom.pose.pose.position.y;
  transform.transform.translation.z = odom.pose.pose.position.z;
  transform.transform.rotation = odom.pose.pose.orientation;
  return transform;
}

}  // namespace smartcar_state_bridge
