#pragma once

#include "smartcar_state_bridge/state_adapter.hpp"

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include <memory>
#include <mutex>
#include <string>

namespace smartcar_state_bridge {

// This node is a publisher/adapter only.  It owns no socket, serial port, or
// command path; a single gateway may call submitTelemetry after parsing its
// already-owned connection.  With the default parameters live input and odom
// publication are both disabled.
class StateNode final : public rclcpp::Node {
 public:
  explicit StateNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  StateSubmitResult submitTelemetry(const TelemetryEnvelope &envelope);

  const TelemetryDecoder &decoder() const noexcept { return adapter_.decoder(); }
  const WheelOdom &odom() const noexcept { return adapter_.odom(); }

 private:
  void publishOdometry(const OdomUpdate &update);
  void publishDiagnostics();

  StateAdapter adapter_{};
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
      diagnostics_publisher_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
  std::mutex mutex_;
  std::string odom_topic_{"/odom"};
  std::string diagnostics_topic_{"/diagnostics"};
  std::string frame_id_{"odom"};
  std::string child_frame_id_{"base_link"};
  bool enable_live_odom_{false};
  bool publish_odom_{false};
  bool publish_tf_{false};
  uint64_t submitted_samples_{0U};
  uint64_t published_samples_{0U};
  uint64_t rejected_samples_{0U};
  TelemetryDecodeStatus last_decode_status_{TelemetryDecodeStatus::kNotConfigured};
};

}  // namespace smartcar_state_bridge
