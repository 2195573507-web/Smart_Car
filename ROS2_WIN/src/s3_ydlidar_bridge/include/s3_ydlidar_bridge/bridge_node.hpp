#pragma once

#include "rclcpp/rclcpp.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "smartcar_state_bridge/chassis_state.hpp"
#include "smartcar_state_bridge/odom_message.hpp"
#include "smartcar_state_bridge/state_adapter.hpp"
#include "s3_ydlidar_bridge/framing.hpp"
#include "s3_ydlidar_bridge/official_decoder.hpp"
#include "s3_ydlidar_bridge/scan_mapper.hpp"
#include "s3_ydlidar_bridge/transport.hpp"

#include <atomic>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace s3_ydlidar_bridge {

class BridgeNode final : public rclcpp::Node {
 public:
  BridgeNode();
  ~BridgeNode() override;

 private:
  void onFrame(ReceivedFrame frame);
  void onConnectionEvent(ConnectionEvent event);
  void publishDiagnostics();
  void handleOpaqueFrame(
      const ReceivedFrame &frame,
      smartcar_state_bridge::TelemetryOuterSequenceStatus sequence_status,
      uint64_t sequence_gap);
  void publishOdometry(const smartcar_state_bridge::OdomUpdate &update);
  void publishChassisOdometry(
      const smartcar_state_bridge::ChassisOdomUpdate &update);
  void publishPlanarOdometry(
      const smartcar_state_bridge::PlanarOdomData &data);

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr telemetry_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
  std::unique_ptr<FrameTransport> transport_;
  OfficialDecoder decoder_;
  ScanMapper mapper_;
  smartcar_state_bridge::StateAdapter state_adapter_;
  smartcar_state_bridge::SrpV4TelemetryAdapter srp_v4_telemetry_adapter_;
  mutable std::mutex sequence_mutex_;
  mutable std::mutex accumulation_mutex_;
  mutable std::mutex telemetry_mutex_;
  // S3RD sequence belongs to the complete TCP connection, including every
  // admitted message type. The outer extractor already enforces identity.
  SequenceTracker sequence_tracker_;
  uint64_t accumulation_epoch_{0};
  uint64_t sequence_connection_epoch_{0};
  int stale_after_ms_{500};
  int zero_packet_timeout_ms_{1000};
  std::atomic<uint64_t> ydlidar_errors_{0};
  std::atomic<uint64_t> opaque_frames_{0};
  std::atomic<uint64_t> decoded_packets_{0};
  std::atomic<uint64_t> accumulated_packets_{0};
  std::atomic<uint64_t> revolutions_published_{0};
  std::atomic<uint64_t> valid_points_{0};
  std::atomic<uint64_t> partial_revolutions_dropped_{0};
  std::atomic<uint64_t> scan_timeouts_{0};
  std::atomic<uint64_t> incomplete_revolutions_{0};
  std::atomic<uint64_t> duplicate_sequences_{0};
  std::atomic<uint64_t> out_of_order_sequences_{0};
  std::atomic<uint64_t> sequence_jumps_{0};
  std::atomic<uint64_t> sequence_gaps_{0};
  std::atomic<uint64_t> sequence_wraps_{0};
  std::atomic<uint64_t> zero_packets_{0};
  std::atomic<uint64_t> published_scans_{0};
  std::atomic<uint64_t> telemetry_not_configured_{0};
  std::atomic<uint64_t> telemetry_disabled_{0};
  std::atomic<uint64_t> telemetry_rejected_{0};
  std::atomic<uint64_t> telemetry_accepted_{0};
  std::atomic<uint64_t> telemetry_published_raw_{0};
  std::atomic<uint64_t> stale_frames_{0};
  std::atomic<uint64_t> odom_published_{0};
  bool publish_opaque_telemetry_{false};
  std::string telemetry_topic_{"/smartcar/telemetry/raw"};
  std::string odom_topic_{"/odom"};
  std::string odom_frame_id_{"odom"};
  std::string odom_child_frame_id_{"base_link"};
  bool enable_live_odom_{false};
  bool publish_odom_{false};
  bool publish_tf_{false};
  std::atomic<uint64_t> last_valid_packet_ns_{0};
  std::atomic<uint64_t> last_valid_scan_ns_{0};
  uint64_t last_diagnostics_ns_{0};
  uint64_t last_diagnostics_published_{0};
};

}  // namespace s3_ydlidar_bridge
