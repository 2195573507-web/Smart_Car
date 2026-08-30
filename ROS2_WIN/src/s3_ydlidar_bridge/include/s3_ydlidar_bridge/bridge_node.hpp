#pragma once

#include "rclcpp/rclcpp.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"

#include <atomic>
#include <mutex>
#include "s3_ydlidar_bridge/framing.hpp"
#include "s3_ydlidar_bridge/official_decoder.hpp"
#include "s3_ydlidar_bridge/scan_mapper.hpp"
#include "s3_ydlidar_bridge/telemetry_decoder.hpp"
#include "s3_ydlidar_bridge/transport.hpp"

namespace s3_ydlidar_bridge {

class BridgeNode final : public rclcpp::Node {
 public:
  BridgeNode();
  ~BridgeNode() override;

 private:
  void onFrame(ReceivedFrame frame);
  void onConnectionEvent(ConnectionEvent event);
  void publishDiagnostics();

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
  std::unique_ptr<FrameTransport> transport_;
  OfficialDecoder decoder_;
  TelemetryDecoder telemetry_decoder_;
  ScanMapper mapper_;
  mutable std::mutex sequence_mutex_;
  mutable std::mutex accumulation_mutex_;
  mutable std::mutex telemetry_mutex_;
  SequenceTracker sequence_tracker_;
  uint64_t accumulation_epoch_{0};
  int stale_after_ms_{500};
  int zero_packet_timeout_ms_{1000};
  uint8_t raw_message_type_{1};
  std::atomic<uint64_t> ydlidar_errors_{0};
  std::atomic<uint64_t> decoded_packets_{0};
  std::atomic<uint64_t> telemetry_packets_{0};
  std::atomic<uint64_t> telemetry_bytes_{0};
  std::atomic<uint64_t> telemetry_invalid_{0};
  std::atomic<uint64_t> telemetry_unsupported_{0};
  std::atomic<uint64_t> wheel_speed_packets_{0};
  std::atomic<uint64_t> attitude_packets_{0};
  std::atomic<uint64_t> imu_packets_{0};
  std::atomic<uint64_t> wheel_freshness_unavailable_{0};
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
  std::atomic<uint64_t> last_valid_packet_ns_{0};
  std::atomic<uint64_t> last_valid_scan_ns_{0};
  uint64_t last_diagnostics_ns_{0};
  uint64_t last_diagnostics_published_{0};
};

}  // namespace s3_ydlidar_bridge
