#pragma once

#include "geometry_msgs/msg/twist.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "smartcar_motion_gateway/health_gate.hpp"
#include "smartcar_motion_gateway/motion_protocol.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

#include <atomic>
#include <functional>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace tf2_ros {
class TransformBroadcaster;
}

namespace smartcar_state_bridge {
class SrpV4TelemetryAdapter;
}

namespace smartcar_motion_gateway {

enum class ConnectionCloseReason : uint8_t {
  kNone,
  kPollError,
  kPollHup,
  kRecvZero,
  kRecvError,
  kDecodeSize,
  kDecodeMagic,
  kDecodeVersion,
  kDecodeFlags,
  kDecodeLength,
  kDecodeReserved,
  kDecodeAuth,
  kDecodeCrc,
  kHelloInvalid,
  kHelloAckSendFailed,
  kLeaseRequestSendFailed,
  kSessionInvalid,
  kSequenceInvalid,
  kLeaseInvalid,
  kStatusInvalid,
  kStatusDecodeFailed,
  kLeaseExpired,
  kOutboundSendFailed,
  kHandlerReturned,
};

bool parseRawAsciiPsk(const std::string &text, std::vector<uint8_t> &output);

class GatewayNode final : public rclcpp::Node {
 public:
  explicit GatewayNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions{});
  ~GatewayNode() override;

 private:
  void onNavCommand(const geometry_msgs::msg::Twist::SharedPtr message);
  void onScan(const sensor_msgs::msg::LaserScan::SharedPtr message);
  void onOdom(const nav_msgs::msg::Odometry::SharedPtr message);
  void onTf(const tf2_msgs::msg::TFMessage::SharedPtr message);
  void onSetMotionEnabled(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                          std::shared_ptr<std_srvs::srv::SetBool::Response> response);
  void onStop(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
              std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void tick();

  void serverLoop();
  void handleClient(int client_fd);
  bool handleFrame(const MotionFrame &frame, int client_fd);
  bool handleStatus(const MotionFrame &frame);
  void serviceOutbound(int client_fd);
  bool startLeaseRequest(int client_fd);
  void revokeSession(int client_fd, uint8_t error_code,
                     bool send_error, bool send_stop);
  void clearSession();

  bool sendMessage(int client_fd, MessageType type,
                   const std::vector<uint8_t> &payload = {},
                   uint16_t ttl_ms = 0U);
  bool sendAll(int client_fd, const std::vector<uint8_t> &frame);
  bool loadPskFromLocalConfig(const std::string &path);
  void recordCloseReason(ConnectionCloseReason reason);
  void recordDecodeFailure(DecodeStatus status);
  bool leaseValid(uint64_t now_ns) const;
  bool motionPermitted(uint64_t now_ns, VelocityCommand &command,
                       HealthResult *health = nullptr) const;
  void publishZero();
  void sendWireStop();
  uint64_t steadyNowNs() const noexcept;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr safe_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr health_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr status_odom_publisher_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr motion_enable_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> status_tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;

  mutable std::mutex state_mutex_;
  VelocityCommand latest_command_{};
  uint64_t command_ns_{0U};
  uint64_t scan_ns_{0U};
  uint64_t odom_ns_{0U};
  uint64_t tf_ns_{0U};
  bool latest_command_valid_{false};

  HealthGate health_gate_;
  uint64_t health_timeout_ns_{500000000U};
  uint64_t command_timeout_ns_{250000000U};
  uint64_t command_period_ns_{50000000U};
  uint64_t heartbeat_period_ns_{50000000U};
  std::atomic<bool> enable_motion_{false};
  bool protocol_ready_{false};
  bool status_odom_enabled_{false};
  bool status_tf_enabled_{false};
  std::string nav_cmd_topic_{"/nav2/cmd_vel"};
  std::string safe_cmd_topic_{"/cmd_vel_safe"};

  mutable std::mutex session_mutex_;
  bool session_active_{false};
  bool telemetry_only_session_{false};
  bool lease_requested_{false};
  bool lease_active_{false};
  uint32_t session_id_{0U};
  uint32_t lease_id_{0U};
  uint32_t last_in_sequence_{0U};
  uint32_t next_out_sequence_{1U};
  uint64_t lease_deadline_ns_{0U};
  uint64_t connection_epoch_{0U};
  uint64_t last_motion_sent_ns_{0U};
  uint64_t last_heartbeat_sent_ns_{0U};
  bool motion_active_{false};

  std::atomic<uint64_t> accept_count_{0U};
  std::atomic<uint64_t> recv_zero_count_{0U};
  std::atomic<uint64_t> recv_error_count_{0U};
  std::atomic<uint64_t> poll_hup_count_{0U};
  std::atomic<uint64_t> poll_error_count_{0U};
  std::atomic<uint64_t> decode_size_count_{0U};
  std::atomic<uint64_t> decode_magic_count_{0U};
  std::atomic<uint64_t> decode_version_count_{0U};
  std::atomic<uint64_t> decode_flags_count_{0U};
  std::atomic<uint64_t> decode_length_count_{0U};
  std::atomic<uint64_t> decode_reserved_count_{0U};
  std::atomic<uint64_t> decode_auth_count_{0U};
  std::atomic<uint64_t> decode_crc_count_{0U};
  std::atomic<uint64_t> hello_invalid_count_{0U};
  std::atomic<uint64_t> hello_ack_sent_count_{0U};
  std::atomic<uint64_t> hello_ack_send_failed_count_{0U};
  std::atomic<uint64_t> telemetry_only_session_count_{0U};
  std::atomic<uint64_t> lease_request_sent_count_{0U};
  std::atomic<uint64_t> lease_request_send_failed_count_{0U};
  std::atomic<uint64_t> lease_expired_count_{0U};
  std::atomic<uint64_t> status_received_count_{0U};
  std::atomic<uint64_t> status_accept_count_{0U};
  std::atomic<uint64_t> status_reject_count_{0U};
  std::atomic<ConnectionCloseReason> last_close_reason_{ConnectionCloseReason::kNone};

  std::vector<uint8_t> psk_;
  bool authentication_ready_{false};
  std::unique_ptr<smartcar_state_bridge::SrpV4TelemetryAdapter> status_adapter_;

  std::atomic<bool> running_{false};
  std::thread server_thread_;
  std::mutex send_mutex_;
  std::mutex socket_mutex_;
  int server_fd_{-1};
  int client_fd_{-1};
  std::string listen_address_{"0.0.0.0"};
  uint16_t listen_port_{8766U};
};

}  // namespace smartcar_motion_gateway
