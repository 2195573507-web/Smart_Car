#include "smartcar_motion_gateway/gateway_node.hpp"

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "smartcar_state_bridge/chassis_state.hpp"
#include "smartcar_state_bridge/odom_message.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include <arpa/inet.h>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iterator>
#include <limits>
#include <netinet/in.h>
#include <poll.h>
#include <random>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

namespace smartcar_motion_gateway {

bool parseRawAsciiPsk(const std::string &text, std::vector<uint8_t> &output) {
  output.clear();
  std::size_t key_size = text.size();
  if (key_size > 0U && text.back() == '\n') {
    --key_size;
    if (key_size > 0U && text[key_size - 1U] == '\r') --key_size;
  }
  if (key_size == 0U) return false;

  for (std::size_t index = 0U; index < key_size; ++index) {
    const uint8_t byte = static_cast<uint8_t>(text[index]);
    if (byte == 0U || byte > 0x7FU || byte == '\r' || byte == '\n') return false;
  }
  if (text.front() == ' ' || text.front() == '\t' ||
      text[key_size - 1U] == ' ' || text[key_size - 1U] == '\t') {
    return false;
  }

  output.assign(text.begin(), text.begin() + static_cast<std::ptrdiff_t>(key_size));
  return true;
}

namespace {

constexpr uint8_t kErrorMalformed = 1U;
constexpr uint8_t kErrorSequence = 2U;
constexpr uint8_t kErrorSession = 3U;
constexpr uint8_t kErrorLease = 4U;
constexpr uint8_t kErrorStatus = 5U;

void addKey(diagnostic_msgs::msg::DiagnosticStatus &status,
            const std::string &key, const std::string &value) {
  diagnostic_msgs::msg::KeyValue item;
  item.key = key;
  item.value = value;
  status.values.push_back(std::move(item));
}

void addCounter(diagnostic_msgs::msg::DiagnosticStatus &status,
                const std::string &key, uint64_t value) {
  addKey(status, key, std::to_string(value));
}

const char *toString(ConnectionCloseReason reason) noexcept {
  switch (reason) {
    case ConnectionCloseReason::kNone: return "none";
    case ConnectionCloseReason::kPollError: return "poll_error";
    case ConnectionCloseReason::kPollHup: return "poll_hup";
    case ConnectionCloseReason::kRecvZero: return "recv_zero";
    case ConnectionCloseReason::kRecvError: return "recv_error";
    case ConnectionCloseReason::kDecodeSize: return "decode_size";
    case ConnectionCloseReason::kDecodeMagic: return "decode_magic";
    case ConnectionCloseReason::kDecodeVersion: return "decode_version";
    case ConnectionCloseReason::kDecodeFlags: return "decode_flags";
    case ConnectionCloseReason::kDecodeLength: return "decode_length";
    case ConnectionCloseReason::kDecodeReserved: return "decode_reserved";
    case ConnectionCloseReason::kDecodeAuth: return "decode_auth";
    case ConnectionCloseReason::kDecodeCrc: return "decode_crc";
    case ConnectionCloseReason::kHelloInvalid: return "hello_invalid";
    case ConnectionCloseReason::kHelloAckSendFailed: return "hello_ack_send_failed";
    case ConnectionCloseReason::kLeaseRequestSendFailed: return "lease_request_send_failed";
    case ConnectionCloseReason::kSessionInvalid: return "session_invalid";
    case ConnectionCloseReason::kSequenceInvalid: return "sequence_invalid";
    case ConnectionCloseReason::kLeaseInvalid: return "lease_invalid";
    case ConnectionCloseReason::kStatusInvalid: return "status_invalid";
    case ConnectionCloseReason::kStatusDecodeFailed: return "status_decode_failed";
    case ConnectionCloseReason::kLeaseExpired: return "lease_expired";
    case ConnectionCloseReason::kOutboundSendFailed: return "outbound_send_failed";
    case ConnectionCloseReason::kHandlerReturned: return "handler_returned";
  }
  return "unknown";
}

bool parseAsciiPsk(const std::string &text, std::vector<uint8_t> &output) {
  return parseRawAsciiPsk(text, output);
}

bool validEmptyControl(const MotionFrame &frame) noexcept {
  return frame.payload.empty() && frame.ttl_ms == 0U;
}

}  // namespace

GatewayNode::GatewayNode(const rclcpp::NodeOptions &options)
    : Node("smartcar_motion_gateway", options), health_gate_(health_timeout_ns_) {
  enable_motion_.store(declare_parameter("enable_motion", false));
  protocol_ready_ = declare_parameter("protocol_ready", false);
  listen_address_ = declare_parameter("listen_address", listen_address_);
  const int64_t port = declare_parameter("listen_port", static_cast<int64_t>(listen_port_));
  if (port > 0 && port <= 65535) listen_port_ = static_cast<uint16_t>(port);
  nav_cmd_topic_ = declare_parameter("nav_cmd_topic", nav_cmd_topic_);
  safe_cmd_topic_ = declare_parameter("safe_cmd_topic", safe_cmd_topic_);
  const int64_t health_timeout_ms = declare_parameter("health_timeout_ms", 500);
  const int64_t command_timeout_ms = declare_parameter("command_timeout_ms", 250);
  const int64_t command_period_ms = declare_parameter("command_period_ms", 50);
  const int64_t heartbeat_period_ms = declare_parameter("heartbeat_period_ms", 50);
  status_odom_enabled_ = declare_parameter("enable_status_odom", false);
  status_tf_enabled_ = declare_parameter("publish_status_tf", false) && status_odom_enabled_;
  if (health_timeout_ms > 0) health_timeout_ns_ = static_cast<uint64_t>(health_timeout_ms) * 1000000U;
  if (command_timeout_ms > 0) command_timeout_ns_ = static_cast<uint64_t>(command_timeout_ms) * 1000000U;
  if (command_period_ms > 0) command_period_ns_ = static_cast<uint64_t>(command_period_ms) * 1000000U;
  if (heartbeat_period_ms > 0) heartbeat_period_ns_ = static_cast<uint64_t>(heartbeat_period_ms) * 1000000U;
  health_gate_ = HealthGate(health_timeout_ns_);

  const std::string psk_config_path = declare_parameter("psk_config_path", std::string{});
  authentication_ready_ = loadPskFromLocalConfig(psk_config_path);

  smartcar_state_bridge::ChassisStateAdapterConfig chassis_config;
  chassis_config.allow_live_telemetry = status_odom_enabled_;
  chassis_config.enable_live_odom = status_odom_enabled_;
  chassis_config.allow_offline_fixtures = false;
  chassis_config.require_outer_sequence = false;
  status_adapter_ = std::make_unique<smartcar_state_bridge::SrpV4TelemetryAdapter>(
      chassis_config);

  safe_publisher_ = create_publisher<geometry_msgs::msg::Twist>(safe_cmd_topic_, rclcpp::QoS(10));
  health_publisher_ = create_publisher<std_msgs::msg::Bool>("/smartcar/motion_healthy", rclcpp::QoS(10));
  diagnostics_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/smartcar/motion_diagnostics", rclcpp::QoS(10));
  status_odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>("/odom", rclcpp::QoS(10));
  status_tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
  motion_enable_service_ = create_service<std_srvs::srv::SetBool>(
      "~/set_motion_enabled",
      std::bind(&GatewayNode::onSetMotionEnabled, this, std::placeholders::_1,
                std::placeholders::_2));
  stop_service_ = create_service<std_srvs::srv::Trigger>(
      "~/stop", std::bind(&GatewayNode::onStop, this, std::placeholders::_1,
                           std::placeholders::_2));
  nav_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      nav_cmd_topic_, rclcpp::QoS(10),
      std::bind(&GatewayNode::onNavCommand, this, std::placeholders::_1));
  scan_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", rclcpp::SensorDataQoS(),
      std::bind(&GatewayNode::onScan, this, std::placeholders::_1));
  odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", rclcpp::QoS(10), std::bind(&GatewayNode::onOdom, this, std::placeholders::_1));
  tf_subscription_ = create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf", rclcpp::QoS(10), std::bind(&GatewayNode::onTf, this, std::placeholders::_1));
  timer_ = create_wall_timer(std::chrono::milliseconds(20), std::bind(&GatewayNode::tick, this));

  if (!authentication_ready_) {
    RCLCPP_WARN(get_logger(), "motion control listener disabled: local PSK configuration is unavailable");
    return;
  }
  running_ = true;
  server_thread_ = std::thread(&GatewayNode::serverLoop, this);
  RCLCPP_INFO(get_logger(), "ROS Motion Control v1 listener ready on TCP %u", listen_port_);
}

GatewayNode::~GatewayNode() {
  running_ = false;
  int client_for_stop = -1;
  {
    std::lock_guard<std::mutex> lock(socket_mutex_);
    client_for_stop = client_fd_;
  }
  // A live socket still has a usable protocol session during orderly ROS
  // teardown. The peer may disappear concurrently; that falls back to zero
  // publication below without treating a failed best-effort STOP as success.
  if (client_for_stop >= 0) sendMessage(client_for_stop, MessageType::kStop);
  {
    std::lock_guard<std::mutex> lock(socket_mutex_);
    if (client_fd_ >= 0) {
      shutdown(client_fd_, SHUT_RDWR);
      close(client_fd_);
      client_fd_ = -1;
    }
    if (server_fd_ >= 0) {
      shutdown(server_fd_, SHUT_RDWR);
      close(server_fd_);
      server_fd_ = -1;
    }
  }
  if (server_thread_.joinable()) server_thread_.join();
  publishZero();
  clearSession();
  std::fill(psk_.begin(), psk_.end(), 0U);
}

uint64_t GatewayNode::steadyNowNs() const noexcept {
  return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count());
}

bool GatewayNode::loadPskFromLocalConfig(const std::string &path) {
  if (path.empty()) return false;
  std::ifstream input(path, std::ios::in | std::ios::binary);
  if (!input.is_open()) return false;
  const std::string text((std::istreambuf_iterator<char>(input)),
                         std::istreambuf_iterator<char>());
  std::vector<uint8_t> parsed;
  if (!parseAsciiPsk(text, parsed)) return false;
  psk_ = std::move(parsed);
  return true;
}

void GatewayNode::recordCloseReason(ConnectionCloseReason reason) {
  ConnectionCloseReason expected = ConnectionCloseReason::kNone;
  (void)last_close_reason_.compare_exchange_strong(expected, reason);
}

void GatewayNode::recordDecodeFailure(DecodeStatus status) {
  switch (status) {
    case DecodeStatus::kSizeError:
      ++decode_size_count_;
      recordCloseReason(ConnectionCloseReason::kDecodeSize);
      return;
    case DecodeStatus::kMagicError:
      ++decode_magic_count_;
      recordCloseReason(ConnectionCloseReason::kDecodeMagic);
      return;
    case DecodeStatus::kVersionError:
      ++decode_version_count_;
      recordCloseReason(ConnectionCloseReason::kDecodeVersion);
      return;
    case DecodeStatus::kFlagsError:
      ++decode_flags_count_;
      recordCloseReason(ConnectionCloseReason::kDecodeFlags);
      return;
    case DecodeStatus::kLengthError:
      ++decode_length_count_;
      recordCloseReason(ConnectionCloseReason::kDecodeLength);
      return;
    case DecodeStatus::kReservedError:
      ++decode_reserved_count_;
      recordCloseReason(ConnectionCloseReason::kDecodeReserved);
      return;
    case DecodeStatus::kAuthError:
      ++decode_auth_count_;
      recordCloseReason(ConnectionCloseReason::kDecodeAuth);
      return;
    case DecodeStatus::kCrcError:
      ++decode_crc_count_;
      recordCloseReason(ConnectionCloseReason::kDecodeCrc);
      return;
    case DecodeStatus::kAccepted:
      return;
  }
}

void GatewayNode::onNavCommand(const geometry_msgs::msg::Twist::SharedPtr message) {
  const float linear = static_cast<float>(message->linear.x);
  const float angular = static_cast<float>(message->angular.z);
  std::lock_guard<std::mutex> lock(state_mutex_);
  latest_command_ = VelocityCommand{linear, angular};
  latest_command_valid_ = validVelocity(linear, angular);
  command_ns_ = steadyNowNs();
}

void GatewayNode::onScan(const sensor_msgs::msg::LaserScan::SharedPtr message) {
  if (!message->ranges.empty() && message->header.frame_id == "laser_frame") {
    std::lock_guard<std::mutex> lock(state_mutex_);
    scan_ns_ = steadyNowNs();
  }
}

void GatewayNode::onOdom(const nav_msgs::msg::Odometry::SharedPtr message) {
  if (message->header.frame_id == "odom" && message->child_frame_id == "base_link" &&
      std::isfinite(message->pose.pose.position.x) &&
      std::isfinite(message->pose.pose.position.y)) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    odom_ns_ = steadyNowNs();
  }
}

void GatewayNode::onTf(const tf2_msgs::msg::TFMessage::SharedPtr message) {
  for (const auto &transform : message->transforms) {
    if (transform.header.frame_id == "odom" && transform.child_frame_id == "base_link") {
      std::lock_guard<std::mutex> lock(state_mutex_);
      tf_ns_ = steadyNowNs();
      return;
    }
  }
}

bool GatewayNode::leaseValid(uint64_t now_ns) const {
  std::lock_guard<std::mutex> lock(session_mutex_);
  return lease_active_ && now_ns <= lease_deadline_ns_;
}

bool GatewayNode::motionPermitted(uint64_t now_ns, VelocityCommand &command,
                                  HealthResult *health) const {
  HealthSnapshot snapshot;
  bool command_valid = false;
  uint64_t command_ns = 0U;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    snapshot.now_ns = now_ns;
    snapshot.scan_ns = scan_ns_;
    snapshot.odom_ns = odom_ns_;
    snapshot.tf_ns = tf_ns_;
    command = latest_command_;
    command_valid = latest_command_valid_;
    command_ns = command_ns_;
  }
  snapshot.lease = leaseValid(now_ns);
  const HealthResult evaluated = health_gate_.evaluate(snapshot);
  if (health != nullptr) *health = evaluated;
  const bool command_fresh = command_ns != 0U && now_ns >= command_ns &&
                             now_ns - command_ns <= command_timeout_ns_;
  return authentication_ready_ && enable_motion_.load() && protocol_ready_ &&
         evaluated.all() && command_fresh && command_valid;
}

void GatewayNode::publishZero() {
  geometry_msgs::msg::Twist zero;
  safe_publisher_->publish(zero);
}

void GatewayNode::sendWireStop() {
  int client_fd = -1;
  bool motion_session = false;
  {
    std::lock_guard<std::mutex> lock(session_mutex_);
    motion_session = session_active_ && !telemetry_only_session_ && lease_requested_;
  }
  {
    std::lock_guard<std::mutex> lock(socket_mutex_);
    client_fd = client_fd_;
  }
  if (motion_session && client_fd >= 0) {
    (void)sendMessage(client_fd, MessageType::kStop);
  }
  publishZero();
}

void GatewayNode::onSetMotionEnabled(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
  if (!request->data) {
    enable_motion_.store(false);
    sendWireStop();
    int client_fd = -1;
    bool close_motion_session = false;
    {
      std::lock_guard<std::mutex> lock(session_mutex_);
      close_motion_session = session_active_ && !telemetry_only_session_ && lease_requested_;
      if (close_motion_session) {
        // The S3 owns lease revocation. Close the old motion socket after its
        // explicit STOP so its next HELLO establishes a fresh telemetry session.
        telemetry_only_session_ = true;
        lease_requested_ = false;
        lease_active_ = false;
        lease_id_ = 0U;
        lease_deadline_ns_ = 0U;
        motion_active_ = false;
      }
    }
    {
      std::lock_guard<std::mutex> lock(socket_mutex_);
      client_fd = client_fd_;
    }
    if (close_motion_session && client_fd >= 0) {
      shutdown(client_fd, SHUT_RDWR);
    }
    response->success = true;
    response->message = "motion disabled; active motion lease sessions are stopped and rehandshaken";
    return;
  }
  if (!authentication_ready_ || !protocol_ready_) {
    enable_motion_.store(false);
    publishZero();
    response->success = false;
    response->message = "authentication and protocol readiness are required";
    return;
  }
  enable_motion_.store(true);
  int client_fd = -1;
  bool telemetry_session = false;
  {
    std::lock_guard<std::mutex> lock(session_mutex_);
    telemetry_session = session_active_ && telemetry_only_session_;
  }
  {
    std::lock_guard<std::mutex> lock(socket_mutex_);
    client_fd = client_fd_;
  }
  if (telemetry_session) {
    if (client_fd < 0 || !startLeaseRequest(client_fd)) {
      enable_motion_.store(false);
      publishZero();
      response->success = false;
      response->message = "unable to send a new lease request for the telemetry session";
      return;
    }
    response->success = true;
    response->message = "motion authorization enabled; awaiting a valid TCP 8766 lease response";
    return;
  }
  response->success = true;
  response->message = "motion authorization enabled; a lease will be requested after HELLO";
}

void GatewayNode::onStop(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                         std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  (void)request;
  sendWireStop();
  response->success = true;
  response->message = "zero command published; STOP is sent only for an active motion session";
}

void GatewayNode::tick() {
  const uint64_t now_ns = steadyNowNs();
  VelocityCommand command;
  HealthResult health;
  const bool permitted = motionPermitted(now_ns, command, &health);
  geometry_msgs::msg::Twist safe;
  if (permitted) {
    safe.linear.x = command.linear_mps;
    safe.angular.z = command.angular_rps;
  }
  safe_publisher_->publish(safe);

  std_msgs::msg::Bool healthy;
  healthy.data = authentication_ready_ && enable_motion_.load() && protocol_ready_ && health.all();
  health_publisher_->publish(healthy);

  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = now();
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "smartcar_motion_gateway";
  status.hardware_id = "ros_motion_control_v1_tcp_8766";
  status.level = permitted ? diagnostic_msgs::msg::DiagnosticStatus::OK
                           : diagnostic_msgs::msg::DiagnosticStatus::WARN;
  status.message = permitted ? "motion_permitted" : "motion_inhibited";
  bool telemetry_only = false;
  {
    std::lock_guard<std::mutex> lock(session_mutex_);
    telemetry_only = session_active_ && telemetry_only_session_;
  }
  addKey(status, "authentication_ready", authentication_ready_ ? "true" : "false");
  addKey(status, "enable_motion", enable_motion_.load() ? "true" : "false");
  addKey(status, "protocol_ready", protocol_ready_ ? "true" : "false");
  addKey(status, "telemetry_only_active", telemetry_only ? "true" : "false");
  addKey(status, "scan", health.scan ? "true" : "false");
  addKey(status, "odom", health.odom ? "true" : "false");
  addKey(status, "tf_odom_base_link", health.tf ? "true" : "false");
  addKey(status, "lease", health.lease ? "true" : "false");
  addKey(status, "tcp_port", std::to_string(listen_port_));
  addCounter(status, "accept", accept_count_.load());
  addCounter(status, "recv_zero", recv_zero_count_.load());
  addCounter(status, "recv_error", recv_error_count_.load());
  addCounter(status, "poll_hup", poll_hup_count_.load());
  addCounter(status, "poll_error", poll_error_count_.load());
  addCounter(status, "decode_size", decode_size_count_.load());
  addCounter(status, "decode_magic", decode_magic_count_.load());
  addCounter(status, "decode_version", decode_version_count_.load());
  addCounter(status, "decode_flags", decode_flags_count_.load());
  addCounter(status, "decode_length", decode_length_count_.load());
  addCounter(status, "decode_reserved", decode_reserved_count_.load());
  addCounter(status, "decode_auth", decode_auth_count_.load());
  addCounter(status, "decode_crc", decode_crc_count_.load());
  addCounter(status, "hello_invalid", hello_invalid_count_.load());
  addCounter(status, "hello_ack_sent", hello_ack_sent_count_.load());
  addCounter(status, "hello_ack_send_failed", hello_ack_send_failed_count_.load());
  addCounter(status, "telemetry_only_session", telemetry_only_session_count_.load());
  addCounter(status, "lease_request_sent", lease_request_sent_count_.load());
  addCounter(status, "lease_request_send_failed", lease_request_send_failed_count_.load());
  addCounter(status, "lease_expired", lease_expired_count_.load());
  addCounter(status, "status_received", status_received_count_.load());
  addCounter(status, "status_accept", status_accept_count_.load());
  addCounter(status, "status_reject", status_reject_count_.load());
  addKey(status, "last_close_reason", toString(last_close_reason_.load()));
  array.status.push_back(std::move(status));
  diagnostics_publisher_->publish(array);
}

bool GatewayNode::sendAll(int client_fd, const std::vector<uint8_t> &frame) {
  std::lock_guard<std::mutex> lock(send_mutex_);
  const auto *data = reinterpret_cast<const char *>(frame.data());
  std::size_t sent = 0U;
  while (sent < frame.size()) {
    const ssize_t count = send(client_fd, data + sent, frame.size() - sent, MSG_NOSIGNAL);
    if (count <= 0) return false;
    sent += static_cast<std::size_t>(count);
  }
  return true;
}

bool GatewayNode::sendMessage(int client_fd, MessageType type,
                              const std::vector<uint8_t> &payload,
                              uint16_t ttl_ms) {
  MotionFrame frame;
  {
    std::lock_guard<std::mutex> lock(session_mutex_);
    if (!session_active_) return false;
    if (telemetry_only_session_ &&
        (type == MessageType::kLeaseRequest || type == MessageType::kHeartbeat ||
         type == MessageType::kMotionCommand || type == MessageType::kStop)) {
      return false;
    }
    frame.type = type;
    frame.session_id = session_id_;
    frame.sequence = next_out_sequence_++;
    frame.lease_id = lease_id_;
    frame.ttl_ms = ttl_ms;
    frame.payload = payload;
  }
  const auto encoded = MotionProtocol::encode(frame, psk_);
  return !encoded.empty() && sendAll(client_fd, encoded);
}

void GatewayNode::clearSession() {
  {
    std::lock_guard<std::mutex> lock(session_mutex_);
    session_active_ = false;
    telemetry_only_session_ = false;
    lease_requested_ = false;
    lease_active_ = false;
    session_id_ = 0U;
    lease_id_ = 0U;
    last_in_sequence_ = 0U;
    next_out_sequence_ = 1U;
    lease_deadline_ns_ = 0U;
    last_motion_sent_ns_ = 0U;
    last_heartbeat_sent_ns_ = 0U;
    motion_active_ = false;
  }
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    latest_command_ = VelocityCommand{};
    latest_command_valid_ = false;
    command_ns_ = 0U;
  }
  if (status_adapter_) status_adapter_->endSession();
}

bool GatewayNode::startLeaseRequest(int client_fd) {
  std::random_device random_device;
  uint32_t lease_id = (static_cast<uint32_t>(random_device()) << 1U) | 1U;
  if (lease_id == 0U) lease_id = 1U;
  {
    std::lock_guard<std::mutex> lock(session_mutex_);
    if (!session_active_ || !enable_motion_.load()) return false;
    telemetry_only_session_ = false;
    lease_requested_ = true;
    lease_active_ = false;
    lease_id_ = lease_id;
    lease_deadline_ns_ = 0U;
    last_motion_sent_ns_ = 0U;
    last_heartbeat_sent_ns_ = 0U;
    motion_active_ = false;
  }
  if (!sendMessage(client_fd, MessageType::kLeaseRequest, {}, kMaxLeaseTtlMs)) {
    ++lease_request_send_failed_count_;
    recordCloseReason(ConnectionCloseReason::kLeaseRequestSendFailed);
    return false;
  }
  ++lease_request_sent_count_;
  return true;
}

void GatewayNode::revokeSession(int client_fd, uint8_t error_code,
                                bool send_error, bool send_stop) {
  bool motion_session = false;
  {
    std::lock_guard<std::mutex> lock(session_mutex_);
    motion_session = session_active_ && !telemetry_only_session_ && lease_requested_;
  }
  if (send_error) sendMessage(client_fd, MessageType::kError, {error_code});
  if (send_stop && motion_session) sendMessage(client_fd, MessageType::kStop);
  publishZero();
  clearSession();
}

bool GatewayNode::handleStatus(const MotionFrame &frame) {
  if (frame.payload.size() == 1U) return true;
  if (frame.payload.size() != 1U + smartcar_state_bridge::kSrpV4ChassisFrameBytes ||
      frame.payload.front() != smartcar_state_bridge::kSrpV4ChassisMessageId) {
    return false;
  }
  smartcar_state_bridge::ChassisTelemetryFrame telemetry;
  telemetry.origin = smartcar_state_bridge::TelemetryOrigin::kLiveGateway;
  telemetry.payload.assign(frame.payload.begin() + 1U, frame.payload.end());
  telemetry.connection_epoch = connection_epoch_;
  telemetry.host_received_steady_ns = steadyNowNs();
  const auto result = status_adapter_->submit(telemetry);
  if (result.status == smartcar_state_bridge::SrpV4TelemetrySubmitStatus::kFrameDecodeRejected ||
      result.chassis_result.status == smartcar_state_bridge::ChassisSubmitStatus::kDecodeRejected ||
      result.chassis_result.status == smartcar_state_bridge::ChassisSubmitStatus::kSequenceRejected ||
      result.chassis_result.status == smartcar_state_bridge::ChassisSubmitStatus::kOdomRejected) {
    publishZero();
    return false;
  }
  const auto &update = result.chassis_result.odom_update;
  if (!status_odom_enabled_ || !update.state.valid ||
      (result.chassis_result.status != smartcar_state_bridge::ChassisSubmitStatus::kAnchored &&
       result.chassis_result.status != smartcar_state_bridge::ChassisSubmitStatus::kAccepted)) {
    return true;
  }
  smartcar_state_bridge::PlanarOdomData odom_data;
  odom_data.x_m = update.state.x_m;
  odom_data.y_m = update.state.y_m;
  odom_data.yaw_rad = update.state.yaw_rad;
  odom_data.linear_x_mps = update.state.linear_x_mps;
  odom_data.linear_y_mps = update.state.linear_y_mps;
  odom_data.angular_z_rps = update.state.angular_z_rps;
  const auto odom = smartcar_state_bridge::makeOdometryMessage(odom_data, now());
  status_odom_publisher_->publish(odom);
  if (status_tf_enabled_) status_tf_broadcaster_->sendTransform(
      smartcar_state_bridge::makeOdomTransform(odom));
  return true;
}

bool GatewayNode::handleFrame(const MotionFrame &frame, int client_fd) {
  const uint64_t now_ns = steadyNowNs();
  if (frame.type == MessageType::kHello) {
    if (frame.session_id == 0U || frame.lease_id != 0U || !validEmptyControl(frame)) {
      ++hello_invalid_count_;
      recordCloseReason(ConnectionCloseReason::kHelloInvalid);
      revokeSession(client_fd, kErrorMalformed, true, true);
      return false;
    }
    clearSession();
    {
      std::lock_guard<std::mutex> lock(session_mutex_);
      session_active_ = true;
      session_id_ = frame.session_id;
      last_in_sequence_ = frame.sequence;
      next_out_sequence_ = 1U;
      telemetry_only_session_ = !enable_motion_.load();
      lease_requested_ = false;
      ++connection_epoch_;
      if (connection_epoch_ == 0U) ++connection_epoch_;
    }
    status_adapter_->beginSession(connection_epoch_);
    if (!sendMessage(client_fd, MessageType::kHelloAck)) {
      ++hello_ack_send_failed_count_;
      recordCloseReason(ConnectionCloseReason::kHelloAckSendFailed);
      revokeSession(client_fd, kErrorLease, false, false);
      return false;
    }
    ++hello_ack_sent_count_;
    bool telemetry_only = false;
    {
      std::lock_guard<std::mutex> lock(session_mutex_);
      telemetry_only = telemetry_only_session_;
    }
    if (telemetry_only) {
      ++telemetry_only_session_count_;
      return true;
    }
    if (!startLeaseRequest(client_fd)) {
      revokeSession(client_fd, kErrorLease, false, false);
      return false;
    }
    return true;
  }

  bool wrong_session = false;
  bool bad_sequence = false;
  {
    std::lock_guard<std::mutex> lock(session_mutex_);
    wrong_session = !session_active_ || frame.session_id != session_id_;
    bad_sequence = !wrong_session && !sequenceIsNewer(frame.sequence, last_in_sequence_);
    if (!wrong_session && !bad_sequence) last_in_sequence_ = frame.sequence;
  }
  if (wrong_session) {
    recordCloseReason(ConnectionCloseReason::kSessionInvalid);
    revokeSession(client_fd, kErrorSession, true, true);
    return false;
  }
  if (bad_sequence) {
    recordCloseReason(ConnectionCloseReason::kSequenceInvalid);
    revokeSession(client_fd, kErrorSequence, true, true);
    return false;
  }

  if (frame.type == MessageType::kLeaseResponse) {
    if (frame.payload.size() != 3U || frame.payload[0] != 0U ||
        !validLeaseTtl(frame.ttl_ms)) {
      recordCloseReason(ConnectionCloseReason::kLeaseInvalid);
      revokeSession(client_fd, kErrorLease, true, true);
      return false;
    }
    const uint16_t payload_ttl = static_cast<uint16_t>(frame.payload[1]) |
                                 static_cast<uint16_t>(frame.payload[2] << 8U);
    bool lease_mismatch = false;
    {
      std::lock_guard<std::mutex> lock(session_mutex_);
      lease_mismatch = telemetry_only_session_ || !lease_requested_ ||
                       !enable_motion_.load() || frame.lease_id != lease_id_ ||
                       payload_ttl != frame.ttl_ms;
      if (!lease_mismatch) {
        lease_active_ = true;
        lease_deadline_ns_ = now_ns + static_cast<uint64_t>(frame.ttl_ms) * 1000000U;
      }
    }
    if (lease_mismatch) {
      recordCloseReason(ConnectionCloseReason::kLeaseInvalid);
      revokeSession(client_fd, kErrorLease, true, true);
      return false;
    }
    return true;
  }

  if (frame.type == MessageType::kStatus) {
    bool telemetry_only = false;
    {
      std::lock_guard<std::mutex> lock(session_mutex_);
      telemetry_only = telemetry_only_session_;
    }
    const bool valid_status_payload = frame.ttl_ms == 0U && (frame.payload.size() == 1U ||
        (frame.payload.size() == 1U + smartcar_state_bridge::kSrpV4ChassisFrameBytes &&
         frame.payload.front() == smartcar_state_bridge::kSrpV4ChassisMessageId)) &&
        (!telemetry_only || frame.lease_id == 0U);
    if (!valid_status_payload) {
      ++status_reject_count_;
      recordCloseReason(ConnectionCloseReason::kStatusInvalid);
      revokeSession(client_fd, kErrorStatus, true, true);
      return false;
    }
    if (!handleStatus(frame)) {
      ++status_reject_count_;
      recordCloseReason(ConnectionCloseReason::kStatusDecodeFailed);
      revokeSession(client_fd, kErrorStatus, true, true);
      return false;
    }
    ++status_received_count_;
    ++status_accept_count_;
    return true;
  }

  if (frame.type == MessageType::kError) {
    recordCloseReason(ConnectionCloseReason::kStatusInvalid);
    revokeSession(client_fd, kErrorStatus, false, true);
    return false;
  }

  recordCloseReason(ConnectionCloseReason::kLeaseInvalid);
  revokeSession(client_fd, kErrorMalformed, true, true);
  return false;
}

void GatewayNode::serviceOutbound(int client_fd) {
  const uint64_t now_ns = steadyNowNs();
  bool active_session = false;
  bool telemetry_only = false;
  bool active_lease = false;
  bool lease_expired = false;
  bool previous_motion = false;
  {
    std::lock_guard<std::mutex> lock(session_mutex_);
    active_session = session_active_;
    telemetry_only = telemetry_only_session_;
    active_lease = lease_active_;
    lease_expired = active_lease && now_ns > lease_deadline_ns_;
    previous_motion = motion_active_;
  }
  if (!active_session || telemetry_only) return;
  if (lease_expired) {
    ++lease_expired_count_;
    recordCloseReason(ConnectionCloseReason::kLeaseExpired);
    revokeSession(client_fd, kErrorLease, true, true);
    return;
  }
  if (!active_lease) return;

  VelocityCommand command;
  if (motionPermitted(now_ns, command) &&
      (last_motion_sent_ns_ == 0U || now_ns - last_motion_sent_ns_ >= command_period_ns_)) {
    const auto payload = encodeVelocityPayload(command.linear_mps, command.angular_rps);
    if (payload.empty() || !sendMessage(client_fd, MessageType::kMotionCommand,
                                        payload, kMaxLeaseTtlMs)) {
      recordCloseReason(ConnectionCloseReason::kOutboundSendFailed);
      revokeSession(client_fd, kErrorLease, false, false);
      return;
    }
    std::lock_guard<std::mutex> lock(session_mutex_);
    last_motion_sent_ns_ = now_ns;
    motion_active_ = command.linear_mps != 0.0F || command.angular_rps != 0.0F;
    lease_deadline_ns_ = now_ns + static_cast<uint64_t>(kMaxLeaseTtlMs) * 1000000U;
    return;
  }
  if (previous_motion) {
    if (!sendMessage(client_fd, MessageType::kStop)) {
      recordCloseReason(ConnectionCloseReason::kOutboundSendFailed);
      revokeSession(client_fd, kErrorLease, false, false);
      return;
    }
    std::lock_guard<std::mutex> lock(session_mutex_);
    motion_active_ = false;
  }
  if (last_heartbeat_sent_ns_ == 0U ||
      now_ns - last_heartbeat_sent_ns_ >= heartbeat_period_ns_) {
    if (!sendMessage(client_fd, MessageType::kHeartbeat)) {
      recordCloseReason(ConnectionCloseReason::kOutboundSendFailed);
      revokeSession(client_fd, kErrorLease, false, false);
      return;
    }
    std::lock_guard<std::mutex> lock(session_mutex_);
    last_heartbeat_sent_ns_ = now_ns;
    lease_deadline_ns_ = now_ns + static_cast<uint64_t>(kMaxLeaseTtlMs) * 1000000U;
  }
}

void GatewayNode::handleClient(int client_fd) {
  StreamParser parser;
  std::array<uint8_t, kMotionMaxFrameBytes> receive_buffer{};
  while (running_) {
    pollfd descriptor{};
    descriptor.fd = client_fd;
    descriptor.events = POLLIN;
    const int ready = poll(&descriptor, 1U, 20);
    if (ready < 0 || (descriptor.revents & (POLLERR | POLLNVAL)) != 0) {
      ++poll_error_count_;
      recordCloseReason(ConnectionCloseReason::kPollError);
      break;
    }
    if ((descriptor.revents & POLLHUP) != 0) {
      ++poll_hup_count_;
      recordCloseReason(ConnectionCloseReason::kPollHup);
      break;
    }
    if ((descriptor.revents & POLLIN) != 0) {
      const ssize_t received = recv(client_fd, receive_buffer.data(), receive_buffer.size(), 0);
      if (received == 0) {
        ++recv_zero_count_;
        recordCloseReason(ConnectionCloseReason::kRecvZero);
        break;
      }
      if (received < 0) {
        ++recv_error_count_;
        recordCloseReason(ConnectionCloseReason::kRecvError);
        break;
      }
      const auto results = parser.push(receive_buffer.data(),
                                       static_cast<std::size_t>(received), psk_);
      for (const auto &result : results) {
        if (!result.accepted()) {
          recordDecodeFailure(result.status);
          revokeSession(client_fd, kErrorMalformed, true, true);
          return;
        }
        if (!handleFrame(result.frame, client_fd)) return;
      }
    }
    serviceOutbound(client_fd);
  }
  publishZero();
  clearSession();
}

void GatewayNode::serverLoop() {
  const int fd = socket(AF_INET, SOCK_STREAM, 0);
  if (fd < 0) {
    RCLCPP_ERROR(get_logger(), "cannot create TCP 8766 listener");
    return;
  }
  int reuse = 1;
  setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_port = htons(listen_port_);
  if (inet_pton(AF_INET, listen_address_.c_str(), &address.sin_addr) != 1 ||
      bind(fd, reinterpret_cast<sockaddr *>(&address), sizeof(address)) != 0 ||
      listen(fd, 1) != 0) {
    RCLCPP_ERROR(get_logger(), "cannot bind TCP motion listener on port %u", listen_port_);
    close(fd);
    return;
  }
  {
    std::lock_guard<std::mutex> lock(socket_mutex_);
    server_fd_ = fd;
  }
  while (running_) {
    pollfd listener{};
    listener.fd = fd;
    listener.events = POLLIN;
    if (poll(&listener, 1U, 100) <= 0 || (listener.revents & POLLIN) == 0) continue;
    sockaddr_in peer{};
    socklen_t peer_size = sizeof(peer);
    const int accepted = accept(fd, reinterpret_cast<sockaddr *>(&peer), &peer_size);
    if (accepted < 0) continue;
    ++accept_count_;
    last_close_reason_.store(ConnectionCloseReason::kNone);
    RCLCPP_INFO(get_logger(), "motion connection accepted");
    {
      std::lock_guard<std::mutex> lock(socket_mutex_);
      client_fd_ = accepted;
    }
    handleClient(accepted);
    if (last_close_reason_.load() == ConnectionCloseReason::kNone) {
      recordCloseReason(ConnectionCloseReason::kHandlerReturned);
    }
    RCLCPP_INFO(get_logger(), "motion connection closed: reason=%s",
                toString(last_close_reason_.load()));
    {
      std::lock_guard<std::mutex> lock(socket_mutex_);
      if (client_fd_ == accepted) client_fd_ = -1;
    }
    shutdown(accepted, SHUT_RDWR);
    close(accepted);
  }
}

}  // namespace smartcar_motion_gateway
