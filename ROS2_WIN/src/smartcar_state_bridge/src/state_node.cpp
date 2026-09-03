#include "smartcar_state_bridge/state_node.hpp"

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <utility>
#include <vector>

namespace smartcar_state_bridge {

namespace {

uint64_t steadyNowNs() noexcept {
  const auto now = std::chrono::steady_clock::now().time_since_epoch();
  return static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(now).count());
}

template <typename T>
T readOrDeclareParameter(rclcpp::Node &node, const char *name,
                         const T &default_value) {
  if (node.has_parameter(name)) {
    T value = default_value;
    if (node.get_parameter(name, value)) {
      return value;
    }
  }
  return node.declare_parameter<T>(name, default_value);
}

int64_t readTelemetryIdentityParameter(rclcpp::Node &node,
                                       const char *canonical_name,
                                       const char *legacy_name) {
  if (node.has_parameter(canonical_name)) {
    return node.get_parameter(canonical_name).as_int();
  }
  if (node.has_parameter(legacy_name)) {
    RCLCPP_WARN(node.get_logger(),
                "%s is deprecated; use %s", legacy_name, canonical_name);
    return node.get_parameter(legacy_name).as_int();
  }
  return node.declare_parameter<int64_t>(canonical_name, -1);
}

void addKey(diagnostic_msgs::msg::DiagnosticStatus &status,
            const std::string &key, const std::string &value) {
  diagnostic_msgs::msg::KeyValue item;
  item.key = key;
  item.value = value;
  status.values.push_back(std::move(item));
}

template <typename T>
void addNumber(diagnostic_msgs::msg::DiagnosticStatus &status,
               const std::string &key, T value) {
  addKey(status, key, std::to_string(value));
}

WheelOdomConfig readOdomConfig(rclcpp::Node &node) {
  WheelOdomConfig config;
  const double track_width =
      node.declare_parameter("track_width_m", config.track_width_m);
  const double diameter =
      node.declare_parameter("wheel_diameter_m", config.wheel_diameter_m);
  const double tick_period = node.declare_parameter(
      "sample_tick_period_s", config.sample_tick_period_s);
  const double min_dt = node.declare_parameter("min_dt_s", config.min_dt_s);
  const double max_dt = node.declare_parameter("max_dt_s", config.max_dt_s);
  const int64_t stale_ms = node.declare_parameter(
      "stale_timeout_ms", static_cast<int64_t>(config.stale_timeout_ms));
  const int64_t fifo_depth = node.declare_parameter(
      "wheel_fifo_depth", static_cast<int64_t>(config.fifo_depth));
  config.track_width_m = track_width;
  config.wheel_diameter_m = diameter;
  config.sample_tick_period_s = tick_period;
  config.min_dt_s = min_dt;
  config.max_dt_s = max_dt;
  config.stale_timeout_ms = stale_ms > 0 &&
                                    stale_ms <= std::numeric_limits<uint32_t>::max()
                                ? static_cast<uint32_t>(stale_ms)
                                : config.stale_timeout_ms;
  config.fifo_depth = fifo_depth > 0 ? static_cast<std::size_t>(fifo_depth) : 0U;
  config.require_source_freshness = readOrDeclareParameter(
      node, "require_source_freshness", config.require_source_freshness);
  config.allow_sequence_wrap = node.declare_parameter(
      "allow_sequence_wrap", config.allow_sequence_wrap);
  config.allow_tick_wrap =
      node.declare_parameter("allow_tick_wrap", config.allow_tick_wrap);

  const std::vector<double> signs = node.declare_parameter(
      "wheel_speed_sign", std::vector<double>{1.0, 1.0, 1.0, 1.0});
  if (signs.size() == kWheelCount) {
    std::copy(signs.begin(), signs.end(), config.wheel_speed_sign.begin());
  } else {
    RCLCPP_WARN(node.get_logger(),
                "wheel_speed_sign must contain RR, RF, LR, LF; using defaults");
  }
  return config;
}

TelemetryDecoderConfig readDecoderConfig(rclcpp::Node &node) {
  TelemetryDecoderConfig config;
  config.allow_live =
      node.declare_parameter("allow_live_telemetry", config.allow_live);
  config.allow_offline_fixtures = node.declare_parameter(
      "allow_offline_fixtures", config.allow_offline_fixtures);
  config.require_source_freshness = node.declare_parameter(
      "require_source_freshness", config.require_source_freshness);
  const int64_t max_payload = node.declare_parameter(
      "telemetry_max_payload_bytes",
      static_cast<int64_t>(config.max_payload_bytes));
  if (max_payload > 0 &&
      max_payload <= static_cast<int64_t>(kScbpMaximumFrameBytes)) {
    config.max_payload_bytes = static_cast<std::size_t>(max_payload);
  } else {
    RCLCPP_WARN(node.get_logger(),
                "telemetry_max_payload_bytes must be in 1..%zu; using %zu",
                kScbpMaximumFrameBytes, config.max_payload_bytes);
  }
  const int64_t source_id = readTelemetryIdentityParameter(
      node, "telemetry_expected_source_id", "telemetry_source_id");
  if (source_id >= 0 && source_id <= std::numeric_limits<uint16_t>::max()) {
    config.expected_source_id = static_cast<uint16_t>(source_id);
  }
  const int64_t destination_id = readTelemetryIdentityParameter(
      node, "telemetry_expected_destination_id", "telemetry_destination_id");
  if (destination_id >= 0 &&
      destination_id <= std::numeric_limits<uint16_t>::max()) {
    config.expected_destination_id = static_cast<uint16_t>(destination_id);
  }
  return config;
}

}  // namespace

StateNode::StateNode(const rclcpp::NodeOptions &options)
    : Node("smartcar_state_bridge", options) {
  odom_topic_ = declare_parameter("odom_topic", odom_topic_);
  diagnostics_topic_ = declare_parameter("diagnostics_topic", diagnostics_topic_);
  frame_id_ = declare_parameter("frame_id", frame_id_);
  child_frame_id_ = declare_parameter("child_frame_id", child_frame_id_);
  enable_live_odom_ =
      declare_parameter("enable_live_odom", enable_live_odom_);
  publish_odom_ = declare_parameter("publish_odom", publish_odom_);
  publish_tf_ = declare_parameter("publish_tf", publish_tf_);

  // Parameters are read once here so a runtime reconfiguration cannot bypass
  // the explicit session/freshness gate.
  StateAdapterConfig adapter_config;
  adapter_config.decoder = readDecoderConfig(*this);
  adapter_config.odom = readOdomConfig(*this);
  adapter_config.enable_live_odom = enable_live_odom_;
  adapter_ = StateAdapter(std::move(adapter_config));

  diagnostics_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      diagnostics_topic_, rclcpp::QoS(10));
  if (publish_odom_) {
    odom_publisher_ =
        create_publisher<nav_msgs::msg::Odometry>(odom_topic_, rclcpp::QoS(10));
  }
  if (publish_tf_) {
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  }
  diagnostics_timer_ = create_wall_timer(
      std::chrono::milliseconds(250),
      std::bind(&StateNode::publishDiagnostics, this));

  if (enable_live_odom_) {
    RCLCPP_WARN(get_logger(),
                "enable_live_odom is requested; only an approved gateway parser may submit live samples");
  }
  if (!publish_odom_) {
    RCLCPP_INFO(get_logger(),
                "odom publication is disabled by default; this node owns no live transport");
  }
}

StateSubmitResult StateNode::submitTelemetry(
    const TelemetryEnvelope &envelope) {
  std::lock_guard<std::mutex> lock(mutex_);
  StateSubmitResult result = adapter_.submitTelemetry(envelope);
  last_decode_status_ = result.decode_status;
  if (result.odom_update.status == OdomUpdateStatus::kAccepted ||
      result.odom_update.status == OdomUpdateStatus::kAnchored) {
    ++submitted_samples_;
  }
  if (result.odom_update.status == OdomUpdateStatus::kAccepted &&
      odom_publisher_) {
    publishOdometry(result.odom_update);
    ++published_samples_;
  }
  if (result.decode_status != TelemetryDecodeStatus::kAccepted ||
      (result.odom_update.status != OdomUpdateStatus::kAccepted &&
       result.odom_update.status != OdomUpdateStatus::kAnchored)) {
    ++rejected_samples_;
  }
  return result;
}

void StateNode::publishOdometry(const OdomUpdate &update) {
  if (!odom_publisher_ || update.status != OdomUpdateStatus::kAccepted) {
    return;
  }
  nav_msgs::msg::Odometry message;
  message.header.stamp = now();
  message.header.frame_id = frame_id_;
  message.child_frame_id = child_frame_id_;
  message.pose.pose.position.x = update.state.x_m;
  message.pose.pose.position.y = update.state.y_m;
  message.pose.pose.position.z = 0.0;
  message.pose.pose.orientation.z = std::sin(update.state.heading_rad * 0.5);
  message.pose.pose.orientation.w = std::cos(update.state.heading_rad * 0.5);
  message.twist.twist.linear.x = update.state.linear_mps;
  message.twist.twist.angular.z = update.state.angular_rps;

  // Conservative covariance until wheel radius, track width, and slip have
  // been measured.  Unobserved planar axes remain explicitly unknown.
  message.pose.covariance.fill(0.0);
  message.twist.covariance.fill(0.0);
  message.pose.covariance[0] = 1.0;
  message.pose.covariance[7] = 1.0;
  message.pose.covariance[35] = 1.0;
  message.pose.covariance[14] = 1.0e6;
  message.pose.covariance[21] = 1.0e6;
  message.pose.covariance[28] = 1.0e6;
  message.twist.covariance[0] = 0.25;
  message.twist.covariance[35] = 0.25;
  message.twist.covariance[7] = 1.0e6;
  message.twist.covariance[14] = 1.0e6;
  message.twist.covariance[21] = 1.0e6;
  message.twist.covariance[28] = 1.0e6;
  odom_publisher_->publish(message);

  if (tf_broadcaster_) {
    geometry_msgs::msg::TransformStamped transform;
    transform.header = message.header;
    transform.child_frame_id = child_frame_id_;
    transform.transform.translation.x = update.state.x_m;
    transform.transform.translation.y = update.state.y_m;
    transform.transform.translation.z = 0.0;
    transform.transform.rotation = message.pose.pose.orientation;
    tf_broadcaster_->sendTransform(transform);
  }
}

void StateNode::publishDiagnostics() {
  std::lock_guard<std::mutex> lock(mutex_);
  adapter_.odom().checkStale(steadyNowNs());
  const auto decoder_counters = adapter_.decoder().counters();
  const auto &odom_state = adapter_.odom().state();
  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = now();
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "smartcar_state_bridge";
  status.hardware_id = "stm32_wheel_telemetry";
  status.level = odom_state.invalid_latched
                     ? diagnostic_msgs::msg::DiagnosticStatus::ERROR
                     : (submitted_samples_ == 0U
                            ? diagnostic_msgs::msg::DiagnosticStatus::WARN
                            : diagnostic_msgs::msg::DiagnosticStatus::OK);
  status.message = odom_state.invalid_latched
                       ? "odom_invalid"
                       : (submitted_samples_ == 0U ? "no_samples" :
                                                       toString(adapter_.odom().lastStatus()));
  addKey(status, "telemetry_protocol", adapter_.decoder().config().allow_live
                                             ? "explicit_live_opt_in"
                                             : "offline_or_disabled");
  addKey(status, "last_telemetry_status",
         telemetryStatusString(last_decode_status_));
  addKey(status, "odom_status", toString(adapter_.odom().lastStatus()));
  addKey(status, "odom_invalid", odom_state.invalid_latched ? "true" : "false");
  addKey(status, "live_odom_enabled", enable_live_odom_ ? "true" : "false");
  addKey(status, "publish_odom", publish_odom_ ? "true" : "false");
  addNumber(status, "submitted_samples", submitted_samples_);
  addNumber(status, "published_samples", published_samples_);
  addNumber(status, "rejected_samples", rejected_samples_);
  addNumber(status, "decoder_accepted", decoder_counters.accepted);
  addNumber(status, "decoder_disabled", decoder_counters.disabled);
  addNumber(status, "decoder_not_configured", decoder_counters.not_configured);
  addNumber(status, "decoder_invalid_samples", decoder_counters.invalid_samples);
  addNumber(status, "decoder_identity_errors", decoder_counters.identity_errors);
  addNumber(status, "fifo_depth", adapter_.odom().fifo().depth());
  addNumber(status, "fifo_size", adapter_.odom().fifo().size());
  addNumber(status, "fifo_overflow", adapter_.odom().fifo().overflowCount());
  addNumber(status, "fifo_invalid", adapter_.odom().fifo().invalidCount());
  addNumber(status, "odom_accepted_samples", odom_state.accepted_samples);
  addKey(status, "session_epoch",
         odom_state.session_epoch.has_value()
             ? std::to_string(*odom_state.session_epoch)
             : std::string("unset"));
  addKey(status, "last_sample_seq",
         odom_state.last_sample_seq.has_value()
             ? std::to_string(*odom_state.last_sample_seq)
             : std::string("unset"));
  addKey(status, "last_sample_tick",
         odom_state.last_sample_tick.has_value()
             ? std::to_string(*odom_state.last_sample_tick)
             : std::string("unset"));
  addKey(status, "last_host_received_steady_ns",
         odom_state.last_host_received_steady_ns == 0U
             ? std::string("never")
             : std::to_string(odom_state.last_host_received_steady_ns));
  const uint64_t now_steady_ns = steadyNowNs();
  addKey(status, "last_sample_age_ns",
         odom_state.last_host_received_steady_ns == 0U ||
                 now_steady_ns < odom_state.last_host_received_steady_ns
             ? std::string("unknown")
             : std::to_string(now_steady_ns -
                              odom_state.last_host_received_steady_ns));
  addKey(status, "source_time_s",
         odom_state.last_source_time_s.has_value()
             ? std::to_string(*odom_state.last_source_time_s)
             : std::string("unset"));
  addKey(status, "source_age_ms",
         odom_state.last_source_age_ms.has_value()
             ? std::to_string(*odom_state.last_source_age_ms)
             : std::string("unset"));
  addKey(status, "source_epoch",
         odom_state.source_epoch.has_value()
             ? std::to_string(*odom_state.source_epoch)
             : std::string("unset"));
  array.status.push_back(std::move(status));
  diagnostics_publisher_->publish(std::move(array));
}

}  // namespace smartcar_state_bridge

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<smartcar_state_bridge::StateNode>());
  rclcpp::shutdown();
  return 0;
}
