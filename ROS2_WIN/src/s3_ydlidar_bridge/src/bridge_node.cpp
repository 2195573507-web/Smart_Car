#include "s3_ydlidar_bridge/bridge_node.hpp"

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"

#include <algorithm>
#include <cinttypes>
#include <chrono>
#include <cstdint>
#include <cmath>
#include <limits>
#include <map>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace s3_ydlidar_bridge {

namespace {

ScanMapperConfig readMapperConfig(rclcpp::Node &node) {
  ScanMapperConfig config;
  config.frame_id = node.declare_parameter("frame_id", config.frame_id);
  const double angle_min = node.declare_parameter(
      "angle_min", static_cast<double>(config.angle_min));
  const double angle_max = node.declare_parameter(
      "angle_max", static_cast<double>(config.angle_max));
  const double range_min = node.declare_parameter(
      "range_min", static_cast<double>(config.range_min));
  const double range_max = node.declare_parameter(
      "range_max", static_cast<double>(config.range_max));
  const int64_t samples = node.declare_parameter(
      "samples", static_cast<int64_t>(config.samples));
  const double scan_frequency_hz = node.declare_parameter(
      "scan_frequency_hz", static_cast<double>(config.scan_frequency_hz));
  config.angle_min = static_cast<float>(angle_min);
  config.angle_max = static_cast<float>(angle_max);
  config.range_min = static_cast<float>(range_min);
  config.range_max = static_cast<float>(range_max);
  if (samples > 0) {
    config.samples = static_cast<size_t>(samples);
  } else {
    config.samples = 0U;
  }
  config.scan_frequency_hz = static_cast<float>(scan_frequency_hz);
  config.invalid_range_is_inf = node.declare_parameter(
      "invalid_range_is_inf", config.invalid_range_is_inf);
  config.publish_intensities = node.declare_parameter(
      "publish_intensities", config.publish_intensities);
  return config;
}

uint32_t readU32Parameter(rclcpp::Node &node, const char *name,
                          uint32_t fallback) {
  const int64_t value = node.declare_parameter(name,
                                                static_cast<int64_t>(fallback));
  if (value < 0 || value > static_cast<int64_t>(UINT32_MAX)) {
    RCLCPP_WARN(node.get_logger(), "%s outside uint32 range; using %u", name,
                fallback);
    return fallback;
  }
  return static_cast<uint32_t>(value);
}

std::vector<uint8_t> readByteListParameter(rclcpp::Node &node,
                                           const char *name,
                                           const std::vector<int64_t> &defaults) {
  std::vector<int64_t> values;
  if (node.has_parameter(name)) {
    rclcpp::Parameter parameter;
    if (!node.get_parameter(name, parameter) ||
        parameter.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
      // Humble represents an explicitly empty YAML sequence as NOT_SET.
      return {};
    }
    if (parameter.get_type() !=
        rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY) {
      RCLCPP_WARN(node.get_logger(),
                  "%s must be an integer array; ignoring the override", name);
      return {};
    }
    values = parameter.as_integer_array();
  } else {
    try {
      values = node.declare_parameter<std::vector<int64_t>>(
          name, defaults);
    } catch (const rclcpp::exceptions::InvalidParameterValueException &error) {
      // An unset/empty override can still reach declare_parameter as NOT_SET
      // on Humble.  Treat it as the documented empty allow-list.
      RCLCPP_WARN(node.get_logger(), "%s ignored: %s", name, error.what());
      return {};
    }
  }
  std::vector<uint8_t> result;
  result.reserve(values.size());
  for (const int64_t value : values) {
    if (value < 0 || value > 255) {
      RCLCPP_WARN(node.get_logger(),
                  "%s contains a value outside 0..255; ignoring it", name);
      continue;
    }
    result.push_back(static_cast<uint8_t>(value));
  }
  return result;
}

uint8_t readU8Parameter(rclcpp::Node &node, const char *name,
                        uint8_t fallback) {
  const int64_t value =
      node.declare_parameter(name, static_cast<int64_t>(fallback));
  if (value < 0 || value > 255) {
    RCLCPP_WARN(node.get_logger(), "%s outside uint8 range; using %u", name,
                static_cast<unsigned>(fallback));
    return fallback;
  }
  return static_cast<uint8_t>(value);
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
}  // namespace

BridgeNode::BridgeNode()
    : Node("s3_ydlidar_bridge"),
      mapper_(readMapperConfig(*this)) {
  const std::string topic = declare_parameter("scan_topic", std::string("/scan"));
  const std::string diagnostics_topic =
      declare_parameter("diagnostics_topic", std::string("/diagnostics"));
  const int64_t stale_after_ms = declare_parameter(
      "stale_after_ms", static_cast<int64_t>(stale_after_ms_));
  if (stale_after_ms > std::numeric_limits<int>::max()) {
    stale_after_ms_ = std::numeric_limits<int>::max();
  } else if (stale_after_ms < std::numeric_limits<int>::min()) {
    stale_after_ms_ = std::numeric_limits<int>::min();
  } else {
    stale_after_ms_ = static_cast<int>(stale_after_ms);
  }
  const int64_t zero_packet_timeout_ms = declare_parameter(
      "zero_packet_timeout_ms", static_cast<int64_t>(zero_packet_timeout_ms_));
  if (zero_packet_timeout_ms > 0 &&
      zero_packet_timeout_ms <= std::numeric_limits<int>::max()) {
    zero_packet_timeout_ms_ = static_cast<int>(zero_packet_timeout_ms);
  } else {
    RCLCPP_WARN(get_logger(),
                "zero_packet_timeout_ms must be in 1..%d; using %d",
                std::numeric_limits<int>::max(), zero_packet_timeout_ms_);
  }
  const std::string transport =
      declare_parameter("transport", std::string("unconfigured"));
  const std::string replay_file = declare_parameter("replay_file", std::string());
  decoder_.setIntensities(
      declare_parameter("ydlidar_intensities", false));

  publisher_ = create_publisher<sensor_msgs::msg::LaserScan>(
      topic, rclcpp::SensorDataQoS());
  diagnostics_publisher_ =
      create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
          diagnostics_topic, rclcpp::QoS(10));

  if (transport == "replay" || !replay_file.empty()) {
    transport_ = std::make_unique<ReplayTransport>(replay_file);
  } else if (transport == "tcp") {
    TcpServerConfig tcp_config;
    tcp_config.listen_address = declare_parameter(
        "tcp_listen_address", tcp_config.listen_address);
    const int64_t port = declare_parameter(
        "tcp_listen_port", static_cast<int64_t>(tcp_config.listen_port));
    if (port > 0 && port <= 65535) {
      tcp_config.listen_port = static_cast<uint16_t>(port);
    } else {
      RCLCPP_WARN(get_logger(), "tcp_listen_port outside range; using %u",
                  tcp_config.listen_port);
    }
    const int64_t max_buffer = declare_parameter(
        "tcp_max_buffer_bytes", static_cast<int64_t>(tcp_config.max_buffer_bytes));
    if (max_buffer >= 0) {
      tcp_config.max_buffer_bytes = static_cast<size_t>(max_buffer);
    }
    const int64_t max_ready = declare_parameter(
        "tcp_max_ready_frames", static_cast<int64_t>(tcp_config.max_ready_frames));
    if (max_ready >= 0) {
      tcp_config.max_ready_frames = static_cast<size_t>(max_ready);
    }
    tcp_config.protocol.expected_version =
        readU8Parameter(*this, "s3_expected_version",
                        tcp_config.protocol.expected_version);
    tcp_config.protocol.expected_message_type =
        readU8Parameter(*this, "s3_expected_message_type",
                        tcp_config.protocol.expected_message_type);
    tcp_config.protocol.allowed_flags_mask = static_cast<uint16_t>(
        readU32Parameter(*this, "s3_allowed_flags_mask",
                         tcp_config.protocol.allowed_flags_mask));
    tcp_config.protocol.expected_device_id =
        readU32Parameter(*this, "s3_expected_device_id",
                         tcp_config.protocol.expected_device_id);
    tcp_config.protocol.expected_stream_id =
        readU32Parameter(*this, "s3_expected_stream_id",
                         tcp_config.protocol.expected_stream_id);
    tcp_config.protocol.opaque_message_types =
        readByteListParameter(*this, "s3_opaque_message_types",
                              {});
    const int64_t opaque_min_payload = declare_parameter(
        "s3_opaque_min_payload_bytes",
        static_cast<int64_t>(tcp_config.protocol.opaque_min_payload_bytes));
    if (opaque_min_payload >= 0 && opaque_min_payload <= 65535) {
      tcp_config.protocol.opaque_min_payload_bytes =
          static_cast<size_t>(opaque_min_payload);
    }
    const int64_t expected_policy = declare_parameter(
        "s3_expected_message_policy",
        static_cast<int64_t>(S3MessageTypePolicy::kRawYdlidar));
    if (expected_policy ==
        static_cast<int64_t>(S3MessageTypePolicy::kOpaque)) {
      tcp_config.protocol.message_type_policy = S3MessageTypePolicy::kOpaque;
    } else if (expected_policy !=
               static_cast<int64_t>(S3MessageTypePolicy::kRawYdlidar)) {
      RCLCPP_WARN(get_logger(),
                  "s3_expected_message_policy must be 0 (raw) or 1 (opaque); using raw");
    }
    const int64_t min_payload = declare_parameter(
        "s3_min_payload_bytes",
        static_cast<int64_t>(tcp_config.protocol.min_payload_bytes));
    if (min_payload >= 10 && min_payload <= 65535) {
      tcp_config.protocol.min_payload_bytes = static_cast<size_t>(min_payload);
    } else {
      RCLCPP_WARN(get_logger(),
                  "s3_min_payload_bytes must be in 10..65535; using %zu",
                  tcp_config.protocol.min_payload_bytes);
    }
    const int64_t max_payload = declare_parameter(
        "s3_max_payload_bytes",
        static_cast<int64_t>(tcp_config.protocol.max_payload_bytes));
    if (max_payload > 0 && max_payload <= 65535) {
      tcp_config.protocol.max_payload_bytes = static_cast<size_t>(max_payload);
    }
    if (tcp_config.protocol.min_payload_bytes >
        tcp_config.protocol.max_payload_bytes) {
      RCLCPP_ERROR(get_logger(),
                   "s3_min_payload_bytes exceeds s3_max_payload_bytes");
    }
    transport_ = std::make_unique<TcpServerTransport>(std::move(tcp_config));
  } else {
    transport_ = std::make_unique<UnconfiguredTransport>();
  }

  transport_->setConnectionCallback(
      [this](ConnectionEvent event) { onConnectionEvent(event); });
  std::string error;
  if (!transport_->start(
          [this](ReceivedFrame frame) { onFrame(std::move(frame)); }, error)) {
    RCLCPP_ERROR(get_logger(), "%s", error.c_str());
  }
  diagnostics_timer_ = create_wall_timer(std::chrono::seconds(1),
                                         [this]() { publishDiagnostics(); });
}

BridgeNode::~BridgeNode() {
  if (transport_) {
    transport_->stop();
  }
}

void BridgeNode::onConnectionEvent(ConnectionEvent event) {
  {
    std::lock_guard<std::mutex> lock(sequence_mutex_);
    sequence_connection_epoch_ = event.type == ConnectionEventType::kOpened
                                      ? event.connection_epoch
                                      : 0U;
    if (event.type == ConnectionEventType::kOpened) {
      sequence_tracker_.beginConnection(event.connection_epoch);
    } else {
      sequence_tracker_.endConnection(event.connection_epoch);
    }
  }
  {
    std::lock_guard<std::mutex> lock(accumulation_mutex_);
    const auto reset = mapper_.resetAccumulation();
    if (reset.partial_revolution_dropped) {
      ++partial_revolutions_dropped_;
      ++incomplete_revolutions_;
    }
    accumulation_epoch_ = event.type == ConnectionEventType::kOpened
                              ? event.connection_epoch
                              : 0U;
  }
}

void BridgeNode::onFrame(ReceivedFrame frame) {
  if (stale_after_ms_ > 0) {
    const auto now = std::chrono::steady_clock::now().time_since_epoch();
    const uint64_t now_ns = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(now).count());
    const uint64_t age_ns = now_ns >= frame.received_steady_ns
                                ? now_ns - frame.received_steady_ns
                                : 0U;
    if (age_ns > static_cast<uint64_t>(stale_after_ms_) * 1000000U) {
      RCLCPP_WARN(get_logger(), "dropping stale S3RD frame (%" PRIu64 " ns)",
                  age_ns);
      ++stale_frames_;
      return;
    }
  }

  uint64_t sequence_gap = 0U;
  if (frame.sequence.has_value()) {
    SequenceStatus sequence_status;
    {
      std::lock_guard<std::mutex> lock(sequence_mutex_);
      if (sequence_connection_epoch_ != 0U &&
          sequence_tracker_.snapshot().connection_epoch !=
              sequence_connection_epoch_) {
        sequence_tracker_.beginConnection(sequence_connection_epoch_);
      }
      const SequenceSnapshot before = sequence_tracker_.snapshot();
      sequence_status = sequence_tracker_.observe(*frame.sequence, frame.flags);
      if ((sequence_status == SequenceStatus::kJump ||
           sequence_status == SequenceStatus::kWrap) &&
          before.last_sequence.has_value()) {
        const uint32_t previous = static_cast<uint32_t>(*before.last_sequence);
        const uint32_t current = static_cast<uint32_t>(*frame.sequence);
        const uint32_t delta = current - previous;
        sequence_gap = delta == 0U ? 0U : static_cast<uint64_t>(delta - 1U);
      }
    }
    if (sequence_status == SequenceStatus::kDuplicate ||
        sequence_status == SequenceStatus::kOutOfOrder) {
      if (sequence_status == SequenceStatus::kDuplicate) {
        ++duplicate_sequences_;
      } else {
        ++out_of_order_sequences_;
      }
      return;
    }
    if (sequence_status == SequenceStatus::kJump || sequence_gap != 0U) {
      ++sequence_jumps_;
    }
    if (sequence_gap != 0U) {
      sequence_gaps_.fetch_add(sequence_gap);
    }
    if (sequence_status == SequenceStatus::kWrap) {
      ++sequence_wraps_;
    }
  }

  // TCP 8765 is scan-only. An explicitly configured opaque type is discarded
  // without decoding or publishing any telemetry, odometry, or TF.
  if (frame.isOpaque()) {
    ++opaque_frames_;
    return;
  }

  std::vector<::node_info> nodes;
  std::string error;
  if (!decoder_.decode(frame.payload.data(), frame.payload.size(), nodes, error)) {
    ++ydlidar_errors_;
    RCLCPP_ERROR(get_logger(), "%s", error.c_str());
    return;
  }
  ++decoded_packets_;
  if (frame.zero_packet) {
    ++zero_packets_;
  }

  const auto valid_packet_now = std::chrono::steady_clock::now().time_since_epoch();
  const uint64_t valid_packet_ns = static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(valid_packet_now).count());
  last_valid_packet_ns_.store(valid_packet_ns);

  ScanAccumulationResult accumulation;
  {
    std::lock_guard<std::mutex> lock(accumulation_mutex_);
    if (frame.connection_epoch != 0U &&
        accumulation_epoch_ != frame.connection_epoch) {
      const auto reset = mapper_.resetAccumulation();
      if (reset.partial_revolution_dropped) {
        ++partial_revolutions_dropped_;
        ++incomplete_revolutions_;
      }
      accumulation_epoch_ = frame.connection_epoch;
    }
    const auto expired = mapper_.expire(
        valid_packet_ns,
        static_cast<uint64_t>(zero_packet_timeout_ms_) * 1000000U);
    if (expired.partial_revolution_dropped) {
      ++partial_revolutions_dropped_;
      ++incomplete_revolutions_;
    }
    if (expired.zero_timeout) {
      ++scan_timeouts_;
    }
    accumulation = mapper_.accumulate(nodes, frame.zero_packet, sequence_gap,
                                      frame.received_steady_ns, now());
  }
  ++accumulated_packets_;
  valid_points_.fetch_add(accumulation.valid_points);
  if (accumulation.partial_revolution_dropped) {
    ++partial_revolutions_dropped_;
    ++incomplete_revolutions_;
  }
  if (accumulation.zero_timeout) {
    ++scan_timeouts_;
  }
  for (auto &scan : accumulation.completed_scans) {
    publisher_->publish(scan);
    ++published_scans_;
    ++revolutions_published_;
    last_valid_scan_ns_.store(valid_packet_ns);
  }
}

void BridgeNode::publishDiagnostics() {
  const auto transport_stats = transport_ ? transport_->stats() : TransportStats{};
  const auto now_steady = std::chrono::steady_clock::now().time_since_epoch();
  const uint64_t now_ns = static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(now_steady).count());
  const uint64_t last_valid = last_valid_packet_ns_.load();
  const uint64_t last_valid_scan = last_valid_scan_ns_.load();
  SequenceSnapshot sequence_snapshot;
  std::size_t sequence_domain_count = 0U;
  {
    std::lock_guard<std::mutex> lock(sequence_mutex_);
    sequence_snapshot = sequence_tracker_.snapshot();
    sequence_domain_count = sequence_snapshot.first_sequence.has_value() ? 1U : 0U;
  }
  ScanAccumulationSnapshot accumulation_snapshot;
  {
    std::lock_guard<std::mutex> lock(accumulation_mutex_);
    const auto expired = mapper_.expire(
        now_ns, static_cast<uint64_t>(zero_packet_timeout_ms_) * 1000000U);
    if (expired.partial_revolution_dropped) {
      ++partial_revolutions_dropped_;
      ++incomplete_revolutions_;
    }
    if (expired.zero_timeout) {
      ++scan_timeouts_;
    }
    accumulation_snapshot = mapper_.accumulationSnapshot();
  }
  const bool stale = last_valid == 0U ||
                     (stale_after_ms_ > 0 &&
                      now_ns > last_valid &&
                      now_ns - last_valid >
                          static_cast<uint64_t>(stale_after_ms_) * 1000000U);
  const uint64_t published = published_scans_.load();
  double publish_hz = 0.0;
  if (last_diagnostics_ns_ != 0U && now_ns > last_diagnostics_ns_) {
    publish_hz = static_cast<double>(published - last_diagnostics_published_) /
                 (static_cast<double>(now_ns - last_diagnostics_ns_) / 1e9);
  }
  last_diagnostics_ns_ = now_ns;
  last_diagnostics_published_ = published;

  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = now();
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "s3_ydlidar_bridge";
  status.hardware_id = "s3rd_radar_tcp_8765";
  status.level = stale ? diagnostic_msgs::msg::DiagnosticStatus::WARN
                       : diagnostic_msgs::msg::DiagnosticStatus::OK;
  status.message = stale ? "stale" : transport_stats.connection_state;
  addKey(status, "tcp_connection_state", transport_stats.connection_state);
  addNumber(status, "connection_epoch", sequence_snapshot.connection_epoch);
  addNumber(status, "accepted_connections", transport_stats.accepted_connections);
  addNumber(status, "disconnects", transport_stats.disconnects);
  addNumber(status, "recv_bytes", transport_stats.recv_bytes);
  addNumber(status, "received_packets", transport_stats.protocol.accepted_frames);
  addNumber(status, "received", transport_stats.protocol.accepted_frames);
  addNumber(status, "stale_frames", stale_frames_.load());
  addNumber(status, "raw_frames", transport_stats.protocol.raw_frames);
  addNumber(status, "opaque_frames", transport_stats.protocol.opaque_frames);
  addNumber(status, "opaque_dispatched", opaque_frames_.load());
  addKey(status, "chassis_frames", "not_applicable");
  addKey(status, "chassis_decode_status", "not_applicable");
  addKey(status, "chassis_updates_accepted", "not_applicable");
  addKey(status, "odometry_source", "not_applicable_use_tcp_8766_status");
  addNumber(status, "ready_queue_dropped", transport_stats.dropped_ready);
  addNumber(status, "ready_queue_overflow", transport_stats.overflow);
  addNumber(status, "sequence_domains", sequence_domain_count);
  addNumber(status, "magic_errors", transport_stats.protocol.magic_errors);
  addNumber(status, "crc_errors", transport_stats.protocol.crc_errors);
  addNumber(status, "outer_crc_error", transport_stats.protocol.crc_errors);
  addNumber(status, "length_errors", transport_stats.protocol.length_errors);
  addNumber(status, "version_errors", transport_stats.protocol.version_errors);
  addNumber(status, "message_type_errors", transport_stats.protocol.type_errors);
  addNumber(status, "flags_errors", transport_stats.protocol.flags_errors);
  addNumber(status, "identity_errors", transport_stats.protocol.identity_errors);
  addNumber(status, "ydlidar_checksum_errors", ydlidar_errors_.load());
  addNumber(status, "inner_frame_error", ydlidar_errors_.load());
  addNumber(status, "reconnect",
            transport_stats.accepted_connections == 0U
                ? 0U
                : transport_stats.accepted_connections - 1U);
  addNumber(status, "decoded_packets", decoded_packets_.load());
  addNumber(status, "accumulated_packets", accumulated_packets_.load());
  addNumber(status, "revolutions_published", revolutions_published_.load());
  addNumber(status, "valid_points", valid_points_.load());
  addNumber(status, "partial_revolutions_dropped",
            partial_revolutions_dropped_.load());
  addNumber(status, "scan_timeout", scan_timeouts_.load());
  addNumber(status, "incomplete_revolutions",
            incomplete_revolutions_.load());
  const RevolutionDiagnostics revolution =
      accumulation_snapshot.last_revolution.has_value()
          ? *accumulation_snapshot.last_revolution
          : RevolutionDiagnostics{accumulation_snapshot.frame_count,
                                  accumulation_snapshot.sequence_gaps,
                                  accumulation_snapshot.samples == 0U
                                      ? 0.0F
                                      : static_cast<float>(accumulation_snapshot.coverage) /
                                            static_cast<float>(accumulation_snapshot.samples),
                                  accumulation_snapshot.revolution_start_steady_ns == 0U ||
                                          now_ns < accumulation_snapshot.revolution_start_steady_ns
                                      ? 0.0F
                                      : static_cast<float>(
                                            static_cast<double>(now_ns -
                                                accumulation_snapshot.revolution_start_steady_ns) /
                                            1e9),
                                  false, false};
  addNumber(status, "frame_count", revolution.frame_count);
  addNumber(status, "sequence_gap", revolution.sequence_gaps);
  addKey(status, "coverage_ratio", std::to_string(revolution.coverage_ratio));
  addKey(status, "scan_time", std::to_string(revolution.scan_time));
  addKey(status, "zero_timeout", revolution.zero_timeout ? "true" : "false");
  addKey(status, "incomplete", revolution.incomplete ? "true" : "false");
  addKey(status, "coverage",
         std::to_string(accumulation_snapshot.coverage) + "/" +
             std::to_string(accumulation_snapshot.samples));
  addNumber(status, "zero_packets", zero_packets_.load());
  addNumber(status, "duplicate_sequences", duplicate_sequences_.load());
  addNumber(status, "out_of_order_sequences", out_of_order_sequences_.load());
  addNumber(status, "sequence_jumps", sequence_jumps_.load());
  addNumber(status, "sequence_gaps", sequence_gaps_.load());
  addNumber(status, "sequence_wraps", sequence_wraps_.load());
  addNumber(status, "published_scans", published);
  addKey(status, "first_sequence",
         sequence_snapshot.first_sequence.has_value()
             ? std::to_string(*sequence_snapshot.first_sequence)
             : std::string("never"));
  addKey(status, "last_sequence",
         sequence_snapshot.last_sequence.has_value()
             ? std::to_string(*sequence_snapshot.last_sequence)
             : std::string("never"));
  addKey(status, "last_flags",
         sequence_snapshot.last_flags.has_value()
             ? std::to_string(*sequence_snapshot.last_flags)
             : std::string("never"));
  addKey(status, "last_valid_packet_time_ns",
         last_valid == 0U ? std::string("never") : std::to_string(last_valid));
  addKey(status, "recent_valid_packet_time_ns",
         last_valid == 0U ? std::string("never") : std::to_string(last_valid));
  addKey(status, "last_valid_scan_age",
         last_valid_scan == 0U
             ? std::string("never")
             : std::to_string(now_ns >= last_valid_scan ? now_ns - last_valid_scan
                                                         : 0U));
  addKey(status, "stale", stale ? "true" : "false");
  addKey(status, "published_scan_frequency_hz", std::to_string(publish_hz));
  array.status.push_back(std::move(status));
  diagnostics_publisher_->publish(std::move(array));
}

}  // namespace s3_ydlidar_bridge
