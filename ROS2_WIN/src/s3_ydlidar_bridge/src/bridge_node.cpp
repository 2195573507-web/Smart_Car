#include "s3_ydlidar_bridge/bridge_node.hpp"

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "smartcar_state_bridge/state_adapter.hpp"

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

constexpr uint8_t kS3ChassisStateMessageType = 2U;

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

smartcar_state_bridge::TelemetryDecoderConfig readTelemetryConfig(
    rclcpp::Node &node) {
  smartcar_state_bridge::TelemetryDecoderConfig config;
  config.allow_live =
      node.declare_parameter("allow_live_telemetry", config.allow_live);
  config.allow_offline_fixtures = node.declare_parameter(
      "allow_offline_fixtures", config.allow_offline_fixtures);
  config.require_source_freshness = readOrDeclareParameter(
      node, "require_source_freshness", config.require_source_freshness);
  const int64_t max_payload = node.declare_parameter(
      "telemetry_max_payload_bytes",
      static_cast<int64_t>(config.max_payload_bytes));
  if (max_payload > 0 &&
      max_payload <= static_cast<int64_t>(
                          smartcar_state_bridge::kScbpMaximumFrameBytes)) {
    config.max_payload_bytes = static_cast<std::size_t>(max_payload);
  } else {
    RCLCPP_WARN(
        node.get_logger(),
        "telemetry_max_payload_bytes must be in 1..%zu; using %zu",
        smartcar_state_bridge::kScbpMaximumFrameBytes, config.max_payload_bytes);
  }
  const int64_t source_id =
      node.declare_parameter("telemetry_expected_source_id", static_cast<int64_t>(-1));
  if (source_id >= 0 && source_id <= std::numeric_limits<uint16_t>::max()) {
    config.expected_source_id = static_cast<uint16_t>(source_id);
  }
  const int64_t destination_id = node.declare_parameter(
      "telemetry_expected_destination_id", static_cast<int64_t>(-1));
  if (destination_id >= 0 &&
      destination_id <= std::numeric_limits<uint16_t>::max()) {
    config.expected_destination_id = static_cast<uint16_t>(destination_id);
  }
  const int64_t wheel_type = node.declare_parameter(
      "telemetry_wheel_message_type",
      static_cast<int64_t>(config.wheel_message_type));
  if (wheel_type ==
      static_cast<int64_t>(smartcar_state_bridge::kWheelStatusMessageType)) {
    config.wheel_message_type = smartcar_state_bridge::kWheelStatusMessageType;
  } else {
    RCLCPP_WARN(node.get_logger(),
                "telemetry_wheel_message_type must be reviewed wheel type 0x0210; using %u",
                static_cast<unsigned>(
                    smartcar_state_bridge::kWheelStatusMessageType));
    config.wheel_message_type = smartcar_state_bridge::kWheelStatusMessageType;
  }
  return config;
}

smartcar_state_bridge::WheelOdomConfig readWheelOdomConfig(
    rclcpp::Node &node) {
  smartcar_state_bridge::WheelOdomConfig config;
  config.track_width_m = node.declare_parameter("track_width_m",
                                                config.track_width_m);
  config.wheel_diameter_m = node.declare_parameter(
      "wheel_diameter_m", config.wheel_diameter_m);
  config.sample_tick_period_s = node.declare_parameter(
      "sample_tick_period_s", config.sample_tick_period_s);
  config.min_dt_s = node.declare_parameter("min_dt_s", config.min_dt_s);
  config.max_dt_s = node.declare_parameter("max_dt_s", config.max_dt_s);
  const int64_t stale_ms = node.declare_parameter(
      "stale_timeout_ms", static_cast<int64_t>(config.stale_timeout_ms));
  if (stale_ms > 0 &&
      stale_ms <= static_cast<int64_t>(std::numeric_limits<uint32_t>::max())) {
    config.stale_timeout_ms = static_cast<uint32_t>(stale_ms);
  }
  const int64_t fifo_depth = node.declare_parameter(
      "wheel_fifo_depth", static_cast<int64_t>(config.fifo_depth));
  if (fifo_depth >= 0) {
    config.fifo_depth = static_cast<std::size_t>(fifo_depth);
  }
  config.require_source_freshness = readOrDeclareParameter(
      node, "require_source_freshness", config.require_source_freshness);
  config.allow_sequence_wrap = node.declare_parameter(
      "allow_sequence_wrap", config.allow_sequence_wrap);
  config.allow_tick_wrap =
      node.declare_parameter("allow_tick_wrap", config.allow_tick_wrap);
  const std::vector<double> signs = node.declare_parameter(
      "wheel_speed_sign", std::vector<double>{1.0, 1.0, 1.0, 1.0});
  if (signs.size() == smartcar_state_bridge::kWheelCount) {
    std::copy(signs.begin(), signs.end(), config.wheel_speed_sign.begin());
  } else {
    RCLCPP_WARN(node.get_logger(),
                "wheel_speed_sign must contain RR, RF, LR, LF; using defaults");
  }
  return config;
}

smartcar_state_bridge::StateAdapterConfig readStateAdapterConfig(
    rclcpp::Node &node) {
  smartcar_state_bridge::StateAdapterConfig config;
  config.decoder = readTelemetryConfig(node);
  config.odom = readWheelOdomConfig(node);
  config.require_outer_sequence = node.declare_parameter(
      "require_outer_sequence", config.require_outer_sequence);
  if (node.has_parameter("enable_live_odom")) {
    node.get_parameter("enable_live_odom", config.enable_live_odom);
  } else {
    config.enable_live_odom = node.declare_parameter("enable_live_odom",
                                                     config.enable_live_odom);
  }
  return config;
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
  odom_topic_ = declare_parameter("odom_topic", odom_topic_);
  odom_frame_id_ = declare_parameter("odom_frame_id", odom_frame_id_);
  odom_child_frame_id_ =
      declare_parameter("odom_child_frame_id", odom_child_frame_id_);
  enable_live_odom_ =
      declare_parameter("enable_live_odom", enable_live_odom_);
  publish_odom_ = declare_parameter("publish_odom", publish_odom_);
  publish_tf_ = declare_parameter("publish_tf", publish_tf_);
  const auto state_config = readStateAdapterConfig(*this);
  state_adapter_ = smartcar_state_bridge::StateAdapter(state_config);
  smartcar_state_bridge::ChassisStateAdapterConfig chassis_config;
  chassis_config.allow_live_telemetry = state_config.decoder.allow_live;
  chassis_config.enable_live_odom = state_config.enable_live_odom;
  chassis_config.allow_offline_fixtures =
      state_config.decoder.allow_offline_fixtures;
  chassis_config.require_outer_sequence = state_config.require_outer_sequence;
  chassis_config.odom.min_dt_s = state_config.odom.min_dt_s;
  chassis_config.odom.max_dt_s = state_config.odom.max_dt_s;
  chassis_config.odom.stale_timeout_ms = state_config.odom.stale_timeout_ms;
  srp_v4_telemetry_adapter_ =
      smartcar_state_bridge::SrpV4TelemetryAdapter(chassis_config);

  publisher_ = create_publisher<sensor_msgs::msg::LaserScan>(
      topic, rclcpp::SensorDataQoS());
  diagnostics_publisher_ =
      create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
          diagnostics_topic, rclcpp::QoS(10));
  publish_opaque_telemetry_ = declare_parameter(
      "publish_opaque_telemetry", publish_opaque_telemetry_);
  telemetry_topic_ = declare_parameter("telemetry_topic", telemetry_topic_);
  if (publish_opaque_telemetry_) {
    telemetry_publisher_ =
        create_publisher<std_msgs::msg::UInt8MultiArray>(telemetry_topic_,
                                                         rclcpp::QoS(10));
  }
  if (publish_odom_) {
    odom_publisher_ =
        create_publisher<nav_msgs::msg::Odometry>(odom_topic_, rclcpp::QoS(10));
  }
  if (publish_tf_) {
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  }

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
                              {kS3ChassisStateMessageType});
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
  if (event.type == ConnectionEventType::kOpened) {
    std::lock_guard<std::mutex> lock(telemetry_mutex_);
    state_adapter_.beginSession(event.connection_epoch);
    srp_v4_telemetry_adapter_.beginSession(event.connection_epoch);
  } else {
    std::lock_guard<std::mutex> lock(telemetry_mutex_);
    state_adapter_.endSession();
    srp_v4_telemetry_adapter_.endSession();
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
      const uint8_t message_type =
          frame.message_type != 0U ? frame.message_type
                                   : frame.metadata.message_type;
      if (frame.isOpaque() && message_type == kS3ChassisStateMessageType) {
        std::lock_guard<std::mutex> lock(telemetry_mutex_);
        srp_v4_telemetry_adapter_.invalidateChassis(
            smartcar_state_bridge::ChassisOdomStatus::kStale,
            "stale S3RD type-2 frame was dropped");
      }
      ++stale_frames_;
      return;
    }
  }

  uint64_t sequence_gap = 0U;
  smartcar_state_bridge::TelemetryOuterSequenceStatus telemetry_sequence_status =
      smartcar_state_bridge::TelemetryOuterSequenceStatus::kUnknown;
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
    switch (sequence_status) {
      case SequenceStatus::kFirst:
        telemetry_sequence_status =
            smartcar_state_bridge::TelemetryOuterSequenceStatus::kFirst;
        break;
      case SequenceStatus::kInOrder:
        telemetry_sequence_status =
            smartcar_state_bridge::TelemetryOuterSequenceStatus::kInOrder;
        break;
      case SequenceStatus::kDuplicate:
        telemetry_sequence_status =
            smartcar_state_bridge::TelemetryOuterSequenceStatus::kDuplicate;
        break;
      case SequenceStatus::kOutOfOrder:
        telemetry_sequence_status =
            smartcar_state_bridge::TelemetryOuterSequenceStatus::kOutOfOrder;
        break;
      case SequenceStatus::kJump:
        telemetry_sequence_status =
            smartcar_state_bridge::TelemetryOuterSequenceStatus::kJump;
        break;
      case SequenceStatus::kWrap:
        telemetry_sequence_status =
            smartcar_state_bridge::TelemetryOuterSequenceStatus::kWrap;
        break;
    }
    const bool sequence_fault =
        sequence_status == SequenceStatus::kDuplicate ||
        sequence_status == SequenceStatus::kOutOfOrder ||
        sequence_status == SequenceStatus::kJump || sequence_gap != 0U;
    if (sequence_fault && !frame.isOpaque()) {
      std::lock_guard<std::mutex> lock(telemetry_mutex_);
      srp_v4_telemetry_adapter_.invalidateChassis(
          smartcar_state_bridge::ChassisOdomStatus::kInvalidSample,
          "global S3RD outer sequence fault");
    }
    if (sequence_status == SequenceStatus::kDuplicate ||
        sequence_status == SequenceStatus::kOutOfOrder) {
      if (sequence_status == SequenceStatus::kDuplicate) {
        ++duplicate_sequences_;
      } else {
        ++out_of_order_sequences_;
      }
      if (frame.isOpaque()) {
        handleOpaqueFrame(frame, telemetry_sequence_status, sequence_gap);
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

  // The outer parser admits opaque types only when explicitly configured.
  // They are dispatched through the structured telemetry boundary and are
  // never fed to the YDLIDAR decoder or accidentally published as /scan.
  if (frame.isOpaque()) {
    handleOpaqueFrame(frame, telemetry_sequence_status, sequence_gap);
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

void BridgeNode::handleOpaqueFrame(
    const ReceivedFrame &frame,
    smartcar_state_bridge::TelemetryOuterSequenceStatus sequence_status,
    uint64_t sequence_gap) {
  ++opaque_frames_;

  // Preserve the gateway-owned bytes as an optional observation stream.  The
  // message intentionally carries only bytes: until SCBP is approved there is
  // no safe host-side schema to serialize into a ROS message.
  if (publish_opaque_telemetry_ && telemetry_publisher_) {
    std_msgs::msg::UInt8MultiArray raw;
    raw.data = frame.payload;
    telemetry_publisher_->publish(std::move(raw));
    ++telemetry_published_raw_;
  }

  const uint8_t outer_message_type =
      frame.message_type != 0U ? frame.message_type : frame.metadata.message_type;
  if (outer_message_type == kS3ChassisStateMessageType) {
    smartcar_state_bridge::ChassisTelemetryFrame telemetry_frame;
    telemetry_frame.origin =
        smartcar_state_bridge::TelemetryOrigin::kLiveGateway;
    telemetry_frame.payload = frame.payload;
    telemetry_frame.outer_sequence_status = sequence_status;
    telemetry_frame.outer_sequence_gap = sequence_gap;
    telemetry_frame.connection_epoch = frame.connection_epoch;
    telemetry_frame.host_received_steady_ns = frame.received_steady_ns;

    smartcar_state_bridge::SrpV4TelemetrySubmitResult telemetry_result;
    {
      std::lock_guard<std::mutex> lock(telemetry_mutex_);
      telemetry_result = srp_v4_telemetry_adapter_.submit(telemetry_frame);
    }
    switch (telemetry_result.status) {
      case smartcar_state_bridge::SrpV4TelemetrySubmitStatus::kIgnored:
        break;
      case smartcar_state_bridge::SrpV4TelemetrySubmitStatus::
          kFrameDecodeRejected:
      case smartcar_state_bridge::SrpV4TelemetrySubmitStatus::
          kOuterSequenceRejected:
        ++telemetry_rejected_;
        break;
      case smartcar_state_bridge::SrpV4TelemetrySubmitStatus::kChassis:
        switch (telemetry_result.chassis_result.status) {
          case smartcar_state_bridge::ChassisSubmitStatus::kAccepted:
            ++telemetry_accepted_;
            if (odom_publisher_) {
              publishChassisOdometry(
                  telemetry_result.chassis_result.odom_update);
              ++odom_published_;
            }
            break;
          case smartcar_state_bridge::ChassisSubmitStatus::kAnchored:
            ++telemetry_accepted_;
            break;
          case smartcar_state_bridge::ChassisSubmitStatus::kDisabled:
            ++telemetry_disabled_;
            break;
          default:
            ++telemetry_rejected_;
            break;
        }
        break;
    }
    if (!telemetry_result.error.empty()) {
      RCLCPP_DEBUG(get_logger(), "SRP v4 type-2 frame gated: %s",
                   telemetry_result.error.c_str());
    }
    // Type 2 is a multiplexed SRP stream. Its valid non-chassis messages are
    // observed but never sent to chassis odometry, the legacy wheel fixture,
    // or the YDLIDAR decoder.
    return;
  }

  smartcar_state_bridge::TelemetryEnvelope envelope;
  envelope.origin = smartcar_state_bridge::TelemetryOrigin::kLiveGateway;
  // The outer type is only a transport discriminator.  Until the SCBP wire
  // contract is approved, do not promote it to an inner 16-bit message type.
  envelope.outer_message_type = outer_message_type;
  envelope.message_type = 0U;
  envelope.flags = frame.flags != 0U ? frame.flags : frame.metadata.flags;
  envelope.outer_sequence = frame.sequence.value_or(frame.metadata.sequence);
  envelope.outer_sequence_status = sequence_status;
  envelope.outer_sequence_gap = sequence_gap;
  envelope.connection_epoch = frame.connection_epoch;
  envelope.host_received_steady_ns = frame.received_steady_ns;
  envelope.payload = frame.payload;

  smartcar_state_bridge::StateSubmitResult result;
  {
    std::lock_guard<std::mutex> lock(telemetry_mutex_);
    result = state_adapter_.submitTelemetry(envelope);
  }
  switch (result.decode_status) {
    case smartcar_state_bridge::TelemetryDecodeStatus::kAccepted:
      ++telemetry_accepted_;
      if (result.odom_update.status ==
              smartcar_state_bridge::OdomUpdateStatus::kAccepted &&
          odom_publisher_) {
        publishOdometry(result.odom_update);
        ++odom_published_;
      }
      break;
    case smartcar_state_bridge::TelemetryDecodeStatus::kDisabled:
      ++telemetry_disabled_;
      break;
    case smartcar_state_bridge::TelemetryDecodeStatus::kNotConfigured:
      ++telemetry_not_configured_;
      break;
    default:
      ++telemetry_rejected_;
      break;
  }
  if (!result.error.empty()) {
    RCLCPP_DEBUG(get_logger(), "opaque telemetry frame gated: %s",
                 result.error.c_str());
  }
}

void BridgeNode::publishOdometry(
    const smartcar_state_bridge::OdomUpdate &update) {
  if (!odom_publisher_ ||
      update.status != smartcar_state_bridge::OdomUpdateStatus::kAccepted) {
    return;
  }
  smartcar_state_bridge::PlanarOdomData data;
  data.x_m = update.state.x_m;
  data.y_m = update.state.y_m;
  data.yaw_rad = update.state.heading_rad;
  data.linear_x_mps = update.state.linear_mps;
  data.angular_z_rps = update.state.angular_rps;
  publishPlanarOdometry(data);
}

void BridgeNode::publishChassisOdometry(
    const smartcar_state_bridge::ChassisOdomUpdate &update) {
  if (!odom_publisher_ ||
      update.status != smartcar_state_bridge::ChassisOdomStatus::kAccepted) {
    return;
  }
  smartcar_state_bridge::PlanarOdomData data;
  data.x_m = update.state.x_m;
  data.y_m = update.state.y_m;
  data.yaw_rad = update.state.yaw_rad;
  data.linear_x_mps = update.state.linear_x_mps;
  data.linear_y_mps = update.state.linear_y_mps;
  data.angular_z_rps = update.state.angular_z_rps;
  publishPlanarOdometry(data);
}

void BridgeNode::publishPlanarOdometry(
    const smartcar_state_bridge::PlanarOdomData &data) {
  if (!odom_publisher_) {
    return;
  }
  smartcar_state_bridge::OdomMessageConfig config;
  config.frame_id = odom_frame_id_;
  config.child_frame_id = odom_child_frame_id_;
  const auto message =
      smartcar_state_bridge::makeOdometryMessage(data, now(), config);
  odom_publisher_->publish(message);
  if (tf_broadcaster_) {
    tf_broadcaster_->sendTransform(
        smartcar_state_bridge::makeOdomTransform(message));
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

  smartcar_state_bridge::StateAdapterCounters state_counters;
  smartcar_state_bridge::OdomState odom_state;
  smartcar_state_bridge::TelemetryDecodeStatus last_state_decode =
      smartcar_state_bridge::TelemetryDecodeStatus::kNotConfigured;
  bool live_odom_enabled = false;
  std::size_t wheel_fifo_depth = 0U;
  std::size_t wheel_fifo_size = 0U;
  std::size_t wheel_fifo_overflow = 0U;
  smartcar_state_bridge::OdomUpdateStatus odom_status =
      smartcar_state_bridge::OdomUpdateStatus::kNoSample;
  smartcar_state_bridge::ChassisStateAdapterCounters chassis_counters;
  smartcar_state_bridge::SrpV4TelemetryCounters srp_counters;
  smartcar_state_bridge::ChassisOdomState chassis_state;
  smartcar_state_bridge::ChassisOdomStatus chassis_odom_status =
      smartcar_state_bridge::ChassisOdomStatus::kNoSample;
  smartcar_state_bridge::SrpV4DecodeStatus chassis_decode_status =
      smartcar_state_bridge::SrpV4DecodeStatus::kNotAttempted;
  smartcar_state_bridge::SrpV4FrameDecodeStatus srp_frame_decode_status =
      smartcar_state_bridge::SrpV4FrameDecodeStatus::kNotAttempted;
  {
    std::lock_guard<std::mutex> lock(telemetry_mutex_);
    state_adapter_.odom().checkStale(now_ns);
    srp_v4_telemetry_adapter_.checkStale(now_ns);
    state_counters = state_adapter_.counters();
    odom_state = state_adapter_.odom().state();
    last_state_decode = state_adapter_.lastDecodeStatus();
    live_odom_enabled = state_adapter_.config().enable_live_odom;
    wheel_fifo_depth = state_adapter_.odom().fifo().depth();
    wheel_fifo_size = state_adapter_.odom().fifo().size();
    wheel_fifo_overflow = state_adapter_.odom().fifo().overflowCount();
    odom_status = state_adapter_.odom().lastStatus();
    srp_counters = srp_v4_telemetry_adapter_.counters();
    chassis_counters = srp_v4_telemetry_adapter_.chassis().counters();
    chassis_state = srp_v4_telemetry_adapter_.chassis().odom().state();
    chassis_odom_status =
        srp_v4_telemetry_adapter_.chassis().odom().lastStatus();
    chassis_decode_status =
        srp_v4_telemetry_adapter_.chassis().lastDecodeStatus();
    srp_frame_decode_status =
        srp_v4_telemetry_adapter_.lastFrameDecodeStatus();
  }

  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = now();
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "s3_ydlidar_bridge";
  status.hardware_id = "s3_gateway_experimental";
  status.level = odom_state.invalid_latched
                     ? diagnostic_msgs::msg::DiagnosticStatus::ERROR
                     : (stale ? diagnostic_msgs::msg::DiagnosticStatus::WARN
                              : diagnostic_msgs::msg::DiagnosticStatus::OK);
  status.message = odom_state.invalid_latched
                       ? "odom_invalid"
                       : (stale ? "stale" : transport_stats.connection_state);
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
  addNumber(status, "telemetry_accepted", telemetry_accepted_.load());
  addNumber(status, "telemetry_disabled", telemetry_disabled_.load());
  addNumber(status, "telemetry_not_configured",
            telemetry_not_configured_.load());
  addNumber(status, "telemetry_rejected", telemetry_rejected_.load());
  addNumber(status, "telemetry_published_raw", telemetry_published_raw_.load());
  addNumber(status, "odom_published", odom_published_.load());
  addNumber(status, "srp_frames", srp_counters.received);
  addNumber(status, "srp_decoder_rejected",
            srp_counters.frame_decode_rejected);
  addNumber(status, "srp_outer_sequence_rejected",
            srp_counters.outer_sequence_rejected);
  addNumber(status, "srp_imu_frames", srp_counters.imu_frames);
  addNumber(status, "srp_wheel_frames", srp_counters.wheel_frames);
  addNumber(status, "srp_other_frames", srp_counters.other_frames);
  addNumber(status, "chassis_frames", srp_counters.chassis_frames);
  addNumber(status, "chassis_anchored", chassis_counters.anchored);
  addNumber(status, "chassis_rejected",
            chassis_counters.decode_rejected +
                chassis_counters.sequence_rejected +
                chassis_counters.odom_rejected);
  addNumber(status, "chassis_decoder_rejected",
            chassis_counters.decode_rejected);
  addNumber(status, "chassis_sequence_rejected",
            chassis_counters.sequence_rejected);
  addNumber(status, "chassis_odom_rejected", chassis_counters.odom_rejected);
  addNumber(status, "chassis_updates_accepted", chassis_counters.accepted);
  addKey(status, "chassis_decode_status",
         smartcar_state_bridge::toString(chassis_decode_status));
  addKey(status, "srp_decode_status",
         smartcar_state_bridge::toString(srp_frame_decode_status));
  addKey(status, "chassis_odom_status",
         smartcar_state_bridge::toString(chassis_odom_status));
  addKey(status, "chassis_baseline_ready",
         chassis_state.baseline_ready ? "true" : "false");
  addKey(status, "chassis_last_timestamp_ms",
         chassis_state.last_timestamp_ms.has_value()
             ? std::to_string(*chassis_state.last_timestamp_ms)
              : std::string("unset"));
  addKey(status, "chassis_last_inner_sequence",
         chassis_state.last_inner_sequence.has_value()
             ? std::to_string(*chassis_state.last_inner_sequence)
             : std::string("unset"));
  addKey(status, "chassis_last_host_received_steady_ns",
         chassis_state.last_host_received_steady_ns == 0U
             ? std::string("never")
             : std::to_string(chassis_state.last_host_received_steady_ns));
  addKey(status, "live_odom_enabled", live_odom_enabled ? "true" : "false");
  addKey(status, "state_last_decode",
         smartcar_state_bridge::telemetryStatusString(last_state_decode));
  addKey(status, "odom_status",
         smartcar_state_bridge::toString(odom_status));
  addKey(status, "odom_invalid", odom_state.invalid_latched ? "true" : "false");
  addKey(status, "wheel_last_host_received_steady_ns",
         odom_state.last_host_received_steady_ns == 0U
             ? std::string("never")
             : std::to_string(odom_state.last_host_received_steady_ns));
  addKey(status, "wheel_last_sample_age_ns",
         odom_state.last_host_received_steady_ns == 0U ||
                 now_ns < odom_state.last_host_received_steady_ns
             ? std::string("unknown")
             : std::to_string(now_ns - odom_state.last_host_received_steady_ns));
  addKey(status, "wheel_source_time_s",
         odom_state.last_source_time_s.has_value()
             ? std::to_string(*odom_state.last_source_time_s)
             : std::string("unset"));
  addKey(status, "wheel_source_age_ms",
         odom_state.last_source_age_ms.has_value()
             ? std::to_string(*odom_state.last_source_age_ms)
             : std::string("unset"));
  addKey(status, "wheel_source_epoch",
         odom_state.source_epoch.has_value()
             ? std::to_string(*odom_state.source_epoch)
             : std::string("unset"));
  addNumber(status, "wheel_samples_submitted", state_counters.submitted_samples);
  addNumber(status, "wheel_samples_accepted", state_counters.accepted_samples);
  addNumber(status, "wheel_samples_rejected", state_counters.rejected_samples);
  addNumber(status, "wheel_outer_sequence_gaps",
            state_counters.outer_sequence_gaps);
  addNumber(status, "wheel_outer_sequence_duplicates",
            state_counters.outer_sequence_duplicates);
  addNumber(status, "wheel_outer_sequence_out_of_order",
            state_counters.outer_sequence_out_of_order);
  addNumber(status, "wheel_fifo_depth", wheel_fifo_depth);
  addNumber(status, "wheel_fifo_size", wheel_fifo_size);
  addNumber(status, "wheel_fifo_overflow", wheel_fifo_overflow);
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
