#include "s3_ydlidar_bridge/scan_mapper.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace s3_ydlidar_bridge {

namespace {
constexpr float kPi = 3.14159265358979323846F;
constexpr float kRadiansPerDegree = kPi / 180.0F;
constexpr float kFullTurn = 2.0F * kPi;
}  // namespace

sensor_msgs::msg::LaserScan ScanMapper::emptyScan(
    const rclcpp::Time &stamp) const {
  sensor_msgs::msg::LaserScan message;
  message.header.stamp = stamp;
  message.header.frame_id = config_.frame_id;
  message.angle_min = config_.angle_min;
  message.angle_max = config_.angle_max;
  const size_t sample_count = std::max<size_t>(2U, config_.samples);
  message.angle_increment =
      (message.angle_max - message.angle_min) /
      static_cast<float>(sample_count - 1U);
  message.scan_time = config_.scan_frequency_hz > 0.0F
                          ? 1.0F / config_.scan_frequency_hz
                          : 0.0F;
  message.time_increment = message.scan_time /
                           static_cast<float>(sample_count - 1U);
  message.range_min = config_.range_min;
  message.range_max = config_.range_max;
  const float invalid = config_.invalid_range_is_inf
                            ? std::numeric_limits<float>::infinity()
                            : std::numeric_limits<float>::quiet_NaN();
  message.ranges.assign(sample_count, invalid);
  message.intensities.assign(sample_count, 0.0F);

  return message;
}

bool ScanMapper::rawAngleDegrees(const ::node_info &node,
                                 float &angle_degrees) const {
  if ((node.angle_q6_checkbit & LIDAR_RESP_MEASUREMENT_CHECKBIT) == 0U) {
    return false;
  }
  const uint16_t angle_q6 = static_cast<uint16_t>(
      node.angle_q6_checkbit >> LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT);
  if (angle_q6 > 360U * 64U) {
    return false;
  }
  angle_degrees = static_cast<float>(angle_q6) / 64.0F;
  return true;
}

bool ScanMapper::addNode(sensor_msgs::msg::LaserScan &message,
                         const ::node_info &node) const {
  float raw_angle_degrees = 0.0F;
  if (!rawAngleDegrees(node, raw_angle_degrees) ||
      !std::isfinite(message.angle_increment) ||
      message.angle_increment <= 0.0F) {
    return false;
  }

  // Preserve the existing ROS scan convention while using the raw [0, 360]
  // angle only to identify a physical revolution boundary.
  float angle = raw_angle_degrees * kRadiansPerDegree;
  while (angle > kPi) {
    angle -= kFullTurn;
  }
  while (angle < -kPi) {
    angle += kFullTurn;
  }
  const float range = static_cast<float>(node.distance_q2 / 4000.0F);
  if (node.distance_q2 == 0U || !std::isfinite(range) ||
      range < config_.range_min || range > config_.range_max) {
    return false;
  }
  const auto index = static_cast<int>(std::ceil(
      (angle - message.angle_min) / message.angle_increment));
  if (index < 0 || static_cast<size_t>(index) >= message.ranges.size()) {
    return false;
  }
  message.ranges[static_cast<size_t>(index)] = range;
  message.intensities[static_cast<size_t>(index)] =
      config_.publish_intensities ? static_cast<float>(node.sync_quality)
                                  : 0.0F;
  return true;
}

sensor_msgs::msg::LaserScan ScanMapper::map(
    const std::vector<::node_info> &nodes,
    const rclcpp::Time &stamp) const {
  auto message = emptyScan(stamp);
  for (const auto &node : nodes) {
    (void)addNode(message, node);
  }
  return message;
}

void ScanMapper::startRevolution(uint64_t received_steady_ns,
                                 const rclcpp::Time &stamp) {
  accumulated_scan_ = emptyScan(stamp);
  coverage_bins_.assign(accumulated_scan_.ranges.size(), false);
  accumulating_ = true;
  revolution_start_steady_ns_ = received_steady_ns;
  frame_count_ = 0U;
  sequence_gaps_ = 0U;
}

size_t ScanMapper::coverage(const std::vector<bool> &bins) {
  return static_cast<size_t>(std::count_if(
      bins.begin(), bins.end(), [](bool covered) { return covered; }));
}

void ScanMapper::addCoverage(const std::vector<::node_info> &nodes) {
  if (coverage_bins_.empty()) {
    return;
  }

  std::vector<float> angles;
  angles.reserve(nodes.size());
  for (const auto &node : nodes) {
    float raw_angle_degrees = 0.0F;
    if (rawAngleDegrees(node, raw_angle_degrees)) {
      angles.push_back(raw_angle_degrees == 360.0F ? 0.0F
                                                   : raw_angle_degrees);
    }
  }
  if (angles.empty()) {
    return;
  }

  const size_t samples = coverage_bins_.size();
  const auto binForAngle = [samples](float angle_degrees) {
    const float normalized = std::fmod(angle_degrees + 360.0F, 360.0F);
    const size_t bin = static_cast<size_t>(std::floor(
        normalized * static_cast<float>(samples) / 360.0F));
    return std::min(bin, samples - 1U);
  };
  for (size_t index = 1U; index < angles.size(); ++index) {
    size_t bin = binForAngle(angles[index - 1U]);
    const size_t end = binForAngle(angles[index]);
    for (;;) {
      coverage_bins_[bin] = true;
      if (bin == end) {
        break;
      }
      bin = (bin + 1U) % samples;
    }
  }
  coverage_bins_[binForAngle(angles.front())] = true;
  coverage_bins_[binForAngle(angles.back())] = true;
}

RevolutionDiagnostics ScanMapper::revolutionDiagnostics(
    uint64_t end_steady_ns, bool zero_timeout, bool incomplete) const {
  RevolutionDiagnostics diagnostics;
  diagnostics.frame_count = frame_count_;
  diagnostics.sequence_gaps = sequence_gaps_;
  diagnostics.coverage_ratio = coverage_bins_.empty()
                                   ? 0.0F
                                   : static_cast<float>(coverage(coverage_bins_)) /
                                         static_cast<float>(coverage_bins_.size());
  if (revolution_start_steady_ns_ != 0U &&
      end_steady_ns >= revolution_start_steady_ns_) {
    diagnostics.scan_time = static_cast<float>(
        static_cast<double>(end_steady_ns - revolution_start_steady_ns_) / 1e9);
  }
  diagnostics.zero_timeout = zero_timeout;
  diagnostics.incomplete = incomplete;
  return diagnostics;
}

ScanAccumulationResult ScanMapper::accumulate(
    const std::vector<::node_info> &nodes, bool zero_packet,
    uint64_t sequence_gap, uint64_t received_steady_ns,
    const rclcpp::Time &stamp) {
  ScanAccumulationResult result;
  if (zero_packet) {
    if (accumulating_) {
      const auto diagnostics = revolutionDiagnostics(received_steady_ns, false,
                                                     false);
      accumulated_scan_.scan_time = diagnostics.scan_time;
      accumulated_scan_.time_increment =
          accumulated_scan_.ranges.size() > 1U
              ? diagnostics.scan_time /
                    static_cast<float>(accumulated_scan_.ranges.size() - 1U)
              : 0.0F;
      result.completed_scans.push_back(std::move(accumulated_scan_));
      result.revolution_diagnostics = diagnostics;
      last_revolution_ = diagnostics;
    }
    // A zero-position packet is the first data packet of the next turn. The
    // first one after connect only establishes this start boundary.
    startRevolution(received_steady_ns, stamp);
  }

  if (!accumulating_) {
    return result;
  }
  ++frame_count_;
  if (sequence_gap != 0U) {
    sequence_gaps_ += sequence_gap;
  }
  addCoverage(nodes);
  for (const auto &node : nodes) {
    if (addNode(accumulated_scan_, node)) {
      ++result.valid_points;
    }
  }
  return result;
}

ScanAccumulationResult ScanMapper::expire(uint64_t now_steady_ns,
                                          uint64_t timeout_ns) {
  ScanAccumulationResult result;
  if (!accumulating_ || revolution_start_steady_ns_ == 0U ||
      now_steady_ns < revolution_start_steady_ns_ ||
      now_steady_ns - revolution_start_steady_ns_ <= timeout_ns) {
    return result;
  }
  const auto diagnostics = revolutionDiagnostics(now_steady_ns, true, true);
  result.partial_revolution_dropped = true;
  result.zero_timeout = true;
  result.revolution_diagnostics = diagnostics;
  last_revolution_ = diagnostics;
  accumulated_scan_ = sensor_msgs::msg::LaserScan{};
  coverage_bins_.clear();
  accumulating_ = false;
  revolution_start_steady_ns_ = 0U;
  frame_count_ = 0U;
  sequence_gaps_ = 0U;
  return result;
}

ScanAccumulationResult ScanMapper::resetAccumulation() {
  ScanAccumulationResult result;
  if (accumulating_) {
    const auto diagnostics = revolutionDiagnostics(revolution_start_steady_ns_,
                                                   false, true);
    result.partial_revolution_dropped = true;
    result.revolution_diagnostics = diagnostics;
    last_revolution_ = diagnostics;
  }
  accumulated_scan_ = sensor_msgs::msg::LaserScan{};
  coverage_bins_.clear();
  accumulating_ = false;
  revolution_start_steady_ns_ = 0U;
  frame_count_ = 0U;
  sequence_gaps_ = 0U;
  return result;
}

ScanAccumulationSnapshot ScanMapper::accumulationSnapshot() const {
  ScanAccumulationSnapshot snapshot;
  snapshot.samples = std::max<size_t>(2U, config_.samples);
  snapshot.has_partial_revolution = accumulating_;
  snapshot.coverage = accumulating_ ? coverage(coverage_bins_) : 0U;
  snapshot.frame_count = frame_count_;
  snapshot.sequence_gaps = sequence_gaps_;
  snapshot.revolution_start_steady_ns = revolution_start_steady_ns_;
  snapshot.last_revolution = last_revolution_;
  return snapshot;
}

}  // namespace s3_ydlidar_bridge
