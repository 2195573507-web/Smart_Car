#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/time.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "core/common/ydlidar_protocol.h"

namespace s3_ydlidar_bridge {

struct ScanMapperConfig {
  std::string frame_id{"laser_frame"};
  float angle_min{-3.14159265358979323846F};
  float angle_max{3.14159265358979323846F};
  float range_min{0.10F};
  float range_max{8.0F};
  size_t samples{360};
  float scan_frequency_hz{10.0F};
  bool invalid_range_is_inf{true};
  bool publish_intensities{false};
};

struct RevolutionDiagnostics {
  uint64_t frame_count{0};
  uint64_t sequence_gaps{0};
  float coverage_ratio{0.0F};
  float scan_time{0.0F};
  bool zero_timeout{false};
  bool incomplete{false};
};

struct ScanAccumulationResult {
  std::vector<sensor_msgs::msg::LaserScan> completed_scans;
  uint64_t valid_points{0};
  bool partial_revolution_dropped{false};
  bool zero_timeout{false};
  std::optional<RevolutionDiagnostics> revolution_diagnostics;
};

struct ScanAccumulationSnapshot {
  size_t coverage{0};
  size_t samples{0};
  bool has_partial_revolution{false};
  uint64_t frame_count{0};
  uint64_t sequence_gaps{0};
  uint64_t revolution_start_steady_ns{0};
  float current_scan_time{0.0F};
  std::optional<RevolutionDiagnostics> last_revolution;
};

class ScanMapper {
 public:
  explicit ScanMapper(ScanMapperConfig config) : config_(std::move(config)) {}

  sensor_msgs::msg::LaserScan map(
      const std::vector<::node_info> &nodes,
      const rclcpp::Time &stamp) const;
  ScanAccumulationResult accumulate(const std::vector<::node_info> &nodes,
                                    bool zero_packet, uint64_t sequence_gap,
                                    uint64_t received_steady_ns,
                                    const rclcpp::Time &stamp);
  ScanAccumulationResult expire(uint64_t now_steady_ns,
                                uint64_t timeout_ns);
  ScanAccumulationResult resetAccumulation();
  ScanAccumulationSnapshot accumulationSnapshot() const;

 private:
  sensor_msgs::msg::LaserScan emptyScan(const rclcpp::Time &stamp) const;
  bool rawAngleDegrees(const ::node_info &node, float &angle_degrees) const;
  bool addNode(sensor_msgs::msg::LaserScan &message,
               const ::node_info &node) const;
  void startRevolution(uint64_t received_steady_ns,
                       const rclcpp::Time &stamp);
  void addCoverage(const std::vector<::node_info> &nodes);
  RevolutionDiagnostics revolutionDiagnostics(uint64_t end_steady_ns,
                                              bool zero_timeout,
                                              bool incomplete) const;
  static size_t coverage(const std::vector<bool> &bins);

  ScanMapperConfig config_;
  sensor_msgs::msg::LaserScan accumulated_scan_;
  std::vector<bool> coverage_bins_;
  bool accumulating_{false};
  uint64_t revolution_start_steady_ns_{0};
  uint64_t frame_count_{0};
  uint64_t sequence_gaps_{0};
  std::optional<RevolutionDiagnostics> last_revolution_;
};

}  // namespace s3_ydlidar_bridge
