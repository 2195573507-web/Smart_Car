#pragma once

#include <cstdint>

namespace smartcar_motion_gateway {

struct HealthSnapshot {
  uint64_t now_ns{0U};
  uint64_t scan_ns{0U};
  uint64_t odom_ns{0U};
  uint64_t tf_ns{0U};
  bool lease{false};
};

struct HealthResult {
  bool scan{false};
  bool odom{false};
  bool tf{false};
  bool lease{false};

  bool all() const noexcept { return scan && odom && tf && lease; }
};

class HealthGate final {
 public:
  explicit HealthGate(uint64_t timeout_ns = 500000000U)
      : timeout_ns_(timeout_ns) {}

  HealthResult evaluate(const HealthSnapshot &snapshot) const noexcept;

 private:
  bool fresh(uint64_t now_ns, uint64_t sample_ns) const noexcept;
  uint64_t timeout_ns_;
};

}  // namespace smartcar_motion_gateway
