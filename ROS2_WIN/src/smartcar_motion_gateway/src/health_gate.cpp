#include "smartcar_motion_gateway/health_gate.hpp"

namespace smartcar_motion_gateway {

bool HealthGate::fresh(uint64_t now_ns, uint64_t sample_ns) const noexcept {
  return sample_ns != 0U && now_ns >= sample_ns && now_ns - sample_ns <= timeout_ns_;
}

HealthResult HealthGate::evaluate(const HealthSnapshot &snapshot) const noexcept {
  return HealthResult{fresh(snapshot.now_ns, snapshot.scan_ns),
                      fresh(snapshot.now_ns, snapshot.odom_ns),
                      fresh(snapshot.now_ns, snapshot.tf_ns), snapshot.lease};
}

}  // namespace smartcar_motion_gateway
