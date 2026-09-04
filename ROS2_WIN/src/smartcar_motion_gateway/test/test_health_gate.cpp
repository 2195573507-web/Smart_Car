#include "smartcar_motion_gateway/health_gate.hpp"

#include <gtest/gtest.h>

using smartcar_motion_gateway::HealthGate;
using smartcar_motion_gateway::HealthSnapshot;

TEST(HealthGate, RequiresAllFourInputs) {
  HealthGate gate(500U);
  const HealthSnapshot healthy{1000U, 900U, 900U, 900U, true};
  EXPECT_TRUE(gate.evaluate(healthy).all());
  auto no_lease = healthy;
  no_lease.lease = false;
  EXPECT_FALSE(gate.evaluate(no_lease).all());
}

TEST(HealthGate, RejectsStaleOrFutureSamples) {
  HealthGate gate(500U);
  const HealthSnapshot stale{1000U, 499U, 900U, 900U, true};
  EXPECT_FALSE(gate.evaluate(stale).all());
  const HealthSnapshot future{1000U, 1100U, 900U, 900U, true};
  EXPECT_FALSE(gate.evaluate(future).all());
}
