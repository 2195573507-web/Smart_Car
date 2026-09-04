"""Mock Nav2 integration boundary for the pure coordinator.

The explorer can request only arm, NavigateToPose, STOP, and save effects. It
has no non-zero Twist effect, so a preflight cannot create a drive command.
"""

from smartcar_auto_exploration.core import EffectKind, ExplorationCore, Health
from smartcar_auto_exploration.frontier import Frontier


GOAL = Frontier((4, 5), 4, 5, 5.5, 4.5, 6, 3.0)
PREFLIGHT = Health(scan=True, odom=True, tf=True, gateway_ready=True)
HEALTHY = Health(scan=True, odom=True, tf=True, gateway_ready=True, motion_healthy=True)


class MockNav2:
    def __init__(self):
        self.goals = []
        self.zero_stops = 0

    def apply(self, effects):
        for effect in effects:
            if effect.kind is EffectKind.NAVIGATE:
                self.goals.append(effect.goal)
            if effect.kind in (EffectKind.ZERO_STOP, EffectKind.HARD_STOP):
                self.zero_stops += 1


def test_mock_nav2_receives_no_goal_before_preflight_and_zero_stop_on_failure():
    core = ExplorationCore(max_consecutive_failures=2)
    nav2 = MockNav2()
    assert core.start()
    nav2.apply(core.update_health(Health()))
    assert nav2.goals == []
    nav2.apply(core.update_health(PREFLIGHT))
    assert nav2.goals == []
    nav2.apply(core.arm_result(True))
    nav2.apply(core.update_health(HEALTHY))
    nav2.apply(core.consider_frontiers([GOAL]))
    assert nav2.goals == [GOAL]
    nav2.apply(core.navigation_result(False))
    assert nav2.zero_stops == 1
