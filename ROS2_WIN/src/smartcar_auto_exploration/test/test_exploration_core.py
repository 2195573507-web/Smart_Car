from smartcar_auto_exploration.core import EffectKind, ExplorationCore, ExplorationState, Health
from smartcar_auto_exploration.frontier import Frontier


HEALTHY = Health(scan=True, odom=True, tf=True, gateway_ready=True, motion_healthy=True)
PREFLIGHT_READY = Health(scan=True, odom=True, tf=True, gateway_ready=True)
GOAL = Frontier((2, 3), 2, 3, 3.5, 2.5, 5, 2.0)


def make_exploring():
    core = ExplorationCore(max_consecutive_failures=2)
    assert core.start()
    assert [effect.kind for effect in core.update_health(PREFLIGHT_READY)] == [EffectKind.ARM]
    assert core.arm_result(True) == []
    assert core.update_health(HEALTHY) == []
    assert core.state is ExplorationState.EXPLORING
    return core


def test_preflight_never_emits_navigation_before_health_and_arm():
    core = ExplorationCore()
    assert core.start()
    assert core.consider_frontiers([GOAL]) == []
    assert core.update_health(Health()) == []
    effects = core.update_health(PREFLIGHT_READY)
    assert [effect.kind for effect in effects] == [EffectKind.ARM]
    assert all(effect.kind is not EffectKind.NAVIGATE for effect in effects)
    assert core.arm_result(True) == []
    assert core.update_health(HEALTHY) == []
    assert [effect.kind for effect in core.consider_frontiers([GOAL])] == [EffectKind.NAVIGATE]


def test_no_frontier_stops_before_saving():
    core = make_exploring()
    effects = core.consider_frontiers([])
    assert core.state is ExplorationState.SAVING
    assert [effect.kind for effect in effects] == [EffectKind.HARD_STOP, EffectKind.SAVE]
    assert core.save_result(True) == []
    assert core.state is ExplorationState.COMPLETE


def test_failed_frontier_is_blacklisted_then_repeated_failure_faults():
    core = make_exploring()
    core.consider_frontiers([GOAL])
    effects = core.navigation_result(False)
    assert core.state is ExplorationState.EXPLORING
    assert GOAL.key in core.failed_frontiers
    assert [effect.kind for effect in effects] == [EffectKind.ZERO_STOP]
    core.consider_frontiers([GOAL])
    effects = core.navigation_result(False)
    assert core.state is ExplorationState.FAULT
    assert [effect.kind for effect in effects] == [EffectKind.HARD_STOP]


def test_health_loss_and_user_cancel_hard_stop_without_recovery():
    core = make_exploring()
    core.consider_frontiers([GOAL])
    effects = core.update_health(Health(scan=True, odom=True, tf=True, gateway_ready=True))
    assert core.state is ExplorationState.FAULT
    assert [effect.kind for effect in effects] == [EffectKind.HARD_STOP]
    assert core.start()
    assert [effect.kind for effect in core.stop()] == [EffectKind.HARD_STOP]
    assert core.state is ExplorationState.STOPPED


def test_each_required_sensor_loss_hard_stops_an_active_exploration():
    for missing in ("scan", "odom", "tf"):
        core = make_exploring()
        core.consider_frontiers([GOAL])
        values = dict(scan=True, odom=True, tf=True, gateway_ready=True, motion_healthy=True)
        values[missing] = False
        effects = core.update_health(Health(**values))
        assert core.state is ExplorationState.FAULT
        assert [effect.kind for effect in effects] == [EffectKind.HARD_STOP]
