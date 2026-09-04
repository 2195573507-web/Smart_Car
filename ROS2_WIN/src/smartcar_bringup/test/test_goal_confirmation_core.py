import importlib.util
from pathlib import Path


CORE = Path(__file__).parents[1] / "scripts" / "goal_confirmation_core.py"
spec = importlib.util.spec_from_file_location("goal_confirmation_core_package", CORE)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def test_selection_only_creates_pending_goal():
    state = module.GoalConfirmation()
    state.select(1.0, 2.0, 0.5)
    assert state.state is module.GoalState.PENDING
    assert state.goal == (1.0, 2.0, 0.5)


def test_start_requires_health_and_cancel_clears():
    state = module.GoalConfirmation()
    state.select(1.0, 2.0, 0.5)
    assert not state.start(False)
    assert state.state is module.GoalState.PENDING
    assert state.start(True)
    assert state.state is module.GoalState.ACTIVE
    assert state.health_lost()
    assert state.state is module.GoalState.EMPTY


def test_preview_path_is_pending_only_and_cancel_clears_it():
    state = module.GoalConfirmation()
    assert not state.set_preview([(0.0, 0.0, 0.0)])
    state.select(1.0, 2.0, 0.5)
    assert state.set_preview([(0.0, 0.0, 0.0), (1.0, 2.0, 0.5)])
    assert state.preview == ((0.0, 0.0, 0.0), (1.0, 2.0, 0.5))
    assert state.cancel()
    assert state.preview is None
