from dataclasses import dataclass
from enum import Enum
from typing import Iterable, Optional, Tuple


class GoalState(Enum):
    EMPTY = "empty"
    PENDING = "pending"
    ACTIVE = "active"


@dataclass
class GoalConfirmation:
    state: GoalState = GoalState.EMPTY
    goal: Optional[Tuple[float, float, float]] = None
    preview: Optional[Tuple[Tuple[float, float, float], ...]] = None

    def select(self, x: float, y: float, yaw: float) -> None:
        self.goal = (float(x), float(y), float(yaw))
        self.state = GoalState.PENDING
        self.preview = None

    def set_preview(self, poses: Iterable[Tuple[float, float, float]]) -> bool:
        if self.state is not GoalState.PENDING:
            return False
        self.preview = tuple((float(x), float(y), float(yaw)) for x, y, yaw in poses)
        return True

    def clear_preview(self) -> None:
        self.preview = None

    def start(self, healthy: bool) -> bool:
        if self.state is not GoalState.PENDING or not healthy:
            return False
        self.state = GoalState.ACTIVE
        return True

    def cancel(self) -> bool:
        changed = self.state is not GoalState.EMPTY
        self.state = GoalState.EMPTY
        self.goal = None
        self.preview = None
        return changed

    def health_lost(self) -> bool:
        return self.cancel()

    def navigation_failed(self) -> bool:
        return self.cancel()
