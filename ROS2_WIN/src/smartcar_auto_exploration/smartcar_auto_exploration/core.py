"""Pure state machine for autonomous frontier exploration."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Iterable, Optional

from .frontier import Frontier


class ExplorationState(Enum):
    IDLE = "IDLE"
    PREFLIGHT = "PREFLIGHT"
    EXPLORING = "EXPLORING"
    SAVING = "SAVING"
    COMPLETE = "COMPLETE"
    FAULT = "FAULT"
    STOPPED = "STOPPED"


class EffectKind(Enum):
    ARM = "ARM"
    NAVIGATE = "NAVIGATE"
    ZERO_STOP = "ZERO_STOP"
    HARD_STOP = "HARD_STOP"
    SAVE = "SAVE"


@dataclass(frozen=True)
class Health:
    scan: bool = False
    odom: bool = False
    tf: bool = False
    gateway_ready: bool = False
    motion_healthy: bool = False

    @property
    def preflight_ready(self) -> bool:
        return self.scan and self.odom and self.tf and self.gateway_ready

    @property
    def all(self) -> bool:
        return self.preflight_ready and self.motion_healthy


@dataclass(frozen=True)
class Effect:
    kind: EffectKind
    goal: Optional[Frontier] = None
    reason: str = ""


class ExplorationCore:
    """Coordinates safe transitions while leaving all driving to Nav2."""

    def __init__(self, max_consecutive_failures: int = 3) -> None:
        if max_consecutive_failures <= 0:
            raise ValueError("max_consecutive_failures must be positive")
        self.state = ExplorationState.IDLE
        self._max_failures = max_consecutive_failures
        self._consecutive_failures = 0
        self._failed_frontiers: set[tuple[int, int]] = set()
        self._active_goal: Optional[Frontier] = None
        self._arm_requested = False
        self._arm_acknowledged = False
        self.fault_reason = ""

    @property
    def active_goal(self) -> Optional[Frontier]:
        return self._active_goal

    @property
    def failed_frontiers(self) -> set[tuple[int, int]]:
        return set(self._failed_frontiers)

    def start(self) -> bool:
        if self.state in (ExplorationState.PREFLIGHT, ExplorationState.EXPLORING,
                          ExplorationState.SAVING):
            return False
        self.state = ExplorationState.PREFLIGHT
        self._consecutive_failures = 0
        self._failed_frontiers.clear()
        self._active_goal = None
        self._arm_requested = False
        self._arm_acknowledged = False
        self.fault_reason = ""
        return True

    def update_health(self, health: Health) -> list[Effect]:
        if self.state is ExplorationState.PREFLIGHT:
            if not health.preflight_ready:
                return []
            if not self._arm_requested:
                self._arm_requested = True
                return [Effect(EffectKind.ARM)]
            if self._arm_acknowledged and health.all:
                self.state = ExplorationState.EXPLORING
            return []
        if self.state is ExplorationState.EXPLORING and not health.all:
            return self.fault("motion health lost")
        return []

    def arm_result(self, accepted: bool) -> list[Effect]:
        if self.state is not ExplorationState.PREFLIGHT:
            return []
        if accepted:
            self._arm_acknowledged = True
            return []
        return self.fault("motion gateway refused arm request")

    def consider_frontiers(self, frontiers: Iterable[Frontier]) -> list[Effect]:
        if self.state is not ExplorationState.EXPLORING or self._active_goal is not None:
            return []
        candidate = next(iter(frontiers), None)
        if candidate is None:
            self.state = ExplorationState.SAVING
            return [Effect(EffectKind.HARD_STOP, reason="no reachable frontier"),
                    Effect(EffectKind.SAVE)]
        self._active_goal = candidate
        return [Effect(EffectKind.NAVIGATE, goal=candidate)]

    def navigation_result(self, succeeded: bool) -> list[Effect]:
        if self.state is not ExplorationState.EXPLORING or self._active_goal is None:
            return []
        active_goal = self._active_goal
        self._active_goal = None
        if succeeded:
            self._consecutive_failures = 0
            return [Effect(EffectKind.ZERO_STOP, reason="frontier reached")]
        self._failed_frontiers.add(active_goal.key)
        self._consecutive_failures += 1
        if self._consecutive_failures >= self._max_failures:
            return self.fault("repeated Nav2 frontier failure")
        return [Effect(EffectKind.ZERO_STOP, reason="frontier navigation failed")]

    def stop(self, reason: str = "stop requested") -> list[Effect]:
        if self.state in (ExplorationState.IDLE, ExplorationState.COMPLETE,
                          ExplorationState.STOPPED):
            self.state = ExplorationState.STOPPED
            return [Effect(EffectKind.HARD_STOP, reason=reason)]
        self.state = ExplorationState.STOPPED
        self._active_goal = None
        return [Effect(EffectKind.HARD_STOP, reason=reason)]

    def save_result(self, succeeded: bool) -> list[Effect]:
        if self.state is not ExplorationState.SAVING:
            return []
        if succeeded:
            self.state = ExplorationState.COMPLETE
            return []
        return self.fault("map or pose graph save failed")

    def fault(self, reason: str) -> list[Effect]:
        if self.state is ExplorationState.FAULT:
            return []
        self.state = ExplorationState.FAULT
        self.fault_reason = reason
        self._active_goal = None
        return [Effect(EffectKind.HARD_STOP, reason=reason)]
