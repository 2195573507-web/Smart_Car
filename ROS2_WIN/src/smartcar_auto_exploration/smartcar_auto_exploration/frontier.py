"""Reachable frontier extraction for an OccupancyGrid.

This module only selects map goals. It deliberately contains no velocity,
path-following, or obstacle-avoidance logic; Nav2 owns those responsibilities.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
from typing import Iterable, Sequence


Cell = tuple[int, int]


@dataclass(frozen=True)
class Grid:
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    data: Sequence[int]

    def __post_init__(self) -> None:
        if self.width <= 0 or self.height <= 0 or self.resolution <= 0.0:
            raise ValueError("grid dimensions and resolution must be positive")
        if len(self.data) != self.width * self.height:
            raise ValueError("grid data length does not match dimensions")

    def value(self, row: int, column: int) -> int:
        return int(self.data[row * self.width + column])

    def contains(self, row: int, column: int) -> bool:
        return 0 <= row < self.height and 0 <= column < self.width

    def cell_from_world(self, x: float, y: float) -> Cell:
        return (int(math.floor((y - self.origin_y) / self.resolution)),
                int(math.floor((x - self.origin_x) / self.resolution)))

    def world_from_cell(self, row: int, column: int) -> tuple[float, float]:
        return (self.origin_x + (column + 0.5) * self.resolution,
                self.origin_y + (row + 0.5) * self.resolution)


@dataclass(frozen=True)
class Frontier:
    key: Cell
    row: int
    column: int
    x: float
    y: float
    cells: int
    distance_m: float


def _neighbors8(grid: Grid, row: int, column: int) -> Iterable[Cell]:
    for row_delta in (-1, 0, 1):
        for column_delta in (-1, 0, 1):
            if row_delta == 0 and column_delta == 0:
                continue
            candidate = row + row_delta, column + column_delta
            if grid.contains(*candidate):
                yield candidate


def _reachable_free_cells(grid: Grid, start: Cell) -> set[Cell]:
    if not grid.contains(*start) or grid.value(*start) != 0:
        return set()
    reachable = {start}
    pending: deque[Cell] = deque([start])
    while pending:
        row, column = pending.popleft()
        for candidate in _neighbors8(grid, row, column):
            if candidate not in reachable and grid.value(*candidate) == 0:
                reachable.add(candidate)
                pending.append(candidate)
    return reachable


def _has_unknown_neighbor(grid: Grid, cell: Cell) -> bool:
    return any(grid.value(*candidate) < 0 for candidate in _neighbors8(grid, *cell))


def _clear_of_known_obstacle(grid: Grid, row: int, column: int,
                             clearance_cells: int, occupied_threshold: int) -> bool:
    for candidate_row in range(row - clearance_cells, row + clearance_cells + 1):
        for candidate_column in range(column - clearance_cells, column + clearance_cells + 1):
            if not grid.contains(candidate_row, candidate_column):
                return False
            if grid.value(candidate_row, candidate_column) >= occupied_threshold:
                return False
    return True


def select_frontiers(grid: Grid, robot_x: float, robot_y: float,
                     min_cluster_cells: int = 4, clearance_cells: int = 1,
                     min_distance_m: float = 0.30,
                     excluded_keys: Iterable[Cell] = ()) -> list[Frontier]:
    """Return reachable free frontier goals ordered for Nav2.

    A frontier is a reachable free cell adjacent to unknown space. Components
    that are too small, too close to a known obstacle, too near the robot, or
    previously failed are omitted. The returned point is always a free cell.
    """
    if min_cluster_cells <= 0 or clearance_cells < 0 or min_distance_m < 0.0:
        raise ValueError("frontier thresholds must be non-negative")
    reachable = _reachable_free_cells(grid, grid.cell_from_world(robot_x, robot_y))
    frontier_cells = {cell for cell in reachable if _has_unknown_neighbor(grid, cell)}
    excluded = set(excluded_keys)
    candidates: list[tuple[float, Frontier]] = []
    while frontier_cells:
        seed = frontier_cells.pop()
        component = {seed}
        pending: deque[Cell] = deque([seed])
        while pending:
            cell = pending.popleft()
            for neighbor in _neighbors8(grid, *cell):
                if neighbor in frontier_cells:
                    frontier_cells.remove(neighbor)
                    component.add(neighbor)
                    pending.append(neighbor)
        key = min(component)
        if len(component) < min_cluster_cells or key in excluded:
            continue
        centroid_row = sum(cell[0] for cell in component) / len(component)
        centroid_column = sum(cell[1] for cell in component) / len(component)
        ordered_cells = sorted(component, key=lambda cell: (
            (cell[0] - centroid_row) ** 2 + (cell[1] - centroid_column) ** 2,
            cell,
        ))
        selected: Frontier | None = None
        for row, column in ordered_cells:
            if not _clear_of_known_obstacle(grid, row, column, clearance_cells, 50):
                continue
            x, y = grid.world_from_cell(row, column)
            distance = math.hypot(x - robot_x, y - robot_y)
            if distance < min_distance_m:
                continue
            selected = Frontier(key, row, column, x, y, len(component), distance)
            break
        if selected is not None:
            # Prefer nearer components while modestly rewarding useful frontier extent.
            candidates.append((selected.distance_m - 0.02 * selected.cells * grid.resolution,
                               selected))
    return [candidate for _, candidate in sorted(candidates, key=lambda item: (item[0], item[1].key))]
