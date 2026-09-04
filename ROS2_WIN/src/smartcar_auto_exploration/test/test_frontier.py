from smartcar_auto_exploration.frontier import Grid, select_frontiers


def make_grid(rows):
    return Grid(len(rows[0]), len(rows), 1.0, 0.0, 0.0,
                tuple(value for row in rows for value in row))


def test_selects_reachable_free_frontier_only():
    grid = make_grid([
        [100, 100, 100, 100, 100, 100, 100],
        [100,   0,   0,   0,  -1,  -1, 100],
        [100,   0,   0,   0,  -1,  -1, 100],
        [100,   0,   0,   0,  -1,  -1, 100],
        [100, 100, 100, 100, 100, 100, 100],
    ])
    frontiers = select_frontiers(grid, 1.5, 2.5, min_cluster_cells=2,
                                 clearance_cells=0, min_distance_m=0.1)
    assert frontiers
    assert all(grid.value(frontier.row, frontier.column) == 0 for frontier in frontiers)
    assert all(frontier.column == 3 for frontier in frontiers)


def test_excludes_failed_component_and_known_obstacle_clearance():
    grid = make_grid([
        [100, 100, 100, 100, 100, 100, 100, 100, 100],
        [100,   0,   0,  -1,   0,   0,   0,  -1, 100],
        [100,   0,   0,   0,   0,   0,   0,   0, 100],
        [100,   0,   0,  -1, 100,   0,   0,  -1, 100],
        [100, 100, 100, 100, 100, 100, 100, 100, 100],
    ])
    initial = select_frontiers(grid, 1.5, 3.5, min_cluster_cells=2,
                               clearance_cells=0, min_distance_m=0.1)
    assert len(initial) == 2
    remaining = select_frontiers(grid, 1.5, 3.5, min_cluster_cells=2,
                                 clearance_cells=0, min_distance_m=0.1,
                                 excluded_keys=[initial[0].key])
    assert [frontier.key for frontier in remaining] == [initial[1].key]


def test_returns_no_frontier_for_fully_known_or_unreachable_map():
    grid = make_grid([
        [100, 100, 100, 100, 100],
        [100,   0,   0,   0, 100],
        [100,   0,   0,   0, 100],
        [100, 100, 100, 100, 100],
    ])
    assert select_frontiers(grid, 1.5, 1.5, min_cluster_cells=1,
                            clearance_cells=0, min_distance_m=0.0) == []
