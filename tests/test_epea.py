from src.conflict import is_valid_transition
from src.grid_map import GridMap
from src.mapf_epea import epea_star_mapf


def test_epea_finds_path_for_two_agents():
    grid = [
        [0, 0, 0, 0],
        [0, 0, 0, 0],
    ]

    starts = (
        (0, 0),
        (0, 1),
    )

    goals = (
        (3, 0),
        (3, 1),
    )

    grid_map = GridMap(grid)

    path, stats = epea_star_mapf(
        grid_map=grid_map,
        starts=starts,
        goals=goals,
        max_expansions=10_000,
    )

    assert path is not None
    assert path[0] == starts
    assert path[-1] == goals
    assert stats["expanded"] > 0


def test_epea_path_has_no_conflicts():
    grid = [
        [0, 0, 0, 0],
        [0, 0, 0, 0],
    ]

    starts = (
        (0, 0),
        (0, 1),
    )

    goals = (
        (3, 0),
        (3, 1),
    )

    grid_map = GridMap(grid)

    path, _ = epea_star_mapf(
        grid_map=grid_map,
        starts=starts,
        goals=goals,
        max_expansions=10_000,
    )

    assert path is not None

    for current_state, next_state in zip(path, path[1:]):
        assert is_valid_transition(current_state, next_state) is True