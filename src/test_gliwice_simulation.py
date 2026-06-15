"""Testy nowej symulacji miejskiej."""

try:
    from .gliwice_scenario import generate_gliwice_scenario
    from .grid_map import GridMap
    from .mapf_cbs import plan_all_drones_mapf, validate_paths
except ImportError:
    from gliwice_scenario import generate_gliwice_scenario
    from grid_map import GridMap
    from mapf_cbs import plan_all_drones_mapf, validate_paths


def test_scenario_has_40_drones_and_unique_goals():
    scenario = generate_gliwice_scenario()

    assert len(scenario.drones) == 40
    assert len(scenario.tasks) == 40
    assert len(scenario.landing_goals) == 40
    assert len({task.location for task in scenario.tasks}) == 40
    assert len(set(scenario.landing_goals.values())) == 40


def test_all_drones_finish_without_conflicts():
    scenario = generate_gliwice_scenario()
    grid_map = GridMap(scenario.grid, scenario.chargers)

    paths, _ = plan_all_drones_mapf(
        grid_map=grid_map,
        drones=scenario.drones,
        tasks=scenario.tasks,
        landing_goals=scenario.landing_goals,
    )

    assert len(paths) == 40
    assert validate_paths(paths, scenario.chargers) == []

    for drone_id, path in paths.items():
        visited_positions = [state[0].position for state in path]

        assert scenario.tasks[drone_id].location in visited_positions
        assert path[-1][0].position == scenario.landing_goals[drone_id]
        assert path[-1][0].energy >= 1
