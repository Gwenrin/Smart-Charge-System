"""Final 2D stage using joint energy MAPF planner."""

from __future__ import annotations

from typing import Dict, List

from .battery import MAX_ENERGY
from .energy_mapf_epea import energy_mapf_epea
from .grid_map import GridMap
from .map_generator import generate_random_scenario, print_scenario_ascii
from .simulation import print_paths_timeline, print_simulation_summary
from .state import JointEnergyState
from .visualization import draw_environment


def convert_joint_path_to_paths(
    joint_path: List[JointEnergyState],
) -> Dict[int, List[JointEnergyState]]:
    """
    Zamienia wspólną ścieżkę:

        [
            (dron0, dron1),
            (dron0, dron1),
            ...
        ]

    na słownik:

        {
            0: [(dron0,), (dron0,), ...],
            1: [(dron1,), (dron1,), ...],
        }

    Ten format jest wygodniejszy do rysowania i wypisywania timeline.
    """

    paths: Dict[int, List[JointEnergyState]] = {}

    if not joint_path:
        return paths

    drone_count = len(joint_path[0])

    for drone_id in range(drone_count):
        paths[drone_id] = []

    for joint_state in joint_path:
        for drone_id, drone_state in enumerate(joint_state):
            paths[drone_id].append((drone_state,))

    return paths


def main() -> None:
    """
    Etap końcowy 2D.

    Nie używamy już schedulera ani osobnego conflict_resolvera.
    Planner dostaje wszystkie drony naraz i planuje ich wspólny ruch.
    """

    # None = losowa mapa przy każdym uruchomieniu.
    # Liczba, np. 12345 = zawsze ta sama mapa.
    SEED = None

    MAP_WIDTH = 8
    MAP_HEIGHT = 6

    DRONE_COUNT = 2
    TASK_COUNT = DRONE_COUNT

    MIN_CHARGERS = 1
    MAX_CHARGERS = 3

    MAX_SCENARIO_ATTEMPTS = 100

    final_scenario = None
    final_result = None
    final_stats = None

    for attempt in range(1, MAX_SCENARIO_ATTEMPTS + 1):
        scenario = generate_random_scenario(
            width=MAP_WIDTH,
            height=MAP_HEIGHT,
            drone_count=DRONE_COUNT,
            task_count=TASK_COUNT,
            min_chargers=MIN_CHARGERS,
            max_chargers=MAX_CHARGERS,
            initial_energy=MAX_ENERGY,
            charger_capacity=1,
            obstacle_count_min=1,
            obstacle_count_max=4,
            obstacle_width_max=2,
            obstacle_height_max=2,
            seed=SEED,
        )

        grid_map = GridMap(
            grid=scenario.grid,
            chargers=scenario.chargers,
        )

        starts = tuple(scenario.drones)
        goals = tuple(task.location for task in scenario.tasks)

        result = energy_mapf_epea(
            grid_map=grid_map,
            starts=starts,
            goals=goals,
            tasks=scenario.tasks,
            max_expansions=250_000,
            max_time_steps=45,
        )

        if result is not None:
            joint_path, stats = result
            final_scenario = scenario
            final_result = joint_path
            final_stats = stats
            print(f"[OK] Znaleziono mapę i trasę w próbie {attempt}.")
            break

        if SEED is not None:
            break

    if final_scenario is None or final_result is None or final_stats is None:
        print("Nie znaleziono wspólnej trasy dla wielu dronów.")
        print("Spróbuj zwiększyć MAX_ENERGY albo zmniejszyć liczbę przeszkód.")
        return

    scenario = final_scenario
    joint_path = final_result
    stats = final_stats

    paths = convert_joint_path_to_paths(joint_path)

    print_scenario_ascii(scenario)

    print()
    print("=" * 80)
    print("DRONY")
    print("=" * 80)

    for drone_id, drone in enumerate(scenario.drones):
        print(f"Dron {drone_id}: start={drone.position}, energia={drone.energy}")

    print()
    print("=" * 80)
    print("PUNKTY DOSTAW")
    print("=" * 80)

    for drone_id, task in enumerate(scenario.tasks):
        print(f"Dron {drone_id} -> Zadanie {task.id}: cel={task.location}")

    print()
    print("=" * 80)
    print("ŁADOWARKI")
    print("=" * 80)

    for pos, capacity in scenario.chargers.items():
        print(f"Ładowarka: {pos}, pojemność={capacity}")

    print()
    print("=" * 80)
    print("STATYSTYKI ALGORYTMU")
    print("=" * 80)

    for key, value in stats.items():
        print(f"{key}: {value}")

    print_paths_timeline(
        paths,
        title="WSPÓLNA TRASA WIELU DRONÓW",
    )

    print_simulation_summary(paths)

    draw_environment(
        grid=scenario.grid,
        chargers=scenario.chargers,
        tasks=scenario.tasks,
        joint_paths=paths,
    )


if __name__ == "__main__":
    main()