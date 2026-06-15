"""Uruchomienie symulacji 40 dronów bez JupyterLab."""

from __future__ import annotations

try:
    from .gliwice_scenario import generate_gliwice_scenario
    from .grid_map import GridMap
    from .mapf_cbs import plan_all_drones_mapf, validate_paths
    from .visualization import draw_gliwice_environment
except ImportError:
    from gliwice_scenario import generate_gliwice_scenario
    from grid_map import GridMap
    from mapf_cbs import plan_all_drones_mapf, validate_paths
    from visualization import draw_gliwice_environment


def main() -> None:
    scenario = generate_gliwice_scenario()
    grid_map = GridMap(scenario.grid, scenario.chargers)

    print("Planowanie tras 40 dronów...")

    paths, statistics = plan_all_drones_mapf(
        grid_map=grid_map,
        drones=scenario.drones,
        tasks=scenario.tasks,
        landing_goals=scenario.landing_goals,
    )

    errors = validate_paths(paths, scenario.chargers)

    print()
    print("=" * 72)
    print("PODSUMOWANIE")
    print("=" * 72)
    print(f"Drony: {len(paths)}")
    print(f"Czas zakończenia wszystkich lotów: {statistics['makespan']}")
    print(f"Rozwinięte stany: {statistics['total_expanded']}")
    print(f"Wygenerowane stany: {statistics['total_generated']}")
    print(f"Liczba ładowań: {statistics['total_charging_steps']}")
    print(f"Wykryte konflikty: {len(errors)}")

    if errors:
        for error in errors:
            print(f"[BŁĄD] {error}")
    else:
        print("[OK] Wszystkie trasy są bezkolizyjne.")

    draw_gliwice_environment(
        grid=scenario.grid,
        chargers=scenario.chargers,
        tasks=scenario.tasks,
        paths=paths,
        landing_goals=scenario.landing_goals,
    )

    import matplotlib.pyplot as plt

    plt.show()


if __name__ == "__main__":
    main()
