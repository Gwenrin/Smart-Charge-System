"""Generator scenariusza 40 dronów dla mapy Gliwic."""

from __future__ import annotations

from dataclasses import dataclass
import random
from typing import Dict, Iterable, List, Set

try:
    from .city_config import (
        ALTITUDE_LEVELS_M,
        CHARGER_CAPACITY,
        DRONES_PER_SIDE,
        GRID_HEIGHT,
        GRID_WIDTH,
        MAX_INITIAL_ENERGY,
        MIN_INITIAL_ENERGY,
        RANDOM_SEED,
    )
    from .state import DroneState, Position
    from .tasks import Task
except ImportError:
    from city_config import (
        ALTITUDE_LEVELS_M,
        CHARGER_CAPACITY,
        DRONES_PER_SIDE,
        GRID_HEIGHT,
        GRID_WIDTH,
        MAX_INITIAL_ENERGY,
        MIN_INITIAL_ENERGY,
        RANDOM_SEED,
    )
    from state import DroneState, Position
    from tasks import Task


@dataclass
class GliwiceScenario:
    grid: List[List[int]]
    chargers: Dict[Position, int]
    drones: List[DroneState]
    tasks: List[Task]
    landing_goals: Dict[int, Position]
    seed: int


def _distributed_rows(count: int) -> List[int]:
    """Rozmieszcza pola równomiernie od góry do dołu mapy."""
    margin = 2
    usable = GRID_HEIGHT - 2 * margin

    return [
        margin + round(index * (usable - 1) / (count - 1))
        for index in range(count)
    ]


def _paint_rectangle(
    grid: List[List[int]],
    x0: int,
    y0: int,
    width: int,
    height: int,
) -> None:
    for y in range(y0, min(y0 + height, GRID_HEIGHT)):
        for x in range(x0, min(x0 + width, GRID_WIDTH)):
            grid[y][x] = 1


def _create_grid() -> List[List[int]]:
    """
    Tworzy uproszczone strefy niedostępne.

    Nie są to rzeczywiste budynki Gliwic. Podkład OSM służy do zachowania
    skali i położenia, a prostokąty reprezentują modelowe strefy zakazu lotu.
    """
    grid = [
        [0 for _ in range(GRID_WIDTH)]
        for _ in range(GRID_HEIGHT)
    ]

    restricted_rectangles = [
        (27, 5, 5, 10),
        (41, 16, 6, 9),
        (25, 35, 7, 10),
        (48, 39, 5, 8),
    ]

    for rectangle in restricted_rectangles:
        _paint_rectangle(grid, *rectangle)

    return grid


def _is_walkable(grid: List[List[int]], position: Position) -> bool:
    x, y = position
    return (
        0 <= x < GRID_WIDTH
        and 0 <= y < GRID_HEIGHT
        and grid[y][x] == 0
    )


def _nearest_unique_slots(
    task_positions: Iterable[Position],
    landing_slots: Iterable[Position],
) -> List[Position]:
    available: Set[Position] = set(landing_slots)
    assigned: List[Position] = []

    for task_position in task_positions:
        selected = min(
            available,
            key=lambda candidate: (
                abs(candidate[0] - task_position[0])
                + abs(candidate[1] - task_position[1]),
                candidate[1],
            ),
        )
        assigned.append(selected)
        available.remove(selected)

    return assigned


def generate_gliwice_scenario(
    seed: int = RANDOM_SEED,
) -> GliwiceScenario:
    """Tworzy powtarzalny scenariusz dla 40 dronów."""
    rng = random.Random(seed)
    grid = _create_grid()
    rows = _distributed_rows(DRONES_PER_SIDE)

    left_starts = [(1, row) for row in rows]
    right_starts = [(GRID_WIDTH - 2, row) for row in rows]
    start_positions = left_starts + right_starts

    drones: List[DroneState] = []

    for drone_id, position in enumerate(start_positions):
        drones.append(
            DroneState(
                position=position,
                energy=rng.randint(
                    MIN_INITIAL_ENERGY,
                    MAX_INITIAL_ENERGY,
                ),
                altitude=ALTITUDE_LEVELS_M[
                    drone_id % len(ALTITUDE_LEVELS_M)
                ],
            )
        )

    chargers: Dict[Position, int] = {
        (18, 8): CHARGER_CAPACITY,
        (18, 20): CHARGER_CAPACITY,
        (18, 32): CHARGER_CAPACITY,
        (18, 44): CHARGER_CAPACITY,
        (36, 8): CHARGER_CAPACITY,
        (36, 20): CHARGER_CAPACITY,
        (36, 32): CHARGER_CAPACITY,
        (36, 44): CHARGER_CAPACITY,
        (54, 8): CHARGER_CAPACITY,
        (54, 20): CHARGER_CAPACITY,
        (54, 32): CHARGER_CAPACITY,
        (54, 44): CHARGER_CAPACITY,
    }

    occupied = set(start_positions) | set(chargers)
    task_positions: List[Position] = []

    while len(task_positions) < len(drones):
        position = (
            rng.randint(10, GRID_WIDTH - 11),
            rng.randint(3, GRID_HEIGHT - 4),
        )

        if position in occupied:
            continue

        if not _is_walkable(grid, position):
            continue

        occupied.add(position)
        task_positions.append(position)

    tasks = [
        Task(id=drone_id + 1, location=position)
        for drone_id, position in enumerate(task_positions)
    ]

    right_landings = [(GRID_WIDTH - 2, row) for row in rows]
    left_landings = [(1, row) for row in rows]

    assigned_right = _nearest_unique_slots(
        task_positions[:DRONES_PER_SIDE],
        right_landings,
    )
    assigned_left = _nearest_unique_slots(
        task_positions[DRONES_PER_SIDE:],
        left_landings,
    )

    all_landings = assigned_right + assigned_left
    landing_goals = {
        drone_id: position
        for drone_id, position in enumerate(all_landings)
    }

    return GliwiceScenario(
        grid=grid,
        chargers=chargers,
        drones=drones,
        tasks=tasks,
        landing_goals=landing_goals,
        seed=seed,
    )
