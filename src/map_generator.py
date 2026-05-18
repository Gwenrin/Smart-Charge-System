"""Random map/scenario generator for Smart Charge System."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import random
from typing import Dict, List, Optional, Set

try:
    from .battery import MAX_ENERGY
    from .state import DroneState, Position
    from .tasks import Task
except ImportError:
    from battery import MAX_ENERGY
    from state import DroneState, Position
    from tasks import Task


@dataclass
class GeneratedScenario:
    grid: List[List[int]]
    chargers: Dict[Position, int]
    drones: List[DroneState]
    tasks: List[Task]
    seed: int


def in_bounds(grid: List[List[int]], position: Position) -> bool:
    x, y = position
    height = len(grid)
    width = len(grid[0])

    return 0 <= x < width and 0 <= y < height


def is_walkable(grid: List[List[int]], position: Position) -> bool:
    x, y = position
    return in_bounds(grid, position) and grid[y][x] == 0


def get_neighbors(grid: List[List[int]], position: Position) -> List[Position]:
    x, y = position

    moves = [
        (0, -1),
        (1, 0),
        (0, 1),
        (-1, 0),
    ]

    result: List[Position] = []

    for dx, dy in moves:
        next_position = (x + dx, y + dy)

        if is_walkable(grid, next_position):
            result.append(next_position)

    return result


def find_connected_components(grid: List[List[int]]) -> List[Set[Position]]:
    """
    Szuka spójnych obszarów wolnych pól.

    Starty, cele i ładowarki wybieramy z jednego największego obszaru,
    żeby mapa była fizycznie przechodnia.
    """

    height = len(grid)
    width = len(grid[0])

    visited: Set[Position] = set()
    components: List[Set[Position]] = []

    for y in range(height):
        for x in range(width):
            start = (x, y)

            if start in visited:
                continue

            if not is_walkable(grid, start):
                continue

            component: Set[Position] = set()
            queue = deque([start])
            visited.add(start)

            while queue:
                current = queue.popleft()
                component.add(current)

                for neighbor in get_neighbors(grid, current):
                    if neighbor not in visited:
                        visited.add(neighbor)
                        queue.append(neighbor)

            components.append(component)

    return components


def generate_obstacles(
    width: int,
    height: int,
    rng: random.Random,
    obstacle_count_min: int,
    obstacle_count_max: int,
    obstacle_width_max: int,
    obstacle_height_max: int,
) -> List[List[int]]:
    """
    Generuje przeszkody jako losowe prostokąty.

    W etapie końcowym parametry domyślne są dość łagodne,
    żeby wspólny planner miał realną szansę znaleźć trasę.
    """

    grid = [
        [0 for _ in range(width)]
        for _ in range(height)
    ]

    obstacle_count = rng.randint(obstacle_count_min, obstacle_count_max)

    for _ in range(obstacle_count):
        rect_width = rng.randint(1, obstacle_width_max)
        rect_height = rng.randint(1, obstacle_height_max)

        start_x = rng.randint(0, width - 1)
        start_y = rng.randint(0, height - 1)

        for y in range(start_y, min(start_y + rect_height, height)):
            for x in range(start_x, min(start_x + rect_width, width)):
                grid[y][x] = 1

    return grid


def generate_random_scenario(
    width: int = 8,
    height: int = 6,
    drone_count: int = 2,
    task_count: int = 2,
    min_chargers: int = 1,
    max_chargers: int = 3,
    initial_energy: int = MAX_ENERGY,
    charger_capacity: int = 1,
    obstacle_count_min: int = 1,
    obstacle_count_max: int = 4,
    obstacle_width_max: int = 2,
    obstacle_height_max: int = 2,
    max_attempts: int = 200,
    seed: Optional[int] = None,
) -> GeneratedScenario:
    """
    Generuje losowy scenariusz.

    Ważne:
    - liczba zadań powinna być równa liczbie dronów,
    - wtedy każdy dron dostaje jeden cel,
    - pełny wspólny planer planuje ich ruch jednocześnie.
    """

    if task_count != drone_count:
        raise ValueError(
            "W tej końcowej wersji 2D task_count musi być równe drone_count."
        )

    if seed is None:
        seed = random.randrange(1, 1_000_000_000)

    rng = random.Random(seed)

    charger_count = rng.randint(min_chargers, max_chargers)

    required_positions = drone_count + task_count + charger_count

    for _ in range(max_attempts):
        grid = generate_obstacles(
            width=width,
            height=height,
            rng=rng,
            obstacle_count_min=obstacle_count_min,
            obstacle_count_max=obstacle_count_max,
            obstacle_width_max=obstacle_width_max,
            obstacle_height_max=obstacle_height_max,
        )

        components = find_connected_components(grid)

        if not components:
            continue

        largest_component = max(components, key=len)

        if len(largest_component) < required_positions:
            continue

        positions = rng.sample(
            sorted(largest_component),
            required_positions,
        )

        drone_positions = positions[:drone_count]
        task_positions = positions[drone_count:drone_count + task_count]
        charger_positions = positions[drone_count + task_count:]

        drones = [
            DroneState(position=position, energy=initial_energy)
            for position in drone_positions
        ]

        tasks = [
            Task(id=i + 1, location=position)
            for i, position in enumerate(task_positions)
        ]

        chargers = {
            position: charger_capacity
            for position in charger_positions
        }

        return GeneratedScenario(
            grid=grid,
            chargers=chargers,
            drones=drones,
            tasks=tasks,
            seed=seed,
        )

    raise RuntimeError(
        "Nie udało się wygenerować poprawnej mapy. "
        "Zmniejsz liczbę przeszkód albo zwiększ rozmiar mapy."
    )


def print_scenario_ascii(scenario: GeneratedScenario) -> None:
    """
    Wypisuje mapę w konsoli.

    Oznaczenia:
    . = wolne pole
    # = przeszkoda
    S = start drona
    T = punkt dostawy
    C = ładowarka
    """

    grid_chars: List[List[str]] = []

    for row in scenario.grid:
        grid_chars.append([
            "#" if cell == 1 else "."
            for cell in row
        ])

    for charger_pos in scenario.chargers.keys():
        x, y = charger_pos
        grid_chars[y][x] = "C"

    for task in scenario.tasks:
        x, y = task.location
        grid_chars[y][x] = "T"

    for drone in scenario.drones:
        x, y = drone.position
        grid_chars[y][x] = "S"

    print()
    print("=" * 80)
    print(f"LOSOWA MAPA | seed={scenario.seed}")
    print("=" * 80)

    for row in grid_chars:
        print(" ".join(row))