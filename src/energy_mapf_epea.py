"""Joint energy-aware MAPF planner for multiple drones.

To jest etap końcowy 2D:
- planujemy wszystkie drony jednocześnie,
- stan zawiera pozycję i energię każdego drona,
- sprawdzamy kolizje,
- sprawdzamy zamianę miejsc,
- uwzględniamy ładowarki,
- dron nie może zakończyć z energią 0.
"""

from __future__ import annotations

from collections import deque
import heapq
import itertools
from typing import Dict, List, Optional, Tuple

try:
    from .battery import (
        MAX_ENERGY,
        MIN_FINAL_ENERGY,
        consume,
        get_move_cost,
        recharge,
        will_have_energy,
    )
    from .grid_map import GridMap
    from .state import DroneState, JointEnergyState, Position
    from .tasks import Task
except ImportError:
    from battery import (
        MAX_ENERGY,
        MIN_FINAL_ENERGY,
        consume,
        get_move_cost,
        recharge,
        will_have_energy,
    )
    from grid_map import GridMap
    from state import DroneState, JointEnergyState, Position
    from tasks import Task


def build_distance_map(
    grid_map: GridMap,
    goal: Position,
) -> Dict[Position, int]:
    """
    BFS od celu do wszystkich pól.

    Dzięki temu heurystyka jest szybka:
    zamiast za każdym razem liczyć BFS, korzystamy z gotowej mapy odległości.
    """

    distances: Dict[Position, int] = {
        goal: 0
    }

    queue = deque([goal])

    while queue:
        current = queue.popleft()
        current_distance = distances[current]

        for neighbor in grid_map.neighbors_with_wait(current):
            if neighbor == current:
                continue

            if neighbor not in distances:
                distances[neighbor] = current_distance + 1
                queue.append(neighbor)

    return distances


def heuristic(
    state: JointEnergyState,
    distance_maps: List[Dict[Position, int]],
) -> int:
    """
    Heurystyka dla wspólnego planowania.

    Używamy maksymalnej odległości do celu, bo koszt g oznacza czas,
    czyli liczbę wspólnych kroków całej symulacji.
    """

    h = 0

    for drone, distance_map in zip(state, distance_maps):
        distance = distance_map.get(drone.position, 10_000)
        h = max(h, distance)

    return h


def is_goal_state(
    state: JointEnergyState,
    goals: Tuple[Position, ...],
) -> bool:
    """
    Sprawdza, czy każdy dron jest w swoim celu
    i czy ma minimalny zapas energii.
    """

    for drone, goal in zip(state, goals):
        if drone.position != goal:
            return False

        if drone.energy < MIN_FINAL_ENERGY:
            return False

    return True


def has_vertex_conflict(
    next_state: JointEnergyState,
    grid_map: GridMap,
) -> bool:
    """
    Konflikt wierzchołkowy:
    dwa drony nie mogą być w tym samym polu w tym samym czasie.

    Wyjątek:
    ładowarka może mieć capacity > 1.
    """

    counts: Dict[Position, int] = {}

    for drone in next_state:
        counts[drone.position] = counts.get(drone.position, 0) + 1

    for position, count in counts.items():
        if count <= 1:
            continue

        if grid_map.is_charging_station(position):
            capacity = grid_map.get_station_capacity(position)

            if count <= capacity:
                continue

        return True

    return False


def has_swap_conflict(
    current_state: JointEnergyState,
    next_state: JointEnergyState,
) -> bool:
    """
    Konflikt zamiany:
    dron A: X -> Y
    dron B: Y -> X
    """

    drone_count = len(current_state)

    for i in range(drone_count):
        for j in range(i + 1, drone_count):
            current_i = current_state[i].position
            current_j = current_state[j].position

            next_i = next_state[i].position
            next_j = next_state[j].position

            if current_i == next_j and current_j == next_i:
                return True

    return False


def is_valid_transition(
    current_state: JointEnergyState,
    next_state: JointEnergyState,
    grid_map: GridMap,
) -> bool:
    """Sprawdza, czy wspólny ruch jest bezkolizyjny."""

    if has_vertex_conflict(next_state, grid_map):
        return False

    if has_swap_conflict(current_state, next_state):
        return False

    return True


def single_drone_successors(
    grid_map: GridMap,
    drone: DroneState,
    goal: Position,
    distance_map: Dict[Position, int],
) -> List[DroneState]:
    """
    Generuje możliwe następne stany jednego drona.

    Optymalizacja:
    - jeśli dron jest już w celu i ma energię >= MIN_FINAL_ENERGY,
      to pozwalamy mu tylko czekać w celu,
    - to mocno zmniejsza przestrzeń stanów.
    """

    if drone.position == goal and drone.energy >= MIN_FINAL_ENERGY:
        return [
            DroneState(position=drone.position, energy=drone.energy)
        ]

    result: List[DroneState] = []

    for next_position in grid_map.neighbors_with_wait(drone.position):
        move_cost = get_move_cost(drone.position, next_position)

        if not will_have_energy(drone.energy, move_cost):
            continue

        if grid_map.is_charging_station(next_position):
            new_energy = recharge(drone.energy)
        else:
            new_energy = consume(drone.energy, move_cost)

        if new_energy < 0:
            continue

        result.append(
            DroneState(
                position=next_position,
                energy=new_energy,
            )
        )

    # Lepsze ruchy najpierw: sortujemy po odległości do celu.
    result.sort(
        key=lambda state: (
            distance_map.get(state.position, 10_000),
            -state.energy,
        )
    )

    return result


def generate_joint_successors(
    grid_map: GridMap,
    current_state: JointEnergyState,
    goals: Tuple[Position, ...],
    distance_maps: List[Dict[Position, int]],
) -> List[JointEnergyState]:
    """
    Generuje wspólne następne stany wszystkich dronów.
    """

    options_per_drone: List[List[DroneState]] = []

    for drone, goal, distance_map in zip(current_state, goals, distance_maps):
        options = single_drone_successors(
            grid_map=grid_map,
            drone=drone,
            goal=goal,
            distance_map=distance_map,
        )

        if not options:
            return []

        options_per_drone.append(options)

    successors: List[JointEnergyState] = []

    for combination in itertools.product(*options_per_drone):
        next_state = tuple(combination)

        if not is_valid_transition(
            current_state=current_state,
            next_state=next_state,
            grid_map=grid_map,
        ):
            continue

        successors.append(next_state)

    return successors


def reconstruct_path(
    parents: Dict[JointEnergyState, Optional[JointEnergyState]],
    goal_state: JointEnergyState,
) -> List[JointEnergyState]:
    """Odtwarza ścieżkę od startu do celu."""

    path: List[JointEnergyState] = []
    current: Optional[JointEnergyState] = goal_state

    while current is not None:
        path.append(current)
        current = parents[current]

    path.reverse()
    return path


def energy_mapf_epea(
    grid_map: GridMap,
    starts: JointEnergyState,
    goals: Tuple[Position, ...],
    tasks: Optional[List[Task]] = None,
    max_expansions: int = 250_000,
    max_time_steps: int = 45,
) -> Optional[Tuple[List[JointEnergyState], Dict[str, int]]]:
    """
    Wspólny planner dla wielu dronów.

    Uwaga:
    To jest praktyczna wersja końcowa 2D. Nazwa zostaje energy_mapf_epea,
    ale implementacja jest stabilnym wspólnym A*/MAPF z elementami
    redukcji przestrzeni stanów.
    """

    if len(starts) != len(goals):
        raise ValueError("Liczba dronów musi być taka sama jak liczba celów.")

    for drone in starts:
        if not grid_map.is_walkable(drone.position):
            raise ValueError(f"Niepoprawna pozycja startowa: {drone.position}")

    for goal in goals:
        if not grid_map.is_walkable(goal):
            raise ValueError(f"Niepoprawny cel: {goal}")

    if has_vertex_conflict(starts, grid_map):
        raise ValueError("Dwa drony startują z tego samego pola.")

    distance_maps = [
        build_distance_map(grid_map, goal)
        for goal in goals
    ]

    for drone, distance_map in zip(starts, distance_maps):
        if drone.position not in distance_map:
            return None

    open_heap: List[Tuple[int, int, int, JointEnergyState, int]] = []

    counter = 0
    start_g = 0
    start_h = heuristic(starts, distance_maps)
    start_f = start_g + start_h

    heapq.heappush(
        open_heap,
        (start_f, start_h, counter, starts, 0),
    )

    parents: Dict[JointEnergyState, Optional[JointEnergyState]] = {
        starts: None
    }

    best_g: Dict[JointEnergyState, int] = {
        starts: 0
    }

    stats = {
        "expanded": 0,
        "generated": 0,
        "skipped": 0,
        "max_open_size": 1,
    }

    while open_heap:
        _, _, _, current_state, current_g = heapq.heappop(open_heap)

        if current_g != best_g.get(current_state, 10**9):
            stats["skipped"] += 1
            continue

        stats["expanded"] += 1

        if stats["expanded"] > max_expansions:
            return None

        if current_g > max_time_steps:
            stats["skipped"] += 1
            continue

        if is_goal_state(
            state=current_state,
            goals=goals,
        ):
            path = reconstruct_path(
                parents=parents,
                goal_state=current_state,
            )

            return path, stats

        successors = generate_joint_successors(
            grid_map=grid_map,
            current_state=current_state,
            goals=goals,
            distance_maps=distance_maps,
        )

        for next_state in successors:
            next_g = current_g + 1

            if next_g > max_time_steps:
                continue

            old_g = best_g.get(next_state)

            if old_g is not None and next_g >= old_g:
                stats["skipped"] += 1
                continue

            best_g[next_state] = next_g
            parents[next_state] = current_state

            next_h = heuristic(next_state, distance_maps)
            next_f = next_g + next_h

            counter += 1

            heapq.heappush(
                open_heap,
                (next_f, next_h, counter, next_state, next_g),
            )

            stats["generated"] += 1

        stats["max_open_size"] = max(
            stats["max_open_size"],
            len(open_heap),
        )

    return None