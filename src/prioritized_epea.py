"""Skalowalny planer 40 dronów: EPEA* niskiego poziomu i rezerwacje."""

from __future__ import annotations

from dataclasses import dataclass
import heapq
import itertools
from typing import Dict, List, Optional, Tuple

try:
    from .battery import get_move_cost
    from .city_config import (
        CITY_MAX_ENERGY,
        MAX_EXPANSIONS_PER_DRONE,
        MAX_TIME_STEPS,
        MIN_FINAL_ENERGY,
    )
    from .grid_map import GridMap
    from .reservation_table import ReservationTable
    from .state import DroneState, JointEnergyState, Position
    from .tasks import Task
except ImportError:
    from battery import get_move_cost
    from city_config import (
        CITY_MAX_ENERGY,
        MAX_EXPANSIONS_PER_DRONE,
        MAX_TIME_STEPS,
        MIN_FINAL_ENERGY,
    )
    from grid_map import GridMap
    from reservation_table import ReservationTable
    from state import DroneState, JointEnergyState, Position
    from tasks import Task


@dataclass(frozen=True)
class SearchState:
    position: Position
    energy: int
    time_step: int
    delivery_completed: bool


@dataclass
class SinglePlanResult:
    path: List[DroneState]
    expanded: int
    generated: int
    partial_reexpansions: int
    charging_steps: int
    delivery_time: int


@dataclass(frozen=True)
class OperatorSelection:
    successors: Tuple[SearchState, ...]
    next_delta_f: Optional[int]


def manhattan(first: Position, second: Position) -> int:
    return abs(first[0] - second[0]) + abs(first[1] - second[1])


def heuristic(
    position: Position,
    delivery_goal: Position,
    landing_goal: Position,
    delivery_completed: bool,
) -> int:
    if delivery_completed:
        return manhattan(position, landing_goal)

    return (
        manhattan(position, delivery_goal)
        + manhattan(delivery_goal, landing_goal)
    )


def _reconstruct_path(
    parents: Dict[SearchState, Optional[SearchState]],
    goal_state: SearchState,
    altitude: int,
) -> List[DroneState]:
    states: List[SearchState] = []
    current: Optional[SearchState] = goal_state

    while current is not None:
        states.append(current)
        current = parents[current]

    states.reverse()

    return [
        DroneState(
            position=state.position,
            energy=state.energy,
            altitude=altitude,
        )
        for state in states
    ]


def _legal_successor_operators(
    grid_map: GridMap,
    current: SearchState,
    delivery_goal: Position,
    landing_goal: Position,
    altitude: int,
    reservations: ReservationTable,
    max_energy: int,
    forbidden_positions: set[Position],
) -> List[SearchState]:
    if current.time_step >= MAX_TIME_STEPS:
        return []

    next_time = current.time_step + 1
    result: List[SearchState] = []

    # Oddzielny krok ładowania. Energia rośnie dopiero po pozostaniu
    # na stacji przez jedną jednostkę czasu.
    if (
        grid_map.is_charging_station(current.position)
        and current.energy < max_energy
        and reservations.is_position_free(
            current.position,
            altitude,
            next_time,
            is_charger=True,
        )
        and reservations.is_edge_free(
            current.position,
            current.position,
            altitude,
            next_time,
        )
    ):
        result.append(
            SearchState(
                position=current.position,
                energy=max_energy,
                time_step=next_time,
                delivery_completed=current.delivery_completed,
            )
        )

    for next_position in grid_map.neighbors_with_wait(current.position):
        if (
            next_position in forbidden_positions
            and next_position != landing_goal
            and not (
                current.time_step == 0
                and next_position == current.position
            )
        ):
            continue

        move_cost = get_move_cost(current.position, next_position)

        if move_cost > current.energy:
            continue

        next_is_charger = grid_map.is_charging_station(next_position)

        if not reservations.is_position_free(
            next_position,
            altitude,
            next_time,
            is_charger=next_is_charger,
        ):
            continue

        if not reservations.is_edge_free(
            current.position,
            next_position,
            altitude,
            next_time,
        ):
            continue

        next_delivery_completed = (
            current.delivery_completed
            or next_position == delivery_goal
        )

        result.append(
            SearchState(
                position=next_position,
                energy=current.energy - move_cost,
                time_step=next_time,
                delivery_completed=next_delivery_completed,
            )
        )

    result.sort(
        key=lambda state: (
            state.time_step
            + heuristic(
                state.position,
                delivery_goal,
                landing_goal,
                state.delivery_completed,
            ),
            -state.energy,
            state.position,
        )
    )

    return result


def _delta_f(
    current: SearchState,
    successor: SearchState,
    delivery_goal: Position,
    landing_goal: Position,
) -> int:
    current_f = current.time_step + heuristic(
        current.position,
        delivery_goal,
        landing_goal,
        current.delivery_completed,
    )
    successor_f = successor.time_step + heuristic(
        successor.position,
        delivery_goal,
        landing_goal,
        successor.delivery_completed,
    )

    return successor_f - current_f


def _operator_selection_function(
    grid_map: GridMap,
    current: SearchState,
    delivery_goal: Position,
    landing_goal: Position,
    altitude: int,
    reservations: ReservationTable,
    max_energy: int,
    forbidden_positions: set[Position],
    requested_delta_f: int,
) -> OperatorSelection:
    """
    EPEA* operator selection function.

    Zwraca tylko tych następców, których operator powoduje dokładnie
    requested_delta_f. Reszta nie jest rozwijana teraz; wyznaczamy tylko
    najbliższą większą wartość delta_f, aby ponownie włożyć stan do OPEN.
    """
    selected: List[SearchState] = []
    next_delta_f: Optional[int] = None

    for successor in _legal_successor_operators(
        grid_map=grid_map,
        current=current,
        delivery_goal=delivery_goal,
        landing_goal=landing_goal,
        altitude=altitude,
        reservations=reservations,
        max_energy=max_energy,
        forbidden_positions=forbidden_positions,
    ):
        successor_delta_f = _delta_f(
            current=current,
            successor=successor,
            delivery_goal=delivery_goal,
            landing_goal=landing_goal,
        )

        if successor_delta_f == requested_delta_f:
            selected.append(successor)
        elif successor_delta_f > requested_delta_f:
            if next_delta_f is None or successor_delta_f < next_delta_f:
                next_delta_f = successor_delta_f

    selected.sort(
        key=lambda state: (
            -state.energy,
            state.time_step,
            state.position,
        )
    )

    return OperatorSelection(
        successors=tuple(selected),
        next_delta_f=next_delta_f,
    )


def plan_single_drone_epea(
    grid_map: GridMap,
    start: DroneState,
    delivery_goal: Position,
    landing_goal: Position,
    reservations: ReservationTable,
    max_energy: int = CITY_MAX_ENERGY,
    max_expansions: int = MAX_EXPANSIONS_PER_DRONE,
    forbidden_positions: Optional[set[Position]] = None,
) -> Optional[SinglePlanResult]:
    """
    Planuje trasę start -> dostawa -> lądowanie.

    To jest niskopoziomowe EPEA*: stan jest rozwijany częściowo.
    Operator selection function generuje tylko następców o zadanej
    wartości delta_f. Ten sam stan wraca do OPEN z następną możliwą
    wartością delta_f, jeżeli istnieją jeszcze nierozwinięte operatory.
    """
    if forbidden_positions is None:
        forbidden_positions = set()

    if not grid_map.is_walkable(start.position):
        raise ValueError(f"Niepoprawny start: {start.position}")

    if not grid_map.is_walkable(delivery_goal):
        raise ValueError(f"Niepoprawny punkt dostawy: {delivery_goal}")

    if not grid_map.is_walkable(landing_goal):
        raise ValueError(f"Niepoprawne lądowisko: {landing_goal}")

    start_state = SearchState(
        position=start.position,
        energy=start.energy,
        time_step=0,
        delivery_completed=(start.position == delivery_goal),
    )

    start_h = heuristic(
        start.position,
        delivery_goal,
        landing_goal,
        start_state.delivery_completed,
    )

    # epea_f, h, licznik, delta_f obsługiwane przy tym rozwinięciu, stan
    open_heap: List[Tuple[int, int, int, int, SearchState]] = []
    counter = itertools.count()

    heapq.heappush(
        open_heap,
        (start_h, start_h, next(counter), 0, start_state),
    )

    parents: Dict[SearchState, Optional[SearchState]] = {
        start_state: None
    }

    # Dla tej samej pozycji, czasu i etapu większa energia dominuje mniejszą.
    best_energy: Dict[Tuple[Position, int, bool], int] = {
        (start.position, 0, start_state.delivery_completed): start.energy
    }

    expanded = 0
    generated = 0
    partial_reexpansions = 0

    while open_heap:
        _, _, _, requested_delta_f, current = heapq.heappop(open_heap)

        key = (
            current.position,
            current.time_step,
            current.delivery_completed,
        )

        if current.energy < best_energy.get(key, -1):
            continue

        if (
            current.delivery_completed
            and current.position == landing_goal
            and current.energy >= MIN_FINAL_ENERGY
        ):
            path = _reconstruct_path(
                parents,
                current,
                start.altitude,
            )

            charging_steps = sum(
                1
                for previous, following in zip(path, path[1:])
                if (
                    previous.position == following.position
                    and following.energy > previous.energy
                )
            )

            delivery_time = next(
                index
                for index, state in enumerate(path)
                if state.position == delivery_goal
            )

            return SinglePlanResult(
                path=path,
                expanded=expanded,
                generated=generated,
                partial_reexpansions=partial_reexpansions,
                charging_steps=charging_steps,
                delivery_time=delivery_time,
            )

        if requested_delta_f == 0:
            expanded += 1

            if expanded > max_expansions:
                return None
        else:
            partial_reexpansions += 1

        selection = _operator_selection_function(
            grid_map=grid_map,
            current=current,
            delivery_goal=delivery_goal,
            landing_goal=landing_goal,
            altitude=start.altitude,
            reservations=reservations,
            max_energy=max_energy,
            forbidden_positions=forbidden_positions,
            requested_delta_f=requested_delta_f,
        )

        for successor in selection.successors:
            successor_h = heuristic(
                successor.position,
                delivery_goal,
                landing_goal,
                successor.delivery_completed,
            )
            successor_f = successor.time_step + successor_h

            successor_key = (
                successor.position,
                successor.time_step,
                successor.delivery_completed,
            )

            previous_energy = best_energy.get(successor_key, -1)

            if successor.energy > previous_energy:
                best_energy[successor_key] = successor.energy
                parents[successor] = current
                heapq.heappush(
                    open_heap,
                    (
                        successor_f,
                        successor_h,
                        next(counter),
                        0,
                        successor,
                    ),
                )
                generated += 1

        if selection.next_delta_f is not None:
            current_h = heuristic(
                current.position,
                delivery_goal,
                landing_goal,
                current.delivery_completed,
            )
            next_f = current.time_step + current_h + selection.next_delta_f

            heapq.heappush(
                open_heap,
                (
                    next_f,
                    current_h,
                    next(counter),
                    selection.next_delta_f,
                    current,
                ),
            )

    return None


def _reserve_path(
    path: List[DroneState],
    grid_map: GridMap,
    reservations: ReservationTable,
) -> None:
    for arrival_time, (current, following) in enumerate(
        zip(path, path[1:]),
        start=1,
    ):
        reservations.reserve_step(
            current=current.position,
            next_position=following.position,
            altitude=current.altitude,
            arrival_time=arrival_time,
            next_is_charger=grid_map.is_charging_station(
                following.position
            ),
        )

    reservations.reserve_landing_permanently(
        path[-1].position,
        len(path) - 1,
    )


def plan_all_drones(
    grid_map: GridMap,
    drones: List[DroneState],
    tasks: List[Task],
    landing_goals: Dict[int, Position],
    max_energy: int = CITY_MAX_ENERGY,
) -> Tuple[Dict[int, List[JointEnergyState]], Dict[str, object]]:
    """
    Planuje drony priorytetowo, lecz wszystkie ścieżki zaczynają się w t=0.
    Najpierw obsługiwane są drony z mniejszą energią początkową.
    """
    if len(drones) != len(tasks):
        raise ValueError("Liczba dronów musi być równa liczbie zadań.")

    reservations = ReservationTable(grid_map.station_capacity)

    for drone in drones:
        reservations.reserve_start(
            drone.position,
            drone.altitude,
            is_charger=grid_map.is_charging_station(drone.position),
        )

    order = sorted(
        range(len(drones)),
        key=lambda drone_id: (
            drones[drone_id].energy,
            -manhattan(
                drones[drone_id].position,
                tasks[drone_id].location,
            ),
            drone_id,
        ),
    )

    paths: Dict[int, List[JointEnergyState]] = {}
    per_drone: Dict[int, Dict[str, int]] = {}

    for planning_index, drone_id in enumerate(order, start=1):
        drone = drones[drone_id]
        task = tasks[drone_id]
        landing_goal = landing_goals[drone_id]

        result = plan_single_drone_epea(
            grid_map=grid_map,
            start=drone,
            delivery_goal=task.location,
            landing_goal=landing_goal,
            reservations=reservations,
            max_energy=max_energy,
            forbidden_positions=(
                set(landing_goals.values()) - {landing_goal}
            ),
        )

        if result is None:
            raise RuntimeError(
                "Nie znaleziono trasy dla drona "
                f"{drone_id} w kolejności planowania {planning_index}."
            )

        _reserve_path(
            result.path,
            grid_map,
            reservations,
        )

        paths[drone_id] = [
            (state,)
            for state in result.path
        ]

        per_drone[drone_id] = {
            "expanded": result.expanded,
            "generated": result.generated,
            "partial_reexpansions": result.partial_reexpansions,
            "charging_steps": result.charging_steps,
            "delivery_time": result.delivery_time,
            "landing_time": len(result.path) - 1,
        }

    statistics: Dict[str, object] = {
        "planning_order": order,
        "per_drone": per_drone,
        "total_expanded": sum(
            values["expanded"]
            for values in per_drone.values()
        ),
        "total_generated": sum(
            values["generated"]
            for values in per_drone.values()
        ),
        "total_charging_steps": sum(
            values["charging_steps"]
            for values in per_drone.values()
        ),
        "makespan": max(
            values["landing_time"]
            for values in per_drone.values()
        ),
    }

    return paths, statistics


def validate_paths(
    paths: Dict[int, List[JointEnergyState]],
    chargers: Dict[Position, int],
) -> List[str]:
    """Zwraca listę wykrytych konfliktów. Pusta lista oznacza poprawność."""
    errors: List[str] = []

    if not paths:
        return errors

    max_length = max(len(path) for path in paths.values())

    def state_at(drone_id: int, time_step: int) -> DroneState:
        path = paths[drone_id]

        if time_step < len(path):
            return path[time_step][0]

        return path[-1][0]

    drone_ids = sorted(paths)

    for time_step in range(max_length):
        air_occupancy: Dict[Tuple[Position, int], int] = {}
        charger_occupancy: Dict[Position, int] = {}

        for drone_id in drone_ids:
            state = state_at(drone_id, time_step)

            if state.position in chargers:
                charger_occupancy[state.position] = (
                    charger_occupancy.get(state.position, 0) + 1
                )
            else:
                key = (state.position, state.altitude)

                if key in air_occupancy:
                    errors.append(
                        f"Konflikt pola {key} w czasie {time_step}: "
                        f"drony {air_occupancy[key]} i {drone_id}."
                    )
                else:
                    air_occupancy[key] = drone_id

        for position, count in charger_occupancy.items():
            if count > chargers[position]:
                errors.append(
                    f"Przekroczona pojemność ładowarki {position} "
                    f"w czasie {time_step}: {count}/{chargers[position]}."
                )

        if time_step == 0:
            continue

        for first_index, first_id in enumerate(drone_ids):
            first_previous = state_at(first_id, time_step - 1)
            first_current = state_at(first_id, time_step)

            for second_id in drone_ids[first_index + 1:]:
                second_previous = state_at(second_id, time_step - 1)
                second_current = state_at(second_id, time_step)

                if first_current.altitude != second_current.altitude:
                    continue

                if (
                    first_previous.position == second_current.position
                    and second_previous.position == first_current.position
                    and first_previous.position != first_current.position
                ):
                    errors.append(
                        f"Konflikt zamiany dronów {first_id} i {second_id} "
                        f"w czasie {time_step}."
                    )

    return errors