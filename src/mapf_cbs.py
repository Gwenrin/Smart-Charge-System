"""Full MAPF planner: CBS high level with EPEA* low-level search."""

from __future__ import annotations

from dataclasses import dataclass, replace
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
    from .prioritized_epea import plan_all_drones as plan_prioritized_epea
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
    from prioritized_epea import plan_all_drones as plan_prioritized_epea
    from state import DroneState, JointEnergyState, Position
    from tasks import Task


Move = Tuple[Position, Position]


@dataclass(frozen=True)
class SearchState:
    position: Position
    energy: int
    time_step: int
    delivery_completed: bool


@dataclass(frozen=True)
class OperatorSelection:
    successors: Tuple[SearchState, ...]
    next_delta_f: Optional[int]


@dataclass(frozen=True)
class VertexConstraint:
    drone_id: int
    position: Position
    time_step: int


@dataclass(frozen=True)
class EdgeConstraint:
    drone_id: int
    current: Position
    next_position: Position
    arrival_time: int


@dataclass(frozen=True)
class ConstraintSet:
    vertex: Tuple[VertexConstraint, ...] = ()
    edge: Tuple[EdgeConstraint, ...] = ()


@dataclass(frozen=True)
class ConstraintTable:
    vertex: Dict[int, frozenset[Position]]
    edge: Dict[int, frozenset[Move]]
    max_time: int


@dataclass(frozen=True)
class SinglePlanResult:
    path: List[DroneState]
    expanded: int
    generated: int
    partial_reexpansions: int
    charging_steps: int
    delivery_time: int


@dataclass(frozen=True)
class Conflict:
    kind: str
    first_id: int
    second_id: int
    time_step: int
    position: Optional[Position] = None
    first_from: Optional[Position] = None
    first_to: Optional[Position] = None
    second_from: Optional[Position] = None
    second_to: Optional[Position] = None


@dataclass
class CBSNode:
    constraints: ConstraintSet
    paths: Dict[int, List[DroneState]]
    per_drone: Dict[int, Dict[str, int]]
    sum_of_costs: int
    makespan: int


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


def _state_at(path: List[DroneState], time_step: int) -> DroneState:
    if time_step < len(path):
        return path[time_step]

    return path[-1]


def _path_cost(path: List[DroneState]) -> int:
    return len(path) - 1


def _charging_steps(path: List[DroneState]) -> int:
    return sum(
        1
        for previous, following in zip(path, path[1:])
        if previous.position == following.position
        and following.energy > previous.energy
    )


def _delivery_time(path: List[DroneState], delivery_goal: Position) -> int:
    return next(
        index
        for index, state in enumerate(path)
        if state.position == delivery_goal
    )


def _make_plan_stats(
    result: SinglePlanResult,
    path: List[DroneState],
) -> Dict[str, int]:
    return {
        "expanded": result.expanded,
        "generated": result.generated,
        "partial_reexpansions": result.partial_reexpansions,
        "charging_steps": result.charging_steps,
        "delivery_time": result.delivery_time,
        "landing_time": len(path) - 1,
    }


def _build_constraint_table(
    constraints: ConstraintSet,
    drone_id: int,
) -> ConstraintTable:
    vertex: Dict[int, set[Position]] = {}
    edge: Dict[int, set[Move]] = {}
    max_time = 0

    for constraint in constraints.vertex:
        if constraint.drone_id != drone_id:
            continue

        vertex.setdefault(constraint.time_step, set()).add(
            constraint.position
        )
        max_time = max(max_time, constraint.time_step)

    for constraint in constraints.edge:
        if constraint.drone_id != drone_id:
            continue

        edge.setdefault(constraint.arrival_time, set()).add(
            (constraint.current, constraint.next_position)
        )
        max_time = max(max_time, constraint.arrival_time)

    return ConstraintTable(
        vertex={
            time_step: frozenset(positions)
            for time_step, positions in vertex.items()
        },
        edge={
            time_step: frozenset(moves)
            for time_step, moves in edge.items()
        },
        max_time=max_time,
    )


def _violates_vertex_constraint(
    table: ConstraintTable,
    position: Position,
    time_step: int,
) -> bool:
    return position in table.vertex.get(time_step, frozenset())


def _violates_edge_constraint(
    table: ConstraintTable,
    current: Position,
    next_position: Position,
    arrival_time: int,
) -> bool:
    return (
        current,
        next_position,
    ) in table.edge.get(arrival_time, frozenset())


def _goal_is_safe(
    state: SearchState,
    landing_goal: Position,
    constraint_table: ConstraintTable,
) -> bool:
    if state.position != landing_goal:
        return False

    if not state.delivery_completed or state.energy < MIN_FINAL_ENERGY:
        return False

    for time_step, blocked_positions in constraint_table.vertex.items():
        if time_step >= state.time_step and state.position in blocked_positions:
            return False

    return True


def _legal_successor_operators(
    grid_map: GridMap,
    current: SearchState,
    delivery_goal: Position,
    landing_goal: Position,
    altitude: int,
    constraint_table: ConstraintTable,
    max_energy: int,
    forbidden_positions: set[Position],
) -> List[SearchState]:
    if current.time_step >= MAX_TIME_STEPS:
        return []

    next_time = current.time_step + 1
    result: List[SearchState] = []

    if (
        grid_map.is_charging_station(current.position)
        and current.energy < max_energy
        and not _violates_vertex_constraint(
            constraint_table,
            current.position,
            next_time,
        )
        and not _violates_edge_constraint(
            constraint_table,
            current.position,
            current.position,
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

        if _violates_vertex_constraint(
            constraint_table,
            next_position,
            next_time,
        ):
            continue

        if _violates_edge_constraint(
            constraint_table,
            current.position,
            next_position,
            next_time,
        ):
            continue

        result.append(
            SearchState(
                position=next_position,
                energy=current.energy - move_cost,
                time_step=next_time,
                delivery_completed=(
                    current.delivery_completed
                    or next_position == delivery_goal
                ),
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
    constraint_table: ConstraintTable,
    max_energy: int,
    forbidden_positions: set[Position],
    requested_delta_f: int,
) -> OperatorSelection:
    selected: List[SearchState] = []
    next_delta_f: Optional[int] = None

    for successor in _legal_successor_operators(
        grid_map=grid_map,
        current=current,
        delivery_goal=delivery_goal,
        landing_goal=landing_goal,
        altitude=altitude,
        constraint_table=constraint_table,
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


def _plan_single_drone_constrained_epea(
    grid_map: GridMap,
    drone_id: int,
    start: DroneState,
    delivery_goal: Position,
    landing_goal: Position,
    constraints: ConstraintSet,
    max_energy: int,
    max_expansions: int,
    forbidden_positions: set[Position],
) -> Optional[SinglePlanResult]:
    constraint_table = _build_constraint_table(
        constraints=constraints,
        drone_id=drone_id,
    )

    if _violates_vertex_constraint(
        constraint_table,
        start.position,
        0,
    ):
        return None

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

    open_heap: List[Tuple[int, int, int, int, SearchState]] = []
    counter = itertools.count()
    heapq.heappush(
        open_heap,
        (start_h, start_h, next(counter), 0, start_state),
    )

    parents: Dict[SearchState, Optional[SearchState]] = {
        start_state: None
    }
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

        if _goal_is_safe(
            state=current,
            landing_goal=landing_goal,
            constraint_table=constraint_table,
        ):
            path = _reconstruct_path(
                parents=parents,
                goal_state=current,
                altitude=start.altitude,
            )

            return SinglePlanResult(
                path=path,
                expanded=expanded,
                generated=generated,
                partial_reexpansions=partial_reexpansions,
                charging_steps=_charging_steps(path),
                delivery_time=_delivery_time(path, delivery_goal),
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
            constraint_table=constraint_table,
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

            if successor.energy <= best_energy.get(successor_key, -1):
                continue

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
            heapq.heappush(
                open_heap,
                (
                    current.time_step
                    + current_h
                    + selection.next_delta_f,
                    current_h,
                    next(counter),
                    selection.next_delta_f,
                    current,
                ),
            )

    return None


def _find_first_conflict(
    paths: Dict[int, List[DroneState]],
    chargers: Dict[Position, int],
) -> Optional[Conflict]:
    drone_ids = sorted(paths)
    max_length = max(len(path) for path in paths.values())

    for time_step in range(max_length):
        air_occupancy: Dict[Tuple[Position, int], int] = {}
        charger_occupancy: Dict[Position, List[int]] = {}

        for drone_id in drone_ids:
            state = _state_at(paths[drone_id], time_step)

            if state.position in chargers:
                charger_occupancy.setdefault(state.position, []).append(
                    drone_id
                )
                continue

            key = (state.position, state.altitude)
            other_id = air_occupancy.get(key)

            if other_id is not None:
                return Conflict(
                    kind="vertex",
                    first_id=other_id,
                    second_id=drone_id,
                    time_step=time_step,
                    position=state.position,
                )

            air_occupancy[key] = drone_id

        for position, occupants in charger_occupancy.items():
            capacity = chargers.get(position, 1)

            if len(occupants) > capacity:
                return Conflict(
                    kind="charger",
                    first_id=occupants[0],
                    second_id=occupants[1],
                    time_step=time_step,
                    position=position,
                )

        if time_step == 0:
            continue

        for first_index, first_id in enumerate(drone_ids):
            first_previous = _state_at(paths[first_id], time_step - 1)
            first_current = _state_at(paths[first_id], time_step)

            if first_previous.position == first_current.position:
                continue

            for second_id in drone_ids[first_index + 1:]:
                second_previous = _state_at(paths[second_id], time_step - 1)
                second_current = _state_at(paths[second_id], time_step)

                if first_current.altitude != second_current.altitude:
                    continue

                if second_previous.position == second_current.position:
                    continue

                if (
                    first_previous.position == second_current.position
                    and second_previous.position == first_current.position
                ):
                    return Conflict(
                        kind="edge",
                        first_id=first_id,
                        second_id=second_id,
                        time_step=time_step,
                        first_from=first_previous.position,
                        first_to=first_current.position,
                        second_from=second_previous.position,
                        second_to=second_current.position,
                    )

    return None


def _constraints_for_conflict(
    conflict: Conflict,
) -> Tuple[Tuple[int, VertexConstraint | EdgeConstraint], ...]:
    if conflict.kind in {"vertex", "charger"}:
        if conflict.position is None:
            raise ValueError("Vertex conflict without a position.")

        return (
            (
                conflict.first_id,
                VertexConstraint(
                    drone_id=conflict.first_id,
                    position=conflict.position,
                    time_step=conflict.time_step,
                ),
            ),
            (
                conflict.second_id,
                VertexConstraint(
                    drone_id=conflict.second_id,
                    position=conflict.position,
                    time_step=conflict.time_step,
                ),
            ),
        )

    if conflict.kind == "edge":
        if (
            conflict.first_from is None
            or conflict.first_to is None
            or conflict.second_from is None
            or conflict.second_to is None
        ):
            raise ValueError("Edge conflict without move data.")

        return (
            (
                conflict.first_id,
                EdgeConstraint(
                    drone_id=conflict.first_id,
                    current=conflict.first_from,
                    next_position=conflict.first_to,
                    arrival_time=conflict.time_step,
                ),
            ),
            (
                conflict.second_id,
                EdgeConstraint(
                    drone_id=conflict.second_id,
                    current=conflict.second_from,
                    next_position=conflict.second_to,
                    arrival_time=conflict.time_step,
                ),
            ),
        )

    raise ValueError(f"Unknown conflict kind: {conflict.kind}")


def _extend_constraints(
    constraints: ConstraintSet,
    new_constraint: VertexConstraint | EdgeConstraint,
) -> ConstraintSet:
    if isinstance(new_constraint, VertexConstraint):
        return replace(
            constraints,
            vertex=constraints.vertex + (new_constraint,),
        )

    return replace(
        constraints,
        edge=constraints.edge + (new_constraint,),
    )


def _node_cost(paths: Dict[int, List[DroneState]]) -> Tuple[int, int]:
    costs = [_path_cost(path) for path in paths.values()]
    return sum(costs), max(costs)


def _format_paths(
    paths: Dict[int, List[DroneState]],
) -> Dict[int, List[JointEnergyState]]:
    return {
        drone_id: [(state,) for state in path]
        for drone_id, path in paths.items()
    }


def plan_all_drones_mapf(
    grid_map: GridMap,
    drones: List[DroneState],
    tasks: List[Task],
    landing_goals: Dict[int, Position],
    max_energy: int = CITY_MAX_ENERGY,
    max_expansions: int = MAX_EXPANSIONS_PER_DRONE,
    max_cbs_nodes: int = 10_000,
    use_warm_start: bool = True,
) -> Tuple[Dict[int, List[JointEnergyState]], Dict[str, object]]:
    """Plan all drones with Conflict-Based Search and EPEA* low-level plans."""
    if len(drones) != len(tasks):
        raise ValueError("Drone count must match task count.")

    for drone_id, drone in enumerate(drones):
        if not grid_map.is_walkable(drone.position):
            raise ValueError(f"Invalid start for drone {drone_id}: {drone.position}")

        if not grid_map.is_walkable(tasks[drone_id].location):
            raise ValueError(
                f"Invalid delivery goal for drone {drone_id}: "
                f"{tasks[drone_id].location}"
            )

        if not grid_map.is_walkable(landing_goals[drone_id]):
            raise ValueError(
                f"Invalid landing goal for drone {drone_id}: "
                f"{landing_goals[drone_id]}"
            )

    root_constraints = ConstraintSet()
    root_paths: Dict[int, List[DroneState]] = {}
    root_per_drone: Dict[int, Dict[str, int]] = {}
    low_level_expanded_total = 0
    low_level_generated_total = 0
    used_warm_start = False

    try:
        if not use_warm_start:
            raise RuntimeError("Warm start disabled.")

        warm_paths, warm_statistics = plan_prioritized_epea(
            grid_map=grid_map,
            drones=drones,
            tasks=tasks,
            landing_goals=landing_goals,
            max_energy=max_energy,
        )

        root_paths = {
            drone_id: [joint_state[0] for joint_state in path]
            for drone_id, path in warm_paths.items()
        }
        root_per_drone = {
            drone_id: dict(values)
            for drone_id, values in warm_statistics["per_drone"].items()
        }
        low_level_expanded_total = int(warm_statistics["total_expanded"])
        low_level_generated_total = int(warm_statistics["total_generated"])
        used_warm_start = True
    except Exception:
        root_paths = {}
        root_per_drone = {}

        for drone_id, drone in enumerate(drones):
            result = _plan_single_drone_constrained_epea(
                grid_map=grid_map,
                drone_id=drone_id,
                start=drone,
                delivery_goal=tasks[drone_id].location,
                landing_goal=landing_goals[drone_id],
                constraints=root_constraints,
                max_energy=max_energy,
                max_expansions=max_expansions,
                forbidden_positions=(
                    set(landing_goals.values()) - {landing_goals[drone_id]}
                ),
            )

            if result is None:
                raise RuntimeError(f"No initial path for drone {drone_id}.")

            root_paths[drone_id] = result.path
            root_per_drone[drone_id] = _make_plan_stats(result, result.path)
            low_level_expanded_total += result.expanded
            low_level_generated_total += result.generated

    sum_of_costs, makespan = _node_cost(root_paths)
    root = CBSNode(
        constraints=root_constraints,
        paths=root_paths,
        per_drone=root_per_drone,
        sum_of_costs=sum_of_costs,
        makespan=makespan,
    )

    open_heap: List[Tuple[int, int, int, int, CBSNode]] = []
    counter = itertools.count()
    heapq.heappush(
        open_heap,
        (
            root.sum_of_costs,
            root.makespan,
            0,
            next(counter),
            root,
        ),
    )

    high_level_expanded = 0
    high_level_generated = 1
    conflicts_resolved = 0

    while open_heap:
        _, _, _, _, node = heapq.heappop(open_heap)
        high_level_expanded += 1

        if high_level_expanded > max_cbs_nodes:
            raise RuntimeError("CBS node limit exceeded.")

        conflict = _find_first_conflict(
            paths=node.paths,
            chargers=grid_map.station_capacity,
        )

        if conflict is None:
            statistics: Dict[str, object] = {
                "planner": "CBS + EPEA*",
                "warm_start": used_warm_start,
                "per_drone": node.per_drone,
                "sum_of_costs": node.sum_of_costs,
                "makespan": node.makespan,
                "total_expanded": sum(
                    values["expanded"]
                    for values in node.per_drone.values()
                ),
                "total_generated": sum(
                    values["generated"]
                    for values in node.per_drone.values()
                ),
                "total_charging_steps": sum(
                    values["charging_steps"]
                    for values in node.per_drone.values()
                ),
                "cbs_high_level_expanded": high_level_expanded,
                "cbs_high_level_generated": high_level_generated,
                "cbs_conflicts_resolved": conflicts_resolved,
                "low_level_expanded_total": low_level_expanded_total,
                "low_level_generated_total": low_level_generated_total,
            }
            return _format_paths(node.paths), statistics

        conflicts_resolved += 1

        for drone_id, new_constraint in _constraints_for_conflict(conflict):
            next_constraints = _extend_constraints(
                node.constraints,
                new_constraint,
            )
            next_paths = dict(node.paths)
            next_per_drone = dict(node.per_drone)

            result = _plan_single_drone_constrained_epea(
                grid_map=grid_map,
                drone_id=drone_id,
                start=drones[drone_id],
                delivery_goal=tasks[drone_id].location,
                landing_goal=landing_goals[drone_id],
                constraints=next_constraints,
                max_energy=max_energy,
                max_expansions=max_expansions,
                forbidden_positions=(
                    set(landing_goals.values()) - {landing_goals[drone_id]}
                ),
            )

            if result is None:
                continue

            next_paths[drone_id] = result.path
            next_per_drone[drone_id] = _make_plan_stats(
                result,
                result.path,
            )
            low_level_expanded_total += result.expanded
            low_level_generated_total += result.generated
            sum_of_costs, makespan = _node_cost(next_paths)
            child = CBSNode(
                constraints=next_constraints,
                paths=next_paths,
                per_drone=next_per_drone,
                sum_of_costs=sum_of_costs,
                makespan=makespan,
            )
            conflict_count = len(next_constraints.vertex) + len(
                next_constraints.edge
            )

            heapq.heappush(
                open_heap,
                (
                    child.sum_of_costs,
                    child.makespan,
                    conflict_count,
                    next(counter),
                    child,
                ),
            )
            high_level_generated += 1

    raise RuntimeError("CBS failed to find conflict-free paths.")


def validate_paths(
    paths: Dict[int, List[JointEnergyState]],
    chargers: Dict[Position, int],
) -> List[str]:
    """Return a list of detected MAPF conflicts."""
    simple_paths = {
        drone_id: [state[0] for state in path]
        for drone_id, path in paths.items()
    }
    conflict = _find_first_conflict(simple_paths, chargers)

    if conflict is None:
        return []

    return [
        (
            f"{conflict.kind} conflict between drones "
            f"{conflict.first_id} and {conflict.second_id} "
            f"at time {conflict.time_step}"
        )
    ]
