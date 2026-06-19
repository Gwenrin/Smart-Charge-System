"""Tablica rezerwacji czasu, pozycji, krawędzi i ładowarek."""

from __future__ import annotations

from collections import defaultdict
from typing import DefaultDict, Dict, Set, Tuple

try:
    from .state import Position
except ImportError:
    from state import Position

AirPosition = Tuple[Position, int]
AirEdge = Tuple[AirPosition, AirPosition]


class ReservationTable:
    # Puste struktury rezerwacji.
    def __init__(self, charger_capacities: Dict[Position, int]) -> None:
        self.air_positions: DefaultDict[int, Set[AirPosition]] = defaultdict(set)
        self.edges: DefaultDict[int, Set[AirEdge]] = defaultdict(set)
        self.charger_usage: DefaultDict[int, DefaultDict[Position, int]] = (
            defaultdict(lambda: defaultdict(int))
        )
        self.permanent_landings: Dict[Position, int] = {}
        self.charger_capacities = dict(charger_capacities)

    # Wolne pole lub wolna ladowarka.
    def is_position_free(
        self,
        position: Position,
        altitude: int,
        time_step: int,
        is_charger: bool,
    ) -> bool:
        landing_time = self.permanent_landings.get(position)

        if landing_time is not None and time_step >= landing_time:
            return False

        if is_charger:
            usage_at_time = self.charger_usage.get(time_step)
            current_usage = (
                usage_at_time.get(position, 0)
                if usage_at_time is not None
                else 0
            )
            capacity = self.charger_capacities.get(position, 1)
            return current_usage < capacity

        occupied_at_time = self.air_positions.get(time_step)

        return (
            occupied_at_time is None
            or (position, altitude) not in occupied_at_time
        )

    # Brak przejazdu w przeciwnym kierunku.
    def is_edge_free(
        self,
        current: Position,
        next_position: Position,
        altitude: int,
        arrival_time: int,
    ) -> bool:
        reverse_edge: AirEdge = (
            (next_position, altitude),
            (current, altitude),
        )
        reserved_edges = self.edges.get(arrival_time)

        return reserved_edges is None or reverse_edge not in reserved_edges

    # Rezerwacja jednego kroku ruchu.
    def reserve_step(
        self,
        current: Position,
        next_position: Position,
        altitude: int,
        arrival_time: int,
        next_is_charger: bool,
    ) -> None:
        if next_is_charger:
            self.charger_usage[arrival_time][next_position] += 1
        else:
            self.air_positions[arrival_time].add((next_position, altitude))

        self.edges[arrival_time].add(
            (
                (current, altitude),
                (next_position, altitude),
            )
        )

    # Rezerwacja pozycji startowej.
    def reserve_start(
        self,
        position: Position,
        altitude: int,
        is_charger: bool = False,
    ) -> None:
        if is_charger:
            self.charger_usage[0][position] += 1
        else:
            self.air_positions[0].add((position, altitude))

    # Zablokowanie koncowego ladowiska.
    def reserve_landing_permanently(
        self,
        position: Position,
        from_time: int,
    ) -> None:
        previous = self.permanent_landings.get(position)

        if previous is None or from_time < previous:
            self.permanent_landings[position] = from_time
