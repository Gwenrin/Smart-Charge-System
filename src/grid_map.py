"""Grid map with obstacles and charging stations."""

from typing import Dict, List, Set, Tuple

Position = Tuple[int, int]


class GridMap:
    """
    Mapa kratowa 2D.

    Oznaczenia:
    0 = wolne pole
    1 = przeszkoda
    """

    ACTIONS: List[Position] = [
        (0, 0),    # czekaj
        (0, -1),   # góra
        (1, 0),    # prawo
        (0, 1),    # dół
        (-1, 0),   # lewo
    ]

    def __init__(self, grid: List[List[int]], chargers: Set[Position] | Dict[Position, int]):
        if not grid or not grid[0]:
            raise ValueError("Mapa nie może być pusta.")

        width = len(grid[0])

        for row in grid:
            if len(row) != width:
                raise ValueError("Wszystkie wiersze mapy muszą mieć taką samą długość.")

        self.grid = grid
        self.height = len(grid)
        self.width = width

        if isinstance(chargers, dict):
            self.charging_stations: Set[Position] = set(chargers.keys())
            self.station_capacity: Dict[Position, int] = dict(chargers)
        else:
            self.charging_stations = set(chargers)
            self.station_capacity = {pos: 1 for pos in chargers}

    def in_bounds(self, position: Position) -> bool:
        x, y = position
        return 0 <= x < self.width and 0 <= y < self.height

    def is_walkable(self, position: Position) -> bool:
        if not self.in_bounds(position):
            return False

        x, y = position
        return self.grid[y][x] == 0

    def is_charging_station(self, position: Position) -> bool:
        return position in self.charging_stations

    def get_station_capacity(self, position: Position) -> int:
        return self.station_capacity.get(position, 0)

    def neighbors_with_wait(self, position: Position) -> List[Position]:
        result: List[Position] = []

        x, y = position

        for dx, dy in self.ACTIONS:
            next_position = (x + dx, y + dy)

            if self.is_walkable(next_position):
                result.append(next_position)

        return result

    def print_map(self) -> None:
        for row in self.grid:
            print(" ".join(str(cell) for cell in row))