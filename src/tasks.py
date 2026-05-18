"""Delivery task definitions."""

from dataclasses import dataclass
from typing import Optional, Tuple

Position = Tuple[int, int]


@dataclass
class Task:
    """
    Pojedyncze zadanie dostawy.

    W tej wersji końcowej:
    - każde zadanie jest punktem docelowym jednego drona,
    - liczba zadań = liczba dronów.
    """

    id: int
    location: Position
    weight: float = 0.0
    priority: int = 1
    deadline: Optional[float] = None