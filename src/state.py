"""State definitions for Smart Charge System."""

from dataclasses import dataclass
from typing import Tuple

Position = Tuple[int, int]


@dataclass(frozen=True)
class DroneState:
    """
    Stan pojedynczego drona.

    position:
        pozycja na siatce 2D w formacie (x, y)

    energy:
        aktualny poziom energii

    altitude:
        stała wysokość przelotowa drona w metrach

    Wysokość pozostaje osobnym polem, dlatego dotychczasowe algorytmy
    operujące na mapie 2D nadal mogą używać pozycji (x, y).
    """

    position: Position
    energy: int
    altitude: int = 80


JointEnergyState = Tuple[DroneState, ...]
