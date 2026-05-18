"""State definitions for Smart Charge System."""

from dataclasses import dataclass
from typing import Tuple

Position = Tuple[int, int]


@dataclass(frozen=True)
class DroneState:
    """
    Stan pojedynczego drona.

    position:
        pozycja (x, y)

    energy:
        aktualny poziom energii
    """

    position: Position
    energy: int


# Wspólny stan wielu dronów.
# Przykład:
# (
#   DroneState((0, 0), 12),
#   DroneState((5, 4), 10),
# )
JointEnergyState = Tuple[DroneState, ...]