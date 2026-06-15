"""Battery model for Smart Charge System."""

from typing import Tuple

Position = Tuple[int, int]

MAX_ENERGY: int = 12
MOVE_COST: int = 1
WAIT_COST: int = 0
MIN_FINAL_ENERGY: int = 1


def get_move_cost(current: Position, next_pos: Position) -> int:
    """Zwraca koszt energii dla przejścia current -> next_pos."""
    if current == next_pos:
        return WAIT_COST

    return MOVE_COST


def consume(energy: int, cost: int) -> int:
    """Zmniejsza energię o koszt ruchu."""
    return energy - cost


def recharge(_: int, max_energy: int = MAX_ENERGY) -> int:
    """
    Zwraca poziom energii po pełnym ładowaniu.

    Parametr max_energy pozwala pozostawić MAX_ENERGY=12 dla mapy 8x6,
    a w symulacji miejskiej użyć większej pojemności baterii.
    """
    return max_energy


def will_have_energy(energy: int, cost: int) -> bool:
    """Sprawdza, czy dron ma energię na wykonanie ruchu."""
    return energy - cost >= 0
