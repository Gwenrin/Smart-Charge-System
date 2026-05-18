"""Battery model for Smart Charge System."""

from typing import Tuple

Position = Tuple[int, int]

# Maksymalna pojemność baterii.
MAX_ENERGY: int = 12

# Koszt energetyczny ruchu na sąsiednie pole.
MOVE_COST: int = 1

# Koszt energetyczny czekania.
# Na razie 0, bo dron może chwilę zawisnąć bez uproszczonego zużycia.
WAIT_COST: int = 0

# Minimalna energia, z jaką dron może zakończyć zadanie.
# Dzięki temu dron nie może skończyć z E=0.
MIN_FINAL_ENERGY: int = 1


def get_move_cost(current: Position, next_pos: Position) -> int:
    """Zwraca koszt energii dla przejścia current -> next_pos."""
    if current == next_pos:
        return WAIT_COST

    return MOVE_COST


def consume(energy: int, cost: int) -> int:
    """Zmniejsza energię o koszt ruchu."""
    return energy - cost


def recharge(_: int) -> int:
    """
    Ładowanie na stacji.

    W tej końcowej wersji 2D przyjmujemy uproszczenie:
    wejście na ładowarkę natychmiast odnawia baterię do pełna.
    """
    return MAX_ENERGY


def will_have_energy(energy: int, cost: int) -> bool:
    """Sprawdza, czy dron ma energię na wykonanie ruchu."""
    return energy - cost >= 0