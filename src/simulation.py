"""Simulation utilities for Smart Charge System."""

from typing import Dict, List

try:
    from .state import DroneState, JointEnergyState
except ImportError:
    from state import DroneState, JointEnergyState


def normalize_paths_length(
    paths: Dict[int, List[JointEnergyState]],
) -> Dict[int, List[JointEnergyState]]:
    """Wyrównuje długości ścieżek."""

    if not paths:
        return {}

    max_len = max(len(path) for path in paths.values())
    normalized: Dict[int, List[JointEnergyState]] = {}

    for drone_id, path in paths.items():
        new_path = list(path)

        while len(new_path) < max_len:
            new_path.append(new_path[-1])

        normalized[drone_id] = new_path

    return normalized


def count_agent_moves(agent_path: List[DroneState]) -> int:
    """Liczy ruchy bez czekania."""

    moves = 0

    for previous_state, next_state in zip(agent_path, agent_path[1:]):
        if previous_state.position != next_state.position:
            moves += 1

    return moves


def print_paths_timeline(
    paths: Dict[int, List[JointEnergyState]],
    title: str = "TIMELINE",
) -> None:
    """Wypisuje pozycję i energię każdego drona w czasie."""

    if not paths:
        print("Brak ścieżek do wypisania.")
        return

    normalized = normalize_paths_length(paths)
    max_len = len(next(iter(normalized.values())))

    print()
    print("=" * 80)
    print(title)
    print("=" * 80)

    for t in range(max_len):
        parts = []

        for drone_id in sorted(normalized.keys()):
            drone_state = normalized[drone_id][t][0]
            parts.append(
                f"dron {drone_id}: pos={drone_state.position}, E={drone_state.energy}"
            )

        print(f"t={t:02d} | " + " | ".join(parts))


def print_simulation_summary(
    paths: Dict[int, List[JointEnergyState]],
) -> None:
    """Wypisuje podsumowanie symulacji."""

    if not paths:
        print("Brak ścieżek.")
        return

    print()
    print("=" * 80)
    print("PODSUMOWANIE SYMULACJI")
    print("=" * 80)

    for drone_id, joint_path in sorted(paths.items()):
        agent_path = [state[0] for state in joint_path]

        moves = count_agent_moves(agent_path)
        waits = len(agent_path) - 1 - moves
        final_energy = agent_path[-1].energy

        print(f"Dron {drone_id}:")
        print(f"  liczba kroków czasu: {len(agent_path) - 1}")
        print(f"  liczba ruchów: {moves}")
        print(f"  liczba czekań: {waits}")
        print(f"  energia końcowa: {final_energy}")