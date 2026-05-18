"""Tests for the conflict resolution module."""

import sys
import os

PROJECT_ROOT = os.path.dirname(os.path.dirname(__file__))
SRC_PATH = os.path.join(PROJECT_ROOT, "src")
if SRC_PATH not in sys.path:
    sys.path.insert(0, SRC_PATH)

from conflict_resolver import resolve_conflicts  # type: ignore
from state import DroneState  # type: ignore


def has_vertex_conflict(paths: dict[int, list[tuple[DroneState]]]) -> bool:
    """Check for vertex conflicts in resolved paths."""
    max_len = max(len(p) for p in paths.values())
    extended = {}
    for idx, p in paths.items():
        states = [st[0] for st in p]
        while len(states) < max_len:
            states.append(states[-1])
        extended[idx] = states
    for t in range(max_len):
        positions = [extended[idx][t].position for idx in extended]
        if len(set(positions)) < len(positions):
            return True
    return False


def test_resolve_simple_crossing_conflict():
    # Drone 0 path: (0,0)->(1,0)->(2,0)
    d0 = [DroneState((0, 0), 10), DroneState((1, 0), 9), DroneState((2, 0), 8)]
    # Drone 1 path: (2,0)->(1,0)->(0,0)
    d1 = [DroneState((2, 0), 10), DroneState((1, 0), 9), DroneState((0, 0), 8)]
    paths = {
        0: [(s,) for s in d0],
        1: [(s,) for s in d1],
    }
    resolved = resolve_conflicts(paths)
    assert not has_vertex_conflict(resolved)


def test_swap_conflict_resolved():
    d0 = [DroneState((0, 0), 10), DroneState((1, 0), 9)]
    d1 = [DroneState((1, 0), 10), DroneState((0, 0), 9)]
    paths = {
        0: [(s,) for s in d0],
        1: [(s,) for s in d1],
    }
    resolved = resolve_conflicts(paths)
    assert not has_vertex_conflict(resolved)