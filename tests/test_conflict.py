from src.conflict import (
    has_swap_conflict,
    has_vertex_conflict,
    is_valid_transition,
)


def test_vertex_conflict_detected():
    next_state = ((1, 1), (1, 1))

    assert has_vertex_conflict(next_state) is True


def test_vertex_conflict_not_detected():
    next_state = ((1, 1), (2, 1))

    assert has_vertex_conflict(next_state) is False


def test_swap_conflict_detected():
    current_state = ((1, 1), (2, 1))
    next_state = ((2, 1), (1, 1))

    assert has_swap_conflict(current_state, next_state) is True


def test_swap_conflict_not_detected():
    current_state = ((1, 1), (2, 1))
    next_state = ((1, 2), (2, 2))

    assert has_swap_conflict(current_state, next_state) is False


def test_valid_transition():
    current_state = ((1, 1), (3, 1))
    next_state = ((2, 1), (3, 2))

    assert is_valid_transition(current_state, next_state) is True


def test_invalid_transition_due_to_vertex_conflict():
    current_state = ((1, 1), (3, 1))
    next_state = ((2, 1), (2, 1))

    assert is_valid_transition(current_state, next_state) is False