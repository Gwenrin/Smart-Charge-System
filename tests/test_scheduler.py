"""Tests for the scheduler module."""

from smart_charge_epea.scheduler import assign_tasks_to_drones
from smart_charge_epea.state import DroneState
from smart_charge_epea.tasks import Task
from smart_charge_epea.grid_map import GridMap


def test_assignment_prefers_nearest_with_sufficient_energy():
    grid = [[0, 0, 0]]
    chargers = {}  # no stations
    gm = GridMap(grid, chargers)
    drones = [
        DroneState(position=(0, 0), energy=10),
        DroneState(position=(2, 0), energy=10),
    ]
    tasks = [Task(id=1, location=(1, 0))]
    assignments = assign_tasks_to_drones(drones, tasks, gm)
    assert assignments[0][0].id == 1


def test_assignment_when_insufficient_energy():
    grid = [[0, 0, 0, 0]]
    chargers = {}
    gm = GridMap(grid, chargers)
    drones = [
        DroneState(position=(0, 0), energy=1),
        DroneState(position=(3, 0), energy=10),
    ]
    tasks = [Task(id=1, location=(3, 0))]
    assignments = assign_tasks_to_drones(drones, tasks, gm)
    assert assignments[1][0].id == 1