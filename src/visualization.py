"""Visualization routines for Smart Charge System."""

from typing import Dict, List, Set, Tuple

import matplotlib.pyplot as plt
import numpy as np

try:
    from .state import JointEnergyState, Position
    from .tasks import Task
except ImportError:
    from state import JointEnergyState, Position
    from tasks import Task


def draw_environment(
    grid: List[List[int]],
    chargers: Set[Position] | Dict[Position, int],
    tasks: List[Task],
    joint_paths: Dict[int, List[JointEnergyState]] | None = None,
) -> None:
    """Rysuje mapę, ładowarki, zadania i trasy dronów."""

    array = np.array(grid)
    height, width = array.shape

    plt.figure(figsize=(max(6, width), max(6, height)))
    plt.imshow(array, cmap="gray_r")

    if chargers:
        if isinstance(chargers, dict):
            charger_positions = list(chargers.keys())
            capacities = [chargers[pos] for pos in charger_positions]
        else:
            charger_positions = list(chargers)
            capacities = [1 for _ in charger_positions]

        xs = [pos[0] for pos in charger_positions]
        ys = [pos[1] for pos in charger_positions]

        plt.scatter(
            xs,
            ys,
            marker="s",
            s=220,
            c="lime",
            edgecolors="black",
            label="Charging stations",
        )

        for (x, y), capacity in zip(charger_positions, capacities):
            if capacity > 1:
                plt.text(x + 0.1, y - 0.2, f"cap={capacity}", fontsize=8)

    for task in tasks:
        x, y = task.location

        plt.scatter(
            x,
            y,
            marker="D",
            s=180,
            c="cornflowerblue",
            edgecolors="black",
        )

        plt.text(x + 0.1, y + 0.1, f"T{task.id}")

    if joint_paths:
        for drone_id, path in joint_paths.items():
            positions = [state[0].position for state in path]

            xs = [p[0] for p in positions]
            ys = [p[1] for p in positions]

            plt.plot(
                xs,
                ys,
                marker="o",
                linewidth=2,
                label=f"Drone {drone_id}",
            )

            start_x, start_y = positions[0]
            end_x, end_y = positions[-1]
            final_energy = path[-1][0].energy

            plt.text(start_x + 0.1, start_y - 0.2, f"S{drone_id}")
            plt.text(end_x + 0.25, end_y, f"E={final_energy}")

    plt.xticks(range(width))
    plt.yticks(range(height))
    plt.gca().invert_yaxis()
    plt.grid(True)
    plt.legend()
    plt.title("Smart Charge System - Joint MAPF 2D")
    plt.show()