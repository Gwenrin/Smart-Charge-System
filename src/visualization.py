"""Wizualizacje małej mapy 2D i miejskiej symulacji Gliwic."""

from __future__ import annotations

from typing import Dict, List, Set

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
from matplotlib.lines import Line2D
from matplotlib.patches import Patch, Rectangle

try:
    from .city_config import CELL_SIZE_M, METRIC_CRS
    from .geo_map import (
        GRID_MAX_Y,
        GRID_MIN_X,
        add_gliwice_basemap,
        grid_to_metric,
    )
    from .state import JointEnergyState, Position
    from .tasks import Task
except ImportError:
    from city_config import CELL_SIZE_M, METRIC_CRS
    from geo_map import (
        GRID_MAX_Y,
        GRID_MIN_X,
        add_gliwice_basemap,
        grid_to_metric,
    )
    from state import JointEnergyState, Position
    from tasks import Task


# Rysunek malej mapy siatkowej.
def draw_environment(
    grid: List[List[int]],
    chargers: Set[Position] | Dict[Position, int],
    tasks: List[Task],
    joint_paths: Dict[int, List[JointEnergyState]] | None = None,
) -> None:
    """Rysuje dotychczasową wersję siatkową, np. mapę 8x6."""
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
            label="Ładowarki",
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
            xs = [position[0] for position in positions]
            ys = [position[1] for position in positions]
            altitude = path[0][0].altitude

            plt.plot(
                xs,
                ys,
                marker="o",
                linewidth=2,
                label=f"Dron {drone_id}, h={altitude} m",
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
    plt.title("Smart Charge System - MAPF 2D")
    plt.show()


# Kolory przypisane do pulapow.
def _altitude_colors(paths: Dict[int, List[JointEnergyState]]) -> Dict[int, object]:
    altitudes = sorted({path[0][0].altitude for path in paths.values()})
    color_map = plt.get_cmap("tab10")

    return {
        altitude: color_map(index % 10)
        for index, altitude in enumerate(altitudes)
    }


# Obiekty stale na mapie.
def _draw_grid_objects(
    ax,
    grid: List[List[int]],
    chargers: Dict[Position, int],
    tasks: List[Task],
    landing_goals: Dict[int, Position],
) -> None:
    height = len(grid)
    width = len(grid[0])

    for y in range(height):
        for x in range(width):
            if grid[y][x] != 1:
                continue

            rectangle = Rectangle(
                (
                    GRID_MIN_X + x * CELL_SIZE_M,
                    GRID_MAX_Y - (y + 1) * CELL_SIZE_M,
                ),
                CELL_SIZE_M,
                CELL_SIZE_M,
                facecolor="black",
                edgecolor="black",
                alpha=0.45,
                zorder=3,
            )
            ax.add_patch(rectangle)

    if chargers:
        charger_xy = [grid_to_metric(position) for position in chargers]
        ax.scatter(
            [point[0] for point in charger_xy],
            [point[1] for point in charger_xy],
            marker="s",
            s=70,
            facecolor="lime",
            edgecolor="black",
            linewidth=0.8,
            zorder=7,
        )

    if tasks:
        task_xy = [grid_to_metric(task.location) for task in tasks]
        ax.scatter(
            [point[0] for point in task_xy],
            [point[1] for point in task_xy],
            marker="D",
            s=38,
            facecolor="deepskyblue",
            edgecolor="black",
            linewidth=0.5,
            zorder=6,
        )

    if landing_goals:
        unique_landings = sorted(set(landing_goals.values()))
        landing_xy = [grid_to_metric(position) for position in unique_landings]
        ax.scatter(
            [point[0] for point in landing_xy],
            [point[1] for point in landing_xy],
            marker="X",
            s=48,
            facecolor="gold",
            edgecolor="black",
            linewidth=0.6,
            zorder=6,
        )


# Rysunek tras na podkladzie Gliwic.
def draw_gliwice_environment(
    grid: List[List[int]],
    chargers: Dict[Position, int],
    tasks: List[Task],
    paths: Dict[int, List[JointEnergyState]],
    landing_goals: Dict[int, Position],
    show_grid: bool = True,
):
    """Rysuje trasy w skali 250 m na rzeczywistym podkładzie Gliwic."""
    fig, ax = plt.subplots(figsize=(15, 11))
    add_gliwice_basemap(ax)
    _draw_grid_objects(ax, grid, chargers, tasks, landing_goals)

    altitude_colors = _altitude_colors(paths)

    for drone_id in sorted(paths):
        path = paths[drone_id]
        altitude = path[0][0].altitude
        coordinates = [
            grid_to_metric(state[0].position)
            for state in path
        ]

        xs = [point[0] for point in coordinates]
        ys = [point[1] for point in coordinates]

        ax.plot(
            xs,
            ys,
            linewidth=1.2,
            alpha=0.72,
            color=altitude_colors[altitude],
            zorder=5,
        )

        start_x, start_y = coordinates[0]
        ax.scatter(
            start_x,
            start_y,
            marker="o",
            s=18,
            color=altitude_colors[altitude],
            edgecolor="black",
            linewidth=0.3,
            zorder=8,
        )

    if show_grid:
        width = len(grid[0])
        height = len(grid)

        for x in range(width + 1):
            metric_x = GRID_MIN_X + x * CELL_SIZE_M
            ax.axvline(metric_x, linewidth=0.2, alpha=0.18, color="black")

        for y in range(height + 1):
            metric_y = GRID_MAX_Y - y * CELL_SIZE_M
            ax.axhline(metric_y, linewidth=0.2, alpha=0.18, color="black")

    legend_items = [
        Line2D(
            [0],
            [0],
            color=color,
            linewidth=2,
            label=f"Pułap {altitude} m",
        )
        for altitude, color in altitude_colors.items()
    ]

    legend_items.extend(
        [
            Line2D(
                [0],
                [0],
                marker="s",
                linestyle="none",
                markerfacecolor="lime",
                markeredgecolor="black",
                label="Ładowarka",
            ),
            Line2D(
                [0],
                [0],
                marker="D",
                linestyle="none",
                markerfacecolor="deepskyblue",
                markeredgecolor="black",
                label="Punkt dostawy",
            ),
            Line2D(
                [0],
                [0],
                marker="X",
                linestyle="none",
                markerfacecolor="gold",
                markeredgecolor="black",
                label="Lądowisko",
            ),
            Patch(facecolor="black", alpha=0.45, label="Strefa niedostępna"),
        ]
    )

    ax.legend(handles=legend_items, loc="upper right", fontsize=8)
    ax.set_title(
        "Smart Charge System – 40 dronów nad Gliwicami\n"
        "1 pole siatki = 250 m × 250 m"
    )
    ax.set_xlabel(f"Współrzędna X [{METRIC_CRS}, m]")
    ax.set_ylabel(f"Współrzędna Y [{METRIC_CRS}, m]")
    fig.tight_layout()

    return fig, ax


# Animacja ruchu dronow.
def create_gliwice_animation(
    grid: List[List[int]],
    chargers: Dict[Position, int],
    tasks: List[Task],
    paths: Dict[int, List[JointEnergyState]],
    landing_goals: Dict[int, Position],
    interval_ms: int = 200,
    frame_step: int = 2,
):
    """Tworzy animację Matplotlib gotową do wyświetlenia w JupyterLab."""
    fig, ax = plt.subplots(figsize=(15, 11))
    add_gliwice_basemap(ax)
    _draw_grid_objects(ax, grid, chargers, tasks, landing_goals)

    altitude_colors = _altitude_colors(paths)
    drone_ids = sorted(paths)
    colors = [
        altitude_colors[paths[drone_id][0][0].altitude]
        for drone_id in drone_ids
    ]

    initial_coordinates = [
        grid_to_metric(paths[drone_id][0][0].position)
        for drone_id in drone_ids
    ]

    scatter = ax.scatter(
        [point[0] for point in initial_coordinates],
        [point[1] for point in initial_coordinates],
        s=32,
        c=colors,
        edgecolor="black",
        linewidth=0.4,
        zorder=10,
    )

    time_text = ax.text(
        0.015,
        0.985,
        "",
        transform=ax.transAxes,
        ha="left",
        va="top",
        fontsize=11,
        bbox={"facecolor": "white", "alpha": 0.8, "edgecolor": "black"},
        zorder=11,
    )

    max_time = max(len(path) for path in paths.values())

    # Stan drona dla klatki animacji.
    def state_at(drone_id: int, time_step: int):
        path = paths[drone_id]

        if time_step < len(path):
            return path[time_step][0]

        return path[-1][0]

    # Aktualizacja jednej klatki.
    def update(time_step: int):
        coordinates = [
            grid_to_metric(state_at(drone_id, time_step).position)
            for drone_id in drone_ids
        ]

        scatter.set_offsets(np.asarray(coordinates))
        time_text.set_text(f"t = {time_step}")
        return scatter, time_text

    frame_values = list(range(0, max_time, max(1, frame_step)))

    if frame_values[-1] != max_time - 1:
        frame_values.append(max_time - 1)

    animation = FuncAnimation(
        fig,
        update,
        frames=frame_values,
        interval=interval_ms,
        blit=False,
        repeat=True,
    )

    ax.set_title(
        "Smart Charge System – jednoczesny lot 40 dronów nad Gliwicami"
    )
    ax.set_xlabel(f"Współrzędna X [{METRIC_CRS}, m]")
    ax.set_ylabel(f"Współrzędna Y [{METRIC_CRS}, m]")
    fig.tight_layout()

    return animation
