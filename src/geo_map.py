"""Przeliczanie pól siatki na metry i rysowanie podkładu Gliwic."""

from __future__ import annotations

from typing import Tuple

from pyproj import Transformer

try:
    from .city_config import (
        CELL_SIZE_M,
        GLIWICE_CENTER_LAT,
        GLIWICE_CENTER_LON,
        GRID_HEIGHT,
        GRID_WIDTH,
        METRIC_CRS,
    )
    from .state import Position
except ImportError:
    from city_config import (
        CELL_SIZE_M,
        GLIWICE_CENTER_LAT,
        GLIWICE_CENTER_LON,
        GRID_HEIGHT,
        GRID_WIDTH,
        METRIC_CRS,
    )
    from state import Position

_TRANSFORMER_TO_METRIC = Transformer.from_crs(
    "EPSG:4326",
    METRIC_CRS,
    always_xy=True,
)

_TRANSFORMER_TO_WGS84 = Transformer.from_crs(
    METRIC_CRS,
    "EPSG:4326",
    always_xy=True,
)

CENTER_X, CENTER_Y = _TRANSFORMER_TO_METRIC.transform(
    GLIWICE_CENTER_LON,
    GLIWICE_CENTER_LAT,
)

MAP_WIDTH_M = GRID_WIDTH * CELL_SIZE_M
MAP_HEIGHT_M = GRID_HEIGHT * CELL_SIZE_M

GRID_MIN_X = CENTER_X - MAP_WIDTH_M / 2
GRID_MAX_X = CENTER_X + MAP_WIDTH_M / 2
GRID_MIN_Y = CENTER_Y - MAP_HEIGHT_M / 2
GRID_MAX_Y = CENTER_Y + MAP_HEIGHT_M / 2


def grid_to_metric(position: Position) -> Tuple[float, float]:
    """Zwraca środek pola siatki w układzie EPSG:2180."""
    grid_x, grid_y = position

    metric_x = GRID_MIN_X + (grid_x + 0.5) * CELL_SIZE_M
    metric_y = GRID_MAX_Y - (grid_y + 0.5) * CELL_SIZE_M

    return metric_x, metric_y


def metric_to_grid(metric_x: float, metric_y: float) -> Position:
    """Zamienia współrzędne metryczne na indeks pola siatki."""
    grid_x = int((metric_x - GRID_MIN_X) // CELL_SIZE_M)
    grid_y = int((GRID_MAX_Y - metric_y) // CELL_SIZE_M)
    return grid_x, grid_y


def grid_to_wgs84(position: Position) -> Tuple[float, float]:
    """Zwraca punkt jako (szerokość, długość geograficzna)."""
    metric_x, metric_y = grid_to_metric(position)
    longitude, latitude = _TRANSFORMER_TO_WGS84.transform(metric_x, metric_y)
    return latitude, longitude


def set_gliwice_extent(ax) -> None:
    """Ustawia na osi zakres odpowiadający całej siatce miejskiej."""
    ax.set_xlim(GRID_MIN_X, GRID_MAX_X)
    ax.set_ylim(GRID_MIN_Y, GRID_MAX_Y)


def add_gliwice_basemap(ax, zoom: int = 12) -> bool:
    """
    Dodaje podkład OpenStreetMap.

    Zwraca True, gdy kafelki zostały pobrane. Przy braku internetu
    notebook nadal działa i pokazuje siatkę na jasnym tle.
    """
    set_gliwice_extent(ax)

    try:
        import contextily as ctx

        ctx.add_basemap(
            ax,
            crs=METRIC_CRS,
            source=ctx.providers.OpenStreetMap.Mapnik,
            zoom=zoom,
            attribution="© OpenStreetMap contributors",
            attribution_size=7,
        )
        return True
    except Exception as error:
        ax.set_facecolor("#eeeeee")
        ax.text(
            0.5,
            0.02,
            f"Podkład mapowy niedostępny: {error}",
            transform=ax.transAxes,
            ha="center",
            va="bottom",
            fontsize=8,
        )
        return False
